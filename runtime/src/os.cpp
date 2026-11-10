// Dolphin OS threads and exceptions over recompiled code.
//
// The game's own scheduler (OSThread.c) runs recompiled. Only the two context primitives
// are replaced: OSSaveContext and OSLoadContext. Every guest thread runs on its own host
// fiber, so a recompiled call chain stays on the stack it started on.
//
//  - A call site of OSSaveContext sets a resume point (gcrt_setjmp) in the caller's
//    frame and binds it to the saved OSContext by a token written into its SRR0.
//  - OSLoadContext(ctx) finds the token: the resume point belongs to the current fiber
//    (unwind to it) or to a suspended one (switch to that fiber, which then unwinds).
//    A context without a token was set up by OSCreateThread: a new fiber starts at SRR0
//    with the registers stored in the context and returns into its LR (OSExitThread).
//  - Interrupts are taken only at safepoints (loop back-edges, MSR[EE] being set, spin
//    loops). Delivery saves the interrupted state into the current OSContext as the
//    exception vector would, binds a resume point the same way, and calls the handler
//    registered in the OS exception table on a copy of the registers.
#include "gcrt/gx.hpp"
#include "gcrt/runtime.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <unordered_map>

extern "C" void gcrt_fiber_switch(void** save_sp, void* load_sp);
extern "C" void gcrt_fiber_start(void);

namespace gcrt {
namespace {

constexpr std::uint32_t kCtxSrr0 = 0x198, kCtxSrr1 = 0x19C, kCtxState = 0x1A2, kCtxGqr = 0x1A4;
constexpr std::uint32_t kCtxCr = 0x80, kCtxLr = 0x84, kCtxCtr = 0x88, kCtxXer = 0x8C, kCtxFpr = 0x90, kCtxFpscr = 0x194;
constexpr std::uint32_t kCtxPsf = 0x1C8;
constexpr std::uint16_t kStateExc = 2;
constexpr std::size_t kFiberStack = 4u * 1024 * 1024;

struct Fiber {
    void* sp = nullptr;
    std::uint8_t* stack = nullptr;
    gc_ctx regs{};
    std::uint32_t entry = 0, exit_lr = 0, home = 0, id = 0;
    gc_jmp pending{};
    bool has_pending = false;
};

struct Resume {
    Fiber* fiber = nullptr;
    std::uint32_t token = 0;
    gc_jmp jb{};
};

Fiber boot;
Fiber* current = &boot;
std::unordered_map<std::uint32_t, Resume> resumes;    // by OSContext address
std::unordered_map<std::uint32_t, Fiber*> homes;      // fiber started for an OSContext
std::uint32_t next_token = 0;
std::uint32_t next_fiber_id = 1;
std::uint64_t fibers = 1;

std::uint32_t make_token() {
    // Low physical addresses never hold code, so a token cannot be mistaken for an entry point.
    next_token = (next_token + 1) & 0x000FFFFF;
    return 0x00100000u | next_token;
}

std::uint32_t pack_cr(const gc_ctx* c) {
    std::uint32_t v = 0;
    for (unsigned n = 0; n < 32; ++n) v |= std::uint32_t(c->cr[n] & 1) << (31 - n);
    return v;
}
void unpack_cr(gc_ctx* c, std::uint32_t v) {
    for (unsigned n = 0; n < 32; ++n) c->cr[n] = (v >> (31 - n)) & 1;
}
std::uint32_t pack_xer(const gc_ctx* c) {
    return (std::uint32_t(c->so) << 31) | (std::uint32_t(c->ov) << 30) | (std::uint32_t(c->ca) << 29) | c->bc;
}

void save_registers(const gc_ctx* c, std::uint32_t ctx) {
    for (unsigned n = 0; n < 32; ++n) write32(ctx + n * 4, c->r[n]);
    write32(ctx + kCtxCr, pack_cr(c));
    write32(ctx + kCtxLr, c->lr);
    write32(ctx + kCtxCtr, c->ctr);
    write32(ctx + kCtxXer, pack_xer(c));
    for (unsigned n = 1; n < 8; ++n) write32(ctx + kCtxGqr + n * 4, c->gqr[n]);
}

void load_registers(gc_ctx* c, std::uint32_t ctx) {
    for (unsigned n = 0; n < 32; ++n) c->r[n] = read32(ctx + n * 4);
    unpack_cr(c, read32(ctx + kCtxCr));
    c->lr = read32(ctx + kCtxLr);
    c->ctr = read32(ctx + kCtxCtr);
    std::uint32_t xer = read32(ctx + kCtxXer);
    c->so = xer >> 31;
    c->ov = (xer >> 30) & 1;
    c->ca = (xer >> 29) & 1;
    c->bc = xer & 0x7F;
    c->fpscr = read32(ctx + kCtxFpscr);
    // As OSSaveContext: GQR0 is not part of a context (the EABI keeps it zero).
    for (unsigned n = 1; n < 8; ++n) c->gqr[n] = read32(ctx + kCtxGqr + n * 4);
    for (unsigned n = 0; n < 32; ++n) {
        c->f[n] = gc_u2d(gc_ld64(gc_mem, ctx + kCtxFpr + n * 8));
        c->ps1[n] = gc_u2d(gc_ld64(gc_mem, ctx + kCtxPsf + n * 8));
    }
}

void release(Fiber* f) {
    if (f == current || f == &boot) return;
    for (auto it = resumes.begin(); it != resumes.end();)
        it = it->second.fiber == f ? resumes.erase(it) : std::next(it);
    munmap(f->stack, kFiberStack);
    delete f;
    --fibers;
}

Fiber* create(std::uint32_t ctx) {
    if (auto old = homes.find(ctx); old != homes.end()) {
        release(old->second);
        homes.erase(old);
    }
    auto* f = new Fiber;
    void* stack = mmap(nullptr, kFiberStack, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON | MAP_NORESERVE, -1, 0);
    if (stack == MAP_FAILED) fatal("cannot allocate a fiber stack");
    f->stack = static_cast<std::uint8_t*>(stack);
    mprotect(f->stack, 4096, PROT_NONE);
    auto* frame = reinterpret_cast<std::uint64_t*>(f->stack + kFiberStack - 160);
    std::memset(frame, 0, 160);
    frame[0] = reinterpret_cast<std::uint64_t>(f);                       // x19
    frame[11] = reinterpret_cast<std::uint64_t>(&gcrt_fiber_start);      // x30
    f->sp = frame;
    load_registers(&f->regs, ctx);
    f->regs.mem = gc_mem;
    f->entry = read32(ctx + kCtxSrr0);
    f->exit_lr = f->regs.lr;
    f->home = ctx;
    f->id = next_fiber_id++;
    homes[ctx] = f;
    ++fibers;
    if (verbosity > 1) log("gcrt: thread context %08x starts at %08x (fiber %u)\n", ctx, f->entry, f->id);
    return f;
}

// Switch to another fiber. Returns when some fiber switches back here; then unwinds to
// the resume point that the switching fiber selected.
void switch_to(Fiber* next) {
    Fiber* self = current;
    current = next;
    set_attention_context(&next->regs);
    gcrt_fiber_switch(&self->sp, next->sp);
    if (self->has_pending) {
        gc_jmp jb = self->pending;
        self->has_pending = false;
        gcrt_longjmp(&jb);
    }
}

void load_context(std::uint32_t ctx) {
    std::uint16_t state = static_cast<std::uint16_t>(read16(ctx + kCtxState));
    if (state & kStateExc) write16(ctx + kCtxState, state & ~kStateExc);
    std::uint32_t srr0 = read32(ctx + kCtxSrr0), srr1 = read32(ctx + kCtxSrr1);
    auto it = resumes.find(ctx);
    if (it != resumes.end() && it->second.token == srr0) {
        Resume r = it->second;
        resumes.erase(it);
        gc_msr = srr1;
        if (r.fiber == current) gcrt_longjmp(&r.jb);
        r.fiber->pending = r.jb;
        r.fiber->has_pending = true;
        switch_to(r.fiber);
        fatal("context load returned");
    }
    if (srr0 < 0x80000000u) fatal("OSLoadContext(%08x): stale resume token %08x", ctx, srr0);
    Fiber* f = create(ctx);
    gc_msr = srr1;
    switch_to(f);
    fatal("context load returned after starting a thread");
}

__attribute__((noinline)) void deliver(gc_ctx* c, std::uint32_t exception) {
    std::uint32_t ctx = read32(0x800000C0) | 0x80000000u;
    if (ctx == 0x80000000u) fatal("exception %u with no current OSContext", exception);
    std::uint32_t handler = read32(0x80003000u + exception * 4);
    if (!handler) fatal("exception %u with no handler", exception);
    save_registers(c, ctx);
    Resume& r = resumes[ctx];
    r.fiber = current;
    r.token = make_token();
    write32(ctx + kCtxSrr0, r.token);
    write32(ctx + kCtxSrr1, gc_msr);
    write16(ctx + kCtxState, read16(ctx + kCtxState) | kStateExc);
    gc_msr &= ~MSR_EE;
    gc_ctx tmp = *c;
    tmp.r[3] = exception;
    tmp.r[4] = ctx;
    if (gcrt_setjmp(&r.jb) == 0) {
        gc_call(&tmp, handler);
        fatal("exception %u handler returned", exception);
    }
}

// Delivers pending interrupts while MSR[EE] allows; true if any was taken.
bool deliver_pending(gc_ctx* c) {
    bool any = false;
    for (int guard = 0; guard < 16 && (gc_msr & MSR_EE); ++guard) {
        if (decrementer_pending()) {
            clear_decrementer();
            deliver(c, 8);
        } else if (pi_cause() & pi_mask()) {
            deliver(c, 4);
        } else {
            break;
        }
        any = true;
    }
    return any;
}

} // namespace

gc_ctx* current_context() { return &current->regs; }
std::uint64_t fiber_count() { return fibers; }

bool fiber_stack_range(std::uintptr_t& low, std::uintptr_t& high) {
    if (!current || !current->stack) return false;
    low = reinterpret_cast<std::uintptr_t>(current->stack);
    high = low + kFiberStack;
    return true;
}

void os_init(gc_ctx*) {}

std::atomic<std::uint64_t> idle_total_ns{0}, idle_gx_ns{0};
IdleTime idle_time() { return {idle_total_ns.load(), idle_gx_ns.load()}; }

namespace {
// Development (GCRT_IDLE_WHY=1): idle time by what the guest's threads wait in, logged every
// 10 s. Dolphin OS lists its threads from 0x800000DC through each OSThread's active link
// (0x2FC); a waiting thread (state 0x2C8 == 4) is keyed by its priority (0x2D0), wait queue
// (0x2DC) and the return addresses on its saved stack (r1 at 0x04 of its context).
bool in_ram(std::uint32_t a) { return a >= 0x80000000u && a < 0x81800000u; }
std::string waiting_threads() {
    std::string key;
    char text[48];
    std::uint32_t t = read32(0x800000DCu);
    for (int n = 0; in_ram(t) && n < 32; ++n, t = read32(t + 0x2FC)) {
        if (read16(t + 0x2C8) != 4) continue;
        std::snprintf(text, sizeof text, "[p%d q%08x", static_cast<int>(read32(t + 0x2D0)), read32(t + 0x2DC));
        key += text;
        std::uint32_t sp = read32(t + 4);
        for (int depth = 0; depth < 4 && in_ram(sp); ++depth) {
            sp = read32(sp);
            if (!in_ram(sp)) break;
            std::snprintf(text, sizeof text, " %08x", read32(sp + 4));
            key += text;
        }
        key += "]";
    }
    return key;
}
void account_idle(const std::string& key, std::uint64_t ns) {
    static std::unordered_map<std::string, std::uint64_t> by_key;
    static std::uint64_t since = now_ns();
    by_key[key] += ns;
    const std::uint64_t now = now_ns();
    if (now - since < 10000000000ull) return;
    std::vector<std::pair<std::uint64_t, std::string>> top;
    for (const auto& [k, v] : by_key) top.emplace_back(v, k);
    std::sort(top.rbegin(), top.rend());
    for (std::size_t i = 0; i < top.size() && i < 6; ++i)
        log("idle why: %.1f ms/s %s\n", top[i].first / 1e6 / ((now - since) / 1e9), top[i].second.c_str());
    by_key.clear();
    since = now;
}
} // namespace

} // namespace gcrt

using namespace gcrt;

extern "C" {

void gcrt_fiber_main(void* p) {
    auto* f = static_cast<Fiber*>(p);
    gc_ctx* c = &f->regs;
    std::uint32_t exit_lr = f->exit_lr;
    c->lr = exit_lr;
    gc_call(c, f->entry);
    c->lr = 0;
    gc_call(c, exit_lr);
    fatal("guest thread %u returned past %08x", f->id, exit_lr);
}


gc_jmp* gc_resume_slot(gc_ctx*, uint32_t ctx) {
    Resume& r = resumes[ctx];
    r.fiber = current;
    r.token = make_token();
    return &r.jb;
}

void hle_OSSaveContext(gc_ctx* c) {
    std::uint32_t ctx = c->r[3];
    auto it = resumes.find(ctx);
    if (it == resumes.end() || it->second.fiber != current) fatal("OSSaveContext(%08x) without a resume point", ctx);
    save_registers(c, ctx);
    write32(ctx + 3 * 4, 1);
    write32(ctx + kCtxSrr0, it->second.token);
    write32(ctx + kCtxSrr1, gc_msr);
    c->r[3] = 0;
}

void hle_OSLoadContext(gc_ctx* c) { load_context(c->r[3]); }

// For code that only reconfigures the MMU or caches (RealMode with Config24MB/Config48MB
// BAT setup): guest addresses are not translated, so there is nothing to do.
void hle_Nop(gc_ctx*) {}

__attribute__((noinline)) void gc_safepoint(gc_ctx* c) {
    gc_attn = 0;
    c->attn = 0;
    run_due_events();
    deliver_pending(c);
}

__attribute__((noinline)) void gc_spin(gc_ctx* c) {
    gc_attn = 0;
    c->attn = 0;
    // The guest is waiting: let the GP catch up with everything submitted so far.
    gx_flush();
    run_due_events();
    if (deliver_pending(c)) return;
    const std::uint64_t since = virtual_time ? 0 : now_ns();
    const bool gx = !virtual_time && gx_busy();
    static const bool why = std::getenv("GCRT_IDLE_WHY") != nullptr;
    const std::string waiting = why ? waiting_threads() + (gx ? " gx-busy" : "") : std::string();
    wait_for_event(1000000);
    if (!virtual_time) {
        const std::uint64_t waited = now_ns() - since;
        idle_total_ns += waited;
        if (gx) idle_gx_ns += waited;
        if (why) account_idle(waiting, waited);
    }
    run_due_events();
    deliver_pending(c);
}

void gc_mtmsr(gc_ctx* c, uint32_t value) {
    std::uint32_t old = gc_msr;
    gc_msr = value;
    if (!(old & MSR_EE) && (value & MSR_EE)) {
        run_due_events();
        deliver_pending(c);
    }
}

} // extern "C"

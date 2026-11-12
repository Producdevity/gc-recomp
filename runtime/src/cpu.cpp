// Processor state outside the per-thread register file: MSR, special registers,
// time base, decrementer, locked-cache DMA, and dispatch of computed branches.
#include <atomic>
#include "gcrt/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>

uint32_t gc_msr = 0x00002032u;  // FP, IR, DR, RI
uint64_t gc_steps = 0, gc_step_deadline = ~0ull;
uint32_t gc_sr[16];
volatile uint32_t gc_attn = 0;

namespace gcrt {

int verbosity = 1;

void log(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::vfprintf(stdout, format, args);
    va_end(args);
    std::fflush(stdout);
}

[[noreturn]] void fatal(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::fprintf(stderr, "gcrt: fatal: ");
    std::vfprintf(stderr, format, args);
    std::fprintf(stderr, "\n");
    va_end(args);
    std::fflush(nullptr);
    std::abort();
}

namespace {
std::uint32_t sprs[1024];
std::uint64_t tb_offset = 0;          // timebase = ticks(now) + offset
std::uint64_t dec_written_tb = 0;     // timebase when DEC was written
std::uint32_t dec_value = 0xFFFFFFFFu;
bool dec_armed = false;
EventId dec_event = 0;
bool dec_fired = false;

std::vector<gc_func> table;
std::uint32_t table_base = 0;
} // namespace

// Virtual time for deterministic runs: guest time advances by kStepNs per loop back-edge
// counted by the recompiled code, and jumps forward when every guest thread waits.
namespace {
constexpr std::uint64_t kStepNs = 20;
std::uint64_t virtual_base = 1000000000ull, counted_steps = 0;
}
bool virtual_time = false;

std::uint64_t now_ns() {
    if (virtual_time) return virtual_base + (gc_steps - counted_steps) * kStepNs;
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
}

void advance_virtual_time(std::uint64_t to_ns) {
    std::uint64_t now = now_ns();
    virtual_base = std::max(now, to_ns);
    counted_steps = gc_steps;
}

void set_virtual_deadline(std::uint64_t when_ns) {
    if (!virtual_time) return;
    std::uint64_t now = now_ns();
    gc_step_deadline = when_ns <= now ? gc_steps : gc_steps + (when_ns - now + kStepNs - 1) / kStepNs;
}
std::uint64_t ns_to_ticks(std::uint64_t ns) { return ns / 2000 * 81 + ns % 2000 * 81 / 2000; }
std::uint64_t ticks_to_ns(std::uint64_t t) { return t / 81 * 2000 + t % 81 * 2000 / 81; }
std::uint64_t timebase() { return ns_to_ticks(now_ns()) + tb_offset; }
void set_timebase(std::uint64_t value) { tb_offset = value - ns_to_ticks(now_ns()); }

namespace {
std::atomic<gc_ctx*> attention_context{nullptr};
}

void raise_attention() {
    gc_attn = 1;
    // A context switch publishes the new context, then copies gc_attn into it: either it
    // sees this store or this thread sees the new context.
    std::atomic_thread_fence(std::memory_order_seq_cst);
    if (gc_ctx* c = attention_context.load()) c->attn = 1;
}

void set_attention_context(gc_ctx* c) {
    c->mem = gc_mem;
    attention_context.store(c);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    c->attn = gc_attn;
}

std::uint32_t& spr(std::uint32_t n) { return sprs[n & 1023]; }

bool decrementer_pending() { return dec_fired; }
void clear_decrementer() { dec_fired = false; }

static std::uint32_t read_dec() {
    if (!dec_armed) return dec_value;
    return dec_value - static_cast<std::uint32_t>(timebase() - dec_written_tb);
}

static void write_dec(std::uint32_t value) {
    dec_value = value;
    dec_written_tb = timebase();
    dec_armed = true;
    dec_fired = false;
    if (dec_event) cancel(dec_event);
    dec_event = 0;
    // The exception is requested when the count passes zero into negative values.
    if (static_cast<std::int32_t>(value) < 0) {
        dec_fired = true;
        raise_attention();
        return;
    }
    std::uint64_t ticks = std::uint64_t(value) + 1;
    dec_event = schedule_in(ticks_to_ns(ticks), [] {
        dec_event = 0;
        dec_fired = true;
        interrupts_changed();
    });
}

// Locked-cache DMA (HID2[LCE], DMA_U/DMA_L).
static void locked_cache_dma() {
    std::uint32_t u = sprs[922], l = sprs[923];
    std::uint32_t lines = ((u & 0x1F) << 2) | ((l >> 2) & 3);
    if (!lines) lines = 128;
    std::uint32_t mem = u & 0xFFFFFFE0u, cache = l & 0xFFFFFFE0u;
    std::uint8_t* lc = gc_mem + 0xE0000000u + (cache & 0x3FFF);
    std::uint32_t bytes = lines * 32;
    if ((cache & 0x3FFF) + bytes > 0x4000) bytes = 0x4000 - (cache & 0x3FFF);
    if (!phys_ok(mem, bytes)) fatal("locked-cache DMA outside RAM: %08x", mem);
    if (l & 0x10) std::memcpy(lc, phys(mem), bytes);     // LD: memory to cache
    else std::memcpy(phys(mem), lc, bytes);
    sprs[923] &= ~2u;
}

void cpu_init(const Program& program) {
    std::memset(sprs, 0, sizeof sprs);
    sprs[287] = 0x00083214;   // PVR: Gekko
    sprs[1008] = 0x0011C464;  // HID0 as the IPL leaves it: caches and BHT enabled
    sprs[920] = 0xE0000000;   // HID2: LSQE, WPE, PSE
    set_timebase(0);
    std::uint32_t lo = 0xFFFFFFFF, hi = 0;
    for (unsigned i = 0; i < program.count; ++i) {
        lo = std::min(lo, program.functions[i].start);
        hi = std::max(hi, program.functions[i].start + 4);
    }
    table_base = lo;
    table.assign(program.count ? (hi - lo) / 4 : 0, nullptr);
    for (unsigned i = 0; i < program.count; ++i) table[(program.functions[i].start - lo) / 4] = program.functions[i].func;
}

gc_func lookup(std::uint32_t address) {
    std::uint32_t index = (address - table_base) / 4;
    if ((address & 3) || index >= table.size()) return nullptr;
    return table[index];
}

} // namespace gcrt

using namespace gcrt;

extern "C" {

void gc_call(gc_ctx* c, uint32_t target) {
    gc_func f = lookup(target);
    if (!f) fatal("call to %08x: no recompiled function there (lr %08x)", target, c->lr);
    f(c);
}

void gc_jump(gc_ctx* c, uint32_t target, uint32_t from) {
    gc_func f = lookup(target);
    if (!f) fatal("indirect jump from %08x to %08x: not a function start", from, target);
    f(c);
}

uint32_t gc_mfspr(gc_ctx*, uint32_t n) {
    switch (n) {
    case 22: return read_dec();
    case 268: return static_cast<uint32_t>(timebase());
    case 269: return static_cast<uint32_t>(timebase() >> 32);
    case 921: return sprs[921] & ~1u;   // WPAR: gather buffer always drained
    default: return sprs[n & 1023];
    }
}

void gc_mtspr(gc_ctx* c, uint32_t n, uint32_t value) {
    switch (n) {
    case 912:
        // Translated code handles psq with GQR0 as the float format, which the EABI keeps.
        if (value & 0x00070007u) fatal("GQR0 set to %08x: quantized formats in GQR0 are not supported", value);
        c->gqr[0] = value;
        return;
    case 22: write_dec(value); return;
    case 284: set_timebase((timebase() & 0xFFFFFFFF00000000ull) | value); return;
    case 285: set_timebase((timebase() & 0xFFFFFFFFull) | (std::uint64_t(value) << 32)); return;
    case 923:
        sprs[923] = value;
        if (value & 2) locked_cache_dma();
        return;
    default: sprs[n & 1023] = value; return;
    }
}

uint32_t gc_mftb(uint32_t tbr) {
    std::uint64_t t = timebase();
    return tbr == 269 ? static_cast<uint32_t>(t >> 32) : static_cast<uint32_t>(t);
}

void gc_sc(gc_ctx*) {}

void gc_rfi(gc_ctx*, uint32_t pc) { fatal("rfi at %08x outside the replaced OS context code", pc); }

void gc_trap(gc_ctx* c, uint32_t pc) {
    fatal("trap at %08x (lr %08x r3 %08x r4 %08x)", pc, c->lr, c->r[3], c->r[4]);
}

void gc_illegal(gc_ctx*, uint32_t pc, uint32_t word) { fatal("illegal instruction %08x at %08x", word, pc); }

void gc_bad_return(uint32_t pc, uint32_t lr, uint32_t expected) {
    static unsigned reported = 0;
    if (reported++ < 20) log("gcrt: return at %08x with lr %08x, entered from %08x\n", pc, lr, expected);
}

double gc_fctiw(gc_ctx* c, double d) {
    double r;
    switch (c->fpscr & 3) {
    case 0: r = std::nearbyint(d); break;
    case 1: r = std::trunc(d); break;
    case 2: r = std::ceil(d); break;
    default: r = std::floor(d); break;
    }
    return gc_u2d(0xFFF8000000000000ull | gc_saturate_s32(r));
}

void gc_mtfsf(gc_ctx* c, uint32_t fm, uint32_t value) {
    for (unsigned f = 0; f < 8; ++f) {
        if (!(fm >> (7 - f) & 1)) continue;
        uint32_t mask = 0xFu << (28 - 4 * f);
        c->fpscr = (c->fpscr & ~mask) | (value & mask);
    }
}

} // extern "C"

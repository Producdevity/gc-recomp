// Direct AArch64 backend. Each guest function becomes one host function
// `void f(gc_ctx*)` with the same synchronisation contract as the C emitter: guest registers
// live in host registers or frame slots, and at calls and returns exactly the EABI-visible
// state is written back to and reloaded from the gc_ctx (tail branches and indirect jumps
// also synchronise the scratch condition fields and XER[CA]). Direct calls between translated
// functions hand arguments and results over in host registers instead (Summary); arguments
// that are not results are then undefined after the call, as under the EABI.
//
// Per program: a fixed point chooses each function's register convention. Per function:
// decode, record each instruction's register uses and definitions by running the code
// generator in analysis mode, solve liveness, assign host registers by weighted use, then
// generate code. Slow paths (device registers, safepoints, quantized formats) go after the
// function body, followed by its literal pool and jump tables. The host code of body and slow
// paths is then scheduled for the Cortex-A53 (a64_sched.cpp).
#include "a64_backend.hpp"

#include "a64_asm.hpp"
#include "a64_sched.hpp"
#include "gcbe/blob.h"
#include "gcrt/recomp.h"
#include "ppc.hpp"
#include "program.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace gcbe {
namespace {

using a64::Cond;
using gcr::Insn;
using gcr::Op;

// Integer class: r0-r31, CR bits 0-31, LR, CTR, XER CA/SO/OV. FP class: f0-f31 (ps0), ps1 0-31.
enum : unsigned { V_R = 0, V_CR = 32, V_LR = 64, V_CTR = 65, V_CA = 66, V_SO = 67, V_OV = 68, NI = 69 };
constexpr unsigned V_F = 0, V_P = 32, NF = 64;

struct VSet {
    std::uint64_t a = 0, b = 0, f = 0;
    void add_i(unsigned v) { if (v < 64) a |= 1ull << v; else b |= 1ull << (v - 64); }
    bool has_i(unsigned v) const { return v < 64 ? (a >> v) & 1 : (b >> (v - 64)) & 1; }
    void add_f(unsigned v) { f |= 1ull << v; }
    bool has_f(unsigned v) const { return (f >> v) & 1; }
    VSet& operator|=(const VSet& o) { a |= o.a; b |= o.b; f |= o.f; return *this; }
    VSet operator|(const VSet& o) const { VSet r = *this; r |= o; return r; }
    VSet operator&(const VSet& o) const { VSet r; r.a = a & o.a; r.b = b & o.b; r.f = f & o.f; return r; }
    VSet operator-(const VSet& o) const { VSet r; r.a = a & ~o.a; r.b = b & ~o.b; r.f = f & ~o.f; return r; }
    bool operator==(const VSet& o) const { return a == o.a && b == o.b && f == o.f; }
    bool operator!=(const VSet& o) const { return !(*this == o); }
    static VSet all() { VSet s; s.a = ~0ull; s.b = 0x1F; s.f = ~0ull; return s; }
};

// EABI volatile state, as in emit.cpp: r0, r3-r12 plus shared r1, r2, r13; f0-f13 (both halves);
// CR fields 0, 1, 5, 6, 7; LR, CTR and XER.
constexpr std::uint32_t kVolatileGpr = 0x00001FF9u, kSharedGpr = 0x00002006u, kVolatileFpr = 0x00003FFFu;
constexpr std::uint32_t kVolatileCr = 0xFFF000FFu;
// At calls: CR0, CR5-CR7 and XER[CA] are scratch under the EABI (no argument travels in
// them); CR1 stays, it flags floating-point arguments to varargs calls. At returns the
// caller can read only the results (r3/r4, f1/f2) and the shared r1, r2 and r13.
constexpr std::uint32_t kAbiCr = 0x000000F0u;
constexpr std::uint32_t kRetGpr = 0x0000201Eu, kRetFpr = 0x00000006u;
enum class Sync { full, call, ret };
// Direct calls between translated functions hand r1, r3-r10 and f1-f8 (both halves) over in host
// registers when the callee reads them, and the callee hands r1, r3, r4, f1 and f2 back in the same
// registers when it may have changed them (Summary); everything else goes through the context.
constexpr std::uint32_t kPassGpr = 0x000007FAu, kPassFpr = 0x000001FEu;
constexpr std::uint32_t kPassRetGpr = 0x0000001Au, kPassRetFpr = 0x00000006u;

VSet sync_mask(bool exact, Sync kind = Sync::full) {
    if (exact) return VSet::all();
    VSet s;
    if (kind == Sync::ret) {
        s.a = kRetGpr;
        s.b = 0x18u;   // SO, OV
        s.f = kRetFpr | (std::uint64_t(kRetFpr) << 32);
        return s;
    }
    s.a = (kVolatileGpr | kSharedGpr) | (std::uint64_t(kind == Sync::call ? kAbiCr : kVolatileCr) << 32);
    s.b = kind == Sync::call ? 0x1Bu : 0x1Fu;   // LR, CTR, CA, SO, OV
    s.f = kVolatileFpr | (std::uint64_t(kVolatileFpr) << 32);
    return s;
}

VSet pass_set(bool ret) {
    VSet s;
    s.a = ret ? kPassRetGpr : kPassGpr;
    std::uint32_t f = ret ? kPassRetFpr : kPassFpr;
    s.f = f | (std::uint64_t(f) << 32);
    return s;
}

// Host register of a passed value: r3-r10 in w0-w7, r1 in w8; f1-f8 in d0-d7, their ps1 in d16-d23.
int arg_reg_i(unsigned v) {
    if (v == V_R + 1) return 8;
    return v >= V_R + 3 && v <= V_R + 10 ? static_cast<int>(v - V_R - 3) : -1;
}
int arg_reg_f(unsigned v) {
    if (v >= V_F + 1 && v <= V_F + 8) return static_cast<int>(v - V_F - 1);
    return v >= V_P + 1 && v <= V_P + 8 ? static_cast<int>(16 + v - V_P - 1) : -1;
}

unsigned ctx_off_i(unsigned v) {
    if (v < V_CR) return offsetof(gc_ctx, r) + 4 * v;
    if (v < V_LR) return offsetof(gc_ctx, cr) + (v - V_CR);
    switch (v) {
    case V_LR: return offsetof(gc_ctx, lr);
    case V_CTR: return offsetof(gc_ctx, ctr);
    case V_CA: return offsetof(gc_ctx, ca);
    case V_SO: return offsetof(gc_ctx, so);
    default: return offsetof(gc_ctx, ov);
    }
}
bool ctx_word_i(unsigned v) { return v < V_CR || v == V_LR || v == V_CTR; }
unsigned ctx_off_f(unsigned v) { return v < V_P ? offsetof(gc_ctx, f) + 8 * v : offsetof(gc_ctx, ps1) + 8 * (v - V_P); }

constexpr unsigned kXC = 27, kXM = 28;      // gc_ctx*, guest memory base (callee-saved)
// Translated functions call each other at their internal entry with x27/x28 already set and
// never change them; the external entry (host callers, the function table) sets them first and
// moves the passed registers between the context and their host registers around the body.
constexpr unsigned kT0 = 16, kT1 = 17;      // temporaries
constexpr unsigned kScrI[4] = {12, 13, 14, 15};
constexpr unsigned kScrF[4] = {28, 29, 30, 31};
const std::vector<unsigned> kCallerI = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const std::vector<unsigned> kCalleeI = {19, 20, 21, 22, 23, 24, 25, 26};
const std::vector<unsigned> kCallerF = {0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
const std::vector<unsigned> kCalleeF = {8, 9, 10, 11, 12, 13, 14, 15};
bool caller_i(int r) { return r >= 0 && r <= 11; }
bool caller_f(int r) { return (r >= 0 && r <= 7) || (r >= 16 && r <= 27); }

enum Import : unsigned {
    I_MEM, I_ATTN, I_STEPS, I_DEADLINE, I_MSR, I_SR, I_SAFEPOINT, I_SPIN, I_CALL, I_JUMP, I_MTMSR, I_MFSPR, I_MTSPR,
    I_MFTB, I_SC, I_RFI, I_TRAP, I_ILLEGAL, I_SETJMP, I_RESUME_SLOT, I_MTFSF, I_FCTIW, I_IO_R8, I_IO_R16, I_IO_R32,
    I_IO_R64, I_IO_W8, I_IO_W16, I_IO_W32, I_IO_W64, I_PSQ_L, I_PSQ_ST, I_COUNT
};
const char* const kImportNames[I_COUNT] = {
    "gc_mem", "gc_attn", "gc_steps", "gc_step_deadline", "gc_msr", "gc_sr", "gc_safepoint", "gc_spin", "gc_call",
    "gc_jump", "gc_mtmsr", "gc_mfspr", "gc_mtspr", "gc_mftb", "gc_sc", "gc_rfi", "gc_trap", "gc_illegal",
    "gcrt_setjmp", "gc_resume_slot", "gc_mtfsf", "gc_fctiw", "gc_io_read8", "gc_io_read16", "gc_io_read32",
    "gcbe_io_read64", "gc_io_write8", "gc_io_write16", "gc_io_write32", "gc_io_write64", "gcbe_psq_l", "gcbe_psq_st",
};

// Register convention of a function's internal entry: values passed in host registers (`in`) and
// values returned in host registers (`out`). Fixed: runtime replacements and exact functions, which
// keep both empty.
struct Summary {
    VSet in, out;
    bool fixed = false;
};

// A direct call or tail branch: the callee, the passable values modified before it, and the
// results demanded after it.
struct Site {
    std::size_t callee;
    VSet modified, demanded;
    bool operator==(const Site& o) const { return callee == o.callee && modified == o.modified && demanded == o.demanded; }
    bool operator!=(const Site& o) const { return !(*this == o); }
};

struct FunctionCode {
    std::vector<std::uint32_t> code;
    std::vector<std::pair<std::uint32_t, std::string>> relocs;   // byte offset, import name
    std::vector<std::pair<std::uint32_t, std::uint32_t>> calls;  // word index of b/bl, guest target
    std::vector<std::pair<std::uint32_t, std::uint32_t>> external_calls;   // the same, to the external entry
    std::uint32_t entry = 0;   // word index of the internal entry
    std::size_t spilled = 0, instructions = 0, invalid = 0, spin_loops = 0;
};

enum PathFlags : unsigned { P_FLUSH = 1, P_FLUSH_ALL = 2, P_RELOAD = 4, P_RELOAD_ALL = 8, P_LR_IMM = 16, P_ABI = 32, P_RET = 64 };
constexpr int kFall = -2, kExit = -1;

struct Path {
    int target = kFall;   // instruction index, kFall (next instruction) or kExit
    unsigned flags = 0;
    VSet uses, defs;
    int callee = -1;      // direct call or tail branch to this function (its internal entry)
};

struct Info {
    VSet uses, defs;
    VSet flush;   // values stored to the context by this instruction's call, return or exit
    VSet flush_live;   // passed values also stored at a direct call when live after it
    VSet so_copies;   // CR bits set from XER[SO]: SO is used only while one of them is live
    bool falls = true;
    std::vector<Path> paths;
    VSet live_in, after;
    unsigned weight = 1;
};

class Compiler {
public:
    Compiler(const gcr::Program& p, const gcr::Function& f, const Options& o, const std::vector<Summary>& sums)
        : p_(p), f_(f), opt_(o), sums_(sums), in_(sums[p.by_start.at(f.start)].in), out_regs_(sums[p.by_start.at(f.start)].out) {}
    FunctionCode run();
    // Given everyone's current conventions: the passable values live at the entry (in), the
    // results modified on every path to every exit (out), and for each direct call or tail
    // branch the callee and the passable values modified there.
    Summary summarize(std::vector<Site>& sites);

private:
    void decode();
    void schedule();
    void analyze();
    void liveness();
    void allocate();
    void layout_frame();
    void generate();
    bool finish();   // false: a TBNZ is out of range
    void schedule_code();

    int new_label() { label_pos_.push_back(-1); return static_cast<int>(label_pos_.size() - 1); }
    void bind(int l) { label_pos_[l] = static_cast<int>(a_.pos()); }
    enum FixKind { F_IMM26, F_IMM19, F_IMM14, F_ADR, F_WORD };
    struct Fix { std::size_t at; int label; FixKind kind; std::size_t base; };
    void fix(int label, FixKind k, std::size_t base = 0) { fixes_.push_back({a_.pos(), label, k, base}); }
    void jmp(int l) { fix(l, F_IMM26); a_.b(0); }
    void jcc(Cond c, int l) { fix(l, F_IMM19); a_.bcond(c, 0); }
    void jz(unsigned r, int l, bool nz) { fix(l, F_IMM19); a_.cbz(r, 0, nz); }
    int guest_label(std::uint32_t pc) { return labels_.at(pc); }

    void load_import(unsigned xd, const std::string& name);
    void load_import(unsigned xd, Import i) { load_import(xd, kImportNames[i]); }
    void call_import(const std::string& name) { load_import(kT0, name); a_.blr(kT0); }
    void call_import(Import i) { call_import(kImportNames[i]); }

    unsigned ri(unsigned v, int into = -1);
    unsigned wi(unsigned v, int into = -1);
    void ci(unsigned v, unsigned r);
    unsigned rf(unsigned v, int into = -1);
    unsigned wf(unsigned v, int into = -1);
    void cf(unsigned v, unsigned r);
    void set_i(unsigned v, unsigned src) {
        if (!analyzing_ && v < 32 && pend_[v]) {
            if (cold_mode_) to_raw(src, pend_[v]);   // the hot path continues with the loaded bytes
            else pend_[v] = 0;
        }
        unsigned d = wi(v, static_cast<int>(src));
        if (d != src) a_.mov(d, src);
        ci(v, d);
    }
    // Deferred byte swaps: a GPR loaded from guest memory keeps the bytes as loaded until its
    // first use, so the swap does not stall on the load. 1: rev, 2: rev16, 3: rev16 + sxth.
    // Labels, calls, flushes and every instruction other than pure ones and loads/stores
    // materialise them first; cold paths see the state of their hot instruction and swap
    // into a scratch register.
    std::uint8_t pend_[32] = {};
    bool cold_mode_ = false;
    void swap_value(unsigned rd, unsigned rn, unsigned kind) {
        if (kind == 1) a_.rev(rd, rn);
        else { a_.rev16(rd, rn); if (kind == 3) a_.sxth(rd, rd); }
    }
    void to_raw(unsigned r, unsigned kind) {
        if (kind == 1) { a_.rev(r, r); return; }
        a_.ubfx(r, r, 0, 16);
        a_.rev16(r, r);
    }
    void resolve(unsigned n) {
        if (!pend_[n]) return;
        unsigned r = static_cast<unsigned>(ireg_[V_R + n]);
        swap_value(r, r, pend_[n]);
        pend_[n] = 0;
    }
    // Before instruction k_: values dead from here on are dropped without a swap.
    void resolve_all() {
        for (unsigned n = 0; n < 32; ++n) {
            if (!pend_[n]) continue;
            if (info_[k_].live_in.has_i(V_R + n)) resolve(n);
            else pend_[n] = 0;
        }
    }
    bool keeps_pending(Op op, Insn i) const;
    void set_f(unsigned v, unsigned src) { unsigned d = wf(v, static_cast<int>(src)); a_.fmov(d, src); cf(v, d); }
    void note_use_i(unsigned v) { if (analyzing_ && !cur().defs.has_i(v)) cur().uses.add_i(v); }
    void note_def_i(unsigned v) { if (analyzing_) cur().defs.add_i(v); }
    void note_use_f(unsigned v) { if (analyzing_ && !cur().defs.has_f(v)) cur().uses.add_f(v); }
    Info& cur() { return info_[k_]; }
    [[noreturn]] void fail(const char* what) const {
        char b[160];
        std::size_t k = std::min(k_, n_ ? n_ - 1 : 0);
        std::snprintf(b, sizeof b, "%s at %08X (%s)", what, f_.start + 4 * static_cast<unsigned>(k),
                      n_ ? gcr::op_name(ops_[k]) : "-");
        throw std::runtime_error(b);
    }
    // Scratch registers for spilled values, handed out once per instruction phase.
    unsigned scratch_i() {
        if (iscr_ >= 4) fail("out of integer scratch registers");
        return kScrI[iscr_++];
    }
    unsigned scratch_f() {
        if (fscr_ >= 4) fail("out of floating-point scratch registers");
        return kScrF[fscr_++];
    }

    void home_to_reg_i(unsigned v, unsigned r);
    void reg_to_home_i(unsigned v, unsigned r);

    const VSet& flush_here() const { return info_[k_].flush; }
    void flush(const VSet& s, bool lr_imm = false, std::uint32_t lr_value = 0);
    void reload(const VSet& s);
    VSet keep_set() const;        // caller-saved homes that must survive a runtime call here
    void save(const VSet& keep);
    void restore(const VSet& keep);
    void prologue();
    void epilogue();
    void path(int target, unsigned flags, VSet uses = {}, VSet defs = {}, int callee = -1);
    VSet sync(const Path& p) const { return (p.flags & P_RET) ? ret_ : (p.flags & P_ABI) ? abi_ : mask_; }
    // Index of the translated function starting at target (-1: none).
    int callee_at(std::uint32_t target) const {
        auto it = p_.by_start.find(target);
        return it == p_.by_start.end() ? -1 : static_cast<int>(it->second);
    }
    // Moves passed values between their homes and their host registers (to_regs: homes to
    // registers, before calls and returns; otherwise after entries and calls).
    void arg_moves(const VSet& s, bool to_regs);
    // After a call whose callee returns `got` in registers, this function returns `out_regs_`:
    // moves the difference through the context, then returns.
    void return_after_call(const VSet& got);

    void add_imm32(unsigned rd, unsigned rn, std::int32_t v);
    unsigned ea_d(unsigned ra, std::int32_t d, bool copy);
    unsigned ea_x(unsigned ra, unsigned rb, bool copy);
    // Stack and small-data bases never address hardware; neither do known RAM addresses.
    bool needs_io_check(unsigned ra, bool indexed, std::int32_t d = 0) const {
        if (indexed) return true;
        if (ra == 1 || ra == 2 || ra == 13) return false;
        std::uint32_t base;
        if (!constant(ra, base)) return true;
        return gc_is_io(base + static_cast<std::uint32_t>(d));
    }
    // Known register values (lis/addi/ori chains), reset at labels and calls. The same
    // tracking runs during analysis, so recorded uses match the generated code.
    struct Known { bool known = false; std::uint32_t value = 0; };
    Known known_[32];
    bool constant(unsigned ra, std::uint32_t& v) const {
        if (ra == 0) { v = 0; return true; }
        if (!known_[ra].known) return false;
        v = known_[ra].value;
        return true;
    }
    void forget_all() { for (auto& k : known_) k.known = false; }
    void track(std::size_t k);
    bool pure(Op op, Insn i) const;
    // Valid addresses have bit 27 set only in the EFB and register window (gc_is_io). TBNZ
    // reaches 32 KiB; a function whose cold code lies further away is generated again with
    // the compare-and-branch form.
    bool near_io_ = true;
    void io_check(unsigned ea, int cold, unsigned tmp = kT1) {
        if (near_io_) {
            fix(cold, F_IMM14);
            a_.tbz(ea, 27, 0, true);
            return;
        }
        a_.lsr_imm(tmp, ea, 27);
        a_.cmp_imm(tmp, 25);
        jcc(a64::EQ, cold);
    }
    void cold(std::function<void()> fn);
    void rounding(unsigned d) { a_.fcvt_sd(d, d); a_.fcvt_ds(d, d); }
    void logic_imm32(unsigned opc, unsigned rd, unsigned rn, std::uint32_t imm);

    struct Flags { bool valid = false; std::size_t k = 0; unsigned field = 0; Cond cond[3]; };
    void set_field_from_flags(unsigned field, Cond lt, Cond gt, Cond eq, bool fp);
    void record(unsigned reg);
    // XER[SO] |= XER[OV]; reads come before the write so analysis sees SO as used.
    void sticky_overflow() {
        unsigned so = ri(V_SO), ov = ri(V_OV);
        unsigned sd = wi(V_SO);
        a_.orr(sd, so, ov);
        ci(V_SO, sd);
    }
    void record_fp();
    bool fused(unsigned bi, Cond& c) const;

    void emit_insn(std::uint32_t pc, Insn i, Op op);
    void emit_load(Insn i, Op op);
    void emit_store(Insn i, Op op);
    void emit_lmw(std::uint32_t pc, Insn i, bool load);
    void emit_string(Insn i, bool load);
    void emit_fload(Insn i, Op op);
    void emit_fstore(Insn i, Op op);
    void emit_psq(Insn i, Op op);
    void gather_append(unsigned size, const std::function<void(unsigned at)>& store, int other, int cont);
    void gather_inline(unsigned size, const std::function<void(unsigned at)>& store);
    // A D-form access whose address is known to be the write-gather pipe.
    bool fifo_address(unsigned ra, std::int32_t d) const {
        std::uint32_t base;
        return ra != 0 && constant(ra, base) && base + static_cast<std::uint32_t>(d) == 0xCC008000u;
    }
    void emit_arith(Insn i, Op op);
    void emit_rotate(Insn i, Op op);
    void emit_fp(std::uint32_t pc, Insn i, Op op);
    void emit_ps(Insn i, Op op);
    void emit_cr(Insn i, Op op);
    void emit_spr(std::uint32_t pc, Insn i, Op op);
    void emit_branch(std::uint32_t pc, Insn i, Op op);
    void emit_trap(std::uint32_t pc, Insn i, Op op);
    void emit_runtime_call(Import fn, std::uint32_t pc, std::uint32_t word);

    void branch_if(bool when_taken, unsigned bo, unsigned bi, bool decrement, int label);
    void decrement_ctr();
    void backedge(std::uint32_t pc, std::uint32_t target);
    bool is_spin_loop(std::uint32_t target, std::uint32_t branch) const;
    void guest_call(std::uint32_t pc, std::uint32_t target);
    void guest_tail(std::uint32_t target);
    void guest_return();
    void indirect_call(std::uint32_t pc, unsigned target_value);
    void emit_switch(std::uint32_t pc);

    const gcr::Program& p_;
    const gcr::Function& f_;
    const Options& opt_;
    const std::vector<Summary>& sums_;
    const VSet in_, out_regs_;   // this function's own convention
    VSet ret_must_;              // results modified on every path to every return and tail branch
    bool exits_ = false;         // whether any return or tail branch constrains ret_must_
    std::vector<Site> sites_;
    std::vector<VSet> demand_;   // live values not counting flushes (analyze)
    a64::Asm a_;
    FunctionCode out_;
    bool analyzing_ = false;
    // Leaf functions (no guest calls, everything in caller-saved registers) have no frame;
    // slow paths push one around their runtime calls.
    bool leaf_ = false;
    unsigned xc_ = kXC, xm_ = kXM;

    std::vector<Insn> insns_;
    std::vector<Op> ops_;
    std::vector<std::uint32_t> pcs_;   // guest address of the instruction at each position
    std::size_t n_ = 0, k_ = 0;
    std::map<std::uint32_t, int> labels_;
    std::set<std::uint32_t> label_pcs_;
    std::vector<Info> info_;
    VSet mask_, abi_, ret_, across_, written_, used_, end_uses_;
    bool ended_ = false, has_calls_ = false;

    int ireg_[NI], islot_[NI], freg_[NF], fslot_[NF];
    int isave_[32], fsave_[32];
    std::vector<unsigned> callee_i_, callee_f_;
    unsigned frame_ = 0;
    unsigned iscr_ = 0, fscr_ = 0;

    std::vector<int> label_pos_;
    std::vector<Fix> fixes_;
    std::vector<std::function<void()>> cold_;
    std::map<std::string, int> literals_;
    struct Table { int label; int fallback; std::vector<int> entries; };
    std::vector<Table> tables_;
    Flags flags_;
};

void Compiler::decode() {
    n_ = (f_.end - f_.start) / 4;
    insns_.resize(n_);
    ops_.resize(n_);
    pcs_.resize(n_);
    for (std::size_t k = 0; k < n_; ++k) {
        pcs_[k] = f_.start + 4 * static_cast<std::uint32_t>(k);
        insns_[k].w = p_.image->read32(pcs_[k]);
        ops_[k] = gcr::decode(insns_[k]);
    }
    // Labels: branch targets inside the function and indirect-jump candidates (emit.cpp body()).
    for (std::size_t k = 0; k < n_; ++k) {
        std::uint32_t pc = f_.start + 4 * static_cast<std::uint32_t>(k);
        Insn i = insns_[k];
        Op op = ops_[k];
        auto inside = [&](std::uint32_t t) { return t >= f_.start && t < f_.end; };
        if (op == Op::b && !i.lk()) {
            std::uint32_t t = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
            if (inside(t)) label_pcs_.insert(t);
        } else if (op == Op::bc && !i.lk()) {
            std::uint32_t t = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
            if (inside(t)) label_pcs_.insert(t);
        } else if (op == Op::bcctr && !i.lk()) {
            auto lo = std::lower_bound(p_.code_pointers.begin(), p_.code_pointers.end(), f_.start);
            for (auto it = lo; it != p_.code_pointers.end() && *it < f_.end; ++it)
                if (*it != f_.start) label_pcs_.insert(*it);
        }
    }
    label_pcs_.insert(f_.start);
    if (n_) {
        Insn i = insns_[n_ - 1];
        Op op = ops_[n_ - 1];
        ended_ = (op == Op::b && !i.lk()) ||
                 ((op == Op::bc || op == Op::bclr || op == Op::bcctr) && !i.lk() && (i.bo() & 0x14) == 0x14) || op == Op::rfi;
    }
}

void Compiler::path(int target, unsigned flags, VSet uses, VSet defs, int callee) {
    if (!analyzing_) return;
    Path p;
    p.target = target;
    p.flags = flags;
    p.uses = uses;
    p.defs = defs;
    p.callee = callee;
    cur().paths.push_back(p);
}

void Compiler::analyze() {
    analyzing_ = true;
    forget_all();
    info_.assign(n_ + 1, Info{});
    mask_ = sync_mask(f_.exact);
    abi_ = sync_mask(f_.exact, Sync::call);
    ret_ = sync_mask(f_.exact, Sync::ret);
    for (std::size_t k = 0; k < n_; ++k) {
        k_ = k;
        iscr_ = fscr_ = 0;
        std::uint32_t pc = f_.start + 4 * static_cast<std::uint32_t>(k);
        if (label_pcs_.count(pc)) forget_all();
        emit_insn(pcs_[k], insns_[k], ops_[k]);
        for (auto& fn : cold_) fn();
        cold_.clear();
        track(k);
    }
    written_ = used_ = VSet{};
    for (std::size_t k = 0; k < n_; ++k) {
        written_ |= info_[k].defs;
        used_ |= info_[k].uses | info_[k].defs;
        for (auto& p : info_[k].paths) {
            written_ |= p.defs;
            used_ |= p.uses | p.defs;
            if (p.callee >= 0 && (p.flags & P_RELOAD)) written_ |= sums_[static_cast<std::size_t>(p.callee)].out;
        }
    }
    // Values passed in registers are not in the context: they count as written at the entry.
    written_ |= in_;
    // Demand: values read by instructions, passed to callees or returned in registers, but not
    // the ones only stored by flushes. A result a callee returns in a register stays in one only
    // where it is demanded after the call; elsewhere it is stored to the context there.
    const int next = ended_ ? -1 : callee_at(f_.end);
    demand_.assign(n_ + 1, VSet{});
    if (next >= 0) demand_[n_] = sums_[static_cast<std::size_t>(next)].in;
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t k = n_; k-- > 0;) {
            const Info& in = info_[k];
            VSet l;
            if (in.falls) l |= demand_[k + 1];
            for (auto& p : in.paths) {
                VSet t;
                if (p.target == kFall) t = demand_[k + 1];
                else if (p.target >= 0) t = demand_[static_cast<std::size_t>(p.target)];
                VSet u = p.uses, d = p.defs;
                if (p.callee >= 0) u |= sums_[static_cast<std::size_t>(p.callee)].in;
                if (p.flags & P_RET) u |= out_regs_;
                if (p.flags & P_RELOAD) d |= sync(p);
                if (p.flags & P_RELOAD_ALL) d |= VSet::all();
                l |= u | (t - d);
            }
            VSet live = in.uses | (l - in.defs);
            if ((l & in.so_copies) != VSet{}) live.add_i(V_SO);
            if (live != demand_[k]) { demand_[k] = live; changed = true; }
        }
    }
    auto returned = [&](std::size_t k, const Path& p) {
        const VSet& t = p.target == kFall ? demand_[k + 1] : demand_[static_cast<std::size_t>(p.target)];
        return sums_[static_cast<std::size_t>(p.callee)].out & t;
    };
    // Values modified since the last synchronisation with the context ("dirty"). A flush
    // stores only those: the context already holds every other value, so storing it again
    // would change nothing (the C path stores all written values).
    std::vector<VSet> dirty(n_ + 1);
    dirty[0] = in_;
    for (bool changed = true; changed;) {
        changed = false;
        auto send = [&](std::size_t t, const VSet& v) {
            VSet m = dirty[t] | v;
            if (m != dirty[t]) { dirty[t] = m; changed = true; }
        };
        for (std::size_t k = 0; k < n_; ++k) {
            VSet out = dirty[k] | info_[k].defs;
            if (info_[k].falls) send(k + 1, out);
            for (auto& p : info_[k].paths) {
                VSet po = out;
                if (p.flags & P_RELOAD) {
                    po = out - sync(p);
                    if (p.callee >= 0) po |= returned(k, p);
                }
                if (p.flags & P_RELOAD_ALL) { po = VSet{}; po.add_i(V_R + 3); }
                if (p.target == kFall) send(k + 1, po);
                else if (p.target >= 0) send(static_cast<std::size_t>(p.target), po);
            }
        }
    }
    // Values modified on every path since the entry or the last synchronisation. Results that
    // hold this at every return and tail branch are returned in registers (Summary::out).
    std::vector<VSet> must(n_ + 1);
    std::vector<char> reached(n_ + 1, 0);
    must[0] = in_;
    reached[0] = 1;
    for (bool changed = true; changed;) {
        changed = false;
        auto meet = [&](std::size_t t, const VSet& v) {
            VSet m = reached[t] ? (must[t] & v) : v;
            if (!reached[t] || m != must[t]) { must[t] = m; reached[t] = 1; changed = true; }
        };
        for (std::size_t k = 0; k < n_; ++k) {
            if (!reached[k]) continue;
            VSet out = must[k] | info_[k].defs;
            if (info_[k].falls) meet(k + 1, out);
            for (auto& p : info_[k].paths) {
                VSet po = out;
                if (p.flags & P_RELOAD) {
                    po = out - sync(p);
                    if (p.callee >= 0) po |= returned(k, p);
                }
                if (p.flags & P_RELOAD_ALL) { po = VSet{}; po.add_i(V_R + 3); }
                if (p.target == kFall) meet(k + 1, po);
                else if (p.target >= 0) meet(static_cast<std::size_t>(p.target), po);
            }
        }
    }
    VSet lr;
    lr.add_i(V_LR);
    const VSet pass = pass_set(false);
    ret_must_ = pass_set(true);
    exits_ = false;
    sites_.clear();
    for (std::size_t k = 0; k < n_; ++k) {
        VSet out = dirty[k] | info_[k].defs;
        for (auto& p : info_[k].paths) {
            VSet f;
            if (p.flags & P_FLUSH) f |= written_ & sync(p) & out;
            if (p.flags & P_FLUSH_ALL) f |= written_ & out;
            if (p.callee >= 0) {
                // Values the callee reads travel in registers; the context copy is needed only
                // when this function reloads the value after the call. Exact functions store all.
                const Summary& s = sums_[static_cast<std::size_t>(p.callee)];
                const VSet wanted = (p.flags & P_RELOAD) ? demand_[k + 1] : out_regs_;
                sites_.push_back({static_cast<std::size_t>(p.callee), out & written_ & pass, wanted & pass_set(true)});
                if (!f_.exact) {
                    if (p.flags & P_RELOAD) info_[k].flush_live = f & s.in - s.out;
                    f = f - s.in;
                }
                p.uses |= s.in;
                if (!(p.flags & P_RELOAD) && reached[k]) {   // tail branch: its results are ours
                    ret_must_ = ret_must_ & s.out;
                    exits_ = true;
                }
            }
            if (p.flags & P_RET) {
                if (reached[k]) {
                    ret_must_ = ret_must_ & (must[k] | info_[k].defs);
                    exits_ = true;
                }
                f = f - out_regs_;
                p.uses |= out_regs_;
            }
            if (p.flags & (P_FLUSH | P_FLUSH_ALL)) info_[k].flush = f;
            if (p.flags & P_LR_IMM) f = f - lr;   // LR is stored as the return address
            p.uses |= f;
            if (p.flags & P_RELOAD) p.defs |= sync(p);
            if (p.flags & P_RELOAD_ALL) p.defs |= VSet::all();
        }
    }
    // Falling off the end continues in the next function.
    info_[n_].flush = ended_ ? VSet{} : (written_ & mask_ & dirty[n_]);
    end_uses_ = info_[n_].flush;
    if (next >= 0) {
        const Summary& s = sums_[static_cast<std::size_t>(next)];
        sites_.push_back({static_cast<std::size_t>(next), dirty[n_] & written_ & pass, out_regs_});
        if (!f_.exact) info_[n_].flush = info_[n_].flush - s.in;
        end_uses_ = info_[n_].flush | s.in;
        if (reached[n_]) {
            ret_must_ = ret_must_ & s.out;
            exits_ = true;
        }
    }
    used_ |= end_uses_;
    std::vector<unsigned> depth(n_, 0);
    for (std::size_t k = 0; k < n_; ++k) {
        Op op = ops_[k];
        Insn i = insns_[k];
        std::uint32_t pc = f_.start + 4 * static_cast<std::uint32_t>(k);
        std::uint32_t t = 0;
        if (op == Op::b && !i.lk()) t = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
        else if (op == Op::bc && !i.lk()) t = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
        else continue;
        if (t < f_.start || t > pc) continue;
        for (std::size_t j = (t - f_.start) / 4; j <= k; ++j) ++depth[j];
    }
    for (std::size_t k = 0; k < n_; ++k) info_[k].weight = 1u << (3 * std::min(depth[k], 3u));
    analyzing_ = false;
}

void Compiler::liveness() {
    info_[n_].live_in = end_uses_;
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t k = n_; k-- > 0;) {
            Info& in = info_[k];
            VSet l;
            if (in.falls) l |= info_[k + 1].live_in;
            for (auto& p : in.paths) {
                VSet t;
                if (p.target == kFall) t = info_[k + 1].live_in;
                else if (p.target >= 0) t = info_[p.target].live_in;
                l |= p.uses | (t - p.defs);
            }
            VSet live = in.uses | (l - in.defs);
            if ((l & in.so_copies) != VSet{}) live.add_i(V_SO);
            if (live != in.live_in || l != in.after) {
                in.live_in = live;
                in.after = l;
                changed = true;
            }
        }
    }
}

void Compiler::allocate() {
    std::vector<std::uint64_t> wi(NI, 0), wf(NF, 0);
    for (std::size_t k = 0; k < n_; ++k) {
        const Info& in = info_[k];
        VSet touched = in.uses | in.defs;
        for (unsigned v = 0; v < NI; ++v)
            if (touched.has_i(v)) wi[v] += in.weight * 4;
        for (unsigned v = 0; v < NF; ++v)
            if (touched.has_f(v)) wf[v] += in.weight * 4;
        for (auto& p : in.paths) {
            VSet s = p.uses | (p.defs & in.after);
            for (unsigned v = 0; v < NI; ++v)
                if (s.has_i(v)) wi[v] += in.weight;
            for (unsigned v = 0; v < NF; ++v)
                if (s.has_f(v)) wf[v] += in.weight;
        }
    }
    VSet need = used_;
    for (const Info& in : info_) need |= in.live_in;
    for (unsigned v = 0; v < NI; ++v) ireg_[v] = islot_[v] = -1;
    for (unsigned v = 0; v < NF; ++v) freg_[v] = fslot_[v] = -1;
    for (unsigned r = 0; r < 32; ++r) isave_[r] = fsave_[r] = -1;
    // Values kept across guest calls without a reload (not in the sync mask) need
    // callee-saved registers; everything else prefers caller-saved ones.
    auto run = [&](unsigned count, auto has, auto preserved, int (*pref)(unsigned), const std::vector<std::uint64_t>& w,
                   const std::vector<unsigned>& caller, const std::vector<unsigned>& callee, int* reg, int* slot) {
        std::vector<unsigned> order;
        for (unsigned v = 0; v < count; ++v)
            if (has(v)) order.push_back(v);
        std::stable_sort(order.begin(), order.end(), [&](unsigned x, unsigned y) { return w[x] > w[y]; });
        std::size_t ce = 0;
        std::vector<unsigned> group;   // values that get caller-saved registers
        for (unsigned pass = 0; pass < 2; ++pass)
            for (unsigned v : order) {
                bool pres = has_calls_ && preserved(v);
                if ((pass == 0) != pres) continue;
                if (pres) {
                    if (ce < callee.size()) reg[v] = static_cast<int>(callee[ce++]);
                } else if (group.size() < caller.size()) {
                    group.push_back(v);
                    continue;
                } else if (ce < callee.size()) {
                    reg[v] = static_cast<int>(callee[ce++]);
                }
                if (reg[v] < 0) {
                    slot[v] = 0;   // frame offset assigned in layout_frame
                    ++out_.spilled;
                }
            }
        // Passed values take the register they travel in when it is free; the others take
        // registers no passed value of this function wants first.
        bool taken[32] = {}, wanted[32] = {}, pooled[32] = {};
        for (unsigned r : caller) pooled[r] = true;
        for (unsigned v : order)
            if (pref(v) >= 0) wanted[pref(v)] = true;
        for (unsigned v : group) {
            int p = pref(v);
            if (p >= 0 && pooled[p] && !taken[p]) { reg[v] = p; taken[p] = true; }
        }
        std::vector<unsigned> pool;
        for (unsigned r : caller) if (!wanted[r]) pool.push_back(r);
        for (unsigned r : caller) if (wanted[r]) pool.push_back(r);
        std::size_t at = 0;
        for (unsigned v : group) {
            if (reg[v] >= 0) continue;
            while (taken[pool[at]]) ++at;
            reg[v] = static_cast<int>(pool[at]);
            taken[pool[at]] = true;
        }
    };
    std::size_t spilled = out_.spilled;
    if (leaf_) {
        std::vector<unsigned> pool;
        for (unsigned r : kCallerI) pool.push_back(r);
        run(NI, [&](unsigned v) { return need.has_i(v); }, [](unsigned) { return false; }, arg_reg_i, wi, pool, {}, ireg_, islot_);
        run(NF, [&](unsigned v) { return need.has_f(v); }, [](unsigned) { return false; }, arg_reg_f, wf, kCallerF, {}, freg_, fslot_);
        if (out_.spilled == spilled) return;
        leaf_ = false;
        out_.spilled = spilled;
        for (unsigned v = 0; v < NI; ++v) ireg_[v] = islot_[v] = -1;
        for (unsigned v = 0; v < NF; ++v) freg_[v] = fslot_[v] = -1;
    }
    run(NI, [&](unsigned v) { return need.has_i(v); }, [&](unsigned v) { return !mask_.has_i(v) || across_.has_i(v); },
        arg_reg_i, wi, kCallerI, kCalleeI, ireg_, islot_);
    run(NF, [&](unsigned v) { return need.has_f(v); }, [&](unsigned v) { return !mask_.has_f(v) || across_.has_f(v); },
        arg_reg_f, wf, kCallerF, kCalleeF, freg_, fslot_);
}

void Compiler::layout_frame() {
    std::set<unsigned> ci, cf, si, sf;
    for (unsigned v = 0; v < NI; ++v) {
        if (ireg_[v] >= 19) ci.insert(static_cast<unsigned>(ireg_[v]));
        else if (caller_i(ireg_[v])) si.insert(static_cast<unsigned>(ireg_[v]));
    }
    for (unsigned v = 0; v < NF; ++v) {
        if (freg_[v] >= 8 && freg_[v] <= 15) cf.insert(static_cast<unsigned>(freg_[v]));
        else if (caller_f(freg_[v])) sf.insert(static_cast<unsigned>(freg_[v]));
    }
    callee_i_.assign(ci.begin(), ci.end());
    callee_f_.assign(cf.begin(), cf.end());
    xc_ = kXC;
    xm_ = kXM;
    unsigned off = leaf_ ? 16 : 32;   // x29/x30 (and x27/x28 with a full frame)
    off += 8 * static_cast<unsigned>((callee_i_.size() + 1) & ~std::size_t(1));
    off += 8 * static_cast<unsigned>((callee_f_.size() + 1) & ~std::size_t(1));
    for (unsigned v = 0; v < NI; ++v)
        if (islot_[v] >= 0) { islot_[v] = static_cast<int>(off); off += 8; }
    for (unsigned v = 0; v < NF; ++v)
        if (fslot_[v] >= 0) { fslot_[v] = static_cast<int>(off); off += 8; }
    for (unsigned r : si) { isave_[r] = static_cast<int>(off); off += 8; }
    for (unsigned r : sf) { fsave_[r] = static_cast<int>(off); off += 8; }
    frame_ = (off + 15) & ~15u;
}

unsigned Compiler::ri(unsigned v, int into) {
    if (analyzing_) {
        note_use_i(v);
        return into >= 0 ? static_cast<unsigned>(into) : 0;
    }
    if (v < 32 && pend_[v]) {
        if (!cold_mode_) {
            resolve(v);
        } else {
            unsigned r = into >= 0 ? static_cast<unsigned>(into) : scratch_i();
            swap_value(r, static_cast<unsigned>(ireg_[v]), pend_[v]);
            return r;
        }
    }
    if (ireg_[v] >= 0) return static_cast<unsigned>(ireg_[v]);
    unsigned r = into >= 0 ? static_cast<unsigned>(into) : scratch_i();
    a_.ldrw(r, a64::SP, static_cast<unsigned>(islot_[v]));
    return r;
}

unsigned Compiler::wi(unsigned v, int into) {
    if (analyzing_) {
        cur().defs.add_i(v);
        return 1;
    }
    if (v < 32 && pend_[v] && !cold_mode_) resolve(v);   // the emitter may still read it
    if (ireg_[v] >= 0) return static_cast<unsigned>(ireg_[v]);
    return into >= 0 ? static_cast<unsigned>(into) : scratch_i();
}

void Compiler::ci(unsigned v, unsigned r) {
    if (analyzing_) return;
    if (ireg_[v] < 0) a_.strw(r, a64::SP, static_cast<unsigned>(islot_[v]));
}

unsigned Compiler::rf(unsigned v, int into) {
    if (analyzing_) {
        note_use_f(v);
        return into >= 0 ? static_cast<unsigned>(into) : 0;
    }
    if (freg_[v] >= 0) return static_cast<unsigned>(freg_[v]);
    unsigned r = into >= 0 ? static_cast<unsigned>(into) : scratch_f();
    a_.ldrd(r, a64::SP, static_cast<unsigned>(fslot_[v]));
    return r;
}

unsigned Compiler::wf(unsigned v, int into) {
    if (analyzing_) {
        cur().defs.add_f(v);
        return 1;
    }
    if (freg_[v] >= 0) return static_cast<unsigned>(freg_[v]);
    return into >= 0 ? static_cast<unsigned>(into) : scratch_f();
}

void Compiler::cf(unsigned v, unsigned r) {
    if (analyzing_) return;
    if (freg_[v] < 0) a_.strd(r, a64::SP, static_cast<unsigned>(fslot_[v]));
}

void Compiler::home_to_reg_i(unsigned v, unsigned r) {
    if (v < 32 && pend_[v]) {
        if (cold_mode_) throw std::runtime_error("deferred swap read in a cold path");
        resolve(v);
    }
    if (ireg_[v] >= 0) { if (static_cast<unsigned>(ireg_[v]) != r) a_.mov(r, static_cast<unsigned>(ireg_[v])); }
    else a_.ldrw(r, a64::SP, static_cast<unsigned>(islot_[v]));
}
void Compiler::reg_to_home_i(unsigned v, unsigned r) {
    if (ireg_[v] >= 0) { if (static_cast<unsigned>(ireg_[v]) != r) a_.mov(static_cast<unsigned>(ireg_[v]), r); }
    else a_.strw(r, a64::SP, static_cast<unsigned>(islot_[v]));
}

void Compiler::load_import(unsigned xd, const std::string& name) {
    auto it = literals_.find(name);
    int l;
    if (it == literals_.end()) {
        l = new_label();
        literals_[name] = l;
    } else {
        l = it->second;
    }
    fix(l, F_IMM19);
    a_.ldr_lit(xd, 0);
}

void Compiler::flush(const VSet& s, bool lr_imm, std::uint32_t lr_value) {
    if (analyzing_) return;
    for (unsigned v = 0; v < NI; ++v) {
        if (!s.has_i(v)) continue;
        unsigned off = ctx_off_i(v);
        unsigned r;
        if (v == V_LR && lr_imm) {
            a_.mov32(kT1, lr_value);
            r = kT1;
        } else if (v < 32 && pend_[v] && cold_mode_) {
            swap_value(kT1, static_cast<unsigned>(ireg_[v]), pend_[v]);
            r = kT1;
        } else if (ireg_[v] >= 0) {
            if (v < 32) resolve(v);
            r = static_cast<unsigned>(ireg_[v]);
        } else {
            a_.ldrw(kT1, a64::SP, static_cast<unsigned>(islot_[v]));
            r = kT1;
        }
        if (ctx_word_i(v)) a_.strw(r, xc_, off);
        else a_.strb(r, xc_, off);
    }
    for (unsigned v = 0; v < NF; ++v) {
        if (!s.has_f(v)) continue;
        if (freg_[v] >= 0) {
            a_.strd(static_cast<unsigned>(freg_[v]), xc_, ctx_off_f(v));
        } else {
            a_.ldrx(kT1, a64::SP, static_cast<unsigned>(fslot_[v]));
            a_.strx(kT1, xc_, ctx_off_f(v));
        }
    }
}

void Compiler::reload(const VSet& s) {
    if (analyzing_) return;
    for (unsigned v = 0; v < NI; ++v) {
        if (!s.has_i(v)) continue;
        unsigned off = ctx_off_i(v);
        unsigned r = ireg_[v] >= 0 ? static_cast<unsigned>(ireg_[v]) : kT1;
        if (ireg_[v] < 0 && islot_[v] < 0) continue;
        if (v < 32) pend_[v] = 0;
        if (ctx_word_i(v)) a_.ldrw(r, xc_, off);
        else a_.ldrb(r, xc_, off);
        if (ireg_[v] < 0) a_.strw(r, a64::SP, static_cast<unsigned>(islot_[v]));
    }
    for (unsigned v = 0; v < NF; ++v) {
        if (!s.has_f(v)) continue;
        if (freg_[v] >= 0) {
            a_.ldrd(static_cast<unsigned>(freg_[v]), xc_, ctx_off_f(v));
        } else if (fslot_[v] >= 0) {
            a_.ldrx(kT1, xc_, ctx_off_f(v));
            a_.strx(kT1, a64::SP, static_cast<unsigned>(fslot_[v]));
        }
    }
}

void Compiler::arg_moves(const VSet& s, bool to_regs) {
    if (analyzing_) return;
    struct Move { unsigned dst, src; };
    std::vector<Move> mi, mf;
    std::vector<std::pair<unsigned, unsigned>> si, sf;   // host register, frame offset
    for (unsigned v = 0; v < NI; ++v) {
        if (!s.has_i(v)) continue;
        if (v >= 32) throw std::runtime_error("passed value is not a GPR");
        unsigned a = static_cast<unsigned>(arg_reg_i(v));
        if (to_regs) resolve(v);
        else pend_[v] = 0;
        if (ireg_[v] >= 0) mi.push_back(to_regs ? Move{a, static_cast<unsigned>(ireg_[v])} : Move{static_cast<unsigned>(ireg_[v]), a});
        else if (islot_[v] >= 0) si.push_back({a, static_cast<unsigned>(islot_[v])});
        else if (to_regs) throw std::runtime_error("passed value without a home");
    }
    for (unsigned v = 0; v < NF; ++v) {
        if (!s.has_f(v)) continue;
        unsigned a = static_cast<unsigned>(arg_reg_f(v));
        if (freg_[v] >= 0) mf.push_back(to_regs ? Move{a, static_cast<unsigned>(freg_[v])} : Move{static_cast<unsigned>(freg_[v]), a});
        else if (fslot_[v] >= 0) sf.push_back({a, static_cast<unsigned>(fslot_[v])});
        else if (to_regs) throw std::runtime_error("passed value without a home");
    }
    // Registers into frame slots first, frame slots into registers last; register moves in
    // between, in an order that reads each source before it is overwritten.
    if (!to_regs) {
        for (auto& [r, off] : si) a_.strw(r, a64::SP, off);
        for (auto& [r, off] : sf) a_.strd(r, a64::SP, off);
    }
    auto parallel = [&](std::vector<Move>& moves, bool fp, unsigned tmp) {
        auto mov = [&](unsigned d, unsigned r) { if (fp) a_.fmov(d, r); else a_.mov(d, r); };
        moves.erase(std::remove_if(moves.begin(), moves.end(), [](const Move& m) { return m.dst == m.src; }), moves.end());
        while (!moves.empty()) {
            auto free = std::find_if(moves.begin(), moves.end(), [&](const Move& m) {
                return std::none_of(moves.begin(), moves.end(), [&](const Move& o) { return o.src == m.dst; });
            });
            if (free != moves.end()) {
                mov(free->dst, free->src);
                moves.erase(free);
                continue;
            }
            unsigned from = moves.front().src;   // a cycle: one source moves to the temporary
            mov(tmp, from);
            for (auto& m : moves)
                if (m.src == from) m.src = tmp;
        }
    };
    parallel(mi, false, kT1);
    parallel(mf, true, kScrF[3]);
    if (to_regs) {
        for (auto& [r, off] : si) a_.ldrw(r, a64::SP, off);
        for (auto& [r, off] : sf) a_.ldrd(r, a64::SP, off);
    }
}

void Compiler::return_after_call(const VSet& got) {
    for (unsigned v = 0; v < NI; ++v) {
        if (out_regs_.has_i(v) && !got.has_i(v)) a_.ldrw(static_cast<unsigned>(arg_reg_i(v)), xc_, ctx_off_i(v));
        if (got.has_i(v) && !out_regs_.has_i(v)) a_.strw(static_cast<unsigned>(arg_reg_i(v)), xc_, ctx_off_i(v));
    }
    for (unsigned v = 0; v < NF; ++v) {
        if (out_regs_.has_f(v) && !got.has_f(v)) a_.ldrd(static_cast<unsigned>(arg_reg_f(v)), xc_, ctx_off_f(v));
        if (got.has_f(v) && !out_regs_.has_f(v)) a_.strd(static_cast<unsigned>(arg_reg_f(v)), xc_, ctx_off_f(v));
    }
    if (leaf_) a_.ldstp(3, true, false, 1, a64::FP, a64::LR, a64::SP, 16);
    else epilogue();
    a_.ret();
}

VSet Compiler::keep_set() const {
    if (analyzing_) return {};
    VSet live = info_[k_].live_in | info_[k_].after;
    VSet keep;
    for (unsigned v = 0; v < NI; ++v)
        if (live.has_i(v) && caller_i(ireg_[v])) keep.add_i(v);
    for (unsigned v = 0; v < NF; ++v)
        if (live.has_f(v) && caller_f(freg_[v])) keep.add_f(v);
    return keep;
}

void Compiler::save(const VSet& keep) {
    if (leaf_) {
        a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -static_cast<int>(frame_));
        a_.add_imm(a64::FP, a64::SP, 0, true);
    }
    for (unsigned v = 0; v < NI; ++v)
        if (keep.has_i(v)) a_.strx(static_cast<unsigned>(ireg_[v]), a64::SP, static_cast<unsigned>(isave_[ireg_[v]]));
    for (unsigned v = 0; v < NF; ++v)
        if (keep.has_f(v)) a_.strd(static_cast<unsigned>(freg_[v]), a64::SP, static_cast<unsigned>(fsave_[freg_[v]]));
}

void Compiler::restore(const VSet& keep) {
    for (unsigned v = 0; v < NI; ++v)
        if (keep.has_i(v)) a_.ldrx(static_cast<unsigned>(ireg_[v]), a64::SP, static_cast<unsigned>(isave_[ireg_[v]]));
    for (unsigned v = 0; v < NF; ++v)
        if (keep.has_f(v)) a_.ldrd(static_cast<unsigned>(freg_[v]), a64::SP, static_cast<unsigned>(fsave_[freg_[v]]));
    if (leaf_) a_.ldstp(3, true, false, 1, a64::FP, a64::LR, a64::SP, static_cast<int>(frame_));
}

void Compiler::prologue() {
    if (leaf_) return;
    if (frame_ <= 504) {
        a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -static_cast<int>(frame_));
    } else {
        unsigned f = frame_;
        if (f >= 4096) { a_.addsub_imm(true, true, false, a64::SP, a64::SP, f >> 12, true); f &= 0xFFF; }
        if (f) a_.sub_imm(a64::SP, a64::SP, f, true);
        a_.ldstp(3, false, false, 2, a64::FP, a64::LR, a64::SP, 0);
    }
    a_.add_imm(a64::FP, a64::SP, 0, true);
    unsigned off = 32;
    for (std::size_t n = 0; n < callee_i_.size(); n += 2, off += 16) {
        if (n + 1 < callee_i_.size()) a_.ldstp(3, false, false, 2, callee_i_[n], callee_i_[n + 1], a64::SP, static_cast<int>(off));
        else a_.strx(callee_i_[n], a64::SP, off);
    }
    for (std::size_t n = 0; n < callee_f_.size(); n += 2, off += 16) {
        if (n + 1 < callee_f_.size()) a_.ldstp(3, false, true, 2, callee_f_[n], callee_f_[n + 1], a64::SP, static_cast<int>(off));
        else a_.strd(callee_f_[n], a64::SP, off);
    }
}

void Compiler::epilogue() {
    if (leaf_) return;
    unsigned off = 32;
    for (std::size_t n = 0; n < callee_i_.size(); n += 2, off += 16) {
        if (n + 1 < callee_i_.size()) a_.ldstp(3, true, false, 2, callee_i_[n], callee_i_[n + 1], a64::SP, static_cast<int>(off));
        else a_.ldrx(callee_i_[n], a64::SP, off);
    }
    for (std::size_t n = 0; n < callee_f_.size(); n += 2, off += 16) {
        if (n + 1 < callee_f_.size()) a_.ldstp(3, true, true, 2, callee_f_[n], callee_f_[n + 1], a64::SP, static_cast<int>(off));
        else a_.ldrd(callee_f_[n], a64::SP, off);
    }
    if (frame_ <= 504) {
        a_.ldstp(3, true, false, 1, a64::FP, a64::LR, a64::SP, static_cast<int>(frame_));
    } else {
        a_.ldstp(3, true, false, 2, a64::FP, a64::LR, a64::SP, 0);
        unsigned f = frame_;
        if (f >= 4096) { a_.addsub_imm(true, false, false, a64::SP, a64::SP, f >> 12, true); f &= 0xFFF; }
        if (f) a_.add_imm(a64::SP, a64::SP, f, true);
    }
}

void Compiler::cold(std::function<void()> fn) {
    std::array<std::uint8_t, 32> state;
    std::memcpy(state.data(), pend_, sizeof pend_);
    cold_.push_back([this, state, fn = std::move(fn)] {
        std::memcpy(pend_, state.data(), sizeof pend_);
        cold_mode_ = true;
        fn();
        cold_mode_ = false;
        std::memset(pend_, 0, sizeof pend_);
    });
}

void Compiler::add_imm32(unsigned rd, unsigned rn, std::int32_t v) {
    std::uint32_t u = static_cast<std::uint32_t>(v);
    std::uint32_t m = static_cast<std::uint32_t>(-static_cast<std::int64_t>(v));
    if (v == 0) { if (rd != rn) a_.mov(rd, rn); }
    else if (v > 0 && u < 4096) a_.add_imm(rd, rn, u);
    else if (v < 0 && m < 4096) a_.sub_imm(rd, rn, m);
    else if (v > 0 && !(u & 0xFFF) && (u >> 12) < 4096) a_.addsub_imm(false, false, false, rd, rn, u >> 12, true);
    else if (v < 0 && !(m & 0xFFF) && (m >> 12) < 4096) a_.addsub_imm(false, true, false, rd, rn, m >> 12, true);
    else { a_.mov32(kT1, u); a_.add(rd, rn, kT1); }
}

void Compiler::logic_imm32(unsigned opc, unsigned rd, unsigned rn, std::uint32_t imm) {
    if (a_.logic_imm(false, opc, rd, rn, imm)) return;
    a_.mov32(kT1, imm);
    a_.logic_reg(false, opc, false, rd, rn, kT1);
}

unsigned Compiler::ea_d(unsigned ra, std::int32_t d, bool copy) {
    std::uint32_t value;
    if (constant(ra, value)) {
        a_.mov32(kT0, value + static_cast<std::uint32_t>(d));
        return kT0;
    }
    unsigned base = ri(V_R + ra);
    if (d == 0 && !copy) return base;
    add_imm32(kT0, base, d);
    return kT0;
}

unsigned Compiler::ea_x(unsigned ra, unsigned rb, bool copy) {
    if (ra == 0) {
        unsigned b = ri(V_R + rb);
        if (!copy) return b;
        a_.mov(kT0, b);
        return kT0;
    }
    unsigned a = ri(V_R + ra), b = ri(V_R + rb);
    a_.add(kT0, a, b);
    return kT0;
}

void Compiler::set_field_from_flags(unsigned field, Cond lt, Cond gt, Cond eq, bool fp) {
    Cond c[3] = {lt, gt, eq};
    // A conditional branch right after the compare can test the flags itself.
    bool fusable = false;
    unsigned tested = 0;
    if (!analyzing_ && k_ + 1 < n_ && !label_pcs_.count(f_.start + 4 * static_cast<std::uint32_t>(k_ + 1))) {
        Op next = ops_[k_ + 1];
        Insn ni = insns_[k_ + 1];
        if ((next == Op::bc || next == Op::bclr || next == Op::bcctr) && (ni.bo() & 4) && !(ni.bo() & 16) &&
            ni.bi() / 4 == field && ni.bi() % 4 < 3) {
            fusable = true;
            tested = ni.bi();
        }
    }
    for (unsigned b = 0; b < 4; ++b) {
        iscr_ = 0;   // compare operands are consumed; each bit is committed before the next
        unsigned v = V_CR + field * 4 + b;
        bool needed;
        if (analyzing_) needed = true;
        else if (fusable && field * 4 + b == tested) needed = info_[k_ + 1].after.has_i(v);
        else needed = info_[k_ + 1].live_in.has_i(v);
        if (!needed) continue;
        if (b < 3) {
            unsigned r = wi(v);
            a_.cset(r, c[b]);
            ci(v, r);
        } else if (fp) {
            unsigned r = wi(v);
            a_.cset(r, a64::VS);
            ci(v, r);
        } else if (analyzing_) {
            wi(v);
            cur().so_copies.add_i(v);
        } else {
            unsigned s = ri(V_SO);
            set_i(v, s);
        }
    }
    flags_.valid = fusable;
    flags_.k = k_;
    flags_.field = field;
    for (unsigned b = 0; b < 3; ++b) flags_.cond[b] = c[b];
}

void Compiler::record(unsigned reg) {
    a_.cmp_imm(reg, 0);
    set_field_from_flags(0, a64::LT, a64::GT, a64::EQ, false);
}

void Compiler::record_fp() {
    a_.ldrw(kT1, xc_, offsetof(gc_ctx, fpscr));
    for (unsigned b = 0; b < 4; ++b) {
        unsigned v = V_CR + 4 + b;
        unsigned r = wi(v);
        a_.ubfx(r, kT1, 31 - b, 1);
        ci(v, r);
    }
}

bool Compiler::fused(unsigned bi, Cond& c) const {
    if (analyzing_ || !flags_.valid || flags_.k + 1 != k_ || flags_.field != bi / 4 || bi % 4 >= 3) return false;
    c = flags_.cond[bi % 4];
    return true;
}

void Compiler::emit_runtime_call(Import fn, std::uint32_t pc, std::uint32_t word) {
    VSet keep = keep_set();
    save(keep);
    a_.movx(0, xc_);
    a_.mov32(1, pc);
    a_.mov32(2, word);
    call_import(fn);
    restore(keep);
}

void Compiler::emit_load(Insn i, Op op) {
    unsigned size = 4;
    bool sign = false, brev = false, update = false, indexed = false;
    switch (op) {
    case Op::lwz: break;
    case Op::lwzu: update = true; break;
    case Op::lwzx: case Op::lwarx: indexed = true; break;
    case Op::lwzux: indexed = update = true; break;
    case Op::lbz: size = 1; break;
    case Op::lbzu: size = 1; update = true; break;
    case Op::lbzx: size = 1; indexed = true; break;
    case Op::lbzux: size = 1; indexed = update = true; break;
    case Op::lhz: size = 2; break;
    case Op::lhzu: size = 2; update = true; break;
    case Op::lhzx: size = 2; indexed = true; break;
    case Op::lhzux: size = 2; indexed = update = true; break;
    case Op::lha: size = 2; sign = true; break;
    case Op::lhau: size = 2; sign = update = true; break;
    case Op::lhax: size = 2; sign = indexed = true; break;
    case Op::lhaux: size = 2; sign = indexed = update = true; break;
    case Op::lwbrx: brev = indexed = true; break;
    case Op::lhbrx: size = 2; brev = indexed = true; break;
    default: break;
    }
    unsigned d = i.rd(), a = i.ra();
    bool check = needs_io_check(a, indexed, i.simm());
    unsigned ea = indexed ? ea_x(a, i.rb(), update) : ea_d(a, i.simm(), update);
    int cold_label = new_label(), cont = new_label();
    if (check) io_check(ea, cold_label);
    unsigned code = size == 4 ? 2 : size == 2 ? 1 : 0;
    unsigned dst = wi(V_R + d);
    a_.ldst_uxtw(code, true, false, dst, xm_, ea);
    const bool defer = !analyzing_ && !brev && size >= 2 && ireg_[V_R + d] >= 0 && !(update && a == d);
    if (defer) {
        pend_[d] = size == 4 ? 1 : sign ? 3 : 2;
    } else {
        if (size == 4 && !brev) a_.rev(dst, dst);
        if (size == 2) {
            if (!brev) a_.rev16(dst, dst);
            if (sign) a_.sxth(dst, dst);
        }
    }
    ci(V_R + d, dst);
    if (update) set_i(V_R + a, ea);
    bind(cont);
    if (!check) return;
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(cold_label);
        if (ea != kT0) a_.mov(kT0, ea);
        if (update) set_i(V_R + a, kT0);
        VSet keep = keep_set();
        save(keep);
        a_.mov(0, kT0);
        call_import(size == 4 ? I_IO_R32 : size == 2 ? I_IO_R16 : I_IO_R8);
        a_.mov(kT1, 0);
        restore(keep);
        if (size == 2) a_.ubfx(kT1, kT1, 0, 16);
        if (size == 1) a_.ubfx(kT1, kT1, 0, 8);
        if (brev && size == 4) a_.rev(kT1, kT1);
        if (brev && size == 2) a_.rev16(kT1, kT1);
        if (sign) a_.sxth(kT1, kT1);
        if (!(update && a == d)) set_i(V_R + d, kT1);
        jmp(cont);
    });
}

void Compiler::emit_store(Insn i, Op op) {
    unsigned size = 4;
    bool brev = false, update = false, indexed = false;
    switch (op) {
    case Op::stw: break;
    case Op::stwu: update = true; break;
    case Op::stwx: case Op::stwcx: indexed = true; break;
    case Op::stwux: indexed = update = true; break;
    case Op::stb: size = 1; break;
    case Op::stbu: size = 1; update = true; break;
    case Op::stbx: size = 1; indexed = true; break;
    case Op::stbux: size = 1; indexed = update = true; break;
    case Op::sth: size = 2; break;
    case Op::sthu: size = 2; update = true; break;
    case Op::sthx: size = 2; indexed = true; break;
    case Op::sthux: size = 2; indexed = update = true; break;
    case Op::stwbrx: brev = indexed = true; break;
    case Op::sthbrx: size = 2; brev = indexed = true; break;
    default: break;
    }
    unsigned s = i.rs(), a = i.ra();
    if (!indexed && !update && !brev && op != Op::stwcx && fifo_address(a, i.simm())) {
        const unsigned kind = analyzing_ ? 0u : pend_[s];
        const bool raw = (size == 4 && kind == 1) || (size == 2 && kind >= 2);
        unsigned val = raw ? static_cast<unsigned>(ireg_[V_R + s]) : ri(V_R + s);
        if (analyzing_) return;
        a_.mov(kT1, val);
        gather_inline(size, [this, size, raw](unsigned at) {
            const unsigned v = kScrI[3];
            if (size == 4) { if (raw) a_.strw(kT1, at, 0); else { a_.rev(v, kT1); a_.strw(v, at, 0); } }
            else if (size == 2) { if (!raw) a_.rev16(kT1, kT1); a_.ldst_uimm(1, false, false, kT1, at, 0); }
            else a_.ldst_uimm(0, false, false, kT1, at, 0);
        });
        return;
    }
    bool check = needs_io_check(a, indexed, i.simm());
    unsigned ea = indexed ? ea_x(a, i.rb(), update) : ea_d(a, i.simm(), update);
    // A value still holding the bytes as loaded is stored as it is (both swaps cancel).
    const unsigned kind = analyzing_ || brev || op == Op::stwcx ? 0u : pend_[s];
    const bool raw = (size == 4 && kind == 1) || (size == 2 && kind >= 2);
    unsigned val = raw ? static_cast<unsigned>(ireg_[V_R + s]) : ri(V_R + s);
    int cold_label = new_label(), cont = new_label();
    if (check) io_check(ea, cold_label);
    unsigned out = val;
    if (size == 4 && !brev && !raw) { a_.rev(kT1, val); out = kT1; }
    if (size == 2 && !brev && !raw) { a_.rev16(kT1, val); out = kT1; }
    unsigned code = size == 4 ? 2 : size == 2 ? 1 : 0;
    a_.ldst_uxtw(code, false, false, out, xm_, ea);
    if (update) set_i(V_R + a, ea);
    bind(cont);
    if (op == Op::stwcx) {
        for (unsigned b = 0; b < 3; ++b) {
            iscr_ = 0;
            unsigned v = V_CR + b;
            unsigned r = wi(v);
            a_.movz(r, b == 2 ? 1 : 0);
            ci(v, r);
        }
        iscr_ = 0;
        set_i(V_CR + 3, ri(V_SO));
    }
    if (!check) return;
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(cold_label);
        a_.mov(kT1, val);
        if (raw) { if (size == 4) a_.rev(kT1, kT1); else a_.rev16(kT1, kT1); }
        if (brev && size == 4) a_.rev(kT1, kT1);
        if (brev && size == 2) a_.rev16(kT1, kT1);
        if (size == 2) a_.ubfx(kT1, kT1, 0, 16);
        if (size == 1) a_.ubfx(kT1, kT1, 0, 8);
        if (ea != kT0) a_.mov(kT0, ea);
        if (update) set_i(V_R + a, kT0);
        int io = new_label();
        gather_append(size, [this, size](unsigned at) {
            const unsigned v = kScrI[3];
            if (size == 4) { a_.rev(v, kT1); a_.strw(v, at, 0); }
            else if (size == 2) { a_.rev16(v, kT1); a_.ldst_uimm(1, false, false, v, at, 0); }
            else a_.ldst_uimm(0, false, false, kT1, at, 0);
        }, io, cont);
        bind(io);
        VSet keep = keep_set();
        save(keep);
        a_.mov(0, kT0);
        a_.mov(1, kT1);
        call_import(size == 4 ? I_IO_W32 : size == 2 ? I_IO_W16 : I_IO_W8);
        restore(keep);
        jmp(cont);
    });
}

// lmw/stmw: stack and small-data bases never address hardware registers; other bases are
// checked once and a hardware address is reported as an illegal access.
void Compiler::emit_lmw(std::uint32_t pc, Insn i, bool load) {
    unsigned d = i.rd(), a = i.ra();
    unsigned ea = ea_d(a, i.simm(), true);
    int bad = new_label();
    bool check = needs_io_check(a, false, i.simm());
    if (check) io_check(ea, bad);
    a_.add_uxtw(kT0, xm_, ea);
    for (unsigned r = d; r < 32; r += 2) {
        iscr_ = 0;
        unsigned off = (r - d) * 4;
        bool pair = r + 1 < 32;
        if (load) {
            unsigned x = wi(V_R + r);
            unsigned y = pair ? wi(V_R + r + 1) : 0;
            if (pair && x != y) a_.ldstp(2, true, false, 2, x, y, kT0, static_cast<int>(off));
            else { a_.ldrw(x, kT0, off); if (pair) a_.ldrw(y, kT0, off + 4); }
            a_.rev(x, x);
            ci(V_R + r, x);
            if (pair) { a_.rev(y, y); ci(V_R + r + 1, y); }
        } else {
            unsigned x = ri(V_R + r);
            a_.rev(kT1, x);
            if (pair) {
                unsigned y = ri(V_R + r + 1);
                a_.rev(14, y);
                a_.ldstp(2, false, false, 2, kT1, 14, kT0, static_cast<int>(off));
            } else {
                a_.strw(kT1, kT0, off);
            }
        }
    }
    if (!check) return;
    std::size_t k = k_;
    std::uint32_t word = i.w;
    cold([=] {
        k_ = k;
        bind(bad);
        emit_runtime_call(I_ILLEGAL, pc, word);
    });
}

void Compiler::emit_string(Insn i, bool load) {
    unsigned d = i.rd(), a = i.ra(), nb = i.rb() ? i.rb() : 32;
    // Byte by byte (rare instructions); the base stays in kT0 and is re-read after calls.
    auto base = [&] {
        if (a) a_.mov(kT0, ri(V_R + a));
        else a_.movz(kT0, 0);
    };
    base();
    for (unsigned k = 0; k < nb; ++k) {
        unsigned r = (d + k / 4) % 32, shift = 24 - 8 * (k % 4);
        iscr_ = 0;
        int cold_label = new_label(), cont = new_label();
        if (!load) a_.lsr_imm(15, ri(V_R + r), shift);
        add_imm32(kT1, kT0, static_cast<std::int32_t>(k));
        io_check(kT1, cold_label, 14);
        a_.ldst_uxtw(0, load, false, 15, xm_, kT1);
        bind(cont);
        if (load) {
            iscr_ = 0;
            unsigned dst = wi(V_R + r);
            if (k % 4 == 0) {
                a_.lsl_imm(dst, 15, shift);
            } else {
                unsigned prev = ri(V_R + r);
                a_.orr(dst, prev, 15, 0, shift);
            }
            ci(V_R + r, dst);
        }
        std::size_t kk = k_;
        cold([=] {
            k_ = kk;
            iscr_ = 0;
            bind(cold_label);
            VSet keep = keep_set();
            save(keep);
            a_.mov(0, kT1);
            if (load) {
                call_import(I_IO_R8);
                a_.ubfx(15, 0, 0, 8);
            } else {
                a_.ubfx(1, 15, 0, 8);
                call_import(I_IO_W8);
            }
            restore(keep);
            if (a) a_.mov(kT0, ri(V_R + a));
            else a_.movz(kT0, 0);
            jmp(cont);
        });
    }
}

void Compiler::emit_fload(Insn i, Op op) {
    bool indexed = op == Op::lfsx || op == Op::lfsux || op == Op::lfdx || op == Op::lfdux;
    bool update = op == Op::lfsu || op == Op::lfsux || op == Op::lfdu || op == Op::lfdux;
    bool single = op == Op::lfs || op == Op::lfsu || op == Op::lfsx || op == Op::lfsux;
    unsigned d = i.rd(), a = i.ra();
    bool check = needs_io_check(a, indexed, i.simm());
    unsigned ea = indexed ? ea_x(a, i.rb(), update) : ea_d(a, i.simm(), update);
    int cold_label = new_label(), conv = new_label();
    if (check) io_check(ea, cold_label);
    if (single) {
        a_.ldst_uxtw(2, true, false, kT1, xm_, ea);
        a_.rev(kT1, kT1);
    } else {
        a_.ldst_uxtw(3, true, false, kT1, xm_, ea);
        a_.rev64(kT1, kT1);
    }
    if (update) set_i(V_R + a, ea);
    bind(conv);
    if (single) {
        a_.fmov_to_s(31, kT1);
        unsigned fd = wf(V_F + d, 31);
        a_.fcvt_ds(fd, 31);
        cf(V_F + d, fd);
        unsigned pd = wf(V_P + d);
        a_.fmov(pd, fd);
        cf(V_P + d, pd);
    } else {
        unsigned fd = wf(V_F + d, 31);
        a_.fmov_to_d(fd, kT1);
        cf(V_F + d, fd);
    }
    if (!check) return;
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(cold_label);
        if (ea != kT0) a_.mov(kT0, ea);
        if (update) set_i(V_R + a, kT0);
        VSet keep = keep_set();
        save(keep);
        a_.mov(0, kT0);
        call_import(single ? I_IO_R32 : I_IO_R64);
        a_.movx(kT1, 0);
        restore(keep);
        if (single) a_.mov(kT1, kT1);
        jmp(conv);
    });
}

void Compiler::emit_fstore(Insn i, Op op) {
    bool indexed = op == Op::stfsx || op == Op::stfsux || op == Op::stfdx || op == Op::stfdux || op == Op::stfiwx;
    bool update = op == Op::stfsu || op == Op::stfsux || op == Op::stfdu || op == Op::stfdux;
    bool single = op == Op::stfs || op == Op::stfsu || op == Op::stfsx || op == Op::stfsux;
    bool word = single || op == Op::stfiwx;
    unsigned s = i.rs(), a = i.ra();
    if (!indexed && !update && op != Op::stfiwx && fifo_address(a, i.simm())) {
        unsigned fs = rf(V_F + s);
        if (analyzing_) return;
        if (single) {
            a_.fcvt_sd(31, fs);
            a_.rev32_8b(31, 31);
        } else {
            a_.fmov(31, fs);
        }
        gather_inline(single ? 4 : 8, [this, single](unsigned at) {
            if (single) {
                a_.ldst_uimm(2, false, true, 31, at, 0);
            } else {
                a_.fmov_from_d(kT1, 31);
                a_.rev64(kT1, kT1);
                a_.ldst_uimm(3, false, false, kT1, at, 0);
            }
        });
        return;
    }
    bool check = needs_io_check(a, indexed, i.simm());
    unsigned fs = rf(V_F + s);
    unsigned ea = indexed ? ea_x(a, i.rb(), update) : ea_d(a, i.simm(), update);
    int cold_label = new_label(), cont = new_label();
    if (single) {
        a_.fcvt_sd(31, fs);
        a_.fmov_from_s(15, 31);
    } else {
        a_.fmov_from_d(15, fs);
        if (op == Op::stfiwx) a_.mov(15, 15);
    }
    if (check) io_check(ea, cold_label);
    if (word) {
        a_.rev(kT1, 15);
        a_.ldst_uxtw(2, false, false, kT1, xm_, ea);
    } else {
        a_.rev64(kT1, 15);
        a_.ldst_uxtw(3, false, false, kT1, xm_, ea);
    }
    if (update) set_i(V_R + a, ea);
    bind(cont);
    if (!check) return;
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(cold_label);
        if (ea != kT0) a_.mov(kT0, ea);
        if (update) set_i(V_R + a, kT0);
        a_.movx(kT1, 15);
        int io = new_label();
        gather_append(word ? 4 : 8, [this, word](unsigned at) {
            const unsigned v = kScrI[3];
            if (word) { a_.rev(v, kT1); a_.strw(v, at, 0); }
            else { a_.rev64(v, kT1); a_.ldst_uimm(3, false, false, v, at, 0); }
        }, io, cont);
        bind(io);
        VSet keep = keep_set();
        save(keep);
        a_.mov(0, kT0);
        a_.movx(1, kT1);
        call_import(word ? I_IO_W32 : I_IO_W64);
        restore(keep);
        jmp(cont);
    });
}

// In a cold hardware-register path with the address in kT0: a store to the write-gather
// pipe (0xCC008000) appends its bytes to gc_gp; store writes the size bytes at the address
// in its register. A full batch goes to gc_gather_burst, which keeps the bytes past its
// last complete burst. Other addresses continue at other.
void Compiler::gather_append(unsigned size, const std::function<void(unsigned at)>& store, int other, int cont) {
    const unsigned gp = kScrI[0], fill = kScrI[1], at = kScrI[2];
    a_.movz(fill, 0x8000);
    a_.movk(fill, 0xCC00, 1);
    a_.cmp(kT0, fill);
    jcc(a64::NE, other);
    load_import(gp, "gc_gp");
    a_.ldrw(fill, gp, offsetof(gc_gather, fill));
    a_.add_uxtw(at, gp, fill);
    store(at);
    a_.add_imm(fill, fill, size);
    a_.strw(fill, gp, offsetof(gc_gather, fill));
    a_.cmp_imm(fill, GC_GATHER_BATCH);
    jcc(a64::LO, cont);
    VSet keep = keep_set();
    save(keep);
    call_import("gc_gather_burst");
    restore(keep);
    jmp(cont);
}

// A store whose address is known to be the write-gather pipe appends in line; store writes
// the size bytes at the address in its register (any of kScrI[3], kT0, kT1 hold the value).
void Compiler::gather_inline(unsigned size, const std::function<void(unsigned at)>& store) {
    const unsigned gp = kScrI[0], fill = kScrI[1], at = kScrI[2];
    load_import(gp, "gc_gp");
    a_.ldrw(fill, gp, offsetof(gc_gather, fill));
    a_.add_uxtw(at, gp, fill);
    store(at);
    a_.add_imm(fill, fill, size);
    a_.strw(fill, gp, offsetof(gc_gather, fill));
    a_.cmp_imm(fill, GC_GATHER_BATCH);
    int burst = new_label(), cont = new_label();
    jcc(a64::HS, burst);
    bind(cont);
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(burst);
        VSet keep = keep_set();
        save(keep);
        call_import("gc_gather_burst");
        restore(keep);
        jmp(cont);
    });
}

void Compiler::emit_psq(Insn i, Op op) {
    bool load = op == Op::psq_l || op == Op::psq_lu || op == Op::psq_lx || op == Op::psq_lux;
    bool indexed = op == Op::psq_lx || op == Op::psq_lux || op == Op::psq_stx || op == Op::psq_stux;
    bool update = op == Op::psq_lu || op == Op::psq_lux || op == Op::psq_stu || op == Op::psq_stux;
    unsigned w = indexed ? i.psqx_w() : i.psq_w(), q = indexed ? i.psqx_i() : i.psq_i();
    unsigned d = i.rd(), a = i.ra();
    bool check = needs_io_check(a, indexed, indexed ? 0 : i.psq_d());
    unsigned f0 = 0, f1 = 0;
    if (!load) {
        f0 = rf(V_F + d, 30);
        if (!w) f1 = rf(V_P + d, 31);
    }
    if (indexed) ea_x(a, i.rb(), true);   // EA in kT0
    else ea_d(a, i.psq_d(), true);
    int generic = new_label(), cont = new_label();
    // Only the float format is handled inline; others go through the runtime's helper.
    // GQR0 always holds it (mtspr GQR0 checks).
    if (q != 0) {
        a_.ldrw(kT1, xc_, offsetof(gc_ctx, gqr) + 4 * q);
        a_.tst_imm(kT1, load ? 0x70000u : 0x7u);
        jcc(a64::NE, generic);
    }
    if (check) io_check(kT0, generic);
    if (update) set_i(V_R + a, kT0);
    // Both halves move as one 64-bit vector: byte swap per word, then convert per lane.
    if (load) {
        if (w) {
            a_.ldst_uxtw(2, true, true, 30, xm_, kT0);
            a_.rev32_8b(30, 30);
            unsigned fd = wf(V_F + d, 30);
            a_.fcvt_ds(fd, 30);
            cf(V_F + d, fd);
            unsigned pd = wf(V_P + d, 31);
            a_.fmov_imm(pd, 0x70);
            cf(V_P + d, pd);
        } else {
            a_.ldst_uxtw(3, true, true, 30, xm_, kT0);
            a_.rev32_8b(30, 30);
            a_.fcvtl_2d(30, 30);
            unsigned pd = wf(V_P + d, 31);
            a_.dup_d1(pd, 30);
            cf(V_P + d, pd);
            unsigned fd = wf(V_F + d, 30);
            a_.fmov(fd, 30);
            cf(V_F + d, fd);
        }
    } else if (w) {
        a_.fcvt_sd(29, f0);
        a_.rev32_8b(29, 29);
        a_.ldst_uxtw(2, false, true, 29, xm_, kT0);
    } else {
        a_.fmov(29, f0);
        a_.ins_d1(29, f1);
        a_.fcvtn_2s(29, 29);
        a_.rev32_8b(29, 29);
        a_.ldst_uxtw(3, false, true, 29, xm_, kT0);
    }
    bind(cont);
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        iscr_ = fscr_ = 0;
        bind(generic);
        if (update) set_i(V_R + a, kT0);
        VSet keep = keep_set();
        if (!load) {
            unsigned x0 = rf(V_F + d, 30);
            a_.fmov(30, x0);
            if (!w) { unsigned x1 = rf(V_P + d, 31); a_.fmov(31, x1); }
            // Float quantization to the write-gather pipe (GX matrix and vertex uploads).
            int io = new_label();
            a_.ldrw(kScrI[3], xc_, offsetof(gc_ctx, gqr) + 4 * q);
            a_.tst_imm(kScrI[3], 0x7u);
            jcc(a64::NE, io);
            gather_append(w ? 4 : 8, [this, w](unsigned at) {
                const unsigned v = kScrI[3];
                a_.fcvt_sd(29, 30);
                a_.fmov_from_s(v, 29);
                a_.rev(v, v);
                a_.strw(v, at, 0);
                if (!w) {
                    a_.fcvt_sd(29, 31);
                    a_.fmov_from_s(v, 29);
                    a_.rev(v, v);
                    a_.strw(v, at, 4);
                }
            }, io, cont);
            bind(io);
        }
        save(keep);
        a_.mov(1, kT0);
        a_.movx(0, xc_);
        a_.movz(2, w);
        a_.movz(3, q);
        if (load) {
            call_import(I_PSQ_L);
            a_.fmov(30, 0);
            a_.fmov(31, 1);
            restore(keep);
            set_f(V_F + d, 30);
            set_f(V_P + d, 31);
        } else {
            a_.fmov(0, 30);
            a_.fmov(1, w ? 30 : 31);
            call_import(I_PSQ_ST);
            restore(keep);
        }
        jmp(cont);
    });
}

void Compiler::emit_arith(Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    bool oe = i.oe();
    // Sets C from XER[CA] (C = CA because CA is 0 or 1).
    auto carry_in = [&] { unsigned c = ri(V_CA); a_.cmp_imm(c, 1); };
    auto carry_out = [&] { unsigned c = wi(V_CA); a_.cset(c, a64::HS); ci(V_CA, c); };
    auto overflow = [&] {
        unsigned o = wi(V_OV);
        a_.cset(o, a64::VS);
        ci(V_OV, o);
        sticky_overflow();
    };
    auto minus_one = [&] { a_.movn(kT1, 0); return kT1; };
    switch (op) {
    case Op::add: case Op::addc: case Op::adde: case Op::addze: case Op::addme:
    case Op::subf: case Op::subfc: case Op::subfe: case Op::subfze: case Op::subfme: {
        bool sets_ca = op != Op::add && op != Op::subf;
        bool flags = sets_ca || oe;
        unsigned x = ri(V_R + a);
        unsigned y = 0;
        bool uses_b = op == Op::add || op == Op::addc || op == Op::adde || op == Op::subf || op == Op::subfc || op == Op::subfe;
        if (uses_b) y = ri(V_R + b);
        if (op == Op::adde || op == Op::addze || op == Op::addme || op == Op::subfe || op == Op::subfze || op == Op::subfme)
            carry_in();
        unsigned r = wi(V_R + d);
        switch (op) {
        case Op::add: a_.addsub_reg(false, false, flags, r, x, y); break;
        case Op::addc: a_.addsub_reg(false, false, true, r, x, y); break;
        case Op::adde: a_.adc(r, x, y, true); break;
        case Op::addze: a_.adc(r, x, a64::ZR, true); break;
        case Op::addme: a_.adc(r, x, minus_one(), true); break;
        case Op::subf: a_.addsub_reg(false, true, flags, r, y, x); break;
        case Op::subfc: a_.addsub_reg(false, true, true, r, y, x); break;
        case Op::subfe: a_.sbc(r, y, x, true); break;
        case Op::subfze: a_.sbc(r, a64::ZR, x, true); break;
        case Op::subfme: a_.sbc(r, minus_one(), x, true); break;
        default: break;
        }
        // The write-back does not change NZCV; CA and OV are read from the flags after it.
        ci(V_R + d, r);
        iscr_ = 0;
        if (sets_ca) carry_out();
        iscr_ = 0;
        if (oe) overflow();
        break;
    }
    case Op::neg: {
        unsigned x = ri(V_R + a);
        if (oe) {
            a_.mov32(kT1, 0x80000000u);
            a_.cmp(x, kT1);
            a_.cset(kT0, a64::EQ);
        }
        a_.neg(kT1, x);
        set_i(V_R + d, kT1);
        if (oe) { iscr_ = 0; set_i(V_OV, kT0); sticky_overflow(); }
        break;
    }
    case Op::mullw: {
        unsigned x = ri(V_R + a), y = ri(V_R + b);
        if (oe) {
            a_.smull(kT1, x, y);
            a_.cmp_sxtw(kT1, kT1);
            a_.cset(kT0, a64::NE);
            a_.mul(kT1, x, y);
            set_i(V_R + d, kT1);
            iscr_ = 0;
            set_i(V_OV, kT0);
            sticky_overflow();
            break;
        }
        unsigned r = wi(V_R + d);
        a_.mul(r, x, y);
        ci(V_R + d, r);
        break;
    }
    case Op::mulhw: case Op::mulhwu: {
        unsigned x = ri(V_R + a), y = ri(V_R + b);
        unsigned r = wi(V_R + d);
        if (op == Op::mulhw) a_.smull(r, x, y);
        else a_.umull(r, x, y);
        a_.lsr_imm(r, r, 32, true);
        ci(V_R + d, r);
        break;
    }
    case Op::divw: {
        unsigned x = ri(V_R + a), y = ri(V_R + b);
        // gc_divw: division by zero and INT_MIN / -1 give x < 0 ? -1 : 0.
        int special = new_label(), done = new_label();
        if (oe) {
            a_.mov32(kT1, 0x80000000u);
            a_.cmp(x, kT1);
            a_.ccmn_imm(y, 1, 0, a64::EQ);
            a_.cset(kT0, a64::EQ);
            a_.cmp_imm(y, 0);
            a_.csinc(kT0, kT0, a64::ZR, a64::NE);
        }
        jz(y, special, false);
        a_.mov32(kT1, 0x80000000u);
        a_.cmp(x, kT1);
        a_.ccmn_imm(y, 1, 0, a64::EQ);
        jcc(a64::EQ, special);
        a_.sdiv(kT1, x, y);
        jmp(done);
        bind(special);
        a_.asr_imm(kT1, x, 31);
        bind(done);
        set_i(V_R + d, kT1);
        if (oe) { iscr_ = 0; set_i(V_OV, kT0); sticky_overflow(); }
        break;
    }
    case Op::divwu: {
        unsigned x = ri(V_R + a), y = ri(V_R + b);
        if (oe) {
            a_.cmp_imm(y, 0);
            a_.cset(kT0, a64::EQ);
        }
        a_.udiv(kT1, x, y);
        set_i(V_R + d, kT1);
        if (oe) { iscr_ = 0; set_i(V_OV, kT0); sticky_overflow(); }
        break;
    }
    default: break;
    }
    if (i.rcbit()) { iscr_ = 0; record(ri(V_R + d)); }
}

void Compiler::emit_rotate(Insn i, Op op) {
    unsigned s = i.rs(), a = i.ra(), sh = i.sh(), mb = i.mb(), me = i.me();
    std::uint32_t m = gcr::rotate_mask(mb, me);
    bool full = m == 0xFFFFFFFFu;
    if (op == Op::rlwinm) {
        unsigned x = ri(V_R + s);
        unsigned r = wi(V_R + a);
        unsigned p = 31 - me, width = me - mb + 1;      // mask as a bit run [p, p + width)
        unsigned src = (p + 32 - sh) % 32;              // source bit that lands at p
        if (full) {
            if (sh) a_.ror_imm(r, x, (32 - sh) % 32);
            else if (r != x) a_.mov(r, x);
        } else if (mb <= me && src + width <= 32 && p == src) {
            a_.logic_imm(false, 0, r, x, m);
        } else if (mb <= me && src + width <= 32 && p == 0) {
            a_.ubfx(r, x, src, width);
        } else if (mb <= me && src == 0) {
            a_.ubfiz(r, x, p, width);
        } else {
            a_.ror_imm(r, x, (32 - sh) % 32);
            a_.logic_imm(false, 0, r, r, m);
        }
        ci(V_R + a, r);
    } else if (op == Op::rlwimi) {
        unsigned x = ri(V_R + s);
        unsigned old = ri(V_R + a);
        if (full) {
            if (sh) a_.ror_imm(kT1, x, (32 - sh) % 32);
            else a_.mov(kT1, x);
            set_i(V_R + a, kT1);
        } else if (mb <= me) {
            unsigned p = 31 - me, width = me - mb + 1;
            unsigned src = (p + 32 - sh) % 32;
            unsigned from = x;
            if (src) { a_.ror_imm(kT1, x, src); from = kT1; }
            unsigned r = wi(V_R + a);
            if (r != old) a_.mov(r, old);
            a_.bfi(r, from, p, width);
            ci(V_R + a, r);
        } else {
            if (sh) a_.ror_imm(kT1, x, (32 - sh) % 32);
            else a_.mov(kT1, x);
            a_.logic_imm(false, 0, kT1, kT1, m);
            a_.logic_imm(false, 0, kT0, old, ~m);
            unsigned r = wi(V_R + a);
            a_.orr(r, kT0, kT1);
            ci(V_R + a, r);
        }
    } else {   // rlwnm
        unsigned x = ri(V_R + s), n = ri(V_R + i.rb());
        a_.neg(kT1, n);
        unsigned r = wi(V_R + a);
        a_.rorv(r, x, kT1);
        if (!full) a_.logic_imm(false, 0, r, r, m);
        ci(V_R + a, r);
    }
    if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
}

void Compiler::emit_fp(std::uint32_t pc, Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb(), c = i.rc();
    auto A = [&] { return rf(V_F + a, 28); };
    auto B = [&] { return rf(V_F + b, 29); };
    auto C = [&] { return rf(V_F + c, 30); };
    auto dst = [&] { return wf(V_F + d, 31); };
    auto single_done = [&](unsigned r) {
        rounding(r);
        cf(V_F + d, r);
        unsigned p = wf(V_P + d, 30);
        a_.fmov(p, r);
        cf(V_P + d, p);
    };
    auto one = [&](unsigned reg) { a_.fmov_imm(reg, 0x70); };
    switch (op) {
    case Op::fdivs: case Op::fsubs: case Op::fadds: case Op::fdiv: case Op::fsub: case Op::fadd: {
        unsigned x = A(), y = B(), r = dst();
        bool single = op == Op::fdivs || op == Op::fsubs || op == Op::fadds;
        if (op == Op::fdivs || op == Op::fdiv) a_.fdiv(r, x, y);
        else if (op == Op::fsubs || op == Op::fsub) a_.fsub(r, x, y);
        else a_.fadd(r, x, y);
        if (single) single_done(r); else cf(V_F + d, r);
        break;
    }
    case Op::fmuls: case Op::fmul: {
        unsigned x = A(), z = C(), r = dst();
        a_.fmul(r, x, z);
        if (op == Op::fmuls) single_done(r); else cf(V_F + d, r);
        break;
    }
    case Op::fres: {
        unsigned y = B();
        one(28);
        unsigned r = dst();
        a_.fdiv(r, 28, y);
        single_done(r);
        break;
    }
    case Op::frsqrte: {
        unsigned y = B();
        a_.fsqrt(29, y);
        one(28);
        unsigned r = dst();
        a_.fdiv(r, 28, 29);
        cf(V_F + d, r);
        break;
    }
    case Op::fmadds: case Op::fmsubs: case Op::fnmadds: case Op::fnmsubs:
    case Op::fmadd: case Op::fmsub: case Op::fnmadd: case Op::fnmsub: {
        unsigned x = A(), y = B(), z = C(), r = dst();
        switch (op) {
        case Op::fmadds: case Op::fmadd: a_.fmadd(r, x, z, y); break;
        case Op::fmsubs: case Op::fmsub: a_.fnmsub(r, x, z, y); break;              // a*c - b
        case Op::fnmadds: a_.fmadd(r, x, z, y); a_.fneg(r, r); break;               // -(a*c + b)
        case Op::fnmsubs: a_.fnmsub(r, x, z, y); a_.fneg(r, r); break;              // -(a*c - b)
        case Op::fnmadd: a_.fnmadd(r, x, z, y); break;                              // as clang: -b - a*c
        case Op::fnmsub: a_.fmsub(r, x, z, y); break;                               // as clang: b - a*c
        default: break;
        }
        if (op == Op::fmadds || op == Op::fmsubs || op == Op::fnmadds || op == Op::fnmsubs) single_done(r);
        else cf(V_F + d, r);
        break;
    }
    case Op::fsel: {
        unsigned x = A(), y = B(), z = C(), r = dst();
        a_.fcmp_zero(x);
        a_.fcsel(r, z, y, a64::GE);
        cf(V_F + d, r);
        break;
    }
    case Op::fneg: case Op::fabs: case Op::fnabs: case Op::fmr: {
        unsigned y = B(), r = dst();
        if (op == Op::fneg) a_.fneg(r, y);
        else if (op == Op::fabs) a_.fabs(r, y);
        else if (op == Op::fnabs) { a_.fabs(r, y); a_.fneg(r, r); }
        else a_.fmov(r, y);
        cf(V_F + d, r);
        break;
    }
    case Op::frsp: {
        unsigned y = B(), r = dst();
        a_.fmov(r, y);
        single_done(r);
        break;
    }
    case Op::fctiwz: {
        unsigned y = B();
        a_.fcvtzs_wd(kT1, y);
        a_.fcmp(y, y);
        a_.mov32(kT0, 0x80000000u);
        a_.csel(kT1, kT0, kT1, a64::VS);
        a_.movz(kT0, 0xFFF8, 3, true);
        a_.logic_reg(true, 1, false, kT1, kT0, kT1);
        unsigned r = dst();
        a_.fmov_to_d(r, kT1);
        cf(V_F + d, r);
        break;
    }
    case Op::fctiw: {
        unsigned y = B();
        a_.fmov(31, y);
        VSet keep = keep_set();
        save(keep);
        a_.fmov(0, 31);
        a_.movx(0, xc_);
        call_import(I_FCTIW);
        a_.fmov(31, 0);
        restore(keep);
        set_f(V_F + d, 31);
        break;
    }
    case Op::fcmpu: case Op::fcmpo: {
        unsigned x = A(), y = B();
        a_.fcmp(x, y);
        set_field_from_flags(i.crfd(), a64::MI, a64::GT, a64::EQ, true);
        return;
    }
    case Op::mffs: {
        a_.ldrw(kT1, xc_, offsetof(gc_ctx, fpscr));
        a_.movz(kT0, 0xFFF8, 3, true);
        a_.logic_reg(true, 1, false, kT1, kT0, kT1);
        unsigned r = dst();
        a_.fmov_to_d(r, kT1);
        cf(V_F + d, r);
        break;
    }
    case Op::mtfsf: {
        unsigned y = B();
        a_.fmov_from_d(kT1, y);
        VSet keep = keep_set();
        save(keep);
        a_.mov(2, kT1);
        a_.movz(1, i.fm());
        a_.movx(0, xc_);
        call_import(I_MTFSF);
        restore(keep);
        break;
    }
    case Op::mtfsfi: {
        unsigned n = i.crfd(), imm = (i.w >> 12) & 15, sh = 28 - 4 * n;
        a_.ldrw(kT1, xc_, offsetof(gc_ctx, fpscr));
        a_.logic_imm(false, 0, kT1, kT1, ~(0xFu << sh));
        if (imm) {
            a_.mov32(kT0, imm << sh);
            a_.orr(kT1, kT1, kT0);
        }
        a_.strw(kT1, xc_, offsetof(gc_ctx, fpscr));
        break;
    }
    case Op::mtfsb0: case Op::mtfsb1: {
        a_.ldrw(kT1, xc_, offsetof(gc_ctx, fpscr));
        std::uint32_t bit = 1u << (31 - d);
        a_.logic_imm(false, op == Op::mtfsb1 ? 1 : 0, kT1, kT1, op == Op::mtfsb1 ? bit : ~bit);
        a_.strw(kT1, xc_, offsetof(gc_ctx, fpscr));
        break;
    }
    case Op::mcrfs: {
        unsigned n = i.crfd(), s = i.crfs();
        a_.ldrw(kT1, xc_, offsetof(gc_ctx, fpscr));
        a_.ubfx(kT1, kT1, 28 - 4 * s, 4);
        for (unsigned b = 0; b < 4; ++b) {
            unsigned v = V_CR + n * 4 + b;
            unsigned r = wi(v);
            a_.ubfx(r, kT1, 3 - b, 1);
            ci(v, r);
        }
        break;
    }
    default:
        emit_runtime_call(I_ILLEGAL, pc, i.w);
        return;
    }
    if (i.rcbit()) record_fp();
}

void Compiler::emit_ps(Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb(), c = i.rc();
    // Lane 0 into d31, lane 1 into d30; operands loaded into d28/d29/d30 when spilled.
    enum Kind { ADD, SUB, MUL, DIV, MADD, MSUB, NMADD, NMSUB, RES, RSQRTE, SEL, COPY_RS, NEG, ABS, NABS, COPY };
    auto exact = [](Kind k) { return k == NEG || k == ABS || k == NABS || k == COPY; };
    auto lane = [&](unsigned out, Kind kind, unsigned xa, unsigned xb, unsigned xc) {
        switch (kind) {
        case ADD: a_.fadd(out, xa, xb); break;
        case SUB: a_.fsub(out, xa, xb); break;
        case MUL: a_.fmul(out, xa, xc); break;
        case DIV: a_.fdiv(out, xa, xb); break;
        case MADD: a_.fmadd(out, xa, xc, xb); break;
        case MSUB: a_.fnmsub(out, xa, xc, xb); break;
        case NMADD: a_.fmadd(out, xa, xc, xb); a_.fneg(out, out); break;
        case NMSUB: a_.fnmsub(out, xa, xc, xb); a_.fneg(out, out); break;
        case RES: a_.fmov_imm(28, 0x70); a_.fdiv(out, 28, xb); break;
        case RSQRTE: a_.fsqrt(out, xb); a_.fmov_imm(28, 0x70); a_.fdiv(out, 28, out); break;
        case SEL: a_.fcmp_zero(xa); a_.fcsel(out, xc, xb, a64::GE); break;
        case COPY_RS: case COPY: a_.fmov(out, xb); break;
        case NEG: a_.fneg(out, xb); break;
        case ABS: a_.fabs(out, xb); break;
        case NABS: a_.fabs(out, xb); a_.fneg(out, out); break;
        }
        // Rounded results stay single in the scratch; finish() widens them into the home.
        if (!exact(kind)) a_.fcvt_sd(out, out);
    };
    auto uses_a = [](Kind k) { return k != RES && k != RSQRTE && k != COPY_RS && k != COPY && k != NEG && k != ABS && k != NABS; };
    auto uses_b = [](Kind k) { return k != MUL; };
    auto uses_c = [](Kind k) { return k == MUL || k == MADD || k == MSUB || k == NMADD || k == NMSUB || k == SEL; };
    struct Lane { Kind kind; unsigned va, vb, vc; };
    auto run = [&](Lane l0, Lane l1) {
        auto one = [&](unsigned out, Lane l) {
            unsigned xa = uses_a(l.kind) ? rf(l.va, 28) : 28;
            unsigned xb = uses_b(l.kind) ? rf(l.vb, 29) : 29;
            unsigned xc = uses_c(l.kind) ? rf(l.vc, 30) : 30;
            lane(out, l.kind, xa, xb, xc);
        };
        one(31, l0);
        one(30, l1);
        auto finish = [&](unsigned v, unsigned out, Kind k) {
            unsigned h = wf(v, static_cast<int>(out));
            if (exact(k)) a_.fmov(h, out);
            else a_.fcvt_ds(h, out);
            cf(v, h);
        };
        finish(V_F + d, 31, l0.kind);
        finish(V_P + d, 30, l1.kind);
    };
    unsigned fa = V_F + a, pa = V_P + a, fb = V_F + b, pb = V_P + b, fc = V_F + c, pc = V_P + c;
    switch (op) {
    case Op::ps_add: run({ADD, fa, fb, 0}, {ADD, pa, pb, 0}); break;
    case Op::ps_sub: run({SUB, fa, fb, 0}, {SUB, pa, pb, 0}); break;
    case Op::ps_mul: run({MUL, fa, 0, fc}, {MUL, pa, 0, pc}); break;
    case Op::ps_div: run({DIV, fa, fb, 0}, {DIV, pa, pb, 0}); break;
    case Op::ps_madd: run({MADD, fa, fb, fc}, {MADD, pa, pb, pc}); break;
    case Op::ps_msub: run({MSUB, fa, fb, fc}, {MSUB, pa, pb, pc}); break;
    case Op::ps_nmadd: run({NMADD, fa, fb, fc}, {NMADD, pa, pb, pc}); break;
    case Op::ps_nmsub: run({NMSUB, fa, fb, fc}, {NMSUB, pa, pb, pc}); break;
    case Op::ps_madds0: run({MADD, fa, fb, fc}, {MADD, pa, pb, fc}); break;
    case Op::ps_madds1: run({MADD, fa, fb, pc}, {MADD, pa, pb, pc}); break;
    case Op::ps_muls0: run({MUL, fa, 0, fc}, {MUL, pa, 0, fc}); break;
    case Op::ps_muls1: run({MUL, fa, 0, pc}, {MUL, pa, 0, pc}); break;
    case Op::ps_sum0: run({ADD, fa, pb, 0}, {COPY_RS, 0, pc, 0}); break;
    case Op::ps_sum1: run({COPY_RS, 0, fc, 0}, {ADD, fa, pb, 0}); break;
    case Op::ps_res: run({RES, 0, fb, 0}, {RES, 0, pb, 0}); break;
    case Op::ps_rsqrte: run({RSQRTE, 0, fb, 0}, {RSQRTE, 0, pb, 0}); break;
    case Op::ps_sel: run({SEL, fa, fb, fc}, {SEL, pa, pb, pc}); break;
    case Op::ps_neg: run({NEG, 0, fb, 0}, {NEG, 0, pb, 0}); break;
    case Op::ps_abs: run({ABS, 0, fb, 0}, {ABS, 0, pb, 0}); break;
    case Op::ps_nabs: run({NABS, 0, fb, 0}, {NABS, 0, pb, 0}); break;
    case Op::ps_mr: run({COPY, 0, fb, 0}, {COPY, 0, pb, 0}); break;
    case Op::ps_merge00: run({COPY, 0, fa, 0}, {COPY, 0, fb, 0}); break;
    case Op::ps_merge01: run({COPY, 0, fa, 0}, {COPY, 0, pb, 0}); break;
    case Op::ps_merge10: run({COPY, 0, pa, 0}, {COPY, 0, fb, 0}); break;
    case Op::ps_merge11: run({COPY, 0, pa, 0}, {COPY, 0, pb, 0}); break;
    case Op::ps_cmpu0: case Op::ps_cmpo0: case Op::ps_cmpu1: case Op::ps_cmpo1: {
        bool one = op == Op::ps_cmpu1 || op == Op::ps_cmpo1;
        unsigned x = rf(one ? pa : fa, 28), y = rf(one ? pb : fb, 29);
        a_.fcmp(x, y);
        set_field_from_flags(i.crfd(), a64::MI, a64::GT, a64::EQ, true);
        return;
    }
    default: break;
    }
    if (i.rcbit()) record_fp();
}

void Compiler::emit_cr(Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    switch (op) {
    case Op::mcrf: {
        unsigned n = i.crfd(), s = i.crfs();
        if (n == s) return;
        for (unsigned k = 0; k < 4; ++k) {
            iscr_ = 0;
            set_i(V_CR + n * 4 + k, ri(V_CR + s * 4 + k));
        }
        return;
    }
    case Op::crxor:
        if (a == b && a == d) { unsigned r = wi(V_CR + d); a_.movz(r, 0); ci(V_CR + d, r); return; }
        break;
    case Op::creqv:
        if (a == b && a == d) { unsigned r = wi(V_CR + d); a_.movz(r, 1); ci(V_CR + d, r); return; }
        break;
    case Op::mfcr: {
        for (unsigned n = 0; n < 32; ++n) {
            iscr_ = 0;
            unsigned bit = ri(V_CR + n);
            if (n == 0) a_.lsl_imm(kT0, bit, 31);
            else a_.orr(kT0, kT0, bit, 0, 31 - n);
        }
        iscr_ = 0;
        set_i(V_R + d, kT0);
        return;
    }
    case Op::mtcrf: {
        unsigned s = ri(V_R + d, kT0);
        if (s != kT0) { a_.mov(kT0, s); }
        for (unsigned f = 0; f < 8; ++f) {
            if (!(i.crm() >> (7 - f) & 1)) continue;
            for (unsigned k = 0; k < 4; ++k) {
                iscr_ = 0;
                unsigned n = f * 4 + k;
                unsigned r = wi(V_CR + n);
                a_.ubfx(r, kT0, 31 - n, 1);
                ci(V_CR + n, r);
            }
        }
        return;
    }
    case Op::mcrxr: {
        unsigned n = i.crfd();
        set_i(V_CR + n * 4, ri(V_SO));
        iscr_ = 0;
        set_i(V_CR + n * 4 + 1, ri(V_OV));
        iscr_ = 0;
        set_i(V_CR + n * 4 + 2, ri(V_CA));
        iscr_ = 0;
        unsigned r = wi(V_CR + n * 4 + 3);
        a_.movz(r, 0);
        ci(V_CR + n * 4 + 3, r);
        for (unsigned v : {V_SO, V_OV, V_CA}) {
            iscr_ = 0;
            unsigned z = wi(v);
            a_.movz(z, 0);
            ci(v, z);
        }
        return;
    }
    default: break;
    }
    unsigned x = ri(V_CR + a), y = ri(V_CR + b);
    switch (op) {
    case Op::crand: a_.and_(kT1, x, y); break;
    case Op::crandc: a_.bic(kT1, x, y); break;
    case Op::cror: a_.orr(kT1, x, y); break;
    case Op::crorc: a_.logic_imm(false, 2, kT1, y, 1); a_.orr(kT1, x, kT1); break;
    case Op::crxor: a_.eor(kT1, x, y); break;
    case Op::crnand: a_.and_(kT1, x, y); a_.logic_imm(false, 2, kT1, kT1, 1); break;
    case Op::crnor: a_.orr(kT1, x, y); a_.logic_imm(false, 2, kT1, kT1, 1); break;
    case Op::creqv: a_.eor(kT1, x, y); a_.logic_imm(false, 2, kT1, kT1, 1); break;
    default: break;
    }
    set_i(V_CR + d, kT1);
}

void Compiler::emit_spr(std::uint32_t, Insn i, Op op) {
    unsigned d = i.rd();
    auto store_r1 = [&] {
        unsigned r1 = ri(V_R + 1, kT1);
        a_.strw(r1, xc_, offsetof(gc_ctx, r) + 4);
    };
    switch (op) {
    case Op::mfspr: {
        unsigned spr = i.spr();
        if (spr == 1) {
            a_.ldrb(kT1, xc_, offsetof(gc_ctx, bc));
            a_.orr(kT1, kT1, ri(V_SO), 0, 31);
            a_.orr(kT1, kT1, ri(V_OV), 0, 30);
            a_.orr(kT1, kT1, ri(V_CA), 0, 29);
            iscr_ = 0;
            set_i(V_R + d, kT1);
        } else if (spr == 8 || spr == 9) {
            unsigned s = ri(spr == 8 ? V_LR : V_CTR);
            set_i(V_R + d, s);
        } else if (spr >= 912 && spr <= 919) {
            a_.ldrw(kT1, xc_, offsetof(gc_ctx, gqr) + 4 * (spr - 912));
            set_i(V_R + d, kT1);
        } else {
            VSet keep = keep_set();
            save(keep);
            a_.movx(0, xc_);
            a_.mov32(1, spr);
            call_import(I_MFSPR);
            a_.mov(kT1, 0);
            restore(keep);
            set_i(V_R + d, kT1);
        }
        return;
    }
    case Op::mtspr: {
        unsigned spr = i.spr();
        if (spr == 1) {
            unsigned s = ri(V_R + d, kT0);
            if (s != kT0) a_.mov(kT0, s);
            const unsigned bits[3][2] = {{V_SO, 31}, {V_OV, 30}, {V_CA, 29}};
            for (auto& [v, bit] : bits) {
                iscr_ = 0;
                unsigned r = wi(v);
                a_.ubfx(r, kT0, bit, 1);
                ci(v, r);
            }
            a_.logic_imm(false, 0, kT1, kT0, 0x7F);
            a_.strb(kT1, xc_, offsetof(gc_ctx, bc));
        } else if (spr == 8 || spr == 9) {
            set_i(spr == 8 ? V_LR : V_CTR, ri(V_R + d));
        } else if (spr == 912) {
            // psq with GQR0 assume the float format (the EABI keeps GQR0 zero); anything else
            // stops in the runtime.
            unsigned v = ri(V_R + d);
            int bad = new_label(), cont = new_label();
            a_.tst_imm(v, 0x00070007u);
            jcc(a64::NE, bad);
            a_.strw(v, xc_, offsetof(gc_ctx, gqr));
            bind(cont);
            std::size_t k = k_;
            cold([=] {
                k_ = k;
                bind(bad);
                VSet keep = keep_set();
                save(keep);
                a_.mov(2, v);
                a_.movx(0, xc_);
                a_.movz(1, 912);
                call_import(I_MTSPR);   // stops
                restore(keep);
                jmp(cont);
            });
        } else if (spr >= 913 && spr <= 919) {
            a_.strw(ri(V_R + d), xc_, offsetof(gc_ctx, gqr) + 4 * (spr - 912));
        } else {
            unsigned s = ri(V_R + d);
            a_.mov(kT0, s);
            store_r1();
            VSet keep = keep_set();
            save(keep);
            a_.mov(2, kT0);
            a_.mov32(1, spr);
            a_.movx(0, xc_);
            call_import(I_MTSPR);
            restore(keep);
        }
        return;
    }
    case Op::mftb: {
        VSet keep = keep_set();
        save(keep);
        a_.mov32(0, i.spr());
        call_import(I_MFTB);
        a_.mov(kT1, 0);
        restore(keep);
        set_i(V_R + d, kT1);
        return;
    }
    case Op::mfmsr:
        load_import(kT1, I_MSR);
        a_.ldrw(kT1, kT1, 0);
        set_i(V_R + d, kT1);
        return;
    case Op::mtmsr: {
        unsigned s = ri(V_R + d);
        a_.mov(kT0, s);
        store_r1();
        VSet keep = keep_set();
        save(keep);
        a_.mov(1, kT0);
        a_.movx(0, xc_);
        call_import(I_MTMSR);
        restore(keep);
        return;
    }
    case Op::mfsr:
        load_import(kT1, I_SR);
        a_.ldrw(kT1, kT1, 4 * i.sr());
        set_i(V_R + d, kT1);
        return;
    case Op::mtsr:
        load_import(kT1, I_SR);
        a_.strw(ri(V_R + d), kT1, 4 * i.sr());
        return;
    case Op::mfsrin: case Op::mtsrin: {
        unsigned rb = ri(V_R + i.rb());
        a_.lsr_imm(kT0, rb, 28);
        load_import(kT1, I_SR);
        a_.addsub_reg(true, false, false, kT1, kT1, kT0, 0, 2);
        if (op == Op::mfsrin) {
            a_.ldrw(kT1, kT1, 0);
            set_i(V_R + d, kT1);
        } else {
            a_.strw(ri(V_R + d), kT1, 0);
        }
        return;
    }
    default: return;
    }
}

void Compiler::emit_trap(std::uint32_t pc, Insn i, Op op) {
    unsigned to = i.rd();
    if (!to) return;
    unsigned x = ri(V_R + i.ra());
    if (op == Op::tw) {
        a_.cmp(x, ri(V_R + i.rb()));
    } else {
        std::int32_t v = i.simm();
        if (v >= 0 && v < 4096) a_.cmp_imm(x, static_cast<unsigned>(v));
        else if (v < 0 && -v < 4096) a_.cmn_imm(x, static_cast<unsigned>(-v));
        else { a_.mov32(kT1, static_cast<std::uint32_t>(v)); a_.cmp(x, kT1); }
    }
    path(kExit, P_FLUSH_ALL);
    int trap = new_label(), cont = new_label();
    if (to & 16) jcc(a64::LT, trap);
    if (to & 8) jcc(a64::GT, trap);
    if (to & 4) jcc(a64::EQ, trap);
    if (to & 2) jcc(a64::LO, trap);
    if (to & 1) jcc(a64::HI, trap);
    bind(cont);
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(trap);
        flush(flush_here());
        VSet keep = keep_set();
        save(keep);
        a_.movx(0, xc_);
        a_.mov32(1, pc);
        call_import(I_TRAP);
        restore(keep);
        jmp(cont);
    });
}

void Compiler::decrement_ctr() {
    unsigned c = ri(V_CTR);
    unsigned r = wi(V_CTR);
    a_.sub_imm(r, c, 1);
    ci(V_CTR, r);
}

void Compiler::branch_if(bool when_taken, unsigned bo, unsigned bi, bool test_ctr, int label) {
    test_ctr = test_ctr && !(bo & 4);
    bool test_cr = !(bo & 16);
    auto ctr_test = [&](bool goto_if_ok, int l) {
        unsigned c = ri(V_CTR);
        bool ok_when_zero = bo & 2;
        jz(c, l, ok_when_zero != goto_if_ok);
    };
    auto cr_test = [&](bool goto_if_ok, int l) {
        bool branch_when_set = ((bo & 8) != 0) == goto_if_ok;
        Cond c;
        if (fused(bi, c)) {
            jcc(branch_when_set ? c : a64::invert(c), l);
            return;
        }
        unsigned r = ri(V_CR + bi);
        jz(r, l, branch_when_set);
    };
    if (when_taken) {
        if (test_ctr && test_cr) {
            int skip = new_label();
            ctr_test(false, skip);
            cr_test(true, label);
            bind(skip);
        } else if (test_ctr) {
            ctr_test(true, label);
        } else if (test_cr) {
            cr_test(true, label);
        } else {
            jmp(label);
        }
    } else {
        if (test_ctr) ctr_test(false, label);
        if (test_cr) cr_test(false, label);
    }
}

// A loop that only rereads memory through unchanged address registers (emit.cpp).
bool Compiler::is_spin_loop(std::uint32_t target, std::uint32_t branch) const {
    if (branch - target > 12 * 4) return false;
    std::uint32_t loaded = 0, addressing = 0;
    for (std::uint32_t a = target; a <= branch; a += 4) {
        Insn i{p_.image->read32(a)};
        Op op = gcr::decode(i);
        switch (op) {
        case Op::lwz: case Op::lhz: case Op::lha: case Op::lbz:
            if (i.ra()) addressing |= 1u << i.ra();
            loaded |= 1u << i.rd();
            break;
        case Op::cmpi: case Op::cmpli: case Op::cmp: case Op::cmpl:
            break;
        case Op::andi_rc: case Op::andis_rc:
            loaded |= 1u << i.ra();
            break;
        case Op::rlwinm:
            loaded |= 1u << i.ra();
            break;
        case Op::bc:
            if (!(i.bo() & 4)) return false;
            break;
        case Op::b:
            if (i.lk()) return false;
            break;
        default:
            return false;
        }
    }
    return (loaded & addressing) == 0;
}

void Compiler::backedge(std::uint32_t pc, std::uint32_t target) {
    note_use_i(V_R + 1);
    if (is_spin_loop(target, pc)) {
        if (!analyzing_) ++out_.spin_loops;
        VSet keep = keep_set();
        save(keep);
        unsigned r1 = ri(V_R + 1, kT1);
        a_.strw(r1, xc_, offsetof(gc_ctx, r) + 4);
        a_.movx(0, xc_);
        call_import(I_SPIN);
        restore(keep);
        if (!analyzing_) jmp(guest_label(target));
        return;
    }
    int slow = new_label();
    if (opt_.virtual_time) {
        load_import(kT0, I_STEPS);
        a_.ldrx(kT1, kT0, 0);
        a_.add_imm(kT1, kT1, 1, true);
        a_.strx(kT1, kT0, 0);
        load_import(kT0, I_DEADLINE);
        a_.ldrx(kT0, kT0, 0);
        a_.cmp(kT1, kT0, true);
        jcc(a64::HS, slow);
    }
    // The running context's copy of gc_attn: one load from a line the code already uses. The
    // loop continues on zero; otherwise the safepoint runs out of line first.
    a_.ldrw(kT0, xc_, offsetof(gc_ctx, attn));
    if (!analyzing_) jz(kT0, guest_label(target), false);
    jmp(slow);
    std::size_t k = k_;
    cold([=] {
        k_ = k;
        bind(slow);
        VSet keep = keep_set();
        save(keep);
        unsigned r1 = ri(V_R + 1, kT1);
        a_.strw(r1, xc_, offsetof(gc_ctx, r) + 4);
        a_.movx(0, xc_);
        call_import(I_SAFEPOINT);
        restore(keep);
        if (!analyzing_) jmp(guest_label(target));
    });
}

void Compiler::guest_call(std::uint32_t pc, std::uint32_t target) {
    VSet lr;
    lr.add_i(V_LR);
    if (p_.save_context.count(target)) {
        // Returns once now and again when the saved context is loaded (gcrt_setjmp).
        VSet r3;
        r3.add_i(V_R + 3);
        path(kFall, P_FLUSH_ALL | P_RELOAD_ALL | P_LR_IMM, r3, lr | r3);
        if (analyzing_) { ri(V_R + 3); return; }
        flush(flush_here() | lr, true, pc + 4);
        home_to_reg_i(V_R + 3, 1);
        a_.movx(0, xc_);
        call_import(I_RESUME_SLOT);
        call_import(I_SETJMP);
        int resumed = new_label(), done = new_label();
        jz(0, resumed, true);
        a_.movx(0, xc_);
        if (p_.function_at(target)) {
            out_.external_calls.push_back({static_cast<std::uint32_t>(a_.pos()), target});
            a_.bl(0);
        } else {
            a_.mov32(1, target);
            call_import(I_CALL);
        }
        VSet after = info_[k_ + 1].live_in;
        reload(after);
        jmp(done);
        bind(resumed);
        reload(after);
        if (after.has_i(V_R + 3)) {
            a_.movz(kT1, 1);
            reg_to_home_i(V_R + 3, kT1);
        }
        bind(done);
        return;
    }
    int callee = callee_at(target);
    path(kFall, P_FLUSH | P_RELOAD | P_LR_IMM | P_ABI, {}, lr, callee);
    if (analyzing_) return;
    VSet after = info_[k_ + 1].live_in;
    flush((flush_here() | (info_[k_].flush_live & after)) - lr, false);
    a_.mov32(kT1, pc + 4);
    a_.strw(kT1, xc_, offsetof(gc_ctx, lr));
    if (callee >= 0) {
        const Summary& s = sums_[static_cast<std::size_t>(callee)];
        arg_moves(s.in, true);
        out_.calls.push_back({static_cast<std::uint32_t>(a_.pos()), target});
        a_.bl(0);
        const VSet stored = s.out - demand_[k_ + 1];   // results this function does not use
        for (unsigned v = 0; v < NI; ++v)
            if (stored.has_i(v)) a_.strw(static_cast<unsigned>(arg_reg_i(v)), xc_, ctx_off_i(v));
        for (unsigned v = 0; v < NF; ++v)
            if (stored.has_f(v)) a_.strd(static_cast<unsigned>(arg_reg_f(v)), xc_, ctx_off_f(v));
        arg_moves(s.out & after, false);
        reload((abi_ & after) - s.out);
    } else {
        a_.movx(0, xc_);
        a_.mov32(1, target);
        call_import(I_CALL);
        reload(abi_ & after);
    }
}

// A tail branch whose target returns other registers than this function becomes a call
// followed by this function's return.
void Compiler::guest_tail(std::uint32_t target) {
    if (analyzing_) return;
    int callee = callee_at(target);
    flush(flush_here());
    if (callee >= 0) {
        const Summary& s = sums_[static_cast<std::size_t>(callee)];
        arg_moves(s.in, true);
        if (s.out == out_regs_) {
            epilogue();
            out_.calls.push_back({static_cast<std::uint32_t>(a_.pos()), target});
            a_.b(0);
            return;
        }
        if (leaf_) a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -16);
        out_.calls.push_back({static_cast<std::uint32_t>(a_.pos()), target});
        a_.bl(0);
        return_after_call(s.out);
        return;
    }
    a_.movx(0, xc_);
    a_.mov32(1, target);
    if (out_regs_ == VSet{}) {
        epilogue();
        load_import(kT0, I_CALL);
        a_.br(kT0);
        return;
    }
    if (leaf_) a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -16);
    call_import(I_CALL);
    return_after_call({});
}

void Compiler::guest_return() {
    if (analyzing_) return;
    flush(flush_here());
    arg_moves(out_regs_, true);
    epilogue();
    a_.ret();
}

void Compiler::indirect_call(std::uint32_t pc, unsigned target_value) {
    if (analyzing_) return;
    VSet lr;
    lr.add_i(V_LR);
    a_.mov(kT0, target_value);
    flush(flush_here() - lr, false);
    a_.mov32(kT1, pc + 4);
    a_.strw(kT1, xc_, offsetof(gc_ctx, lr));
    a_.mov(1, kT0);
    a_.movx(0, xc_);
    call_import(I_CALL);
    reload(abi_ & info_[k_ + 1].live_in);
}

void Compiler::emit_switch(std::uint32_t pc) {
    auto lo_it = std::lower_bound(p_.code_pointers.begin(), p_.code_pointers.end(), f_.start);
    std::vector<std::uint32_t> cases;
    for (auto it = lo_it; it != p_.code_pointers.end() && *it < f_.end; ++it)
        if (*it != f_.start) cases.push_back(*it);
    VSet ctr;
    ctr.add_i(V_CTR);
    for (auto c : cases) path(static_cast<int>((c - f_.start) / 4), 0);
    path(kExit, P_FLUSH, ctr);
    unsigned tv = ri(V_CTR);
    if (analyzing_) return;
    if (!cases.empty()) {
        std::uint32_t lo = cases.front(), hi = cases.back();
        std::uint32_t count = (hi - lo) / 4 + 1;
        if (cases.size() <= 4 || count > 4 * cases.size() + 64) {
            for (auto c : cases) {
                a_.mov32(kT1, c);
                a_.cmp(tv, kT1);
                jcc(a64::EQ, guest_label(c));
            }
        } else {
            int table = new_label(), fallback = new_label();
            a_.mov32(kT1, lo);
            a_.sub(kT0, tv, kT1);
            a_.ror_imm(kT0, kT0, 2);
            a_.mov32(kT1, count);
            a_.cmp(kT0, kT1);
            jcc(a64::HS, fallback);
            fix(table, F_ADR);
            a_.adr(kT1, 0);
            a_.ldrsw_lsl2(kT0, kT1, kT0);
            a_.addsub_reg(true, false, false, kT1, kT1, kT0);
            a_.br(kT1);
            bind(fallback);
            std::set<std::uint32_t> present(cases.begin(), cases.end());
            Table t;
            t.label = table;
            t.fallback = fallback;
            for (std::uint32_t n = 0; n < count; ++n) {
                std::uint32_t a = lo + 4 * n;
                t.entries.push_back(present.count(a) ? guest_label(a) : fallback);
            }
            tables_.push_back(t);
        }
    }
    a_.mov(kT0, tv);
    flush(flush_here());
    a_.mov(1, kT0);
    a_.mov32(2, pc);
    a_.movx(0, xc_);
    if (out_regs_ == VSet{}) {
        epilogue();
        load_import(kT0, I_JUMP);
        a_.br(kT0);
        return;
    }
    if (leaf_) a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -16);
    call_import(I_JUMP);
    return_after_call({});
}

void Compiler::emit_branch(std::uint32_t pc, Insn i, Op op) {
    auto inside = [&](std::uint32_t t) { return t >= f_.start && t < f_.end; };
    auto index = [&](std::uint32_t t) { return static_cast<int>((t - f_.start) / 4); };
    VSet r1;
    r1.add_i(V_R + 1);
    if (op == Op::b) {
        std::uint32_t target = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
        if (i.lk()) {
            if (target == pc + 4) {
                unsigned r = wi(V_LR);
                a_.mov32(r, pc + 4);
                ci(V_LR, r);
            } else {
                if (analyzing_) cur().falls = false;
                guest_call(pc, target);
            }
            return;
        }
        if (analyzing_) cur().falls = false;
        if (inside(target)) {
            path(index(target), 0, target <= pc ? r1 : VSet{});
            if (target <= pc) backedge(pc, target);   // ends with the branch to target
            else if (!analyzing_) jmp(guest_label(target));
            return;
        }
        path(kExit, P_FLUSH, {}, {}, callee_at(target));
        guest_tail(target);
        return;
    }
    unsigned bo = i.bo(), bi = i.bi();
    bool uncond = (bo & 0x14) == 0x14;
    if (analyzing_ && uncond) cur().falls = false;
    if (op == Op::bc) {
        std::uint32_t target = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
        if (!(bo & 4)) decrement_ctr();
        if (i.lk()) {
            int skip = new_label();
            branch_if(false, bo, bi, true, skip);
            guest_call(pc, target);
            bind(skip);
        } else if (inside(target)) {
            path(index(target), 0, target <= pc ? r1 : VSet{});
            if (target <= pc) {
                int skip = new_label();
                branch_if(false, bo, bi, true, skip);
                backedge(pc, target);
                bind(skip);
            } else if (!analyzing_) {
                branch_if(true, bo, bi, true, guest_label(target));
            } else {
                branch_if(true, bo, bi, true, new_label());
            }
        } else {
            path(kExit, P_FLUSH, {}, {}, callee_at(target));
            int skip = new_label();
            branch_if(false, bo, bi, true, skip);
            guest_tail(target);
            bind(skip);
        }
        return;
    }
    if (op == Op::bclr) {
        if (!(bo & 4)) decrement_ctr();
        int skip = new_label();
        if (i.lk()) {
            VSet lr;
            lr.add_i(V_LR);
            path(kFall, P_FLUSH | P_RELOAD | P_LR_IMM | P_ABI, lr, lr);
            branch_if(false, bo, bi, true, skip);
            unsigned t = ri(V_LR);
            indirect_call(pc, t);
        } else {
            path(kExit, P_FLUSH | P_RET);
            branch_if(false, bo, bi, true, skip);
            guest_return();
        }
        bind(skip);
        return;
    }
    if (op == Op::bcctr) {
        int skip = new_label();
        if (i.lk()) {
            VSet ctr, lr;
            ctr.add_i(V_CTR);
            lr.add_i(V_LR);
            path(kFall, P_FLUSH | P_RELOAD | P_LR_IMM | P_ABI, ctr, lr);
            branch_if(false, bo, bi, false, skip);
            unsigned t = ri(V_CTR);
            indirect_call(pc, t);
        } else {
            branch_if(false, bo, bi, false, skip);
            emit_switch(pc);
        }
        bind(skip);
    }
}

void Compiler::emit_insn(std::uint32_t pc, Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    switch (op) {
    case Op::invalid:
        if (!analyzing_) ++out_.invalid;
        emit_runtime_call(I_ILLEGAL, pc, i.w);
        return;
    case Op::addi:
        if (!a) { unsigned r = wi(V_R + d); a_.mov32(r, static_cast<std::uint32_t>(i.simm())); ci(V_R + d, r); }
        else { unsigned x = ri(V_R + a); unsigned r = wi(V_R + d); add_imm32(r, x, i.simm()); ci(V_R + d, r); }
        return;
    case Op::addis: {
        std::uint32_t v = static_cast<std::uint32_t>(i.simm()) << 16;
        if (!a) { unsigned r = wi(V_R + d); a_.mov32(r, v); ci(V_R + d, r); }
        else { unsigned x = ri(V_R + a); unsigned r = wi(V_R + d); add_imm32(r, x, static_cast<std::int32_t>(v)); ci(V_R + d, r); }
        return;
    }
    case Op::addic: case Op::addic_rc: {
        std::int32_t v = i.simm();
        unsigned x = ri(V_R + a);
        unsigned r = wi(V_R + d);
        if (v >= 0 && v < 4096) a_.addsub_imm(false, false, true, r, x, static_cast<unsigned>(v));
        else if (v < 0 && -v < 4096) a_.addsub_imm(false, true, true, r, x, static_cast<unsigned>(-v));
        else { a_.mov32(kT1, static_cast<std::uint32_t>(v)); a_.adds(r, x, kT1); }
        unsigned c = wi(V_CA);
        a_.cset(c, a64::HS);
        ci(V_CA, c);
        ci(V_R + d, r);
        if (op == Op::addic_rc) { iscr_ = 0; record(ri(V_R + d)); }
        return;
    }
    case Op::subfic: {
        unsigned x = ri(V_R + a);
        a_.mov32(kT1, static_cast<std::uint32_t>(i.simm()));
        unsigned r = wi(V_R + d);
        a_.subs(r, kT1, x);
        unsigned c = wi(V_CA);
        a_.cset(c, a64::HS);
        ci(V_CA, c);
        ci(V_R + d, r);
        return;
    }
    case Op::mulli: {
        unsigned x = ri(V_R + a);
        a_.mov32(kT1, static_cast<std::uint32_t>(i.simm()));
        unsigned r = wi(V_R + d);
        a_.mul(r, x, kT1);
        ci(V_R + d, r);
        return;
    }
    case Op::cmpi: {
        unsigned x = ri(V_R + a);
        std::int32_t v = i.simm();
        if (v >= 0 && v < 4096) a_.cmp_imm(x, static_cast<unsigned>(v));
        else if (v < 0 && -v < 4096) a_.cmn_imm(x, static_cast<unsigned>(-v));
        else { a_.mov32(kT1, static_cast<std::uint32_t>(v)); a_.cmp(x, kT1); }
        set_field_from_flags(i.crfd(), a64::LT, a64::GT, a64::EQ, false);
        return;
    }
    case Op::cmpli: {
        unsigned x = ri(V_R + a);
        std::uint32_t v = i.uimm();
        if (v < 4096) a_.cmp_imm(x, v);
        else { a_.mov32(kT1, v); a_.cmp(x, kT1); }
        set_field_from_flags(i.crfd(), a64::LO, a64::HI, a64::EQ, false);
        return;
    }
    case Op::cmp: case Op::cmpl: {
        unsigned x = ri(V_R + a), y = ri(V_R + b);
        a_.cmp(x, y);
        if (op == Op::cmp) set_field_from_flags(i.crfd(), a64::LT, a64::GT, a64::EQ, false);
        else set_field_from_flags(i.crfd(), a64::LO, a64::HI, a64::EQ, false);
        return;
    }
    case Op::andi_rc: case Op::andis_rc: {
        unsigned x = ri(V_R + d);
        unsigned r = wi(V_R + a);
        std::uint32_t m = op == Op::andi_rc ? i.uimm() : i.uimm() << 16;
        if (m) logic_imm32(0, r, x, m);
        else a_.movz(r, 0);
        ci(V_R + a, r);
        iscr_ = 0;
        record(ri(V_R + a));
        return;
    }
    case Op::ori: case Op::oris: case Op::xori: case Op::xoris: {
        if (op == Op::ori && !d && !a && !i.uimm()) return;
        std::uint32_t m = (op == Op::ori || op == Op::xori) ? i.uimm() : i.uimm() << 16;
        unsigned x = ri(V_R + d);
        unsigned r = wi(V_R + a);
        if (m) logic_imm32((op == Op::ori || op == Op::oris) ? 1 : 2, r, x, m);
        else if (r != x) a_.mov(r, x);
        ci(V_R + a, r);
        return;
    }
    case Op::rlwinm: case Op::rlwimi: case Op::rlwnm:
        emit_rotate(i, op);
        return;
    case Op::and_: case Op::andc: case Op::or_: case Op::orc: case Op::xor_: case Op::nand: case Op::nor: case Op::eqv: {
        unsigned x = ri(V_R + d), y = ri(V_R + b);
        unsigned r = wi(V_R + a);
        switch (op) {
        case Op::and_: a_.and_(r, x, y); break;
        case Op::andc: a_.bic(r, x, y); break;
        case Op::or_: if (x == y) { if (r != x) a_.mov(r, x); } else a_.orr(r, x, y); break;
        case Op::orc: a_.orn(r, x, y); break;
        case Op::xor_: a_.eor(r, x, y); break;
        case Op::nand: a_.and_(r, x, y); a_.mvn(r, r); break;
        case Op::nor: a_.orr(r, x, y); a_.mvn(r, r); break;
        default: a_.eon(r, x, y); break;
        }
        ci(V_R + a, r);
        if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
        return;
    }
    case Op::extsb: case Op::extsh: case Op::cntlzw: {
        unsigned x = ri(V_R + d);
        unsigned r = wi(V_R + a);
        if (op == Op::extsb) a_.sxtb(r, x);
        else if (op == Op::extsh) a_.sxth(r, x);
        else a_.clz(r, x);
        ci(V_R + a, r);
        if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
        return;
    }
    case Op::slw: case Op::srw: {
        unsigned x = ri(V_R + d), y = ri(V_R + b);
        if (op == Op::slw) {
            a_.lslv(kT1, x, y, true);
            set_i(V_R + a, kT1);
        } else {
            unsigned r = wi(V_R + a);
            a_.lsrv(r, x, y, true);
            ci(V_R + a, r);
        }
        if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
        return;
    }
    case Op::srawi: {
        unsigned sh = i.sh();
        unsigned x = ri(V_R + d);
        if (sh == 0) {
            set_i(V_R + a, x);
            unsigned c = wi(V_CA);
            a_.movz(c, 0);
            ci(V_CA, c);
        } else {
            a_.tst_imm(x, (1u << sh) - 1);
            a_.cset(kT1, a64::NE);
            a_.logic_reg(false, 0, false, kT1, kT1, x, 1, 31);   // & (x >> 31)
            unsigned c = wi(V_CA);
            a_.mov(c, kT1);
            ci(V_CA, c);
            unsigned r = wi(V_R + a);
            a_.asr_imm(r, x, sh);
            ci(V_R + a, r);
        }
        if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
        return;
    }
    case Op::sraw: {
        unsigned x = ri(V_R + d), y = ri(V_R + b);
        a_.sxtw(kT0, x);
        a_.asrv(kT1, kT0, y, true);
        a_.lslv(15, kT1, y, true);
        a_.cmp(15, kT0, true);
        a_.cset(15, a64::NE);
        a_.logic_reg(false, 0, false, 15, 15, x, 1, 31);
        unsigned c = wi(V_CA);
        a_.mov(c, 15);
        ci(V_CA, c);
        set_i(V_R + a, kT1);
        if (i.rcbit()) { iscr_ = 0; record(ri(V_R + a)); }
        return;
    }
    case Op::add: case Op::addc: case Op::adde: case Op::addze: case Op::addme: case Op::subf: case Op::subfc:
    case Op::subfe: case Op::subfze: case Op::subfme: case Op::neg: case Op::mullw: case Op::mulhw: case Op::mulhwu:
    case Op::divw: case Op::divwu:
        emit_arith(i, op);
        return;
    case Op::lwz: case Op::lwzu: case Op::lwzx: case Op::lwzux: case Op::lbz: case Op::lbzu: case Op::lbzx: case Op::lbzux:
    case Op::lhz: case Op::lhzu: case Op::lhzx: case Op::lhzux: case Op::lha: case Op::lhau: case Op::lhax: case Op::lhaux:
    case Op::lwbrx: case Op::lhbrx: case Op::lwarx:
        emit_load(i, op);
        return;
    case Op::stw: case Op::stwu: case Op::stwx: case Op::stwux: case Op::stb: case Op::stbu: case Op::stbx: case Op::stbux:
    case Op::sth: case Op::sthu: case Op::sthx: case Op::sthux: case Op::stwbrx: case Op::sthbrx: case Op::stwcx:
        emit_store(i, op);
        return;
    case Op::lmw: emit_lmw(pc, i, true); return;
    case Op::stmw: emit_lmw(pc, i, false); return;
    case Op::lswi: emit_string(i, true); return;
    case Op::stswi: emit_string(i, false); return;
    case Op::lfs: case Op::lfsu: case Op::lfsx: case Op::lfsux: case Op::lfd: case Op::lfdu: case Op::lfdx: case Op::lfdux:
        emit_fload(i, op);
        return;
    case Op::stfs: case Op::stfsu: case Op::stfsx: case Op::stfsux: case Op::stfd: case Op::stfdu: case Op::stfdx:
    case Op::stfdux: case Op::stfiwx:
        emit_fstore(i, op);
        return;
    case Op::psq_l: case Op::psq_lu: case Op::psq_lx: case Op::psq_lux:
    case Op::psq_st: case Op::psq_stu: case Op::psq_stx: case Op::psq_stux:
        emit_psq(i, op);
        return;
    case Op::b: case Op::bc: case Op::bclr: case Op::bcctr:
        emit_branch(pc, i, op);
        return;
    case Op::mcrf: case Op::crand: case Op::crandc: case Op::cror: case Op::crorc: case Op::crxor: case Op::crnand:
    case Op::crnor: case Op::creqv: case Op::mfcr: case Op::mtcrf: case Op::mcrxr:
        emit_cr(i, op);
        return;
    case Op::mfspr: case Op::mtspr: case Op::mftb: case Op::mfmsr: case Op::mtmsr: case Op::mfsr: case Op::mtsr:
    case Op::mfsrin: case Op::mtsrin:
        emit_spr(pc, i, op);
        return;
    case Op::sync: case Op::isync: case Op::eieio: case Op::dcbst: case Op::dcbf: case Op::dcbt: case Op::dcbtst:
    case Op::dcbi: case Op::icbi: case Op::tlbie: case Op::tlbsync:
        return;
    case Op::dcbz: case Op::dcbz_l: {
        unsigned ea = ea_x(a, b, false);
        int skip = new_label();
        a_.logic_imm(false, 0, kT0, ea, ~31u);
        io_check(kT0, skip);
        a_.add_uxtw(kT0, xm_, kT0);
        a_.ldstp(3, false, false, 2, a64::ZR, a64::ZR, kT0, 0);
        a_.ldstp(3, false, false, 2, a64::ZR, a64::ZR, kT0, 16);
        bind(skip);
        return;
    }
    case Op::sc: {
        VSet keep = keep_set();
        save(keep);
        a_.movx(0, xc_);
        call_import(I_SC);
        restore(keep);
        return;
    }
    case Op::rfi:
        if (analyzing_) { cur().falls = false; path(kExit, P_FLUSH_ALL); return; }
        flush(flush_here());
        a_.movx(0, xc_);
        a_.mov32(1, pc);
        epilogue();
        load_import(kT0, I_RFI);
        a_.br(kT0);
        return;
    case Op::tw: case Op::twi:
        emit_trap(pc, i, op);
        return;
    case Op::eciwx: case Op::ecowx: case Op::lswx: case Op::stswx:
        if (!analyzing_) ++out_.invalid;
        emit_runtime_call(I_ILLEGAL, pc, i.w);
        return;
    default:
        break;
    }
    if (op >= Op::fdivs && op <= Op::mtfsf) {
        emit_fp(pc, i, op);
        return;
    }
    if (op >= Op::ps_sum0 && op <= Op::ps_merge11) {
        emit_ps(i, op);
        return;
    }
    if (!analyzing_) ++out_.invalid;
    emit_runtime_call(I_ILLEGAL, pc, i.w);
}

void Compiler::track(std::size_t k) {
    Insn i = insns_[k];
    Op op = ops_[k];
    bool have = false;
    unsigned dst = 0;
    std::uint32_t value = 0, x = 0;
    switch (op) {
    case Op::addi:
    case Op::addis:
        dst = i.rd();
        if (constant(i.ra(), x)) {
            have = true;
            value = x + (op == Op::addi ? static_cast<std::uint32_t>(i.simm()) : static_cast<std::uint32_t>(i.simm()) << 16);
        }
        break;
    case Op::ori: case Op::oris: case Op::xori: case Op::xoris:
        dst = i.ra();
        if (i.rs() == 0 ? false : known_[i.rs()].known) {
            have = true;
            x = known_[i.rs()].value;
            std::uint32_t imm = (op == Op::ori || op == Op::xori) ? i.uimm() : i.uimm() << 16;
            value = (op == Op::ori || op == Op::oris) ? (x | imm) : (x ^ imm);
        }
        break;
    case Op::rlwinm:
        dst = i.ra();
        if (i.rs() != 0 && known_[i.rs()].known) {
            have = true;
            value = GC_ROTL(known_[i.rs()].value, i.sh()) & gcr::rotate_mask(i.mb(), i.me());
        }
        break;
    case Op::or_:
        dst = i.ra();
        if (i.rs() == i.rb() && i.rs() != 0 && known_[i.rs()].known) {
            have = true;
            value = known_[i.rs()].value;
        }
        break;
    default:
        break;
    }
    const Info& in = info_[k];
    if (!in.paths.empty()) forget_all();
    for (unsigned r = 1; r < 32; ++r)
        if (in.defs.has_i(V_R + r)) known_[r].known = false;
    if (have && dst != 0) known_[dst] = {true, value};
}

bool Compiler::keeps_pending(Op op, Insn i) const {
    switch (op) {
    case Op::lwz: case Op::lwzu: case Op::lwzx: case Op::lwzux: case Op::lbz: case Op::lbzu: case Op::lbzx: case Op::lbzux:
    case Op::lhz: case Op::lhzu: case Op::lhzx: case Op::lhzux: case Op::lha: case Op::lhau: case Op::lhax: case Op::lhaux:
    case Op::lwbrx: case Op::lhbrx: case Op::lwarx:
    case Op::stw: case Op::stwu: case Op::stwx: case Op::stwux: case Op::stb: case Op::stbu: case Op::stbx: case Op::stbux:
    case Op::sth: case Op::sthu: case Op::sthx: case Op::sthux: case Op::stwbrx: case Op::sthbrx: case Op::stwcx:
    case Op::lfs: case Op::lfsu: case Op::lfsx: case Op::lfsux: case Op::lfd: case Op::lfdu: case Op::lfdx: case Op::lfdux:
    case Op::stfs: case Op::stfsu: case Op::stfsx: case Op::stfsux: case Op::stfd: case Op::stfdu: case Op::stfdx:
    case Op::stfdux: case Op::stfiwx:
    case Op::psq_l: case Op::psq_lu: case Op::psq_lx: case Op::psq_lux:
    case Op::psq_st: case Op::psq_stu: case Op::psq_stx: case Op::psq_stux:
        return true;
    default:
        return pure(op, i);
    }
}

bool Compiler::pure(Op op, Insn i) const {
    switch (op) {
    case Op::addi: case Op::addis: case Op::addic: case Op::addic_rc: case Op::subfic: case Op::mulli:
    case Op::cmpi: case Op::cmpli: case Op::cmp: case Op::cmpl: case Op::andi_rc: case Op::andis_rc:
    case Op::ori: case Op::oris: case Op::xori: case Op::xoris: case Op::rlwinm: case Op::rlwimi: case Op::rlwnm:
    case Op::and_: case Op::andc: case Op::or_: case Op::orc: case Op::xor_: case Op::nand: case Op::nor: case Op::eqv:
    case Op::extsb: case Op::extsh: case Op::cntlzw: case Op::slw: case Op::srw: case Op::sraw: case Op::srawi:
    case Op::add: case Op::addc: case Op::adde: case Op::addze: case Op::addme: case Op::subf: case Op::subfc:
    case Op::subfe: case Op::subfze: case Op::subfme: case Op::neg: case Op::mullw: case Op::mulhw: case Op::mulhwu:
    case Op::divw: case Op::divwu:
    case Op::mcrf: case Op::crand: case Op::crandc: case Op::cror: case Op::crorc: case Op::crxor: case Op::crnand:
    case Op::crnor: case Op::creqv: case Op::mfcr: case Op::mtcrf:
    case Op::fdivs: case Op::fsubs: case Op::fadds: case Op::fres: case Op::fmuls: case Op::fmsubs: case Op::fmadds:
    case Op::fnmsubs: case Op::fnmadds: case Op::fdiv: case Op::fsub: case Op::fadd: case Op::fsel: case Op::fmul:
    case Op::frsqrte: case Op::fmsub: case Op::fmadd: case Op::fnmsub: case Op::fnmadd: case Op::fcmpu: case Op::frsp:
    case Op::fctiwz: case Op::fcmpo: case Op::fneg: case Op::fmr: case Op::fnabs: case Op::fabs: case Op::mffs:
    case Op::ps_sum0: case Op::ps_sum1: case Op::ps_muls0: case Op::ps_muls1: case Op::ps_madds0: case Op::ps_madds1:
    case Op::ps_div: case Op::ps_sub: case Op::ps_add: case Op::ps_sel: case Op::ps_res: case Op::ps_mul:
    case Op::ps_rsqrte: case Op::ps_msub: case Op::ps_madd: case Op::ps_nmsub: case Op::ps_nmadd: case Op::ps_cmpu0:
    case Op::ps_cmpo0: case Op::ps_neg: case Op::ps_cmpu1: case Op::ps_mr: case Op::ps_cmpo1: case Op::ps_nabs:
    case Op::ps_abs: case Op::ps_merge00: case Op::ps_merge01: case Op::ps_merge10: case Op::ps_merge11:
        return true;
    case Op::mfspr:
        return i.spr() == 1 || i.spr() == 8 || i.spr() == 9 || (i.spr() >= 912 && i.spr() <= 919);
    case Op::mtspr:
        return i.spr() == 8 || i.spr() == 9;
    // Loads through the stack and small-data bases never reach hardware registers.
    case Op::lwz: case Op::lhz: case Op::lha: case Op::lbz: case Op::lfs: case Op::lfd:
        return i.ra() == 1 || i.ra() == 2 || i.ra() == 13;
    default:
        return false;
    }
}

void Compiler::generate() {
    a_.code.clear();
    label_pos_.clear();
    fixes_.clear();
    literals_.clear();
    tables_.clear();
    cold_.clear();
    labels_.clear();
    for (auto pc : label_pcs_) labels_[pc] = new_label();
    // External entry: keeps the host caller's x27/x28 and calls the internal entry below, with
    // the passed registers loaded from the context and the returned ones stored back.
    a_.ldstp(3, false, false, 3, a64::FP, a64::LR, a64::SP, -32);
    a_.add_imm(a64::FP, a64::SP, 0, true);
    a_.ldstp(3, false, false, 2, kXC, kXM, a64::SP, 16);
    a_.movx(kXC, 0);
    a_.ldrx(kXM, kXC, offsetof(gc_ctx, mem));
    for (unsigned v = 0; v < NI; ++v)
        if (in_.has_i(v)) a_.ldrw(static_cast<unsigned>(arg_reg_i(v)), kXC, ctx_off_i(v));
    for (unsigned v = 0; v < NF; ++v)
        if (in_.has_f(v)) a_.ldrd(static_cast<unsigned>(arg_reg_f(v)), kXC, ctx_off_f(v));
    int body = new_label();
    fix(body, F_IMM26);
    a_.bl(0);
    for (unsigned v = 0; v < NI; ++v)
        if (out_regs_.has_i(v)) a_.strw(static_cast<unsigned>(arg_reg_i(v)), kXC, ctx_off_i(v));
    for (unsigned v = 0; v < NF; ++v)
        if (out_regs_.has_f(v)) a_.strd(static_cast<unsigned>(arg_reg_f(v)), kXC, ctx_off_f(v));
    a_.ldstp(3, true, false, 2, kXC, kXM, a64::SP, 16);
    a_.ldstp(3, true, false, 1, a64::FP, a64::LR, a64::SP, 32);
    a_.ret();
    bind(body);
    out_.entry = static_cast<std::uint32_t>(a_.pos());
    if (!f_.hle.empty()) {
        a_.movx(0, kXC);
        load_import(kT0, "hle_" + f_.hle);
        a_.br(kT0);
        return;
    }
    if (!f_.wrap.empty()) {
        fail("wrap functions are not supported by the translated build");
        int body = new_label();
        fix(body, F_ADR);
        a_.adr(1, 0);
        load_import(kT0, "wrap_" + f_.wrap);
        a_.br(kT0);
        bind(body);
    }
    prologue();
    std::memset(pend_, 0, sizeof pend_);
    arg_moves(in_ & info_[0].live_in, false);
    reload(info_[0].live_in - in_);
    flags_.valid = false;
    for (std::size_t k = 0; k < n_; ++k) {
        k_ = k;
        iscr_ = fscr_ = 0;
        std::uint32_t pc = f_.start + 4 * static_cast<std::uint32_t>(k);
        if (label_pcs_.count(pc)) {
            resolve_all();
            bind(labels_[pc]);
            forget_all();
        }
        // Pure instructions whose results are all dead are dropped.
        const Info& in = info_[k];
        bool dead = pure(ops_[k], insns_[k]) && (in.defs & in.after) == VSet{};
        if (!dead && !keeps_pending(ops_[k], insns_[k])) resolve_all();
        if (!dead) emit_insn(pcs_[k], insns_[k], ops_[k]);
        track(k);
    }
    if (!ended_) {
        k_ = n_;
        guest_tail(f_.end);
    }
}

bool Compiler::finish() {
    for (std::size_t c = 0; c < cold_.size(); ++c) {
        iscr_ = fscr_ = 0;
        auto fn = cold_[c];
        fn();
    }
    schedule_code();
    for (auto& t : tables_) {
        bind(t.label);
        std::size_t base = a_.pos();
        for (int e : t.entries) {
            fixes_.push_back({a_.pos(), e, F_WORD, base});
            a_.emit(0);
        }
    }
    if (a_.pos() & 1) a_.emit(0xD503201Fu);   // nop: 8-byte aligned literals
    for (auto& [name, l] : literals_) {
        bind(l);
        out_.relocs.push_back({static_cast<std::uint32_t>(a_.pos() * 4), name});
        a_.emit(0);
        a_.emit(0);
    }
    for (auto& fx : fixes_) {
        int target = label_pos_[static_cast<std::size_t>(fx.label)];
        if (target < 0) throw std::runtime_error("unbound label");
        std::int64_t off = (static_cast<std::int64_t>(target) - static_cast<std::int64_t>(fx.at)) * 4;
        std::uint32_t w = a_.code[fx.at];
        switch (fx.kind) {
        case F_IMM26:
            w |= static_cast<std::uint32_t>(off >> 2) & 0x03FFFFFFu;
            break;
        case F_IMM19:
            if (off < -(1 << 20) || off >= (1 << 20)) throw std::runtime_error("branch out of range");
            w |= (static_cast<std::uint32_t>(off >> 2) & 0x7FFFFu) << 5;
            break;
        case F_IMM14:
            if (off < -(1 << 15) || off >= (1 << 15)) return false;
            w |= (static_cast<std::uint32_t>(off >> 2) & 0x3FFFu) << 5;
            break;
        case F_ADR: {
            std::uint32_t imm = static_cast<std::uint32_t>(off) & 0x1FFFFFu;
            w |= ((imm & 3) << 29) | ((imm >> 2) << 5);
            break;
        }
        case F_WORD:
            w = static_cast<std::uint32_t>(static_cast<std::int32_t>((static_cast<std::int64_t>(target) -
                                                                      static_cast<std::int64_t>(fx.base)) * 4));
            break;
        }
        a_.code[fx.at] = w;
    }
    return true;
}

// Host instruction scheduling (a64_sched.cpp) of the body and slow paths, before the tables and
// literals: blocks start where labels that branches, tables or entries use are bound. Patch sites
// move with their instructions.
void Compiler::schedule_code() {
    const std::size_t end = a_.pos();
    std::vector<bool> leader(end + 1, false);
    leader[0] = true;
    leader[std::min<std::size_t>(out_.entry, end)] = true;
    auto mark = [&](int label) {
        const int p = label_pos_[static_cast<std::size_t>(label)];
        if (p >= 0 && static_cast<std::size_t>(p) <= end) leader[static_cast<std::size_t>(p)] = true;
    };
    std::vector<int> target(end, -1);
    for (const auto& fx : fixes_) {
        mark(fx.label);
        const int p = label_pos_[static_cast<std::size_t>(fx.label)];
        if (fx.at < end && p >= 0 && (fx.kind == F_IMM26 || fx.kind == F_IMM19 || fx.kind == F_IMM14)) target[fx.at] = p;
    }
    for (const auto& t : tables_) {
        mark(t.fallback);
        for (int e : t.entries) mark(e);
    }
    const std::vector<std::uint32_t> moved = a64::schedule_a53(a_.code, end, leader, target);
    for (auto& fx : fixes_)
        if (fx.at < end) fx.at = moved[fx.at];
    for (auto& c : out_.calls) c.first = moved[c.first];
    for (auto& c : out_.external_calls) c.first = moved[c.first];
}

// Load scheduling for the in-order A53 (3-cycle load-to-use): within a basic block, each
// load moves above earlier instructions it does not depend on, so its value (and its
// deferred byte swap) is ready when used. Positions, labels and branches stay; only which
// instruction sits at a position changes. Anything not decoded below is a barrier.
namespace {
struct Effects {
    std::uint32_t gr = 0, gw = 0, fr = 0, fw = 0;   // GPRs and FPRs (both halves) read/written
    int mem = 0;                                    // 1 load, 2 store
    bool barrier = false, dform = false;
    unsigned base = 0, size = 0;
    std::int32_t disp = 0;
};
Effects effects(Op op, Insn i) {
    Effects e;
    auto g = [](unsigned r) { return 1u << r; };
    auto ga = [&] { return i.ra() ? g(i.ra()) : 0u; };
    auto mem = [&](int kind, unsigned size, bool indexed) {
        e.mem = kind;
        e.gr |= ga() | (indexed ? g(i.rb()) : 0u);
        e.dform = !indexed;
        e.base = i.ra();
        e.disp = i.simm();
        e.size = size;
    };
    switch (op) {
    case Op::addi: case Op::addis: e.gr = ga(); e.gw = g(i.rd()); break;
    case Op::addic: case Op::addic_rc: case Op::subfic: case Op::mulli: e.gr = g(i.ra()); e.gw = g(i.rd()); break;
    case Op::ori: case Op::oris: case Op::xori: case Op::xoris: case Op::andi_rc: case Op::andis_rc: case Op::rlwinm:
    case Op::srawi: case Op::extsb: case Op::extsh: case Op::cntlzw:
        e.gr = g(i.rs()); e.gw = g(i.ra()); break;
    case Op::rlwimi: e.gr = g(i.rs()) | g(i.ra()); e.gw = g(i.ra()); break;
    case Op::rlwnm: case Op::and_: case Op::andc: case Op::or_: case Op::orc: case Op::xor_: case Op::nand: case Op::nor:
    case Op::eqv: case Op::slw: case Op::srw: case Op::sraw:
        e.gr = g(i.rs()) | g(i.rb()); e.gw = g(i.ra()); break;
    case Op::cmpi: case Op::cmpli: e.gr = g(i.ra()); break;
    case Op::cmp: case Op::cmpl: e.gr = g(i.ra()) | g(i.rb()); break;
    case Op::add: case Op::addc: case Op::adde: case Op::subf: case Op::subfc: case Op::subfe: case Op::mullw:
    case Op::mulhw: case Op::mulhwu: case Op::divw: case Op::divwu:
        e.gr = g(i.ra()) | g(i.rb()); e.gw = g(i.rd()); break;
    case Op::addze: case Op::addme: case Op::subfze: case Op::subfme: case Op::neg: e.gr = g(i.ra()); e.gw = g(i.rd()); break;
    case Op::mfcr: e.gw = g(i.rd()); break;
    case Op::mtcrf: e.gr = g(i.rs()); break;
    case Op::mcrf: case Op::crand: case Op::crandc: case Op::cror: case Op::crorc: case Op::crxor: case Op::crnand:
    case Op::crnor: case Op::creqv:
        break;
    case Op::mfspr: if (i.spr() == 8 || i.spr() == 9) e.gw = g(i.rd()); else e.barrier = true; break;
    case Op::mtspr: if (i.spr() == 8 || i.spr() == 9) e.gr = g(i.rs()); else e.barrier = true; break;
    case Op::lwz: mem(1, 4, false); e.gw = g(i.rd()); break;
    case Op::lhz: case Op::lha: mem(1, 2, false); e.gw = g(i.rd()); break;
    case Op::lbz: mem(1, 1, false); e.gw = g(i.rd()); break;
    case Op::lwzx: case Op::lhzx: case Op::lhax: case Op::lbzx: mem(1, 4, true); e.gw = g(i.rd()); break;
    case Op::lfs: mem(1, 4, false); e.fw = g(i.rd()); break;
    case Op::lfd: mem(1, 8, false); e.fw = g(i.rd()); break;
    case Op::lfsx: case Op::lfdx: mem(1, 8, true); e.fw = g(i.rd()); break;
    case Op::stw: mem(2, 4, false); e.gr |= g(i.rs()); break;
    case Op::sth: mem(2, 2, false); e.gr |= g(i.rs()); break;
    case Op::stb: mem(2, 1, false); e.gr |= g(i.rs()); break;
    case Op::stwx: case Op::sthx: case Op::stbx: mem(2, 4, true); e.gr |= g(i.rs()); break;
    case Op::stfs: mem(2, 4, false); e.fr = g(i.rs()); break;
    case Op::stfd: mem(2, 8, false); e.fr = g(i.rs()); break;
    case Op::stfsx: case Op::stfdx: case Op::stfiwx: mem(2, 8, true); e.fr = g(i.rs()); break;
    case Op::fcmpu: case Op::fcmpo: case Op::ps_cmpu0: case Op::ps_cmpo0: case Op::ps_cmpu1: case Op::ps_cmpo1:
        e.fr = g(i.ra()) | g(i.rb()); break;
    case Op::fdivs: case Op::fsubs: case Op::fadds: case Op::fres: case Op::fmuls: case Op::fmsubs: case Op::fmadds:
    case Op::fnmsubs: case Op::fnmadds: case Op::fdiv: case Op::fsub: case Op::fadd: case Op::fsel: case Op::fmul:
    case Op::frsqrte: case Op::fmsub: case Op::fmadd: case Op::fnmsub: case Op::fnmadd: case Op::frsp: case Op::fctiwz:
    case Op::fneg: case Op::fmr: case Op::fnabs: case Op::fabs:
    case Op::ps_sum0: case Op::ps_sum1: case Op::ps_muls0: case Op::ps_muls1: case Op::ps_madds0: case Op::ps_madds1:
    case Op::ps_div: case Op::ps_sub: case Op::ps_add: case Op::ps_sel: case Op::ps_res: case Op::ps_mul:
    case Op::ps_rsqrte: case Op::ps_msub: case Op::ps_madd: case Op::ps_nmsub: case Op::ps_nmadd: case Op::ps_neg:
    case Op::ps_mr: case Op::ps_nabs: case Op::ps_abs: case Op::ps_merge00: case Op::ps_merge01: case Op::ps_merge10:
    case Op::ps_merge11:
        e.fr = g(i.ra()) | g(i.rb()) | g(i.rc()); e.fw = g(i.rd()); break;
    default: e.barrier = true; break;
    }
    return e;
}
// Bases that never address hardware registers (needs_io_check).
bool plain_base(const Effects& e) { return e.dform && (e.base == 1 || e.base == 2 || e.base == 13); }
// Whether load x (below) may move above y.
bool may_pass(const Effects& x, const Effects& y) {
    if (y.barrier) return false;
    if ((y.gw & x.gr) || (y.fw & x.fr) || (x.gw & (y.gr | y.gw)) || (x.fw & (y.fr | y.fw))) return false;
    if (y.mem == 2) {
        if (!(x.dform && y.dform && x.base == 1 && y.base == 1)) return false;
        return x.disp + static_cast<std::int32_t>(x.size) <= y.disp || y.disp + static_cast<std::int32_t>(y.size) <= x.disp;
    }
    if (y.mem == 1) return plain_base(x) || plain_base(y);
    return true;
}
} // namespace

void Compiler::schedule() {
    std::size_t block = 0;
    for (std::size_t k = 0; k < n_; ++k) {
        if (label_pcs_.count(f_.start + 4 * static_cast<std::uint32_t>(k))) block = k;
        const Effects x = effects(ops_[k], insns_[k]);
        if (x.barrier) { block = k + 1; continue; }
        if (x.mem != 1) continue;
        std::size_t j = k;
        while (j > block && may_pass(x, effects(ops_[j - 1], insns_[j - 1]))) {
            std::swap(insns_[j], insns_[j - 1]);
            std::swap(ops_[j], ops_[j - 1]);
            std::swap(pcs_[j], pcs_[j - 1]);
            --j;
        }
    }
}

FunctionCode Compiler::run() {
    decode();
    if (f_.hle.empty()) {
        schedule();
        analyze();
        for (std::size_t k = 0; k < n_; ++k)
            for (auto& p : info_[k].paths)
                if (p.flags & (P_RELOAD | P_RELOAD_ALL)) has_calls_ = true;
        liveness();
        // Live after a guest call without being reloaded: scratch state that the call does not
        // synchronise (non-EABI code reads it), kept in callee-saved registers.
        across_ = VSet{};
        for (std::size_t k = 0; k < n_; ++k)
            for (auto& p : info_[k].paths) {
                if (!(p.flags & P_RELOAD) || p.target == kExit) continue;
                const VSet& t = p.target == kFall ? info_[k + 1].live_in : info_[static_cast<std::size_t>(p.target)].live_in;
                across_ |= (t - p.defs) & mask_;
            }
        leaf_ = !has_calls_;
        allocate();
        layout_frame();
    }
    const FunctionCode counts = out_;
    generate();
    if (!finish()) {
        near_io_ = false;
        out_ = counts;
        generate();
        if (!finish()) throw std::runtime_error("branch out of range");
    }
    out_.code = std::move(a_.code);
    out_.instructions = n_;
    return std::move(out_);
}

Summary Compiler::summarize(std::vector<Site>& sites) {
    decode();
    schedule();
    analyze();
    liveness();
    sites = std::move(sites_);
    Summary s;
    s.in = info_[0].live_in & pass_set(false);
    if (exits_) s.out = ret_must_;
    return s;
}

} // namespace

void sha1(const std::uint8_t* data, std::size_t size, std::uint8_t out[20]) {
    std::uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    auto rotl = [](std::uint32_t x, unsigned n) { return (x << n) | (x >> (32 - n)); };
    auto block = [&](const std::uint8_t* p) {
        std::uint32_t w[80];
        for (unsigned t = 0; t < 16; ++t)
            w[t] = (std::uint32_t(p[4 * t]) << 24) | (std::uint32_t(p[4 * t + 1]) << 16) | (std::uint32_t(p[4 * t + 2]) << 8) | p[4 * t + 3];
        for (unsigned t = 16; t < 80; ++t) w[t] = rotl(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
        std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (unsigned t = 0; t < 80; ++t) {
            std::uint32_t f, k;
            if (t < 20) { f = (b & c) | (~b & d); k = 0x5A827999u; }
            else if (t < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1u; }
            else if (t < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
            else { f = b ^ c ^ d; k = 0xCA62C1D6u; }
            std::uint32_t temp = rotl(a, 5) + f + e + k + w[t];
            e = d; d = c; c = rotl(b, 30); b = a; a = temp;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    };
    std::size_t full = size / 64 * 64;
    for (std::size_t o = 0; o < full; o += 64) block(data + o);
    std::uint8_t tail[128] = {};
    std::size_t rest = size - full;
    std::memcpy(tail, data + full, rest);
    tail[rest] = 0x80;
    std::size_t len = rest + 1 + 8 <= 64 ? 64 : 128;
    std::uint64_t bits = std::uint64_t(size) * 8;
    for (unsigned n = 0; n < 8; ++n) tail[len - 1 - n] = static_cast<std::uint8_t>(bits >> (8 * n));
    for (std::size_t o = 0; o < len; o += 64) block(tail + o);
    for (unsigned n = 0; n < 5; ++n)
        for (unsigned b = 0; b < 4; ++b) out[4 * n + b] = static_cast<std::uint8_t>(h[n] >> (24 - 8 * b));
}

namespace {

void function_error(const gcr::Function& f, const std::exception& e, std::string& error) {
    char b[96];
    std::snprintf(b, sizeof b, "function %08X: ", f.start);
    error = b + std::string(e.what());
}

// Register conventions of the internal entries, a fixed point over direct calls and tail
// branches starting from empty sets. A function takes a passable value in its register when
// the value is live at its entry and modified before half or more of its call sites (those
// callers hold it in a register anyway; values most callers leave alone, like the unused
// argument registers a varargs prologue saves, stay in the context). It returns a result in its
// register when the result is modified on every path to every exit and half or more of the
// call sites use it. A function is analysed again when a convention it uses, or its own,
// changes; everything only grows, so this ends. Any convention is valid as long as callers and
// callee use the same one.
bool summarize_all(const gcr::Program& program, const Options& options, std::vector<Summary>& sums,
                   const Progress& progress, std::size_t total, std::string& error) {
    const auto& functions = program.functions;
    const std::size_t count = functions.size();
    sums.assign(count, Summary{});
    for (std::size_t n = 0; n < count; ++n)
        sums[n].fixed = !functions[n].hle.empty() || !functions[n].wrap.empty() || functions[n].exact;
    std::vector<Summary> own(count);   // live passable values at the entry; results modified at every exit
    std::vector<std::vector<Site>> sites(count);
    std::vector<std::vector<std::size_t>> users(count);   // functions with call sites of each function
    std::vector<char> queued(count, 0);
    std::deque<std::size_t> work;
    auto requeue = [&](std::size_t n) {
        if (!queued[n]) { work.push_back(n); queued[n] = 1; }
    };
    auto vote = [&](std::size_t h) {
        Summary r;
        unsigned total = 0, in_i[32] = {}, in_f[NF] = {}, out_i[32] = {}, out_f[NF] = {};
        for (std::size_t g : users[h])
            for (const Site& site : sites[g]) {
                if (site.callee != h) continue;
                ++total;
                for (unsigned v = 0; v < 32; ++v) {
                    in_i[v] += site.modified.has_i(v);
                    out_i[v] += site.demanded.has_i(v);
                }
                for (unsigned v = 0; v < NF; ++v) {
                    in_f[v] += site.modified.has_f(v);
                    out_f[v] += site.demanded.has_f(v);
                }
            }
        if (!total) return r;
        for (unsigned v = 0; v < 32; ++v) {
            if (own[h].in.has_i(v) && 2 * in_i[v] >= total) r.in.add_i(v);
            if (own[h].out.has_i(v) && 2 * out_i[v] >= total) r.out.add_i(v);
        }
        for (unsigned v = 0; v < NF; ++v) {
            if (own[h].in.has_f(v) && 2 * in_f[v] >= total) r.in.add_f(v);
            if (own[h].out.has_f(v) && 2 * out_f[v] >= total) r.out.add_f(v);
        }
        return r;
    };
    auto update = [&](std::size_t n) {
        if (sums[n].fixed) return;
        Summary v = vote(n);
        VSet i = sums[n].in | v.in, o = sums[n].out | v.out;
        if (i == sums[n].in && o == sums[n].out) return;
        sums[n].in = i;
        sums[n].out = o;
        requeue(n);
        for (std::size_t u : users[n]) requeue(u);
    };
    // r1 always travels in its register: nearly every function uses the stack pointer.
    VSet r1;
    r1.add_i(V_R + 1);
    for (std::size_t n = 0; n < count; ++n)
        if (!sums[n].fixed) sums[n].in = sums[n].out = r1;
    // Analyses n; true when its call sites changed.
    std::vector<std::size_t> callees;
    auto analyse = [&](std::size_t n, bool& moved) {
        Summary s;
        std::vector<Site> here;
        try {
            Compiler c(program, functions[n], options, sums);
            s = c.summarize(here);
        } catch (const std::exception& e) {
            function_error(functions[n], e, error);
            return false;
        }
        callees.clear();
        for (const Site& site : here) callees.push_back(site.callee);
        std::sort(callees.begin(), callees.end());
        callees.erase(std::unique(callees.begin(), callees.end()), callees.end());
        own[n].in = own[n].in | s.in;
        own[n].out = own[n].out | s.out;
        moved = here != sites[n];
        sites[n] = std::move(here);
        return true;
    };
    // Every call site is recorded before the first vote: a vote over part of them could admit
    // a value that the complete vote rejects, and conventions never shrink.
    bool moved = false;
    std::size_t first = 0;
    for (std::size_t n = 0; n < count; ++n) {
        if (sums[n].fixed) continue;
        if (!analyse(n, moved)) return false;
        for (std::size_t c : callees) users[c].push_back(n);
        if (progress) progress(++first, total);
    }
    for (std::size_t n = 0; n < count; ++n) update(n);
    while (!work.empty()) {
        std::size_t n = work.front();
        work.pop_front();
        queued[n] = 0;
        if (!analyse(n, moved)) return false;
        update(n);
        if (moved)
            for (std::size_t c : callees) update(c);
    }
    return true;
}

} // namespace

bool translate(const gcr::Program& program, const Options& options, std::vector<std::uint8_t>& blob, Stats& stats,
               const Progress& progress, std::string& error) {
    const auto& functions = program.functions;
    std::vector<Summary> sums;
    std::size_t first = 0;
    for (auto& f : functions) first += f.hle.empty() && f.wrap.empty() && !f.exact;
    if (!summarize_all(program, options, sums, progress, first + functions.size(), error)) return false;
    std::vector<FunctionCode> code(functions.size());
    for (std::size_t n = 0; n < functions.size(); ++n) {
        try {
            Compiler c(program, functions[n], options, sums);
            code[n] = c.run();
        } catch (const std::exception& e) {
            function_error(functions[n], e, error);
            return false;
        }
        if (progress) progress(first + n + 1, first + functions.size());
    }
    std::vector<std::uint32_t> offset(functions.size());
    std::uint32_t at = 0;
    for (std::size_t n = 0; n < functions.size(); ++n) {
        at = (at + 15) & ~15u;
        offset[n] = at;
        at += static_cast<std::uint32_t>(code[n].code.size() * 4);
    }
    std::uint32_t code_size = (at + 15) & ~15u;
    std::vector<std::uint8_t> text(code_size, 0);
    std::vector<std::string> imports;
    std::map<std::string, std::uint32_t> import_index;
    for (unsigned i = 0; i < I_COUNT; ++i) {
        import_index[kImportNames[i]] = i;
        imports.push_back(kImportNames[i]);
    }
    std::vector<gcbe_reloc> relocs;
    stats = Stats{};
    for (std::size_t n = 0; n < functions.size(); ++n) {
        auto& fc = code[n];
        // Direct calls enter at the callee's internal entry; resume points at its external entry.
        for (int external = 0; external < 2; ++external)
            for (auto& [word, target] : external ? fc.external_calls : fc.calls) {
                auto it = program.by_start.find(target);
                if (it == program.by_start.end()) { error = "call to a missing function"; return false; }
                std::int64_t entry = external ? 0 : 4 * std::int64_t(code[it->second].entry);
                std::int64_t off = static_cast<std::int64_t>(offset[it->second]) + entry - (offset[n] + std::int64_t(word) * 4);
                if (off < -(1ll << 27) || off >= (1ll << 27)) { error = "code image exceeds the branch range"; return false; }
                fc.code[word] |= static_cast<std::uint32_t>(off >> 2) & 0x03FFFFFFu;
            }
        for (auto& [byte, name] : fc.relocs) {
            auto it = import_index.find(name);
            std::uint32_t index;
            if (it == import_index.end()) {
                index = static_cast<std::uint32_t>(imports.size());
                import_index[name] = index;
                imports.push_back(name);
            } else {
                index = it->second;
            }
            relocs.push_back({offset[n] + byte, index});
        }
        std::memcpy(text.data() + offset[n], fc.code.data(), fc.code.size() * 4);
        stats.instructions += fc.instructions;
        stats.spilled_values += fc.spilled;
        stats.invalid += fc.invalid;
        stats.spin_loops += fc.spin_loops;
    }
    stats.functions = functions.size();

    stats.code_bytes = code_size;

    std::vector<std::uint8_t> names;
    for (auto& s : imports) {
        names.insert(names.end(), s.begin(), s.end());
        names.push_back(0);
    }
    while (names.size() % 8) names.push_back(0);
    gcbe_header h{};
    h.magic = GCBE_MAGIC;
    h.version = GCBE_VERSION;
    h.flags = options.virtual_time ? GCBE_FLAG_VIRTUAL_TIME : 0;
    h.translator = GCBE_TRANSLATOR;
    h.dol_hash = gcr::fnv1a(program.image->bytes());
    sha1(program.image->bytes().data(), program.image->bytes().size(), h.dol_sha1);
    h.function_count = static_cast<std::uint32_t>(functions.size());
    h.import_count = static_cast<std::uint32_t>(imports.size());
    h.reloc_count = static_cast<std::uint32_t>(relocs.size());
    h.code_size = code_size;
    std::uint32_t pos = (sizeof h + 15) & ~15u;
    h.functions_offset = pos;
    pos += h.function_count * sizeof(gcbe_function);
    h.imports_offset = pos;
    pos += static_cast<std::uint32_t>(names.size());
    h.relocs_offset = pos;
    pos += h.reloc_count * sizeof(gcbe_reloc);
    pos = (pos + 4095) & ~4095u;
    h.code_offset = pos;
    pos += code_size;
    h.file_size = pos;
    blob.assign(pos, 0);
    std::memcpy(blob.data(), &h, sizeof h);
    for (std::size_t n = 0; n < functions.size(); ++n) {
        gcbe_function f{functions[n].start, functions[n].end, offset[n], static_cast<std::uint32_t>(code[n].code.size() * 4)};
        std::memcpy(blob.data() + h.functions_offset + n * sizeof f, &f, sizeof f);
    }
    std::memcpy(blob.data() + h.imports_offset, names.data(), names.size());
    std::memcpy(blob.data() + h.relocs_offset, relocs.data(), relocs.size() * sizeof(gcbe_reloc));
    std::memcpy(blob.data() + h.code_offset, text.data(), text.size());
    return true;
}

} // namespace gcbe

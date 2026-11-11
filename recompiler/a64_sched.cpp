// List scheduling of emitted A64 code for the Cortex-A53, a two-wide in-order core: within a
// basic block, an instruction moves earlier when it can issue in a cycle that would otherwise go
// unused. Latencies and issue resources follow LLVM's Cortex-A53 machine model
// (AArch64SchedA53.td): integer results reach ALU operands after 1 cycle (2 for shifted and
// multiply operands) but addresses, store data and branches after 3; loads 4, multiplies 4,
// floating-point and SIMD 6, fused multiply-add 10. Two instructions issue per cycle, of which at
// most one load/store, one branch, one multiply, one FP/SIMD ALU and one FP multiply/divide.
//
// Before scheduling, values that live only within a block in a temporary or scratch register
// (x12-x17, v28-v31) move to a caller-saved register that is free over their lifetime, so reuse of
// the temporaries does not serialise independent instructions. Liveness is solved over the
// function's blocks; calls and exits use the registers the calling convention passes (arguments,
// results, callee-saved).
//
// Dependencies: registers (X0-X30, SP, V0-V31, NZCV) and memory. Accesses through x27 (gc_ctx),
// SP (the frame) and x28 plus an index (guest memory) are disjoint from each other, and two
// context or two frame accesses are disjoint when their byte ranges are. Literal-pool loads never
// alias a store. Any other access is ordered against every store.
#include "a64_sched.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace a64 {
namespace {

enum Unit : std::uint8_t { U_ALU, U_MAC, U_DIV, U_LDST, U_BR, U_FPALU, U_FPMDS, U_COUNT };
constexpr std::uint8_t kUnitWidth[U_COUNT] = {2, 1, 1, 1, 1, 1, 1};
enum Cls : std::uint8_t { M_NONE, M_CTX, M_STACK, M_GUEST, M_LIT, M_OTHER };

// Register numbers in the dependency tracker: X0-X30, 31 SP, 32-63 V0-V31, 64 NZCV.
constexpr unsigned kSP = 31, kV = 32, kNZCV = 64, kRegs = 65;

struct Info {
    std::uint32_t rx = 0, wx = 0, rv = 0, wv = 0;   // bit 31 of rx/wx: SP
    bool rf = false, wf = false;
    std::uint32_t addr_x = 0;   // integer registers read without forwarding (addresses, store data)
    std::uint32_t late_x = 0;   // integer registers read one cycle later than ALU operands
    std::uint8_t unit = U_ALU, slots = 1, busy = 1, lat = 3;
    bool int_result = false;    // integer pipeline result, forwarded early to ALU operands
    bool alu_read = false;      // reads integer operands at ALU timing
    bool term = false, barrier = false;
    std::uint8_t mem = 0;       // 1 load, 2 store
    Cls cls = M_NONE;
    std::int32_t off = 0;
    std::int32_t size = 0;
    // Register operand fields: bit position of each, and which of them name SIMD&FP registers.
    std::uint8_t nf = 0, fv = 0;
    std::array<std::uint8_t, 4> fs{};
};

inline std::uint32_t gx(unsigned r) { return r == 31 ? 0u : 1u << r; }        // 31 is XZR
inline std::uint32_t gs(unsigned r) { return 1u << r; }                         // 31 is SP
inline std::uint32_t vx(unsigned r) { return 1u << r; }
inline std::int32_t sext(std::uint32_t v, unsigned bits) {
    return static_cast<std::int32_t>(v << (32 - bits)) >> (32 - bits);
}

Info decode(std::uint32_t w) {
    Info d;
    const unsigned rd = w & 31, rn = (w >> 5) & 31, rm = (w >> 16) & 31, ra = (w >> 10) & 31;
    auto alu = [&](std::uint8_t lat) {
        d.unit = U_ALU;
        d.lat = lat;
        d.int_result = d.alu_read = true;
    };
    auto fp = [&](std::uint8_t unit, std::uint8_t lat, std::uint8_t busy = 1) {
        d.unit = unit;
        d.lat = lat;
        d.busy = busy;
    };
    auto fg = [&](unsigned shift) { d.fs[d.nf++] = static_cast<std::uint8_t>(shift); };
    auto fsimd = [&](unsigned shift, bool simd = true) {
        if (simd) d.fv = static_cast<std::uint8_t>(d.fv | (1u << d.nf));
        d.fs[d.nf++] = static_cast<std::uint8_t>(shift);
    };
    auto mem_base = [&](bool load, unsigned base, Cls cls_if_known) {
        d.mem = load ? 1 : 2;
        d.unit = U_LDST;
        d.lat = load ? 4 : 1;
        d.rx |= gs(base);
        d.addr_x |= gs(base);
        d.cls = base == 27 ? M_CTX : base == 31 ? M_STACK : base == 28 ? M_GUEST : cls_if_known;
    };
    auto transfer = [&](bool load, bool simd, unsigned rt) {
        if (simd) (load ? d.wv : d.rv) |= vx(rt);
        else if (load) d.wx |= gx(rt);
        else { d.rx |= gx(rt); d.addr_x |= gx(rt); }
    };

    if ((w & 0x1F800000u) == 0x11000000u) {            // add/sub (immediate)
        fg(0); fg(5);
        const bool s = (w >> 29) & 1;
        alu(3);
        d.rx = gs(rn);
        d.wx = s ? gx(rd) : gs(rd);
        d.wf = s;
    } else if ((w & 0x1F200000u) == 0x0B000000u) {     // add/sub (shifted register)
        fg(0); fg(5); fg(16);
        alu(3);
        d.rx = gx(rn) | gx(rm);
        d.wx = gx(rd);
        d.wf = (w >> 29) & 1;
        if ((w >> 10) & 63) d.late_x = gx(rm);
    } else if ((w & 0x1FE00000u) == 0x0B200000u) {     // add/sub (extended register)
        fg(0); fg(5); fg(16);
        const bool s = (w >> 29) & 1;
        alu(3);
        d.rx = gs(rn) | gx(rm);
        d.wx = s ? gx(rd) : gs(rd);
        d.wf = s;
        d.late_x = gx(rm);
    } else if ((w & 0x1FE0FC00u) == 0x1A000000u) {     // adc/sbc
        fg(0); fg(5); fg(16);
        alu(3);
        d.rx = gx(rn) | gx(rm);
        d.rf = true;
        d.wx = gx(rd);
        d.wf = (w >> 29) & 1;
    } else if ((w & 0x1F000000u) == 0x0A000000u) {     // logical (shifted register)
        fg(0); fg(5); fg(16);
        alu(3);
        d.rx = gx(rn) | gx(rm);
        d.wx = gx(rd);
        d.wf = ((w >> 29) & 3) == 3;
        if ((w >> 10) & 63) d.late_x = gx(rm);
    } else if ((w & 0x1F800000u) == 0x12000000u) {     // logical (immediate)
        fg(0); fg(5);
        const bool s = ((w >> 29) & 3) == 3;
        alu(3);
        d.rx = gx(rn);
        d.wx = s ? gx(rd) : gs(rd);
        d.wf = s;
    } else if ((w & 0x1F800000u) == 0x12800000u) {     // move wide
        fg(0);
        const unsigned opc = (w >> 29) & 3;
        alu(3);
        if (opc == 3) d.rx = gx(rd);
        d.wx = gx(rd);
        if (opc == 1) d.barrier = true;
    } else if ((w & 0x1F800000u) == 0x13000000u) {     // bitfield
        fg(0); fg(5);
        const unsigned opc = (w >> 29) & 3;
        alu(2);
        d.rx = gx(rn) | (opc == 1 ? gx(rd) : 0u);
        d.wx = gx(rd);
        if (opc == 3) d.barrier = true;
    } else if ((w & 0x1FA00000u) == 0x13800000u) {     // extract
        fg(0); fg(5); fg(16);
        alu(3);
        d.rx = gx(rn) | gx(rm);
        d.wx = gx(rd);
    } else if ((w & 0x5FE00000u) == 0x1AC00000u) {     // data processing (2 source)
        fg(0); fg(5); fg(16);
        const unsigned op = (w >> 10) & 63;
        if (op == 2 || op == 3) {
            alu(4);
            d.unit = U_DIV;
            d.late_x = gx(rn) | gx(rm);
        } else if (op >= 8 && op <= 11) {
            alu(2);
        } else {
            d.barrier = true;
        }
        d.rx = gx(rn) | gx(rm);
        d.wx = gx(rd);
    } else if ((w & 0x5FE00000u) == 0x5AC00000u) {     // data processing (1 source)
        fg(0); fg(5);
        alu(3);
        d.rx = gx(rn);
        d.wx = gx(rd);
        if ((w >> 16) & 31) d.barrier = true;
    } else if ((w & 0x1F000000u) == 0x1B000000u) {     // data processing (3 source)
        fg(0); fg(5); fg(16); fg(10);
        alu(4);
        d.unit = U_MAC;
        d.rx = gx(rn) | gx(rm) | gx(ra);
        d.wx = gx(rd);
        d.late_x = gx(rn) | gx(rm);
    } else if ((w & 0x3FE00800u) == 0x1A800000u) {     // conditional select
        fg(0); fg(5); fg(16);
        alu(3);
        d.rx = gx(rn) | gx(rm);
        d.rf = true;
        d.wx = gx(rd);
    } else if ((w & 0x3FE00410u) == 0x3A400000u) {     // conditional compare
        fg(5);
        if (!((w >> 11) & 1)) fg(16);
        alu(3);
        d.rx = gx(rn) | (((w >> 11) & 1) ? 0u : gx(rm));
        d.rf = d.wf = true;
    } else if ((w & 0x3B000000u) == 0x39000000u) {     // load/store (unsigned offset)
        fsimd(0, (w >> 26) & 1); fg(5);
        const unsigned size = w >> 30, opc = (w >> 22) & 3;
        const bool simd = (w >> 26) & 1;
        if (!simd && size == 3 && opc == 2) { d.barrier = true; return d; }
        const bool load = opc != 0;
        const unsigned bytes = simd && opc >= 2 ? 16u : 1u << size;
        mem_base(load, rn, M_OTHER);
        transfer(load, simd, rd);
        d.off = static_cast<std::int32_t>(((w >> 10) & 0xFFF) * bytes);
        d.size = static_cast<std::int32_t>(bytes);
    } else if ((w & 0x3B200C00u) == 0x38200800u) {     // load/store (register offset)
        fsimd(0, (w >> 26) & 1); fg(5); fg(16);
        const unsigned opc = (w >> 22) & 3;
        const bool simd = (w >> 26) & 1;
        const bool load = opc != 0;
        if (!simd && (w >> 30) == 3 && opc == 2) { d.barrier = true; return d; }
        mem_base(load, rn, M_OTHER);
        if (d.cls != M_GUEST) d.cls = M_OTHER;
        d.rx |= gx(rm);
        d.addr_x |= gx(rm);
        transfer(load, simd, rd);
    } else if ((w & 0x3A000000u) == 0x28000000u) {     // load/store pair
        fsimd(0, (w >> 26) & 1); fsimd(10, (w >> 26) & 1); fg(5);
        const unsigned opc = w >> 30, mode = (w >> 23) & 3, rt2 = (w >> 10) & 31;
        const bool simd = (w >> 26) & 1, load = (w >> 22) & 1;
        if (mode == 0 || opc == 3) { d.barrier = true; return d; }
        const unsigned bytes = simd ? 4u << opc : opc == 2 ? 8u : 4u;
        mem_base(load, rn, M_OTHER);
        transfer(load, simd, rd);
        transfer(load, simd, rt2);
        if (mode == 1 || mode == 3) {
            d.wx |= gs(rn);
            if (rn == kSP) d.barrier = true;
        }
        d.off = mode == 1 ? 0 : sext((w >> 15) & 0x7F, 7) * static_cast<std::int32_t>(bytes);
        d.size = 2 * static_cast<std::int32_t>(bytes);
        d.slots = 2;
        d.busy = load ? 2 : 1;
    } else if ((w & 0x3B000000u) == 0x18000000u) {     // load literal
        fsimd(0, (w >> 26) & 1);
        const bool simd = (w >> 26) & 1;
        d.mem = 1;
        d.unit = U_LDST;
        d.lat = 4;
        d.cls = M_LIT;
        transfer(true, simd, rd);
        if (!simd && (w >> 30) == 3) d.barrier = true;   // prfm
    } else if ((w & 0x9F000000u) == 0x10000000u) {     // adr
        fg(0);
        alu(3);
        d.wx = gx(rd);
    } else if ((w & 0x7C000000u) == 0x14000000u) {     // b, bl
        d.unit = U_BR;
        d.lat = 1;
        d.term = true;
        if (w >> 31) d.wx = gx(30);
    } else if ((w & 0xFF000010u) == 0x54000000u) {     // b.cond
        d.unit = U_BR;
        d.lat = 1;
        d.term = true;
        d.rf = true;
    } else if ((w & 0x7C000000u) == 0x34000000u) {     // cbz/cbnz, tbz/tbnz
        fg(0);
        d.unit = U_BR;
        d.lat = 1;
        d.term = true;
        d.rx = gx(rd);
    } else if ((w & 0xFE000000u) == 0xD6000000u) {     // br, blr, ret
        fg(5);
        d.unit = U_BR;
        d.lat = 1;
        d.term = true;
        d.rx = gx(rn);
        if (((w >> 21) & 15) == 1) d.wx = gx(30);
        if (((w >> 21) & 15) > 2) d.barrier = true;
    } else if ((w & 0xFF200C00u) == 0x1E200800u) {     // FP data processing (2 source)
        fsimd(0); fsimd(5); fsimd(16);
        const unsigned op = (w >> 12) & 15, type = (w >> 22) & 3;
        d.rv = vx(rn) | vx(rm);
        d.wv = vx(rd);
        if (type > 1) d.barrier = true;
        if (op == 0 || op == 8) fp(U_FPMDS, 6);
        else if (op == 1) fp(U_FPMDS, type ? 32 : 18, type ? 28 : 14);
        else if (op <= 7) fp(U_FPALU, 6);
        else d.barrier = true;
    } else if ((w & 0xFF207C00u) == 0x1E204000u) {     // FP data processing (1 source)
        fsimd(0); fsimd(5);
        const unsigned op = (w >> 15) & 63, type = (w >> 22) & 3;
        d.rv = vx(rn);
        d.wv = vx(rd);
        if (op == 3) fp(U_FPMDS, type ? 32 : 17, type ? 28 : 13);
        else if (op <= 15) fp(U_FPALU, 6);
        else d.barrier = true;
        if (type == 2) d.barrier = true;
    } else if ((w & 0xFF000000u) == 0x1F000000u) {     // FP data processing (3 source)
        fsimd(0); fsimd(5); fsimd(16); fsimd(10);
        d.rv = vx(rn) | vx(rm) | vx(ra);
        d.wv = vx(rd);
        fp(U_FPMDS, 10);
        if (((w >> 22) & 3) > 1) d.barrier = true;
    } else if ((w & 0xFF20FC07u) == 0x1E202000u) {     // FP compare
        fsimd(5);
        if (!((w >> 3) & 1)) fsimd(16);
        d.rv = vx(rn) | (((w >> 3) & 1) ? 0u : vx(rm));
        d.wf = true;
        fp(U_FPALU, 6);
    } else if ((w & 0xFF200C00u) == 0x1E200C00u) {     // FP conditional select
        fsimd(0); fsimd(5); fsimd(16);
        d.rv = vx(rn) | vx(rm);
        d.rf = true;
        d.wv = vx(rd);
        fp(U_FPALU, 6);
    } else if ((w & 0xFF201FE0u) == 0x1E201000u) {     // FP immediate
        fsimd(0);
        d.wv = vx(rd);
        fp(U_FPALU, 6);
    } else if ((w & 0x7F20FC00u) == 0x1E200000u) {     // FP <-> integer conversion and moves
        const unsigned op = (w >> 16) & 7;
        const bool to_int = op == 0 || op == 1 || op == 4 || op == 5 || op == 6;
        if (to_int) { d.rv = vx(rn); d.wx = gx(rd); fg(0); fsimd(5); }
        else { d.rx = gx(rn); d.addr_x = gx(rn); d.wv = vx(rd); fsimd(0); fg(5); }
        fp(U_FPALU, 6);
        if (((w >> 22) & 3) > 1) d.barrier = true;
    } else if ((w & 0x9F3E0C00u) == 0x0E200800u) {     // AdvSIMD two-register miscellaneous
        fsimd(0); fsimd(5);
        d.rv = vx(rn) | (((w >> 30) & 1) ? vx(rd) : 0u);
        d.wv = vx(rd);
        fp(U_FPALU, 6);
    } else if ((w & 0xFFE08400u) == 0x6E000400u) {     // INS (element)
        fsimd(0); fsimd(5);
        d.rv = vx(rn) | vx(rd);
        d.wv = vx(rd);
        fp(U_FPALU, 6);
    } else if ((w & 0xBFE0FC00u) == 0x0E000400u || (w & 0xFFE0FC00u) == 0x5E000400u) {   // DUP (element)
        fsimd(0); fsimd(5);
        d.rv = vx(rn);
        d.wv = vx(rd);
        fp(U_FPALU, 6);
    } else {
        d.barrier = true;
    }
    if (d.wx & gs(kSP)) d.barrier = true;
    return d;
}

bool conflict(const Info& a, const Info& b) {
    if (a.mem != 2 && b.mem != 2) return false;
    if (a.cls == M_LIT || b.cls == M_LIT) return false;
    if (a.cls == M_OTHER || b.cls == M_OTHER) return true;
    if (a.cls != b.cls) return false;
    if (a.cls == M_GUEST) return true;
    return a.off < b.off + b.size && b.off < a.off + a.size;
}

// Cycles, relative to the start of the next block, at which each register's value is complete,
// and whether an integer-pipeline instruction produced it.
struct State {
    std::array<int, kRegs> ready{};
    std::array<bool, kRegs> int_result{};
    void clear() { ready.fill(0); int_result.fill(false); }
};

template <typename F> void each_x(std::uint32_t m, F f) {
    while (m) { unsigned r = static_cast<unsigned>(__builtin_ctz(m)); m &= m - 1; f(r); }
}

// Cycles after the producer issues until a consumer can issue that reads register `reg`.
int latency(const Info& p, const Info& c, unsigned reg) {
    int l = p.lat;
    if (reg < kV && p.int_result) {
        const std::uint32_t bit = 1u << reg;
        if (c.addr_x & bit) {}
        else if (c.late_x & bit) l -= 1;
        else if (c.alu_read) l -= 2;
    }
    return std::max(l, 1);
}
int incoming(const State& st, const Info& c, unsigned reg) {
    int t = st.ready[reg];
    if (reg < kV && st.int_result[reg]) {
        const std::uint32_t bit = 1u << reg;
        if (c.addr_x & bit) {}
        else if (c.late_x & bit) t -= 1;
        else if (c.alu_read) t -= 2;
    }
    return t;
}

constexpr std::size_t kWindow = 256;

class Block {
public:
    // Schedules n instructions (a branch at the end stays there), starting from st and leaving
    // in it the state after them. order receives the original indices in their new order.
    void run(const Info* info, std::size_t n, State& st, std::vector<std::uint32_t>& order) {
        n_ = n;
        edges_.clear();
        npred_.assign(n, 0);
        ready_.assign(n, 0);
        height_.assign(n, 0);
        issue_.assign(n, 0);
        build(info, st);
        // Successor lists, grouped by source.
        first_.assign(n + 1, 0);
        for (const Edge& e : edges_) ++first_[e.from + 1];
        for (std::size_t j = 0; j < n; ++j) first_[j + 1] += first_[j];
        succ_.resize(edges_.size());
        fill_.assign(first_.begin(), first_.end() - 1);
        for (const Edge& e : edges_) succ_[fill_[e.from]++] = {e.to, e.lat};
        for (std::size_t j = n; j-- > 0;) {
            int h = info[j].lat;
            for (std::uint32_t q = first_[j]; q < first_[j + 1]; ++q) h = std::max(h, succ_[q].lat + height_[succ_[q].to]);
            height_[j] = h;
        }
        select(info, order);
        finish(info, order, st);
    }

private:
    void edge(std::size_t a, std::size_t b, int lat) {
        edges_.push_back({static_cast<std::uint32_t>(a), static_cast<std::uint32_t>(b), lat});
        ++npred_[b];
    }

    void build(const Info* info, const State& st) {
        std::array<int, kRegs> last;
        last.fill(-1);
        for (unsigned r : touched_) readers_[r].clear();
        touched_.clear();
        mems_.clear();
        for (std::size_t j = 0; j < n_; ++j) {
            const Info& c = info[j];
            auto read = [&](unsigned reg) {
                if (last[reg] >= 0) edge(static_cast<std::size_t>(last[reg]), j, latency(info[last[reg]], c, reg));
                else ready_[j] = std::max(ready_[j], incoming(st, c, reg));
                if (readers_[reg].empty()) touched_.push_back(reg);
                readers_[reg].push_back(static_cast<int>(j));
            };
            auto write = [&](unsigned reg) {
                for (int r : readers_[reg])
                    if (r != static_cast<int>(j)) edge(static_cast<std::size_t>(r), j, 0);
                if (last[reg] >= 0) edge(static_cast<std::size_t>(last[reg]), j, 0);
                last[reg] = static_cast<int>(j);
                readers_[reg].clear();
            };
            each_x(c.rx, [&](unsigned r) { read(r); });
            each_x(c.rv, [&](unsigned r) { read(kV + r); });
            if (c.rf) read(kNZCV);
            if (c.mem) {
                for (std::size_t i : mems_)
                    if (conflict(info[i], c)) edge(i, j, info[i].mem == 2 && c.mem == 1 ? 1 : 0);
                mems_.push_back(j);
            }
            each_x(c.wx, [&](unsigned r) { write(r); });
            each_x(c.wv, [&](unsigned r) { write(kV + r); });
            if (c.wf) write(kNZCV);
        }
    }

    void select(const Info* info, std::vector<std::uint32_t>& order) {
        order.clear();
        if (n_ == 1) {
            order.push_back(0);
            issue_[0] = std::max(ready_[0], 0);
            return;
        }
        // Available instructions, highest priority first (height, then original order).
        std::vector<std::size_t>& avail = avail_;
        avail.clear();
        auto ahead = [&](std::size_t a, std::size_t b) {
            if (height_[a] != height_[b]) return height_[a] > height_[b];
            const std::uint32_t sa = first_[a + 1] - first_[a], sb = first_[b + 1] - first_[b];
            if (sa != sb) return sa > sb;
            return a < b;
        };
        auto add = [&](std::size_t j) {
            std::size_t k = avail.size();
            avail.push_back(j);
            for (; k > 0 && ahead(j, avail[k - 1]); --k) avail[k] = avail[k - 1];
            avail[k] = j;
        };
        for (std::size_t j = 0; j < n_; ++j)
            if (!npred_[j]) add(j);
        std::array<int, U_COUNT> used{}, free_at{};
        int cycle = 0, slots = 2;
        const std::size_t pinned = info[n_ - 1].term ? n_ - 1 : n_;   // a branch issues last
        while (order.size() < n_) {
            std::size_t best = n_, at = 0;
            for (std::size_t k = 0; k < avail.size(); ++k) {
                const std::size_t j = avail[k];
                const Info& c = info[j];
                if (j == pinned && order.size() + 1 < n_) continue;
                if (ready_[j] > cycle || c.slots > slots || used[c.unit] >= kUnitWidth[c.unit] || free_at[c.unit] > cycle)
                    continue;
                best = j;
                at = k;
                break;
            }
            if (best == n_) {
                // Nothing issues in this cycle: continue at the first cycle where something can.
                int next = cycle + 1;
                int soonest = std::numeric_limits<int>::max();
                for (std::size_t j : avail)
                    if (j != pinned || order.size() + 1 == n_) soonest = std::min(soonest, std::max(ready_[j], free_at[info[j].unit]));
                cycle = std::max(next, soonest);
                slots = 2;
                used.fill(0);
                continue;
            }
            const Info& c = info[best];
            avail.erase(avail.begin() + static_cast<std::ptrdiff_t>(at));
            order.push_back(static_cast<std::uint32_t>(best));
            issue_[best] = cycle;
            slots -= c.slots;
            ++used[c.unit];
            if (c.busy > 1) free_at[c.unit] = cycle + c.busy;
            for (std::uint32_t q = first_[best]; q < first_[best + 1]; ++q) {
                const std::uint32_t t = succ_[q].to;
                ready_[t] = std::max(ready_[t], cycle + succ_[q].lat);
                if (!--npred_[t]) add(t);
            }
            if (!slots) {
                ++cycle;
                slots = 2;
                used.fill(0);
            }
        }
    }

    void finish(const Info* info, const std::vector<std::uint32_t>& order, State& st) {
        const std::size_t last = order.back();
        const int end = issue_[last] + 1;
        for (auto& r : st.ready) r -= end;
        for (std::size_t j : order) {
            const Info& c = info[j];
            auto set = [&](unsigned reg) {
                st.ready[reg] = issue_[j] + c.lat - end;
                st.int_result[reg] = c.int_result;
            };
            each_x(c.wx, set);
            each_x(c.wv, [&](unsigned r) { set(kV + r); });
            if (c.wf) set(kNZCV);
        }
    }

    std::size_t n_ = 0;
    struct Edge { std::uint32_t from, to; int lat; };
    struct Succ { std::uint32_t to; int lat; };
    std::vector<Edge> edges_;
    std::vector<Succ> succ_;
    std::vector<std::uint32_t> first_, fill_;
    std::vector<int> npred_, ready_, height_, issue_;
    std::array<std::vector<int>, kRegs> readers_;
    std::vector<unsigned> touched_;
    std::vector<std::size_t> mems_, avail_;
};

// Whether execution continues into the next instruction after this terminator.
bool falls_through(std::uint32_t w) {
    if ((w & 0xFC000000u) == 0x14000000u) return false;              // b
    if ((w & 0xFF00001Fu) == 0x5400000Eu) return false;              // b.al
    if ((w & 0xFE000000u) == 0xD6000000u && ((w >> 21) & 15) != 1) return false;   // br, ret
    return true;
}
bool is_call(std::uint32_t w) {
    return (w & 0xFC000000u) == 0x94000000u || (w & 0xFFFFFC1Fu) == 0xD63F0000u;
}

// Liveness masks: bits 0-30 X0-X30, bits 32-63 V0-V31 (SP and NZCV are never renamed).
using Mask = std::uint64_t;
constexpr Mask range(unsigned lo, unsigned hi) { return ((~0ull) >> (63 - hi)) & ~((1ull << lo) - 1); }
constexpr Mask kAll = range(0, 30) | range(32, 63);
// Translated functions pass and return values in x0-x8, v0-v7 and v16-v23 and keep x19-x30 and
// v8-v15; runtime functions follow the AAPCS64. Everything else is free at calls and exits.
constexpr Mask kCallUses = range(0, 8) | range(27, 30) | range(32, 39) | range(48, 55);
constexpr Mask kCallClobbers = range(0, 18) | range(30, 30) | range(32, 39) | range(48, 63);
constexpr Mask kExitUses = range(0, 8) | range(19, 30) | range(32, 55);
// Registers whose block-local values may move, and where they may move to (caller-saved).
constexpr Mask kRenamed = range(12, 17) | range(60, 63);
constexpr Mask kFree = range(0, 17) | range(32, 39) | range(48, 63);

Mask reads(const Info& c) { return (c.rx & 0x7FFFFFFFull) | (Mask(c.rv) << 32); }
Mask writes(const Info& c) { return (c.wx & 0x7FFFFFFFull) | (Mask(c.wv) << 32); }

// Registers live before instruction w, given those live after it. has_target: a branch whose
// target is known (after then includes the target's live registers).
Mask live_before(std::uint32_t w, const Info& c, Mask after, bool has_target) {
    if (c.barrier) return kAll;
    if (is_call(w)) return (after & ~kCallClobbers & ~writes(c)) | kCallUses | reads(c);
    if ((w & 0xFE000000u) == 0xD6000000u) {
        if (((w >> 21) & 15) == 2) return kExitUses | reads(c);   // ret
        return kAll;                                               // br: tail jumps and tables
    }
    if (c.term && !has_target) return (w & 0xFC000000u) == 0x14000000u ? kExitUses : kAll;   // tail branch
    return (after & ~writes(c)) | reads(c);
}

// Rewrites the register fields of w that name register `from` (general or SIMD&FP) to `to`.
std::uint32_t rewrite(std::uint32_t w, const Info& c, unsigned from, unsigned to, bool simd) {
    for (unsigned f = 0; f < c.nf; ++f) {
        if (((c.fv >> f) & 1) != simd) continue;
        const unsigned shift = c.fs[f];
        if (((w >> shift) & 31) == from) w = (w & ~(31u << shift)) | (to << shift);
    }
    return w;
}

// Renames values that live only inside a block of n instructions out of the registers in
// kRenamed when another free register breaks an earlier use. live_out: registers live after the
// block; target: branch target of each instruction (-1: none).
void rename_block(std::uint32_t* code, Info* info, const int* target, std::size_t n, Mask live_out, std::vector<Mask>& after) {
    // Only a temporary written again after an earlier use can gain.
    Mask seen = 0;
    bool any = false;
    for (std::size_t j = 0; j < n && !any; ++j) {
        any = (writes(info[j]) & kRenamed & ~reads(info[j]) & seen) != 0;
        seen |= reads(info[j]) | writes(info[j]);
    }
    if (!any) return;
    after.resize(n);
    Mask live = live_out;
    for (std::size_t k = n; k-- > 0;) {
        after[k] = live;
        live = live_before(code[k], info[k], live, target[k] >= 0);
    }
    std::array<int, 64> last_ref;
    last_ref.fill(-1);
    for (std::size_t j = 0; j < n; ++j) {
        const Mask defs = writes(info[j]) & kRenamed & ~reads(info[j]);
        for (Mask m = defs; m;) {
            const unsigned r = static_cast<unsigned>(__builtin_ctzll(m));
            m &= m - 1;
            const Mask bit = 1ull << r;
            if (last_ref[r] < 0) continue;   // nothing earlier to break away from
            // The value's lifetime: through readers and read-modify-writes up to the next
            // instruction that only writes r.
            std::size_t last = j, k = j + 1;
            for (; k < n; ++k) {
                const Mask rd = reads(info[k]), wr = writes(info[k]);
                if (rd & bit) last = k;
                else if (wr & bit) break;
            }
            if (last == j || (k == n && (live_out & bit))) continue;
            if (info[j].barrier) continue;
            Mask busy = 0, used_after = 0;
            for (std::size_t q = j; q <= last; ++q) busy |= after[q];
            for (std::size_t q = j + 1; q <= last; ++q) busy |= reads(info[q]) | writes(info[q]);
            busy |= writes(info[j]);
            for (std::size_t q = last + 1; q < n && q < last + 64; ++q) used_after |= reads(info[q]) | writes(info[q]);
            const bool simd = r >= 32;
            const Mask pool = kFree & (simd ? range(32, 63) : range(0, 30)) & ~busy;
            int best = -1;
            for (Mask c = pool; c;) {
                const unsigned t = static_cast<unsigned>(__builtin_ctzll(c));
                c &= c - 1;
                if (last_ref[t] >= last_ref[r]) continue;
                if (best < 0) { best = static_cast<int>(t); continue; }
                const bool tu = (used_after >> t) & 1, bu = (used_after >> best) & 1;
                if (last_ref[t] < last_ref[best] || (last_ref[t] == last_ref[best] && !tu && bu)) best = static_cast<int>(t);
            }
            if (best < 0) continue;
            const unsigned from = simd ? r - 32 : r, to = simd ? static_cast<unsigned>(best) - 32 : static_cast<unsigned>(best);
            const Mask nbit = 1ull << best;
            code[j] = rewrite(code[j], info[j], from, to, simd);
            info[j] = decode(code[j]);
            for (std::size_t q = j + 1; q <= last; ++q) {
                code[q] = rewrite(code[q], info[q], from, to, simd);
                info[q] = decode(code[q]);
            }
            for (std::size_t q = j; q < last; ++q) after[q] = (after[q] & ~bit) | nbit;
        }
        for (Mask m = reads(info[j]) | writes(info[j]); m;) {
            const unsigned r = static_cast<unsigned>(__builtin_ctzll(m));
            m &= m - 1;
            last_ref[r] = static_cast<int>(j);
        }
    }
}

} // namespace

std::vector<std::uint32_t> schedule_a53(std::vector<std::uint32_t>& code, std::size_t end, const std::vector<bool>& leader,
                                        const std::vector<int>& target) {
    std::vector<std::uint32_t> moved(end);
    for (std::size_t k = 0; k < end; ++k) moved[k] = static_cast<std::uint32_t>(k);
    std::vector<Info> info(end);
    for (std::size_t k = 0; k < end; ++k) info[k] = decode(code[k]);

    // Blocks and their liveness, then renaming within each block.
    std::vector<std::size_t> start;
    std::vector<std::uint32_t> block_of(end + 1, 0);
    for (std::size_t k = 0; k < end; ++k) {
        if (k == 0 || leader[k] || info[k - 1].term || info[k - 1].barrier || info[k].barrier) start.push_back(k);
        block_of[k] = static_cast<std::uint32_t>(start.size() - 1);
    }
    const std::size_t blocks = start.size();
    start.push_back(end);
    // Each block's transfer is live_in = (live_out & ~kill) | gen.
    std::vector<Mask> kill(blocks, 0), gen(blocks, 0), live_in(blocks, 0), live_out(blocks, 0);
    for (std::size_t b = 0; b < blocks; ++b) {
        Mask k0 = 0, g0 = 0;
        for (std::size_t k = start[b + 1]; k-- > start[b];) {
            // live_before is (after & ~K) | G for each instruction; K and G follow from two probes.
            const Mask g = live_before(code[k], info[k], 0, target[k] >= 0);
            const Mask keep = live_before(code[k], info[k], kAll, target[k] >= 0) & ~g;
            k0 |= ~keep & kAll;
            g0 = (g0 & keep) | g;
        }
        kill[b] = k0;
        gen[b] = g0;
    }
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t b = blocks; b-- > 0;) {
            const std::size_t last = start[b + 1] - 1;
            Mask out = 0;
            if ((!info[last].term || falls_through(code[last])) && start[b + 1] < end) out |= live_in[b + 1];
            if (target[last] >= 0 && static_cast<std::size_t>(target[last]) < end) out |= live_in[block_of[static_cast<std::size_t>(target[last])]];
            else if (target[last] >= 0) out = kAll;
            live_out[b] = out;
            const Mask live = (out & ~kill[b]) | gen[b];
            if (live != live_in[b]) {
                live_in[b] = live;
                changed = true;
            }
        }
    }
    std::vector<Mask> after;
    for (std::size_t b = 0; b < blocks; ++b)
        rename_block(code.data() + start[b], info.data() + start[b], target.data() + start[b], start[b + 1] - start[b], live_out[b], after);
    State st;
    st.clear();
    Block block;
    std::vector<std::uint32_t> order, words;
    std::vector<Info> saved;
    std::size_t s = 0;
    while (s < end) {
        if (info[s].barrier) {
            st.clear();
            ++s;
            continue;
        }
        std::size_t e = s + 1;
        while (e < end && e - s < kWindow && !info[e - 1].term && !leader[e] && !info[e].barrier) ++e;
        block.run(info.data() + s, e - s, st, order);
        bool same = true;
        for (std::size_t k = 0; k < order.size() && same; ++k) same = order[k] == k;
        if (!same) {
            words.assign(code.begin() + static_cast<std::ptrdiff_t>(s), code.begin() + static_cast<std::ptrdiff_t>(e));
            saved.assign(info.begin() + static_cast<std::ptrdiff_t>(s), info.begin() + static_cast<std::ptrdiff_t>(e));
            for (std::size_t k = 0; k < order.size(); ++k) {
                code[s + k] = words[order[k]];
                info[s + k] = saved[order[k]];
                moved[s + order[k]] = static_cast<std::uint32_t>(s + k);
            }
        }
        const std::uint32_t last = code[e - 1];
        if (info[e - 1].term && (!falls_through(last) || is_call(last))) st.clear();
        s = e;
    }
    return moved;
}

} // namespace a64

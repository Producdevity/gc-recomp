// Differential test of the AArch64 backend against the C path: deterministic random Gekko
// functions (straight-line code, forward branches, counted loops, conditional returns and
// calls between functions) run with random register and memory states through both
// translations; the resulting gc_ctx and memory must match bit for bit (outside exact mode,
// the context's r5-r10 and f3-f8 after the outermost return are not compared).
//
// Register discipline: r1 stack, r10 data base, r11 small index; r9 is set from r10 at
// function entry and after calls, and only moved by update-form accesses. Other registers are
// random, except that after a call r5-r8 and f3-f8 are written before they are read: a callee
// may change them without returning them (EABI), and translated calls leave them undefined.
#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace gcbef {

using W = std::uint32_t;

constexpr W kText = 0x80003100u;
constexpr W kData = 0x80100000u;      // r10 = kData + 0x1000
constexpr W kDataSize = 0x4000u;
constexpr W kStack = 0x80180000u;     // r1
constexpr W kStackLow = kStack - 0x1000u;

struct Function {
    W address = 0;
    bool leaf = true;
    std::vector<W> code;
};

class Gen {
public:
    explicit Gen(std::uint32_t seed) : rng_(seed) {}

    std::vector<Function> program(unsigned leaves, unsigned others) {
        std::vector<Function> fns;
        W at = kText;
        for (unsigned n = 0; n < leaves + others; ++n) {
            Function f;
            f.address = at;
            f.leaf = n < leaves;
            leaf_addresses_.clear();
            for (unsigned k = 0; k < std::min(n, leaves); ++k) leaf_addresses_.push_back(fns[k].address);
            f.code = f.leaf ? leaf() : nonleaf(at);
            at += static_cast<W>(f.code.size() * 4);
            fns.push_back(f);
        }
        return fns;
    }

private:
    std::mt19937 rng_;
    std::vector<W> leaf_addresses_;

    unsigned pick(unsigned n) { return static_cast<unsigned>(rng_() % n); }
    bool chance(unsigned percent) { return pick(100) < percent; }
    W dreg() {
        static const W regs[] = {0, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
        return regs[pick(sizeof regs / sizeof regs[0])];
    }
    W sreg() { return chance(10) ? 11 : dreg(); }
    W freg() { return pick(32); }
    W crb() { return pick(32); }
    std::int32_t simm() {
        switch (pick(4)) {
        case 0: return static_cast<std::int32_t>(pick(16)) - 8;
        case 1: return static_cast<std::int16_t>(rng_());
        case 2: return chance(50) ? 0x7FFF : -0x8000;
        default: return static_cast<std::int32_t>(pick(4096));
        }
    }

    static W d(W op, W rt, W ra, std::int32_t imm) { return (op << 26) | (rt << 21) | (ra << 16) | (W(imm) & 0xFFFF); }
    static W x(W rs, W ra, W rb, W xo, W rc = 0) { return (31u << 26) | (rs << 21) | (ra << 16) | (rb << 11) | (xo << 1) | rc; }
    static W xo(W rt, W ra, W rb, W xo, W oe, W rc) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (oe << 10) | (xo << 1) | rc; }
    static W m(W op, W rs, W ra, W sh, W mb, W me, W rc) { return (op << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1) | rc; }
    static W a(W op, W d, W fa, W fb, W fc, W xo, W rc = 0) { return (op << 26) | (d << 21) | (fa << 16) | (fb << 11) | (fc << 6) | (xo << 1) | rc; }
    static W xl(W bt, W ba, W bb, W xo) { return (19u << 26) | (bt << 21) | (ba << 16) | (bb << 11) | (xo << 1); }
    static W bc(W bo, W bi, std::int32_t off, W lk = 0) { return (16u << 26) | (bo << 21) | (bi << 16) | (W(off) & 0xFFFC) | lk; }
    static W spr(W op, W rs, W n) { return x(rs, n & 31, n >> 5, op); }

    // A memory operand (base, displacement) inside the data buffer or the stack area.
    void mem(W& base, std::int32_t& disp, unsigned align, bool leaf_stack) {
        unsigned which = pick(3);
        if (which == 0) { base = 10; disp = static_cast<std::int32_t>(pick(0x400) & ~(align - 1)) - 0x200; }
        else if (which == 1) { base = 9; disp = static_cast<std::int32_t>(pick(0x100) & ~(align - 1)) - 0x80; }
        else { base = 1; disp = leaf_stack ? -static_cast<std::int32_t>(8 + (pick(0x80) & ~(align - 1))) : static_cast<std::int32_t>(8 + (pick(0x30) & ~(align - 1))); }
    }

    W one(bool leaf) {
        W rd = dreg(), ra = sreg(), rb = sreg(), rc = chance(30);
        W oe = chance(20);
        W base;
        std::int32_t disp;
        switch (pick(48)) {
        case 0: return d(14, rd, ra, simm());                        // addi
        case 1: return d(15, rd, ra, simm());                        // addis
        case 2: return d(12 + pick(2), rd, ra, simm());              // addic(.)
        case 3: return d(8, rd, ra, simm());                         // subfic
        case 4: return d(7, rd, ra, simm());                         // mulli
        case 5: return d(11, pick(8) << 2, ra, simm());              // cmpwi
        case 6: return d(10, pick(8) << 2, ra, simm() & 0xFFFF);     // cmplwi
        case 7: return x(pick(8) << 2, ra, rb, chance(50) ? 0 : 32); // cmpw/cmplw
        case 8: return d(24 + pick(6), ra, dreg(), simm());          // ori oris xori xoris andi. andis.
        case 9: return m(21, ra, dreg(), pick(32), pick(32), pick(32), rc);   // rlwinm
        case 10: return m(20, ra, dreg(), pick(32), pick(32), pick(32), rc);  // rlwimi
        case 11: return m(23, ra, dreg(), rb, pick(32), pick(32), rc);        // rlwnm
        case 12: {
            static const W ops[] = {28, 60, 444, 412, 316, 476, 124, 284, 954, 922, 26, 24, 536, 792};
            return x(ra, dreg(), rb, ops[pick(sizeof ops / sizeof ops[0])], rc);
        }
        case 13: return x(ra, dreg(), pick(32), 824, rc);            // srawi
        case 14: {
            static const W ops[] = {266, 10, 138, 202, 234, 40, 8, 136, 200, 232, 104, 235, 75, 11, 491, 459};
            W op = ops[pick(sizeof ops / sizeof ops[0])];
            bool no_oe = op == 75 || op == 11;
            return xo(rd, ra, rb, op, no_oe ? 0 : oe, rc);
        }
        case 15: case 16: case 17: {                                  // integer loads/stores, D-form
            static const W ops[] = {32, 33, 34, 35, 40, 41, 42, 43, 36, 37, 38, 39, 44, 45};
            W op = ops[pick(sizeof ops / sizeof ops[0])];
            bool update = op & 1;
            mem(base, disp, op == 32 || op == 33 || op == 36 || op == 37 ? 4 : 2, leaf);
            if (update) base = 9;
            if (update) disp = static_cast<std::int32_t>(pick(0x40) & ~3u) - 0x20;
            W r = (op >= 36 && op <= 39) || op >= 44 ? sreg() : rd;
            return d(op, r, base, disp);
        }
        case 18: case 19: {                                           // indexed loads/stores off r10 + r11
            static const W ops[] = {23, 87, 279, 343, 151, 215, 407, 534, 662, 790, 918, 55, 119, 311, 375, 183, 247, 439};
            W op = ops[pick(sizeof ops / sizeof ops[0])];
            bool update = op == 55 || op == 119 || op == 311 || op == 375 || op == 183 || op == 247 || op == 439;
            bool store = op == 151 || op == 215 || op == 407 || op == 662 || op == 918 || op == 183 || op == 247 || op == 439;
            return x(store ? sreg() : rd, update ? 9 : 10, 11, op);
        }
        case 20: {                                                    // lmw/stmw
            W r = 14 + pick(18);
            mem(base, disp, 4, leaf);
            if (base == 1 && !leaf) disp = 8;
            if (base == 1 && leaf) disp = -0x100;
            return d(chance(50) ? 46 : 47, r, base, disp);
        }
        case 21: {                                                    // lswi/stswi
            W nb = 1 + pick(12);
            return x(chance(50) ? 14 + pick(10) : 12, 10, nb, chance(50) ? 597 : 725);
        }
        case 22: return xl(pick(8) << 2, pick(8) << 2, 0, 0);         // mcrf
        case 23: {
            static const W ops[] = {257, 129, 449, 417, 193, 225, 33, 289};
            return xl(crb(), crb(), crb(), ops[pick(8)]);
        }
        case 24: return x(rd, 0, 0, 19);                               // mfcr
        case 25: return (31u << 26) | (sreg() << 21) | (pick(256) << 12) | (144u << 1);   // mtcrf
        case 26: return x(pick(8) << 2, 0, 0, 512);                    // mcrxr
        case 27: {                                                     // mfspr/mtspr XER LR CTR GQR
            static const W sprs[] = {1, 8, 9, 913, 914, 915, 916, 917, 918, 919};
            W n = sprs[pick(sizeof sprs / sizeof sprs[0])];
            if (chance(50)) return spr(339, rd, n);
            if (n == 9 || (n == 8 && !leaf)) return spr(339, rd, n);   // CTR counts the loops
            return spr(467, sreg(), n);
        }
        case 28: case 29: case 30: {                                   // fp arithmetic
            static const W ops[] = {18, 20, 21, 25, 28, 29, 30, 31, 23, 26};
            W op = ops[pick(sizeof ops / sizeof ops[0])];
            W prim = op == 23 || op == 26 ? 63 : (chance(50) ? 59 : 63);
            if (op == 26) return a(63, freg(), 0, freg(), 0, 26, rc);
            return a(prim, freg(), freg(), freg(), freg(), op, rc);
        }
        case 31: return a(59, freg(), 0, freg(), 0, 24, rc);           // fres
        case 32: {                                                     // fneg fabs fnabs fmr frsp fctiw fctiwz
            static const W ops[] = {40, 264, 136, 72, 12, 14, 15};
            return x(freg(), 0, freg(), ops[pick(7)], rc) | (63u << 26 ^ 31u << 26);
        }
        case 33: return (63u << 26) | ((pick(8) << 2) << 21) | (freg() << 16) | (freg() << 11) | ((chance(50) ? 0 : 32) << 1);   // fcmpu/fcmpo
        case 34: {                                                     // fpscr ops
            switch (pick(5)) {
            case 0: return (63u << 26) | (freg() << 21) | (583u << 1);                     // mffs
            case 1: return (63u << 26) | (pick(256) << 17) | (freg() << 11) | (711u << 1);  // mtfsf
            case 2: return (63u << 26) | ((pick(8) << 2) << 21) | (pick(16) << 12) | (134u << 1);   // mtfsfi
            case 3: return (63u << 26) | ((1 + pick(30)) << 21) | ((chance(50) ? 70u : 38u) << 1);  // mtfsb0/1
            default: return (63u << 26) | ((pick(8) << 2) << 21) | ((pick(8) << 2) << 16) | (64u << 1);   // mcrfs
            }
        }
        case 35: case 36: {                                            // fp loads/stores
            static const W ops[] = {48, 49, 50, 51, 52, 53, 54, 55};
            W op = ops[pick(8)];
            bool update = op & 1;
            mem(base, disp, op >= 50 && op <= 51 ? 8 : (op >= 54 ? 8 : 4), leaf);
            if (update) { base = 9; disp = static_cast<std::int32_t>(pick(0x40) & ~7u) - 0x20; }
            return d(op, freg(), base, disp);
        }
        case 37: {                                                     // fp indexed
            static const W ops[] = {535, 567, 599, 631, 663, 695, 727, 759, 983};
            W op = ops[pick(9)];
            bool update = op == 567 || op == 631 || op == 695 || op == 759;
            return x(freg(), update ? 9 : 10, 11, op);
        }
        case 38: case 39: case 40: {                                   // paired singles (A-form)
            static const W ops[] = {10, 11, 12, 13, 14, 15, 18, 20, 21, 23, 24, 25, 26, 28, 29, 30, 31};
            return a(4, freg(), freg(), freg(), freg(), ops[pick(sizeof ops / sizeof ops[0])], rc);
        }
        case 41: {                                                     // paired singles (X-form)
            static const W ops[] = {0, 32, 40, 64, 72, 96, 136, 264, 528, 560, 592, 624};
            W op = ops[pick(sizeof ops / sizeof ops[0])];
            if (op == 0 || op == 32 || op == 64 || op == 96) return (4u << 26) | ((pick(8) << 2) << 21) | (freg() << 16) | (freg() << 11) | (op << 1);
            return (4u << 26) | (freg() << 21) | (freg() << 16) | (freg() << 11) | (op << 1) | rc;
        }
        case 42: case 43: {                                            // psq_l/psq_st (+u)
            W op = 56 + pick(2) + (chance(50) ? 4 : 0);
            bool update = op == 57 || op == 61;
            W b = update ? 9 : 10;
            std::int32_t o = static_cast<std::int32_t>(pick(0x100) & ~7u) - 0x80;
            return (op << 26) | (freg() << 21) | (b << 16) | (pick(2) << 15) | (pick(8) << 12) | (W(o) & 0xFFF);
        }
        case 44: {                                                     // psq_lx/stx (+ux)
            static const W ops[] = {6, 7, 38, 39};
            W op = ops[pick(4)];
            bool update = op >= 38;
            return (4u << 26) | (freg() << 21) | ((update ? 9u : 10u) << 16) | (11u << 11) | (pick(2) << 10) | (pick(8) << 7) | (op << 1);
        }
        case 45: return x(0, 10, 11, 1014);                             // dcbz
        case 46: return chance(50) ? x(rd, 10, 11, 20) : x(sreg(), 10, 11, 150, 1);   // lwarx / stwcx.
        default: return x(0, 0, 0, 598);                                // sync
        }
    }

    // Straight-line block with optional structure (branches, loops, conditional returns, calls).
    void body(std::vector<W>& c, bool leaf, unsigned length) {
        for (unsigned n = 0; n < length; ++n) {
            unsigned kind = pick(100);
            if (kind < 8) {                    // compare + branch over 1-2 instructions
                W crf = pick(8);
                c.push_back(d(11, crf << 2, sreg(), simm()));
                unsigned skip = 1 + pick(2);
                c.push_back(bc(chance(50) ? 12 : 4, crf * 4 + pick(3), static_cast<std::int32_t>(4 * (skip + 1))));
                for (unsigned k = 0; k < skip; ++k) c.push_back(one(leaf));
            } else if (kind < 11) {            // branch on a random CR bit / always
                unsigned skip = 1 + pick(2);
                static const W bos[] = {12, 4, 20};
                c.push_back(bc(bos[pick(3)], crb(), static_cast<std::int32_t>(4 * (skip + 1))));
                for (unsigned k = 0; k < skip; ++k) c.push_back(one(leaf));
            } else if (kind < 14) {            // counted loop
                W r = 12 + pick(3);
                c.push_back(d(14, r, 0, static_cast<std::int32_t>(1 + pick(4))));
                c.push_back(spr(467, r, 9));
                unsigned len = 1 + pick(3);
                for (unsigned k = 0; k < len; ++k) c.push_back(one(leaf));
                c.push_back(bc(16, 0, -static_cast<std::int32_t>(4 * len)));
            } else if (kind < 16 && leaf) {    // conditional return
                c.push_back(xl(chance(50) ? 12 : 4, crb(), 0, 16));
            } else if (kind < 22 && !leaf && !leaf_addresses_.empty()) {   // call a leaf
                W target = leaf_addresses_[pick(static_cast<unsigned>(leaf_addresses_.size()))];
                W here = call_base_ + static_cast<W>(c.size() * 4);
                c.push_back((18u << 26) | ((target - here) & 0x03FFFFFC) | 1);
                c.push_back(d(14, 9, 10, 0x800));
                for (W r = 5; r <= 8; ++r) c.push_back(d(14, r, 0, simm()));
                for (W f = 3; f <= 8; ++f) c.push_back((4u << 26) | (f << 21) | ((14 + pick(18)) << 11) | (72u << 1));   // ps_mr
            } else if (kind < 27) {            // store, then a load at or next to its address
                W base;
                std::int32_t disp;
                mem(base, disp, 8, leaf);
                std::int32_t near = disp + static_cast<std::int32_t>(pick(9)) - 4;
                if (chance(70)) {
                    static const W stores[] = {36, 44, 38}, loads[] = {32, 40, 42, 34};   // stw sth stb / lwz lhz lha lbz
                    c.push_back(d(stores[pick(3)], sreg(), base, disp));
                    c.push_back(one(leaf));
                    c.push_back(d(loads[pick(4)], dreg(), base, near));
                } else {
                    c.push_back(d(chance(50) ? 52 : 54, freg(), base, disp));          // stfs / stfd
                    c.push_back(one(leaf));
                    c.push_back(d(chance(50) ? 48 : 50, freg(), base, near & ~3));     // lfs / lfd
                }
            } else {
                c.push_back(one(leaf));
            }
        }
    }

    std::vector<W> leaf() {
        std::vector<W> c;
        c.push_back(d(14, 9, 10, 0x800));      // r9 = r10 + 0x800
        body(c, true, 6 + pick(20));
        c.push_back(0x4E800020u);              // blr
        return c;
    }

    W call_base_ = 0;

    std::vector<W> nonleaf(W at) {
        std::vector<W> c;
        call_base_ = at;
        c.push_back(spr(339, 0, 8));           // mflr r0
        c.push_back(d(36, 0, 1, 4));           // stw r0, 4(r1)
        c.push_back(d(37, 1, 1, -64));         // stwu r1, -64(r1)
        c.push_back(d(14, 9, 10, 0x800));
        body(c, false, 8 + pick(24));
        c.push_back(d(32, 0, 1, 68));          // lwz r0, 68(r1)
        c.push_back(d(14, 1, 1, 64));          // addi r1, r1, 64
        c.push_back(spr(467, 0, 8));           // mtlr r0
        c.push_back(0x4E800020u);
        return c;
    }
};

inline std::vector<Function> program(std::uint32_t seed) { return Gen(seed).program(120, 280); }

} // namespace gcbef

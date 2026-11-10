// Instruction-level tests: small Gekko routines with inputs and results worked out from
// the PowerPC 750CL user's manual (Book I arithmetic, XER[CA], CR updates, rounding,
// paired singles and quantization), independent of the recompiler's own code.
// Shared by the image generator and the runner; routines are laid out in order.
#pragma once

#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace ppct {

using W = std::uint32_t;

// Encoders.
inline W d(W op, W rt, W ra, std::int32_t imm) { return (op << 26) | (rt << 21) | (ra << 16) | (W(imm) & 0xFFFF); }
inline W x(W rs, W ra, W rb, W xo, W rc = 0) { return (31u << 26) | (rs << 21) | (ra << 16) | (rb << 11) | (xo << 1) | rc; }
inline W xo(W rt, W ra, W rb, W xo, W oe = 0, W rc = 0) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (oe << 10) | (xo << 1) | rc; }
inline W rlwinm(W ra, W rs, W sh, W mb, W me, W rc = 0) { return (21u << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1) | rc; }
inline W rlwimi(W ra, W rs, W sh, W mb, W me) { return (20u << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1); }
inline W li(W rt, std::int32_t v) { return d(14, rt, 0, v); }
inline W addi(W rt, W ra, std::int32_t v) { return d(14, rt, ra, v); }
inline W addis(W rt, W ra, std::int32_t v) { return d(15, rt, ra, v); }
inline W addic(W rt, W ra, std::int32_t v) { return d(12, rt, ra, v); }
inline W subfic(W rt, W ra, std::int32_t v) { return d(8, rt, ra, v); }
inline W ori(W ra, W rs, W v) { return d(24, rs, ra, std::int32_t(v)); }
inline W cmpwi(W crf, W ra, std::int32_t v) { return d(11, crf << 2, ra, v); }
inline W cmplwi(W crf, W ra, W v) { return d(10, crf << 2, ra, std::int32_t(v)); }
inline W cmpw(W crf, W ra, W rb) { return x(crf << 2, ra, rb, 0); }
inline W cmplw(W crf, W ra, W rb) { return x(crf << 2, ra, rb, 32); }
inline W lwz(W rt, W ra, std::int32_t o) { return d(32, rt, ra, o); }
inline W stw(W rs, W ra, std::int32_t o) { return d(36, rs, ra, o); }
inline W lha(W rt, W ra, std::int32_t o) { return d(42, rt, ra, o); }
inline W lmw(W rt, W ra, std::int32_t o) { return d(46, rt, ra, o); }
inline W stmw(W rs, W ra, std::int32_t o) { return d(47, rs, ra, o); }
inline W lfs(W ft, W ra, std::int32_t o) { return d(48, ft, ra, o); }
inline W lfd(W ft, W ra, std::int32_t o) { return d(50, ft, ra, o); }
inline W stfs(W fs, W ra, std::int32_t o) { return d(52, fs, ra, o); }
inline W stfd(W fs, W ra, std::int32_t o) { return d(54, fs, ra, o); }
inline W mtspr(W spr, W rs) { return x(rs, spr & 31, spr >> 5, 467); }
inline W mfspr(W rt, W spr) { return x(rt, spr & 31, spr >> 5, 339); }
inline W mtctr(W rs) { return mtspr(9, rs); }
inline W mflr(W rt) { return mfspr(rt, 8); }
inline W mtlr(W rs) { return mtspr(8, rs); }
inline W bc(W bo, W bi, std::int32_t offset, W lk = 0) { return (16u << 26) | (bo << 21) | (bi << 16) | (W(offset) & 0xFFFC) | lk; }
inline W b(std::int32_t offset, W lk = 0) { return (18u << 26) | (W(offset) & 0x03FFFFFC) | lk; }
inline W bdnz(std::int32_t offset) { return bc(16, 0, offset); }
constexpr W blr = 0x4E800020u, bctr = 0x4E800420u, bctrl = 0x4E800421u;
inline W fa(W op, W frd, W fra, W frb, W frc, W xo, W rc = 0) {
    return (op << 26) | (frd << 21) | (fra << 16) | (frb << 11) | (frc << 6) | (xo << 1) | rc;
}
inline W fx(W frd, W fra, W frb, W xo) { return (63u << 26) | (frd << 21) | (fra << 16) | (frb << 11) | (xo << 1); }
inline W ps(W frd, W fra, W frb, W frc, W xo5) { return (4u << 26) | (frd << 21) | (fra << 16) | (frb << 11) | (frc << 6) | (xo5 << 1); }
inline W psx(W frd, W fra, W frb, W xo10) { return (4u << 26) | (frd << 21) | (fra << 16) | (frb << 11) | (xo10 << 1); }
inline W psq_l(W frd, W ra, W w, W i, std::int32_t o) { return (56u << 26) | (frd << 21) | (ra << 16) | (w << 15) | (i << 12) | (W(o) & 0xFFF); }
inline W psq_st(W frs, W ra, W w, W i, std::int32_t o) { return (60u << 26) | (frs << 21) | (ra << 16) | (w << 15) | (i << 12) | (W(o) & 0xFFF); }

// Results to check: GPRs, CR fields (LT GT EQ SO as a 4-bit value), XER[CA], FPR/PS1 bits.
// The tests are translated in exact mode: returns write back every register, so results
// in EABI scratch registers stay visible.
struct Check {
    enum Kind { gpr, cr, ca, fpr, ps1, mem32 } kind;
    unsigned index;
    std::uint64_t value;
};
struct Test {
    std::string name;
    std::vector<W> code;
    std::vector<std::pair<unsigned, W>> gpr_in;
    std::vector<std::pair<unsigned, double>> fpr_in;
    std::vector<std::pair<unsigned, W>> gqr_in;
    std::vector<Check> checks;
};

// Data area the tests address through r10.
constexpr W kData = 0x80100000u;

// Data area contents: 1.5f at +0x40, quantized bytes at +0x50, jump table at +0x80.
inline std::vector<std::uint8_t> data_image(W case0, W case1) {
    std::vector<std::uint8_t> d(0x100, 0);
    auto put = [&](unsigned at, W v) { d[at] = v >> 24; d[at + 1] = v >> 16; d[at + 2] = v >> 8; d[at + 3] = v; };
    put(0x40, 0x3FC00000);
    d[0x50] = 0x20;
    d[0x51] = 0xF0;
    put(0x80, case0);
    put(0x84, case1);
    return d;
}

inline std::uint64_t bits(double v) { std::uint64_t u; __builtin_memcpy(&u, &v, 8); return u; }

inline std::vector<Test> tests() {
    std::vector<Test> t;
    // Carry chain: 64-bit add (r3:r4) + (r5:r6).
    t.push_back({"add64", {xo(8, 4, 6, 10), xo(7, 3, 5, 138), blr},
                 {{3, 0x00000001}, {4, 0xFFFFFFFF}, {5, 0x00000002}, {6, 0x00000001}},
                 {}, {}, {{Check::gpr, 7, 0x00000004}, {Check::gpr, 8, 0x00000000}, {Check::ca, 0, 0}}});
    // 64-bit subtract (r3:r4) - (r5:r6) = (1:0) - (0:1).
    t.push_back({"sub64", {xo(8, 6, 4, 8), xo(7, 5, 3, 136), blr},
                 {{3, 1}, {4, 0}, {5, 0}, {6, 1}}, {}, {},
                 {{Check::gpr, 7, 0}, {Check::gpr, 8, 0xFFFFFFFF}, {Check::ca, 0, 1}}});
    // addic carry out and subfic.
    t.push_back({"addic_subfic", {addic(4, 3, 1), subfic(5, 3, 0), blr}, {{3, 0xFFFFFFFF}}, {}, {},
                 {{Check::gpr, 4, 0}, {Check::gpr, 5, 1}, {Check::ca, 0, 0}}});
    // Signed division by two with srawi + addze: -5 / 2 = -2.
    t.push_back({"srawi_addze", {x(3, 4, 1, 824), xo(5, 4, 0, 202), blr}, {{3, 0xFFFFFFFB}}, {}, {},
                 {{Check::gpr, 4, 0xFFFFFFFD}, {Check::gpr, 5, 0xFFFFFFFE}, {Check::ca, 0, 0}}});
    // sraw by 40 sign-fills; srw by 32 clears; slw by 31.
    t.push_back({"shifts", {x(3, 4, 5, 792), x(3, 6, 7, 536), x(8, 9, 11, 24), blr},
                 {{3, 0x80000001}, {5, 40}, {7, 32}, {8, 3}, {11, 31}}, {}, {},
                 {{Check::gpr, 4, 0xFFFFFFFF}, {Check::ca, 0, 1}, {Check::gpr, 6, 0}, {Check::gpr, 9, 0x80000000}}});
    // Rotates and masks.
    t.push_back({"rotate", {rlwinm(4, 3, 8, 24, 31), rlwinm(5, 3, 0, 28, 3), rlwimi(6, 3, 16, 0, 15), blr},
                 {{3, 0x12345678}, {6, 0xAAAABBBB}}, {}, {},
                 {{Check::gpr, 4, 0x12}, {Check::gpr, 5, 0x10000008}, {Check::gpr, 6, 0x5678BBBB}}});
    // cntlzw, extsb, extsh.
    t.push_back({"count_extend", {x(3, 4, 0, 26), x(5, 6, 0, 26), x(7, 8, 0, 954), x(7, 9, 0, 922), blr},
                 {{3, 0x00010000}, {5, 0}, {7, 0x000000F0}}, {}, {},
                 {{Check::gpr, 4, 15}, {Check::gpr, 6, 32}, {Check::gpr, 8, 0xFFFFFFF0}, {Check::gpr, 9, 0x000000F0}}});
    // High multiplies and division.
    t.push_back({"mul_div", {xo(5, 3, 4, 11), xo(6, 3, 4, 75), xo(7, 8, 9, 491), xo(10, 11, 9, 459), xo(12, 3, 4, 235), blr},
                 {{3, 0x80000000}, {4, 2}, {8, 0xFFFFFFF9}, {9, 2}, {11, 7}}, {}, {},
                 {{Check::gpr, 5, 1}, {Check::gpr, 6, 0xFFFFFFFF}, {Check::gpr, 7, 0xFFFFFFFD}, {Check::gpr, 10, 3},
                  {Check::gpr, 12, 0}}});
    // Compares into CR fields; record form of add sets cr0 from the signed result.
    t.push_back({"compare", {cmpwi(1, 3, 0), cmplwi(5, 3, 0), cmpw(6, 3, 3), xo(5, 3, 4, 266, 0, 1), blr},
                 {{3, 0xFFFFFFFF}, {4, 0}}, {}, {},
                 {{Check::cr, 1, 0x8}, {Check::cr, 5, 0x4}, {Check::cr, 6, 0x2}, {Check::cr, 0, 0x8}}});
    // Counted loop: r3 = 10 * 3 with mtctr/bdnz.
    t.push_back({"ctr_loop", {li(3, 0), li(4, 10), mtctr(4), addi(3, 3, 3), bdnz(-4), blr}, {}, {}, {},
                 {{Check::gpr, 3, 30}}});
    // Conditional branch on cr0: r4 = r3 > 5 ? 1 : 2.
    t.push_back({"branch", {cmpwi(0, 3, 5), bc(12, 1, 12), li(4, 2), blr, li(4, 1), blr}, {{3, 9}}, {}, {},
                 {{Check::gpr, 4, 1}}});
    // Store/load multiple and halfword sign extension through memory at r10.
    t.push_back({"memory", {stmw(29, 10, 0), li(29, 0), li(30, 0), lmw(29, 10, 0), li(5, -2), d(44, 5, 10, 16),
                            lha(6, 10, 16), x(7, 10, 0, 662), lwz(8, 10, 0), blr},
                 {{10, kData}, {29, 0x11112222}, {30, 0x33334444}, {31, 0x55556666}, {7, 0x11223344}}, {}, {},
                 {{Check::gpr, 29, 0x11112222}, {Check::gpr, 30, 0x33334444}, {Check::gpr, 6, 0xFFFFFFFE},
                  {Check::mem32, 0, 0x44332211}, {Check::gpr, 8, 0x44332211}}});
    // mfcr / mtcrf round trip and CR logic.
    t.push_back({"cr_logic", {(31u << 26) | (3u << 21) | (0xFFu << 12) | (144u << 1),                 // mtcrf 0xFF,r3
                              (19u << 26) | (5u << 21) | (0u << 16) | (1u << 11) | (449u << 1),   // cror 5,0,1
                              (19u << 26) | (6u << 21) | (6u << 16) | (6u << 11) | (193u << 1),   // crclr 6
                              x(4, 0, 0, 19), blr},
                 {{3, 0x80000000 | 0x00000002}}, {}, {},
                 {{Check::gpr, 4, 0x84000002}}});
    // Indirect call through CTR, return via LR, and LR save around a call.
    t.push_back({"call", {mflr(0), stw(0, 1, 4), d(37, 1, 1, -16), mtctr(3), bctrl, lwz(0, 1, 20), addi(1, 1, 16), mtlr(0),
                          addi(4, 4, 1), blr},
                 {{4, 41}}, {}, {}, {{Check::gpr, 4, 43}}});
    // Callee for "call": r4 += 1.
    t.push_back({"callee", {addi(4, 4, 1), blr}, {}, {}, {}, {}});
    // Floating point: single rounding, fused multiply-add, conversion with saturation.
    t.push_back({"float",
                 {fa(59, 3, 1, 2, 0, 21), fa(63, 4, 1, 2, 0, 21), fa(63, 5, 1, 1, 2, 29), fx(6, 0, 7, 15), fx(8, 0, 9, 15),
                  fa(63, 10, 7, 2, 1, 23), blr},
                 {}, {{1, 1.0}, {2, 9.313225746154785e-10}, {7, -2.7}, {9, 3e9}}, {},
                 {{Check::fpr, 3, bits(1.0)}, {Check::fpr, 4, bits(1.0 + 9.313225746154785e-10)},
                  {Check::fpr, 5, bits(1.0 + 9.313225746154785e-10)}, {Check::fpr, 6, 0xFFF80000FFFFFFFEull},
                  {Check::fpr, 8, 0xFFF800007FFFFFFFull}, {Check::fpr, 10, bits(9.313225746154785e-10)}}});
    // fcmpu with NaN reports unordered.
    t.push_back({"fcmpu_nan", {fx(1u << 2, 1, 2, 0), blr}, {}, {{1, __builtin_nan("")}, {2, 1.0}}, {},
                 {{Check::cr, 1, 0x1}}});
    // Single load fills both halves; paired add and merge.
    t.push_back({"paired",
                 {lfs(1, 10, 0), ps(2, 1, 1, 0, 21), psx(3, 2, 1, 592), ps(4, 3, 1, 1, 14), blr},
                 {{10, kData + 0x40}}, {}, {},
                 {{Check::fpr, 1, bits(1.5)}, {Check::ps1, 1, bits(1.5)}, {Check::fpr, 2, bits(3.0)}, {Check::ps1, 2, bits(3.0)},
                  {Check::fpr, 3, bits(3.0)}, {Check::ps1, 3, bits(1.5)}, {Check::fpr, 4, bits(3.0 * 1.5 + 1.5)},
                  {Check::ps1, 4, bits(1.5 * 1.5 + 1.5)}}});
    // Quantized load (u8, scale 4: value / 16) and store (s16, scale 2: value * 4, saturating).
    t.push_back({"quantized", {psq_l(1, 10, 0, 2, 0x10), psq_st(1, 10, 0, 3, 0x20), lwz(5, 10, 0x20), blr},
                 {{10, kData + 0x40}}, {}, {{2, 0x04040000}, {3, 0x00000207}},
                 {{Check::fpr, 1, bits(2.0)}, {Check::ps1, 1, bits(15.0)}, {Check::gpr, 5, 0x0008003C}}});
    // Jump table: bctr to one of two targets selected by r3 (table in data at r10 + 0x80).
    t.push_back({"switch", {rlwinm(0, 3, 2, 0, 29), x(4, 10, 0, 23), mtctr(4), bctr, li(5, 100), blr, li(5, 200), blr},
                 {{3, 1}, {10, kData + 0x80}}, {}, {}, {{Check::gpr, 5, 200}}});
    return t;
}

} // namespace ppct

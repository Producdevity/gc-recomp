// A64 instruction encoder for the direct AArch64 backend. Encodings follow the Arm
// Architecture Reference Manual for A-profile (A64 base and floating-point instructions).
#pragma once

#include <cstdint>
#include <cstdlib>
#include <vector>

namespace a64 {

enum Cond : unsigned { EQ = 0, NE, HS, LO, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE, AL };
inline Cond invert(Cond c) { return static_cast<Cond>(c ^ 1); }

constexpr unsigned ZR = 31, SP = 31;
constexpr unsigned FP = 29, LR = 30;

// Bitmask immediate for logical instructions; false when the value has no encoding.
inline bool bitmask_imm(std::uint64_t value, unsigned width, unsigned& n, unsigned& immr, unsigned& imms) {
    if (width == 32) value = (value & 0xFFFFFFFFull) | (value << 32);
    if (value == 0 || value == ~0ull) return false;
    unsigned size = 64;
    while (size > 2) {
        unsigned half = size / 2;
        std::uint64_t mask = (1ull << half) - 1;
        if ((value & mask) != ((value >> half) & mask)) break;
        size = half;
    }
    std::uint64_t mask = size == 64 ? ~0ull : (1ull << size) - 1;
    std::uint64_t elem = value & mask;
    unsigned ones = static_cast<unsigned>(__builtin_popcountll(elem));
    std::uint64_t run = ones == 64 ? ~0ull : (1ull << ones) - 1;
    for (unsigned r = 0; r < size; ++r) {
        std::uint64_t rotl = r == 0 ? elem : (((elem << r) | (elem >> (size - r))) & mask);
        if (rotl == run) {
            n = size == 64 ? 1 : 0;
            immr = r;
            imms = ((~(size - 1) << 1) & 0x3F) | (ones - 1);
            return true;
        }
    }
    return false;
}

// 8-bit floating-point immediate (FMOV): value = +/- (16..31)/16 * 2^(-3..4).
inline bool fp_imm8(double v, unsigned& imm8) {
    for (unsigned i = 0; i < 256; ++i) {
        unsigned sign = i >> 7, exp = (i >> 4) & 7, frac = i & 15;
        int e = (exp & 4) ? static_cast<int>(exp) - 8 + 1 : static_cast<int>(exp) + 1;
        double x = (16.0 + frac) / 16.0;
        for (int k = 0; k < e; ++k) x *= 2.0;
        for (int k = 0; k > e; --k) x /= 2.0;
        if (sign) x = -x;
        if (x == v && !(v == 0.0)) {
            imm8 = i;
            return true;
        }
    }
    return false;
}

class Asm {
public:
    std::vector<std::uint32_t> code;

    std::size_t pos() const { return code.size(); }
    void emit(std::uint32_t w) { code.push_back(w); }
    void patch(std::size_t at, std::uint32_t w) { code[at] = w; }

    // ADD/SUB (immediate): imm12, optionally shifted left by 12.
    void addsub_imm(bool x, bool sub, bool flags, unsigned rd, unsigned rn, unsigned imm12, bool shift = false) {
        emit((x ? 1u << 31 : 0) | (sub ? 1u << 30 : 0) | (flags ? 1u << 29 : 0) | 0x11000000u | (shift ? 1u << 22 : 0) |
             (imm12 << 10) | (rn << 5) | rd);
    }
    void add_imm(unsigned rd, unsigned rn, unsigned imm, bool x = false) { addsub_imm(x, false, false, rd, rn, imm); }
    void sub_imm(unsigned rd, unsigned rn, unsigned imm, bool x = false) { addsub_imm(x, true, false, rd, rn, imm); }
    void cmp_imm(unsigned rn, unsigned imm, bool x = false) { addsub_imm(x, true, true, ZR, rn, imm); }
    void cmn_imm(unsigned rn, unsigned imm, bool x = false) { addsub_imm(x, false, true, ZR, rn, imm); }
    // ADD/SUB (shifted register). shift: 0 LSL, 1 LSR, 2 ASR.
    void addsub_reg(bool x, bool sub, bool flags, unsigned rd, unsigned rn, unsigned rm, unsigned shift = 0, unsigned amount = 0) {
        emit((x ? 1u << 31 : 0) | (sub ? 1u << 30 : 0) | (flags ? 1u << 29 : 0) | 0x0B000000u | (shift << 22) | (rm << 16) |
             (amount << 10) | (rn << 5) | rd);
    }
    void add(unsigned rd, unsigned rn, unsigned rm, bool x = false) { addsub_reg(x, false, false, rd, rn, rm); }
    void adds(unsigned rd, unsigned rn, unsigned rm) { addsub_reg(false, false, true, rd, rn, rm); }
    void sub(unsigned rd, unsigned rn, unsigned rm, bool x = false) { addsub_reg(x, true, false, rd, rn, rm); }
    void subs(unsigned rd, unsigned rn, unsigned rm) { addsub_reg(false, true, true, rd, rn, rm); }
    void cmp(unsigned rn, unsigned rm, bool x = false) { addsub_reg(x, true, true, ZR, rn, rm); }
    void neg(unsigned rd, unsigned rm) { addsub_reg(false, true, false, rd, ZR, rm); }
    void negs(unsigned rd, unsigned rm) { addsub_reg(false, true, true, rd, ZR, rm); }
    // ADD Xd, Xn, Wm, UXTW (extended register).
    void add_uxtw(unsigned xd, unsigned xn, unsigned wm) { emit(0x8B204000u | (wm << 16) | (xn << 5) | xd); }
    // CMP Xn, Wm, SXTW
    void cmp_sxtw(unsigned xn, unsigned wm) { emit(0xEB20C000u | (wm << 16) | (xn << 5) | ZR); }
    // ADC/ADCS/SBC/SBCS (32-bit).
    void adc(unsigned rd, unsigned rn, unsigned rm, bool flags) { emit((flags ? 0x3A000000u : 0x1A000000u) | (rm << 16) | (rn << 5) | rd); }
    void sbc(unsigned rd, unsigned rn, unsigned rm, bool flags) { emit((flags ? 0x7A000000u : 0x5A000000u) | (rm << 16) | (rn << 5) | rd); }

    // Logical (shifted register). opc: 0 AND, 1 ORR, 2 EOR, 3 ANDS; inv selects BIC/ORN/EON/BICS.
    void logic_reg(bool x, unsigned opc, bool inv, unsigned rd, unsigned rn, unsigned rm, unsigned shift = 0, unsigned amount = 0) {
        emit((x ? 1u << 31 : 0) | (opc << 29) | 0x0A000000u | (shift << 22) | (inv ? 1u << 21 : 0) | (rm << 16) |
             (amount << 10) | (rn << 5) | rd);
    }
    void and_(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 0, false, rd, rn, rm); }
    void bic(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 0, true, rd, rn, rm); }
    void orr(unsigned rd, unsigned rn, unsigned rm, unsigned shift = 0, unsigned amount = 0) {
        logic_reg(false, 1, false, rd, rn, rm, shift, amount);
    }
    void orn(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 1, true, rd, rn, rm); }
    void eor(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 2, false, rd, rn, rm); }
    void eon(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 2, true, rd, rn, rm); }
    void ands(unsigned rd, unsigned rn, unsigned rm) { logic_reg(false, 3, false, rd, rn, rm); }
    void mvn(unsigned rd, unsigned rm) { logic_reg(false, 1, true, rd, ZR, rm); }
    void mov(unsigned rd, unsigned rm) { logic_reg(false, 1, false, rd, ZR, rm); }
    void movx(unsigned rd, unsigned rm) {
        if (rd == SP || rm == SP) add_imm(rd, rm, 0, true);
        else logic_reg(true, 1, false, rd, ZR, rm);
    }
    // Logical (immediate); false when not encodable.
    bool logic_imm(bool x, unsigned opc, unsigned rd, unsigned rn, std::uint64_t imm) {
        unsigned n, immr, imms;
        if (!bitmask_imm(imm, x ? 64 : 32, n, immr, imms)) return false;
        emit((x ? 1u << 31 : 0) | (opc << 29) | 0x12000000u | (n << 22) | (immr << 16) | (imms << 10) | (rn << 5) | rd);
        return true;
    }

    // Move wide.
    void movz(unsigned rd, unsigned imm16, unsigned hw = 0, bool x = false) { emit((x ? 0xD2800000u : 0x52800000u) | (hw << 21) | (imm16 << 5) | rd); }
    void movn(unsigned rd, unsigned imm16, unsigned hw = 0, bool x = false) { emit((x ? 0x92800000u : 0x12800000u) | (hw << 21) | (imm16 << 5) | rd); }
    void movk(unsigned rd, unsigned imm16, unsigned hw = 0, bool x = false) { emit((x ? 0xF2800000u : 0x72800000u) | (hw << 21) | (imm16 << 5) | rd); }
    // Any 32-bit constant in Wd (zero-extended into Xd).
    void mov32(unsigned rd, std::uint32_t v) {
        std::uint32_t lo = v & 0xFFFF, hi = v >> 16;
        if (hi == 0) { movz(rd, lo); return; }
        if (lo == 0) { movz(rd, hi, 1); return; }
        if (hi == 0xFFFF) { movn(rd, (~v) & 0xFFFF); return; }
        if (lo == 0xFFFF) { movn(rd, (~v >> 16) & 0xFFFF, 1); return; }
        if (logic_imm(false, 1, rd, ZR, v)) return;
        movz(rd, lo);
        movk(rd, hi, 1);
    }
    void mov64(unsigned rd, std::uint64_t v) {
        bool first = true;
        for (unsigned hw = 0; hw < 4; ++hw) {
            unsigned part = (v >> (16 * hw)) & 0xFFFF;
            if (!part) continue;
            if (first) movz(rd, part, hw, true);
            else movk(rd, part, hw, true);
            first = false;
        }
        if (first) movz(rd, 0, 0, true);
    }

    // Bitfield moves. 32-bit unless x.
    void bfm_(unsigned opc, bool x, unsigned rd, unsigned rn, unsigned immr, unsigned imms) {
        emit((x ? 0x80400000u : 0) | (opc << 29) | 0x13000000u | (immr << 16) | (imms << 10) | (rn << 5) | rd);
    }
    void sbfm(unsigned rd, unsigned rn, unsigned immr, unsigned imms, bool x = false) { bfm_(0, x, rd, rn, immr, imms); }
    void bfm(unsigned rd, unsigned rn, unsigned immr, unsigned imms) { bfm_(1, false, rd, rn, immr, imms); }
    void ubfm(unsigned rd, unsigned rn, unsigned immr, unsigned imms, bool x = false) { bfm_(2, x, rd, rn, immr, imms); }
    void lsl_imm(unsigned rd, unsigned rn, unsigned s) { ubfm(rd, rn, (32 - s) & 31, 31 - s); }
    void lsr_imm(unsigned rd, unsigned rn, unsigned s, bool x = false) { ubfm(rd, rn, s, x ? 63 : 31, x); }
    void asr_imm(unsigned rd, unsigned rn, unsigned s) { sbfm(rd, rn, s, 31); }
    void ubfx(unsigned rd, unsigned rn, unsigned lsb, unsigned width) { ubfm(rd, rn, lsb, lsb + width - 1); }
    void ubfiz(unsigned rd, unsigned rn, unsigned lsb, unsigned width) { ubfm(rd, rn, (32 - lsb) & 31, width - 1); }
    void bfi(unsigned rd, unsigned rn, unsigned lsb, unsigned width) { bfm(rd, rn, (32 - lsb) & 31, width - 1); }
    void sxtb(unsigned rd, unsigned rn) { sbfm(rd, rn, 0, 7); }
    void sxth(unsigned rd, unsigned rn) { sbfm(rd, rn, 0, 15); }
    void sxtw(unsigned xd, unsigned wn) { sbfm(xd, wn, 0, 31, true); }
    void ror_imm(unsigned rd, unsigned rn, unsigned s) { emit(0x13800000u | (rn << 16) | (s << 10) | (rn << 5) | rd); }

    // Two-source: UDIV 2, SDIV 3, LSLV 8, LSRV 9, ASRV 10, RORV 11.
    void dp2(bool x, unsigned op, unsigned rd, unsigned rn, unsigned rm) {
        emit((x ? 1u << 31 : 0) | 0x1AC00000u | (rm << 16) | (op << 10) | (rn << 5) | rd);
    }
    void udiv(unsigned rd, unsigned rn, unsigned rm) { dp2(false, 2, rd, rn, rm); }
    void sdiv(unsigned rd, unsigned rn, unsigned rm) { dp2(false, 3, rd, rn, rm); }
    void lslv(unsigned rd, unsigned rn, unsigned rm, bool x = false) { dp2(x, 8, rd, rn, rm); }
    void lsrv(unsigned rd, unsigned rn, unsigned rm, bool x = false) { dp2(x, 9, rd, rn, rm); }
    void asrv(unsigned rd, unsigned rn, unsigned rm, bool x = false) { dp2(x, 10, rd, rn, rm); }
    void rorv(unsigned rd, unsigned rn, unsigned rm) { dp2(false, 11, rd, rn, rm); }
    // One-source.
    void rev16(unsigned rd, unsigned rn) { emit(0x5AC00400u | (rn << 5) | rd); }
    void rev(unsigned rd, unsigned rn) { emit(0x5AC00800u | (rn << 5) | rd); }
    void rev64(unsigned rd, unsigned rn) { emit(0xDAC00C00u | (rn << 5) | rd); }
    void clz(unsigned rd, unsigned rn) { emit(0x5AC01000u | (rn << 5) | rd); }
    // Three-source.
    void mul(unsigned rd, unsigned rn, unsigned rm) { emit(0x1B000000u | (rm << 16) | (ZR << 10) | (rn << 5) | rd); }
    void smull(unsigned xd, unsigned wn, unsigned wm) { emit(0x9B200000u | (wm << 16) | (ZR << 10) | (wn << 5) | xd); }
    void umull(unsigned xd, unsigned wn, unsigned wm) { emit(0x9BA00000u | (wm << 16) | (ZR << 10) | (wn << 5) | xd); }

    // Conditional select.
    void csel(unsigned rd, unsigned rn, unsigned rm, Cond c) { emit(0x1A800000u | (rm << 16) | (c << 12) | (rn << 5) | rd); }
    void csinc(unsigned rd, unsigned rn, unsigned rm, Cond c) { emit(0x1A800400u | (rm << 16) | (c << 12) | (rn << 5) | rd); }
    void cset(unsigned rd, Cond c) { csinc(rd, ZR, ZR, invert(c)); }
    void csetm(unsigned rd, Cond c) { emit(0x5A800000u | (ZR << 16) | (invert(c) << 12) | (ZR << 5) | rd); }
    // CCMP Wn, #imm5, #nzcv, cond
    void ccmp_imm(unsigned rn, unsigned imm5, unsigned nzcv, Cond c) { emit(0x7A400800u | (imm5 << 16) | (c << 12) | (rn << 5) | nzcv); }
    void ccmn_imm(unsigned rn, unsigned imm5, unsigned nzcv, Cond c) { emit(0x3A400800u | (imm5 << 16) | (c << 12) | (rn << 5) | nzcv); }
    void ccmp(unsigned rn, unsigned rm, unsigned nzcv, Cond c) { emit(0x7A400000u | (rm << 16) | (c << 12) | (rn << 5) | nzcv); }
    void tst_imm(unsigned rn, std::uint32_t imm) {
        if (!logic_imm(false, 3, ZR, rn, imm)) std::abort();
    }

    // Unsigned scaled offset. size: 0 byte, 1 half, 2 word, 3 dword. fp selects SIMD&FP registers.
    void ldst_uimm(unsigned size, bool load, bool fp, unsigned rt, unsigned rn, unsigned offset) {
        unsigned imm = offset >> size;
        emit((size << 30) | 0x39000000u | (fp ? 1u << 26 : 0) | (load ? 1u << 22 : 0) | (imm << 10) | (rn << 5) | rt);
    }
    void ldrw(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(2, true, false, rt, rn, off); }
    void strw(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(2, false, false, rt, rn, off); }
    void ldrx(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(3, true, false, rt, rn, off); }
    void strx(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(3, false, false, rt, rn, off); }
    void ldrb(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(0, true, false, rt, rn, off); }
    void strb(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(0, false, false, rt, rn, off); }
    void ldrd(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(3, true, true, rt, rn, off); }
    void strd(unsigned rt, unsigned rn, unsigned off) { ldst_uimm(3, false, true, rt, rn, off); }
    // Register offset [Xn, Wm, UXTW].
    void ldst_uxtw(unsigned size, bool load, bool fp, unsigned rt, unsigned rn, unsigned rm) {
        emit((size << 30) | 0x38204800u | (fp ? 1u << 26 : 0) | (load ? 1u << 22 : 0) | (rm << 16) | (rn << 5) | rt);
    }
    // Load/store pair, signed offset (mode 2), pre-index (3), post-index (1). size: 2 word, 3 dword.
    void ldstp(unsigned size, bool load, bool fp, unsigned mode, unsigned rt, unsigned rt2, unsigned rn, int offset) {
        unsigned opc = fp ? (size == 3 ? 1u : 0u) : (size == 3 ? 2u : 0u);
        unsigned imm7 = static_cast<unsigned>(offset >> size) & 0x7F;
        emit((opc << 30) | 0x28000000u | (fp ? 1u << 26 : 0) | (mode << 23) | (load ? 1u << 22 : 0) | (imm7 << 15) |
             (rt2 << 10) | (rn << 5) | rt);
    }
    // LDRSW Xt, [Xn, Xm, LSL #2]
    void ldrsw_lsl2(unsigned xt, unsigned xn, unsigned xm) { emit(0xB8A07800u | (xm << 16) | (xn << 5) | xt); }
    void ldr_lit(unsigned xt, std::int32_t byte_offset) { emit(0x58000000u | ((static_cast<std::uint32_t>(byte_offset >> 2) & 0x7FFFF) << 5) | xt); }
    void adr(unsigned xd, std::int32_t byte_offset) {
        std::uint32_t imm = static_cast<std::uint32_t>(byte_offset) & 0x1FFFFF;
        emit(0x10000000u | ((imm & 3) << 29) | ((imm >> 2) << 5) | xd);
    }

    void b(std::int32_t byte_offset) { emit(0x14000000u | (static_cast<std::uint32_t>(byte_offset >> 2) & 0x03FFFFFF)); }
    void bl(std::int32_t byte_offset) { emit(0x94000000u | (static_cast<std::uint32_t>(byte_offset >> 2) & 0x03FFFFFF)); }
    void bcond(Cond c, std::int32_t byte_offset) { emit(0x54000000u | ((static_cast<std::uint32_t>(byte_offset >> 2) & 0x7FFFF) << 5) | c); }
    void cbz(unsigned rt, std::int32_t byte_offset, bool nz = false, bool x = false) {
        emit((x ? 0xB4000000u : 0x34000000u) | (nz ? 1u << 24 : 0) | ((static_cast<std::uint32_t>(byte_offset >> 2) & 0x7FFFF) << 5) | rt);
    }
    // TBZ/TBNZ Wt, #bit, label (bit < 32)
    void tbz(unsigned rt, unsigned bit, std::int32_t byte_offset, bool nz = false) {
        emit(0x36000000u | (nz ? 1u << 24 : 0) | ((bit & 31) << 19) | ((static_cast<std::uint32_t>(byte_offset >> 2) & 0x3FFF) << 5) | rt);
    }
    void br(unsigned rn) { emit(0xD61F0000u | (rn << 5)); }
    void blr(unsigned rn) { emit(0xD63F0000u | (rn << 5)); }
    void ret() { emit(0xD65F03C0u); }

    void fp2(unsigned op, unsigned dd, unsigned dn, unsigned dm, bool single = false) {
        emit((single ? 0x1E200800u : 0x1E600800u) | (dm << 16) | (op << 12) | (dn << 5) | dd);
    }
    void fmul(unsigned d, unsigned n, unsigned m) { fp2(0, d, n, m); }
    void fdiv(unsigned d, unsigned n, unsigned m) { fp2(1, d, n, m); }
    void fadd(unsigned d, unsigned n, unsigned m) { fp2(2, d, n, m); }
    void fsub(unsigned d, unsigned n, unsigned m) { fp2(3, d, n, m); }
    void fp1(unsigned opc, unsigned dd, unsigned dn, bool single = false) {
        emit((single ? 0x1E204000u : 0x1E604000u) | (opc << 15) | (dn << 5) | dd);
    }
    void fmov(unsigned d, unsigned n) { if (d != n) fp1(0, d, n); }
    void fabs(unsigned d, unsigned n) { fp1(1, d, n); }
    void fneg(unsigned d, unsigned n, bool single = false) { fp1(2, d, n, single); }
    void fsqrt(unsigned d, unsigned n) { fp1(3, d, n); }
    void fcvt_sd(unsigned s, unsigned d) { emit(0x1E624000u | (d << 5) | s); }   // Sd <- Dn
    void fcvt_ds(unsigned d, unsigned s) { emit(0x1E22C000u | (s << 5) | d); }   // Dd <- Sn
    // FMADD a+n*m, FMSUB a-n*m, FNMADD -a-n*m, FNMSUB -a+n*m
    void fp3(unsigned o1, unsigned o0, unsigned d, unsigned n, unsigned m, unsigned a) {
        emit(0x1F400000u | (o1 << 21) | (m << 16) | (o0 << 15) | (a << 10) | (n << 5) | d);
    }
    void fmadd(unsigned d, unsigned n, unsigned m, unsigned a) { fp3(0, 0, d, n, m, a); }
    void fmsub(unsigned d, unsigned n, unsigned m, unsigned a) { fp3(0, 1, d, n, m, a); }
    void fnmadd(unsigned d, unsigned n, unsigned m, unsigned a) { fp3(1, 0, d, n, m, a); }
    void fnmsub(unsigned d, unsigned n, unsigned m, unsigned a) { fp3(1, 1, d, n, m, a); }
    void fcmp(unsigned n, unsigned m) { emit(0x1E602000u | (m << 16) | (n << 5)); }
    void fcmp_zero(unsigned n) { emit(0x1E602008u | (n << 5)); }
    void fcsel(unsigned d, unsigned n, unsigned m, Cond c) { emit(0x1E600C00u | (m << 16) | (c << 12) | (n << 5) | d); }
    void fmov_to_d(unsigned d, unsigned xn) { emit(0x9E670000u | (xn << 5) | d); }
    void fmov_from_d(unsigned xd, unsigned n) { emit(0x9E660000u | (n << 5) | xd); }
    void fmov_to_s(unsigned s, unsigned wn) { emit(0x1E270000u | (wn << 5) | s); }
    void fmov_from_s(unsigned wd, unsigned s) { emit(0x1E260000u | (s << 5) | wd); }
    void fmov_imm(unsigned d, unsigned imm8) { emit(0x1E601000u | (imm8 << 13) | d); }
    // AdvSIMD, 64-bit vectors: REV32 Vd.8B; FCVTL Vd.2D <- Vn.2S; FCVTN Vd.2S <- Vn.2D;
    // DUP Dd, Vn.D[1]; INS Vd.D[1], Vn.D[0].
    void rev32_8b(unsigned d, unsigned n) { emit(0x2E200800u | (n << 5) | d); }
    void fcvtl_2d(unsigned d, unsigned n) { emit(0x0E617800u | (n << 5) | d); }
    void fcvtn_2s(unsigned d, unsigned n) { emit(0x0E616800u | (n << 5) | d); }
    void dup_d1(unsigned d, unsigned n) { emit(0x5E180400u | (n << 5) | d); }
    void ins_d1(unsigned d, unsigned n) { emit(0x6E180400u | (n << 5) | d); }
    void fcvtzs_wd(unsigned wd, unsigned dn) { emit(0x1E780000u | (dn << 5) | wd); }
};

} // namespace a64

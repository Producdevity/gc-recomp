// Gekko (PowerPC 750CL) instruction decoding.
#pragma once

#include <cstdint>

namespace gcr {

#define GCR_OPS(X) \
    X(invalid) \
    X(twi) X(mulli) X(subfic) X(cmpli) X(cmpi) X(addic) X(addic_rc) X(addi) X(addis) \
    X(bc) X(sc) X(b) \
    X(mcrf) X(bclr) X(crnor) X(rfi) X(crandc) X(isync) X(crxor) X(crnand) X(crand) X(creqv) X(crorc) X(cror) X(bcctr) \
    X(rlwimi) X(rlwinm) X(rlwnm) X(ori) X(oris) X(xori) X(xoris) X(andi_rc) X(andis_rc) \
    X(cmp) X(tw) X(subfc) X(addc) X(mulhwu) X(mfcr) X(lwarx) X(lwzx) X(slw) X(cntlzw) X(and_) X(cmpl) X(subf) \
    X(dcbst) X(lwzux) X(andc) X(mulhw) X(mfmsr) X(dcbf) X(lbzx) X(neg) X(lbzux) X(nor) X(subfe) X(adde) X(mtcrf) \
    X(mtmsr) X(stwcx) X(stwx) X(stwux) X(subfze) X(addze) X(mtsr) X(stbx) X(subfme) X(addme) X(mullw) X(mtsrin) \
    X(dcbtst) X(stbux) X(add) X(dcbt) X(lhzx) X(eqv) X(tlbie) X(eciwx) X(lhzux) X(xor_) X(mfspr) X(lhax) X(mftb) \
    X(lhaux) X(sthx) X(orc) X(ecowx) X(sthux) X(or_) X(divwu) X(mtspr) X(dcbi) X(nand) X(divw) X(mcrxr) X(lswx) \
    X(lwbrx) X(lfsx) X(srw) X(tlbsync) X(lfsux) X(mfsr) X(lswi) X(sync) X(lfdx) X(lfdux) X(mfsrin) X(stswx) X(stwbrx) \
    X(stfsx) X(stfsux) X(stswi) X(stfdx) X(stfdux) X(lhbrx) X(sraw) X(srawi) X(eieio) X(sthbrx) X(extsh) X(extsb) \
    X(icbi) X(stfiwx) X(dcbz) \
    X(lwz) X(lwzu) X(lbz) X(lbzu) X(stw) X(stwu) X(stb) X(stbu) X(lhz) X(lhzu) X(lha) X(lhau) X(sth) X(sthu) \
    X(lmw) X(stmw) X(lfs) X(lfsu) X(lfd) X(lfdu) X(stfs) X(stfsu) X(stfd) X(stfdu) \
    X(psq_l) X(psq_lu) X(psq_st) X(psq_stu) X(psq_lx) X(psq_stx) X(psq_lux) X(psq_stux) \
    X(fdivs) X(fsubs) X(fadds) X(fres) X(fmuls) X(fmsubs) X(fmadds) X(fnmsubs) X(fnmadds) \
    X(fdiv) X(fsub) X(fadd) X(fsel) X(fmul) X(frsqrte) X(fmsub) X(fmadd) X(fnmsub) X(fnmadd) \
    X(fcmpu) X(frsp) X(fctiw) X(fctiwz) X(fcmpo) X(mtfsb1) X(fneg) X(mcrfs) X(mtfsb0) X(fmr) X(mtfsfi) X(fnabs) \
    X(fabs) X(mffs) X(mtfsf) \
    X(ps_sum0) X(ps_sum1) X(ps_muls0) X(ps_muls1) X(ps_madds0) X(ps_madds1) X(ps_div) X(ps_sub) X(ps_add) \
    X(ps_sel) X(ps_res) X(ps_mul) X(ps_rsqrte) X(ps_msub) X(ps_madd) X(ps_nmsub) X(ps_nmadd) \
    X(ps_cmpu0) X(ps_cmpo0) X(ps_neg) X(ps_cmpu1) X(ps_mr) X(ps_cmpo1) X(ps_nabs) X(ps_abs) \
    X(ps_merge00) X(ps_merge01) X(ps_merge10) X(ps_merge11) X(dcbz_l)

enum class Op : std::uint16_t {
#define GCR_ENUM(name) name,
    GCR_OPS(GCR_ENUM)
#undef GCR_ENUM
    count
};

const char* op_name(Op);

struct Insn {
    std::uint32_t w = 0;
    std::uint32_t opcd() const { return w >> 26; }
    std::uint32_t rd() const { return (w >> 21) & 31; }
    std::uint32_t rs() const { return rd(); }
    std::uint32_t ra() const { return (w >> 16) & 31; }
    std::uint32_t rb() const { return (w >> 11) & 31; }
    std::uint32_t rc() const { return (w >> 6) & 31; }
    std::int32_t simm() const { return static_cast<std::int16_t>(w & 0xFFFF); }
    std::uint32_t uimm() const { return w & 0xFFFF; }
    bool rcbit() const { return w & 1; }
    bool oe() const { return (w >> 10) & 1; }
    std::uint32_t xo10() const { return (w >> 1) & 0x3FF; }
    std::uint32_t xo9() const { return (w >> 1) & 0x1FF; }
    std::uint32_t xo6() const { return (w >> 1) & 0x3F; }
    std::uint32_t xo5() const { return (w >> 1) & 0x1F; }
    std::uint32_t sh() const { return (w >> 11) & 31; }
    std::uint32_t mb() const { return (w >> 6) & 31; }
    std::uint32_t me() const { return (w >> 1) & 31; }
    std::uint32_t crfd() const { return (w >> 23) & 7; }
    std::uint32_t crfs() const { return (w >> 18) & 7; }
    std::uint32_t bo() const { return rd(); }
    std::uint32_t bi() const { return ra(); }
    std::int32_t bd() const { return static_cast<std::int16_t>(w & 0xFFFC); }
    std::int32_t li() const {
        std::int32_t v = static_cast<std::int32_t>(w & 0x03FFFFFC);
        if (v & 0x02000000) v -= 0x04000000;
        return v;
    }
    bool aa() const { return w & 2; }
    bool lk() const { return w & 1; }
    std::uint32_t spr() const { return ((w >> 16) & 31) | (((w >> 11) & 31) << 5); }
    std::uint32_t crm() const { return (w >> 12) & 0xFF; }
    std::uint32_t fm() const { return (w >> 17) & 0xFF; }
    std::uint32_t sr() const { return (w >> 16) & 15; }
    std::int32_t psq_d() const {
        std::int32_t v = static_cast<std::int32_t>(w & 0xFFF);
        if (v & 0x800) v -= 0x1000;
        return v;
    }
    std::uint32_t psq_w() const { return (w >> 15) & 1; }
    std::uint32_t psq_i() const { return (w >> 12) & 7; }
    std::uint32_t psqx_w() const { return (w >> 10) & 1; }
    std::uint32_t psqx_i() const { return (w >> 7) & 7; }
};

Op decode(Insn);

// MASK(mb, me) in IBM bit numbering (bit 0 is the most significant).
inline std::uint32_t rotate_mask(std::uint32_t mb, std::uint32_t me) {
    std::uint32_t begin = 0xFFFFFFFFu >> mb;
    std::uint32_t end = me == 31 ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> (me + 1));
    return mb <= me ? (begin & end) : (begin | end);
}

} // namespace gcr

#include "ppc.hpp"

namespace gcr {

const char* op_name(Op op) {
    static const char* const names[] = {
#define GCR_NAME(name) #name,
        GCR_OPS(GCR_NAME)
#undef GCR_NAME
    };
    auto i = static_cast<unsigned>(op);
    return i < static_cast<unsigned>(Op::count) ? names[i] : "?";
}

static Op decode4(Insn i) {
    switch (i.xo6()) {
    case 6: return Op::psq_lx;
    case 7: return Op::psq_stx;
    case 38: return Op::psq_lux;
    case 39: return Op::psq_stux;
    }
    switch (i.xo5()) {
    case 10: return Op::ps_sum0;
    case 11: return Op::ps_sum1;
    case 12: return Op::ps_muls0;
    case 13: return Op::ps_muls1;
    case 14: return Op::ps_madds0;
    case 15: return Op::ps_madds1;
    case 18: return Op::ps_div;
    case 20: return Op::ps_sub;
    case 21: return Op::ps_add;
    case 23: return Op::ps_sel;
    case 24: return Op::ps_res;
    case 25: return Op::ps_mul;
    case 26: return Op::ps_rsqrte;
    case 28: return Op::ps_msub;
    case 29: return Op::ps_madd;
    case 30: return Op::ps_nmsub;
    case 31: return Op::ps_nmadd;
    }
    switch (i.xo10()) {
    case 0: return Op::ps_cmpu0;
    case 32: return Op::ps_cmpo0;
    case 40: return Op::ps_neg;
    case 64: return Op::ps_cmpu1;
    case 72: return Op::ps_mr;
    case 96: return Op::ps_cmpo1;
    case 136: return Op::ps_nabs;
    case 264: return Op::ps_abs;
    case 528: return Op::ps_merge00;
    case 560: return Op::ps_merge01;
    case 592: return Op::ps_merge10;
    case 624: return Op::ps_merge11;
    case 1014: return Op::dcbz_l;
    }
    return Op::invalid;
}

static Op decode19(Insn i) {
    switch (i.xo10()) {
    case 0: return Op::mcrf;
    case 16: return Op::bclr;
    case 33: return Op::crnor;
    case 50: return Op::rfi;
    case 129: return Op::crandc;
    case 150: return Op::isync;
    case 193: return Op::crxor;
    case 225: return Op::crnand;
    case 257: return Op::crand;
    case 289: return Op::creqv;
    case 417: return Op::crorc;
    case 449: return Op::cror;
    case 528: return Op::bcctr;
    }
    return Op::invalid;
}

static Op decode31(Insn i) {
    switch (i.xo9()) {
    case 8: return Op::subfc;
    case 10: return Op::addc;
    case 11: return Op::mulhwu;
    case 40: return Op::subf;
    case 75: return Op::mulhw;
    case 104: return Op::neg;
    case 136: return Op::subfe;
    case 138: return Op::adde;
    case 200: return Op::subfze;
    case 202: return Op::addze;
    case 232: return Op::subfme;
    case 234: return Op::addme;
    case 235: return Op::mullw;
    case 266: return Op::add;
    case 459: return Op::divwu;
    case 491: return Op::divw;
    }
    switch (i.xo10()) {
    case 0: return Op::cmp;
    case 4: return Op::tw;
    case 19: return Op::mfcr;
    case 20: return Op::lwarx;
    case 23: return Op::lwzx;
    case 24: return Op::slw;
    case 26: return Op::cntlzw;
    case 28: return Op::and_;
    case 32: return Op::cmpl;
    case 54: return Op::dcbst;
    case 55: return Op::lwzux;
    case 60: return Op::andc;
    case 83: return Op::mfmsr;
    case 86: return Op::dcbf;
    case 87: return Op::lbzx;
    case 119: return Op::lbzux;
    case 124: return Op::nor;
    case 144: return Op::mtcrf;
    case 146: return Op::mtmsr;
    case 150: return Op::stwcx;
    case 151: return Op::stwx;
    case 183: return Op::stwux;
    case 210: return Op::mtsr;
    case 215: return Op::stbx;
    case 242: return Op::mtsrin;
    case 246: return Op::dcbtst;
    case 247: return Op::stbux;
    case 278: return Op::dcbt;
    case 279: return Op::lhzx;
    case 284: return Op::eqv;
    case 306: return Op::tlbie;
    case 310: return Op::eciwx;
    case 311: return Op::lhzux;
    case 316: return Op::xor_;
    case 339: return Op::mfspr;
    case 343: return Op::lhax;
    case 371: return Op::mftb;
    case 375: return Op::lhaux;
    case 407: return Op::sthx;
    case 412: return Op::orc;
    case 438: return Op::ecowx;
    case 439: return Op::sthux;
    case 444: return Op::or_;
    case 467: return Op::mtspr;
    case 470: return Op::dcbi;
    case 476: return Op::nand;
    case 512: return Op::mcrxr;
    case 533: return Op::lswx;
    case 534: return Op::lwbrx;
    case 535: return Op::lfsx;
    case 536: return Op::srw;
    case 566: return Op::tlbsync;
    case 567: return Op::lfsux;
    case 595: return Op::mfsr;
    case 597: return Op::lswi;
    case 598: return Op::sync;
    case 599: return Op::lfdx;
    case 631: return Op::lfdux;
    case 659: return Op::mfsrin;
    case 661: return Op::stswx;
    case 662: return Op::stwbrx;
    case 663: return Op::stfsx;
    case 695: return Op::stfsux;
    case 725: return Op::stswi;
    case 727: return Op::stfdx;
    case 759: return Op::stfdux;
    case 790: return Op::lhbrx;
    case 792: return Op::sraw;
    case 824: return Op::srawi;
    case 854: return Op::eieio;
    case 918: return Op::sthbrx;
    case 922: return Op::extsh;
    case 954: return Op::extsb;
    case 982: return Op::icbi;
    case 983: return Op::stfiwx;
    case 1014: return Op::dcbz;
    }
    return Op::invalid;
}

static Op decode59(Insn i) {
    switch (i.xo5()) {
    case 18: return Op::fdivs;
    case 20: return Op::fsubs;
    case 21: return Op::fadds;
    case 24: return Op::fres;
    case 25: return Op::fmuls;
    case 28: return Op::fmsubs;
    case 29: return Op::fmadds;
    case 30: return Op::fnmsubs;
    case 31: return Op::fnmadds;
    }
    return Op::invalid;
}

static Op decode63(Insn i) {
    switch (i.xo5()) {
    case 18: return Op::fdiv;
    case 20: return Op::fsub;
    case 21: return Op::fadd;
    case 23: return Op::fsel;
    case 25: return Op::fmul;
    case 26: return Op::frsqrte;
    case 28: return Op::fmsub;
    case 29: return Op::fmadd;
    case 30: return Op::fnmsub;
    case 31: return Op::fnmadd;
    }
    switch (i.xo10()) {
    case 0: return Op::fcmpu;
    case 12: return Op::frsp;
    case 14: return Op::fctiw;
    case 15: return Op::fctiwz;
    case 32: return Op::fcmpo;
    case 38: return Op::mtfsb1;
    case 40: return Op::fneg;
    case 64: return Op::mcrfs;
    case 70: return Op::mtfsb0;
    case 72: return Op::fmr;
    case 134: return Op::mtfsfi;
    case 136: return Op::fnabs;
    case 264: return Op::fabs;
    case 583: return Op::mffs;
    case 711: return Op::mtfsf;
    }
    return Op::invalid;
}

Op decode(Insn i) {
    switch (i.opcd()) {
    case 3: return Op::twi;
    case 4: return decode4(i);
    case 7: return Op::mulli;
    case 8: return Op::subfic;
    case 10: return Op::cmpli;
    case 11: return Op::cmpi;
    case 12: return Op::addic;
    case 13: return Op::addic_rc;
    case 14: return Op::addi;
    case 15: return Op::addis;
    case 16: return Op::bc;
    case 17: return Op::sc;
    case 18: return Op::b;
    case 19: return decode19(i);
    case 20: return Op::rlwimi;
    case 21: return Op::rlwinm;
    case 23: return Op::rlwnm;
    case 24: return Op::ori;
    case 25: return Op::oris;
    case 26: return Op::xori;
    case 27: return Op::xoris;
    case 28: return Op::andi_rc;
    case 29: return Op::andis_rc;
    case 31: return decode31(i);
    case 32: return Op::lwz;
    case 33: return Op::lwzu;
    case 34: return Op::lbz;
    case 35: return Op::lbzu;
    case 36: return Op::stw;
    case 37: return Op::stwu;
    case 38: return Op::stb;
    case 39: return Op::stbu;
    case 40: return Op::lhz;
    case 41: return Op::lhzu;
    case 42: return Op::lha;
    case 43: return Op::lhau;
    case 44: return Op::sth;
    case 45: return Op::sthu;
    case 46: return Op::lmw;
    case 47: return Op::stmw;
    case 48: return Op::lfs;
    case 49: return Op::lfsu;
    case 50: return Op::lfd;
    case 51: return Op::lfdu;
    case 52: return Op::stfs;
    case 53: return Op::stfsu;
    case 54: return Op::stfd;
    case 55: return Op::stfdu;
    case 56: return Op::psq_l;
    case 57: return Op::psq_lu;
    case 59: return decode59(i);
    case 60: return Op::psq_st;
    case 61: return Op::psq_stu;
    case 63: return decode63(i);
    }
    return Op::invalid;
}

} // namespace gcr

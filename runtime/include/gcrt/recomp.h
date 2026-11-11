/* Interface between recompiled guest code and the runtime. C11, included by generated code. */
#ifndef GCRT_RECOMP_H
#define GCRT_RECOMP_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One guest thread's register file. Recompiled functions keep registers in locals and
   synchronise with this structure at calls and returns. */
typedef struct gc_ctx {
    uint32_t r[32];
    double f[32];      /* ps0 */
    double ps1[32];
    uint8_t cr[32];    /* one byte per CR bit; bit 4n+0 LT, +1 GT, +2 EQ, +3 SO */
    uint32_t lr, ctr;
    uint8_t ca, so, ov, bc;
    uint32_t fpscr;
    uint32_t gqr[8];
    /* Kept by the runtime for translated code, which holds the context in a register:
       the guest memory base (gc_mem) and, for the running context, a copy of gc_attn. */
    uint8_t* mem;
    volatile uint32_t attn;
} gc_ctx;

typedef void (*gc_func)(gc_ctx*);
typedef struct gc_func_entry {
    uint32_t start, end;
    gc_func func;
} gc_func_entry;

extern const gc_func_entry gc_func_table[];
extern const unsigned gc_func_count;
extern const unsigned long long gc_dol_hash;

/* Base of the guest address space: guest address A lives at gc_mem + A. */
extern uint8_t* gc_mem;
/* Nonzero when an interrupt, timer or device event may need attention. */
extern volatile uint32_t gc_attn;
extern uint32_t gc_msr;
extern uint32_t gc_sr[16];

void gc_safepoint(gc_ctx*);
void gc_spin(gc_ctx*);
void gc_call(gc_ctx*, uint32_t target);
void gc_jump(gc_ctx*, uint32_t target, uint32_t from);
void gc_mtmsr(gc_ctx*, uint32_t);
uint32_t gc_mfspr(gc_ctx*, uint32_t spr);
void gc_mtspr(gc_ctx*, uint32_t spr, uint32_t value);
uint32_t gc_mftb(uint32_t tbr);
void gc_sc(gc_ctx*);
void gc_rfi(gc_ctx*, uint32_t pc);
void gc_trap(gc_ctx*, uint32_t pc);
void gc_illegal(gc_ctx*, uint32_t pc, uint32_t word);
/* Runtime replacements of recognised SDK functions (recompiler/natives.hpp). */
void hle_Nop(gc_ctx*);
void hle_PSMTXConcat(gc_ctx*);
void hle_PSVECAdd(gc_ctx*);
void hle_PSVECSubtract(gc_ctx*);
void hle_PSVECScale(gc_ctx*);
void hle_PSVECNormalize(gc_ctx*);
void hle_PSVECSquareMag(gc_ctx*);
void hle_PSVECCrossProduct(gc_ctx*);
void hle_PSVECMag(gc_ctx*);
void hle_sinf(gc_ctx*);
void hle_cosf(gc_ctx*);
void hle_WriteMTXPS4x3(gc_ctx*);
void hle_WriteMTXPS3x3from3x4(gc_ctx*);
void hle_VecNormalizeSafe(gc_ctx*);
void hle_NameHashFind(gc_ctx*);
void hle_NameParamGet(gc_ctx*);
void hle_FindEntryByKey(gc_ctx*);
void gc_bad_return(uint32_t pc, uint32_t lr, uint32_t expected);
/* Resume points for OSSaveContext call sites: gc_setjmp returns 0, then 1 when resumed. */
typedef struct gc_jmp { uint64_t words[22]; } gc_jmp;
int gcrt_setjmp(gc_jmp*) __attribute__((returns_twice));
void gcrt_longjmp(gc_jmp*) __attribute__((noreturn));
gc_jmp* gc_resume_slot(gc_ctx*, uint32_t context);
void gc_mtfsf(gc_ctx*, uint32_t fm, uint32_t value);

uint8_t gc_io_read8(uint32_t);
uint16_t gc_io_read16(uint32_t);
uint32_t gc_io_read32(uint32_t);
void gc_io_write8(uint32_t, uint8_t);
void gc_io_write16(uint32_t, uint16_t);
void gc_io_write32(uint32_t, uint32_t);
void gc_io_write64(uint32_t, uint64_t);

#define GC_UNLIKELY(x) __builtin_expect(!!(x), 0)

/* Loop back-edges. With GC_VIRTUAL_TIME (deterministic debugging), every back-edge is a
   step of guest time and the runtime stops at the step its next event is due. */
extern uint64_t gc_steps, gc_step_deadline;
#ifdef GC_VIRTUAL_TIME
#define GC_BACKEDGE(r1) do { if (GC_UNLIKELY(gc_attn | (++gc_steps >= gc_step_deadline))) { c->r[1] = (r1); gc_safepoint(c); } } while (0)
#else
#define GC_BACKEDGE(r1) do { if (GC_UNLIKELY(gc_attn)) { c->r[1] = (r1); gc_safepoint(c); } } while (0)
#endif
#define GC_ROTL(x, n) __builtin_rotateleft32((uint32_t)(x), (uint32_t)(n))

#ifdef GC_CHECK_RETURNS
#define GC_CHECK_RETURN(pc, lr, expected) do { if ((lr) != (expected)) gc_bad_return((pc), (lr), (expected)); } while (0)
#else
#define GC_CHECK_RETURN(pc, lr, expected) ((void)0)
#endif

/* Development: with GC_WATCH, guest stores into [gc_watch_lo, gc_watch_hi) (physical) are reported. */
extern uint32_t gc_watch_lo, gc_watch_hi;
void gc_watch_hit(uint32_t ea, uint32_t size, uint64_t value);
#ifdef GC_WATCH
#define GC_WATCH_STORE(ea, size, v) do { if (GC_UNLIKELY(((ea) & 0x01FFFFFFu) - gc_watch_lo < gc_watch_hi - gc_watch_lo)) gc_watch_hit((ea), (size), (v)); } while (0)
#else
#define GC_WATCH_STORE(ea, size, v) ((void)0)
#endif

/* Hardware registers (0xCC000000) and the EFB (0xC8000000) are handled by the runtime. They
   are the only mapped addresses with bit 27 set (RAM ends at 0x81800000), so one bit test
   selects them; any other address with that bit is reported as unmapped hardware. */
static inline int gc_is_io(uint32_t ea) { return (ea & 0x08000000u) != 0; }

/* The write-gather pipe: stores to 0xCC008000 append big-endian bytes here, and the
   runtime takes the complete 32-byte bursts once GC_GATHER_BATCH bytes are gathered
   (gc_gather_burst), and before any other hardware access (gc_gather_sync). */
#define GC_GATHER_BATCH 256u
typedef struct gc_gather {
    uint8_t bytes[GC_GATHER_BATCH + 8];   /* a store may run up to 8 bytes past the batch */
    uint32_t fill;                        /* below GC_GATHER_BATCH between stores */
} gc_gather;
extern gc_gather gc_gp;
void gc_gather_burst(void);
/* Appends size bytes of v, most significant first; false when the store is not a whole
   gather-pipe write (0xCC008000). Bytes past a completed burst open the next one. */
static inline int gc_gather_store(uint32_t ea, uint64_t v, unsigned size) {
    uint32_t f = gc_gp.fill;
    if (ea != 0xCC008000u) return 0;
    for (unsigned k = 0; k < size; ++k) gc_gp.bytes[f + k] = (uint8_t)(v >> (8 * (size - 1 - k)));
    f += size;
    gc_gp.fill = f;
    if (f >= GC_GATHER_BATCH) gc_gather_burst();
    return 1;
}

static inline uint32_t gc_ld32(uint8_t* m, uint32_t ea) {
    if (GC_UNLIKELY(gc_is_io(ea))) return gc_io_read32(ea);
    uint32_t v;
    memcpy(&v, m + ea, 4);
    return __builtin_bswap32(v);
}
static inline uint32_t gc_ld16(uint8_t* m, uint32_t ea) {
    if (GC_UNLIKELY(gc_is_io(ea))) return gc_io_read16(ea);
    uint16_t v;
    memcpy(&v, m + ea, 2);
    return __builtin_bswap16(v);
}
static inline uint32_t gc_ld8(uint8_t* m, uint32_t ea) {
    if (GC_UNLIKELY(gc_is_io(ea))) return gc_io_read8(ea);
    return m[ea];
}
static inline uint64_t gc_ld64(uint8_t* m, uint32_t ea) {
    if (GC_UNLIKELY(gc_is_io(ea))) return ((uint64_t)gc_io_read32(ea) << 32) | gc_io_read32(ea + 4);
    uint64_t v;
    memcpy(&v, m + ea, 8);
    return __builtin_bswap64(v);
}
static inline void gc_st32(uint8_t* m, uint32_t ea, uint32_t v) {
    if (GC_UNLIKELY(gc_is_io(ea))) { if (!gc_gather_store(ea, v, 4)) gc_io_write32(ea, v); return; }
    GC_WATCH_STORE(ea, 4, v);
    v = __builtin_bswap32(v);
    memcpy(m + ea, &v, 4);
}
static inline void gc_st16(uint8_t* m, uint32_t ea, uint32_t v) {
    if (GC_UNLIKELY(gc_is_io(ea))) { if (!gc_gather_store(ea, v & 0xFFFF, 2)) gc_io_write16(ea, (uint16_t)v); return; }
    GC_WATCH_STORE(ea, 2, v);
    uint16_t h = __builtin_bswap16((uint16_t)v);
    memcpy(m + ea, &h, 2);
}
static inline void gc_st8(uint8_t* m, uint32_t ea, uint32_t v) {
    if (GC_UNLIKELY(gc_is_io(ea))) { if (!gc_gather_store(ea, v & 0xFF, 1)) gc_io_write8(ea, (uint8_t)v); return; }
    GC_WATCH_STORE(ea, 1, v);
    m[ea] = (uint8_t)v;
}
static inline void gc_st64(uint8_t* m, uint32_t ea, uint64_t v) {
    if (GC_UNLIKELY(gc_is_io(ea))) { if (!gc_gather_store(ea, v, 8)) gc_io_write64(ea, v); return; }
    GC_WATCH_STORE(ea, 8, v);
    v = __builtin_bswap64(v);
    memcpy(m + ea, &v, 8);
}

static inline uint32_t gc_f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float gc_u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static inline uint64_t gc_d2u(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }
static inline double gc_u2d(uint64_t u) { double d; memcpy(&d, &u, 8); return d; }
/* Round to single precision, as single-precision instructions do. */
static inline double gc_rs(double x) { return (double)(float)x; }

#define LD32(ea) gc_ld32(M, (ea))
#define LD16(ea) gc_ld16(M, (ea))
#define LDS16(ea) ((uint32_t)(int32_t)(int16_t)gc_ld16(M, (ea)))
#define LD8(ea) gc_ld8(M, (ea))
#define LDBR32(ea) __builtin_bswap32(gc_ld32(M, (ea)))
#define LDBR16(ea) ((uint32_t)__builtin_bswap16((uint16_t)gc_ld16(M, (ea))))
#define ST32(ea, v) gc_st32(M, (ea), (v))
#define ST16(ea, v) gc_st16(M, (ea), (v))
#define ST8(ea, v) gc_st8(M, (ea), (v))
#define STBR32(ea, v) gc_st32(M, (ea), __builtin_bswap32((uint32_t)(v)))
#define STBR16(ea, v) gc_st16(M, (ea), __builtin_bswap16((uint16_t)(v)))
#define LDF32(ea) gc_u2f(gc_ld32(M, (ea)))
#define LDF64(ea) gc_u2d(gc_ld64(M, (ea)))
#define STF32(ea, v) gc_st32(M, (ea), gc_f2u(v))
#define STF64(ea, v) gc_st64(M, (ea), gc_d2u(v))

static inline uint32_t gc_divw(uint32_t a, uint32_t b) {
    int32_t x = (int32_t)a, y = (int32_t)b;
    if (y == 0 || (x == INT32_MIN && y == -1)) return x < 0 ? 0xFFFFFFFFu : 0u;
    return (uint32_t)(x / y);
}
static inline uint32_t gc_divwu(uint32_t a, uint32_t b) { return b ? a / b : 0u; }

static inline uint32_t gc_saturate_s32(double d) {
    if (d != d) return 0x80000000u;
    if (d >= 2147483647.0) return 0x7FFFFFFFu;
    if (d <= -2147483648.0) return 0x80000000u;
    return (uint32_t)(int32_t)d;
}
static inline double gc_fctiwz(double d) {
    return gc_u2d(0xFFF8000000000000ull | gc_saturate_s32(d));
}
double gc_fctiw(gc_ctx*, double);

static inline void gc_dcbz(uint8_t* m, uint32_t ea) {
    ea &= ~31u;
    if (GC_UNLIKELY(gc_is_io(ea))) return;
    GC_WATCH_STORE(ea, 32, 0);
    memset(m + ea, 0, 32);
}
static inline void gc_dcbz_l(uint8_t* m, uint32_t ea) { gc_dcbz(m, ea); }

/* Paired-single quantized loads and stores (GQR types 0 float, 4 u8, 5 u16, 6 s8, 7 s16). */
double gc_dequantize(uint32_t raw, uint32_t type, uint32_t scale);
uint32_t gc_quantize(double value, uint32_t type, uint32_t scale);

static inline void gc_psq_l(gc_ctx* c, uint8_t* m, uint32_t ea, uint32_t w, uint32_t i, double* ps0, double* ps1) {
    uint32_t gqr = c->gqr[i];
    uint32_t type = (gqr >> 16) & 7, scale = (gqr >> 24) & 0x3F;
    if (type == 0) {
        *ps0 = (double)gc_u2f(gc_ld32(m, ea));
        *ps1 = w ? 1.0 : (double)gc_u2f(gc_ld32(m, ea + 4));
        return;
    }
    uint32_t size = (type == 4 || type == 6) ? 1 : 2;
    uint32_t a = size == 1 ? gc_ld8(m, ea) : gc_ld16(m, ea);
    *ps0 = gc_dequantize(a, type, scale);
    if (w) {
        *ps1 = 1.0;
    } else {
        uint32_t b = size == 1 ? gc_ld8(m, ea + 1) : gc_ld16(m, ea + 2);
        *ps1 = gc_dequantize(b, type, scale);
    }
}

static inline void gc_psq_st(gc_ctx* c, uint8_t* m, uint32_t ea, uint32_t w, uint32_t i, double ps0, double ps1) {
    uint32_t gqr = c->gqr[i];
    uint32_t type = gqr & 7, scale = (gqr >> 8) & 0x3F;
    if (type == 0) {
        gc_st32(m, ea, gc_f2u((float)ps0));
        if (!w) gc_st32(m, ea + 4, gc_f2u((float)ps1));
        return;
    }
    uint32_t size = (type == 4 || type == 6) ? 1 : 2;
    uint32_t a = gc_quantize(ps0, type, scale);
    if (size == 1) gc_st8(m, ea, a); else gc_st16(m, ea, a);
    if (!w) {
        uint32_t b = gc_quantize(ps1, type, scale);
        if (size == 1) gc_st8(m, ea + 1, b); else gc_st16(m, ea + 2, b);
    }
}

#ifdef __cplusplus
}
#endif

#endif

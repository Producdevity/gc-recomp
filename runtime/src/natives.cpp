// Host implementations of SDK functions the recompilers recognise by their code
// (recompiler/natives.hpp). Each gives the results of the instruction sequence it replaces:
// the same operations in the same order, rounded where the guest rounds, with fused
// multiply-adds as the translated code issues them (addend, then the two factors).
#include "gcrt/recomp.h"
#include "gcrt/runtime.hpp"

#include <arm_neon.h>
#include <cmath>
#include <cstring>

// Products and sums stay separate operations unless the guest fuses them (std::fma).
#pragma clang fp contract(off)

namespace {

// One paired-single register: ps0 in lane 0, ps1 in lane 1.
using Pair = float64x2_t;

std::uint8_t* matrix(std::uint32_t ea, std::uint32_t size) {
    if (gc_is_io(ea) || gc_is_io(ea + size - 1)) gcrt::fatal("matrix at hardware address %08x", ea);
    return gc_mem + ea;
}
// psq_l with a float GQR: two big-endian singles.
Pair load(const std::uint8_t* p) { return vcvt_f64_f32(vreinterpret_f32_u8(vrev32_u8(vld1_u8(p)))); }
// psq_st with a float GQR.
void store(std::uint8_t* p, Pair v) { vst1_u8(p, vrev32_u8(vreinterpret_u8_f32(vcvt_f32_f64(v)))); }
// Paired-single results are rounded to single precision.
Pair rs(Pair v) { return vcvt_f64_f32(vcvt_f32_f64(v)); }
// ps_muls0/ps_muls1: a * c[lane].
template <int L> Pair muls(Pair a, Pair c) { return rs(vmulq_laneq_f64(a, c, L)); }
// ps_madds0/ps_madds1: a * c[lane] + b.
template <int L> Pair madds(Pair a, Pair c, Pair b) { return rs(vfmaq_laneq_f64(b, a, c, L)); }

// Scalar forms for the vector helpers: one single held in a double, as the FPRs hold them.
// Callers check each vector once (vec), then read and write its host bytes directly.
double rs1(double x) { return static_cast<double>(static_cast<float>(x)); }
float lf(const std::uint8_t* p) {
    std::uint32_t w;
    std::memcpy(&w, p, 4);
    return gc_u2f(__builtin_bswap32(w));
}
void sf(std::uint8_t* p, double v) {
    const std::uint32_t w = __builtin_bswap32(gc_f2u(static_cast<float>(v)));
    std::memcpy(p, &w, 4);
}
struct P { double ps0, ps1; };
P psq_l(const std::uint8_t* p) { return {lf(p), lf(p + 4)}; }
void psq_st(std::uint8_t* p, P v) { sf(p, v.ps0); sf(p + 4, v.ps1); }
void set_f(gc_ctx* c, unsigned n, P v) { c->f[n] = v.ps0; c->ps1[n] = v.ps1; }
std::uint8_t* vec(std::uint32_t ea) { return matrix(ea, 12); }
// A small-data constant (r2-relative), read through the guest address map.
double lfs(std::uint32_t ea) { return static_cast<double>(gc_u2f(gcrt::read32(ea))); }
// Two big-endian singles as a NEON pair, and back.
float32x2_t load2(const std::uint8_t* p) { return vreinterpret_f32_u8(vrev32_u8(vld1_u8(p))); }
void store2(std::uint8_t* p, float32x2_t v) { vst1_u8(p, vrev32_u8(vreinterpret_u8_f32(v))); }

// MSL's sinf and cosf as Super Mario Sunshine compiles them (trigf.c): each single-precision
// operation rounds to single and each fused multiply-add is a double fma rounded to single, in
// the compiled order. Callers observe f1 and the LR save word only.
constexpr std::uint32_t kFourOverPiM1 = 0x803E67C8u, kSinCosOnQuadrant = 0x803AB2C4u, kSinCosPoly = 0x803AB2E4u;
double madds(double a, double c, double b) { return gc_rs(std::fma(a, c, b)); }
double poly(unsigned i) { return lfs(kSinCosPoly + 4 * i); }
double quadrant(unsigned n, unsigned i) { return lfs(kSinCosOnQuadrant + 8 * n + 4 * i); }
struct Reduced { double fraction, square; unsigned n; bool small; };
// x minus the nearest multiple of pi/2, scaled to the polynomials' range, and its quadrant.
Reduced trig_reduce(gc_ctx* c) {
    gc_st32(gc_mem, c->r[1] + 4, c->lr);
    const float stored = static_cast<float>(c->f[1]);
    const double x = stored, z = gc_rs(lfs(c->r[2] + 2696) * x), half = lfs(c->r[2] + 2700);
    const std::uint32_t n = gc_saturate_s32((gc_f2u(stored) & 0x80000000u) ? gc_rs(z - half) : gc_rs(half + z));
    const double twice = gc_u2d(0x4330000000000000ull | ((n << 1) ^ 0x80000000u)) - gc_u2d(gc_ld64(gc_mem, c->r[2] + 2712));
    double f = gc_rs(x - gc_rs(twice));
    for (unsigned k = 0; k < 4; ++k) f = madds(lfs(kFourOverPiM1 + 4 * k), x, f);
    const bool small = std::fabs(f) < lfs(c->r[2] + 2704);
    return {f, small ? 0.0 : gc_rs(f * f), n & 3, small};
}
void set_f1(gc_ctx* c, double v) { c->f[1] = c->ps1[1] = v; }

} // namespace

extern "C" {

void hle_sinf(gc_ctx* c) {
    const Reduced r = trig_reduce(c);
    const double f = r.fraction, s = r.square;
    if (r.small) return set_f1(c, madds(poly(9), gc_rs(f * quadrant(r.n, 1)), quadrant(r.n, 0)));
    if (r.n & 1) {
        const double t = madds(s, madds(s, madds(s, madds(poly(0), s, poly(2)), poly(4)), poly(6)), poly(8));
        return set_f1(c, gc_rs(t * quadrant(r.n, 0)));
    }
    const double t = madds(s, madds(s, madds(s, madds(poly(1), s, poly(3)), poly(5)), poly(7)), poly(9));
    set_f1(c, gc_rs(gc_rs(f * t) * quadrant(r.n, 1)));
}

void hle_cosf(gc_ctx* c) {
    const Reduced r = trig_reduce(c);
    const double f = r.fraction, s = r.square;
    if (r.small) return set_f1(c, gc_rs(-std::fma(f, quadrant(r.n, 0), -quadrant(r.n, 1))));
    if (r.n & 1) {
        const double t = gc_rs(-std::fma(s, madds(s, madds(s, madds(poly(1), s, poly(3)), poly(5)), poly(7)), poly(9)));
        return set_f1(c, gc_rs(gc_rs(f * t) * quadrant(r.n, 0)));
    }
    const double t = madds(s, madds(s, madds(s, madds(poly(0), s, poly(2)), poly(4)), poly(6)), poly(8));
    set_f1(c, gc_rs(t * quadrant(r.n, 1)));
}

// PSMTXConcat(a = r3, b = r4, ab = r5): ab = a * b for 3x4 matrices. The guest code loads
// every element before its first store, so aliased arguments behave the same. f1 and f2 are
// left as the guest code leaves them.
void hle_PSMTXConcat(gc_ctx* c) {
    const std::uint8_t* a = matrix(c->r[3], 48);
    const std::uint8_t* b = matrix(c->r[4], 48);
    std::uint8_t* ab = matrix(c->r[5], 48);
    const Pair a0 = load(a), a1 = load(a + 8), a2 = load(a + 16), a3 = load(a + 24), a4 = load(a + 32), a5 = load(a + 40);
    const Pair b0 = load(b), b1 = load(b + 8), b2 = load(b + 16), b3 = load(b + 24), b4 = load(b + 32), b5 = load(b + 40);
    const Pair unit = {0.0, 1.0};   // the SDK's Unit01 constant
    Pair r0 = muls<0>(b0, a0), r1 = muls<0>(b1, a0), r2 = muls<0>(b0, a2), r3 = muls<0>(b1, a2);
    r0 = madds<1>(b2, a0, r0);
    r2 = madds<1>(b2, a2, r2);
    r1 = madds<1>(b3, a0, r1);
    r3 = madds<1>(b3, a2, r3);
    r0 = madds<0>(b4, a1, r0);
    r1 = madds<0>(b5, a1, r1);
    r2 = madds<0>(b4, a3, r2);
    r3 = madds<0>(b5, a3, r3);
    store(ab, r0);
    Pair r4 = muls<0>(b0, a4);
    r1 = madds<1>(unit, a1, r1);
    Pair r5 = muls<0>(b1, a4);
    store(ab + 16, r2);
    r3 = madds<1>(unit, a3, r3);
    store(ab + 8, r1);
    r4 = madds<1>(b2, a4, r4);
    r5 = madds<1>(b3, a4, r5);
    r4 = madds<0>(b4, a5, r4);
    store(ab + 24, r3);
    r5 = madds<0>(b5, a5, r5);
    store(ab + 32, r4);
    r5 = madds<1>(unit, a5, r5);
    store(ab + 40, r5);
    c->f[1] = vgetq_lane_f64(a1, 0);
    c->ps1[1] = vgetq_lane_f64(a1, 1);
    c->f[2] = vgetq_lane_f64(r4, 0);
    c->ps1[2] = vgetq_lane_f64(r4, 1);
}

// PSVECAdd(a = r3, b = r4, ab = r5) and PSVECSubtract: xy as a pair, then z (W=1 loads
// leave ps1 = 1.0, which the W=1 store drops). Returns with f2 = a.xy, as the guest does.
// A single-precision sum of two singles equals their double sum rounded once, so the pair
// arithmetic runs in single precision.
static void vec_add_sub(gc_ctx* c, bool subtract) {
    const std::uint8_t* a = vec(c->r[3]);
    const std::uint8_t* b = vec(c->r[4]);
    std::uint8_t* ab = vec(c->r[5]);
    const float32x2_t axy = load2(a), bxy = load2(b);
    store2(ab, subtract ? vsub_f32(axy, bxy) : vadd_f32(axy, bxy));
    const float az = lf(a + 8), bz = lf(b + 8);
    sf(ab + 8, subtract ? az - bz : az + bz);
    c->f[2] = vget_lane_f32(axy, 0);
    c->ps1[2] = vget_lane_f32(axy, 1);
}
void hle_PSVECAdd(gc_ctx* c) { vec_add_sub(c, false); }
void hle_PSVECSubtract(gc_ctx* c) { vec_add_sub(c, true); }

// PSVECScale(src = r3, dst = r4, scale = f1). The scale is a double, so products round twice.
void hle_PSVECScale(gc_ctx* c) {
    const std::uint8_t* src = vec(c->r[3]);
    std::uint8_t* dst = vec(c->r[4]);
    const double s = c->f[1];
    const float32x2_t xy = load2(src);
    store2(dst, vcvt_f32_f64(vmulq_n_f64(vcvt_f64_f32(xy), s)));
    sf(dst + 8, rs1(lf(src + 8) * s));
    c->f[2] = vget_lane_f32(xy, 0);
    c->ps1[2] = vget_lane_f32(xy, 1);
}

// PSVECSquareMag(v = r3) -> f1.
void hle_PSVECSquareMag(gc_ctx* c) {
    const std::uint8_t* v = vec(c->r[3]);
    const P xy = psq_l(v);
    const P sq = {rs1(xy.ps0 * xy.ps0), rs1(xy.ps1 * xy.ps1)};
    const double z = lf(v + 8);
    const P acc = {rs1(std::fma(z, z, sq.ps0)), rs1(std::fma(z, z, sq.ps1))};
    set_f(c, 1, {rs1(acc.ps0 + sq.ps1), rs1(sq.ps1)});
    set_f(c, 2, xy);
}

// PSVECNormalize(src = r3, dst = r4): one Newton step on the reciprocal square root with the
// 0.5 and 3.0 constants the guest loads from its small-data area (r2).
void hle_PSVECNormalize(gc_ctx* c) {
    const std::uint8_t* src = vec(c->r[3]);
    std::uint8_t* dst = vec(c->r[4]);
    const double half = lfs(c->r[2] - 16728), three = lfs(c->r[2] - 16724);
    const P xy = psq_l(src);
    const P sq = {rs1(xy.ps0 * xy.ps0), rs1(xy.ps1 * xy.ps1)};
    const P z = {lf(src + 8), 1.0};
    const P acc = {rs1(std::fma(z.ps0, z.ps0, sq.ps0)), rs1(std::fma(z.ps1, z.ps1, sq.ps1))};
    const P mag = {rs1(acc.ps0 + sq.ps1), rs1(z.ps1)};
    const double est = rs1(1.0 / std::sqrt(mag.ps0));
    const double est2 = rs1(est * est);
    const double h = rs1(est * half);
    const double t = rs1(est2 * mag.ps0);
    const double u = rs1(three - t);
    const double k = rs1(u * h);
    const P out = {rs1(xy.ps0 * k), rs1(xy.ps1 * k)};
    psq_st(dst, out);
    sf(dst + 8, rs1(z.ps0 * k));
    set_f(c, 1, {three, three});
    set_f(c, 2, out);
}

// PSVECCrossProduct(a = r3, b = r4, axb = r5).
void hle_PSVECCrossProduct(gc_ctx* c) {
    const std::uint8_t* a = vec(c->r[3]);
    const std::uint8_t* b = vec(c->r[4]);
    std::uint8_t* axb = vec(c->r[5]);
    const P bxy = psq_l(b);
    const double az = lf(a + 8);
    const P axy = psq_l(a);
    const P byx = {bxy.ps1, bxy.ps0};
    const double bz = lf(b + 8);
    const P t4 = {rs1(bxy.ps0 * az), rs1(bxy.ps1 * az)};
    const P t7 = {rs1(bxy.ps0 * axy.ps0), rs1(bxy.ps1 * axy.ps0)};
    const P t5 = {rs1(std::fma(axy.ps0, bz, -t4.ps0)), rs1(std::fma(axy.ps1, bz, -t4.ps1))};
    const P t8 = {rs1(std::fma(axy.ps0, byx.ps0, -t7.ps0)), rs1(std::fma(axy.ps1, byx.ps1, -t7.ps1))};
    sf(axb, t5.ps1);
    psq_st(axb + 4, {-t5.ps0, -t8.ps1});
    set_f(c, 1, bxy);
    set_f(c, 2, {az, az});
}

// PSVECMag(v = r3) -> f1: PSVECSquareMag, then, when the square is above the r2 zero, three
// Newton steps on frsqrte (1 / sqrt, as translated) with the r2 constants 0.5 and 3.0; the
// result goes through a single store and load. f2 keeps the last step's scaled estimate.
void hle_PSVECMag(gc_ctx* c) {
    hle_PSVECSquareMag(c);
    const double x = c->f[1];
    if (!(x > lfs(c->r[2] - 16752))) return;
    const double half = gc_u2d(gc_ld64(gc_mem, c->r[2] - 16744)), three = gc_u2d(gc_ld64(gc_mem, c->r[2] - 16736));
    double e = 1.0 / std::sqrt(x), t;
    t = e * e; e = half * e; t = x * t; t = three - t; e = e * t;
    t = e * e; e = half * e; t = x * t; t = three - t; e = e * t;
    t = e * e; e = half * e; t = x * t; t = three - t; t = e * t; t = x * t;
    c->f[1] = c->ps1[1] = rs1(t);
    c->f[2] = e;
}

// WriteMTXPS4x3(m = r3, dst = r4) and WriteMTXPS3x3from3x4: each row's pairs (and, for the
// 3x3 form, its third element as a single) stored to dst in order, through the same store
// path as the translated psq_st and stfs, so gather-pipe bursts split at the same bytes.
// A load and store pair passes a single through double precision, which quiets a
// signalling NaN; the words are copied with that change.
// A store of size bytes (8 or 4) of v to dst as the translated psq_st/stfs makes it: to the
// write-gather buffer for the pipe address, otherwise through the ordinary store path.
static inline void store_fifo(std::uint32_t dst, std::uint64_t v, unsigned size) {
    if (dst != 0xCC008000u) {
        if (size == 8) gc_st64(gc_mem, dst, v);
        else gc_st32(gc_mem, dst, static_cast<std::uint32_t>(v));
        return;
    }
    const std::uint32_t f = gc_gp.fill;
    if (size == 8) {
        const std::uint64_t be = __builtin_bswap64(v);
        std::memcpy(gc_gp.bytes + f, &be, 8);
    } else {
        const std::uint32_t be = __builtin_bswap32(static_cast<std::uint32_t>(v));
        std::memcpy(gc_gp.bytes + f, &be, 4);
    }
    gc_gp.fill = f + size;
    if (f + size >= GC_GATHER_BATCH) gc_gather_burst();
}
static std::uint32_t single_word(const std::uint8_t* p) {
    std::uint32_t w;
    std::memcpy(&w, p, 4);
    w = __builtin_bswap32(w);
    if ((w & 0x7F800000u) == 0x7F800000u && (w & 0x007FFFFFu)) w |= 0x00400000u;
    return w;
}
void hle_WriteMTXPS4x3(gc_ctx* c) {
    const std::uint8_t* m = matrix(c->r[3], 48);
    const std::uint32_t dst = c->r[4];
    std::uint32_t w[12];
    for (unsigned k = 0; k < 12; ++k) w[k] = single_word(m + 4 * k);
    for (unsigned k = 0; k < 6; ++k) store_fifo(dst, (std::uint64_t{w[2 * k]} << 32) | w[2 * k + 1], 8);
    const Pair r1 = load(m + 8), r2 = load(m + 16);
    c->f[1] = vgetq_lane_f64(r1, 0);
    c->ps1[1] = vgetq_lane_f64(r1, 1);
    c->f[2] = vgetq_lane_f64(r2, 0);
    c->ps1[2] = vgetq_lane_f64(r2, 1);
}
void hle_WriteMTXPS3x3from3x4(gc_ctx* c) {
    const std::uint8_t* m = matrix(c->r[3], 48);
    const std::uint32_t dst = c->r[4];
    std::uint32_t w[12];
    for (unsigned k = 0; k < 12; ++k) w[k] = single_word(m + 4 * k);
    for (unsigned row = 0; row < 3; ++row) {
        store_fifo(dst, (std::uint64_t{w[4 * row]} << 32) | w[4 * row + 1], 8);
        store_fifo(dst, w[4 * row + 2], 4);
    }
    const Pair xy = load(m + 16);
    c->f[1] = c->ps1[1] = lf(m + 8);
    c->f[2] = vgetq_lane_f64(xy, 0);
    c->ps1[2] = vgetq_lane_f64(xy, 1);
}

// VecNormalizeSafe(src = r3, dst = r4): below the r2 threshold, dst gets the r2 zero (z, y,
// x); otherwise src * (1 / magnitude) in single precision, element by element as the guest
// reads and writes them.
void hle_VecNormalizeSafe(gc_ctx* c) {
    const std::uint32_t src = c->r[3], dst = c->r[4];
    hle_PSVECMag(c);
    const double mag = c->f[1];
    if (mag < lfs(c->r[2] - 28548)) {
        const double zero = lfs(c->r[2] - 28544);
        gc_st32(gc_mem, dst + 8, gc_f2u(static_cast<float>(zero)));
        gc_st32(gc_mem, dst + 4, gc_f2u(static_cast<float>(zero)));
        gc_st32(gc_mem, dst, gc_f2u(static_cast<float>(zero)));
        return;
    }
    const double one = lfs(c->r[2] - 28540);
    const double x = lfs(src);
    const double inv = rs1(one / mag);
    gc_st32(gc_mem, dst, gc_f2u(static_cast<float>(rs1(x * inv))));
    gc_st32(gc_mem, dst + 4, gc_f2u(static_cast<float>(rs1(lfs(src + 4) * inv))));
    gc_st32(gc_mem, dst + 8, gc_f2u(static_cast<float>(rs1(lfs(src + 8) * inv))));
    c->f[2] = c->ps1[2] = inv;
}

// NameHashFind(set = r3, name = r4): table = set[1], count = table[1], 12-byte entries from
// table + 16 holding the hash. Returns r3 = index (-1 when absent) and r4 = the entry's
// byte offset (12 * count when absent, 0 for an empty table), as the guest leaves them. The
// scan reads RAM directly when the whole table is RAM, four entries per step.
// The hash of a name: h = h * 256 + signed byte, reduced modulo 33554393 through the guest's
// multiply-high, byte by byte.
static std::uint32_t hash_name(std::uint32_t p) {
    std::uint32_t h = 0;
    for (;; ++p) {
        const std::int8_t ch = static_cast<std::int8_t>(gc_ld8(gc_mem, p));
        if (!ch) return h;
        h = static_cast<std::uint32_t>(static_cast<std::int32_t>(ch)) + (h << 8);
        const std::uint32_t q = static_cast<std::uint32_t>((std::uint64_t{4993} * h) >> 32);
        h -= ((((h - q) >> 1) + q) >> 24) * 33554393u;
    }
}
// Names come back every frame from the same addresses: a cached hash is reused when the
// bytes at that address still equal the cached copy (RAM names up to 31 bytes).
struct NameHash {
    std::uint32_t address = 0, hash = 0, length = 0;
    std::uint8_t bytes[32];
};
static NameHash name_cache[256];
static std::uint32_t cached_name_hash(std::uint32_t p) {
    if (gc_is_io(p) || gc_is_io(p + 31) || ((p ^ (p + 31)) >> 27)) return hash_name(p);
    const std::uint8_t* name = gc_mem + p;
    NameHash& e = name_cache[(p >> 2) & 255];
    if (e.address == p && !std::memcmp(e.bytes, name, e.length + 1)) return e.hash;
    const void* end = std::memchr(name, 0, sizeof e.bytes);
    if (!end) return hash_name(p);
    e.address = p;
    e.length = static_cast<std::uint32_t>(static_cast<const std::uint8_t*>(end) - name);
    std::memcpy(e.bytes, name, e.length + 1);
    e.hash = hash_name(p);
    return e.hash;
}
static void name_hash_find(gc_ctx* c) {
    const std::uint32_t table = gc_ld32(gc_mem, c->r[3] + 4);
    const std::int32_t count = static_cast<std::int32_t>(gc_ld32(gc_mem, table + 4));
    const std::uint32_t h = cached_name_hash(c->r[4]);
    if (count <= 0) {
        c->r[3] = 0xFFFFFFFFu;
        c->r[4] = 0;
        return;
    }
    const std::uint32_t n = static_cast<std::uint32_t>(count), first = table + 16, last = first + 12 * (n - 1) + 3;
    std::uint32_t i = 0;
    if (n < (1u << 20) && !gc_is_io(first) && (first >> 27) == (last >> 27) && last > first) {
        const std::uint8_t* e = gc_mem + first;
        auto word = [e](std::uint32_t k) {
            std::uint32_t w;
            std::memcpy(&w, e + 12 * k, 4);
            return __builtin_bswap32(w);
        };
        for (; i + 4 <= n; i += 4) {
            const std::uint32_t w0 = word(i), w1 = word(i + 1), w2 = word(i + 2), w3 = word(i + 3);
            if ((w0 == h) | (w1 == h) | (w2 == h) | (w3 == h)) break;
        }
        for (; i < n; ++i)
            if (word(i) == h) break;
    } else {
        for (; i < n; ++i)
            if (gc_ld32(gc_mem, first + 12 * i) == h) break;
    }
    c->r[3] = i < n ? i : 0xFFFFFFFFu;
    c->r[4] = 12 * i;
}
void hle_NameHashFind(gc_ctx* c) { name_hash_find(c); }

// NameParamGet(set = r3, record = r4, name = r5, out = r6): r3 = 0 when the name is absent;
// otherwise *out = (record word & mask) >> shift (0 for shifts of 32 and up), r3 = 1 and r4 =
// the masked word. The table header holds the data base (+8) and record size (+12); the entry
// the mask (+4), the field offset (+8, 16 bits) and the shift (+10, 8 bits).
void hle_NameParamGet(gc_ctx* c) {
    const std::uint32_t set = c->r[3], record = c->r[4], out = c->r[6];
    c->r[4] = c->r[5];
    name_hash_find(c);
    if (static_cast<std::int32_t>(c->r[3]) < 0) {
        c->r[3] = 0;
        return;
    }
    const std::uint32_t table = gc_ld32(gc_mem, set + 4);
    const std::uint32_t entry = table + c->r[3] * 12 + 16;
    const std::uint32_t size = gc_ld32(gc_mem, table + 12), base = gc_ld32(gc_mem, table + 8);
    const std::uint32_t field = gc_ld16(gc_mem, entry + 8), mask = gc_ld32(gc_mem, entry + 4);
    const std::uint32_t word = gc_ld32(gc_mem, table + base + record * size + field);
    const std::uint32_t shift = gc_ld8(gc_mem, entry + 10) & 63;
    const std::uint32_t masked = word & mask;
    gc_st32(gc_mem, out, shift & 32 ? 0 : masked >> shift);
    c->r[3] = 1;
    c->r[4] = masked;
}

// FindEntryByKey(key = r3), with its callees EntryRange, EntryRangeFrom and EntryKey (see
// recompiler/natives.hpp): r3 = the object of the first type-2 entry whose object holds key at
// +0x808, else 0; r4 = the table base, as EntryRange leaves it.
void hle_FindEntryByKey(gc_ctx* c) {
    const std::uint32_t key = c->r[3];
    const std::uint32_t base = gc_ld32(gc_mem, gc_ld32(gc_mem, c->r[13] + 0xD00) + 8);
    const std::uint32_t first = base + 4, end = base + 0x804;
    std::uint32_t found = 0;
    for (std::uint32_t p = first; p != end; p += 16) {
        const std::uint32_t type = gc_ld32(gc_mem, p + 4), object = gc_ld32(gc_mem, p);
        if (type != 2) continue;
        if ((object ? gc_ld32(gc_mem, object + 0x808) : 0) == key) {
            found = object;
            break;
        }
    }
    c->r[3] = found;
    c->r[4] = base;
}

} // extern "C"

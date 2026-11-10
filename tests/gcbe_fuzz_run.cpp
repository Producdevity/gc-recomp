// Runs the differential test program through the C path (linked) and the AArch64 backend
// (loaded from the image given on the command line) and compares the results.
// Usage: gc-gcbe-fuzz IMAGE SEED [STATES]
#include "gcbe_fuzz.hpp"

#include "gcbe/loader.h"
#include "gcrt/runtime.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {

std::mt19937_64 rng(20261110);

double fp_value() {
    static const std::uint64_t specials[] = {
        0x0000000000000000ull, 0x8000000000000000ull, 0x3FF0000000000000ull, 0xBFF0000000000000ull,
        0x7FF0000000000000ull, 0xFFF0000000000000ull, 0x7FF8000000000000ull, 0xFFF8000000000001ull,
        0x7FF0000000000001ull, 0x0000000000000001ull, 0x47EFFFFFE0000000ull, 0x3FE0000000000000ull,
        0x41DFFFFFFFC00000ull, 0xC1E0000000000000ull, 0x41E0000000000000ull, 0x380FFFFFFFFFFFFFull,
    };
    std::uint64_t bits;
    switch (rng() % 6) {
    case 0: bits = specials[rng() % (sizeof specials / sizeof specials[0])]; break;
    case 1: { double d = static_cast<double>(static_cast<int>(rng() % 64) - 32); std::memcpy(&bits, &d, 8); break; }
    case 2: { std::uint32_t f = static_cast<std::uint32_t>(rng()); float x; std::memcpy(&x, &f, 4); double d = x; std::memcpy(&bits, &d, 8); break; }
    case 3: { double d = static_cast<double>(static_cast<float>((rng() % 2000000) / 1000.0 - 1000.0)); std::memcpy(&bits, &d, 8); break; }
    default: bits = rng(); break;
    }
    double d;
    std::memcpy(&d, &bits, 8);
    return d;
}

std::uint32_t int_value() {
    switch (rng() % 5) {
    case 0: return static_cast<std::uint32_t>(rng() % 64);
    case 1: { static const std::uint32_t s[] = {0, 1, 0xFFFFFFFFu, 0x80000000u, 0x7FFFFFFFu, 0xFFFF, 0x10000, 31, 32, 33, 63}; return s[rng() % 11]; }
    default: return static_cast<std::uint32_t>(rng());
    }
}

std::uint32_t gqr_value() {
    static const std::uint32_t types[] = {0, 4, 5, 6, 7};
    std::uint32_t ld = types[rng() % 5], st = types[rng() % 5];
    return (static_cast<std::uint32_t>(rng() % 64) << 24) | (ld << 16) | (static_cast<std::uint32_t>(rng() % 64) << 8) | st;
}

struct Region { std::uint32_t at, size; };
const Region kRegions[] = {{gcbef::kData, 0x10000}, {gcbef::kStackLow, 0x1100}};

void randomize_memory() {
    for (auto& r : kRegions)
        for (std::uint32_t o = 0; o < r.size; o += 8) {
            std::uint64_t v = (rng() % 4 == 0) ? 0 : rng();
            std::memcpy(gcrt::host(r.at + o), &v, 8);
        }
}

std::vector<std::uint8_t> snapshot() {
    std::vector<std::uint8_t> s;
    for (auto& r : kRegions) s.insert(s.end(), gcrt::host(r.at), gcrt::host(r.at) + r.size);
    return s;
}

void restore(const std::vector<std::uint8_t>& s) {
    std::size_t o = 0;
    for (auto& r : kRegions) {
        std::memcpy(gcrt::host(r.at), s.data() + o, r.size);
        o += r.size;
    }
}

// NaN sign and payload depend on how the C compiler folded the expression (it may turn
// fma(1.0, c, -b) into c - b); they are not compared.
bool is_nan64(std::uint64_t v) { return (v & 0x7FF0000000000000ull) == 0x7FF0000000000000ull && (v & 0xFFFFFFFFFFFFFull); }
bool is_nan32(std::uint32_t v) { return (v & 0x7F800000u) == 0x7F800000u && (v & 0x7FFFFFu); }
bool same_fp(double x, double y) {
    std::uint64_t a = gc_d2u(x), b = gc_d2u(y);
    return a == b || (is_nan64(a) && is_nan64(b));
}
// Direct calls between translated functions hand arguments over in host registers, so the
// context's r5-r10 and f3-f8 (both halves) keep stale values where the C path stores the
// arguments. Exact mode synchronises everything and compares them too.
void contract_view(gc_ctx& backend, const gc_ctx& c) {
#ifndef GCBE_FUZZ_EXACT
    for (unsigned n = 5; n <= 10; ++n) backend.r[n] = c.r[n];
    for (unsigned n = 3; n <= 8; ++n) {
        backend.f[n] = c.f[n];
        backend.ps1[n] = c.ps1[n];
    }
#else
    (void)backend;
    (void)c;
#endif
}

bool same_ctx(const gc_ctx& a, const gc_ctx& b) {
    gc_ctx x = a, y = b;
    contract_view(y, x);
    for (unsigned n = 0; n < 32; ++n) {
        if (same_fp(x.f[n], y.f[n])) y.f[n] = x.f[n];
        if (same_fp(x.ps1[n], y.ps1[n])) y.ps1[n] = x.ps1[n];
    }
    return std::memcmp(&x, &y, sizeof x) == 0;
}
bool same_memory(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    for (std::size_t o = 0; o + 8 <= a.size(); o += 4) {
        if (!std::memcmp(&a[o], &b[o], 4)) continue;
        auto be32 = [](const std::uint8_t* p) { return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3]; };
        std::uint32_t x = be32(&a[o]), y = be32(&b[o]);
        if (is_nan32(x) && is_nan32(y)) continue;
        // Double NaNs: the differing word is either half of an 8-byte value.
        auto nan64_at = [&](const std::vector<std::uint8_t>& m, std::size_t at) {
            return is_nan64((std::uint64_t(be32(&m[at])) << 32) | be32(&m[at + 4]));
        };
        if (nan64_at(a, o) && nan64_at(b, o)) { o += 4; continue; }
        if (o >= 4 && nan64_at(a, o - 4) && nan64_at(b, o - 4)) continue;
        return false;
    }
    return true;
}

void report(const gc_ctx& a, const gc_ctx& backend) {
    gc_ctx b = backend;
    contract_view(b, a);
    for (unsigned n = 0; n < 32; ++n)
        if (a.r[n] != b.r[n]) std::printf("    r%u: c %08x backend %08x\n", n, a.r[n], b.r[n]);
    for (unsigned n = 0; n < 32; ++n)
        if (std::memcmp(&a.f[n], &b.f[n], 8)) std::printf("    f%u: c %016llx backend %016llx\n", n, (unsigned long long)gc_d2u(a.f[n]), (unsigned long long)gc_d2u(b.f[n]));
    for (unsigned n = 0; n < 32; ++n)
        if (std::memcmp(&a.ps1[n], &b.ps1[n], 8)) std::printf("    ps1_%u: c %016llx backend %016llx\n", n, (unsigned long long)gc_d2u(a.ps1[n]), (unsigned long long)gc_d2u(b.ps1[n]));
    for (unsigned n = 0; n < 32; ++n)
        if (a.cr[n] != b.cr[n]) std::printf("    cr bit %u: c %u backend %u\n", n, a.cr[n], b.cr[n]);
    if (a.lr != b.lr) std::printf("    lr: c %08x backend %08x\n", a.lr, b.lr);
    if (a.ctr != b.ctr) std::printf("    ctr: c %08x backend %08x\n", a.ctr, b.ctr);
    if (a.ca != b.ca || a.so != b.so || a.ov != b.ov || a.bc != b.bc)
        std::printf("    xer: c ca %u so %u ov %u bc %u, backend ca %u so %u ov %u bc %u\n", a.ca, a.so, a.ov, a.bc, b.ca, b.so, b.ov, b.bc);
    if (a.fpscr != b.fpscr) std::printf("    fpscr: c %08x backend %08x\n", a.fpscr, b.fpscr);
    for (unsigned n = 0; n < 8; ++n)
        if (a.gqr[n] != b.gqr[n]) std::printf("    gqr%u: c %08x backend %08x\n", n, a.gqr[n], b.gqr[n]);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s IMAGE SEED [STATES]\n", argv[0]);
        return 2;
    }
    unsigned states = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : 8;
    if (!gcrt::memory_init()) return 1;
    gcrt::cpu_init(gcrt::linked_program());
    auto fns = gcbef::program(static_cast<std::uint32_t>(std::strtoul(argv[2], nullptr, 0)));
    char error[256];
    gcbe_program* p = gcbe_load(argv[1], error, sizeof error);
    if (!p) {
        std::fprintf(stderr, "%s\n", error);
        return 1;
    }
    std::map<std::uint32_t, gc_func> c_funcs, b_funcs;
    for (unsigned i = 0; i < gc_func_count; ++i) c_funcs[gc_func_table[i].start] = gc_func_table[i].func;
    for (unsigned i = 0; i < gcbe_function_count(p); ++i) b_funcs[gcbe_functions(p)[i].start] = gcbe_functions(p)[i].func;
    unsigned failures = 0, runs = 0;
    // GCBE_FUZZ_ONLY=N[:S] runs one function (and state) and prints its input state.
    long only = -1, only_state = -1;
    if (const char* o = std::getenv("GCBE_FUZZ_ONLY")) {
        only = std::strtol(o, nullptr, 0);
        if (const char* colon = std::strchr(o, ':')) only_state = std::strtol(colon + 1, nullptr, 0);
    }
    for (std::size_t n = 0; n < fns.size(); ++n) {
        gc_func cf = c_funcs[fns[n].address], bf = b_funcs[fns[n].address];
        if (!cf || !bf) {
            std::printf("function %zu (%08x) missing\n", n, fns[n].address);
            return 1;
        }
        for (unsigned s = 0; s < states; ++s) {
            gc_ctx init{};
            init.mem = gc_mem;
            for (unsigned r = 0; r < 32; ++r) init.r[r] = int_value();
            init.r[1] = gcbef::kStack;
            init.r[10] = gcbef::kData + 0x1000;
            init.r[11] = static_cast<std::uint32_t>(rng() % 0x40) * 4;
            for (unsigned r = 0; r < 32; ++r) {
                init.f[r] = fp_value();
                init.ps1[r] = rng() % 4 ? fp_value() : init.f[r];
                init.cr[r] = rng() & 1;
            }
            init.lr = int_value();
            init.ctr = int_value();
            init.ca = rng() & 1;
            init.so = rng() & 1;
            init.ov = rng() & 1;
            init.bc = rng() & 0x7F;
            init.fpscr = static_cast<std::uint32_t>(rng());
            for (unsigned g = 1; g < 8; ++g) init.gqr[g] = gqr_value();   // GQR0 stays zero (EABI)
            randomize_memory();
            if (only >= 0 && (static_cast<long>(n) != only || (only_state >= 0 && static_cast<long>(s) != only_state))) continue;
            if (only >= 0) {
                std::printf("state %u:\n", s);
                for (unsigned r = 0; r < 32; ++r)
                    std::printf("  r%u %08x f%u %016llx ps1_%u %016llx cr%u %u\n", r, init.r[r], r,
                                (unsigned long long)gc_d2u(init.f[r]), r, (unsigned long long)gc_d2u(init.ps1[r]), r, init.cr[r]);
                std::printf("  lr %08x ctr %08x ca %u so %u ov %u fpscr %08x\n", init.lr, init.ctr, init.ca, init.so, init.ov, init.fpscr);
                for (unsigned g = 0; g < 8; ++g) std::printf("  gqr%u %08x\n", g, init.gqr[g]);
            }
            auto before = snapshot();
            gc_ctx a = init, b = init;
            cf(&a);
            auto mem_c = snapshot();
            restore(before);
            bf(&b);
            auto mem_b = snapshot();
            ++runs;
            bool ctx_ok = same_ctx(a, b), mem_ok = same_memory(mem_c, mem_b);
            if (ctx_ok && mem_ok) continue;
            if (++failures <= 12) {
                std::printf("MISMATCH function %zu at %08x, state %u%s\n", n, fns[n].address, s, mem_ok ? "" : " (memory)");
                report(a, b);
                if (!mem_ok)
                    for (std::size_t o = 0; o < mem_c.size(); o += 4)
                        if (std::memcmp(&mem_c[o], &mem_b[o], 4)) {
                            std::uint32_t at = o < kRegions[0].size ? kRegions[0].at + o : kRegions[1].at + (o - kRegions[0].size);
                            std::printf("    mem %08x: c %02x%02x%02x%02x backend %02x%02x%02x%02x\n", at, mem_c[o], mem_c[o + 1],
                                        mem_c[o + 2], mem_c[o + 3], mem_b[o], mem_b[o + 1], mem_b[o + 2], mem_b[o + 3]);
                            break;
                        }
            }
        }
    }
    std::printf("%u runs over %zu functions, %u mismatches\n", runs, fns.size(), failures);
    return failures ? 1 : 0;
}

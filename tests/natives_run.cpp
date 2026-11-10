// Compares the runtime's replacements of SDK functions with the translated guest code they
// replace, bit for bit, over random and special-value inputs.
#include "../recompiler/natives.hpp"
#include "gcrt/runtime.hpp"
#include "gcbe/loader.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <string>

namespace {

std::mt19937 rng(12345);

float pick_float() {
    static const float special[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, 3.0f, std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::max(),
                                    std::numeric_limits<float>::min(), std::numeric_limits<float>::denorm_min(), 1e-30f, 3e38f};
    switch (rng() % 10) {
    case 0: return special[rng() % (sizeof special / sizeof special[0])];
    case 1: {   // NaNs, quiet and signalling, with payloads
        std::uint32_t bits = 0x7F800000u | (rng() & 0x807FFFFFu);
        if (!(bits & 0x7FFFFF)) bits |= 1;
        float f;
        std::memcpy(&f, &bits, 4);
        return f;
    }
    case 2: {   // any bit pattern
        std::uint32_t bits = rng();
        float f;
        std::memcpy(&f, &bits, 4);
        return f;
    }
    default: return std::ldexp(static_cast<float>(static_cast<std::int32_t>(rng() % 2000001) - 1000000) / 1e6f,
                               static_cast<int>(rng() % 40) - 20);
    }
}
void fill(std::uint32_t at, unsigned words) {
    for (unsigned k = 0; k < words; ++k) {
        float f = pick_float();
        std::uint32_t bits;
        std::memcpy(&bits, &f, 4);
        gcrt::write32(at + 4 * k, bits);
    }
}
std::uint64_t bits(double d) {
    std::uint64_t u;
    std::memcpy(&u, &d, 8);
    return u;
}
// As the differential fuzz: any two NaNs are equal (neither the translated code nor the
// host compiler follows Gekko's NaN propagation, and the compiler may swap commutative
// operands, which picks a different NaN).
bool same_double(double a, double b) { return bits(a) == bits(b) || (std::isnan(a) && std::isnan(b)); }
bool same_words(const std::uint8_t* a, const std::uint8_t* b, std::uint32_t size) {
    for (std::uint32_t o = 0; o < size; o += 4) {
        if (!std::memcmp(a + o, b + o, 4)) continue;
        std::uint32_t x, y;
        std::memcpy(&x, a + o, 4);
        std::memcpy(&y, b + o, 4);
        x = __builtin_bswap32(x);
        y = __builtin_bswap32(y);
        const bool nan_x = (x & 0x7F800000u) == 0x7F800000u && (x & 0x7FFFFF), nan_y = (y & 0x7F800000u) == 0x7F800000u && (y & 0x7FFFFF);
        if (!(nan_x && nan_y)) return false;
    }
    return true;
}

constexpr std::uint32_t kArea = 0x80300000u, kA = 0x80300000u, kB = 0x80300100u,
                        kOut = 0x80300200u, kTable = 0x80301000u, kName = 0x80302000u, kData = 0x80303000u,
                        kSda2 = 0x80310000u, kStack = 0x80380000u;
constexpr std::uint32_t kAreaSize = 0x10000;   // arguments, tables, records and small-data constants
constexpr std::uint32_t kFifo = 0xCC008000u;

void put_float(std::uint32_t at, float f) {
    std::uint32_t w;
    std::memcpy(&w, &f, 4);
    gcrt::write32(at, w);
}
void put_double(std::uint32_t at, double d) {
    std::uint64_t w;
    std::memcpy(&w, &d, 8);
    gcrt::write32(at, static_cast<std::uint32_t>(w >> 32));
    gcrt::write32(at + 4, static_cast<std::uint32_t>(w));
}
// The guest's name hash, written from its description, to plant matching entries.
std::uint32_t name_hash(const char* s) {
    std::uint32_t h = 0;
    for (; *s; ++s) {
        h = (h << 8) + static_cast<std::uint32_t>(static_cast<std::int32_t>(static_cast<std::int8_t>(*s)));
        h %= 33554393u;
    }
    return h;
}
// A name set: set[1] -> table {+4 count, +8 data base, +12 record size, entries from +16:
// +0 hash, +4 mask, +8 field offset (16 bits), +10 shift}, and a NUL-terminated name.
void lookup_case(gc_ctx& g, unsigned run) {
    gcrt::write32(kA + 4, kTable);
    const std::int32_t count = run % 17 == 0 ? -static_cast<std::int32_t>(rng() % 3) : static_cast<std::int32_t>(rng() % 41);
    gcrt::write32(kTable + 4, static_cast<std::uint32_t>(count));
    gcrt::write32(kTable + 8, kData - kTable);
    gcrt::write32(kTable + 12, 4 * (1 + rng() % 8));
    // Half the runs keep the previous name at the same address (the natives cache its hash),
    // the others write a new one there.
    static char name[16] = {};
    if (run % 2 == 0) {
        const unsigned length = rng() % 12;
        for (unsigned k = 0; k < length; ++k) name[k] = static_cast<char>(rng() % 255 + 1);
        name[length] = 0;
    }
    for (unsigned k = 0; k <= std::strlen(name); ++k) gcrt::write8(kName + k, static_cast<std::uint8_t>(name[k]));
    const std::uint32_t h = name_hash(name);
    for (std::int32_t i = 0; i < 40; ++i) {
        const std::uint32_t e = kTable + 16 + 12 * static_cast<std::uint32_t>(i);
        gcrt::write32(e, rng() % 4 ? rng() : h ^ 1);
        gcrt::write32(e + 4, rng() % 3 ? rng() : 0xFFFFFFFFu);
        gcrt::write16(e + 8, (rng() % 64) * 4);
        gcrt::write8(e + 10, rng() % 40);
    }
    if (count > 0 && run % 4) gcrt::write32(kTable + 16 + 12 * (rng() % count), h);
    for (unsigned k = 0; k < 0x800; k += 4) gcrt::write32(kData + k, rng());
    g.r[3] = kA;
    g.r[4] = kName;
}
// FindEntryByKey's world: r13 + 0xD00 -> an object whose word 2 is the table base; 128
// 16-byte entries from base + 4 {+0 object, +4 type}; objects hold their key at +0x808.
void entry_case(gc_ctx& g, unsigned run) {
    constexpr std::uint32_t kSda = 0x80308000u, kHolder = 0x80304000u, kBase = 0x80305000u, kObjects = 0x80309000u;
    g.r[13] = kSda;
    gcrt::write32(kSda + 0xD00, kHolder);
    gcrt::write32(kHolder + 8, kBase);
    const std::uint32_t key = rng() % 6;
    for (std::uint32_t i = 0; i < 128; ++i) {
        const std::uint32_t e = kBase + 4 + 16 * i, object = kObjects + 0x10 * (i % 32);
        gcrt::write32(e, rng() % 5 ? object : 0);
        gcrt::write32(e + 4, run % 7 ? rng() % 4 : 1);
        gcrt::write32(e + 8, rng());
        gcrt::write32(e + 12, rng());
    }
    for (std::uint32_t k = 0; k < 32; ++k) gcrt::write32(kObjects + 0x10 * k + 0x808, rng() % 8);
    g.r[3] = key;
}
} // namespace

extern "C" {
void hle_PSMTXConcat(gc_ctx*);
void hle_PSVECAdd(gc_ctx*);
void hle_PSVECSubtract(gc_ctx*);
void hle_PSVECScale(gc_ctx*);
void hle_PSVECNormalize(gc_ctx*);
void hle_PSVECSquareMag(gc_ctx*);
void hle_PSVECCrossProduct(gc_ctx*);
void hle_PSVECMag(gc_ctx*);
void hle_WriteMTXPS4x3(gc_ctx*);
void hle_WriteMTXPS3x3from3x4(gc_ctx*);
void hle_VecNormalizeSafe(gc_ctx*);
void hle_NameHashFind(gc_ctx*);
void hle_NameParamGet(gc_ctx*);
void hle_FindEntryByKey(gc_ctx*);
}

int main() {
    if (!gcrt::memory_init()) return 1;
    char error[256];
    gcbe_program* p = gcbe_load(GCBE_NATIVES_BLOB, error, sizeof error);
    if (!p) {
        std::fprintf(stderr, "%s\n", error);
        return 1;
    }
    gcrt::cpu_init({gcbe_functions(p), gcbe_function_count(p), gcbe_dol_hash(p)});
    gcrt::write32(0x804D7EC0u, 0);
    gcrt::write32(0x804D7EC4u, 0x3F800000u);
    const std::map<std::string, void (*)(gc_ctx*)> impl = {
        {"PSMTXConcat", hle_PSMTXConcat}, {"PSVECAdd", hle_PSVECAdd}, {"PSVECSubtract", hle_PSVECSubtract},
        {"PSVECScale", hle_PSVECScale}, {"PSVECNormalize", hle_PSVECNormalize}, {"PSVECSquareMag", hle_PSVECSquareMag},
        {"PSVECCrossProduct", hle_PSVECCrossProduct}, {"PSVECMag", hle_PSVECMag}, {"WriteMTXPS4x3", hle_WriteMTXPS4x3},
        {"WriteMTXPS3x3from3x4", hle_WriteMTXPS3x3from3x4}, {"VecNormalizeSafe", hle_VecNormalizeSafe},
        {"NameHashFind", hle_NameHashFind}, {"NameParamGet", hle_NameParamGet}, {"FindEntryByKey", hle_FindEntryByKey}};
    unsigned total_failures = 0;
    for (const auto& n : gcr::natives()) {
        if (!n.address || n.support) continue;
        const std::uint32_t address = n.address;
        auto it = impl.find(n.name);
        if (it == impl.end()) {
            std::printf("%s: no implementation in this test\n", n.name);
            ++total_failures;
            continue;
        }
        const std::string name = n.name;
        const bool lookup = name == "NameHashFind" || name == "NameParamGet";
        const bool writer = name.rfind("WriteMTXPS", 0) == 0;
        const bool entries = name == "FindEntryByKey";
        unsigned failures = 0;
        const unsigned runs = 20000;
        for (unsigned r = 0; r < runs; ++r) {
            gc_ctx guest{};
            guest.mem = gc_mem;
            guest.r[1] = kStack;
            guest.r[2] = kSda2;
            if (entries) {
                entry_case(guest, r);
            } else if (lookup) {
                lookup_case(guest, r);
                if (name == "NameParamGet") {
                    guest.r[5] = guest.r[4];
                    guest.r[4] = rng() % 16;
                    guest.r[6] = kOut;
                }
            } else {
                fill(kA, 12);
                fill(kB, 12);
                fill(kOut, 12);
                fill(kSda2 - 0x8000, 0x2000);
                if (r % 2) {
                    // The games' constants: PSVECNormalize's 0.5 and 3.0, PSVECMag's 0, 0.5 and 3.0
                    // (doubles), VecNormalizeSafe's threshold, zero and one.
                    gcrt::write32(kSda2 - 16728, 0x3F000000u);
                    gcrt::write32(kSda2 - 16724, 0x40400000u);
                    put_float(kSda2 - 16752, 0.0f);
                    put_double(kSda2 - 16744, 0.5);
                    put_double(kSda2 - 16736, 3.0);
                    put_float(kSda2 - 28548, r % 4 == 1 ? 1e-6f : 1.0f);
                    put_float(kSda2 - 28544, 0.0f);
                    put_float(kSda2 - 28540, 1.0f);
                }
                const unsigned alias = r % 3;   // output separate, aliasing the first or the second input
                guest.r[3] = kA;
                guest.r[4] = alias == 2 ? kA : kB;
                guest.r[5] = alias == 0 ? kOut : alias == 1 ? kA : kB;
                if (writer) guest.r[4] = r % 4 ? kFifo : kOut;
                float scale = pick_float();
                guest.f[1] = guest.ps1[1] = scale;
            }
            const std::uint32_t fill_at = (rng() % 25) * 8 + rng() % 8;   // leaves room for 48 bytes
            static std::uint8_t saved[kAreaSize], translated[kAreaSize];
            std::memcpy(saved, gcrt::host(kArea), kAreaSize);
            gc_ctx native = guest;
            gc_gp.fill = fill_at;
            std::memset(gc_gp.bytes, 0, sizeof gc_gp.bytes);
            gc_call(&guest, address);
            gc_gather fifo_translated = gc_gp;
            std::memcpy(translated, gcrt::host(kArea), kAreaSize);
            std::memcpy(gcrt::host(kArea), saved, kAreaSize);
            gc_gp.fill = fill_at;
            std::memset(gc_gp.bytes, 0, sizeof gc_gp.bytes);
            it->second(&native);
            const bool fifo_same = fifo_translated.fill == gc_gp.fill &&
                                   same_words(fifo_translated.bytes, gc_gp.bytes, (gc_gp.fill + 3) & ~3u);
            const bool same = same_words(translated, gcrt::host(kArea), kAreaSize) && fifo_same &&
                              guest.r[3] == native.r[3] && guest.r[4] == native.r[4] && guest.r[1] == native.r[1] &&
                              same_double(guest.f[1], native.f[1]) && same_double(guest.ps1[1], native.ps1[1]) &&
                              same_double(guest.f[2], native.f[2]) && same_double(guest.ps1[2], native.ps1[2]);
            if (!same && ++failures <= 3) {
                std::printf("%s mismatch, run %u\n", n.name, r);
                for (std::uint32_t o = 0; o < kAreaSize; o += 4)
                    if (std::memcmp(translated + o, gcrt::host(kArea) + o, 4)) {
                        std::uint32_t t, h;
                        std::memcpy(&t, translated + o, 4);
                        std::memcpy(&h, gcrt::host(kArea) + o, 4);
                        std::printf("  %08x: translated %08x native %08x\n", kArea + o, __builtin_bswap32(t), __builtin_bswap32(h));
                    }
                if (!fifo_same) {
                    std::printf("  FIFO fill %u / %u from %u:", fifo_translated.fill, gc_gp.fill, fill_at);
                    for (std::uint32_t k = 0; k < gc_gp.fill; ++k)
                        if (fifo_translated.bytes[k] != gc_gp.bytes[k]) std::printf(" [%u] %02x/%02x", k, fifo_translated.bytes[k], gc_gp.bytes[k]);
                    std::printf("\n");
                }
                std::printf("  r3 %08x/%08x r4 %08x/%08x\n", guest.r[3], native.r[3], guest.r[4], native.r[4]);
                std::printf("  f1 %016llx/%016llx ps1 %016llx/%016llx  f2 %016llx/%016llx ps1 %016llx/%016llx (translated/native)\n",
                            static_cast<unsigned long long>(bits(guest.f[1])), static_cast<unsigned long long>(bits(native.f[1])),
                            static_cast<unsigned long long>(bits(guest.ps1[1])), static_cast<unsigned long long>(bits(native.ps1[1])),
                            static_cast<unsigned long long>(bits(guest.f[2])), static_cast<unsigned long long>(bits(native.f[2])),
                            static_cast<unsigned long long>(bits(guest.ps1[2])), static_cast<unsigned long long>(bits(native.ps1[2])));
            }
        }
        std::printf("%s: %u runs, %u mismatches\n", n.name, runs, failures);
        total_failures += failures;
    }
    return total_failures ? 1 : 0;
}

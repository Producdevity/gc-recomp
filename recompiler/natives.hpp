// SDK functions replaced by runtime implementations (hle_<name>) wherever their code matches
// by length and hash; shared by the C emitter and the AArch64 backend.
#pragma once

#include "image.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

namespace gcr {

// SDK functions replaced by runtime implementations (runtime/src/natives.cpp, os.cpp)
// wherever their code matches: same length in words and the same FNV-1a hash of its
// big-endian bytes. The cache-maintenance loops (DCInvalidateRange, DCFlushRange,
// DCStoreRange, their NoSync forms, ICInvalidateRange) only issue cache and sync
// instructions, and the runtime keeps no caches: they are no-ops.
struct Native {
    const char* name;
    std::uint32_t length;   // in instruction words
    std::uint64_t hash;
    // Where the function sits in Luigi's Mansion (GLME01): the natives test copies it from
    // that game's main.dol to the same address, so relative calls between replaced functions
    // keep their targets. Zero for the no-ops, which the test leaves out.
    std::uint32_t address = 0;
    // Functions the replacement assumes at these offsets from its start (the targets of its
    // calls): a match also needs them there.
    std::vector<std::pair<const char*, std::int32_t>> callees = {};
    // Matched only as a callee and placed in the test image; never replaced.
    bool support = false;
};
inline const std::vector<Native>& natives() {
    static const std::vector<Native> table = {
        {"Nop", 12, 0xe9a85c93393f51a2ull},   // dcbi
        {"Nop", 13, 0xf61bc4c7797677cbull},   // dcbf, sc
        {"Nop", 13, 0xb51ef5ff50c2888bull},   // dcbst, sc
        {"Nop", 12, 0x3bf1d077618b7a05ull},   // dcbf
        {"Nop", 14, 0x732b427446a14e9bull},   // icbi, sync, isync
        // PSMTXConcat, with Unit01 at 0x804D7EC0 (verified {0, 1}).
        {"PSMTXConcat", 51, 0xe1920cf46b2e6ba9ull, 0x801E1C00u},
        // PSVEC functions. Normalize reads its constants through r2.
        {"PSVECAdd", 9, 0x4e12e953c55ebcc6ull, 0x801E2604u},
        {"PSVECSubtract", 9, 0xc512e4a523a86756ull, 0x801E2628u},
        {"PSVECScale", 8, 0x526a2ed552021b9dull, 0x801E264Cu},
        {"PSVECNormalize", 18, 0x1b105c3f1ccd0be2ull, 0x801E266Cu},
        {"PSVECSquareMag", 6, 0x071bc586521686d5ull, 0x801E26B4u},
        {"PSVECCrossProduct", 15, 0x791fcd579bc15a2dull, 0x801E2778u},
        // PSVECMag: PSVECSquareMag (called 0x28 bytes back), then three Newton steps on the
        // reciprocal square root with the 0, 0.5 and 3.0 constants it reads through r2.
        {"PSVECMag", 34, 0x9caad3f143f882aeull, 0x801E26D0u, {{"PSVECSquareMag", -0x1C}}},
        // The GX library's matrix writers: a 3x4 matrix, or its 3x3 part, stored to r4 (the
        // write-gather pipe in GXLoadPosMtxImm and GXLoadNrmMtxImm) with paired-single stores.
        {"WriteMTXPS4x3", 13, 0x1d8ae6deb52ce135ull, 0x801FA0D8u},
        {"WriteMTXPS3x3from3x4", 13, 0xd7ee603c6fc9a035ull, 0x801FA10Cu},
        // Luigi's Mansion game code. VecNormalizeSafe(src = r3, dst = r4): src / PSVECMag(src),
        // or the r2 zero vector below the r2 threshold.
        {"VecNormalizeSafe", 33, 0x1774485574e4b210ull, 0x800781F8u, {{"PSVECMag", 0x16A4D8}, {"PSVECSquareMag", 0x16A4BC}}},
        // NameHashFind(set = r3, name = r4): hashes the name (h = h * 256 + signed byte, modulo
        // 33554393) and returns the index of the 12-byte entry holding that hash, or -1.
        {"NameHashFind", 35, 0x0a89ae810df9e534ull, 0x800BCD5Cu},
        // NameParamGet(set = r3, record = r4, name = r5, out = r6): the named field of a record,
        // masked and shifted, through NameHashFind (called 0x2AC bytes ahead).
        {"NameParamGet", 39, 0xbb5fb9ed494d050full, 0x800BCA88u, {{"NameHashFind", 0x2D4}}},
        // FindEntryByKey(key = r3): of the 128 16-byte entries from base + 4 (base = word 2 of
        // the object that r13 + 0xD00 points to, through EntryRange and EntryRangeFrom), the
        // first of type 2 (+4) whose object (+0) holds key at +0x808, through EntryKey; returns
        // that object or 0, with r4 = base.
        {"FindEntryByKey", 43, 0xcbd07d63414578a4ull, 0x80097F00u,
         {{"EntryRange", 0x519E0}, {"EntryRangeFrom", 0x4DB00}, {"EntryKey", 0x51D5C}}},
        {"EntryRange", 13, 0xe59fbdc0b398ce33ull, 0x800E98E0u, {}, true},
        {"EntryRangeFrom", 5, 0x3cb617eefc8a3b09ull, 0x800E5A00u, {}, true},
        {"EntryKey", 7, 0xd36f356ba97bc390ull, 0x800E9C5Cu, {}, true},
    };
    return table;
}
// FNV-1a over the big-endian bytes of length words from at.
inline std::uint64_t code_hash(const Image& image, std::uint32_t at, std::uint32_t length) {
    std::uint64_t h = 1469598103934665603ull;
    for (std::uint32_t k = 0; k < length; ++k) {
        const std::uint32_t word = image.read32(at + 4 * k);
        for (int shift = 24; shift >= 0; shift -= 8) {
            h ^= (word >> shift) & 0xFF;
            h *= 1099511628211ull;
        }
    }
    return h;
}
inline bool native_at(const Image& image, std::uint32_t at, const Native& native) {
    return code_hash(image, at, native.length) == native.hash;
}
inline const char* native_for(const Image& image, std::uint32_t start, std::uint32_t end) {
    const std::size_t n = (end - start) / 4;
    for (const auto& nat : natives()) {
        if (nat.support || nat.length != n || !native_at(image, start, nat)) continue;
        bool callees = true;
        for (const auto& [name, offset] : nat.callees)
            for (const auto& other : natives())
                if (!std::strcmp(other.name, name) && !native_at(image, start + static_cast<std::uint32_t>(offset), other))
                    callees = false;
        if (callees) return nat.name;
    }
    return nullptr;
}

} // namespace gcr

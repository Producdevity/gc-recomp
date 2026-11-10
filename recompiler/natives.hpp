// SDK functions replaced by runtime implementations (hle_<name>) wherever their code matches
// word for word; shared by the C emitter and the AArch64 backend.
#pragma once

#include "image.hpp"

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <vector>

namespace gcr {

// SDK functions replaced by runtime implementations (runtime/src/natives.cpp, os.cpp)
// wherever their code matches word for word. The cache-maintenance loops (DCInvalidateRange, DCFlushRange, DCStoreRange, their
// NoSync forms, ICInvalidateRange) only issue cache and sync instructions, and the runtime
// keeps no caches: they are no-ops.
struct Native {
    const char* name;
    std::vector<std::uint32_t> words;
    // Where the words sit in Luigi's Mansion (GLME01): the natives test places them there,
    // so relative calls between replaced functions keep their targets. Zero: anywhere.
    std::uint32_t address = 0;
    // Functions the replacement assumes at these offsets from its start (the targets of its
    // calls): a match also needs their words there.
    std::vector<std::pair<const char*, std::int32_t>> callees = {};
    // Matched only as a callee and placed in the test image; never replaced.
    bool support = false;
};
inline const std::vector<Native>& natives() {
    static const std::uint32_t kLoop[] = {0x28040000u, 0x4C810020u, 0x546506FFu, 0x41820008u, 0x38840020u,
                                          0x3884001Fu, 0x5484D97Eu, 0x7C8903A6u};
    auto loop = [](std::initializer_list<std::uint32_t> tail) {
        std::vector<std::uint32_t> w(std::begin(kLoop), std::end(kLoop));
        w.insert(w.end(), tail);
        return w;
    };
    static const std::vector<Native> table = {
        {"Nop", loop({0x7C001BACu, 0x38630020u, 0x4200FFF8u, 0x4E800020u})},                            // dcbi
        {"Nop", loop({0x7C0018ACu, 0x38630020u, 0x4200FFF8u, 0x44000002u, 0x4E800020u})},               // dcbf, sc
        {"Nop", loop({0x7C00186Cu, 0x38630020u, 0x4200FFF8u, 0x44000002u, 0x4E800020u})},               // dcbst, sc
        {"Nop", loop({0x7C0018ACu, 0x38630020u, 0x4200FFF8u, 0x4E800020u})},                            // dcbf
        {"Nop", loop({0x7C00186Cu, 0x38630020u, 0x4200FFF8u, 0x4E800020u})},                            // dcbst
        {"Nop", loop({0x7C001FACu, 0x38630020u, 0x4200FFF8u, 0x7C0004ACu, 0x4C00012Cu, 0x4E800020u})},  // icbi, sync, isync
        // PSMTXConcat, with Unit01 at 0x804D7EC0 (Luigi's Mansion GLME01, verified {0, 1}).
        {"PSMTXConcat",
         {0x9421FFC0u, 0xE0030000u, 0xD9C10008u, 0xE0C40000u, 0x3CC0804Du, 0xE0E40008u, 0xD9E10010u, 0x38C67EC0u,
          0xDBE10028u, 0xE1040010u, 0x11860018u, 0xE0430010u, 0x11A70018u, 0xE3E60000u, 0x11C60098u, 0xE1240018u,
          0x11E70098u, 0xE0230008u, 0x1188601Eu, 0xE0630018u, 0x11C8709Eu, 0xE1440020u, 0x11A9681Eu, 0xE1640028u,
          0x11E9789Eu, 0xE0830020u, 0xE0A30028u, 0x118A605Cu, 0x11AB685Cu, 0x11CA70DCu, 0x11EB78DCu, 0xF1850000u,
          0x10460118u, 0x11BF685Eu, 0x10070118u, 0xF1C50010u, 0x11FF78DEu, 0xF1A50008u, 0x1048111Eu, 0x1009011Eu,
          0x104A115Cu, 0xC9C10008u, 0xF1E50018u, 0x100B015Cu, 0xF0450020u, 0x101F015Eu, 0xC9E10010u, 0xF0050028u,
          0xCBE10028u, 0x38210040u, 0x4E800020u}},
        // PSVEC functions (Luigi's Mansion GLME01). Normalize reads its constants through r2.
        {"PSVECAdd", {0xE0430000u, 0xE0840000u, 0x10C2202Au, 0xF0C50000u, 0xE0638008u, 0xE0A48008u, 0x10E3282Au, 0xF0E58008u, 0x4E800020u}},
        {"PSVECSubtract", {0xE0430000u, 0xE0840000u, 0x10C22028u, 0xF0C50000u, 0xE0638008u, 0xE0A48008u, 0x10E32828u, 0xF0E58008u, 0x4E800020u}},
        {"PSVECScale", {0xE0430000u, 0x10810C20u, 0x10C20132u, 0xF0C40000u, 0xE0638008u, 0x10E30132u, 0xF0E48008u, 0x4E800020u}},
        {"PSVECNormalize", {0xC002BEA8u, 0xC022BEACu, 0xE0430000u, 0x10A200B2u, 0xE0638008u, 0x108328FAu, 0x108428D4u, 0x10A02034u, 0xECC50172u, 0xEC050032u, 0xECC60132u, 0xECC13028u, 0xECA60032u, 0x10420158u, 0xF0440000u, 0x10630158u, 0xF0648008u, 0x4E800020u}},
        {"PSVECSquareMag", {0xE0430000u, 0x106200B2u, 0xC0830008u, 0x10A4193Au, 0x102518D4u, 0x4E800020u}, 0x801E26B4u},
        {"PSVECCrossProduct", {0xE0240000u, 0xC0430008u, 0xE0030000u, 0x10C10CA0u, 0xC0640008u, 0x108100B2u, 0x10E10018u, 0x10A020F8u, 0x110039B8u, 0x11252CE0u, 0x11454460u, 0xF1258000u, 0x11405050u, 0xF1450004u, 0x4E800020u}},
        // PSVECMag: PSVECSquareMag (called 0x28 bytes back), then three Newton steps on the
        // reciprocal square root with the 0, 0.5 and 3.0 constants it reads through r2.
        {"PSVECMag",
         {0x7C0802A6u, 0x90010004u, 0x9421FFF0u, 0x4BFFFFD9u, 0xC002BE90u, 0xFC010040u, 0x40810060u, 0xFC400834u,
          0xC882BE98u, 0xC862BEA0u, 0xFC0200B2u, 0xFC4400B2u, 0xFC010032u, 0xFC030028u, 0xFC420032u, 0xFC0200B2u,
          0xFC4400B2u, 0xFC010032u, 0xFC030028u, 0xFC420032u, 0xFC0200B2u, 0xFC4400B2u, 0xFC010032u, 0xFC030028u,
          0xFC020032u, 0xFC010032u, 0xFC000018u, 0xD001000Cu, 0xC021000Cu, 0x48000004u, 0x80010014u, 0x38210010u,
          0x7C0803A6u, 0x4E800020u},
         0x801E26D0u, {{"PSVECSquareMag", -0x1C}}},
        // The GX library's matrix writers: a 3x4 matrix, or its 3x3 part, stored to r4 (the
        // write-gather pipe in GXLoadPosMtxImm and GXLoadNrmMtxImm) with paired-single stores.
        {"WriteMTXPS4x3",
         {0xE0030000u, 0xE0230008u, 0xE0430010u, 0xE0630018u, 0xE0830020u, 0xE0A30028u, 0xF0040000u, 0xF0240000u,
          0xF0440000u, 0xF0640000u, 0xF0840000u, 0xF0A40000u, 0x4E800020u},
         0x801FA0D8u},
        {"WriteMTXPS3x3from3x4",
         {0xE0030000u, 0xC0230008u, 0xE0430010u, 0xC0630018u, 0xE0830020u, 0xC0A30028u, 0xF0040000u, 0xD0240000u,
          0xF0440000u, 0xD0640000u, 0xF0840000u, 0xD0A40000u, 0x4E800020u},
         0x801FA10Cu},
        // Luigi's Mansion game code. VecNormalizeSafe(src = r3, dst = r4): src / PSVECMag(src),
        // or the r2 zero vector below the r2 threshold.
        {"VecNormalizeSafe",
         {0x7C0802A6u, 0x90010004u, 0x9421FFE8u, 0x93E10014u, 0x3BE40000u, 0x93C10010u, 0x3BC30000u, 0x4816A4BDu,
          0xC002907Cu, 0xFC010040u, 0x40800018u, 0xC0029080u, 0xD01F0008u, 0xD01F0004u, 0xD01F0000u, 0x48000030u,
          0xC0429084u, 0xC01E0000u, 0xEC420824u, 0xEC0000B2u, 0xD01F0000u, 0xC01E0004u, 0xEC0000B2u, 0xD01F0004u,
          0xC01E0008u, 0xEC0000B2u, 0xD01F0008u, 0x8001001Cu, 0x83E10014u, 0x83C10010u, 0x7C0803A6u, 0x38210018u,
          0x4E800020u},
         0x800781F8u, {{"PSVECMag", 0x16A4D8}, {"PSVECSquareMag", 0x16A4BC}}},
        // NameHashFind(set = r3, name = r4): hashes the name (h = h * 256 + signed byte, modulo
        // 33554393) and returns the index of the 12-byte entry holding that hash, or -1.
        {"NameHashFind",
         {0x81030004u, 0x3C600200u, 0x3863FFD9u, 0x81280004u, 0x38E00000u, 0x38A01381u, 0x48000030u, 0x7CC60774u,
          0x54E0402Eu, 0x7CE60214u, 0x7CC53816u, 0x7C063850u, 0x5400F87Eu, 0x7C003214u, 0x5400463Eu, 0x7C0019D6u,
          0x7CE03850u, 0x38840001u, 0x88C40000u, 0x7CC00775u, 0x4082FFCCu, 0x2C090000u, 0x7D2903A6u, 0x38600000u,
          0x38800000u, 0x40810020u, 0x38040010u, 0x7C08002Eu, 0x7C070040u, 0x4D820020u, 0x3884000Cu, 0x38630001u,
          0x4200FFE8u, 0x3860FFFFu, 0x4E800020u},
         0x800BCD5Cu},
        // NameParamGet(set = r3, record = r4, name = r5, out = r6): the named field of a record,
        // masked and shifted, through NameHashFind (called 0x2AC bytes ahead).
        {"NameParamGet",
         {0x7C0802A6u, 0x90010004u, 0x9421FFC8u, 0x93E10034u, 0x3BE60000u, 0x93C10030u, 0x3BC40000u, 0x38850000u,
          0x93A1002Cu, 0x3BA30000u, 0x480002ADu, 0x2C030000u, 0x4080000Cu, 0x38600000u, 0x48000048u, 0x80FD0004u,
          0x1C63000Cu, 0x8007000Cu, 0x38C30010u, 0x80870008u, 0x7C7E01D6u, 0x7CC73214u, 0xA0060008u, 0x7CA41A14u,
          0x80860004u, 0x38600001u, 0x7C050214u, 0x7CA7002Eu, 0x8806000Au, 0x7CA42038u, 0x7C800430u, 0x901F0000u,
          0x8001003Cu, 0x83E10034u, 0x83C10030u, 0x7C0803A6u, 0x83A1002Cu, 0x38210038u, 0x4E800020u},
         0x800BCA88u, {{"NameHashFind", 0x2D4}}},
        // FindEntryByKey(key = r3): of the 128 16-byte entries from base + 4 (base = word 2 of
        // the object that r13 + 0xD00 points to, through EntryRange and EntryRangeFrom), the
        // first of type 2 (+4) whose object (+0) holds key at +0x808, through EntryKey; returns
        // that object or 0, with r4 = base.
        {"FindEntryByKey",
         {0x7C0802A6u, 0x90010004u, 0x9421FFA8u, 0x93E10054u, 0x3BE30000u, 0x38610034u, 0x93C10050u, 0x3BC00000u,
          0x480519C1u, 0x80610034u, 0x80010038u, 0x90610044u, 0x90010048u, 0x93E1002Cu, 0x83E10044u, 0x8001002Cu,
          0x281F0000u, 0x90010030u, 0x4182002Cu, 0x48000028u, 0x801F0004u, 0x2C000002u, 0x40820018u, 0x7FE3FB78u,
          0x48051CFDu, 0x80010030u, 0x7C030040u, 0x41820014u, 0x3BFF0010u, 0x80010048u, 0x7C1F0040u, 0x4082FFD4u,
          0x80010048u, 0x7C1F0040u, 0x41820008u, 0x83DF0000u, 0x8001005Cu, 0x7FC3F378u, 0x83E10054u, 0x83C10050u,
          0x7C0803A6u, 0x38210058u, 0x4E800020u},
         0x80097F00u, {{"EntryRange", 0x519E0}, {"EntryRangeFrom", 0x4DB00}, {"EntryKey", 0x51D5C}}},
        {"EntryRange",
         {0x7C0802A6u, 0x90010004u, 0x9421FFD8u, 0x93E10024u, 0x7C7F1B78u, 0x808D0D00u, 0x80840008u, 0x4BFFC105u,
          0x8001002Cu, 0x83E10024u, 0x38210028u, 0x7C0803A6u, 0x4E800020u},
         0x800E98E0u, {}, true},
        {"EntryRangeFrom", {0x38040004u, 0x90030000u, 0x38040804u, 0x90030004u, 0x4E800020u}, 0x800E5A00u, {}, true},
        {"EntryKey", {0x80630000u, 0x28030000u, 0x4082000Cu, 0x38600000u, 0x4E800020u, 0x80630808u, 0x4E800020u},
         0x800E9C5Cu, {}, true},
    };
    return table;
}
inline bool words_at(const Image& image, std::uint32_t at, const std::vector<std::uint32_t>& words) {
    for (std::size_t k = 0; k < words.size(); ++k)
        if (image.read32(at + 4 * static_cast<std::uint32_t>(k)) != words[k]) return false;
    return true;
}
inline const char* native_for(const Image& image, std::uint32_t start, std::uint32_t end) {
    const std::size_t n = (end - start) / 4;
    for (const auto& nat : natives()) {
        if (nat.support || nat.words.size() != n || !words_at(image, start, nat.words)) continue;
        bool callees = true;
        for (const auto& [name, offset] : nat.callees)
            for (const auto& other : natives())
                if (!std::strcmp(other.name, name) && !words_at(image, start + static_cast<std::uint32_t>(offset), other.words))
                    callees = false;
        if (callees) return nat.name;
    }
    return nullptr;
}

} // namespace gcr

// Writes the guest code of the replaced SDK functions (recompiler/natives.hpp) as a DOL, so
// the test can translate it with --no-natives and compare it with the runtime's versions.
// Usage: gc-natives-gen OUT.dol OUT.symbols
#include "../recompiler/natives.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {
constexpr std::uint32_t kText = 0x80003100u;
constexpr std::uint32_t kUnit01 = 0x804D7EC0u;   // PSMTXConcat's {0, 1} constant

void put32(std::vector<std::uint8_t>& v, std::size_t at, std::uint32_t x) {
    v[at] = x >> 24; v[at + 1] = x >> 16; v[at + 2] = x >> 8; v[at + 3] = x;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    // Text section 0 holds the natives without an address, one after another from kText;
    // section 1 spans those with one, each at its address.
    std::vector<std::uint32_t> code;
    std::uint32_t lo = 0xFFFFFFFFu, hi = 0;
    for (const auto& n : gcr::natives())
        if (n.address) {
            lo = std::min(lo, n.address);
            hi = std::max(hi, n.address + static_cast<std::uint32_t>(n.words.size() * 4));
        }
    std::vector<std::uint32_t> placed(lo < hi ? (hi - lo) / 4 : 0, 0);
    std::FILE* symbols = std::fopen(argv[2], "w");
    if (!symbols) return 1;
    for (const auto& n : gcr::natives()) {
        if (!std::strcmp(n.name, "Nop")) continue;
        std::uint32_t at;
        if (n.address) {
            at = n.address;
            std::copy(n.words.begin(), n.words.end(), placed.begin() + (at - lo) / 4);
        } else {
            at = kText + static_cast<std::uint32_t>(code.size() * 4);
            code.insert(code.end(), n.words.begin(), n.words.end());
        }
        std::fprintf(symbols, "%s = .text:0x%08X; // type:function size:0x%zX scope:global\n", n.name, at, n.words.size() * 4);
    }
    std::fclose(symbols);
    const std::uint8_t unit[8] = {0, 0, 0, 0, 0x3F, 0x80, 0, 0};
    const std::size_t text0 = 0x100, text1 = text0 + code.size() * 4, data0 = text1 + placed.size() * 4;
    std::vector<std::uint8_t> dol(data0 + sizeof unit, 0);
    put32(dol, 0x00, static_cast<std::uint32_t>(text0));
    put32(dol, 0x48, kText);
    put32(dol, 0x90, static_cast<std::uint32_t>(code.size() * 4));
    if (!placed.empty()) {
        put32(dol, 0x04, static_cast<std::uint32_t>(text1));
        put32(dol, 0x4C, lo);
        put32(dol, 0x94, static_cast<std::uint32_t>(placed.size() * 4));
    }
    put32(dol, 0x1C, static_cast<std::uint32_t>(data0));
    put32(dol, 0x64, kUnit01);
    put32(dol, 0xAC, sizeof unit);
    put32(dol, 0xE0, kText);
    for (std::size_t i = 0; i < code.size(); ++i) put32(dol, text0 + i * 4, code[i]);
    for (std::size_t i = 0; i < placed.size(); ++i) put32(dol, text1 + i * 4, placed[i]);
    std::memcpy(&dol[data0], unit, sizeof unit);
    std::FILE* f = std::fopen(argv[1], "wb");
    if (!f) return 1;
    std::fwrite(dol.data(), 1, dol.size(), f);
    std::fclose(f);
    return 0;
}

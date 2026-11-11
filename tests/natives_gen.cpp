// Writes the guest code of the replaced functions (recompiler/natives.hpp) as a DOL, so the
// test can translate it with --no-natives and compare it with the runtime's versions. The code
// is copied from Luigi's Mansion's main.dol, or Super Mario Sunshine's for its natives, from each
// native's address there, and must match the native's length and hash. Without the Sunshine DOL
// its natives are left out.
// Usage: gc-natives-gen GLME01-MAIN.dol OUT.dol OUT.symbols [GMSE01-MAIN.dol]
#include "../recompiler/natives.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {
constexpr std::uint32_t kUnit01 = 0x804D7EC0u;   // PSMTXConcat's {0, 1} constant

void put32(std::vector<std::uint8_t>& v, std::size_t at, std::uint32_t x) {
    v[at] = x >> 24; v[at + 1] = x >> 16; v[at + 2] = x >> 8; v[at + 3] = x;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 4 && argc != 5) return 2;
    gcr::Image game, sunshine;
    std::string error;
    if (!game.load_dol(argv[1], error) || (argc == 5 && !sunshine.load_dol(argv[4], error))) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }
    const auto source = [&](const gcr::Native& n) -> const gcr::Image* {
        if (!n.address) return nullptr;
        if (n.sunshine) return argc == 5 ? &sunshine : nullptr;
        return &game;
    };
    // One text section spans the natives, each at its address in the game.
    std::uint32_t lo = 0xFFFFFFFFu, hi = 0;
    for (const auto& n : gcr::natives())
        if (source(n)) {
            lo = std::min(lo, n.address);
            hi = std::max(hi, n.address + n.length * 4);
        }
    std::vector<std::uint32_t> placed((hi - lo) / 4, 0);
    std::FILE* symbols = std::fopen(argv[3], "w");
    if (!symbols) return 1;
    for (const auto& n : gcr::natives()) {
        const gcr::Image* image = source(n);
        if (!image) continue;
        if (!gcr::native_at(*image, n.address, n)) {
            std::fprintf(stderr, "%s is not at 0x%08X in its game's main.dol\n", n.name, n.address);
            return 1;
        }
        for (std::uint32_t k = 0; k < n.length; ++k) placed[(n.address - lo) / 4 + k] = image->read32(n.address + 4 * k);
        std::fprintf(symbols, "%s = .text:0x%08X; // type:function size:0x%X scope:global\n", n.name, n.address, n.length * 4);
    }
    std::fclose(symbols);
    const std::uint8_t unit[8] = {0, 0, 0, 0, 0x3F, 0x80, 0, 0};
    const std::size_t text0 = 0x100, data0 = text0 + placed.size() * 4;
    std::vector<std::uint8_t> dol(data0 + sizeof unit, 0);
    put32(dol, 0x00, static_cast<std::uint32_t>(text0));
    put32(dol, 0x48, lo);
    put32(dol, 0x90, static_cast<std::uint32_t>(placed.size() * 4));
    put32(dol, 0x1C, static_cast<std::uint32_t>(data0));
    put32(dol, 0x64, kUnit01);
    put32(dol, 0xAC, sizeof unit);
    put32(dol, 0xE0, lo);
    for (std::size_t i = 0; i < placed.size(); ++i) put32(dol, text0 + i * 4, placed[i]);
    std::memcpy(&dol[data0], unit, sizeof unit);
    std::FILE* f = std::fopen(argv[2], "wb");
    if (!f) return 1;
    std::fwrite(dol.data(), 1, dol.size(), f);
    std::fclose(f);
    return 0;
}

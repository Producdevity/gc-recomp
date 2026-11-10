// Writes the instruction tests as a DOL and a dtk-style symbols file.
// Usage: gc-ppc-gen OUT.dol OUT.symbols
#include "ppc_tests.hpp"

#include <cstdio>
#include <vector>

namespace {
constexpr ppct::W kText = 0x80003100u;

void put32(std::vector<std::uint8_t>& v, std::size_t at, ppct::W x) {
    v[at] = x >> 24; v[at + 1] = x >> 16; v[at + 2] = x >> 8; v[at + 3] = x;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    auto tests = ppct::tests();
    std::vector<ppct::W> code;
    std::FILE* symbols = std::fopen(argv[2], "w");
    if (!symbols) return 1;
    ppct::W switch_base = 0;
    for (const auto& t : tests) {
        ppct::W at = kText + static_cast<ppct::W>(code.size() * 4);
        if (t.name == "switch") switch_base = at;
        std::fprintf(symbols, "%s = .text:0x%08X; // type:function size:0x%zX scope:global\n", t.name.c_str(), at,
                     t.code.size() * 4);
        code.insert(code.end(), t.code.begin(), t.code.end());
    }
    std::fclose(symbols);
    auto data = ppct::data_image(switch_base + 16, switch_base + 24);
    std::vector<std::uint8_t> dol(0x100 + code.size() * 4 + data.size(), 0);
    put32(dol, 0x00, 0x100);                                  // text 0 offset
    put32(dol, 0x48, kText);                                  // text 0 address
    put32(dol, 0x90, static_cast<ppct::W>(code.size() * 4));  // text 0 size
    put32(dol, 0x1C, static_cast<ppct::W>(0x100 + code.size() * 4));   // data 0 offset
    put32(dol, 0x64, ppct::kData);                                      // data 0 address
    put32(dol, 0xAC, static_cast<ppct::W>(data.size()));                // data 0 size
    put32(dol, 0xE0, kText);
    for (std::size_t i = 0; i < code.size(); ++i) put32(dol, 0x100 + i * 4, code[i]);
    for (std::size_t i = 0; i < data.size(); ++i) dol[0x100 + code.size() * 4 + i] = data[i];
    std::FILE* f = std::fopen(argv[1], "wb");
    if (!f) return 1;
    std::fwrite(dol.data(), 1, dol.size(), f);
    std::fclose(f);
    return 0;
}

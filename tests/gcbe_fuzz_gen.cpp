// Writes the backend differential test program as a DOL and a dtk-style symbols file.
// Usage: gc-gcbe-fuzz-gen OUT.dol OUT.symbols SEED
#include "gcbe_fuzz.hpp"

#include <cstdio>
#include <cstdlib>

namespace {
void put32(std::vector<std::uint8_t>& v, std::size_t at, gcbef::W x) {
    v[at] = x >> 24; v[at + 1] = x >> 16; v[at + 2] = x >> 8; v[at + 3] = x;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 4) return 2;
    auto fns = gcbef::program(static_cast<std::uint32_t>(std::strtoul(argv[3], nullptr, 0)));
    std::vector<gcbef::W> code;
    std::FILE* symbols = std::fopen(argv[2], "w");
    if (!symbols) return 1;
    for (std::size_t n = 0; n < fns.size(); ++n) {
        std::fprintf(symbols, "fn_%zu = .text:0x%08X; // type:function size:0x%zX scope:global\n", n, fns[n].address,
                     fns[n].code.size() * 4);
        code.insert(code.end(), fns[n].code.begin(), fns[n].code.end());
    }
    std::fclose(symbols);
    std::vector<std::uint8_t> dol(0x100 + code.size() * 4 + 0x20, 0);
    put32(dol, 0x00, 0x100);
    put32(dol, 0x48, gcbef::kText);
    put32(dol, 0x90, static_cast<gcbef::W>(code.size() * 4));
    put32(dol, 0x1C, static_cast<gcbef::W>(0x100 + code.size() * 4));
    put32(dol, 0x64, gcbef::kData);
    put32(dol, 0xAC, 0x20);
    put32(dol, 0xE0, gcbef::kText);
    for (std::size_t i = 0; i < code.size(); ++i) put32(dol, 0x100 + i * 4, code[i]);
    std::FILE* f = std::fopen(argv[1], "wb");
    if (!f) return 1;
    std::fwrite(dol.data(), 1, dol.size(), f);
    std::fclose(f);
    return 0;
}

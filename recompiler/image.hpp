// DOL executable image and function list inputs.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace gcr {

struct Section {
    std::uint32_t address = 0, size = 0, offset = 0;
    bool text = false;
};

class Image {
public:
    bool load_dol(const std::string& path, std::string& error);
    // The main executable of a GameCube disc image (plain ISO/GCM).
    bool load_disc(const std::string& path, std::string& error);
    // A DOL already read into memory (from any disc format).
    bool load_dol_bytes(std::vector<std::uint8_t> bytes, std::string& error);
    bool contains(std::uint32_t address, std::uint32_t size = 4) const;
    bool in_text(std::uint32_t address) const;
    std::uint32_t read32(std::uint32_t address) const;
    const std::vector<Section>& sections() const { return sections_; }
    std::uint32_t entry() const { return entry_; }
    std::uint32_t bss_address() const { return bss_address_; }
    std::uint32_t bss_size() const { return bss_size_; }
    const std::vector<std::uint8_t>& bytes() const { return file_; }

private:
    bool parse(std::string& error);
    const Section* find(std::uint32_t address, std::uint32_t size) const;
    std::vector<std::uint8_t> file_;
    std::vector<Section> sections_;
    std::uint32_t entry_ = 0, bss_address_ = 0, bss_size_ = 0;
};

struct FunctionSymbol {
    std::uint32_t address = 0, size = 0;
    std::string name;
};

// dtk symbols.txt: "name = .text:0xADDR; // type:function size:0xSIZE ..."
bool load_dtk_symbols(const std::string& path, std::vector<FunctionSymbol>& out, std::string& error);

// Per-game recompiler configuration, one directive per line:
//   hle ADDR NAME        replace the function with the runtime's hle_NAME(Ctx*)
//   wrap ADDR NAME       call the runtime's wrap_NAME(Ctx*, original) in its place
//   save_context ADDR    calls to ADDR return twice (OSSaveContext semantics)
//   function ADDR SIZE [NAME]  add or resize a function
//   exact ADDR           keep every guest register in the context across calls
//   skip ADDR            do not recompile the function at ADDR
struct Config {
    std::map<std::uint32_t, std::string> hle;
    std::map<std::uint32_t, std::string> wrap;
    std::map<std::uint32_t, bool> save_context;
    std::vector<FunctionSymbol> functions;
    std::map<std::uint32_t, bool> exact;
    std::map<std::uint32_t, bool> skip;
};
bool load_config(const std::string& path, Config& out, std::string& error);

} // namespace gcr

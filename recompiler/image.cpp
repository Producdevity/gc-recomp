#include "image.hpp"

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <sstream>

namespace gcr {

static std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3];
}

bool Image::load_dol(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    file_.assign(std::istreambuf_iterator<char>(in), {});
    return parse(error);
}

bool Image::load_dol_bytes(std::vector<std::uint8_t> bytes, std::string& error) {
    file_ = std::move(bytes);
    return parse(error);
}

bool Image::load_disc(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::uint8_t header[0x440];
    if (!in.read(reinterpret_cast<char*>(header), sizeof header) || be32(header + 0x1C) != 0xC2339F3Du) {
        error = path + " is not a GameCube disc image (ISO/GCM)";
        return false;
    }
    std::uint32_t dol = be32(header + 0x420);
    std::uint8_t dh[0x100];
    in.seekg(dol);
    if (!in.read(reinterpret_cast<char*>(dh), sizeof dh)) {
        error = "cannot read the DOL header";
        return false;
    }
    std::uint32_t size = 0;
    for (int i = 0; i < 18; ++i) {
        std::uint32_t o = be32(dh + i * 4), s = be32(dh + 0x90 + i * 4);
        if (s) size = std::max(size, o + s);
    }
    file_.resize(size);
    in.seekg(dol);
    if (!in.read(reinterpret_cast<char*>(file_.data()), size)) {
        error = "cannot read the DOL";
        return false;
    }
    return parse(error);
}

bool Image::parse(std::string& error) {
    if (file_.size() < 0x100) {
        error = "DOL too small";
        return false;
    }
    for (int i = 0; i < 18; ++i) {
        Section s;
        s.offset = be32(&file_[i * 4]);
        s.address = be32(&file_[0x48 + i * 4]);
        s.size = be32(&file_[0x90 + i * 4]);
        s.text = i < 7;
        if (!s.size) continue;
        if (std::uint64_t(s.offset) + s.size > file_.size()) {
            error = "DOL section outside file";
            return false;
        }
        sections_.push_back(s);
    }
    bss_address_ = be32(&file_[0xD8]);
    bss_size_ = be32(&file_[0xDC]);
    entry_ = be32(&file_[0xE0]);
    return true;
}

const Section* Image::find(std::uint32_t address, std::uint32_t size) const {
    for (const auto& s : sections_)
        if (address >= s.address && std::uint64_t(address) + size <= std::uint64_t(s.address) + s.size) return &s;
    return nullptr;
}

bool Image::contains(std::uint32_t address, std::uint32_t size) const { return find(address, size) != nullptr; }

bool Image::in_text(std::uint32_t address) const {
    const auto* s = find(address, 4);
    return s && s->text;
}

std::uint32_t Image::read32(std::uint32_t address) const {
    const auto* s = find(address, 4);
    if (!s) return 0;
    return be32(&file_[s->offset + (address - s->address)]);
}

bool load_dtk_symbols(const std::string& path, std::vector<FunctionSymbol>& out, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("type:function") == std::string::npos) continue;
        auto eq = line.find(" = ");
        auto colon = line.find(":0x", eq);
        auto size = line.find("size:0x");
        if (eq == std::string::npos || colon == std::string::npos || size == std::string::npos) continue;
        auto section = line.substr(eq + 3, colon - eq - 3);
        if (section != ".text" && section != ".init") continue;
        FunctionSymbol f;
        f.name = line.substr(0, eq);
        f.address = static_cast<std::uint32_t>(std::strtoul(line.c_str() + colon + 3, nullptr, 16));
        f.size = static_cast<std::uint32_t>(std::strtoul(line.c_str() + size + 7, nullptr, 16));
        out.push_back(f);
    }
    return true;
}

bool load_config(const std::string& path, Config& out, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::string line;
    int number = 0;
    while (std::getline(in, line)) {
        ++number;
        auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream words(line);
        std::string kind, address;
        if (!(words >> kind)) continue;
        if (!(words >> address)) {
            error = path + ":" + std::to_string(number) + ": missing address";
            return false;
        }
        auto a = static_cast<std::uint32_t>(std::strtoul(address.c_str(), nullptr, 16));
        if (kind == "hle") {
            std::string name;
            words >> name;
            out.hle[a] = name;
        } else if (kind == "wrap") {
            std::string name;
            words >> name;
            out.wrap[a] = name;
        } else if (kind == "save_context") {
            out.save_context[a] = true;
        } else if (kind == "function") {
            std::string size, name;
            words >> size >> name;
            out.functions.push_back({a, static_cast<std::uint32_t>(std::strtoul(size.c_str(), nullptr, 16)), name});
        } else if (kind == "exact") {
            out.exact[a] = true;
        } else if (kind == "skip") {
            out.skip[a] = true;
        } else {
            error = path + ":" + std::to_string(number) + ": unknown directive " + kind;
            return false;
        }
    }
    return true;
}

} // namespace gcr

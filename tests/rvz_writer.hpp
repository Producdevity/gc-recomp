// Writes GameCube RVZ images for tests (layout as in Dolphin's docs/WiaAndRvz.md): from a
// plain image with unpacked groups, or by recompressing an uncompressed RVZ group by group
// with its packing kept. Compression 0 (none) or 5 (Zstandard); a group whose compressed
// form is not smaller stays stored, as do all-zero groups (stored empty).
#pragma once

#include "starlet/disc/sha1.hpp"

#include <zstd.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace rvzw {

inline std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3];
}
inline std::uint64_t be64(const std::uint8_t* p) { return (std::uint64_t(be32(p)) << 32) | be32(p + 4); }
inline void put32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = std::uint8_t(v >> (24 - 8 * i));
}
inline void put64(std::uint8_t* p, std::uint64_t v) {
    put32(p, std::uint32_t(v >> 32));
    put32(p + 4, std::uint32_t(v));
}

struct Group {
    std::vector<std::uint8_t> data;   // stored bytes
    std::uint32_t packed = 0;         // rvz_packed_size: size of the packed data, 0 if unpacked
    bool compressed = false;
};

inline std::vector<std::uint8_t> zstd(const std::uint8_t* data, std::size_t size, int level) {
    std::vector<std::uint8_t> out(ZSTD_compressBound(size));
    const std::size_t n = ZSTD_compress(out.data(), out.size(), data, size, level);
    out.resize(ZSTD_isError(n) ? 0 : n);
    return out;
}

// Compresses a group in place when that makes it smaller.
inline void compress(Group& g, int level) {
    if (g.data.empty()) return;
    auto c = zstd(g.data.data(), g.data.size(), level);
    if (!c.empty() && c.size() < g.data.size()) {
        g.data.swap(c);
        g.compressed = true;
    }
}

// Stores the SHA-1s of header 1 and header 2 (0x48 + 0xDC bytes at head): header 2's at
// 0x10, then header 1's own over its first 0x34 bytes at 0x34.
inline void seal(std::uint8_t* head) {
    starlet::disc::Sha1 disc_hash;
    disc_hash.update(head + 0x48, 0xDC);
    const auto disc_digest = disc_hash.finish();
    std::memcpy(head + 0x10, disc_digest.data(), disc_digest.size());
    starlet::disc::Sha1 head_hash;
    head_hash.update(head, 0x34);
    const auto head_digest = head_hash.finish();
    std::memcpy(head + 0x34, head_digest.data(), head_digest.size());
}

// Writes header 1 (0x48 bytes), header 2 (0xDC), the groups, then the raw-data and group
// tables (compressed with the image's method).
inline bool write(const std::string& path, std::uint64_t iso_size, const std::uint8_t* dhead, std::uint32_t chunk,
                  const std::vector<std::uint8_t>& raw_entries, const std::vector<Group>& groups, int compression,
                  int level, std::string& error) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        error = "cannot write " + path;
        return false;
    }
    std::vector<std::uint8_t> head(0x48 + 0xDC, 0);
    std::fwrite(head.data(), 1, head.size(), f);
    std::uint64_t at = head.size();
    std::vector<std::uint8_t> entries(groups.size() * 12, 0);
    for (std::size_t i = 0; i < groups.size(); ++i) {
        const Group& g = groups[i];
        put32(&entries[i * 12 + 8], g.packed);
        if (g.data.empty()) continue;
        put32(&entries[i * 12], std::uint32_t(at >> 2));
        put32(&entries[i * 12 + 4], std::uint32_t(g.data.size()) | (g.compressed ? 0x80000000u : 0));
        std::fwrite(g.data.data(), 1, g.data.size(), f);
        at += g.data.size();
        static const std::uint8_t pad[4] = {};
        const std::size_t padding = std::size_t((4 - at % 4) % 4);
        std::fwrite(pad, 1, padding, f);
        at += padding;
    }
    const auto raw_table = compression == 5 ? zstd(raw_entries.data(), raw_entries.size(), level) : raw_entries;
    const auto group_table = compression == 5 ? zstd(entries.data(), entries.size(), level) : entries;
    const std::uint64_t raw_at = at;
    std::fwrite(raw_table.data(), 1, raw_table.size(), f);
    at += raw_table.size();
    const std::uint64_t group_at = at;
    std::fwrite(group_table.data(), 1, group_table.size(), f);
    at += group_table.size();

    std::memcpy(head.data(), "RVZ\x01", 4);
    put32(&head[4], 0x01000000u);
    put32(&head[8], 0x00030000u);
    put32(&head[0x0C], 0xDC);
    put64(&head[0x24], iso_size);
    put64(&head[0x2C], at);
    std::uint8_t* d = &head[0x48];
    put32(d, 1);
    put32(d + 4, std::uint32_t(compression));
    put32(d + 8, std::uint32_t(compression ? level : 0));
    put32(d + 0xC, chunk);
    std::memcpy(d + 0x10, dhead, 0x80);
    put32(d + 0xB4, std::uint32_t(raw_entries.size() / 24));
    put64(d + 0xB8, raw_at);
    put32(d + 0xC0, std::uint32_t(raw_table.size()));
    put32(d + 0xC4, std::uint32_t(groups.size()));
    put64(d + 0xC8, group_at);
    put32(d + 0xD0, std::uint32_t(group_table.size()));
    seal(head.data());
    std::fseek(f, 0, SEEK_SET);
    std::fwrite(head.data(), 1, head.size(), f);
    return std::fclose(f) == 0;
}

// One raw-data entry from 0x80 to the end, covering `groups` groups from index 0.
inline std::vector<std::uint8_t> single_raw_entry(std::uint64_t iso_size, std::size_t groups) {
    std::vector<std::uint8_t> raw(24, 0);
    put64(&raw[0], 0x80);
    put64(&raw[8], iso_size - 0x80);
    put32(&raw[20], std::uint32_t(groups));
    return raw;
}

inline Group plain_group(const std::uint8_t* data, std::size_t size, int compression, int level) {
    Group g;
    if (std::any_of(data, data + size, [](std::uint8_t b) { return b != 0; })) g.data.assign(data, data + size);
    if (compression == 5) compress(g, level);
    return g;
}

// A plain image held in memory.
inline bool from_iso(const std::vector<std::uint8_t>& iso, std::uint32_t chunk, int compression, int level,
                     const std::string& out, std::string& error) {
    std::vector<Group> groups;
    for (std::uint64_t start = 0; start < iso.size(); start += chunk)
        groups.push_back(plain_group(iso.data() + start, std::size_t(std::min<std::uint64_t>(chunk, iso.size() - start)),
                                     compression, level));
    return write(out, iso.size(), iso.data(), chunk, single_raw_entry(iso.size(), groups.size()), groups, compression, level,
                 error);
}

// A plain image read from a file, a group at a time.
inline bool from_iso_file(const std::string& path, std::uint32_t chunk, int compression, int level, const std::string& out,
                          std::string& error) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        error = "cannot read " + path;
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    const std::uint64_t size = std::uint64_t(ftello(f));
    std::fseek(f, 0, SEEK_SET);
    std::vector<Group> groups;
    std::vector<std::uint8_t> buffer(chunk);
    std::uint8_t dhead[0x80] = {};
    for (std::uint64_t start = 0; start < size; start += chunk) {
        const std::size_t n = std::size_t(std::min<std::uint64_t>(chunk, size - start));
        if (std::fread(buffer.data(), 1, n, f) != n) {
            std::fclose(f);
            error = "short read from " + path;
            return false;
        }
        if (!start) std::memcpy(dhead, buffer.data(), 0x80);
        groups.push_back(plain_group(buffer.data(), n, compression, level));
    }
    std::fclose(f);
    return write(out, size, dhead, chunk, single_raw_entry(size, groups.size()), groups, compression, level, error);
}

// Recompresses an uncompressed GameCube RVZ with Zstandard; groups keep their packing.
inline bool recompress(const std::string& in, int level, const std::string& out, std::string& error) {
    std::FILE* f = std::fopen(in.c_str(), "rb");
    if (!f) {
        error = "cannot read " + in;
        return false;
    }
    std::uint8_t head[0x48 + 0xDC];
    if (std::fread(head, 1, sizeof head, f) != sizeof head || std::memcmp(head, "RVZ\x01", 4) != 0) {
        std::fclose(f);
        error = "not an RVZ image";
        return false;
    }
    const std::uint8_t* d = head + 0x48;
    if (be32(d) != 1 || be32(d + 4) != 0 || be32(d + 0x90) != 0) {
        std::fclose(f);
        error = "needs an uncompressed GameCube RVZ";
        return false;
    }
    std::vector<std::uint8_t> raws(std::size_t(be32(d + 0xB4)) * 24), entries(std::size_t(be32(d + 0xC4)) * 12);
    bool ok = !fseeko(f, off_t(be64(d + 0xB8)), SEEK_SET) && std::fread(raws.data(), 1, raws.size(), f) == raws.size() &&
              !fseeko(f, off_t(be64(d + 0xC8)), SEEK_SET) && std::fread(entries.data(), 1, entries.size(), f) == entries.size();
    std::vector<Group> groups(entries.size() / 12);
    for (std::size_t i = 0; ok && i < groups.size(); ++i) {
        const std::uint8_t* e = &entries[i * 12];
        Group& g = groups[i];
        g.packed = be32(e + 8);
        g.data.resize(be32(e + 4) & 0x7FFFFFFFu);
        ok = g.data.empty() || (!fseeko(f, off_t(std::uint64_t(be32(e)) << 2), SEEK_SET) &&
                                std::fread(g.data.data(), 1, g.data.size(), f) == g.data.size());
        compress(g, level);
    }
    std::fclose(f);
    if (!ok) {
        error = "cannot read " + in;
        return false;
    }
    return write(out, be64(head + 0x24), d + 0x10, be32(d + 0xC), raws, groups, 5, level, error);
}

} // namespace rvzw

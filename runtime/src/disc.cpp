// Disc images: plain ISO/GCM, and RVZ as described in Dolphin's docs/WiaAndRvz.md
// (GameCube images, uncompressed groups, RVZ packing with lagged-Fibonacci junk runs).
#include "gcrt/disc.hpp"

#include <array>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

namespace gcrt {
namespace {

std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3];
}
std::uint64_t be64(const std::uint8_t* p) { return (std::uint64_t(be32(p)) << 32) | be32(p + 4); }

bool pread_all(int fd, void* out, std::size_t size, std::uint64_t offset) {
    auto* p = static_cast<std::uint8_t*>(out);
    while (size) {
        ssize_t n = pread(fd, p, size, static_cast<off_t>(offset));
        if (n <= 0) return false;
        p += n;
        offset += static_cast<std::uint64_t>(n);
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

class IsoDisc final : public Disc {
public:
    IsoDisc(int fd, std::uint64_t size) : fd_(fd), size_(size) {}
    ~IsoDisc() override { close(fd_); }
    std::uint64_t size() const override { return size_; }
    bool read(std::uint64_t offset, void* out, std::size_t size) override {
        return offset + size <= size_ && pread_all(fd_, out, size, offset);
    }

private:
    int fd_;
    std::uint64_t size_;
};

// Lagged Fibonacci generator for disc padding (k = 521, j = 32, 17-word seed). The output
// bytes are each state word's big-endian bytes with bits 16-23 taken from bits 18-25.
class Junk {
public:
    explicit Junk(const std::uint8_t* seed) {
        for (int i = 0; i < 17; ++i) state_[i] = be32(seed + i * 4);
        for (int i = 17; i < 521; ++i) state_[i] = (state_[i - 17] << 23) ^ (state_[i - 16] >> 9) ^ state_[i - 1];
        for (auto& x : state_) x = (x & 0xFF00FFFFu) | ((x >> 2) & 0x00FF0000u);
        for (int i = 0; i < 4; ++i) step();
    }
    void skip(std::size_t n) {
        position_ += n;
        while (position_ >= 521 * 4) {
            step();
            position_ -= 521 * 4;
        }
    }
    void take(std::uint8_t* out, std::size_t n) {
        while (n) {
            std::size_t chunk = std::min(n, 521 * 4 - position_);
            for (std::size_t i = 0; i < chunk; ++i) {
                std::size_t at = position_ + i;
                out[i] = static_cast<std::uint8_t>(state_[at / 4] >> (24 - 8 * (at % 4)));
            }
            out += chunk;
            n -= chunk;
            position_ += chunk;
            if (position_ == 521 * 4) {
                step();
                position_ = 0;
            }
        }
    }

private:
    void step() {
        for (int i = 0; i < 32; ++i) state_[i] ^= state_[i + 521 - 32];
        for (int i = 32; i < 521; ++i) state_[i] ^= state_[i - 32];
    }
    std::array<std::uint32_t, 521> state_{};
    std::size_t position_ = 0;
};

class RvzDisc final : public Disc {
public:
    explicit RvzDisc(int fd) : fd_(fd) {}
    ~RvzDisc() override { close(fd_); }

    bool open(std::string& error) {
        std::uint8_t head[0x48 + 0xDC];
        if (!pread_all(fd_, head, sizeof head, 0) || std::memcmp(head, "RVZ\x01", 4) != 0) {
            error = "not an RVZ image";
            return false;
        }
        iso_size_ = be64(head + 0x24);
        const std::uint8_t* d = head + 0x48;
        if (be32(d) != 1) {
            error = "only GameCube RVZ images are supported";
            return false;
        }
        std::uint32_t compression = be32(d + 4);
        if (compression != 0) {
            error = "compressed RVZ images are not supported yet (compression " + std::to_string(compression) + ")";
            return false;
        }
        chunk_ = be32(d + 0xC);
        std::memcpy(dhead_, d + 0x10, 0x80);
        const std::uint8_t* p = d + 0x90 + 16 + 20;
        std::uint32_t raw_count = be32(p);
        std::uint64_t raw_offset = be64(p + 4);
        std::uint32_t raw_size = be32(p + 12);
        std::uint32_t group_count = be32(p + 16);
        std::uint64_t group_offset = be64(p + 20);
        std::uint32_t group_size = be32(p + 28);
        std::vector<std::uint8_t> raws(raw_size), groups(group_size);
        if (!pread_all(fd_, raws.data(), raws.size(), raw_offset) || !pread_all(fd_, groups.data(), groups.size(), group_offset)) {
            error = "cannot read the RVZ tables";
            return false;
        }
        for (std::uint32_t i = 0; i < group_count; ++i)
            groups_.push_back({be32(&groups[i * 12]), be32(&groups[i * 12 + 4]), be32(&groups[i * 12 + 8])});
        for (std::uint32_t i = 0; i < raw_count; ++i) {
            const std::uint8_t* r = &raws[i * 24];
            Raw raw{be64(r), be64(r + 8), be32(r + 16), be32(r + 20)};
            raws_.push_back(raw);
        }
        return true;
    }

    std::uint64_t size() const override { return iso_size_; }

    bool read(std::uint64_t offset, void* destination, std::size_t size) override {
        if (offset + size > iso_size_) return false;
        auto* out = static_cast<std::uint8_t*>(destination);
        while (size) {
            if (offset < 0x80) {
                std::size_t n = std::min<std::size_t>(size, 0x80 - offset);
                std::memcpy(out, dhead_ + offset, n);
                out += n;
                offset += n;
                size -= n;
                continue;
            }
            const Raw* raw = nullptr;
            for (const auto& r : raws_)
                if (offset >= r.offset && offset < r.offset + r.size) raw = &r;
            std::size_t n;
            if (!raw) {
                n = size;
                for (const auto& r : raws_)
                    if (r.offset > offset) n = std::min<std::size_t>(n, r.offset - offset);
                std::memset(out, 0, n);
            } else {
                std::uint64_t base = raw->offset - raw->offset % 0x8000;
                std::uint64_t index = (offset - base) / chunk_;
                std::uint64_t start = base + index * chunk_;
                if (index >= raw->group_count || !load(raw->group_index + static_cast<std::uint32_t>(index), start,
                                                         std::min<std::uint64_t>(chunk_, raw->offset + raw->size - start)))
                    return false;
                std::size_t within = static_cast<std::size_t>(offset - start);
                n = std::min<std::size_t>(size, cached_.size() - within);
                std::memcpy(out, cached_.data() + within, n);
            }
            out += n;
            offset += n;
            size -= n;
        }
        return true;
    }

private:
    struct Group {
        std::uint32_t offset4, size, packed;
    };
    struct Raw {
        std::uint64_t offset, size;
        std::uint32_t group_index, group_count;
    };

    bool load(std::uint32_t group, std::uint64_t start, std::uint64_t length) {
        if (group == cached_group_) return true;
        if (group >= groups_.size()) return false;
        const Group& g = groups_[group];
        std::uint32_t stored = g.size & 0x7FFFFFFFu;
        cached_.assign(static_cast<std::size_t>(length), 0);
        if (stored) {
            std::vector<std::uint8_t> data(stored);
            if (!pread_all(fd_, data.data(), stored, std::uint64_t(g.offset4) << 2)) return false;
            if (!g.packed) {
                std::memcpy(cached_.data(), data.data(), std::min<std::size_t>(stored, cached_.size()));
            } else if (!unpack(data, start)) {
                return false;
            }
        }
        cached_group_ = group;
        return true;
    }

    bool unpack(const std::vector<std::uint8_t>& data, std::uint64_t disc_offset) {
        std::size_t in = 0, out = 0;
        while (out < cached_.size()) {
            if (in + 4 > data.size()) return false;
            std::uint32_t size = be32(&data[in]);
            in += 4;
            bool junk = size & 0x80000000u;
            size &= 0x7FFFFFFFu;
            std::size_t n = std::min<std::size_t>(size, cached_.size() - out);
            if (junk) {
                if (in + 68 > data.size()) return false;
                Junk generator(&data[in]);
                in += 68;
                generator.skip((disc_offset + out) % 0x8000);
                generator.take(&cached_[out], n);
            } else {
                if (in + size > data.size()) return false;
                std::memcpy(&cached_[out], &data[in], n);
                in += size;
            }
            out += size;
        }
        return true;
    }

    int fd_;
    std::uint64_t iso_size_ = 0;
    std::uint32_t chunk_ = 0;
    std::uint8_t dhead_[0x80] = {};
    std::vector<Group> groups_;
    std::vector<Raw> raws_;
    std::vector<std::uint8_t> cached_;
    std::uint32_t cached_group_ = ~0u;
};

} // namespace

std::unique_ptr<Disc> open_disc(const std::string& path, std::string& error) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        error = "cannot open disc image " + path;
        return nullptr;
    }
    std::uint8_t magic[4] = {};
    pread_all(fd, magic, 4, 0);
    if (std::memcmp(magic, "RVZ\x01", 4) == 0) {
        auto disc = std::make_unique<RvzDisc>(fd);
        if (!disc->open(error)) return nullptr;
        return disc;
    }
    std::uint64_t size = static_cast<std::uint64_t>(lseek(fd, 0, SEEK_END));
    return std::make_unique<IsoDisc>(fd, size);
}

} // namespace gcrt

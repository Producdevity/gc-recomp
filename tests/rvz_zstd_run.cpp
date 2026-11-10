// The test RVZ writer against Starlet's reader, which the runtime reads discs with:
// uncompressed and Zstandard images of a synthetic disc read back byte for byte, including
// stored groups inside a compressed image, all-zero groups and a partial last group;
// bzip2/LZMA images are refused with a clear message.
#include "rvz_writer.hpp"
#include "starlet/disc/image.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {
unsigned failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) {
        std::printf("FAIL %s\n", what.c_str());
        ++failures;
    }
}

// Reads the whole image in uneven pieces that cross group boundaries; a read past the end
// must fail.
bool same(const std::string& path, const std::vector<std::uint8_t>& iso, std::string& error) {
    std::unique_ptr<starlet::disc::DiscImage> disc;
    std::vector<std::uint8_t> got(iso.size());
    try {
        disc = starlet::disc::open_disc_image(path);
        if (disc->size() != iso.size()) {
            error = "size " + std::to_string(disc->size());
            return false;
        }
        for (std::size_t at = 0, step = 7; at < iso.size(); step = step * 3 % 70001 + 1) {
            const std::size_t n = std::min(step, iso.size() - at);
            disc->read(at, got.data() + at, n);
            at += n;
        }
    } catch (const starlet::disc::DiscError& e) {
        error = e.what();
        return false;
    }
    for (std::size_t i = 0; i < iso.size(); ++i)
        if (got[i] != iso[i]) {
            error = "byte " + std::to_string(i) + " differs";
            return false;
        }
    try {
        disc->read(iso.size() - 4, got.data(), 8);
    } catch (const starlet::disc::DiscError&) {
        return true;
    }
    error = "a read past the end succeeded";
    return false;
}
} // namespace

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : ".";
    constexpr std::uint32_t kChunk = 0x20000;
    std::vector<std::uint8_t> iso(3 * 0x100000 + 0x1234, 0);
    std::memcpy(iso.data(), "GLME01", 6);
    std::uint32_t x = 12345;
    for (std::size_t i = 0x80; i < 0x100000; ++i) {   // incompressible
        x = x * 1103515245u + 12345u;
        iso[i] = std::uint8_t(x >> 24);
    }
    // 0x100000-0x200000 stays zero; the rest compresses well.
    for (std::size_t i = 0x200000; i < iso.size(); ++i) iso[i] = std::uint8_t("gc-recomp RVZ test "[i % 19]);

    std::string error;
    const std::string plain = dir + "/rvz-none.rvz", zstd = dir + "/rvz-zstd.rvz", again = dir + "/rvz-recompressed.rvz";
    check(rvzw::from_iso(iso, kChunk, 0, 0, plain, error), "write uncompressed: " + error);
    check(same(plain, iso, error), "uncompressed RVZ reads back: " + error);
    check(rvzw::from_iso(iso, kChunk, 5, 5, zstd, error), "write zstd: " + error);
    check(same(zstd, iso, error), "zstd RVZ reads back: " + error);
    check(rvzw::recompress(plain, 9, again, error), "recompress: " + error);
    check(same(again, iso, error), "recompressed RVZ reads back: " + error);

    // A bzip2/LZMA image is refused, naming the method and the supported alternatives.
    for (std::uint32_t method : {2u, 3u, 4u}) {
        std::FILE* f = std::fopen(zstd.c_str(), "r+b");
        std::uint8_t head[0x48 + 0xDC];
        check(std::fread(head, 1, sizeof head, f) == sizeof head, "read back the header");
        rvzw::put32(head + 0x48 + 4, method);
        rvzw::seal(head);
        std::fseek(f, 0, SEEK_SET);
        std::fwrite(head, 1, sizeof head, f);
        std::fclose(f);
        error.clear();
        bool opened = true;
        try {
            starlet::disc::open_disc_image(zstd);
        } catch (const starlet::disc::DiscError& e) {
            opened = false;
            error = e.what();
        }
        const char* name = method == 2 ? "bzip2" : method == 3 ? "LZMA" : "LZMA2";
        check(!opened && error.find(name) != std::string::npos && error.find("Zstandard") != std::string::npos,
              std::string("refuses ") + name + ": " + error);
    }
    for (const auto& p : {plain, zstd, again}) std::remove(p.c_str());
    if (failures) return 1;
    std::printf("rvz zstd: ok\n");
    return 0;
}

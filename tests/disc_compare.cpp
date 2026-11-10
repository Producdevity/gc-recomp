// Usage: gc-disc-compare A B -- reads both images through Starlet's disc readers and
// reports the first differing offset.
#include "starlet/disc/image.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    try {
        auto a = starlet::disc::open_disc_image(argv[1]), b = starlet::disc::open_disc_image(argv[2]);
        if (a->size() != b->size()) {
            std::printf("sizes differ: %llu %llu\n", (unsigned long long)a->size(), (unsigned long long)b->size());
            return 1;
        }
        std::vector<unsigned char> x(1 << 20), y(1 << 20);
        for (std::uint64_t at = 0; at < a->size(); at += x.size()) {
            const std::size_t n = static_cast<std::size_t>(std::min<std::uint64_t>(x.size(), a->size() - at));
            a->read(at, x.data(), n);
            b->read(at, y.data(), n);
            if (std::memcmp(x.data(), y.data(), n) != 0)
                for (std::size_t i = 0; i < n; ++i)
                    if (x[i] != y[i]) {
                        std::printf("differ at %llx\n", (unsigned long long)(at + i));
                        return 1;
                    }
        }
        std::printf("identical, %llu bytes\n", (unsigned long long)a->size());
        return 0;
    } catch (const starlet::disc::DiscError& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
}

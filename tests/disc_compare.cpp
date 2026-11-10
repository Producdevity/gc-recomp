// Usage: gc-disc-compare A B -- reads both images through the runtime's readers and
// reports the first differing offset.
#include "gcrt/disc.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::string error;
    auto a = gcrt::open_disc(argv[1], error), b = gcrt::open_disc(argv[2], error);
    if (!a || !b) { std::fprintf(stderr, "%s\n", error.c_str()); return 1; }
    if (a->size() != b->size()) { std::printf("sizes differ: %llu %llu\n", (unsigned long long)a->size(), (unsigned long long)b->size()); return 1; }
    std::vector<unsigned char> x(1 << 20), y(1 << 20);
    for (std::uint64_t at = 0; at < a->size(); at += x.size()) {
        std::size_t n = static_cast<std::size_t>(std::min<std::uint64_t>(x.size(), a->size() - at));
        if (!a->read(at, x.data(), n) || !b->read(at, y.data(), n)) { std::printf("read failed at %llx\n", (unsigned long long)at); return 1; }
        if (std::memcmp(x.data(), y.data(), n) != 0) {
            for (std::size_t i = 0; i < n; ++i) if (x[i] != y[i]) { std::printf("differ at %llx\n", (unsigned long long)(at + i)); return 1; }
        }
    }
    std::printf("identical, %llu bytes\n", (unsigned long long)a->size());
    return 0;
}

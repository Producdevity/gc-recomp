// gcbe: translates a GameCube DOL to an AArch64 code image for the runtime's loader.
//
//   gcbe (--dol main.dol | --disc game.iso) --symbols symbols.txt [--config game.cfg] --out FILE
//        [--virtual-time] [--exact] [--no-natives] [--progress]
#include "a64_translate.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/resource.h>

int main(int argc, char** argv) {
    gcbe::Request r;
    bool show_progress = false;
    for (int n = 1; n < argc; ++n) {
        std::string a = argv[n];
        auto value = [&]() -> std::string {
            if (n + 1 >= argc) {
                std::fprintf(stderr, "gcbe: missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++n];
        };
        if (a == "--dol") r.dol = value();
        else if (a == "--disc") r.disc = value();
        else if (a == "--symbols") r.symbols = value();
        else if (a == "--config") r.config = value();
        else if (a == "--out") r.out = value();
        else if (a == "--virtual-time") r.options.virtual_time = true;
        else if (a == "--exact") r.exact = true;
        else if (a == "--no-natives") r.natives = false;
        else if (a == "--progress") show_progress = true;
        else {
            std::fprintf(stderr, "gcbe: unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    if ((r.dol.empty() && r.disc.empty()) || r.symbols.empty() || r.out.empty()) {
        std::fprintf(stderr, "usage: gcbe (--dol DOL | --disc ISO) --symbols SYMBOLS [--config CFG] --out FILE [--virtual-time]\n");
        return 2;
    }
    auto start = std::chrono::steady_clock::now();
    gcbe::Stats stats;
    std::string error;
    unsigned last = 0;
    auto progress = [&](std::size_t done, std::size_t total) {
        unsigned pct = static_cast<unsigned>(done * 100 / total);
        if (show_progress && pct != last) {
            last = pct;
            std::fprintf(stderr, "\rgcbe: %u%%", pct);
        }
    };
    if (!gcbe::translate_files(r, stats, progress, error)) {
        std::fprintf(stderr, "gcbe: %s\n", error.c_str());
        return 1;
    }
    double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    rusage u{};
    getrusage(RUSAGE_SELF, &u);
#ifdef __APPLE__
    long rss_kb = u.ru_maxrss / 1024;
#else
    long rss_kb = u.ru_maxrss;
#endif
    if (show_progress) std::fprintf(stderr, "\n");
    std::printf("functions %zu, instructions %zu, code %zu bytes, spilled values %zu, invalid %zu, spin loops %zu, "
                "%.2f s, peak RSS %ld KB\n",
                stats.functions, stats.instructions, stats.code_bytes, stats.spilled_values, stats.invalid,
                stats.spin_loops, seconds, rss_kb);
    return 0;
}

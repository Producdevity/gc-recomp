// Makes Zstandard RVZ test images of a GameCube disc (see rvz_writer.hpp):
//   gc-rvz-zstd iso IMAGE.iso OUT.rvz [LEVEL]   plain groups from an ISO/GCM
//   gc-rvz-zstd rvz IMAGE.rvz OUT.rvz [LEVEL]   an uncompressed RVZ recompressed, packing kept
#include "rvz_writer.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s (iso|rvz) IN OUT.rvz [LEVEL]\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    const int level = argc > 4 ? std::atoi(argv[4]) : 5;
    std::string error;
    const bool ok = mode == "iso" ? rvzw::from_iso_file(argv[2], 0x20000, 5, level, argv[3], error)
                                  : rvzw::recompress(argv[2], level, argv[3], error);
    if (!ok) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }
    return 0;
}

// On-device translation entry point: DOL (or disc image), function list and game
// configuration in, translated code image (gcbe/blob.h) out.
#pragma once

#include "a64_backend.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace gcbe {

struct Request {
    std::string dol, disc, symbols, config, out;
    std::vector<std::uint8_t> dol_bytes;   // used instead of dol/disc when not empty
    Options options;
    bool exact = false;   // every function synchronises all registers (gcrecomp --exact)
    bool natives = true;  // false: translate recognised SDK functions too (gcbe --no-natives)
};

// Writes `out` atomically (temporary file, then rename).
bool translate_files(const Request& request, Stats& stats, const Progress& progress, std::string& error);

} // namespace gcbe

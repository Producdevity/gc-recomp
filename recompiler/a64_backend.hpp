// Direct AArch64 backend: translates a Program into one relocatable code image (gcbe/blob.h)
// that the runtime maps at run time. Same guest semantics and register write-back rules as
// the C emitter (emit.cpp), except that direct calls pass arguments and results in host
// registers (a64_backend.cpp).
#pragma once

#include "emit.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace gcbe {

struct Options {
    bool virtual_time = false;   // count loop back-edges as guest time (GC_VIRTUAL_TIME)
};

struct Stats {
    std::size_t functions = 0, instructions = 0, code_bytes = 0, spilled_values = 0, invalid = 0, spin_loops = 0;
};

// Called after each translated function: (functions done, total).
using Progress = std::function<void(std::size_t done, std::size_t total)>;

bool translate(const gcr::Program& program, const Options& options, std::vector<std::uint8_t>& blob, Stats& stats,
               const Progress& progress, std::string& error);

// SHA-1 (FIPS 180-4) of a byte string.
void sha1(const std::uint8_t* data, std::size_t size, std::uint8_t out[20]);

} // namespace gcbe

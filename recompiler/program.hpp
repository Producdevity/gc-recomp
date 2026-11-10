// Builds the recompiler's Program (function list, configuration, code pointers) from a DOL
// image, a dtk symbols list and the per-game configuration. Shared by gcrecomp and the
// on-device AArch64 backend.
#pragma once

#include "emit.hpp"
#include "image.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace gcr {

struct ProgramOptions {
    bool check_returns = false;
    bool exact = false;
    bool natives = true;   // replace recognised SDK functions (natives.hpp)
};

bool build_program(const Image& image, const std::vector<FunctionSymbol>& symbols, const Config& config,
                   const ProgramOptions& options, Program& program, std::string& error);

std::uint64_t fnv1a(const std::vector<std::uint8_t>& bytes);

} // namespace gcr

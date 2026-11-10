// The guest code the runtime runs: compiled into the executable at build time
// (linked_program) or translated on the device and loaded (gcbe_load).
#pragma once

#include "gcrt/recomp.h"

#include <cstdint>

namespace gcrt {

struct Program {
    const gc_func_entry* functions = nullptr;
    unsigned count = 0;
    std::uint64_t dol_hash = 0;   // FNV-1a of the DOL the code was made from
};

// The table gc_recompile compiled into this executable; defined in the game library.
Program linked_program();

} // namespace gcrt

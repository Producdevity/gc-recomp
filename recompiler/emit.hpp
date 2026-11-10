#pragma once

#include "image.hpp"
#include "ppc.hpp"

#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace gcr {

struct Function {
    std::uint32_t start = 0, end = 0;
    std::string name;
    std::string hle;          // Runtime replacement, if any.
    std::string wrap;         // Runtime function called instead, given the original as gc_func.
    bool exact = false;       // Flush every written register at calls and returns.
};

struct Program {
    const Image* image = nullptr;
    std::vector<Function> functions;                  // Sorted by start.
    std::unordered_map<std::uint32_t, std::size_t> by_start;
    std::set<std::uint32_t> save_context;             // Callees with OSSaveContext semantics.
    std::vector<std::uint32_t> code_pointers;         // Sorted data words that point into text.
    bool check_returns = false;                       // Emit LR checks at returns.

    const Function* function_at(std::uint32_t start) const {
        auto it = by_start.find(start);
        return it == by_start.end() ? nullptr : &functions[it->second];
    }
};

struct EmitStats {
    std::size_t instructions = 0, invalid = 0, indirect_jumps = 0, unresolved = 0, spin_loops = 0;
};

// C source for one function.
std::string emit_function(const Program&, const Function&, EmitStats&);

} // namespace gcr

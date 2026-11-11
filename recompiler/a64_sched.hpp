// Instruction scheduling of emitted A64 code for the in-order, dual-issue Cortex-A53.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace a64 {

// Reorders the instructions of code[0, end) within basic blocks. leader[k] marks positions that
// branches or entries reach; blocks also end after every branch, call and return, which keep their
// positions. target[k] is the position a branch at k reaches (-1: another function or none).
// Instructions this pass does not decode stay in place and split blocks. Returns the new position
// of the instruction that was at each position below end.
std::vector<std::uint32_t> schedule_a53(std::vector<std::uint32_t>& code, std::size_t end, const std::vector<bool>& leader,
                                        const std::vector<int>& target);

} // namespace a64

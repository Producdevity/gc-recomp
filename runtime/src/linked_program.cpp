// Compiled into each gc_recompile library: the table its generated sources define.
#include "gcrt/program.hpp"

namespace gcrt {
Program linked_program() { return {gc_func_table, gc_func_count, gc_dol_hash}; }
} // namespace gcrt

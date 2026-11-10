#pragma once

#include "gcrt/recomp.h"

#include <cstdint>
#include <pthread.h>
#include <string>

namespace gcrt {

// Samples the calling (guest) thread's recompiled call stack at the given rate.
void sampler_start(unsigned hz);
// The same for another thread of this process (the GX or GL thread).
void sampler_start_thread(pthread_t thread, unsigned hz);
void sampler_report(unsigned top);
void sampler_reset();
// Names translated (symbol-less) code: samples inside it count for the guest function
// whose entry point precedes them.
void sampler_add_functions(const gc_func_entry* functions, unsigned count);
// Development: one line per sample, "module+offset" for the PC, its LR and its callers,
// for offline symbolization, then the guest thread and the CPU time the sampled thread
// used since the previous sample (near zero while it waits).
void sampler_dump(const std::string& path);
// Reports guest fault addresses and the recompiled call stack on fatal signals.
void install_fault_handler();
// Logs the recompiled functions on the calling thread's stack.
void log_guest_stack(const char* why);
// Reports a device write into the watched range (gc_watch_lo/hi).
void watch_dma(const char* what, std::uint32_t physical, std::uint32_t size);

} // namespace gcrt

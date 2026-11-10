// Runtime internals shared by the CPU, OS and hardware modules.
#pragma once

#include "gcrt/program.hpp"
#include "gcrt/recomp.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>

namespace gcrt {

constexpr std::uint32_t kRamSize = 24u * 1024 * 1024;
constexpr std::uint32_t kAramSize = 16u * 1024 * 1024;

// ---- Memory -------------------------------------------------------------------------
bool memory_init();
// Write generations of MEM1 for the GX caches. A range query write-protects the pages it
// covers (both mirrors) and returns their newest generation; the first write to such a page
// faults, makes it writable and gives it a newer generation at the next query. Unchanged
// nonzero results promise unchanged bytes. Zero before ram_tracking_enable or out of range.
void ram_tracking_enable();
std::uint64_t ram_range_generation(std::uint32_t physical, std::uint32_t size);
// Before a system call writes guest RAM (a write-protected page would fail it).
void ram_will_write(std::uint32_t physical, std::uint32_t size);
// Write faults taken on tracked pages since start.
std::uint64_t ram_tracking_faults();
inline std::uint8_t* host(std::uint32_t virtual_address) { return gc_mem + virtual_address; }
// Physical RAM address (0x00000000-0x017FFFFF) to host pointer.
inline std::uint8_t* phys(std::uint32_t physical) { return gc_mem + 0x80000000u + (physical & 0x01FFFFFFu); }
inline bool phys_ok(std::uint32_t physical, std::uint32_t size) {
    physical &= 0x3FFFFFFFu;
    return physical < kRamSize && size <= kRamSize - physical;
}
inline std::uint32_t read32(std::uint32_t va) { return gc_ld32(gc_mem, va); }
inline std::uint32_t read16(std::uint32_t va) { return gc_ld16(gc_mem, va); }
inline std::uint32_t read8(std::uint32_t va) { return gc_ld8(gc_mem, va); }
inline void write32(std::uint32_t va, std::uint32_t v) { gc_st32(gc_mem, va, v); }
inline void write16(std::uint32_t va, std::uint32_t v) { gc_st16(gc_mem, va, v); }
inline void write8(std::uint32_t va, std::uint32_t v) { gc_st8(gc_mem, va, v); }
std::uint8_t* aram();

// ---- Diagnostics --------------------------------------------------------------------
[[noreturn]] void fatal(const char* format, ...);
void log(const char* format, ...);
extern int verbosity;

// ---- Time and events (host monotonic clock) ------------------------------------------
constexpr std::uint64_t kTimebaseHz = 40500000;   // bus clock / 4
std::uint64_t now_ns();
// Deterministic mode (GCRT_VIRTUAL_TIME): see cpu.cpp.
extern bool virtual_time;
void advance_virtual_time(std::uint64_t to_ns);
void set_virtual_deadline(std::uint64_t when_ns);
std::uint64_t timebase();
void set_timebase(std::uint64_t);
std::uint64_t ns_to_ticks(std::uint64_t);
std::uint64_t ticks_to_ns(std::uint64_t);
// Runs fn on the guest thread at or after the given host time.
using EventId = std::uint64_t;
EventId schedule_at(std::uint64_t when_ns, std::function<void()> fn);
EventId schedule_in(std::uint64_t delay_ns, std::function<void()> fn);
void cancel(EventId);
// Process due events on the guest thread; true if any ran.
bool run_due_events();
// Block the guest until the next event or a wakeup, at most max_ns.
void wait_for_event(std::uint64_t max_ns);
// Any host thread: wake the guest (interrupt raised from another thread).
void wake_guest();
// A device register that changes with time was read: spin loops must keep polling
// instead of sleeping until the next event.
void note_time_poll();
void events_start();
void events_stop();

// ---- CPU ------------------------------------------------------------------------------
constexpr std::uint32_t MSR_EE = 0x8000, MSR_FP = 0x2000;
// Clears CPU state and installs the program's function table.
void cpu_init(const Program&);
gc_func lookup(std::uint32_t address);
std::uint32_t& spr(std::uint32_t n);
bool decrementer_pending();
// Raises gc_attn and the running context's copy of it (gc_ctx::attn); any thread.
void raise_attention();
// The context that raise_attention marks: the running guest thread's (guest thread only).
void set_attention_context(gc_ctx*);
void clear_decrementer();

// ---- Interrupts -------------------------------------------------------------------------
// Processor-interface interrupt causes (PI INTSR bits).
enum : std::uint32_t {
    PI_ERROR = 0x1, PI_RSW = 0x2, PI_DI = 0x4, PI_SI = 0x8, PI_EXI = 0x10, PI_AI = 0x20, PI_DSP = 0x40,
    PI_MEM = 0x80, PI_VI = 0x100, PI_PE_TOKEN = 0x200, PI_PE_FINISH = 0x400, PI_CP = 0x800, PI_DEBUG = 0x1000,
    PI_HSP = 0x2000,
};
// Devices call this whenever their interrupt output may have changed.
void interrupts_changed();
std::uint32_t pi_cause();      // Recomputed from device state.
std::uint32_t pi_mask();

// ---- OS bridge ------------------------------------------------------------------------
gc_ctx* current_context();
void os_init(gc_ctx* boot);
std::uint64_t fiber_count();
// The running guest fiber's stack, or false on the thread's own stack.
bool fiber_stack_range(std::uintptr_t& low, std::uintptr_t& high);
// Real time the guest spent waiting with no runnable thread, and the part of it during
// which the GX thread still had commands to process.
struct IdleTime { std::uint64_t total_ns, gx_busy_ns; };
IdleTime idle_time();

// ---- Devices ----------------------------------------------------------------------------
struct Device {
    virtual ~Device() = default;
    virtual std::uint32_t read16(std::uint32_t offset) { (void)offset; return 0; }
    virtual std::uint32_t read32(std::uint32_t offset) { return (read16(offset) << 16) | read16(offset + 2); }
    virtual void write16(std::uint32_t offset, std::uint32_t value) { (void)offset; (void)value; }
    virtual void write32(std::uint32_t offset, std::uint32_t value) {
        write16(offset, value >> 16);
        write16(offset + 2, value & 0xFFFF);
    }
    virtual std::uint32_t irq() { return 0; }   // PI cause bits this device raises.
    virtual const char* name() const = 0;
};
void register_device(std::uint32_t base, std::uint32_t size, Device*);

} // namespace gcrt

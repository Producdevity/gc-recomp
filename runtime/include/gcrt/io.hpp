// Hooks between the register dispatcher and the graphics pipeline.
#pragma once

#include "gcrt/recomp.h"

#include <cstddef>
#include <cstdint>

namespace gcrt {

// Writes to the CPU's write-gather pipe (0xCC008000), in program order.
// The write-gather pipe behind 0xCC008000. Its open burst is gc_gp (recomp.h), which
// recompiled stores append to directly; each full burst comes here.
class GatherPipe {
public:
    virtual ~GatherPipe() = default;
    // Bytes stored to 0xCC008000 that gc_gather_store did not take, in order.
    static void write(const std::uint8_t* bytes, std::size_t size) {
        for (std::size_t i = 0; i < size; ++i) {
            gc_gp.bytes[gc_gp.fill++] = bytes[i];
            if (gc_gp.fill >= GC_GATHER_BATCH) gc_gather_burst();
        }
    }
    static std::size_t gathered() { return gc_gp.fill; }
    // count full 32-byte bursts, in order.
    virtual void burst(const std::uint8_t* bytes, std::size_t count) = 0;
    // The CPU FIFO registers in the processor interface.
    virtual void set_cpu_fifo(std::uint32_t base, std::uint32_t end, std::uint32_t write) = 0;
    virtual std::uint32_t write_pointer() = 0;
};

// CPU access to the embedded framebuffer at 0xC8000000.
class EfbAccess {
public:
    virtual ~EfbAccess() = default;
    virtual std::uint32_t read32(std::uint32_t address) = 0;
    virtual void write32(std::uint32_t address, std::uint32_t value) = 0;
    virtual void write16(std::uint32_t address, std::uint32_t value) = 0;
};

struct CpuFifo {
    std::uint32_t base, end, write;
};

void io_init();
void set_gather_pipe(GatherPipe*);
void set_efb_access(EfbAccess*);
void raise_pi(std::uint32_t bits);
CpuFifo cpu_fifo();
void set_cpu_fifo_write(std::uint32_t);

} // namespace gcrt

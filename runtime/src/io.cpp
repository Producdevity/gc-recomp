// Hardware register dispatch (0xCC000000), the processor interface and the memory interface.
#include "gcrt/io.hpp"
#include "gcrt/runtime.hpp"

#include <cstring>
#include <vector>

namespace gcrt {
namespace {
struct Mapping {
    std::uint32_t base, size;
    Device* device;
};
std::vector<Mapping> mappings;
GatherPipe* pipe = nullptr;
EfbAccess* efb = nullptr;

// Processor interface: interrupt cause and mask, CPU FIFO pointers, reset and revision.
struct ProcessorInterface final : Device {
    std::uint32_t mask = 0, fifo_base = 0, fifo_end = 0, fifo_write = 0, reset = 0, latched = 0;
    const char* name() const override { return "PI"; }
    std::uint32_t read32(std::uint32_t o) override {
        switch (o) {
        case 0x00: return pi_cause() | 0x00010000u;   // reset switch released
        case 0x04: return mask;
        case 0x0C: return fifo_base;
        case 0x10: return fifo_end;
        case 0x14: return pipe ? pipe->write_pointer() : fifo_write;
        case 0x24: return reset;
        case 0x2C: return 0x20000000u;                // hardware revision
        default: return 0;
        }
    }
    void write32(std::uint32_t o, std::uint32_t v) override {
        switch (o) {
        case 0x00: latched &= ~v; interrupts_changed(); break;
        case 0x04: mask = v; interrupts_changed(); break;
        case 0x0C: fifo_base = v & 0x03FFFFE0u; if (pipe) pipe->set_cpu_fifo(fifo_base, fifo_end, fifo_write); break;
        case 0x10: fifo_end = v & 0x03FFFFE0u; if (pipe) pipe->set_cpu_fifo(fifo_base, fifo_end, fifo_write); break;
        case 0x14: fifo_write = v & 0x07FFFFE0u; if (pipe) pipe->set_cpu_fifo(fifo_base, fifo_end, fifo_write); break;
        case 0x24: reset = v; break;
        default: break;
        }
    }
    std::uint32_t read16(std::uint32_t o) override {
        std::uint32_t w = read32(o & ~3u);
        return (o & 2) ? (w & 0xFFFF) : (w >> 16);
    }
};

// Memory interface: protection ranges and their interrupt; nothing here is enforced.
struct MemoryInterface final : Device {
    std::uint16_t regs[0x80] = {};
    const char* name() const override { return "MI"; }
    std::uint32_t read16(std::uint32_t o) override { return o < 0x100 ? regs[o / 2] : 0; }
    void write16(std::uint32_t o, std::uint32_t v) override {
        if (o < 0x100) regs[o / 2] = static_cast<std::uint16_t>(v);
    }
};

ProcessorInterface pi_device;
MemoryInterface mi_device;

Device* find(std::uint32_t ea, std::uint32_t& offset) {
    std::uint32_t pa = ea & 0x0000FFFFu;
    for (auto& m : mappings)
        if (pa >= m.base && pa < m.base + m.size) {
            offset = pa - m.base;
            return m.device;
        }
    return nullptr;
}

// Gathered bursts reach the pipe before any other hardware access can observe it.
void gather_sync() {
    if (gc_gp.fill >= 32) gc_gather_burst();
}

[[noreturn]] void unmapped(const char* kind, std::uint32_t ea) { fatal("%s of unmapped hardware address %08x", kind, ea); }

} // namespace

void register_device(std::uint32_t base, std::uint32_t size, Device* device) { mappings.push_back({base, size, device}); }
void set_gather_pipe(GatherPipe* p) { pipe = p; }
void set_efb_access(EfbAccess* e) { efb = e; }

std::uint32_t pi_mask() { return pi_device.mask; }
std::uint32_t pi_cause() {
    std::uint32_t cause = pi_device.latched;
    for (auto& m : mappings) cause |= m.device->irq();
    return cause;
}
void interrupts_changed() {
    if ((pi_cause() & pi_device.mask) || decrementer_pending()) {
        raise_attention();
    }
}
void raise_pi(std::uint32_t bits) {
    pi_device.latched |= bits;
    interrupts_changed();
}
CpuFifo cpu_fifo() { return {pi_device.fifo_base, pi_device.fifo_end, pi_device.fifo_write}; }
void set_cpu_fifo_write(std::uint32_t w) { pi_device.fifo_write = w; }

void io_init() {
    register_device(0x3000, 0x1000, &pi_device);
    register_device(0x4000, 0x1000, &mi_device);
}

} // namespace gcrt

using namespace gcrt;

extern "C" {

uint8_t gc_io_read8(uint32_t ea) {
    gather_sync();
    std::uint32_t v = gc_io_read16(ea & ~1u);
    return static_cast<uint8_t>((ea & 1) ? v : v >> 8);
}

uint16_t gc_io_read16(uint32_t ea) {
    gather_sync();
    if ((ea >> 24) == 0xC8) {
        std::uint32_t w = efb ? efb->read32(ea & ~3u) : 0;
        return static_cast<uint16_t>((ea & 2) ? w : w >> 16);
    }
    std::uint32_t o;
    Device* d = find(ea, o);
    if (!d || (ea & 0xFFFF0000u) != 0xCC000000u) unmapped("16-bit read", ea);
    return static_cast<uint16_t>(d->read16(o));
}

uint32_t gc_io_read32(uint32_t ea) {
    gather_sync();
    if ((ea >> 24) == 0xC8) return efb ? efb->read32(ea) : 0;
    std::uint32_t o;
    Device* d = find(ea, o);
    if (!d || (ea & 0xFFFF0000u) != 0xCC000000u) unmapped("32-bit read", ea);
    return d->read32(o);
}

void gc_io_write8(uint32_t ea, uint8_t v) {
    if (gc_gather_store(ea, v, 1)) return;
    gather_sync();
    if ((ea & 0xFFFFF000u) == 0xCC008000u && pipe) {
        pipe->write(&v, 1);
        return;
    }
    unmapped("8-bit write", ea);
}

void gc_io_write16(uint32_t ea, uint16_t v) {
    if (gc_gather_store(ea, v, 2)) return;
    gather_sync();
    if ((ea & 0xFFFFF000u) == 0xCC008000u && pipe) {
        std::uint8_t b[2] = {static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v)};
        pipe->write(b, 2);
        return;
    }
    if ((ea >> 24) == 0xC8) {
        if (efb) efb->write16(ea, v);
        return;
    }
    std::uint32_t o;
    Device* d = find(ea, o);
    if (!d || (ea & 0xFFFF0000u) != 0xCC000000u) unmapped("16-bit write", ea);
    d->write16(o, v);
}

void gc_io_write32(uint32_t ea, uint32_t v) {
    if (gc_gather_store(ea, v, 4)) return;
    gather_sync();
    if ((ea & 0xFFFFF000u) == 0xCC008000u && pipe) {
        const std::uint8_t b[4] = {static_cast<std::uint8_t>(v >> 24), static_cast<std::uint8_t>(v >> 16),
                                   static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v)};
        pipe->write(b, 4);
        return;
    }
    if ((ea >> 24) == 0xC8) {
        if (efb) efb->write32(ea, v);
        return;
    }
    std::uint32_t o;
    Device* d = find(ea, o);
    if (!d || (ea & 0xFFFF0000u) != 0xCC000000u) unmapped("32-bit write", ea);
    d->write32(o, v);
}

gc_gather gc_gp = {};

void gc_gather_burst(void) {
    const std::uint32_t count = gc_gp.fill / 32, carry = gc_gp.fill % 32;
    if (!count) return;
    std::uint8_t bursts[GC_GATHER_BATCH + 8];
    std::memcpy(bursts, gc_gp.bytes, count * 32);
    std::memmove(gc_gp.bytes, gc_gp.bytes + count * 32, carry);
    gc_gp.fill = carry;
    if (pipe) pipe->burst(bursts, count);
}

void gc_io_write64(uint32_t ea, uint64_t v) {
    gc_io_write32(ea, static_cast<uint32_t>(v >> 32));
    gc_io_write32(ea + ((ea & 0xFFFFF000u) == 0xCC008000u ? 0 : 4), static_cast<uint32_t>(v));
}

} // extern "C"

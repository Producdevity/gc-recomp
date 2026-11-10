// Disc interface: command buffer, DMA into main memory and the transfer interrupt.
#include "gcrt/devices.hpp"
#include "gcrt/gx.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"
#include "starlet/disc/image.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace gcrt {
namespace {

std::unique_ptr<starlet::disc::DiscImage> disc;

struct DiscInterface final : Device {
    std::uint32_t sr = 0, cvr = 0, cmd[3] = {}, mar = 0, length = 0, cr = 0, imm = 0, cfg = 0;
    std::uint32_t stream_offset = 0, stream_length = 0;
    bool streaming = false;
    bool fast = std::getenv("GCRT_FAST_DISC") != nullptr;

    const char* name() const override { return "DI"; }
    std::uint32_t read32(std::uint32_t o) override {
        switch (o) {
        case 0x00: return sr;
        case 0x04: return cvr;
        case 0x08: return cmd[0];
        case 0x0C: return cmd[1];
        case 0x10: return cmd[2];
        case 0x14: return mar;
        case 0x18: return length;
        case 0x1C: return cr;
        case 0x20: return imm;
        case 0x24: return cfg;
        default: return 0;
        }
    }
    std::uint32_t read16(std::uint32_t o) override {
        std::uint32_t w = read32(o & ~3u);
        return (o & 2) ? (w & 0xFFFF) : (w >> 16);
    }
    void write32(std::uint32_t o, std::uint32_t v) override {
        switch (o) {
        case 0x00:
            sr = (sr & ~0x2Au) | (v & 0x2Au);         // masks
            sr &= ~(v & 0x54u);                       // write one to clear interrupts
            if (v & 1) {                              // break
                sr |= 0x40;
            }
            interrupts_changed();
            break;
        case 0x04:
            cvr = (cvr & ~0x2u) | (v & 0x2u);
            cvr &= ~(v & 0x4u);
            interrupts_changed();
            break;
        case 0x08: cmd[0] = v; break;
        case 0x0C: cmd[1] = v; break;
        case 0x10: cmd[2] = v; break;
        case 0x14: mar = v & 0x03FFFFE0u; break;
        case 0x18: length = v & ~0x1Fu; break;
        case 0x1C:
            cr = v & 7;
            if (v & 1) execute();
            break;
        case 0x20: imm = v; break;
        default: break;
        }
    }
    std::uint32_t irq() override {
        bool tc = (sr & 0x10) && (sr & 0x08), de = (sr & 0x04) && (sr & 0x02), brk = (sr & 0x40) && (sr & 0x20);
        bool cover = (cvr & 0x4) && (cvr & 0x2);
        return (tc || de || brk || cover) ? PI_DI : 0;
    }

    void dma_disc(std::uint64_t offset) {
        gx_flush();   // the GP has consumed earlier commands before memory changes under them
        if (!phys_ok(mar, length)) fatal("DI DMA outside RAM: %08x+%x", mar, length);
        if (mar < 0x3100) log("DI DMA into low memory: %08x+%x from disc %llx\n", mar, length, (unsigned long long)offset);
        if (mar < 0x3100) {
            log_guest_stack("DI DMA into low memory");
            if (const char* d = std::getenv("GCRT_HEAP_DUMP")) {   // SDA offset of the current-heap pointer
                std::uint32_t heap = read32(current_context()->r[13] + static_cast<std::uint32_t>(std::atoi(d)));
                log("current heap %08x:\n", heap);
                for (int i = 0; i < 0x80; i += 16)
                    log("  +%02x: %08x %08x %08x %08x\n", i, read32(heap + i), read32(heap + i + 4), read32(heap + i + 8),
                        read32(heap + i + 12));
            }
        }
        watch_dma("DI read", mar, length);
        ram_will_write(mar, length);
        if (!disc_read(offset, phys(mar), length)) fatal("disc read %llx+%x failed", (unsigned long long)offset, length);
    }

    // Drive timing: games pace loading by it and some reuse buffers on that assumption.
    // A seek when the read does not continue the previous one, then about 3 MB/s.
    std::uint64_t head = 0;
    std::uint64_t read_time(std::uint64_t offset, std::uint32_t size) {
        std::uint64_t ns = 0;
        if (offset != head) {
            std::uint64_t distance = offset > head ? offset - head : head - offset;
            ns += distance > 0x4000000 ? 120000000 : 60000000;
        }
        ns += std::uint64_t(size) * 1000000000ull / 3000000ull;
        head = offset + size;
        return std::max<std::uint64_t>(ns, 50000);
    }

    void execute() {
        std::uint32_t command = cmd[0] >> 24;
        bool error = false;
        std::uint64_t delay = 50000;
        switch (command) {
        case 0xA8: {
            std::uint64_t offset = std::uint64_t(cmd[1]) << 2;
            if (verbosity > 2) log("DI read %08llx+%x -> %08x\n", (unsigned long long)offset, cmd[2], mar);
            delay = fast ? 50000 : read_time(offset, length);
            // The data lands when the transfer completes.
            schedule_in(delay, [this, offset] { dma_disc(offset); });
            break;
        }
        case 0xAB:   // seek
            break;
        case 0x12: { // inquiry: revision, device code, release date
            std::uint8_t info[32] = {0x00, 0x02, 0x00, 0x00, 0x20, 0x02, 0x04, 0x02, 0x61};
            if (phys_ok(mar, 32)) std::memcpy(phys(mar), info, std::min<std::uint32_t>(length, 32));
            break;
        }
        case 0xE0: imm = 0; break;        // request error: none
        case 0xE1:                       // audio stream play/stop
            streaming = !(cmd[0] & 0x00010000);
            stream_offset = cmd[1] << 2;
            stream_length = cmd[2];
            break;
        case 0xE2: imm = 0; break;        // audio status: idle
        case 0xE3: break;                 // stop motor
        case 0xE4: break;                 // audio buffer configuration
        default:
            log("DI: unknown command %08x %08x %08x\n", cmd[0], cmd[1], cmd[2]);
            error = true;
            break;
        }
        schedule_in(delay, [this, error] {
            length = 0;
            cr &= ~1u;
            sr |= error ? 0x04 : 0x10;
            interrupts_changed();
        });
    }
};

DiscInterface di;

} // namespace

bool disc_read(std::uint64_t offset, void* destination, std::size_t size) {
    if (!disc) return false;
    try {
        disc->read(offset, destination, size);
        return true;
    } catch (const starlet::disc::DiscError& e) {
        log("disc read %llx+%zx: %s\n", (unsigned long long)offset, size, e.what());
        return false;
    }
}

bool di_init(const std::string& path, std::string& error) {
    try {
        disc = starlet::disc::open_disc_image(path);
    } catch (const starlet::disc::DiscError& e) {
        error = e.what();
        return false;
    }
    register_device(0x6000, 0x40, &di);
    return true;
}

} // namespace gcrt

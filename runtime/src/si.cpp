// Serial interface: four controller ports with standard controllers, command transfers
// through the 128-byte I/O buffer and periodic polling into the input buffers.
#include "gcrt/devices.hpp"
#include "gcrt/runtime.hpp"

#include <cstring>

namespace gcrt {
namespace {

constexpr std::uint32_t TCINT = 0x80000000u, TCINTMSK = 0x40000000u, COMERR = 0x20000000u, RDSTINT = 0x10000000u,
                        RDSTINTMSK = 0x08000000u;

struct SerialInterface final : Device {
    std::uint32_t outbuf[4] = {}, inbuf_h[4] = {}, inbuf_l[4] = {};
    std::uint32_t poll = 0, comcsr = 0, status = 0, exilk = 0;
    std::uint8_t buffer[128] = {};
    PadSource source;
    EventId poller = 0;

    const char* name() const override { return "SI"; }

    std::uint32_t read32(std::uint32_t o) override {
        if (o >= 0x80 && o < 0x100) {
            const std::uint8_t* b = buffer + (o - 0x80);
            return (std::uint32_t(b[0]) << 24) | (std::uint32_t(b[1]) << 16) | (std::uint32_t(b[2]) << 8) | b[3];
        }
        if (o < 0x30) {
            unsigned chan = o / 12, reg = o % 12;
            if (reg == 0) return outbuf[chan];
            if (reg == 4) {
                status &= ~(0x20000000u >> (8 * chan));   // reading clears RDST
                interrupts_changed();
                return inbuf_h[chan];
            }
            return inbuf_l[chan];
        }
        switch (o) {
        case 0x30: return poll;
        case 0x34: return comcsr | ((status & 0x20202020u) ? RDSTINT : 0);
        case 0x38: return status;
        case 0x3C: return exilk;
        default: return 0;
        }
    }
    std::uint32_t read16(std::uint32_t o) override {
        std::uint32_t w = read32(o & ~3u);
        return (o & 2) ? (w & 0xFFFF) : (w >> 16);
    }
    void write32(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x80 && o < 0x100) {
            std::uint8_t* b = buffer + (o - 0x80);
            b[0] = v >> 24; b[1] = v >> 16; b[2] = v >> 8; b[3] = v;
            return;
        }
        if (o < 0x30) {
            if (o % 12 == 0) outbuf[o / 12] = v;
            return;
        }
        switch (o) {
        case 0x30: poll = v; break;
        case 0x34:
            if (v & TCINT) comcsr &= ~TCINT;
            comcsr = (comcsr & (TCINT | COMERR)) | (v & ~(TCINT | COMERR | RDSTINT | 1u));
            if (v & 1) transfer();
            interrupts_changed();
            break;
        case 0x38:
            // Error bits are cleared by writing ones; WR copies the output buffers out.
            status &= ~(v & 0x0F0F0F0Fu);
            break;
        case 0x3C: exilk = v; break;
        default: break;
        }
    }
    std::uint32_t irq() override {
        std::uint32_t c = read32(0x34);
        if ((c & TCINT) && (c & TCINTMSK)) return PI_SI;
        if ((c & RDSTINT) && (c & RDSTINTMSK)) return PI_SI;
        return 0;
    }

    PadState pad(unsigned chan) { return source ? source(static_cast<int>(chan)) : PadState{}; }

    void transfer() {
        unsigned chan = (comcsr >> 1) & 3;
        unsigned out = (comcsr >> 16) & 0x7F, in = (comcsr >> 8) & 0x7F;
        if (!out) out = 128;
        if (!in) in = 128;
        PadState p = pad(chan);
        comcsr &= ~COMERR;
        if (!p.connected) {
            comcsr |= COMERR;
            status |= 0x08000000u >> (8 * chan);
        } else {
            std::uint8_t reply[128] = {};
            switch (buffer[0]) {
            case 0x00:
            case 0xFF:
                reply[0] = 0x09; reply[1] = 0x00; reply[2] = 0x00;
                break;
            case 0x41:
            case 0x42:
                reply[0] = 0x00; reply[1] = 0x80;
                reply[2] = reply[3] = reply[4] = reply[5] = 0x80;
                break;
            case 0x40:
                fill(p, reply);
                break;
            default:
                break;
            }
            std::memcpy(buffer, reply, in);
        }
        comcsr |= TCINT;
        interrupts_changed();
    }

    static void fill(const PadState& p, std::uint8_t* b) {
        b[0] = static_cast<std::uint8_t>((p.buttons >> 8) & 0x1F);
        b[1] = static_cast<std::uint8_t>(0x80 | (p.buttons & 0x7F));
        b[2] = p.stick_x; b[3] = p.stick_y; b[4] = p.cstick_x; b[5] = p.cstick_y;
        b[6] = p.trigger_l; b[7] = p.trigger_r;
    }

    void poll_pads() {
        poller = schedule_in(8333333, [this] { poll_pads(); });
        for (unsigned chan = 0; chan < 4; ++chan) {
            if (!(poll >> (7 - chan) & 1)) continue;
            PadState p = pad(chan);
            if (!p.connected) {
                inbuf_h[chan] = 0x80000000u;
                inbuf_l[chan] = 0;
                status |= 0x08000000u >> (8 * chan);
            } else {
                std::uint8_t b[8];
                fill(p, b);
                inbuf_h[chan] = (std::uint32_t(b[0]) << 24) | (std::uint32_t(b[1]) << 16) | (std::uint32_t(b[2]) << 8) | b[3];
                inbuf_l[chan] = (std::uint32_t(b[4]) << 24) | (std::uint32_t(b[5]) << 16) | (std::uint32_t(b[6]) << 8) | b[7];
            }
            status |= 0x20000000u >> (8 * chan);
        }
        interrupts_changed();
    }
};

SerialInterface si;

} // namespace

void si_init(PadSource source) {
    si.source = std::move(source);
    register_device(0x6400, 0x100, &si);
    si.poll_pads();
}

} // namespace gcrt

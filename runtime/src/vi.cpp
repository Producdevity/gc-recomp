// Video interface: beam position, display interrupts and the framebuffer registers.
// The beam runs on the host clock: 525 lines per 1/29.97 s (NTSC) or 625 per 1/25 s (PAL);
// each enabled display interrupt fires when the beam reaches its line.
#include "gcrt/devices.hpp"
#include "gcrt/runtime.hpp"

namespace gcrt {
namespace {

struct VideoInterface final : Device {
    std::uint16_t regs[0x80] = {};
    std::uint64_t epoch = 0;
    EventId next = 0;
    std::uint32_t last_line = 0;
    std::uint64_t fields = 0;
    RetraceHook hook;

    const char* name() const override { return "VI"; }
    bool pal() const { return ((regs[1] >> 8) & 3) == 1; }
    std::uint32_t lines() const { return pal() ? 625 : 525; }
    std::uint64_t frame_ns() const { return pal() ? 40000000ull : 33366700ull; }
    std::uint64_t line_ns() const { return frame_ns() / lines(); }

    std::uint32_t current_line(std::uint64_t now) const {
        return static_cast<std::uint32_t>(((now - epoch) % frame_ns()) / line_ns()) + 1;
    }

    std::uint32_t read16(std::uint32_t o) override {
        if (o >= 0x100) return 0;
        switch (o) {
        case 0x2C: note_time_poll(); return current_line(now_ns());
        case 0x2E: {
            note_time_poll();
            std::uint64_t t = (now_ns() - epoch) % line_ns();
            std::uint32_t hlw = regs[3] & 0x1FF;
            return static_cast<std::uint32_t>(t * (hlw ? hlw * 2 : 858) / line_ns()) + 1;
        }
        case 0x6E: return 0;   // no component cable
        default: return regs[o / 2];
        }
    }
    void write16(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x100) return;
        regs[o / 2] = static_cast<std::uint16_t>(v);
        if (o >= 0x30 && o < 0x40) {
            interrupts_changed();
            arm();
        }
    }
    std::uint32_t irq() override {
        for (int n = 0; n < 4; ++n) {
            std::uint16_t hi = regs[0x18 + n * 2];
            if ((hi & 0x8000) && (hi & 0x1000)) return PI_VI;
        }
        return 0;
    }

    // Time of the next display interrupt line after `now`.
    void arm() {
        if (next) cancel(next);
        next = 0;
        std::uint64_t now = now_ns(), best = ~0ull;
        for (int n = 0; n < 4; ++n) {
            std::uint16_t hi = regs[0x18 + n * 2];
            if (!(hi & 0x1000)) continue;
            std::uint32_t vct = hi & 0x3FF;
            if (!vct || vct > lines()) continue;
            std::uint64_t frame_start = now - (now - epoch) % frame_ns();
            std::uint64_t t = frame_start + (vct - 1) * line_ns();
            if (t <= now) t += frame_ns();
            best = std::min(best, t);
        }
        if (best == ~0ull) best = now + frame_ns() / 2;   // keep the field clock running
        next = schedule_at(best, [this] { fire(); });
    }

    void fire() {
        next = 0;
        std::uint32_t line = current_line(now_ns());
        bool any = false;
        for (int n = 0; n < 4; ++n) {
            std::uint16_t& hi = regs[0x18 + n * 2];
            if (!(hi & 0x1000)) continue;
            std::uint32_t vct = hi & 0x3FF;
            // Lines passed since the last check, allowing for the frame wrapping.
            bool passed = last_line <= line ? (vct > last_line && vct <= line) : (vct > last_line || vct <= line);
            if (passed || vct == line) {
                hi |= 0x8000;
                any = true;
            }
        }
        last_line = line;
        if (any) {
            ++fields;
            interrupts_changed();
            if (hook) hook(framebuffer(), fields);
        }
        arm();
    }

    // Physical address of the top field's framebuffer.
    std::uint32_t framebuffer() const {
        std::uint32_t tfbl = (std::uint32_t(regs[0x1C / 2]) << 16) | regs[0x1E / 2];
        std::uint32_t a = tfbl & 0x00FFFFFF;
        if (tfbl & 0x10000000) a <<= 5;
        return a;
    }
};

VideoInterface vi;

} // namespace

void vi_init(RetraceHook hook) {
    vi.epoch = now_ns();
    vi.hook = std::move(hook);
    register_device(0x2000, 0x1000, &vi);
    vi.arm();
}

std::uint32_t vi_framebuffer() { return vi.framebuffer(); }
std::uint32_t vi_framebuffer_width() { return (vi.regs[0x48 / 2] & 0x3FF) * 16; }
bool vi_enabled() { return vi.regs[1] & 1; }

} // namespace gcrt

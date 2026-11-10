// DSP interface: mailboxes, control/status, ARAM DMA, audio DMA, and the audio interface.
// The DSP itself is not emulated: its boot ROM handshake is reproduced and the uploaded
// microcode is identified by a hash of its IRAM image, then served by a host model.
#include "gcrt/devices.hpp"
#include "gcrt/dsp.hpp"
#include "gcrt/gx.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <cstring>
#include <deque>
#include <vector>

namespace gcrt {
namespace {

enum : std::uint16_t {
    CR_RES = 0x1, CR_PIINT = 0x2, CR_HALT = 0x4, CR_AIDINT = 0x8, CR_AIDINTMSK = 0x10, CR_ARINT = 0x20,
    CR_ARINTMSK = 0x40, CR_DSPINT = 0x80, CR_DSPINTMSK = 0x100, CR_DSPDMA = 0x200,
};

struct DspInterface final : Device {
    std::uint16_t regs[0x40] = {};
    std::uint32_t to_dsp = 0;
    bool to_dsp_pending = false;
    // Queued mails and whether each raises the DSP interrupt. Microcode writes the next
    // mail only once the CPU has read the previous one, so the interrupt belongs to the
    // moment a mail reaches the mailbox.
    std::deque<std::pair<std::uint32_t, bool>> from_dsp;
    bool from_dsp_valid = false;
    std::uint32_t from_dsp_current = 0;
    bool booted = false, running = false;
    std::vector<std::uint32_t> boot_mails;
    Microcode* ucode = nullptr;
    EventId ai_event = 0;
    AudioSink sink;
    std::uint32_t ai_cr = 0, ai_vr = 0, ai_scnt = 0, ai_it = 0;

    const char* name() const override { return "DSP"; }

    std::uint32_t read16(std::uint32_t o) override {
        switch (o) {
        case 0x00: return ((to_dsp >> 16) & 0x7FFF) | (to_dsp_pending ? 0x8000 : 0);
        case 0x02: return to_dsp & 0xFFFF;
        case 0x04:
            if (verbosity > 3) log("DSP: poll mail valid %d queued %zu\n", from_dsp_valid, from_dsp.size());
            // A DSP thread's mails wait for a safepoint, which the SDK's poll loops may not
            // reach: an empty mailbox lets due events deliver them.
            if (!from_dsp_valid && from_dsp.empty()) run_due_events();
            if (!from_dsp_valid) next_mail();
            return from_dsp_valid ? ((from_dsp_current >> 16) | 0x8000) : (from_dsp_current >> 16) & 0x7FFF;
        case 0x06: {
            std::uint32_t lo = from_dsp_current & 0xFFFF;
            if (from_dsp_valid) {
                from_dsp_valid = false;
                next_mail();
            }
            return lo;
        }
        case 0x0A: return regs[5] & ~CR_RES;
        case 0x16: return regs[0x16 / 2] | 1;   // ARAM mode: initialised
        case 0x3A: return ai_blocks_left();
        default: return o < 0x80 ? regs[o / 2] : 0;
        }
    }
    void write16(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x80) return;
        if (verbosity > 2 && o != 0x00 && o != 0x02) log("DSP: write %02x = %04x\n", o, v);
        switch (o) {
        case 0x00: to_dsp = (to_dsp & 0xFFFF) | (v << 16); return;
        case 0x02:
            to_dsp = (to_dsp & 0xFFFF0000u) | (v & 0xFFFF);
            receive(to_dsp);
            return;
        case 0x0A: control(static_cast<std::uint16_t>(v)); return;
        case 0x2A:
            regs[o / 2] = static_cast<std::uint16_t>(v);
            aram_dma();
            return;
        case 0x36: {
            bool was = regs[0x36 / 2] & 0x8000;
            regs[o / 2] = static_cast<std::uint16_t>(v);
            if (!was && (v & 0x8000)) start_ai();
            if (!(v & 0x8000)) stop_ai();
            return;
        }
        default: regs[o / 2] = static_cast<std::uint16_t>(v); return;
        }
    }
    std::uint32_t irq() override {
        std::uint16_t cr = regs[5];
        if ((cr & CR_AIDINT) && (cr & CR_AIDINTMSK)) return PI_DSP;
        if ((cr & CR_ARINT) && (cr & CR_ARINTMSK)) return PI_DSP;
        if ((cr & CR_DSPINT) && (cr & CR_DSPINTMSK)) return PI_DSP;
        return 0;
    }

    void control(std::uint16_t v) {
        std::uint16_t cr = regs[5];
        std::uint16_t clear = v & (CR_AIDINT | CR_ARINT | CR_DSPINT);
        cr = static_cast<std::uint16_t>((cr & (CR_AIDINT | CR_ARINT | CR_DSPINT | CR_DSPDMA)) & ~clear);
        cr |= v & ~(CR_AIDINT | CR_ARINT | CR_DSPINT | CR_DSPDMA | CR_RES);
        bool halted_before = regs[5] & CR_HALT;
        regs[5] = cr;
        if (v & CR_RES) reset();
        if (halted_before && !(cr & CR_HALT)) start();
        if (!booted && !(cr & CR_HALT) && !running) start();
        if ((v & CR_PIINT) && ucode) ucode->cpu_interrupt();
        interrupts_changed();
    }

    void reset() {
        running = false;
        booted = false;
        boot_mails.clear();
        from_dsp.clear();
        from_dsp_valid = false;
        to_dsp_pending = false;
        ucode = nullptr;
    }

    // The boot ROM greets the CPU and waits for the task upload parameters.
    void start() {
        if (verbosity > 2) log("DSP: start (running %d)\n", running);
        if (running) return;
        running = true;
        booted = false;
        boot_mails.clear();
        send(0x8071FEED, false);
    }

    void send(std::uint32_t mail, bool interrupt) {
        from_dsp.emplace_back(mail, interrupt);
        next_mail();
    }
    void next_mail() {
        if (from_dsp_valid || from_dsp.empty()) return;
        auto [mail, interrupt] = from_dsp.front();
        from_dsp.pop_front();
        from_dsp_current = mail;
        from_dsp_valid = true;
        if (interrupt) {
            regs[5] |= CR_DSPINT;
            interrupts_changed();
        }
    }

    void receive(std::uint32_t mail) {
        to_dsp_pending = false;
        if (!booted) {
            boot_mails.push_back(mail);
            // 0x80F3A001 iram_mmem, 0x80F3C002 iram_addr, 0x80F3A002 iram_length,
            // 0x80F3B002 dram, 0x80F3D001 init vector: ten mails in all.
            if (boot_mails.size() == 10) boot();
            return;
        }
        if (ucode) ucode->mail(mail);
        else if (verbosity > 1) log("DSP: mail %08x to unknown microcode\n", mail);
    }

    void boot() {
        // Boot ROM parameters as (command, value) pairs, in whatever order the game sends them.
        std::uint32_t iram_mmem = 0, iram_length = 0, init_vector = 0;
        for (std::size_t i = 0; i + 1 < boot_mails.size(); i += 2) {
            switch (boot_mails[i]) {
            case 0x80F3A001: iram_mmem = boot_mails[i + 1]; break;
            case 0x80F3A002: iram_length = boot_mails[i + 1]; break;
            case 0x80F3D001: init_vector = boot_mails[i + 1]; break;
            default: break;
            }
        }
        iram_mmem &= 0x3FFFFFFFu;
        booted = true;
        std::uint32_t hash = 0;
        if (phys_ok(iram_mmem, iram_length)) hash = ucode_hash(phys(iram_mmem), iram_length);
        ucode = create_microcode(hash, *this_api());
        log("DSP: microcode %08x (%u bytes at %08x, entry %04x): %s\n", hash, iram_length, iram_mmem, init_vector,
            ucode ? ucode->name() : "no model");
        if (ucode) ucode->start();
    }

    DspApi* this_api();

    void aram_dma() {
        std::uint32_t mm = ((std::uint32_t(regs[0x20 / 2]) << 16) | regs[0x22 / 2]) & 0x03FFFFE0u;
        std::uint32_t ar = ((std::uint32_t(regs[0x24 / 2]) << 16) | regs[0x26 / 2]) & 0x03FFFFE0u;
        std::uint32_t cnt = (std::uint32_t(regs[0x28 / 2]) << 16) | regs[0x2A / 2];
        bool to_main = cnt & 0x80000000u;
        std::uint32_t length = cnt & 0x03FFFFE0u;
        if (!phys_ok(mm, length)) fatal("ARAM DMA outside RAM: %08x+%x", mm, length);
        if (to_main) {
            watch_dma("ARAM to main", mm, length);
            gx_flush();
        }
        if (verbosity > 3) log("ARAM DMA %s main %08x aram %08x len %x mode %04x\n", to_main ? "->main" : "->aram", mm, ar, length, regs[0x12 / 2]);
        if (to_main && mm < 0x3100) log("ARAM DMA into low memory: %08x+%x from %08x\n", mm, length, ar);
        // 16 MiB of internal ARAM and no expansion. The size register's mode (2 MiB << mode)
        // sets how the controller decodes addresses for the internal chip; past that is the
        // expansion bus, where writes are lost and reads return zero. In the 32 MiB mode a
        // 16 MiB chip ignores address bit 22: the older SDK's size probe looks for exactly
        // that alias; the newer one only tests for expansion memory above 16 MiB.
        std::uint32_t mode = regs[0x12 / 2] & 7;
        std::uint32_t decoded = 0x200000u << std::min<std::uint32_t>(mode, 4);
        std::uint8_t* a = aram();
        for (std::uint32_t done = 0; done < length; ++done) {
            std::uint32_t at = ar + done;
            bool present = at < std::max(decoded, 0x200000u);
            if (mode >= 4) at = (at & 0x3FFFFFu) | ((at >> 1) & 0xC00000u);
            else present = present && at < kAramSize;
            at &= kAramSize - 1;
            if (to_main) phys(mm)[done] = present ? a[at] : 0;
            else if (present) a[at] = phys(mm)[done];
        }
        regs[0x2A / 2] = 0;
        regs[0x28 / 2] &= 0x8000;
        regs[5] |= CR_ARINT;
        interrupts_changed();
    }

    // ---- Audio DMA ----
    std::uint64_t ai_started = 0, ai_period = 0, ai_due = 0;
    unsigned rate() const { return (ai_cr & 0x40) ? 32000 : 48000; }
    std::uint32_t ai_blocks() const { return regs[0x36 / 2] & 0x7FFF; }
    std::uint32_t ai_blocks_left() const {
        if (!(regs[0x36 / 2] & 0x8000) || !ai_period) return 0;
        std::uint64_t elapsed = now_ns() - ai_started;
        std::uint64_t left = elapsed >= ai_period ? 0 : (ai_period - elapsed) * ai_blocks() / ai_period;
        return static_cast<std::uint32_t>(left);
    }
    void start_ai() {
        stop_ai();
        ai_due = now_ns();
        play_block();
    }
    void stop_ai() {
        if (ai_event) cancel(ai_event);
        ai_event = 0;
        ai_period = 0;
    }
    void play_block() {
        std::uint32_t address = ((std::uint32_t(regs[0x30 / 2]) << 16) | regs[0x32 / 2]) & 0x03FFFFE0u;
        std::uint32_t bytes = ai_blocks() * 32;
        if (verbosity > 2) log("AI DMA %08x+%x at %u Hz t=%.3f\n", address, bytes, rate(), now_ns() / 1e9);
        std::uint32_t frames = bytes / 4;
        if (!frames) return;
        if (sink && phys_ok(address, bytes)) {
            std::vector<std::int16_t> samples(frames * 2);
            const std::uint8_t* p = phys(address);
            for (std::uint32_t i = 0; i < frames * 2; ++i)
                samples[i] = static_cast<std::int16_t>((p[i * 2] << 8) | p[i * 2 + 1]);
            sink(samples.data(), frames, rate());
        }
        // Blocks follow each other at the sample rate: a late event shortens the next block's
        // wait instead of delaying every later block. A block or more behind restarts the clock.
        const std::uint64_t now = now_ns();
        if (ai_due + ai_period <= now) ai_due = now;
        ai_started = ai_due;
        ai_period = std::uint64_t(frames) * 1000000000ull / rate();
        ai_due += ai_period;
        ai_event = schedule_at(ai_due, [this] {
            ai_event = 0;
            regs[5] |= CR_AIDINT;
            interrupts_changed();
            if (regs[0x36 / 2] & 0x8000) play_block();
        });
    }
};

DspInterface dsp;

struct Api final : DspApi {
    void send_mail(std::uint32_t mail, bool interrupt) override { dsp.send(mail, interrupt); }
    void raise_interrupt() override {
        dsp.regs[5] |= CR_DSPINT;
        interrupts_changed();
    }
    std::uint8_t* main_memory(std::uint32_t address, std::uint32_t size) override {
        return phys_ok(address, size) ? phys(address) : nullptr;
    }
    std::uint8_t* aram_memory() override { return aram(); }
    unsigned output_rate() override { return dsp.rate(); }
};
Api api;
DspInterface* self = &dsp;

DspApi* DspInterface::this_api() { return &api; }

// Audio interface registers (0xCC006C00). The streaming sample counter runs at the
// streaming rate (AFR: 48 or 32 kHz) while PSTAT is set.
struct AudioInterface final : Device {
    std::uint32_t scnt_base = 0;
    std::uint64_t scnt_since = 0;
    const char* name() const override { return "AI"; }
    unsigned stream_rate() const { return (dsp.ai_cr & 0x2) ? 48000 : 32000; }
    std::uint32_t counter() const {
        if (!(dsp.ai_cr & 0x1)) return scnt_base;
        return scnt_base + static_cast<std::uint32_t>((now_ns() - scnt_since) * stream_rate() / 1000000000ull);
    }
    std::uint32_t read32(std::uint32_t o) override {
        switch (o) {
        case 0x00: return dsp.ai_cr;
        case 0x04: return dsp.ai_vr;
        case 0x08: note_time_poll(); return counter();
        case 0x0C: return dsp.ai_it;
        default: return 0;
        }
    }
    std::uint32_t read16(std::uint32_t o) override {
        std::uint32_t w = read32(o & ~3u);
        return (o & 2) ? (w & 0xFFFF) : (w >> 16);
    }
    void write32(std::uint32_t o, std::uint32_t v) override {
        switch (o) {
        case 0x00: {
            std::uint32_t current = counter();
            bool was_playing = dsp.ai_cr & 0x1;
            std::uint32_t status = (dsp.ai_cr & 0x8) & ~(v & 0x8);   // AIINT: write one to clear
            dsp.ai_cr = status | (v & ~0x28u);                         // SCRESET is a strobe
            scnt_base = (v & 0x20) ? 0 : current;
            if ((v & 0x1) && (!was_playing || (v & 0x20))) scnt_since = now_ns();
            else if (v & 0x1) scnt_since = now_ns();
            interrupts_changed();
            break;
        }
        case 0x04: dsp.ai_vr = v; break;
        case 0x08: break;
        case 0x0C: dsp.ai_it = v; break;
        default: break;
        }
    }
    std::uint32_t irq() override { return ((dsp.ai_cr & 0x8) && (dsp.ai_cr & 0x4)) ? PI_AI : 0; }
};
AudioInterface ai;

} // namespace

std::uint32_t ucode_hash(const std::uint8_t* data, std::uint32_t size) {
    std::uint32_t h = 2166136261u;
    for (std::uint32_t i = 0; i < size; ++i) {
        h ^= data[i];
        h *= 16777619u;
    }
    return h;
}

void dsp_init(AudioSink sink) {
    (void)self;
    dsp.sink = std::move(sink);
    dsp.regs[5] = CR_HALT;
    register_device(0x5000, 0x200, &dsp);
    register_device(0x6C00, 0x20, &ai);
}

} // namespace gcrt

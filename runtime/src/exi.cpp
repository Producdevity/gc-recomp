// External interface: three channels, immediate and DMA transfers, and the devices behind
// them: the IPL chip (RTC, SRAM, UART, ROM) on channel 0 device 1 and memory cards on
// channels 0 and 1, device 0.
#include "gcrt/devices.hpp"
#include "gcrt/gx.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <starlet/card/card.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gcrt {
namespace {

struct ExiDevice {
    virtual ~ExiDevice() = default;
    virtual void select() {}
    virtual void deselect() {}
    virtual void write(const std::uint8_t*, std::size_t) {}
    virtual void read(std::uint8_t* out, std::size_t n) { std::memset(out, 0, n); }
    virtual bool present() const { return true; }
};

// The Macronix chip behind channel 0, device 1.
struct IplChip final : ExiDevice {
    std::uint8_t sram[64] = {};
    std::uint32_t command = 0, position = 0;
    unsigned header = 0;
    std::string uart;

    IplChip() {
        sram[0x12] = 0;      // language: English
        sram[0x13] = 0x04;   // flags: stereo
        checksum();
    }
    void checksum() {
        std::uint16_t sum = 0, inv = 0;
        for (int i = 0x0C; i < 0x14; i += 2) {
            std::uint16_t w = static_cast<std::uint16_t>((sram[i] << 8) | sram[i + 1]);
            sum = static_cast<std::uint16_t>(sum + w);
            inv = static_cast<std::uint16_t>(inv + static_cast<std::uint16_t>(~w));
        }
        sram[0] = sum >> 8; sram[1] = sum & 0xFF;
        sram[2] = inv >> 8; sram[3] = inv & 0xFF;
    }
    void select() override { header = 0; command = 0; position = 0; }
    void deselect() override {
        if (!uart.empty()) {
            log("%s", uart.c_str());
            uart.clear();
        }
    }
    std::uint32_t rtc() const {
        // Seconds since 2000-01-01 00:00:00, local time as the console keeps it.
        std::time_t now = std::time(nullptr);
        std::tm local{};
        localtime_r(&now, &local);
        std::time_t local_seconds = now + local.tm_gmtoff;
        return static_cast<std::uint32_t>(local_seconds - 946684800);
    }
    void write(const std::uint8_t* b, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i) {
            if (header < 4) {
                command = (command << 8) | b[i];
                ++header;
                continue;
            }
            if ((command & 0x7FFFFF00u) == 0x20010000u) {   // UART
                if (b[i]) uart.push_back(static_cast<char>(b[i]));
                if (b[i] == '\n') { log("%s", uart.c_str()); uart.clear(); }
            } else if ((command & 0x7FFFFF00u) == 0x20000100u) {
                std::uint32_t at = (command & 0xFF) + position++;
                if (at < 64) sram[at] = b[i];
            }
        }
    }
    void read(std::uint8_t* out, std::size_t n) override {
        std::memset(out, 0, n);
        if ((command & 0x7FFFFFFFu) == 0x20000000u) {
            std::uint32_t t = rtc();
            for (std::size_t i = 0; i < n && position < 4; ++i, ++position) out[i] = static_cast<std::uint8_t>(t >> (24 - 8 * position));
        } else if ((command & 0x7FFFFF00u) == 0x20000100u) {
            for (std::size_t i = 0; i < n; ++i) {
                std::uint32_t at = (command & 0xFF) + position++;
                out[i] = at < 64 ? sram[at] : 0;
            }
        } else if (!(command & 0x20000000u)) {
            // IPL ROM: no image is shipped; reads return zeros.
            position += static_cast<std::uint32_t>(n);
        }
    }
};

// A flash memory card holding the image of a Starlet card folder (Dolphin GCI files). Commands
// as the SDK's CARD library sends them: 0x00 ID, 0x83 status, 0x89 clear status, 0x81
// interrupt enable, 0x52 read, 0xF2 page program, 0xF1 sector erase, 0xF4 card erase. The
// card reports itself unlocked, so the SDK skips its unlock handshake; program and erase
// completions raise the channel's EXI interrupt, which the SDK waits for. Writes reach the
// folder 300 ms after the last change and at exit. A writer thread stores a copy of the image:
// syncing the folder on an SD card can take seconds, and the CARD library fails a request
// that waits that long (the game then reports a damaged card).
struct MemoryCard final : ExiDevice {
    std::unique_ptr<starlet::card::Folder> folder;
    std::vector<std::uint8_t> image;
    std::mutex flushing;                   // guards queued_image, queued and storing
    std::condition_variable wake, idle;
    std::vector<std::uint8_t> queued_image;
    bool queued = false, storing = false, writer_started = false;
    std::uint32_t id = 0;
    std::uint8_t command[5] = {};
    unsigned header = 0, position = 0, programmed = 0;
    std::uint32_t address = 0;
    bool interrupts = false;
    std::atomic<bool> flush_pending{false};
    std::function<void()> raise_interrupt;

    static unsigned command_length(std::uint8_t c) {
        switch (c) {
        case 0x52: case 0xF2: return 5;
        case 0xF1: case 0xF4: return 3;
        case 0x00: case 0x81: case 0x83: return 2;
        default: return 1;
        }
    }
    bool load(const std::string& saves, starlet::card::Region region, unsigned slot, const starlet::card::Console& console) {
        namespace sc = starlet::card;
        folder = std::make_unique<sc::Folder>(sc::folder_for(saves, region, int(slot)), int(slot), console, 16,
                                              [](const std::string& line) { log("memory card: %s\n", line.c_str()); });
        if (!folder->ok()) return false;
        const std::string raw = saves + "/memcard-a.raw";
        if (slot == 0) {
            if (FILE* f = std::fopen(raw.c_str(), "rb")) {
                std::vector<std::uint8_t> old;
                std::fseek(f, 0, SEEK_END);
                const long size = std::ftell(f);
                std::fseek(f, 0, SEEK_SET);
                if (size > 0) {
                    old.resize(static_cast<std::size_t>(size));
                    if (std::fread(old.data(), 1, old.size(), f) != old.size()) old.clear();
                }
                std::fclose(f);
                const auto result = folder->import_image(old.data(), old.size());
                if (result == sc::ready && std::rename(raw.c_str(), (raw + ".imported").c_str()) == 0)
                    log("memory card: imported %s into %s\n", raw.c_str(), folder->path().c_str());
                else
                    log("memory card: %s not imported (result %d)\n", raw.c_str(), int(result));
            }
        }
        image = folder->image();
        // The ID's size field counts megabits.
        id = static_cast<std::uint32_t>(image.size() * 8 / (1024 * 1024));
        return true;
    }
    // An image the game is still changing does not verify, leaves the folder unchanged and is
    // stored after the next change.
    void store(const std::vector<std::uint8_t>& snapshot, bool final) {
        const std::uint64_t start = now_ns();
        const auto result = folder->store_image(snapshot.data(), snapshot.size());
        if (result != starlet::card::ready && (final || verbosity > 0))
            log("memory card: image not stored (result %d)\n", int(result));
        else if (verbosity > 0)
            log("memory card: stored in %.1f ms\n", (now_ns() - start) / 1e6);
    }
    // Guest thread: hands the current image to the writer.
    void flush() {
        flush_pending = false;
        std::lock_guard<std::mutex> lock(flushing);
        queued_image = image;
        queued = true;
        if (!writer_started) {
            writer_started = true;
            std::thread([this] {
                std::unique_lock<std::mutex> held(flushing);
                for (;;) {
                    wake.wait(held, [this] { return queued; });
                    std::vector<std::uint8_t> snapshot = std::move(queued_image);
                    queued = false;
                    storing = true;
                    held.unlock();
                    store(snapshot, false);
                    held.lock();
                    storing = false;
                    idle.notify_all();
                }
            }).detach();
        }
        wake.notify_one();
    }
    // Exiting thread: waits for the writer, then stores a change it has not been handed.
    void finish() {
        std::unique_lock<std::mutex> lock(flushing);
        idle.wait(lock, [this] { return !queued && !storing; });
        if (flush_pending.exchange(false)) store(image, true);
    }
    void changed(std::uint64_t busy_ns) {
        if (!flush_pending) {
            flush_pending = true;
            schedule_in(300000000ull, [this] { flush(); });
        }
        if (interrupts && raise_interrupt) schedule_in(busy_ns, raise_interrupt);
    }
    void select() override { header = 0; position = 0; programmed = 0; }
    void deselect() override {
        if (header < command_length(command[0])) return;
        const std::size_t sector = 8 * 1024;
        switch (command[0]) {
        case 0xF2:
            if (programmed) changed(1000000ull);
            break;
        case 0xF1: {
            std::size_t base = (std::size_t(command[1] & 0x7F) << 17 | std::size_t(command[2]) << 9) & ~(sector - 1);
            if (base < image.size()) std::memset(image.data() + base, 0xFF, sector);
            changed(10000000ull);
            break;
        }
        case 0xF4:
            std::memset(image.data(), 0xFF, image.size());
            changed(10000000ull);
            break;
        default: break;
        }
    }
    void write(const std::uint8_t* b, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i) {
            if (header < command_length(command[0]) || header == 0) {
                command[header++] = b[i];
                if (header == command_length(command[0])) {
                    address = std::uint32_t(command[1] & 0x7F) << 17 | std::uint32_t(command[2]) << 9 |
                              std::uint32_t(command[3] & 3) << 7 | (command[4] & 0x7F);
                    if (command[0] == 0x81) interrupts = command[1] & 1;
                }
                continue;
            }
            if (command[0] == 0xF2 && programmed < 128) {
                // A page program wraps within its 128-byte page.
                std::uint32_t at = (address & ~0x7Fu) | ((address + programmed++) & 0x7F);
                if (at < image.size()) image[at] = b[i];
            }
        }
    }
    void read(std::uint8_t* out, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i, ++position) {
            switch (command[0]) {
            case 0x00: out[i] = position < 4 ? static_cast<std::uint8_t>(id >> (24 - 8 * position)) : 0; break;
            case 0x83: out[i] = 0x40; break;   // unlocked, no program or erase error
            case 0x52: out[i] = address + position < image.size() ? image[address + position] : 0xFF; break;
            default: out[i] = 0xFF; break;
            }
        }
    }
};

struct Absent final : ExiDevice {
    bool present() const override { return false; }
    void read(std::uint8_t* out, std::size_t n) override { std::memset(out, 0xFF, n); }
};

struct Channel {
    std::uint32_t csr = 0, mar = 0, length = 0, cr = 0, data = 0;
    ExiDevice* devices[3] = {};
    ExiDevice* selected = nullptr;
};

struct ExternalInterface final : Device {
    Channel ch[3];
    IplChip ipl;
    Absent absent;
    MemoryCard cards[2];

    const char* name() const override { return "EXI"; }
    std::uint32_t read32(std::uint32_t o) override {
        if (o >= 0x3C) return 0;
        Channel& c = ch[o / 0x14];
        switch (o % 0x14) {
        case 0x00: {
            std::uint32_t v = c.csr;
            ExiDevice* d0 = c.devices[0];
            if (d0 && d0->present() && &c != &ch[2]) v |= 0x1000;   // EXT: device attached
            return v;
        }
        case 0x04: return c.mar;
        case 0x08: return c.length;
        case 0x0C: return c.cr;
        default: return c.data;
        }
    }
    std::uint32_t read16(std::uint32_t o) override {
        std::uint32_t w = read32(o & ~3u);
        return (o & 2) ? (w & 0xFFFF) : (w >> 16);
    }
    void write32(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x3C) return;
        unsigned n = o / 0x14;
        Channel& c = ch[n];
        switch (o % 0x14) {
        case 0x00: {
            std::uint32_t keep = c.csr & 0x80Au;          // interrupt status bits
            keep &= ~(v & 0x80Au);                        // write one to clear
            c.csr = keep | (v & ~0x80Au & 0x3FF5u);
            ExiDevice* next = nullptr;
            for (int d = 0; d < 3; ++d)
                if (v & (0x80u << d)) next = c.devices[d] ? c.devices[d] : &absent;
            if (next != c.selected) {
                if (c.selected) c.selected->deselect();
                c.selected = next;
                if (next) next->select();
            }
            interrupts_changed();
            break;
        }
        case 0x04: c.mar = v & 0x03FFFFE0u; break;
        case 0x08: c.length = v & 0x03FFFFE0u; break;
        case 0x0C:
            c.cr = v & 0x3F;
            if (v & 1) transfer(c);
            break;
        default: c.data = v; break;
        }
    }
    std::uint32_t irq() override {
        for (auto& c : ch) {
            if ((c.csr & 0x2) && (c.csr & 0x1)) return PI_EXI;
            if ((c.csr & 0x8) && (c.csr & 0x4)) return PI_EXI;
            if ((c.csr & 0x800) && (c.csr & 0x400)) return PI_EXI;
        }
        return 0;
    }
    void transfer(Channel& c) {
        ExiDevice* d = c.selected ? c.selected : &absent;
        unsigned rw = (c.cr >> 2) & 3;
        if (c.cr & 2) {
            if (!phys_ok(c.mar, c.length)) fatal("EXI DMA outside RAM: %08x+%x", c.mar, c.length);
            if (rw == 1) d->write(phys(c.mar), c.length);
            else {
                watch_dma("EXI read", c.mar, c.length);
                gx_flush();
                d->read(phys(c.mar), c.length);
            }
        } else {
            unsigned len = ((c.cr >> 4) & 3) + 1;
            std::uint8_t b[4] = {static_cast<std::uint8_t>(c.data >> 24), static_cast<std::uint8_t>(c.data >> 16),
                                 static_cast<std::uint8_t>(c.data >> 8), static_cast<std::uint8_t>(c.data)};
            if (rw == 1) {
                d->write(b, len);
            } else {
                std::uint8_t in[4] = {};
                if (rw == 2) d->write(b, len);
                d->read(in, len);
                c.data = (std::uint32_t(in[0]) << 24) | (std::uint32_t(in[1]) << 16) | (std::uint32_t(in[2]) << 8) | in[3];
            }
        }
        c.cr &= ~1u;
        c.csr |= 0x8;   // TCINT
        interrupts_changed();
    }
};

ExternalInterface exi;

// The SDK mounts an unlocked card only if SRAM holds a flash ID whose check byte is the
// complement of its sum, and accepts a formatted card only if its serial was derived from
// that flash ID (CARDFormat, VerifyID): the card folder derives the serial from the console's.
void set_flash_id(unsigned slot, const std::array<std::uint8_t, 12>& id) {
    std::memcpy(exi.ipl.sram + 0x14 + 12 * slot, id.data(), 12);
    exi.ipl.sram[0x3A + slot] = starlet::card::flash_id_check(id);
}

} // namespace

void exi_init(const std::string& saves, char game_region) {
    namespace sc = starlet::card;
    sc::Console console;
    const sc::Region region = sc::region_of(game_region);
    console.encoding = region == sc::Region::japan ? sc::Encoding::shift_jis : sc::Encoding::ansi;
    for (unsigned slot = 0; slot < 2; ++slot) {
        ExiDevice* device = &exi.absent;
        if (slot == 0 && !saves.empty() && exi.cards[slot].load(saves, region, slot, console)) {
            device = &exi.cards[slot];
            exi.cards[slot].raise_interrupt = [slot] {
                exi.ch[slot].csr |= 0x2;
                interrupts_changed();
            };
        }
        exi.ch[slot].devices[0] = device;
        set_flash_id(slot, console.flash_id[slot]);
    }
    exi.ch[0].devices[1] = &exi.ipl;
    register_device(0x6800, 0x40, &exi);
}

void memory_cards_flush() {
    for (auto& card : exi.cards)
        if (card.folder) card.finish();
}

} // namespace gcrt

// Boot: what the IPL and the disc's apploader leave behind before the DOL's entry point.
// Values follow YAGCD's low-memory map and the Dolphin SDK's OSBootInfo/BI2 layout.
#include "gcrt/gcrt.hpp"
#include "gcrt/io.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <cstdlib>
#include <cstring>
#include <vector>

namespace gcrt {
namespace {

std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3];
}

std::uint64_t fnv1a(const std::uint8_t* p, std::size_t n) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 0x100000001b3ull;
    }
    return h;
}

bool load_dol(std::uint32_t dol_offset, const Program& program, std::uint32_t& entry, std::string& error) {
    std::uint8_t header[0x100];
    if (!disc_read(dol_offset, header, sizeof header)) {
        error = "cannot read the DOL header";
        return false;
    }
    std::uint32_t size = 0;
    for (int i = 0; i < 18; ++i) {
        std::uint32_t o = be32(header + i * 4), s = be32(header + 0x90 + i * 4);
        if (s) size = std::max(size, o + s);
    }
    std::vector<std::uint8_t> dol(size);
    if (!disc_read(dol_offset, dol.data(), size)) {
        error = "cannot read the DOL";
        return false;
    }
    if (fnv1a(dol.data(), dol.size()) != program.dol_hash) {
        error = "this disc's executable is not the one the port was built from";
        return false;
    }
    std::uint32_t bss = be32(header + 0xD8), bss_size = be32(header + 0xDC);
    std::memset(host(bss), 0, bss_size);
    for (int i = 0; i < 18; ++i) {
        std::uint32_t o = be32(header + i * 4), a = be32(header + 0x48 + i * 4), s = be32(header + 0x90 + i * 4);
        if (!s) continue;
        std::memcpy(host(a), dol.data() + o, s);
    }
    entry = be32(header + 0xE0);
    return true;
}

} // namespace

bool boot(const BootOptions& options, std::string& error) {
    install_fault_handler();
    if (std::getenv("GCRT_VIRTUAL_TIME")) virtual_time = true;
    if (const char* w = std::getenv("GCRT_WATCH")) {   // HEX-ADDR:HEX-SIZE, physical
        gc_watch_lo = static_cast<std::uint32_t>(std::strtoul(w, nullptr, 16)) & 0x01FFFFFF;
        const char* colon = std::strchr(w, ':');
        gc_watch_hi = gc_watch_lo + (colon ? static_cast<std::uint32_t>(std::strtoul(colon + 1, nullptr, 16)) : 4);
    }
    if (!memory_init()) {
        error = "cannot reserve the guest address space";
        return false;
    }
    ram_tracking_enable();
    if (!options.program.functions) {
        error = "no game code";
        return false;
    }
    cpu_init(options.program);
    io_init();
    if (!di_init(options.disc, error)) return false;

    std::uint8_t disc[0x440];
    if (!disc_read(0, disc, sizeof disc)) {
        error = "cannot read the disc header";
        return false;
    }
    std::uint32_t dol_offset = be32(disc + 0x420), fst_offset = be32(disc + 0x424);
    std::uint32_t fst_size = be32(disc + 0x428), fst_max = be32(disc + 0x42C);
    std::uint32_t entry = 0;
    if (!load_dol(dol_offset, options.program, entry, error)) return false;

    // FST at the top of memory, BI2 below it; the arena ends there.
    std::uint32_t fst = (0x81800000u - fst_max) & ~0x1Fu;
    std::uint32_t bi2 = fst - 0x2000;
    if (!disc_read(fst_offset, host(fst), fst_size) || !disc_read(0x440, host(bi2), 0x2000)) {
        error = "cannot read the file system table";
        return false;
    }
    std::memcpy(host(0x80000000u), disc, 0x20);
    write32(0x80000020, 0x0D15EA5E);     // boot magic
    write32(0x80000024, 1);
    write32(0x80000028, kRamSize);
    write32(0x8000002C, 1);              // retail console
    write32(0x80000030, 0);              // arena low: from the linker
    write32(0x80000034, bi2);            // arena high
    write32(0x80000038, fst);
    write32(0x8000003C, fst_max);
    write32(0x800000CC, 0);              // NTSC
    write32(0x800000D0, kAramSize);
    write32(0x800000F0, kRamSize);
    write32(0x800000F4, bi2);
    write32(0x800000F8, 162000000);      // bus clock
    write32(0x800000FC, 486000000);      // core clock

    vi_init(options.retrace);
    si_init(options.pads);
    exi_init(options.saves, static_cast<char>(disc[3]));
    dsp_init(options.audio);
    gx_init(options.gx, options.gx_thread);
    events_start();

    gc_ctx* c = current_context();
    set_attention_context(c);
    c->r[1] = bi2 - 0x100;
    gc_msr = 0x00003032u;
    log("gcrt: booting %.6s, entry %08x, %u functions\n", reinterpret_cast<const char*>(disc), entry, options.program.count);
    gc_call(c, entry);
    error = "the game's entry point returned";
    return false;
}

} // namespace gcrt

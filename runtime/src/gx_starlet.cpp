#include "gcrt/gx_starlet.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <map>
#include <memory>

namespace gcrt {
namespace {

namespace sg = starlet::gx;

GxStats stats;

struct Xfb {
    int width = 0, height = 0;
    std::size_t stride = 0;
};
std::map<std::uint32_t, Xfb> xfbs;

// GX addresses are physical; main memory already holds canonical big-endian bytes.
struct RamReader final : sg::MemoryReader {
    bool copy(std::uint32_t address, std::uint8_t* destination, std::size_t size) override {
        if (!phys_ok(address, static_cast<std::uint32_t>(size))) return false;
        std::memcpy(destination, phys(address), size);
        return true;
    }
    sg::ByteView view(std::uint32_t address, std::size_t size) override {
        if (!phys_ok(address, static_cast<std::uint32_t>(size))) return {};
        return {phys(address), size};
    }
    // Vertex and matrix arrays: canonical bytes from the array base to the end of RAM.
    sg::MemoryArrayView array_view(std::uint32_t address) override {
        const std::uint32_t physical = address & 0x3FFFFFFFu;
        if (physical >= kRamSize) return {};
        return {{phys(address), kRamSize - physical}, 0, false, 0};
    }
    // Page write generations (ram_range_generation) for array ranges and for display lists
    // the decoder reads in place from RAM.
    std::uint64_t range_generation(const sg::MemoryArrayView&, std::uint32_t address, std::size_t size) override {
        return ram_range_generation(address & 0x3FFFFFFFu, static_cast<std::uint32_t>(size));
    }
    SourceIdentity source_identity(const void* bytes, std::size_t size) override {
        const auto at = static_cast<const std::uint8_t*>(bytes);
        if (at < phys(0) || at >= phys(0) + kRamSize) return {bytes, 0};
        return {bytes, ram_range_generation(static_cast<std::uint32_t>(at - phys(0)), static_cast<std::uint32_t>(size))};
    }
};

struct Sink final : sg::CommandSink {
    GxRenderer* renderer = nullptr;
    bool skip_draws = false;
    sg::Decoder* decoder = nullptr;

    std::map<std::uint32_t, unsigned> copy_targets;
    bool draw(const sg::State& state, const sg::Draw& d, sg::MemoryReader& memory) override {
        ++stats.draws;
        static const bool log_copies = std::getenv("GCRT_GX_LOGCOPIES") != nullptr;
        if (log_copies && !copy_targets.empty()) {
            static std::map<std::string, unsigned> seen;
            for (unsigned map = 0; map < 8; ++map) {
                const unsigned base = (map < 4 ? 0x80 : 0xa0) + map % 4;
                const std::uint32_t address = (state.bp(static_cast<std::uint8_t>(base + 20)) & 0x00FFFFFFu) << 5;
                if (!copy_targets.count(address)) continue;
                char key[160];
                std::snprintf(key, sizeof key, "%08x image0 %06x mode0 %06x tlut %06x tev stages %u ind %06x", address,
                              state.bp(static_cast<std::uint8_t>(base + 8)), state.bp(static_cast<std::uint8_t>(base)),
                              state.bp(static_cast<std::uint8_t>(base + 24)), (state.bp(0) >> 10 & 15) + 1, state.bp(0x27));
                if (!seen[key]++) {
                    log("gx: map %u samples copy %s\n", map, key);
                    std::string regs;
                    for (unsigned r : {0x00u, 0x28u, 0x29u, 0x2au, 0xc0u, 0xc1u, 0xc2u, 0xc3u, 0xc4u, 0xc5u, 0xc6u, 0xc7u, 0xc8u, 0xc9u,
                                       0xe0u, 0xe1u, 0xe2u, 0xe3u, 0xe4u, 0xe5u, 0xe6u, 0xe7u, 0xf3u, 0xf6u, 0xf7u, 0xf8u, 0xf9u,
                                       0xfau, 0x41u, 0x40u, 0xe8u, 0xf1u, 0x10u, 0x11u, 0x12u, 0x13u}) {
                        char b[20];
                        std::snprintf(b, sizeof b, " %02x=%06x", r, state.bp(static_cast<std::uint8_t>(r)));
                        regs += b;
                    }
                    log("  bp%s\n", regs.c_str());
                    for (unsigned m = 0; m < 4; ++m)
                        log("  map %u: image0 %06x image3 %06x mode0 %06x tlut %06x; texgen %u: %08x post %08x mtx %u\n", m,
                            state.bp(static_cast<std::uint8_t>(0x88 + m)), state.bp(static_cast<std::uint8_t>(0x94 + m)),
                            state.bp(static_cast<std::uint8_t>(0x80 + m)), state.bp(static_cast<std::uint8_t>(0x98 + m)), m,
                            state.xf(static_cast<std::uint16_t>(0x1040 + m)), state.xf(static_cast<std::uint16_t>(0x1050 + m)),
                            state.matrix_index(static_cast<std::uint8_t>(1 + m)));
                }
            }
        }
        if (renderer && !skip_draws) renderer->draw(state, d, memory);
        return true;
    }
    bool invalidate_vertices() override { return true; }
    // TX_SETMODE0 (BP 0x80-0x83, 0xA0-0xA3): the 2001 SDK in Luigi's Mansion leaves 0xF80000
    // in unused maps' modes, with the unused bits 17-18 and 22-23 set and max anisotropy at
    // its unused value 3. The hardware ignores the unused bits; anisotropy 3 samples as 4x.
    bool filter_bp(const sg::State&, std::uint8_t address, std::uint32_t& value) override {
        value &= ~0x00C60000u;
        if (((value >> 19) & 3) == 3) value &= ~(1u << 19);
        (void)address;
        return true;
    }
    bool bp_write(const sg::State& state, std::uint8_t address, std::uint32_t value) override {
        switch (address) {
        case 0x45:
            ++stats.draw_dones;
            gx_draw_done();
            break;
        case 0x47:
        case 0x48:
            ++stats.tokens;
            gx_token(static_cast<std::uint16_t>(value), address == 0x48);
            break;
        case 0x52:
            copy(state, value);
            break;
        default:
            break;
        }
        return true;
    }

    void copy(const sg::State& state, std::uint32_t command) {
        bool clear = command & 0x800;
        std::uint32_t source = (0x49u << 24) | state.bp(0x49), size = (0x4Au << 24) | state.bp(0x4A);
        std::uint32_t stride = (0x4Du << 24) | (state.bp(0x4D) & 0x3FF);
        std::uint32_t destination = (state.bp(0x4B) & 0x00FFFFFFu) << 5;
        if (command & 0x4000) {
            ++stats.display_copies;
            sg::DisplayCopy plan = sg::plan_display_copy(state, {source, size, stride, command & 3}, clear);
            xfbs[destination] = {plan.width, plan.height, plan.stride};
            if (renderer) renderer->display_copy(plan, destination);
        } else {
            ++stats.texture_copies;
            // The SDK selects the Z24 pixel format just for depth copies.
            bool depth = (state.bp(0x43) & 7) == 3;
            sg::TextureCopy plan = sg::plan_texture_copy(state, {source, size, stride, (0x52u << 24) | command, depth}, clear);
            static const bool log_copies = std::getenv("GCRT_GX_LOGCOPIES") != nullptr;
            static std::map<std::uint64_t, unsigned> seen;
            if (log_copies) copy_targets[destination] = plan.format;
            if (log_copies && !seen[(std::uint64_t(destination) << 32) | command]++)
                log("gx: texture copy to %08x format %u depth %d half %d rect %d,%d %dx%d stride %zu clear %d filter %d,%d,%d\n",
                    destination, plan.format, plan.depth, plan.half_scale, plan.x, plan.y, plan.width, plan.height, plan.stride,
                    clear, plan.filter[0], plan.filter[1], plan.filter[2]);
            if (renderer) renderer->texture_copy(plan, destination);
        }
    }
};

struct Backend final : GxBackend {
    RamReader memory;
    Sink sink;
    std::unique_ptr<sg::Decoder> decoder;

    Backend(GxRenderer* renderer, StarletOptions options) {
        sink.renderer = renderer;
        sink.skip_draws = options.skip_draws;
        sg::Limits limits;
        limits.merge_primitives = options.merge_primitives;
        for (unsigned a : {0x80u, 0x81u, 0x82u, 0x83u, 0xA0u, 0xA1u, 0xA2u, 0xA3u}) limits.filtered_bp.set(a);
        sg::EfbLayout layout;
        layout.height = options.efb_height;
        decoder = std::make_unique<sg::Decoder>(memory, sink, limits, layout);
        sink.decoder = decoder.get();
    }
    void feed(const std::uint8_t* bytes, std::size_t size) override {
        stats.bytes += size;
        static const bool fine = std::getenv("GCRT_GX_FINE") != nullptr;
        if (fine) {
            // Development: feed burst by burst to locate a failing command.
            for (std::size_t at = 0; at < size; at += 32) {
                sg::Status s = decoder->feed({bytes + at, std::min<std::size_t>(32, size - at)});
                if (!s) {
                    std::string hex;
                    for (std::size_t i = at >= 64 ? at - 64 : 0; i < std::min(size, at + 32); ++i) {
                        char b[4];
                        std::snprintf(b, sizeof b, "%02x", bytes[i]);
                        hex += b;
                        if (i + 1 == at) hex += " |";
                        if (i % 32 == 31) hex += "\n";
                    }
                    if (const char* d = std::getenv("GCRT_GX_MEM")) {
                        std::uint32_t a = static_cast<std::uint32_t>(std::strtoul(d, nullptr, 16));
                        for (int i = 0; i < 0x80; i += 16)
                            log("%08x: %08x %08x %08x %08x\n", a + i, read32(a + i), read32(a + i + 4), read32(a + i + 8),
                                read32(a + i + 12));
                    }
                    gx_trace_dump();
                    log_guest_stack("GX stream");
                    fatal("GX stream: %s in burst at %zu of %zu:\n%s", s.message.c_str(), at, size, hex.c_str());
                }
            }
            return;
        }
        sg::Status s = decoder->feed({bytes, size});
        if (!s) {
            if (const char* dump = std::getenv("GCRT_GX_DUMP")) {
                if (FILE* f = std::fopen(dump, "wb")) {
                    std::fwrite(bytes, 1, size, f);
                    std::fclose(f);
                }
            }
            gx_trace_dump();
            fatal("GX stream: %s (chunk of %zu bytes, %llu bytes before)", s.message.c_str(), size,
                  static_cast<unsigned long long>(stats.bytes - size));
        }
    }
    std::uint32_t peek(bool depth, int x, int y) override {
        return sink.renderer ? sink.renderer->peek(depth, x, y) : 0;
    }
    int snapshot() override { return sink.renderer && !sink.skip_draws ? sink.renderer->snapshot_efb() : -1; }
    int snapshot(int x, int y, int width, int height) override {
        return sink.renderer && !sink.skip_draws ? sink.renderer->snapshot_efb(x, y, width, height) : -1;
    }
    void release_snapshot(int index) override { sink.renderer->release_snapshot(index); }
    bool snapshot_ready(int index) const override { return sink.renderer->snapshot_ready(index); }
    bool peek_snapshot(int index, bool depth, int x, int y, std::uint32_t& value) const override {
        return sink.renderer->peek_snapshot(index, depth, x, y, value);
    }
    std::uint32_t peek_snapshot_wait(int index, bool depth, int x, int y) override {
        return sink.renderer->peek_snapshot_wait(index, depth, x, y);
    }
    void wait_snapshot(int index) override { sink.renderer->wait_snapshot(index); }
};

} // namespace

GxBackend* starlet_backend(GxRenderer* renderer, StarletOptions options) { return new Backend(renderer, options); }
const GxStats& gx_stats() { return stats; }

bool xfb_info(std::uint32_t address, int& width, int& height, std::size_t& stride) {
    auto it = xfbs.find(address);
    if (it == xfbs.end()) return false;
    width = it->second.width;
    height = it->second.height;
    stride = it->second.stride;
    return true;
}

} // namespace gcrt

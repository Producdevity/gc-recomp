// Decodes the GP command stream with Starlet and hands draws, copies and EFB access to a
// renderer. Without a renderer the stream is still decoded, so draw-done and token
// interrupts and display-copy bookkeeping work headless.
#pragma once

#include "gcrt/gx.hpp"

#include <starlet/gx/decoder.hpp>
#include <starlet/gx/display_copy.hpp>
#include <starlet/gx/texture_copy.hpp>

#include <cstdint>
#include <string>

namespace gcrt {

class GxRenderer {
public:
    virtual ~GxRenderer() = default;
    virtual void draw(const starlet::gx::State&, const starlet::gx::Draw&, starlet::gx::MemoryReader&) = 0;
    virtual void display_copy(const starlet::gx::DisplayCopy&, std::uint32_t xfb) = 0;
    virtual void texture_copy(const starlet::gx::TextureCopy&, std::uint32_t address) = 0;
    virtual std::uint32_t peek(bool depth, int x, int y) { (void)depth; (void)x; (void)y; return 0; }
    // EFB snapshots, as GxBackend describes them.
    virtual int snapshot_efb() { return -1; }
    virtual int snapshot_efb(int x, int y, int width, int height) { (void)x; (void)y; (void)width; (void)height; return snapshot_efb(); }
    virtual void release_snapshot(int) {}
    virtual bool snapshot_ready(int) const { return false; }
    virtual bool peek_snapshot(int, bool, int, int, std::uint32_t&) const { return false; }
    virtual std::uint32_t peek_snapshot_wait(int, bool depth, int x, int y) { return peek(depth, x, y); }
    virtual void wait_snapshot(int index) { (void)peek_snapshot_wait(index, true, 0, 0); }
    // Host DMA, as GxBackend describes it.
    virtual bool resident_in(std::uint32_t physical, std::uint32_t size) const { (void)physical; (void)size; return false; }
    virtual void before_dma(std::uint32_t physical, std::uint32_t size, bool write) { (void)physical; (void)size; (void)write; }
};

struct GxStats {
    std::uint64_t bytes = 0, draws = 0, display_copies = 0, texture_copies = 0, draw_dones = 0, tokens = 0;
};

struct StarletOptions {
    int efb_height = 480;
    bool merge_primitives = false;
    // Development: decode without drawing (copies and presentation still run), to measure
    // what the draws cost the other threads.
    bool skip_draws = false;
};

// The backend to pass to gx_init. The renderer may be null.
GxBackend* starlet_backend(GxRenderer*, StarletOptions);
const GxStats& gx_stats();
// The XFB a display copy last wrote at this physical address: width/height/stride, or false.
bool xfb_info(std::uint32_t address, int& width, int& height, std::size_t& stride);

} // namespace gcrt

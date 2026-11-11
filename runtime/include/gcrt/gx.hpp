// Command processor, pixel engine and the write-gather pipe feeding a GX backend.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace gcrt {

// Consumes the GP command stream: canonical big-endian GX bytes in submission order.
class GxBackend {
public:
    virtual ~GxBackend() = default;
    virtual void feed(const std::uint8_t* bytes, std::size_t size) = 0;
    // With a GX thread: called on it first, before any other call.
    virtual void attach() {}
    // Called before the CPU observes GP results (EFB reads, retraces).
    virtual void sync() {}
    // EFB access through 0xC8000000: x/y in EFB pixels.
    virtual std::uint32_t peek(bool depth, int x, int y) { (void)depth; (void)x; (void)y; return 0; }
    virtual void poke(bool depth, int x, int y, std::uint32_t value) { (void)depth; (void)x; (void)y; (void)value; }
    // EFB snapshots as of a draw-sync token (see gx_token). snapshot and release_snapshot
    // run on the backend's thread; snapshot returns -1 when none is available.
    virtual int snapshot() { return -1; }
    // As snapshot, but only the EFB pixels in the rectangle are read back ahead of the
    // handler's reads; reads elsewhere wait for the backend and stay exact.
    virtual int snapshot(int x, int y, int width, int height) { (void)x; (void)y; (void)width; (void)height; return snapshot(); }
    virtual void release_snapshot(int) {}
    // Any thread: the snapshot has been read back, and its value at (x, y).
    virtual bool snapshot_ready(int) const { return false; }
    virtual bool peek_snapshot(int, bool, int, int, std::uint32_t&) const { return false; }
    // Backend's thread: the snapshot's value at (x, y), waiting for its readback.
    virtual std::uint32_t peek_snapshot_wait(int, bool depth, int x, int y) { return peek(depth, x, y); }
};

// Token interrupts and the EFB reads of their handlers. Snapshot tokens are delivered once
// their snapshot is read back (forced: the GX thread had to wait for it), at most 100 ms
// later (late).
struct GxSyncStats {
    std::uint64_t snapshot_tokens = 0, snapshot_peeks = 0, waited_peeks = 0, late_tokens = 0, forced_tokens = 0;
};
const GxSyncStats& gx_sync_stats();

// threaded: the backend runs on its own thread (see gx.cpp); otherwise on the guest's.
void gx_init(GxBackend*, bool threaded = false);
// Pixel-engine events found in the stream (BP 0x45 draw done, 0x47/0x48 tokens).
void gx_draw_done();
void gx_token(std::uint16_t token, bool interrupt);
// Pushes buffered FIFO bytes to the backend (queues them for the GX thread).
void gx_flush();
// Pushes and waits until the backend has consumed everything.
void gx_drain();
bool gx_busy();   // the GX thread has queued or running work
// Runs a task on the backend's thread after the commands so far, without waiting.
void gx_post(std::function<void()> task);
// Runs a task on the backend's thread after the commands so far and waits for it.
void gx_call(const std::function<void()>& task);
// Development: prints the GCRT_GX_TRACE history.
void gx_trace_dump();

} // namespace gcrt

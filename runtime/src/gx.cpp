// Write-gather pipe, command processor and pixel engine registers.
//
// CPU writes to 0xCC008000 gather into 32-byte bursts that land in main memory at the
// CPU FIFO write pointer, as the hardware does: display lists recorded with
// GXBeginDisplayList are just such a FIFO. While the command processor reads the CPU FIFO
// (GP link and read enabled), each burst also goes to the backend. Synchronously, the
// backend consumes it at once. With a GX thread (gx_init's threaded), the bytes queue to a
// worker that runs the backend, like the console's GP reading its FIFO behind the CPU: the
// guest waits only where the console's CPU observes GP results (CP status reads, EFB
// access, a full queue), and draw-done and token events reach it as interrupts.
#include "gcrt/gx.hpp"
#include "gcrt/io.hpp"
#include "gcrt/runtime.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace gcrt {
namespace {

GxBackend* backend = nullptr;
GxSyncStats sync_stats;
// Token values whose interrupt handler has read the EFB: later interrupts with them take a
// snapshot. Set on the guest thread, read on the GX thread.
std::atomic<std::uint64_t> peek_tokens[1024];
bool reads_efb(std::uint16_t token) { return peek_tokens[token >> 6].load(std::memory_order_relaxed) >> (token & 63) & 1; }
std::vector<std::uint8_t> pending;
constexpr std::size_t kFeedThreshold = 32 * 1024;
// The GP read pointer: the FIFO address after the last byte the backend consumed. Queued
// bytes carry the address after them and the read-pointer epoch they were queued in.
std::atomic<std::uint32_t> gp_read{0};
std::atomic<unsigned> gp_epoch{0};
std::uint32_t pending_end = 0;

// The GX thread: runs the backend on queued FIFO bytes and tasks, in order.
struct Worker {
    // Bytes the guest may run ahead of the worker, about a frame of commands.
    static constexpr std::size_t kQueueLimit = 512 * 1024;
    struct Item {
        std::vector<std::uint8_t> bytes;
        std::function<void()> task;
        std::uint32_t fifo_end = 0;
        unsigned epoch = 0;
    };
    std::mutex mutex;
    std::condition_variable work, progress;
    std::deque<Item> items;
    std::size_t queued = 0;
    bool busy = false;
    std::thread thread;
    std::thread::id id;

    void start() {
        thread = std::thread([this] {
            id = std::this_thread::get_id();
            backend->attach();
            for (;;) {
                Item item;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    work.wait(lock, [this] { return !items.empty(); });
                    item = std::move(items.front());
                    items.pop_front();
                    busy = true;
                }
                if (item.task) item.task();
                else backend->feed(item.bytes.data(), item.bytes.size());
                if (item.fifo_end && item.epoch == gp_epoch.load(std::memory_order_relaxed))
                    gp_read.store(item.fifo_end, std::memory_order_release);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    queued -= item.bytes.size();
                    busy = false;
                }
                progress.notify_all();
            }
        });
        thread.detach();
    }
    void post(Item item) {
        std::unique_lock<std::mutex> lock(mutex);
        progress.wait(lock, [this] { return queued < kQueueLimit; });
        queued += item.bytes.size();
        items.push_back(std::move(item));
        work.notify_one();
    }
    void drain() {
        std::unique_lock<std::mutex> lock(mutex);
        progress.wait(lock, [this] { return items.empty() && !busy; });
    }
    bool on_worker() const { return std::this_thread::get_id() == id; }
};
Worker* worker = nullptr;

struct CommandProcessor final : Device {
    std::uint16_t regs[0x40] = {};
    const char* name() const override { return "CP"; }
    std::uint32_t pair(unsigned lo) const { return (std::uint32_t(regs[lo / 2 + 1]) << 16) | regs[lo / 2]; }
    void set_pair(unsigned lo, std::uint32_t v) {
        regs[lo / 2] = static_cast<std::uint16_t>(v);
        regs[lo / 2 + 1] = static_cast<std::uint16_t>(v >> 16);
    }
    bool linked() const { return (regs[1] & 0x11) == 0x11; }
    // The read pointer (0x38) or read-write distance (0x30) of the linked FIFO, as far as
    // the GX thread got, without waiting for it.
    std::uint32_t progress(unsigned lo) {
        gx_flush();
        const std::uint32_t read = gp_read.load(std::memory_order_acquire);
        if (lo == 0x38) return read;
        const std::uint32_t write = pair(0x34), size = pair(0x24) + 32 - pair(0x20);
        return write >= read ? write - read : write + size - read;
    }
    // A high half read latches the pointer for the low half read after it (GXGetFifoPtrs).
    std::uint32_t latch = 0;
    unsigned latched = 0;
    std::uint32_t read16(std::uint32_t o) override {
        if (worker && linked() && (o & ~0xAu) == 0x30) {
            const unsigned lo = o & ~2u;
            const bool high = o & 2;
            const std::uint32_t value = high || latched != lo ? progress(lo) : latch;
            latch = value;
            latched = high ? lo : 0;
            return high ? value >> 16 : value & 0xFFFF;
        }
        gx_drain();
        if (o == 0x00) return 0x0C;          // GP idle for reads and commands
        return o < 0x80 ? regs[o / 2] : 0;
    }
    void write16(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x80) return;
        regs[o / 2] = static_cast<std::uint16_t>(v);
        // A new read pointer (GXSetGPFifo): bytes still queued no longer move it.
        if (o == 0x38 || o == 0x3A) {
            gp_epoch.fetch_add(1, std::memory_order_relaxed);
            gp_read.store(pair(0x38), std::memory_order_release);
        }
    }
    std::uint32_t irq() override { return 0; }
};

struct PixelEngine final : Device {
    std::uint16_t regs[0x40] = {};
    std::uint16_t token = 0;
    // The EFB snapshot of the delivered token, until its interrupt is acknowledged.
    int snapshot = -1;
    void drop_snapshot() {
        const int s = snapshot;
        snapshot = -1;
        gx_post([s] { backend->release_snapshot(s); });
    }
    // Inside the token interrupt's handler: the token is pending with its interrupt enabled.
    bool in_token_handler() const { return (regs[5] & 0x5) == 0x5; }
    const char* name() const override { return "PE"; }
    std::uint32_t read16(std::uint32_t o) override {
        gx_flush();
        if (o == 0x0E) return token;
        return o < 0x80 ? regs[o / 2] : 0;
    }
    void write16(std::uint32_t o, std::uint32_t v) override {
        if (o >= 0x80) return;
        if (o == 0x0A) {
            std::uint16_t status = regs[5] & 0xC;
            status &= ~(v & 0xC);
            regs[5] = static_cast<std::uint16_t>(status | (v & 0x3));
            if ((v & 0x4) && snapshot >= 0) drop_snapshot();
            release();
            interrupts_changed();
            return;
        }
        regs[o / 2] = static_cast<std::uint16_t>(v);
    }
    // Events wait while an enabled PE interrupt is pending: the SDK handlers clear their
    // bit by writing back the whole register, which would drop a second pending bit.
    std::deque<std::function<void()>> deferred;
    bool blocked() const { return ((regs[5] & 0x4) && (regs[5] & 0x1)) || ((regs[5] & 0x8) && (regs[5] & 0x2)); }
    void raise(std::function<void()> e) {
        if (blocked() || !deferred.empty()) deferred.push_back(std::move(e));
        else e();
    }
    void release() {
        while (!blocked() && !deferred.empty()) {
            auto e = std::move(deferred.front());
            deferred.pop_front();
            e();
        }
    }
    std::uint32_t irq() override {
        std::uint32_t r = 0;
        if ((regs[5] & 0x4) && (regs[5] & 0x1)) r |= PI_PE_TOKEN;
        if ((regs[5] & 0x8) && (regs[5] & 0x2)) r |= PI_PE_FINISH;
        return r;
    }
};

CommandProcessor cp;
PixelEngine pe;

// Development (GCRT_GX_TRACE): recent gather bursts and CPU FIFO changes, printed when the
// backend rejects the stream.
struct TraceEntry {
    char kind;
    std::uint32_t a, b, c;
    std::uint8_t bytes[32];
};
TraceEntry trace_ring[64];
std::size_t trace_next = 0;
const bool tracing = std::getenv("GCRT_GX_TRACE") != nullptr;
void trace(char kind, std::uint32_t a, std::uint32_t b, std::uint32_t c, const std::uint8_t* bytes) {
    TraceEntry& e = trace_ring[trace_next++ % 64];
    e = {kind, a, b, c, {}};
    if (bytes) std::memcpy(e.bytes, bytes, 32);
}

struct Pipe final : GatherPipe {
    std::uint32_t base = 0, end = 0, wptr = 0, wrap = 0;

    void set_cpu_fifo(std::uint32_t b, std::uint32_t e, std::uint32_t w) override {
        if (verbosity > 3 && b != base) log("gx: CPU FIFO base %08x end %08x write %08x (gather %zu)\n", b, e, w, gathered());
        if (tracing) trace('F', b, e, w, nullptr);
        base = b;
        end = e;
        wptr = w & 0x03FFFFE0u;
        wrap = w & 0x04000000u;
    }
    std::uint32_t write_pointer() override { return wptr | wrap; }

    void burst(const std::uint8_t* bytes, std::size_t count) override {
        const bool linked = cp.linked();
        for (std::size_t k = 0; k < count; ++k) {
            const std::uint8_t* b = bytes + 32 * k;
            if (wptr < 0x3100) {
                static int reported = 0;
                if (reported++ < 3) log("gx: gather burst to low memory %08x (fifo %08x-%08x)\n", wptr, base, end);
            }
            if (phys_ok(wptr, 32)) std::memcpy(phys(wptr), b, 32);
            if (tracing) trace(linked ? 'B' : 'b', wptr, 0, 0, b);
            wptr += 32;
            // The end register holds the address of the FIFO's last burst.
            if (end && wptr > end) {
                wptr = base;
                wrap ^= 0x04000000u;
            }
        }
        set_cpu_fifo_write(wptr | wrap);
        if (linked) {
            cp.set_pair(0x34, wptr);
            cp.set_pair(0x38, wptr);
            cp.set_pair(0x30, 0);
            const std::size_t n = pending.size();
            pending.resize(n + 32 * count);
            std::memcpy(pending.data() + n, bytes, 32 * count);
            pending_end = wptr;
            static const std::size_t threshold = std::getenv("GCRT_GX_EAGER") ? 32 : kFeedThreshold;
            if (pending.size() >= threshold) gx_flush();
        }
    }
};

Pipe pipe;


struct Efb final : EfbAccess {
    std::uint32_t read32(std::uint32_t a) override {
        static const bool zero = std::getenv("GCRT_EFB_ZERO") != nullptr;
        static unsigned reads = 0;
        if (verbosity > 1 && reads++ % 1000 == 0) log("gx: EFB read %08x (%u reads)\n", a, reads);
        if (!backend || zero) return 0;
        const bool depth = a & 0x00400000u;
        const int x = (a >> 2) & 0x3FF, y = (a >> 12) & 0x3FF;
        std::uint32_t value = 0;
        if (pe.in_token_handler()) {
            if (const int s = pe.snapshot; s >= 0) {
                ++sync_stats.snapshot_peeks;
                if (backend->peek_snapshot(s, depth, x, y, value)) return value;
                ++sync_stats.waited_peeks;
                gx_call([&] { value = backend->peek_snapshot_wait(s, depth, x, y); });
                return value;
            }
            peek_tokens[pe.token >> 6].fetch_or(1ull << (pe.token & 63), std::memory_order_relaxed);
        }
        gx_call([&] { value = backend->peek(depth, x, y); });
        return value;
    }
    void write32(std::uint32_t a, std::uint32_t v) override {
        if (backend) gx_call([&] { backend->poke(a & 0x00400000u, (a >> 2) & 0x3FF, (a >> 12) & 0x3FF, v); });
    }
    void write16(std::uint32_t a, std::uint32_t v) override { write32(a & ~3u, v); }
};
Efb efb;

} // namespace

void gx_init(GxBackend* b, bool threaded) {
    backend = b;
    if (threaded && backend) {
        worker = new Worker;
        worker->start();
    }
    register_device(0x0000, 0x1000, &cp);
    register_device(0x1000, 0x1000, &pe);
    set_gather_pipe(&pipe);
    set_efb_access(&efb);
}

void gx_trace_dump() {
    if (!tracing) return;
    for (std::size_t i = trace_next > 64 ? trace_next - 64 : 0; i < trace_next; ++i) {
        const TraceEntry& e = trace_ring[i % 64];
        if (e.kind == 'F') {
            log("gx trace: fifo base %08x end %08x write %08x\n", e.a, e.b, e.c);
            continue;
        }
        char hex[65];
        for (int j = 0; j < 32; ++j) std::snprintf(hex + j * 2, 3, "%02x", e.bytes[j]);
        log("gx trace: %c %08x %s\n", e.kind, e.a, hex);
    }
}

void gx_flush() {
    if (gc_gp.fill >= 32) gc_gather_burst();
    if (pending.empty()) return;
    // Development (GCRT_GX_COST_NS): charge guest time per GX byte, to reproduce slow hosts.
    static const std::uint64_t cost = std::getenv("GCRT_GX_COST_NS") ? std::strtoull(std::getenv("GCRT_GX_COST_NS"), nullptr, 10) : 0;
    if (cost) {
        std::uint64_t ns = cost * pending.size();
        if (virtual_time) advance_virtual_time(now_ns() + ns);
        else for (std::uint64_t until = now_ns() + ns; now_ns() < until;) {}
    }
    if (worker) {
        Worker::Item item;
        item.bytes.swap(pending);
        item.fifo_end = pending_end;
        item.epoch = gp_epoch.load(std::memory_order_relaxed);
        pending.reserve(kFeedThreshold + GC_GATHER_BATCH);
        worker->post(std::move(item));
    } else if (backend) {
        std::vector<std::uint8_t> bytes;
        bytes.swap(pending);
        backend->feed(bytes.data(), bytes.size());
        if (pending.empty()) {
            bytes.clear();
            pending.swap(bytes);
        }
    } else {
        pending.clear();
    }
}

bool gx_busy() {
    if (!worker) return false;
    std::lock_guard<std::mutex> lock(worker->mutex);
    return !worker->items.empty() || worker->busy;
}

void gx_drain() {
    gx_flush();
    if (worker && !worker->on_worker()) worker->drain();
}

void gx_post(std::function<void()> task) {
    if (!worker) {
        task();
        return;
    }
    gx_flush();
    Worker::Item item;
    item.task = std::move(task);
    worker->post(std::move(item));
}

void gx_call(const std::function<void()>& task) {
    if (!worker || worker->on_worker()) {
        gx_flush();
        task();
        return;
    }
    gx_post(task);
    worker->drain();
}

namespace {
// Pixel-engine events found by the GX thread reach the guest at its next safepoint.
void on_guest(std::function<void()> change) {
    if (worker && worker->on_worker()) {
        schedule_at(now_ns(), std::move(change));
        wake_guest();
    } else {
        change();
    }
}

// Guest thread: PE events in stream order. One with a snapshot waits until the snapshot is
// read back, and later events wait behind it; after kSnapshotWait it is delivered anyway
// and its handler's EFB reads wait for the readback instead.
constexpr std::uint64_t kSnapshotWait = 100000000;
struct Ordered {
    int snapshot;
    std::uint64_t deadline;
    std::function<void()> event;
    bool forced = false;
};
std::deque<Ordered> ordered;
bool pump_scheduled = false;

void pump() {
    while (!ordered.empty()) {
        Ordered& o = ordered.front();
        if (o.snapshot >= 0 && !backend->snapshot_ready(o.snapshot)) {
            if (now_ns() < o.deadline) {
                // Readbacks land as later GPU work is submitted. With the GX thread idle
                // (the guest waits for this token) none follows, so read one pixel of the
                // snapshot there, which waits for its readback.
                if (!o.forced && !gx_busy()) {
                    o.forced = true;
                    ++sync_stats.forced_tokens;
                    const int s = o.snapshot;
                    gx_post([s] { backend->peek_snapshot_wait(s, true, 0, 0); });
                }
                if (!pump_scheduled) {
                    pump_scheduled = true;
                    schedule_in(500000, [] {
                        pump_scheduled = false;
                        pump();
                    });
                }
                return;
            }
            ++sync_stats.late_tokens;
        }
        std::function<void()> event = std::move(o.event);
        ordered.pop_front();
        pe.raise(std::move(event));
    }
}

void deliver(int snapshot, std::function<void()> event) {
    if (ordered.empty() && snapshot < 0) {
        pe.raise(std::move(event));
        return;
    }
    ordered.push_back({snapshot, now_ns() + kSnapshotWait, std::move(event)});
    pump();
}
} // namespace

void gx_draw_done() {
    on_guest([] {
        deliver(-1, [] {
            pe.regs[5] |= 0x8;
            interrupts_changed();
        });
    });
}

// Once a token's interrupt handler has read the EFB, interrupts with that token snapshot
// it: the handler runs when the GPU has reached the token, as the console's interrupt
// follows its GP, instead of the guest waiting at its first read for everything queued
// since. Other tokens are delivered at once.
// GCRT_SYNC_LIVE=1 keeps the live reads, for A/B checks.
void gx_token(std::uint16_t token, bool interrupt) {
    static const bool live = std::getenv("GCRT_SYNC_LIVE") != nullptr;
    int snapshot = -1;
    if (interrupt && !live && worker && worker->on_worker() && !virtual_time && reads_efb(token))
        snapshot = backend->snapshot();
    on_guest([token, interrupt, snapshot] {
        deliver(snapshot, [token, interrupt, snapshot] {
            if (pe.snapshot >= 0) pe.drop_snapshot();
            pe.token = token;
            pe.snapshot = snapshot;
            if (snapshot >= 0) ++sync_stats.snapshot_tokens;
            if (interrupt) {
                pe.regs[5] |= 0x4;
                interrupts_changed();
            }
        });
    });
}

const GxSyncStats& gx_sync_stats() { return sync_stats; }

} // namespace gcrt

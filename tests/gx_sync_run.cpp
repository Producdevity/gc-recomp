// Draw-sync tokens (runtime/src/gx.cpp): once a token handler reads the EFB, interrupt
// tokens wait for an EFB snapshot, later PE events wait behind them, and the handler's
// reads come from the snapshot.
#include "gcrt/gx.hpp"
#include "gcrt/io.hpp"
#include "gcrt/runtime.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <thread>
#include <vector>

namespace {
unsigned failures = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL %s\n", what);
        ++failures;
    }
}

struct FakeBackend final : gcrt::GxBackend {
    // stuck: the readback never lands, even when waited for.
    std::atomic<bool> ready{false}, hold{false}, stuck{false};
    std::atomic<std::size_t> fed{0};
    int next = 5;
    unsigned taken = 0, live = 0, waited = 0;
    std::vector<int> released;
    void feed(const std::uint8_t*, std::size_t size) override {
        while (hold) std::this_thread::yield();
        fed += size;
    }
    std::uint32_t peek(bool, int x, int y) override {
        ++live;
        return 0x100000u + static_cast<std::uint32_t>(x * 1000 + y);
    }
    int snapshot() override {
        ++taken;
        return next;
    }
    void release_snapshot(int index) override { released.push_back(index); }
    bool snapshot_ready(int) const override { return ready; }
    bool peek_snapshot(int index, bool, int x, int y, std::uint32_t& value) const override {
        if (!ready) return false;
        value = static_cast<std::uint32_t>(index) << 20 | static_cast<std::uint32_t>(x * 1000 + y);
        return true;
    }
    std::uint32_t peek_snapshot_wait(int index, bool, int x, int y) override {
        ++waited;
        if (!stuck) ready = true;
        return static_cast<std::uint32_t>(index) << 20 | static_cast<std::uint32_t>(x * 1000 + y);
    }
};
FakeBackend fake;

constexpr std::uint32_t kPeStatus = 0xCC00100Au, kPeToken = 0xCC00100Eu;
std::uint32_t status() { return gcrt::read16(kPeStatus); }
std::uint32_t depth(int x, int y) {
    return gcrt::read32(0xC8400000u | (static_cast<std::uint32_t>(y) << 12) | (static_cast<std::uint32_t>(x) << 2));
}
// Runs the guest's due events until pred holds, for at most ms.
bool run_until(const std::function<bool()>& pred, int ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    do {
        gcrt::run_due_events();
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    } while (std::chrono::steady_clock::now() < end);
    return pred();
}
void on_gx(std::function<void()> task) {
    gcrt::gx_post(std::move(task));
    gcrt::gx_drain();
}
} // namespace

int main() {
    if (!gcrt::memory_init()) return 1;
    gcrt::io_init();
    gcrt::gx_init(&fake, true);
    gcrt::write16(kPeStatus, 0x3);   // token and finish interrupts enabled

    // Handlers that read the EFB read it live and turn snapshots on for their tokens.
    for (std::uint16_t t = 1; t <= 4; ++t) {
        on_gx([t] { gcrt::gx_token(t, true); });
        check(run_until([] { return status() & 0x4; }, 100), "first tokens raised");
        check(depth(3, 4) == 0x100000u + 3004 && fake.live == t, "first handlers read the live EFB");
        gcrt::write16(kPeStatus, 0x4 | 0x3);
    }
    check(fake.taken == 0 && !(status() & 0x4), "no snapshot before a token's handler read the EFB");

    // A token whose handler never read the EFB is delivered at once, without a snapshot.
    on_gx([] { gcrt::gx_token(9, true); });
    check(run_until([] { return status() & 0x4; }, 100) && fake.taken == 0, "other tokens take no snapshot");
    gcrt::write16(kPeStatus, 0x4 | 0x3);

    // The next token and a draw done wait until the snapshot is read back; the GX thread
    // stays busy, so the readback is left to land by itself.
    on_gx([] {
        gcrt::gx_token(2, true);
        gcrt::gx_draw_done();
    });
    fake.hold = true;
    gcrt::gx_post([] {
        while (fake.hold) std::this_thread::yield();
    });
    check(fake.taken == 1, "interrupt token takes a snapshot");
    check(!run_until([] { return status() & 0xC; }, 5), "token and draw done wait for the snapshot");
    check(gcrt::read16(kPeToken) == 9, "token register keeps the delivered token");
    fake.ready = true;
    check(run_until([] { return status() & 0x4; }, 100), "token raised once the snapshot is ready");
    fake.hold = false;
    gcrt::gx_drain();
    check(fake.waited == 0, "a busy GX thread is not made to wait for the readback");
    check(gcrt::read16(kPeToken) == 2, "token register updated on delivery");
    check(!(status() & 0x8), "draw done stays behind the pending token");
    check(depth(7, 9) == (5u << 20 | 7009u) && fake.live == 4, "handler reads the snapshot");
    gcrt::write16(kPeStatus, 0x4 | 0x3);
    gcrt::gx_drain();
    check(fake.released.size() == 1 && fake.released[0] == 5, "acknowledging the token releases its snapshot");
    check(status() & 0x8, "draw done follows the acknowledged token");
    gcrt::write16(kPeStatus, 0x8 | 0x3);

    // Reads outside a token handler stay live.
    check(depth(1, 1) == 0x100000u + 1001 && fake.live == 5, "reads outside handlers are live");

    // With the GX thread idle, a pending readback is waited for there.
    fake.ready = false;
    fake.next = 7;
    on_gx([] { gcrt::gx_token(4, true); });
    check(run_until([] { return status() & 0x4; }, 50), "idle GX thread waits for the readback");
    check(fake.waited == 1 && depth(5, 6) == (7u << 20 | 5006u), "forced snapshot then reads without waiting");
    gcrt::write16(kPeStatus, 0x4 | 0x3);
    gcrt::gx_drain();

    // A snapshot that is never read back: delivered at the deadline, its reads wait.
    fake.ready = false;
    fake.stuck = true;
    fake.next = 6;
    on_gx([] { gcrt::gx_token(3, true); });
    check(!run_until([] { return status() & 0x4; }, 20), "late token waits");
    check(run_until([] { return status() & 0x4; }, 400), "late token delivered after the deadline");
    check(depth(2, 2) == (6u << 20 | 2002u) && fake.waited == 3 && fake.live == 5, "late handler waits for its snapshot");
    gcrt::write16(kPeStatus, 0x4 | 0x3);
    gcrt::gx_drain();
    check(fake.released.size() == 3 && fake.released[2] == 6, "late snapshot released");
    fake.stuck = false;

    const gcrt::GxSyncStats& s = gcrt::gx_sync_stats();
    check(s.snapshot_tokens == 3 && s.forced_tokens == 2 && s.late_tokens == 1 && s.snapshot_peeks == 3 && s.waited_peeks == 1,
          "sync statistics");

    // FIFO pointer reads report how far the GX thread got instead of waiting for it.
    constexpr std::uint32_t base = 0x00100000u, end = 0x00110000u - 32;
    gcrt::write32(0xCC00300Cu, base);
    gcrt::write32(0xCC003010u, end);
    gcrt::write32(0xCC003014u, base);
    const auto cp_pair = [](std::uint32_t lo, std::uint32_t value) {
        gcrt::write16(0xCC000000u + lo, value & 0xFFFF);
        gcrt::write16(0xCC000002u + lo, value >> 16);
    };
    cp_pair(0x20, base);
    cp_pair(0x24, end);
    cp_pair(0x34, base);
    cp_pair(0x38, base);
    gcrt::write16(0xCC000002u, 0x11);   // GP read and link enabled
    const auto read_ptr = [] { return gcrt::read16(0xCC00003Au) << 16 | gcrt::read16(0xCC000038u); };
    const auto distance = [] { return gcrt::read16(0xCC000032u) << 16 | gcrt::read16(0xCC000030u); };
    fake.hold = true;
    for (int i = 0; i < 16; ++i) gcrt::write32(0xCC008000u, 0x00000000u);   // two bursts of NOPs
    check(read_ptr() == base, "read pointer stays while the GX thread is busy");
    check(distance() == 64, "distance counts the bytes the GX thread has not read");
    fake.hold = false;
    gcrt::gx_drain();
    check(fake.fed == 64, "queued bursts reach the backend");
    check(read_ptr() == base + 64 && distance() == 0, "read pointer catches up with the write pointer");
    if (failures) return 1;
    std::printf("gx sync: ok\n");
    return 0;
}

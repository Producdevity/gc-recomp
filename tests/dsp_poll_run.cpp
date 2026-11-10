// A poll of an empty DSP-to-CPU mailbox runs due events (runtime/src/dsp.cpp): mails a
// DSP thread posts reach the mailbox through an event, and SDK poll loops may never reach
// a safepoint.
#include "gcrt/devices.hpp"
#include "gcrt/io.hpp"
#include "gcrt/runtime.hpp"

#include <cstdio>

int main() {
    if (!gcrt::memory_init()) return 1;
    gcrt::io_init();
    gcrt::dsp_init({});
    bool ran = false;
    gcrt::schedule_at(gcrt::now_ns(), [&ran] { ran = true; });
    const std::uint32_t hi = gcrt::read16(0xCC005004u);
    if (!ran || (hi & 0x8000)) {
        std::printf("FAIL an empty mailbox poll runs due events (ran %d, mail %04x)\n", ran, hi);
        return 1;
    }
    std::printf("dsp poll: ok\n");
    return 0;
}

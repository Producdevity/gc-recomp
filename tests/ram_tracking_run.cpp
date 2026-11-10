// MEM1 write generations (runtime/src/memory.cpp): queries protect pages, writes through
// either mirror or from another thread advance them, and system calls can still write.
#include "gcrt/runtime.hpp"

#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <thread>
#include <unistd.h>

namespace {
unsigned failures = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL %s\n", what);
        ++failures;
    }
}
} // namespace

int main() {
    if (!gcrt::memory_init()) return 1;
    check(gcrt::ram_range_generation(0x1000, 64) == 0, "no generation before tracking");
    gcrt::ram_tracking_enable();
    const std::uint32_t a = 0x00100000u, b = 0x00200000u;
    const std::uint64_t g1 = gcrt::ram_range_generation(a, 0x3000);
    check(g1 != 0, "generation after a query");
    check(gcrt::ram_range_generation(a, 0x3000) == g1, "unchanged without writes");
    check(gcrt::read32(0x80000000u + a) == 0, "reads of protected pages");

    gcrt::write32(0x80000000u + a + 0x2000, 0x12345678u);   // cached mirror
    check(gcrt::read32(0x80000000u + a + 0x2000) == 0x12345678u, "write to a protected page lands");
    const std::uint64_t g2 = gcrt::ram_range_generation(a, 0x3000);
    check(g2 > g1, "a write advances the generation");
    check(gcrt::ram_range_generation(a, 0x3000) == g2, "stable again after the query");

    gcrt::write32(0xC0000000u + a + 4, 1);   // uncached mirror
    check(gcrt::ram_range_generation(a, 0x3000) > g2, "writes through the uncached mirror count");

    const std::uint64_t g3 = gcrt::ram_range_generation(b, 64);
    std::thread([&] { gcrt::write32(0x80000000u + b, 7); }).join();
    check(gcrt::ram_range_generation(b, 64) > g3, "writes from another thread count");

    char path[] = "/tmp/gc-ram-tracking-XXXXXX";
    int fd = mkstemp(path);
    const char text[] = "bytes from a file";
    check(fd >= 0 && write(fd, text, sizeof text) == static_cast<ssize_t>(sizeof text), "temporary file");
    const std::uint64_t g4 = gcrt::ram_range_generation(b, 64);
    gcrt::ram_will_write(b, sizeof text);
    check(pread(fd, gcrt::phys(b), sizeof text, 0) == static_cast<ssize_t>(sizeof text), "pread into a tracked page");
    check(gcrt::ram_range_generation(b, 64) > g4, "a system call write counts");
    close(fd);
    unlink(path);

    // A query over pages in mixed states protects each unprotected run; writes to every
    // page of the range then fault once each.
    const std::uint32_t c = 0x00300000u, page = static_cast<std::uint32_t>(sysconf(_SC_PAGESIZE));
    gcrt::ram_range_generation(c + 2 * page, 1);
    gcrt::write32(0x80000000u + c + 2 * page, 1);
    gcrt::ram_range_generation(c + page, 1);
    const std::uint64_t g5 = gcrt::ram_range_generation(c, 4 * page);
    const std::uint64_t faults = gcrt::ram_tracking_faults();
    for (std::uint32_t k = 0; k < 4; ++k) gcrt::write32(0x80000000u + c + k * page + 8, k);
    check(gcrt::ram_tracking_faults() == faults + 4, "one fault per protected page");
    for (std::uint32_t k = 0; k < 4; ++k) check(gcrt::ram_range_generation(c + k * page, 4) > g5, "every page of a mixed range advances");

    // A page written between three queries in a row is left unprotected: its queries report
    // no generation (callers compare bytes) and its writes stop faulting, until it has been
    // left alone for 65536 queries.
    const std::uint32_t d = 0x00400000u;
    for (int k = 0; k < 3; ++k) {
        check(gcrt::ram_range_generation(d, 16) != 0, "a written page keeps a generation at first");
        gcrt::write32(0x80000000u + d, static_cast<std::uint32_t>(k));
    }
    check(gcrt::ram_range_generation(d, 16) == 0, "a page written at every query is left untracked");
    const std::uint64_t before = gcrt::ram_tracking_faults();
    gcrt::write32(0x80000000u + d, 9);
    check(gcrt::ram_tracking_faults() == before && gcrt::ram_range_generation(d, 16) == 0, "an untracked page does not fault");
    for (int k = 0; k < 65536; ++k) gcrt::ram_range_generation(c, 4);
    check(gcrt::ram_range_generation(d, 16) != 0, "the page is tracked again later");
    gcrt::write32(0x80000000u + d, 10);
    check(gcrt::ram_tracking_faults() == before + 1, "and its writes fault again");

    std::printf("%u failures\n", failures);
    return failures ? 1 : 0;
}

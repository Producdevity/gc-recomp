// Guest address space: a 4 GiB reservation where guest address A is host gc_mem + A.
// MEM1 appears at 0x80000000 (cached) and 0xC0000000 (uncached) through two mappings of
// one shared object; the locked cache sits at 0xE0000000. Everything else stays
// inaccessible, so a stray guest access faults instead of corrupting host memory.
#include "gcrt/runtime.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#ifdef __linux__
#include <sys/syscall.h>
#endif

uint8_t* gc_mem = nullptr;

namespace gcrt {
namespace {
constexpr std::uint64_t kSpace = 0x100000000ull;
constexpr std::size_t kRamMap = 32u * 1024 * 1024;
constexpr std::size_t kLockedCache = 64u * 1024;
std::uint8_t* aram_ = nullptr;

// Write tracking (ram_range_generation). States: 0 writable and untracked, 1 protected,
// 2 written since its last query, 3 writable and left untracked: a page found written
// kVolatileStreak times, each within kVolatileWindow queries of the last, stays
// unprotected for kVolatileQueries queries, since
// a fault (signal plus two mprotect calls, each a TLB shootdown) costs far more than the
// byte comparison its callers fall back to. Changes happen under the lock, with the
// protection.
std::size_t page_size = 0;
std::size_t page_count = 0;
std::atomic<std::uint8_t>* page_state = nullptr;
std::atomic<std::uint64_t>* page_generation = nullptr;
std::uint8_t* page_streak = nullptr;        // recent queries that found the page written
std::uint64_t* page_last_written = nullptr; // query count when that was last the case
std::uint64_t* page_untracked_until = nullptr;
std::uint64_t queries = 0;
constexpr std::uint8_t kVolatileStreak = 3;
constexpr std::uint64_t kVolatileWindow = 2048, kVolatileQueries = 65536;
std::atomic<std::uint64_t> next_generation{1};
std::atomic_flag tracking_lock = ATOMIC_FLAG_INIT;
bool tracking = false;
struct sigaction previous_segv, previous_bus;

void lock() { while (tracking_lock.test_and_set(std::memory_order_acquire)) {} }
void unlock() { tracking_lock.clear(std::memory_order_release); }
std::atomic<std::uint64_t> fault_count{0};

void protect(std::size_t page, bool read_only, std::size_t pages = 1) {
    for (std::uint32_t mirror : {0x80000000u, 0xC0000000u})
        mprotect(gc_mem + mirror + page * page_size, pages * page_size, read_only ? PROT_READ : PROT_READ | PROT_WRITE);
}
void on_fault(int signal, siginfo_t* info, void* context) {
    const auto at = reinterpret_cast<std::uintptr_t>(info->si_addr), base = reinterpret_cast<std::uintptr_t>(gc_mem);
    if (tracking && at >= base) {
        for (std::uint64_t mirror : {0x80000000ull, 0xC0000000ull}) {
            const std::uint64_t offset = at - base;
            if (offset < mirror || offset >= mirror + page_count * page_size) continue;
            const std::size_t page = (offset - mirror) / page_size;
            lock();
            const std::uint8_t state = page_state[page].load();
            if (state == 1) {
                page_state[page] = 2;
                protect(page, false);
                fault_count.fetch_add(1, std::memory_order_relaxed);
            }
            unlock();
            if (state != 0) return;   // retry the store
        }
    }
    const struct sigaction& previous = signal == SIGBUS ? previous_bus : previous_segv;
    if (previous.sa_flags & SA_SIGINFO) {
        previous.sa_sigaction(signal, info, context);
    } else if (previous.sa_handler != SIG_DFL && previous.sa_handler != SIG_IGN) {
        previous.sa_handler(signal);
    } else {
        std::signal(signal, SIG_DFL);
        std::raise(signal);
    }
}

int shared_memory(std::size_t size) {
#ifdef __linux__
    int fd = static_cast<int>(syscall(SYS_memfd_create, "gcram", 0));
#else
    char name[64];
    std::snprintf(name, sizeof name, "/gcram-%d", static_cast<int>(getpid()));
    int fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd >= 0) shm_unlink(name);
#endif
    if (fd < 0) return -1;
    if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}
} // namespace

bool memory_init() {
    void* base = mmap(nullptr, kSpace, PROT_NONE, MAP_PRIVATE | MAP_ANON | MAP_NORESERVE, -1, 0);
    if (base == MAP_FAILED) return false;
    gc_mem = static_cast<std::uint8_t*>(base);
    int fd = shared_memory(kRamMap);
    if (fd < 0) return false;
    for (std::uint32_t at : {0x80000000u, 0xC0000000u}) {
        void* p = mmap(gc_mem + at, kRamMap, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, 0);
        if (p == MAP_FAILED) return false;
    }
    close(fd);
    void* lc = mmap(gc_mem + 0xE0000000u, kLockedCache, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON | MAP_FIXED, -1, 0);
    if (lc == MAP_FAILED) return false;
    aram_ = static_cast<std::uint8_t*>(std::calloc(kAramSize, 1));
    return aram_ != nullptr;
}

std::uint8_t* aram() { return aram_; }

void ram_tracking_enable() {
    if (tracking || !gc_mem) return;
    page_size = static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
    page_count = kRamSize / page_size;
    page_state = new std::atomic<std::uint8_t>[page_count]();
    page_generation = new std::atomic<std::uint64_t>[page_count]();
    page_streak = new std::uint8_t[page_count]();
    page_last_written = new std::uint64_t[page_count]();
    page_untracked_until = new std::uint64_t[page_count]();
    struct sigaction action{};
    action.sa_sigaction = on_fault;
    action.sa_flags = SA_SIGINFO | SA_ONSTACK;
    sigaction(SIGSEGV, &action, &previous_segv);
    sigaction(SIGBUS, &action, &previous_bus);
    tracking = true;
}

std::uint64_t ram_range_generation(std::uint32_t physical, std::uint32_t size) {
    if (!tracking || !size || physical >= kRamSize || size > kRamSize - physical) return 0;
    std::uint64_t newest = 0;
    bool untracked = false;
    const std::size_t first = physical / page_size, last = (physical + size - 1) / page_size;
    lock();
    ++queries;
    std::size_t run = 0, run_pages = 0;   // contiguous pages to protect with one call
    for (std::size_t page = first; page <= last; ++page) {
        std::uint8_t state = page_state[page];
        if (state == 3 && queries >= page_untracked_until[page]) {
            state = 0;
            page_state[page] = 0;
            page_streak[page] = 0;
        }
        if (state == 2) {
            page_streak[page] = queries - page_last_written[page] <= kVolatileWindow ? page_streak[page] + 1 : 1;
            page_last_written[page] = queries;
            if (page_streak[page] >= kVolatileStreak) {
                page_state[page] = state = 3;
                page_untracked_until[page] = queries + kVolatileQueries;
            }
        }
        if (state == 3) {
            untracked = true;
        } else if (state != 1) {
            if (!run_pages) run = page;
            ++run_pages;
            page_state[page] = 1;
            page_generation[page] = next_generation++;
        }
        if (run_pages && (state == 1 || state == 3)) {
            protect(run, true, run_pages);
            run_pages = 0;
        }
        newest = std::max<std::uint64_t>(newest, page_generation[page]);
    }
    if (run_pages) protect(run, true, run_pages);
    unlock();
    return untracked ? 0 : newest;
}

std::uint64_t ram_tracking_faults() { return fault_count.load(std::memory_order_relaxed); }

void ram_will_write(std::uint32_t physical, std::uint32_t size) {
    if (!tracking || !size || physical >= kRamSize) return;
    size = std::min(size, static_cast<std::uint32_t>(kRamSize - physical));
    lock();
    for (std::size_t page = physical / page_size; page <= (physical + size - 1) / page_size; ++page)
        if (page_state[page] == 1) {
            page_state[page] = 2;
            protect(page, false);
        }
    unlock();
}

} // namespace gcrt

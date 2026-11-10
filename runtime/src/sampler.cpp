// In-process sampling of the guest thread: a timer thread signals it, and the handler
// records which recompiled functions (f_XXXXXXXX) are on its host stack. Reports the
// innermost guest function and its callers, by sample count.
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <execinfo.h>
#ifdef __APPLE__
#include <sys/ucontext.h>
#else
#include <ucontext.h>
#endif
#include <map>
#include <pthread.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace gcrt {
namespace {

constexpr int kDepth = 48;
constexpr std::size_t kSlots = 1 << 16;
struct Sample {
    void* frames[kDepth];
    void* pc;   // The interrupted instruction.
    void* lr;   // Its link register: the caller of a leaf function.
    int count;
    std::uint32_t thread;   // The guest's current OSThread (0x800000E4).
    std::uint64_t cpu_ns;   // The sampled thread's CPU time: samples are wall-clock, and the
                            // difference to the previous sample separates running from waiting.
};
Sample* samples = nullptr;
std::atomic<std::size_t> taken{0};
pthread_t guest;
std::atomic<bool> running{false};
std::thread ticker;

// The guest thread's own stack, for frame-pointer walks outside fibers.
std::uintptr_t thread_low = 0, thread_high = 0;

// Signal handler: records the interrupted PC and walks the frame-pointer chain within the
// current stack's bounds. glibc's backtrace() unwinds through frames interrupted mid-
// prologue and then dereferences garbage, so it is not used here.
void handler(int, siginfo_t*, void* context) {
    std::size_t n = taken.fetch_add(1);
    if (n >= kSlots) return;
    auto* uc = static_cast<ucontext_t*>(context);
#if defined(__APPLE__)
    std::uintptr_t pc = uc->uc_mcontext->__ss.__pc, fp = uc->uc_mcontext->__ss.__fp, sp = uc->uc_mcontext->__ss.__sp;
    std::uintptr_t lr = uc->uc_mcontext->__ss.__lr;
#else
    std::uintptr_t pc = uc->uc_mcontext.pc, fp = uc->uc_mcontext.regs[29], sp = uc->uc_mcontext.sp;
    std::uintptr_t lr = uc->uc_mcontext.regs[30];
#endif
    Sample& s = samples[n];
    timespec cpu{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu);
    s.cpu_ns = static_cast<std::uint64_t>(cpu.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(cpu.tv_nsec);
    std::uint32_t current = 0;
    if (gc_mem) std::memcpy(&current, gc_mem + 0x800000E4u, 4);
    s.thread = __builtin_bswap32(current);
    s.pc = reinterpret_cast<void*>(pc);
    s.lr = reinterpret_cast<void*>(lr);
    s.frames[0] = s.pc;
    int count = 1;
    std::uintptr_t low = 0, high = 0;
    if (!fiber_stack_range(low, high) || sp < low || sp >= high) { low = thread_low; high = thread_high; }
    if (sp > low) low = sp;
    while (count < kDepth && fp >= low && fp + 16 <= high && !(fp & 15)) {
        const auto* record = reinterpret_cast<const std::uintptr_t*>(fp);
        s.frames[count++] = reinterpret_cast<void*>(record[1]);
        if (record[0] <= fp) break;
        fp = record[0];
    }
    s.count = count;
}

// Address of the recompiled guest function containing a host code address, or 0.
// Translated code has no symbols: its host entry points, sorted, with their guest addresses.
std::vector<std::pair<std::uintptr_t, std::uint32_t>> translated;

std::uint32_t guest_function(void* address) {
    Dl_info info{};
    if (!dladdr(address, &info) || !info.dli_fname) {
        const auto a = reinterpret_cast<std::uintptr_t>(address);
        auto it = std::upper_bound(translated.begin(), translated.end(), std::make_pair(a, ~0u));
        return it == translated.begin() ? 0 : std::prev(it)->second;
    }
    if (!info.dli_sname) return 0;
    const char* name = info.dli_sname;
    if (name[0] == '_') ++name;
    if (name[0] != 'f' || name[1] != '_') return 0;
    return static_cast<std::uint32_t>(std::strtoul(name + 2, nullptr, 16));
}

} // namespace

void sampler_start(unsigned hz) { sampler_start_thread(pthread_self(), hz); }

void sampler_start_thread(pthread_t thread, unsigned hz) {
    if (running) return;
    samples = new Sample[kSlots];
    guest = thread;
#if defined(__APPLE__)
    thread_high = reinterpret_cast<std::uintptr_t>(pthread_get_stackaddr_np(guest));
    thread_low = thread_high - pthread_get_stacksize_np(guest);
#else
    pthread_attr_t attr;
    if (pthread_getattr_np(guest, &attr) == 0) {
        void* base = nullptr;
        std::size_t size = 0;
        pthread_attr_getstack(&attr, &base, &size);
        thread_low = reinterpret_cast<std::uintptr_t>(base);
        thread_high = thread_low + size;
        pthread_attr_destroy(&attr);
    }
#endif
    struct sigaction action{};
    action.sa_sigaction = handler;
    action.sa_flags = SA_RESTART | SA_SIGINFO;
    sigaction(SIGPROF, &action, nullptr);
    running = true;
    ticker = std::thread([hz] {
        while (running) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000000 / hz));
            pthread_kill(guest, SIGPROF);
        }
    });
    ticker.detach();
}

void sampler_report(unsigned top) {
    std::size_t n = std::min(taken.load(), kSlots);
    // dladdr is slow on the device; each distinct address is resolved once.
    std::map<void*, std::uint32_t> resolved;
    auto guest_of = [&resolved](void* address) {
        auto it = resolved.find(address);
        if (it == resolved.end()) it = resolved.emplace(address, guest_function(address)).first;
        return it->second;
    };
    std::map<void*, std::string> host_names;
    std::map<std::uint32_t, unsigned> self, inclusive;
    std::map<std::vector<std::uint32_t>, unsigned> stacks;
    std::map<std::string, unsigned> host;   // Innermost host function, outside guest code.
    unsigned guest_samples = 0, host_samples = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!guest_of(samples[i].pc)) {
            auto it = host_names.find(samples[i].pc);
            if (it == host_names.end()) {
                Dl_info dl{};
                it = host_names.emplace(samples[i].pc, dladdr(samples[i].pc, &dl) && dl.dli_sname ? dl.dli_sname
                                                         : (dl.dli_fname ? dl.dli_fname : "?")).first;
            }
            ++host[it->second];
            ++host_samples;
        }
        std::vector<std::uint32_t> chain;
        for (int f = 0; f < samples[i].count; ++f)
            if (std::uint32_t g = guest_of(samples[i].frames[f])) chain.push_back(g);
        if (chain.empty()) continue;
        ++guest_samples;
        ++self[chain.front()];
        std::vector<std::uint32_t> seen;
        for (auto g : chain)
            if (std::find(seen.begin(), seen.end(), g) == seen.end()) {
                seen.push_back(g);
                ++inclusive[g];
            }
        chain.resize(std::min<std::size_t>(chain.size(), 6));
        ++stacks[chain];
    }
    log("sampler: %zu samples, %u in guest code\n", n, guest_samples);
    auto print = [&](const char* title, const std::map<std::uint32_t, unsigned>& m) {
        std::vector<std::pair<unsigned, std::uint32_t>> v;
        for (auto& [a, c] : m) v.push_back({c, a});
        std::sort(v.rbegin(), v.rend());
        log("  %s:\n", title);
        for (unsigned i = 0; i < std::min<std::size_t>(top, v.size()); ++i)
            log("    %5.1f%%  %08x\n", 100.0 * v[i].first / std::max(1u, guest_samples), v[i].second);
    };
    print("self", self);
    print("inclusive", inclusive);
    std::vector<std::pair<unsigned, std::string>> hv;
    for (auto& [name, c] : host) hv.push_back({c, name});
    std::sort(hv.rbegin(), hv.rend());
    log("  host self (%u samples outside guest code):\n", host_samples);
    for (unsigned i = 0; i < std::min<std::size_t>(top, hv.size()); ++i)
        log("    %5.1f%%  %s\n", 100.0 * hv[i].first / std::max<std::size_t>(1, n), hv[i].second.c_str());
    std::vector<std::pair<unsigned, std::vector<std::uint32_t>>> v;
    for (auto& [s, c] : stacks) v.push_back({c, s});
    std::sort(v.begin(), v.end(), [](auto& a, auto& b) { return a.first > b.first; });
    log("  stacks:\n");
    for (unsigned i = 0; i < std::min<std::size_t>(top / 2, v.size()); ++i) {
        std::string s;
        for (auto g : v[i].second) {
            char b[16];
            std::snprintf(b, sizeof b, " %08x", g);
            s += b;
        }
        log("    %5u%s\n", v[i].first, s.c_str());
    }
}

void sampler_reset() { taken = 0; }

void sampler_add_functions(const gc_func_entry* functions, unsigned count) {
    for (unsigned i = 0; i < count; ++i)
        translated.emplace_back(reinterpret_cast<std::uintptr_t>(functions[i].func), functions[i].start);
    std::sort(translated.begin(), translated.end());
}

void sampler_dump(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::map<void*, std::string> names;
    auto where = [&names](void* address) -> const std::string& {
        auto it = names.find(address);
        if (it != names.end()) return it->second;
        Dl_info dl{};
        char b[64];
        const auto a = reinterpret_cast<std::uintptr_t>(address);
        auto t = std::upper_bound(translated.begin(), translated.end(), std::make_pair(a, ~0u));
        if ((!dladdr(address, &dl) || !dl.dli_fname) && t != translated.begin()) {
            --t;
            std::snprintf(b, sizeof b, "gcbe:%08x+%zx", t->second, static_cast<std::size_t>(a - t->first));
            return names.emplace(address, b).first->second;
        }
        if (dladdr(address, &dl) && dl.dli_fname) {
            const char* file = std::strrchr(dl.dli_fname, '/');
            std::snprintf(b, sizeof b, "%s+%zx", file ? file + 1 : dl.dli_fname,
                          static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(address) -
                                                   reinterpret_cast<std::uintptr_t>(dl.dli_fbase)));
        } else {
            std::snprintf(b, sizeof b, "?+%zx", static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(address)));
        }
        return names.emplace(address, b).first->second;
    };
    const std::size_t n = std::min(taken.load(), kSlots);
    for (std::size_t i = 0; i < n; ++i) {
        std::fprintf(f, "%s %s", where(samples[i].pc).c_str(), where(samples[i].lr).c_str());
        for (int k = 1; k < samples[i].count; ++k) std::fprintf(f, " %s", where(samples[i].frames[k]).c_str());
        const std::uint64_t cpu = i ? samples[i].cpu_ns - samples[i - 1].cpu_ns : 0;
        std::fprintf(f, " thread:%08x cpu_us:%llu\n", samples[i].thread, static_cast<unsigned long long>(cpu / 1000));
    }
    std::fclose(f);
}

} // namespace gcrt

namespace gcrt {
namespace {
void fault(int signal, siginfo_t* info, void* context) {
    char line[160];
    auto address = reinterpret_cast<std::uintptr_t>(info->si_addr);
    auto base = reinterpret_cast<std::uintptr_t>(gc_mem);
    if (address >= base && address < base + 0x100000000ull)
        std::snprintf(line, sizeof line, "gcrt: fatal signal %d at guest address %08llx\n", signal,
                      static_cast<unsigned long long>(address - base));
    else
        std::snprintf(line, sizeof line, "gcrt: fatal signal %d at host address %p\n", signal, info->si_addr);
    write(2, line, std::strlen(line));
    if (gc_ctx* c = current_context()) {
        std::snprintf(line, sizeof line, "  last guest call: lr %08x r1 %08x r3 %08x r4 %08x r31 %08x\n", c->lr, c->r[1], c->r[3],
                      c->r[4], c->r[31]);
        write(2, line, std::strlen(line));
    }
    if (const char* d = std::getenv("GCRT_HEAP_DUMP")) {
        if (gc_ctx* c = current_context()) {
            std::uint32_t heap = read32(c->r[13] + static_cast<std::uint32_t>(std::atoi(d)));
            std::snprintf(line, sizeof line, "  r13 %08x current heap %08x\n", c->r[13], heap);
            write(2, line, std::strlen(line));
            for (int i = 0; i < 0x80; i += 16) {
                std::snprintf(line, sizeof line, "  +%02x: %08x %08x %08x %08x\n", i, read32(heap + i), read32(heap + i + 4),
                              read32(heap + i + 8), read32(heap + i + 12));
                write(2, line, std::strlen(line));
            }
        }
    }
    // Development: GCRT_FAULT_DUMP=ADDR:WORDS prints guest memory.
    if (const char* d = std::getenv("GCRT_FAULT_DUMP")) {
        std::uint32_t at = static_cast<std::uint32_t>(std::strtoul(d, nullptr, 16));
        const char* colon = std::strchr(d, ':');
        unsigned words = colon ? static_cast<unsigned>(std::atoi(colon + 1)) : 8;
        for (unsigned i = 0; i < words; ++i) {
            std::snprintf(line, sizeof line, "  %08x: %08x\n", at + i * 4, read32(at + i * 4));
            write(2, line, std::strlen(line));
        }
    }
    // Walk the interrupted code's frame-pointer chain.
    void* frames[48];
    int count = 0;
    {
        auto* uc = static_cast<ucontext_t*>(context);
#ifdef __APPLE__
        std::uintptr_t pc = uc->uc_mcontext->__ss.__pc, fp = uc->uc_mcontext->__ss.__fp;
#else
        std::uintptr_t pc = uc->uc_mcontext.pc, fp = uc->uc_mcontext.regs[29];
#endif
        frames[count++] = reinterpret_cast<void*>(pc);
        while (fp && count < 48) {
            auto* f = reinterpret_cast<std::uintptr_t*>(fp);
            if (!f[1]) break;
            frames[count++] = reinterpret_cast<void*>(f[1]);
            if (f[0] <= fp) break;
            fp = f[0];
        }
    }
    for (int i = 0; i < count; ++i) {
        std::uint32_t g = guest_function(frames[i]);
        Dl_info dl{};
        if (g) std::snprintf(line, sizeof line, "  guest %08x\n", g);
        else if (dladdr(frames[i], &dl) && dl.dli_sname) std::snprintf(line, sizeof line, "  host %s\n", dl.dli_sname);
        else std::snprintf(line, sizeof line, "  host %p\n", frames[i]);
        write(2, line, std::strlen(line));
    }
    std::signal(signal, SIG_DFL);
    raise(signal);
}
} // namespace

void install_fault_handler() {
    static char alternate[65536];
    stack_t stack{};
    stack.ss_sp = alternate;
    stack.ss_size = sizeof alternate;
    sigaltstack(&stack, nullptr);
    struct sigaction action{};
    action.sa_sigaction = fault;
    action.sa_flags = SA_SIGINFO | SA_ONSTACK;
    for (int s : {SIGSEGV, SIGBUS, SIGILL, SIGFPE}) sigaction(s, &action, nullptr);
}

} // namespace gcrt

namespace gcrt {
void log_guest_stack(const char* why) {
    void* frames[64];
    int count = backtrace(frames, 64);
    std::string s;
    for (int i = 0; i < count; ++i)
        if (std::uint32_t g = guest_function(frames[i])) {
            char b[12];
            std::snprintf(b, sizeof b, " %08x", g);
            s += b;
        }
    log("%s: guest stack%s\n", why, s.c_str());
}
} // namespace gcrt

uint32_t gc_watch_lo = 0, gc_watch_hi = 0;

extern "C" void gc_watch_hit(uint32_t ea, uint32_t size, uint64_t value) {
    static unsigned hits = 0;
    if (hits++ > 200) return;
    char why[96];
    std::snprintf(why, sizeof why, "watch: store%u %08x = %llx", size * 8, ea, static_cast<unsigned long long>(value));
    gcrt::log_guest_stack(why);
}

namespace gcrt {
void watch_dma(const char* what, std::uint32_t physical, std::uint32_t size) {
    physical &= 0x01FFFFFF;
    if (gc_watch_hi > gc_watch_lo && physical < gc_watch_hi && physical + size > gc_watch_lo) {
        char why[96];
        std::snprintf(why, sizeof why, "watch: %s %08x+%x", what, physical, size);
        log_guest_stack(why);
    }
}
} // namespace gcrt

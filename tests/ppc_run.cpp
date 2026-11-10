// Runs the recompiled instruction tests and checks their results.
#include "ppc_tests.hpp"

#include "gcrt/runtime.hpp"
#ifdef GCRT_TRANSLATED
#include "gcbe/loader.h"
#endif

#include <cstdio>
#include <cstring>

int main() {
    if (!gcrt::memory_init()) {
        std::fprintf(stderr, "cannot map guest memory\n");
        return 1;
    }
#ifdef GCRT_TRANSLATED
    char error[256];
    gcbe_program* translated = gcbe_load(GCBE_TEST_BLOB, error, sizeof error);
    if (!translated) {
        std::fprintf(stderr, "%s\n", error);
        return 1;
    }
    gcrt::cpu_init({gcbe_functions(translated), gcbe_function_count(translated), gcbe_dol_hash(translated)});
#else
    gcrt::cpu_init(gcrt::linked_program());
#endif
    auto tests = ppct::tests();
    std::uint32_t at = 0x80003100u, switch_base = 0, callee = 0;
    std::vector<std::uint32_t> addresses;
    for (const auto& t : tests) {
        addresses.push_back(at);
        if (t.name == "switch") switch_base = at;
        if (t.name == "callee") callee = at;
        at += static_cast<std::uint32_t>(t.code.size() * 4);
    }
    auto data = ppct::data_image(switch_base + 16, switch_base + 24);
    unsigned failures = 0, checks = 0;
    for (std::size_t n = 0; n < tests.size(); ++n) {
        const auto& t = tests[n];
        if (t.checks.empty()) continue;
        std::memcpy(gcrt::host(ppct::kData), data.data(), data.size());
        gc_ctx c{};
        c.mem = gc_mem;
        c.r[1] = ppct::kData + 0x8000;
        if (t.name == "call") c.r[3] = callee;
        for (auto& [r, v] : t.gpr_in) c.r[r] = v;
        for (auto& [r, v] : t.fpr_in) c.f[r] = c.ps1[r] = v;
        for (auto& [r, v] : t.gqr_in) c.gqr[r] = v;
        c.lr = 0;
        gc_call(&c, addresses[n]);
        for (const auto& k : t.checks) {
            std::uint64_t got = 0;
            switch (k.kind) {
            case ppct::Check::gpr: got = c.r[k.index]; break;
            case ppct::Check::cr:
                got = (c.cr[k.index * 4] << 3) | (c.cr[k.index * 4 + 1] << 2) | (c.cr[k.index * 4 + 2] << 1) | c.cr[k.index * 4 + 3];
                break;
            case ppct::Check::ca: got = c.ca; break;
            case ppct::Check::fpr: got = gc_d2u(c.f[k.index]); break;
            case ppct::Check::ps1: got = gc_d2u(c.ps1[k.index]); break;
            case ppct::Check::mem32: got = gcrt::read32(ppct::kData + k.index); break;
            }
            ++checks;
            if (got != k.value) {
                ++failures;
                static const char* kinds[] = {"r", "cr", "ca", "f", "ps1", "mem"};
                std::printf("FAIL %s: %s%u = %llx, expected %llx\n", t.name.c_str(), kinds[k.kind], k.index,
                            static_cast<unsigned long long>(got), static_cast<unsigned long long>(k.value));
            }
        }
    }
    std::printf("%u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}

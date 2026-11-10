// Development wrappers for `wrap` directives: observe guest calls without changing them.
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <map>
#include <vector>

using namespace gcrt;

namespace {
// Calls a guest function on a scratch register file below the caller's stack.
std::uint32_t call_guest(gc_ctx* c, std::uint32_t function, std::uint32_t r3) {
    gc_ctx tmp = *c;
    tmp.r[1] = c->r[1] - 0x400;
    tmp.r[3] = r3;
    tmp.lr = 0;
    gc_call(&tmp, function);
    return tmp.r[3];
}
} // namespace

extern "C" {

// JSystem allocation front end alloc(size, alignment): reports failures with the current
// JKRHeap (GCRT_HEAP_SDA = SDA offset of its pointer) and its free sizes (vtable +36/+40).
void wrap_TraceAlloc(gc_ctx* c, gc_func original) {
    std::uint32_t size = c->r[3], align = c->r[4];
    original(c);
    static const std::uint32_t report_size = std::getenv("GCRT_ALLOC_SIZE") ? std::strtoul(std::getenv("GCRT_ALLOC_SIZE"), nullptr, 16) : 0;
    if (c->r[3] && size != report_size) return;
    std::uint32_t result = c->r[3];
    const char* sda = std::getenv("GCRT_HEAP_SDA");
    std::uint32_t heap = sda ? read32(c->r[13] + static_cast<std::uint32_t>(std::atoi(sda))) : 0;
    log("alloc(%x, %d) -> %08x, heap %08x [%08x-%08x size %x]\n", size, static_cast<int>(align), result, heap,
        read32(heap + 0x30), read32(heap + 0x34), read32(heap + 0x38));
    if (heap) {
        std::uint32_t vtable = read32(heap);
        log("  vtable+36 -> %x, vtable+40 -> %x\n", call_guest(c, read32(vtable + 36), heap), call_guest(c, read32(vtable + 40), heap));
    }
    log_guest_stack("  failed alloc");
    if (heap) {
        // Walk every block list the heap object points at: blocks are {u16, u8 flags, u8 group,
        // u32 size, prev, next} with the data after the 16-byte header.
        std::uint32_t lo = read32(heap + 0x30), hi = read32(heap + 0x34);
        for (std::uint32_t field = 0x40; field < 0x98; field += 4) {
            std::uint32_t head = read32(heap + field);
            if (head < lo || head >= hi) continue;
            std::uint64_t total = 0;
            unsigned count = 0;
            std::vector<std::pair<std::uint32_t, std::uint32_t>> big;
            for (std::uint32_t b = head; b >= lo && b < hi && count < 4000; b = read32(b + 0xC), ++count) {
                std::uint32_t size = read32(b + 4);
                total += size;
                big.push_back({size, b});
            }
            std::sort(big.rbegin(), big.rend());
            log("  list at heap+%02x: %u blocks, %llu bytes\n", field, count, static_cast<unsigned long long>(total));
            for (std::size_t i = 0; i < std::min<std::size_t>(12, big.size()); ++i) {
                std::uint32_t b = big[i].second;
                log("    %08x size %8x group %3u data %08x %08x %08x\n", b, big[i].first, read8(b + 3), read32(b + 16),
                    read32(b + 20), read32(b + 24));
            }
        }
    }
    c->r[3] = result;
}

} // extern "C"

namespace {
// Sums a JKRExpHeap's free (+0x74) and used (+0x7C) block lists, headers included.
std::uint64_t heap_accounted(std::uint32_t heap, unsigned& blocks) {
    std::uint32_t lo = read32(heap + 0x30), hi = read32(heap + 0x34);
    std::uint64_t total = 0;
    blocks = 0;
    for (std::uint32_t field : {0x74u, 0x7Cu})
        for (std::uint32_t b = read32(heap + field); b >= lo && b < hi && blocks < 100000; b = read32(b + 0xC), ++blocks)
            total += read32(b + 4) + 16;
    return total;
}
} // namespace

extern "C" {
// Checks the JKRExpHeap passed in r3 after every call: its blocks must cover the heap.
void wrap_HeapCheck(gc_ctx* c, gc_func original) {
    std::uint32_t heap = c->r[3], r4 = c->r[4], r5 = c->r[5], lr = c->lr;
    unsigned before_blocks = 0;
    std::uint64_t before = heap_accounted(heap, before_blocks);
    original(c);
    static int reports = 0;
    unsigned blocks = 0;
    std::uint64_t after = heap_accounted(heap, blocks);
    std::uint32_t size = read32(heap + 0x38);
    // Alignment gaps leave a few bytes unaccounted; report calls that lose more than 64 KiB.
    std::int64_t lost = static_cast<std::int64_t>(before) - static_cast<std::int64_t>(after);
    if (lost > 0x10000 && reports++ < 5) {
        log("heap %08x lost %llx bytes of block coverage in a call from %08x (r4 %08x r5 %08x): %llx of %x after, %u blocks "
            "(before %u)\n", heap, static_cast<unsigned long long>(lost), lr, r4, r5, static_cast<unsigned long long>(after), size,
            blocks, before_blocks);
        log_guest_stack("  heap call");
    }
}
} // extern "C"

namespace gcrt {
// Development: called every retrace with GCRT_HEAP_MONITOR=HEAP; reports the first frame
// in which the heap's lists stop covering it, with the free list as found.
void heap_monitor() {
    static const char* text = std::getenv("GCRT_HEAP_MONITOR");
    static bool reported = false;
    if (!text || reported) return;
    std::uint32_t heap = static_cast<std::uint32_t>(std::strtoul(text, nullptr, 16));
    std::uint32_t size = read32(heap + 0x38);
    if (!size) return;
    unsigned blocks = 0;
    std::uint64_t covered = heap_accounted(heap, blocks);
    if (covered + 0x10000 >= size) return;
    reported = true;
    log("heap %08x covers %llx of %x (%u blocks)\n", heap, static_cast<unsigned long long>(covered), size, blocks);
    std::uint32_t lo = read32(heap + 0x30), hi = read32(heap + 0x34);
    for (std::uint32_t field : {0x74u, 0x7Cu}) {
        log("  list +%02x:\n", field);
        for (std::uint32_t b = read32(heap + field), n = 0; n < 64; b = read32(b + 0xC), ++n) {
            log("    %08x: %08x size %08x prev %08x next %08x\n", b, b >= lo && b < hi ? read32(b) : 0,
                b >= lo && b < hi ? read32(b + 4) : 0, b >= lo && b < hi ? read32(b + 8) : 0, b >= lo && b < hi ? read32(b + 12) : 0);
            if (b < lo || b >= hi) break;
        }
    }
}
} // namespace gcrt

namespace {
// Reference Yay0 decoder: mask words from 0x10, back-references (u16: count:4 distance:12)
// from the link table, literal bytes and long counts from the chunk table.
std::vector<std::uint8_t> yay0(std::uint32_t src, std::uint32_t& consumed_links) {
    std::uint32_t size = read32(src + 4), links = src + read32(src + 8), chunks = src + read32(src + 12);
    std::vector<std::uint8_t> out;
    out.reserve(size);
    std::uint32_t mask_at = src + 16, mask = 0;
    int bits = 0;
    while (out.size() < size) {
        if (!bits) {
            mask = read32(mask_at);
            mask_at += 4;
            bits = 32;
        }
        bool literal = mask & 0x80000000u;
        mask <<= 1;
        --bits;
        if (literal) {
            out.push_back(static_cast<std::uint8_t>(read8(chunks++)));
            continue;
        }
        std::uint32_t v = read16(links);
        links += 2;
        std::uint32_t distance = (v & 0xFFF) + 1, count = v >> 12;
        count = count ? count + 2 : read8(chunks++) + 0x12;
        for (std::uint32_t i = 0; i < count; ++i) out.push_back(distance <= out.size() ? out[out.size() - distance] : 0);
    }
    consumed_links = links;
    return out;
}
} // namespace

extern "C" void wrap_CheckYay0(gc_ctx* c, gc_func original) {
    std::uint32_t src = c->r[3], dst = c->r[4];
    std::uint32_t size = read32(src + 4);
    std::uint32_t links = 0;
    std::vector<std::uint8_t> expected = (read32(src) == 0x59617930) ? yay0(src, links) : std::vector<std::uint8_t>{};
    if (!dst) {
        original(c);
        return;
    }
    std::uint8_t past[64];
    for (int i = 0; i < 64; ++i) past[i] = static_cast<std::uint8_t>(read8(dst + size + i));
    // Mark the bytes past the end so writes of equal values show too.
    for (int i = 0; i < 64; ++i) write8(dst + size + i, 0xA5);
    if (verbosity > 1) log("yay0 call %08x (%08x) -> %08x size %x\n", src, read32(src), dst, size);
    if (const char* file = std::getenv("GCRT_YAY0_FILE")) {   // FILE:SRC compares the source with a file
        const char* colon = std::strrchr(file, ':');
        std::uint32_t want = colon ? static_cast<std::uint32_t>(std::strtoul(colon + 1, nullptr, 16)) : 0;
        if (src == want) {
            std::string path(file, colon ? static_cast<std::size_t>(colon - file) : std::strlen(file));
            if (FILE* f = std::fopen(path.c_str(), "rb")) {
                std::vector<std::uint8_t> bytes(4 << 20);
                std::size_t n = std::fread(bytes.data(), 1, bytes.size(), f);
                std::fclose(f);
                unsigned differences = 0;
                for (std::size_t i = 0; i < n; ++i)
                    if (read8(src + static_cast<std::uint32_t>(i)) != bytes[i]) {
                        if (differences++ < 8)
                            log("yay0 source %08x+%zx differs: %02x, file %02x\n", src, i, read8(src + static_cast<std::uint32_t>(i)), bytes[i]);
                    }
                log("yay0 source %08x: %u bytes differ from %s\n", src, differences, path.c_str());
            }
        }
    }
    static const bool masked = std::getenv("GCRT_YAY0_MASKED") != nullptr;
    std::uint32_t msr = gc_msr;
    if (masked) gc_msr &= ~MSR_EE;
    original(c);
    if (masked) gc_msr = (gc_msr & ~MSR_EE) | (msr & MSR_EE);
    int written = 0;
    for (int i = 0; i < 64; ++i)
        if (read8(dst + size + i) != 0xA5) written = i + 1;
    for (int i = 0; i < 64; ++i)
        if (read8(dst + size + i) == 0xA5) write8(dst + size + i, past[i]);
    if (written) log("yay0 %08x -> %08x size %x: wrote %d bytes past the end\n", src, dst, size, written);
    if (expected.empty()) return;
    std::uint32_t mismatch = ~0u;
    for (std::uint32_t i = 0; i < size; ++i)
        if (read8(dst + i) != expected[i]) {
            mismatch = i;
            break;
        }
    if (mismatch != ~0u) log("yay0 %08x -> %08x size %x: first difference at %x\n", src, dst, size, mismatch);
}

namespace {
struct Live {
    std::uint32_t size, lr;
    double at;
};
std::map<std::uint32_t, Live> live;
std::uint64_t boot = now_ns();
std::uint32_t watched_heap() {
    static std::uint32_t h = std::getenv("GCRT_HEAP_LOG") ? std::strtoul(std::getenv("GCRT_HEAP_LOG"), nullptr, 16) : 0;
    return h;
}
} // namespace

extern "C" {
// JKRExpHeap::alloc(heap, size, align) and free(heap, ptr) bookkeeping for GCRT_HEAP_LOG=HEAP:
// blocks of 64 KiB or more are logged as they come and go; the live set is dumped on demand.
void wrap_HeapAlloc(gc_ctx* c, gc_func original) {
    std::uint32_t heap = c->r[3], size = c->r[4], lr = c->lr;
    original(c);
    if (heap != watched_heap()) return;
    if (!c->r[3]) {
        std::uint64_t total = 0;
        log("heap alloc %x from %08x failed; live blocks:\n", size, lr);
        for (auto& [p, l] : live) {
            total += l.size;
            if (l.size >= 0x1000) log("  %08x %7x from %08x at %.2fs\n", p, l.size, l.lr, l.at);
        }
        log("  %zu live blocks, %llx bytes\n", live.size(), static_cast<unsigned long long>(total));
        return;
    }
    live[c->r[3]] = {size, lr, (now_ns() - boot) / 1e9};
    if (size >= 0x10000) log("heap +%08x %7x from %08x at %.2fs\n", c->r[3], size, lr, (now_ns() - boot) / 1e9);
}
void wrap_HeapFree(gc_ctx* c, gc_func original) {
    std::uint32_t heap = c->r[3], ptr = c->r[4], lr = c->lr;
    original(c);
    if (heap != watched_heap()) return;
    auto it = live.find(ptr);
    if (it == live.end()) return;
    if (it->second.size >= 0x10000) log("heap -%08x %7x from %08x at %.2fs\n", ptr, it->second.size, lr, (now_ns() - boot) / 1e9);
    live.erase(it);
}
} // extern "C"

// Runs a recompiled game without video or audio output: the GX stream is decoded but
// not drawn. For bring-up and CPU profiling on any host.
//
//   gc-headless DISC [SECONDS]
//
// Input comes from GCRT_INPUT (timed script) or GCRT_NAV (a plan steered by guest-RAM
// state, recorded with GCRT_NAV_RECORD); GCRT_WATCH and GCRT_RAM_DUMP read guest RAM. See
// nav.hpp. GCRT_SLOW_MS adds guest time per field in virtual-time runs.
#include "gcrt/gcrt.hpp"
#include "gcrt/devices.hpp"
#ifdef GCRT_TRANSLATED
#include "gcbe/loader.h"
#endif
#include "gcrt/gx_starlet.hpp"
#include "gcrt/input.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include "nav.hpp"

#include <chrono>
#include <thread>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>

namespace gcrt { void heap_monitor(); }

// GCRT_WAV=FILE writes the audio interface output as 16-bit stereo WAV (header at exit).
static std::FILE* wav = nullptr;
static unsigned wav_rate = 32000;
static std::uint64_t wav_frames = 0;

static void finish_wav() {
    if (wav) {
        auto le32 = [](std::uint8_t* p, std::uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; };
        std::uint8_t h[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 2, 0};
        std::uint32_t bytes = static_cast<std::uint32_t>(wav_frames * 4);
        le32(h + 4, 36 + bytes);
        le32(h + 24, wav_rate);
        le32(h + 28, wav_rate * 4);
        h[32] = 4; h[34] = 16;
        std::memcpy(h + 36, "data", 4);
        le32(h + 40, bytes);
        std::fseek(wav, 0, SEEK_SET);
        std::fwrite(h, 1, 44, wav);
        std::fclose(wav);
        std::printf("headless: %llu audio frames at %u Hz\n", (unsigned long long)wav_frames, wav_rate);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s DISC [SECONDS]\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, nullptr, _IOLBF, 0);
    if (std::getenv("GCRT_VIRTUAL_TIME")) gcrt::virtual_time = true;
    const double seconds = argc > 2 ? std::atof(argv[2]) : 20.0;
    if (const char* v = std::getenv("GCRT_VERBOSE")) gcrt::verbosity = std::atoi(v);
    gcrt::StarletOptions gx;
    gcrt::BootOptions options;
    options.disc = argv[1];
#ifdef GCRT_TRANSLATED
    // Translated code (gcbe): GCBE_BLOB names the image; GCBE_PERFMAP=1 writes a perf map.
    const char* blob = std::getenv("GCBE_BLOB");
    char load_error[256] = "set GCBE_BLOB to a translated code image";
    gcbe_program* translated = blob ? gcbe_load(blob, load_error, sizeof load_error) : nullptr;
    if (!translated) {
        std::fprintf(stderr, "headless: %s\n", load_error);
        return 2;
    }
    if (std::getenv("GCBE_PERFMAP")) gcbe_write_perf_map(translated);
    options.program = {gcbe_functions(translated), gcbe_function_count(translated), gcbe_dol_hash(translated)};
    gcrt::sampler_add_functions(options.program.functions, options.program.count);
#else
    options.program = gcrt::linked_program();
#endif
    if (const char* saves = std::getenv("GCRT_SAVES")) options.saves = saves;
    options.gx = gcrt::starlet_backend(nullptr, gx);
    if (const char* path = std::getenv("GCRT_WAV")) {
        wav = std::fopen(path, "wb");
        std::uint8_t header[44] = {};
        if (wav) std::fwrite(header, 1, 44, wav);
    }
    options.audio = [](const std::int16_t* samples, std::size_t frames, unsigned rate) {
        if (!wav) return;
        wav_rate = rate;
        wav_frames += frames;
        std::fwrite(samples, 4, frames, wav);
    };
    // GCRT_INPUT=SCRIPT presses buttons on port 1 (see gcrt/input.hpp).
    static gcrt::InputScript script;
    if (const char* text = std::getenv("GCRT_INPUT")) {
        std::string error;
        if (!script.parse(text, error)) {
            std::fprintf(stderr, "headless: %s\n", error.c_str());
            return 2;
        }
    }
    static gcnav::Probe probe;
    if (!probe.init()) return 2;
    // GCRT_NAV=PLAN drives port 1 from the game's state instead (see nav.hpp).
    static gcnav::Controller nav;
    static bool navigating = false;
    if (const char* plan = std::getenv("GCRT_NAV")) {
        std::string error;
        if (!nav.load(plan, error)) {
            std::fprintf(stderr, "headless: %s\n", error.c_str());
            return 2;
        }
        if (const char* rec = std::getenv("GCRT_NAV_RECORD"); rec && !nav.record(rec)) {
            std::fprintf(stderr, "headless: cannot write %s\n", rec);
            return 2;
        }
        nav.on_exit([](int status) {
            gcrt::memory_cards_flush();
            finish_wav();
            std::fflush(nullptr);
            std::_Exit(status);
        });
        navigating = true;
    }
    static std::uint64_t boot_ns = gcrt::now_ns();
    options.pads = [](int port) {
        gcrt::PadState p;
        p.connected = port == 0;
        if (port == 0) {
            const double t = (gcrt::now_ns() - boot_ns) / 1e9;
            if (navigating) nav.apply(t, p);
            else script.apply(t, p);
        }
        return p;
    };
    static std::uint64_t start = gcrt::now_ns(), last = start;
    static std::clock_t cpu_last = std::clock();
    options.retrace = [seconds](std::uint32_t xfb, std::uint64_t field) {
        gcrt::heap_monitor();
        probe.retrace((gcrt::now_ns() - boot_ns) / 1e9, field);
        // GCRT_SLOW_MS=MS (virtual time): each field costs MS more guest time, as on a
        // slower CPU, to see how scripted routes hold up when the game drops frames.
        static const double slow_ms = std::getenv("GCRT_SLOW_MS") ? std::atof(std::getenv("GCRT_SLOW_MS")) : 0;
        if (gcrt::virtual_time && slow_ms > 0) gcrt::advance_virtual_time(gcrt::now_ns() + static_cast<std::uint64_t>(slow_ms * 1e6));
        if (gcrt::virtual_time && gcrt::now_ns() - start >= static_cast<std::uint64_t>(seconds * 1e9)) {
            if (std::getenv("GCRT_SAMPLE")) gcrt::sampler_report(30);
            if (const char* dump = std::getenv("GCRT_SAMPLE_DUMP")) gcrt::sampler_dump(dump);
            gcrt::memory_cards_flush();
            finish_wav();
            std::printf("headless: done after %.1f s of guest time\n", seconds);
            std::fflush(nullptr);
            std::_Exit(0);
        }
        std::uint64_t now = gcrt::now_ns();
        if (now - last >= 1000000000ull) {
            const auto& s = gcrt::gx_stats();
            std::clock_t cpu = std::clock();
            int w = 0, h = 0;
            std::size_t stride = 0;
            gcrt::xfb_info(xfb, w, h, stride);
            std::printf("  xfb %dx%d stride %zu, ARAM size %08x\n", w, h, stride, gcrt::read32(0x800000D0));
            std::printf("[%.1fs] fields %llu xfb %08x gx %llu bytes %llu draws %llu copies %llu done, cpu %.0f%%, fibers %llu\n",
                        (now - start) / 1e9, (unsigned long long)field, xfb, (unsigned long long)s.bytes,
                        (unsigned long long)s.draws, (unsigned long long)s.display_copies, (unsigned long long)s.draw_dones,
                        100.0 * double(cpu - cpu_last) / CLOCKS_PER_SEC / ((now - last) / 1e9),
                        (unsigned long long)gcrt::fiber_count());
            last = now;
            cpu_last = cpu;
        }
    };
    if (!gcrt::virtual_time) std::thread([seconds] {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(seconds * 1000)));
        if (std::getenv("GCRT_SAMPLE")) gcrt::sampler_report(30);
        finish_wav();
        gcrt::memory_cards_flush();
        std::printf("headless: done after %.1f s\n", seconds);
        std::fflush(nullptr);
        std::_Exit(0);
    }).detach();
    if (std::getenv("GCRT_SAMPLE")) gcrt::sampler_start(1000);
    if (const char* from = std::getenv("GCRT_SAMPLE_FROM")) {
        double at = std::atof(from);
        std::thread([at] {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(at * 1000)));
            gcrt::sampler_reset();
        }).detach();
    }
    std::string error;
    gcrt::boot(options, error);
    std::fprintf(stderr, "headless: %s\n", error.c_str());
    return 1;
}

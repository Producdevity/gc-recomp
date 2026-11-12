#include "gcrt/frontend.hpp"

#include "gcrt/devices.hpp"
#include "gcrt/gcrt.hpp"
#include "gcrt/gx_starlet.hpp"
#include "gcrt/input.hpp"
#include "gcrt/runtime.hpp"
#include "gcrt/sampler.hpp"

#include <SDL.h>
#include <GLES3/gl31.h>
#include <starlet/disc/image.hpp>
#include <starlet/gx/gles/renderer.hpp>
#include <starlet/host/audio_output.hpp>
#include <starlet/host/controllers.hpp>
#include <starlet/host/pad.hpp>
#include <starlet/host/quit.hpp>
#ifdef GCRT_WITH_GCBE
#include "../../tools/nav.hpp"
#include "a64_translate.hpp"
#include "gcbe/loader.h"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <cstring>
#include <functional>
#include <ctime>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string>
#include <sys/resource.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace gcrt {
namespace {

namespace sg = starlet::gx;

std::uint64_t thread_cpu_ns() {
    timespec t;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
}
// Starlet's GL submission thread, recorded when it binds the context.
std::atomic<bool> gl_known{false};
pthread_t gl_pthread;
clockid_t gl_clock;
std::uint64_t gl_cpu_ns() {
    if (!gl_known) return 0;
    timespec t;
    clock_gettime(gl_clock, &t);
    return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
}
std::uint64_t process_cpu_ns() {
    timespec t;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
}

// Time the guest thread spends decoding and drawing the GX stream.
// CPU time spent in the GX backend, on whichever thread runs it.
std::atomic<std::uint64_t> gx_cpu_ns{0};

// The frontend ends a port with SIGTERM, and gptokeyb's exit chord follows it with SIGKILL after
// 3 s. The guest thread exits at its next retrace, or Starlet's quit thread does when the
// guest has not within 500 ms; either saves the memory cards first.
void handle_quit() {
    namespace host = starlet::host;
    host::install_quit_signals();
    host::on_quit([] {
        const int signal = host::quit_signal();
        const std::uint64_t start = now_ns();
        memory_cards_flush();
        log("quit: %s; memory cards saved in %.1f ms\n",
            signal == SIGTERM ? "SIGTERM" : signal == SIGINT ? "SIGINT" : host::quit_requested() ? "quit requested" : "exit",
            (now_ns() - start) / 1e6);
    });
    host::exit_after_quit(std::chrono::milliseconds(500));
}

// SDL's events on the thread that owns them; exits once a quit is requested.
void poll_quit() {
    starlet::host::poll_events();
    if (starlet::host::quit_requested()) starlet::host::exit_now(0);
}

void write_ppm(const std::string& path, const std::vector<std::uint8_t>& rgba, int width, int height);

class StarletRenderer final : public GxRenderer {
public:
    explicit StarletRenderer(sg::GlesRenderer& r, int efb_height) : r_(r), efb_height_(efb_height) {}
    // Development: GCRT_EFB_DUMP=DIR writes the EFB before every 120th display copy.
    std::string efb_dump;
    unsigned copies = 0;
    void draw(const sg::State& state, const sg::Draw& d, sg::MemoryReader& memory) override {
        try {
            r_.draw_direct(state, d, memory, sg::TextureCmprModel::gx_candidate_v1);
        } catch (const std::exception& e) {
            // A draw Starlet rejects before submission is skipped; the first of each kind
            // is logged with its state. A failed GL context ends the run.
            static const bool strict = std::getenv("GCRT_GX_STRICT") != nullptr;
            const bool fatal_error = r_.failed();
            if (rejected_[e.what()]++ && !strict && !fatal_error) return;
            log("draw failed: %s\n", e.what());
            log("  xf channels %u: %08x %08x %08x %08x, texgens %u, normal mode %u, matrix %u\n", state.xf(0x1009),
                state.xf(0x100e), state.xf(0x100f), state.xf(0x1010), state.xf(0x1011), state.xf(0x103f),
                unsigned(d.layout.normal.mode), state.matrix_index(0));
            for (unsigned l = 0; l < 8; ++l)
                log("  light %u: color %08x pos %08x %08x %08x\n", l, state.xf(static_cast<std::uint16_t>(0x603 + l * 16)),
                    state.xf(static_cast<std::uint16_t>(0x60a + l * 16)), state.xf(static_cast<std::uint16_t>(0x60b + l * 16)),
                    state.xf(static_cast<std::uint16_t>(0x60c + l * 16)));
            for (unsigned map = 0; map < 8; ++map) {
                unsigned base = (map < 4 ? 0x80 : 0xa0) + map % 4;
                log("  map %u: mode0 %06x mode1 %06x image0 %06x even %06x odd %06x image3 %06x tlut %06x\n", map,
                    state.bp(static_cast<std::uint8_t>(base)), state.bp(static_cast<std::uint8_t>(base + 4)),
                    state.bp(static_cast<std::uint8_t>(base + 8)), state.bp(static_cast<std::uint8_t>(base + 12)),
                    state.bp(static_cast<std::uint8_t>(base + 16)), state.bp(static_cast<std::uint8_t>(base + 20)),
                    state.bp(static_cast<std::uint8_t>(base + 24)));
            }
            if (strict || fatal_error) throw;
        }
    }
    void display_copy(const sg::DisplayCopy& plan, std::uint32_t xfb) override {
        if (!efb_dump.empty() && ++copies % 120 == 0) {
            char name[64];
            std::snprintf(name, sizeof name, "/efb-%05u.ppm", copies);
            write_ppm(efb_dump + name, r_.readback(), 640, efb_height_);
            log("efb dump: copy %d,%d %dx%d stride %zu\n", plan.x, plan.y, plan.width, plan.height, plan.stride);
        }
        r_.copy_display(plan, phys(xfb), kRamSize - (xfb & 0x01FFFFFF));
    }
    void texture_copy(const sg::TextureCopy& plan, std::uint32_t address) override {
        static const bool sync = std::getenv("GCRT_COPY_SYNC") != nullptr;
        if (sync) r_.copy_texture(plan, phys(address), kRamSize - (address & 0x01FFFFFF));
        else r_.copy_texture_resident(plan, phys(address), kRamSize - (address & 0x01FFFFFF), address);
    }
    std::uint32_t peek(bool depth, int x, int y) override { return depth ? r_.peek_depth(x, y) : r_.peek_color(x, y); }
    int snapshot_efb() override { return r_.snapshot_efb(); }
    int snapshot_efb(int x, int y, int width, int height) override {
        return r_.snapshot_efb(starlet::gx::EfbRegion{x, y, width, height});
    }
    void release_snapshot(int index) override { r_.release_snapshot(index); }
    bool snapshot_ready(int index) const override { return r_.snapshot_ready(index); }
    bool peek_snapshot(int index, bool depth, int x, int y, std::uint32_t& value) const override {
        return depth ? r_.peek_depth_snapshot(index, x, y, value) : r_.peek_color_snapshot(index, x, y, value);
    }
    std::uint32_t peek_snapshot_wait(int index, bool depth, int x, int y) override {
        r_.select_snapshot(index);
        try {
            const std::uint32_t value = peek(depth, x, y);
            r_.select_snapshot(-1);
            return value;
        } catch (...) {
            r_.select_snapshot(-1);
            throw;
        }
    }
    void wait_snapshot(int index) override { r_.wait_snapshot(index); }

private:
    sg::GlesRenderer& r_;
    int efb_height_;
    std::map<std::string, unsigned> rejected_;
};

// Measures the time spent in the GX backend; on_attach runs on the GX thread first.
class TimedBackend final : public GxBackend {
public:
    explicit TimedBackend(GxBackend* inner) : inner_(inner) {}
    std::function<void()> on_attach;
    void attach() override {
        if (on_attach) on_attach();
    }
    void feed(const std::uint8_t* bytes, std::size_t size) override {
        std::uint64_t t = thread_cpu_ns();
        inner_->feed(bytes, size);
        gx_cpu_ns += thread_cpu_ns() - t;
    }
    std::uint32_t peek(bool depth, int x, int y) override { return inner_->peek(depth, x, y); }
    void poke(bool depth, int x, int y, std::uint32_t v) override { inner_->poke(depth, x, y, v); }
    int snapshot() override { return inner_->snapshot(); }
    int snapshot(int x, int y, int width, int height) override { return inner_->snapshot(x, y, width, height); }
    void release_snapshot(int index) override { inner_->release_snapshot(index); }
    bool snapshot_ready(int index) const override { return inner_->snapshot_ready(index); }
    bool peek_snapshot(int index, bool depth, int x, int y, std::uint32_t& value) const override {
        return inner_->peek_snapshot(index, depth, x, y, value);
    }
    std::uint32_t peek_snapshot_wait(int index, bool depth, int x, int y) override {
        return inner_->peek_snapshot_wait(index, depth, x, y);
    }
    void wait_snapshot(int index) override { inner_->wait_snapshot(index); }

private:
    GxBackend* inner_;
};

struct Display {
    SDL_Window* window = nullptr;
    SDL_GLContext gl = nullptr;
    Display(const char* title, bool windowed) {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_GAMECONTROLLER | SDL_INIT_AUDIO) != 0)
            fatal("SDL_Init: %s", SDL_GetError());
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        Uint32 flags = SDL_WINDOW_OPENGL | (windowed ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
        window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowed ? 640 : 0,
                                  windowed ? 480 : 0, flags);
        if (window) gl = SDL_GL_CreateContext(window);
        if (!gl) fatal("cannot create an OpenGL ES 3.1 context: %s", SDL_GetError());
        if (SDL_GL_SetSwapInterval(1) != 0) log("Display: no vsync (%s)\n", SDL_GetError());
    }
};

// Starlet's push output: a write that does not fit its 120 ms ring is dropped, which keeps
// latency bounded when the game runs ahead.
struct Audio {
    starlet::host::AudioOutput output;
    bool reported = false;
    void push(const std::int16_t* samples, std::size_t frames, unsigned rate) {
        if (!output.is_open() || output.sample_rate() != rate) {
            output.close();
            output.open_push(rate);
            if (const std::string error = output.device_error(); !error.empty() && !reported) {
                log("Audio: %s\n", error.c_str());
                reported = true;
            }
        }
        output.write(samples, frames);
    }
};

struct Pads {
    InputScript script;
    // --nav PLAN: port 0 follows a route plan steered by guest RAM (tools/nav.hpp).
    gcnav::Controller nav;
    bool navigating = false;
    std::uint64_t start = now_ns();

    // GameCube places on the handheld: A south, B west, X east, Y north; L the left shoulder or
    // trigger, R the right trigger, Z the right shoulder; axial stick dead zone of 12%.
    static starlet::host::GameCubeMapping mapping() {
        using starlet::host::bit;
        using starlet::host::Input;
        starlet::host::GameCubeMapping m;
        m.a = bit(Input::south);
        m.b = bit(Input::west);
        m.x = bit(Input::east);
        m.y = bit(Input::north);
        m.start = bit(Input::start);
        m.z = bit(Input::right_shoulder);
        m.up = bit(Input::dpad_up);
        m.down = bit(Input::dpad_down);
        m.left = bit(Input::dpad_left);
        m.right = bit(Input::dpad_right);
        m.l = bit(Input::left_shoulder) | bit(Input::left_trigger);
        m.r = bit(Input::right_trigger);
        m.trigger_threshold = 24000;
        m.trigger_left = {true, true};
        m.trigger_right = {true, false};
        m.main_stick = {starlet::host::StickResponse::Shape::axial, 3932, false, 32768.f, false, 100};
        m.c_stick = m.main_stick;
        return m;
    }
    std::unique_ptr<starlet::host::Controllers> controllers;
    // SDL's controller state changes only when events are pumped (once per field, in the
    // retrace handler), so the port 0 state is sampled there instead of on every SI poll.
    PadState sampled;
    bool have_sample = false;
    void update() {
        sampled = sample();
        have_sample = true;
    }
    PadState read(int port) {
        PadState p;
        p.connected = port == 0;
        if (port != 0) return p;
        if (!have_sample) update();
        p = sampled;
        if (navigating) nav.apply((now_ns() - start) / 1e9, p);
        else if (script.active()) script.apply((now_ns() - start) / 1e9, p);
        return p;
    }
    PadState sample() {
        PadState p;
        p.connected = true;
        // After SDL_Init, on the thread that pumps SDL's events.
        if (!controllers) controllers = std::make_unique<starlet::host::Controllers>(1, [](const char* line) { log("%s\n", line); });
        controllers->update();
        static const starlet::host::GameCubeMapping layout = mapping();
        const auto pad = starlet::host::to_gamecube(controllers->state(0), layout);
        std::uint16_t b = pad.buttons;
        p.stick_x = static_cast<std::uint8_t>(128 + pad.stick_x);
        p.stick_y = static_cast<std::uint8_t>(128 + pad.stick_y);
        p.cstick_x = static_cast<std::uint8_t>(128 + pad.substick_x);
        p.cstick_y = static_cast<std::uint8_t>(128 + pad.substick_y);
        p.trigger_l = pad.trigger_left;
        p.trigger_r = pad.trigger_right;
        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        if (keys[SDL_SCANCODE_RETURN]) b |= PAD_START;
        if (keys[SDL_SCANCODE_Z]) b |= PAD_A;
        if (keys[SDL_SCANCODE_X]) b |= PAD_B;
        if (keys[SDL_SCANCODE_C]) b |= PAD_X;
        if (keys[SDL_SCANCODE_V]) b |= PAD_Y;
        if (keys[SDL_SCANCODE_D]) b |= PAD_Z;
        if (keys[SDL_SCANCODE_LEFT]) p.stick_x = 28;
        if (keys[SDL_SCANCODE_RIGHT]) p.stick_x = 228;
        if (keys[SDL_SCANCODE_UP]) p.stick_y = 228;
        if (keys[SDL_SCANCODE_DOWN]) p.stick_y = 28;
        p.buttons = b;
        return p;
    }
};

void write_ppm(const std::string& path, const std::vector<std::uint8_t>& rgba, int width, int height) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", width, height);
    std::vector<std::uint8_t> row(static_cast<std::size_t>(width) * 3);
    for (int y = height - 1; y >= 0; --y) {   // readback rows run bottom to top
        const std::uint8_t* src = rgba.data() + static_cast<std::size_t>(y) * width * 4;
        for (int x = 0; x < width; ++x) {
            row[x * 3] = src[x * 4];
            row[x * 3 + 1] = src[x * 4 + 1];
            row[x * 3 + 2] = src[x * 4 + 2];
        }
        std::fwrite(row.data(), 1, row.size(), f);
    }
    std::fclose(f);
}

#ifdef GCRT_WITH_GCBE
std::uint32_t be32(const std::uint8_t* p) {
    return std::uint32_t(p[0]) << 24 | std::uint32_t(p[1]) << 16 | std::uint32_t(p[2]) << 8 | p[3];
}

// A bar across the middle of the screen, filled to the given fraction.
void draw_progress(SDL_Window* window, double fraction) {
    int w = 0, h = 0;
    SDL_GL_GetDrawableSize(window, &w, &h);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.02f, 0.02f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    const int bw = w * 3 / 5, bh = std::max(8, h / 40), x = (w - bw) / 2, y = (h - bh) / 2;
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, y, bw, bh);
    glClearColor(0.15f, 0.15f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glScissor(x, y, static_cast<int>(bw * std::clamp(fraction, 0.0, 1.0)), bh);
    glClearColor(0.3f, 0.75f, 0.35f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    SDL_GL_SwapWindow(window);
}

// The program list shipped beside the executable (the port folder), or empty.
std::string shipped_program_list() {
    char path[4096];
    const ssize_t size = readlink("/proc/self/exe", path, sizeof path - 1);
    if (size <= 0) return {};
    const std::string exe(path, static_cast<std::size_t>(size));
    const std::string list = exe.substr(0, exe.rfind('/') + 1) + "programs.list";
    return access(list.c_str(), R_OK) == 0 ? list : std::string();
}

// Builds the listed programs the cache lacks before the game starts, behind a progress
// bar (first launch, new lists, driver updates): beside the running game a Mali link
// stalls the GL thread, and draws of programs not built yet are skipped. Programs
// missing from every list then build in the background.
void prepare_programs(sg::GlesRenderer& renderer, SDL_Window* window) {
    auto s = renderer.program_builder();
    while (s.running && !s.planned) {
        SDL_Delay(10);
        s = renderer.program_builder();
    }
    if (s.running && s.to_build) {
        const std::uint64_t start = now_ns();
        log("Preparing %zu shaders\n", s.to_build);
        for (; s.running && !s.lists_done; s = renderer.program_builder()) {
            poll_quit();
            draw_progress(window, double(s.finished) / double(s.to_build));
            SDL_Delay(30);
        }
        log("Prepared %zu shaders in %.1f s (%zu failed)\n", s.built, (now_ns() - start) / 1e9, s.failed);
    }
    renderer.build_programs_in_background();
}

// The game's code from the player's disc: cache/<ID>.gcbe while it was made from this
// disc's executable by this translator, otherwise translated now with a progress bar.
bool translated_program(SDL_Window* window, const GameInfo& game, const std::string& disc_path,
                        const std::string& cache, Program& program, std::string& error) {
    std::array<std::uint8_t, 0x440> header;
    std::vector<std::uint8_t> dol;
    try {
        auto image = starlet::disc::open_disc_image(disc_path);
        starlet::disc::GameVolume volume(*image);
        header = volume.boot();
        dol = volume.main_dol();
    } catch (const starlet::disc::DiscError& e) {
        error = e.what();
        return false;
    }
    std::uint8_t sha1[20];
    gcbe::sha1(dol.data(), dol.size(), sha1);
    const std::string path = cache + "/" + std::string(reinterpret_cast<const char*>(header.data()), 6) + ".gcbe";
    if (!gcbe_cache_valid(path.c_str(), sha1, 0)) {
        gcbe::Request request;
        request.dol_bytes = std::move(dol);
        request.symbols = game.symbols;
        if (game.config) request.config = game.config;
        request.out = path;
        std::atomic<std::size_t> done{0}, total{1};
        std::atomic<bool> finished{false};
        bool ok = false;
        std::string failure;
        gcbe::Stats stats;
        const std::uint64_t start = now_ns();
        std::thread worker([&] {
            ok = gcbe::translate_files(request, stats, [&](std::size_t d, std::size_t t) { done = d; total = t; }, failure);
            finished = true;
        });
        while (!finished) {
            poll_quit();
            draw_progress(window, double(done) / double(std::max<std::size_t>(1, total)));
            SDL_Delay(16);
        }
        worker.join();
        if (!ok) {
            error = "cannot translate the game's code: " + failure;
            return false;
        }
        rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        log("%s: translated %zu functions (%zu KiB of code) in %.1f s, peak RSS %ld MiB\n", game.title, stats.functions,
            stats.code_bytes / 1024, (now_ns() - start) / 1e9, usage.ru_maxrss / 1024);
    }
    char message[256];
    gcbe_program* loaded = gcbe_load(path.c_str(), message, sizeof message);
    if (!loaded) {
        error = message;
        return false;
    }
    if (std::getenv("GCBE_PERFMAP")) gcbe_write_perf_map(loaded);
    program = {gcbe_functions(loaded), gcbe_function_count(loaded), gcbe_dol_hash(loaded)};
    sampler_add_functions(program.functions, program.count);
    return true;
}
#endif

} // namespace

int run_frontend(const GameInfo& game, int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s DISC SAVES SHADER_CACHE [--seconds S] [--capture DIR] [--capture-every N] "
                             "[--windowed] [--input SCRIPT] [--nav PLAN [--nav-record FILE]] [--profile]\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, nullptr, _IOLBF, 0);
    handle_quit();
    std::string disc = argv[1], saves = argv[2], cache = argv[3], capture;
    double seconds = 0;
    unsigned capture_every = 60;
    bool windowed = false, profile = false;
    double profile_from = 0;   // --profile-from: sample only after this many seconds
    bool profile_gx = false;   // --profile-gx: sample the GX thread instead of the guest
    bool profile_gl = false;   // --profile-gl: sample Starlet's GL submission thread
    bool no_draw = false;      // --no-draw: decode GX without drawing (measurement)
    Pads pads;
    for (int n = 4; n < argc; ++n) {
        std::string a = argv[n];
        auto value = [&]() -> std::string { return n + 1 < argc ? argv[++n] : ""; };
        if (a == "--seconds") seconds = std::atof(value().c_str());
        else if (a == "--capture") capture = value();
        else if (a == "--capture-every") capture_every = static_cast<unsigned>(std::atoi(value().c_str()));
        else if (a == "--windowed") windowed = true;
        else if (a == "--profile") profile = true;
        else if (a == "--profile-from") { profile = true; profile_from = std::atof(value().c_str()); }
        else if (a == "--profile-gx") { profile = true; profile_gx = true; }
        else if (a == "--profile-gl") { profile = true; profile_gl = true; }
        else if (a == "--input") {
            std::string error;
            if (!pads.script.parse(value(), error)) fatal("%s", error.c_str());
        } else if (a == "--env") {
            // Development: sets NAME=VALUE in the environment (device launchers pass arguments only).
            const std::string kv = value();
            const auto eq = kv.find('=');
            if (eq == std::string::npos) fatal("--env needs NAME=VALUE");
            setenv(kv.substr(0, eq).c_str(), kv.substr(eq + 1).c_str(), 1);
        } else if (a == "--no-draw") {
            no_draw = true;
        } else if (a == "--nav") {
            std::string error;
            if (!pads.nav.load(value(), error)) fatal("%s", error.c_str());
            pads.navigating = true;
        } else if (a == "--nav-record") {
            const std::string path = value();
            if (!pads.nav.record(path)) fatal("cannot write %s", path.c_str());
        } else fatal("unknown option %s", a.c_str());
    }
    if (const char* v = std::getenv("GCRT_VERBOSE")) verbosity = std::atoi(v);
    if (pads.navigating)
        pads.nav.on_exit([profile, capture](int status) {
            log("nav: plan ended with status %d\n", status);
            if (profile) {
                sampler_report(40);
                if (!capture.empty()) sampler_dump(capture + "/samples.txt");
            }
            starlet::host::exit_now(status);
        });
    mkdir(saves.c_str(), 0755);
    mkdir(cache.c_str(), 0755);

    Display display(game.title, windowed);
    log("%s: GL renderer %s, %s\n", game.title, glGetString(GL_RENDERER), glGetString(GL_VERSION));
    int width = 0, height = 0;
    SDL_GL_GetDrawableSize(display.window, &width, &height);
    log("Display: %dx%d\n", width, height);
    // The game's code: compiled in, or translated from the disc before the renderer takes
    // the GL context to its submission thread.
    Program program = game.program;
#ifdef GCRT_WITH_GCBE
    if (!program.functions && game.symbols) {
        std::string error;
        if (!translated_program(display.window, game, disc, cache, program, error)) {
            log("%s: %s\n", game.title, error.c_str());
            return 1;
        }
    }
#endif
    if (!program.functions) {
        log("%s: no game code\n", game.title);
        return 1;
    }

    sg::GlesLimits limits;
    limits.efb.height = game.efb_height;
    limits.direct.shading = std::getenv("GCRT_SHADING_EXACT") ? sg::DirectShading::exact : sg::DirectShading::fast;
    // Recompiled code writes guest RAM without telling the renderer.
    limits.direct.stamp_resident_copies = true;
    // Token handlers test single pixels (Sunshine: Mario's visibility marker, the sun's depth).
    limits.direct.snapshot_scale = 1;
    limits.direct.program_cache = cache;
    limits.direct.program_lists = {cache + "/programs.list"};
    // The port's list of the game's programs, prebuilt at first launch.
    if (const std::string shipped = shipped_program_list(); !shipped.empty()) limits.direct.program_lists.push_back(shipped);
    static sg::GlesRenderer renderer(limits);
    // Captures read the presented image back before it is swapped out, on the GL thread.
    static std::mutex capture_mutex;
    static std::string capture_path;
    renderer.set_swap([&display] {
        std::string path;
        {
            std::lock_guard<std::mutex> lock(capture_mutex);
            path.swap(capture_path);
        }
        if (!path.empty()) {
            int w = 0, h = 0;
            SDL_GL_GetDrawableSize(display.window, &w, &h);
            std::vector<std::uint8_t> rgba(static_cast<std::size_t>(w) * h * 4);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            glPixelStorei(GL_PACK_ROW_LENGTH, 0);
            glPixelStorei(GL_PACK_SKIP_ROWS, 0);
            glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
            glPixelStorei(GL_PACK_ALIGNMENT, 4);
            glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
            write_ppm(path, rgba, w, h);
        }
        SDL_GL_SwapWindow(display.window);
    });
    prepare_programs(renderer, display.window);
    static StarletRenderer adapter(renderer, game.efb_height);
    if (const char* d = std::getenv("GCRT_EFB_DUMP")) adapter.efb_dump = d;
    StarletOptions gx;
    gx.efb_height = game.efb_height;
    gx.merge_primitives = true;
    gx.skip_draws = no_draw;
    static TimedBackend backend(starlet_backend(&adapter, gx));
    // Decoding and recording run on the GX thread (GCRT_GX_INLINE=1: on the guest thread);
    // GL calls and swaps run on Starlet's submission thread (GCRT_GL_INLINE=1: inline).
    const bool gx_thread = !std::getenv("GCRT_GX_INLINE");
    auto submission = [&display] {
        if (std::getenv("GCRT_GL_INLINE")) return;
        renderer.start_submission([] {}, [&display] {
                                      SDL_GL_MakeCurrent(display.window, display.gl);
                                      if (!gl_known) {
                                          gl_pthread = pthread_self();
                                          pthread_getcpuclockid(gl_pthread, &gl_clock);
                                          gl_known = true;
                                      }
                                  },
                                  [&display] { SDL_GL_MakeCurrent(display.window, nullptr); });
    };
    if (gx_thread) {
        SDL_GL_MakeCurrent(display.window, nullptr);
        backend.on_attach = [submission, &display] {
            if (std::getenv("GCRT_GL_INLINE")) SDL_GL_MakeCurrent(display.window, display.gl);
            renderer.adopt_thread();
            submission();
        };
    } else {
        if (!std::getenv("GCRT_GL_INLINE")) SDL_GL_MakeCurrent(display.window, nullptr);
        submission();
    }

    static Audio audio;
    BootOptions options;
    options.disc = disc;
    options.program = program;
    options.saves = saves;
    options.gx = &backend;
    options.gx_thread = gx_thread;
    options.audio = [](const std::int16_t* s, std::size_t frames, unsigned rate) { audio.push(s, frames, rate); };
    options.pads = [&pads](int port) { return pads.read(port); };

    struct Stats {
        std::uint64_t wall = now_ns(), cpu = thread_cpu_ns(), gx = 0, copies = 0, fields = 0, process = process_cpu_ns();
        IdleTime idle = idle_time();
        std::uint64_t gl = gl_cpu_ns();
        sg::GlesStats gles;
        std::uint64_t faults = 0;
        std::uint64_t start = now_ns();
        unsigned captures = 0;
    };
    static Stats stats;
    static bool sampling = false;
    options.retrace = [&, seconds, capture, capture_every, profile, profile_from, profile_gx, profile_gl](std::uint32_t xfb,
                                                                                                           std::uint64_t field) {
        if (profile && !sampling && now_ns() - stats.start >= static_cast<std::uint64_t>(profile_from * 1e9)) {
            // The sampler follows the thread that starts it.
            if (profile_gl && gl_known) sampler_start_thread(gl_pthread, 1000);
            else if (profile_gx && gx_thread) gx_post([] { sampler_start(1000); });
            else sampler_start(1000);
            sampling = true;
        }
        poll_quit();
        pads.update();
        gx_flush();
        if (!capture.empty() && field % capture_every == 0) {
            char name[64];
            std::snprintf(name, sizeof name, "/frame-%05llu.ppm", static_cast<unsigned long long>(field));
            std::lock_guard<std::mutex> lock(capture_mutex);
            capture_path = capture + name;
            ++stats.captures;
        }
        int dw = 0, dh = 0;
        SDL_GL_GetDrawableSize(display.window, &dw, &dh);
        // Presentation follows the commands queued so far, on the renderer's thread.
        gx_post([xfb, dw, dh] {
            std::uint64_t t = thread_cpu_ns();
            int w = 0, h = 0;
            std::size_t stride = 0;
            if (xfb_info(xfb, w, h, stride)) {
                sg::FramebufferView view{{phys(xfb), (static_cast<std::size_t>(h) - 1) * stride + static_cast<std::size_t>(w) * 2},
                                         static_cast<unsigned>(w), static_cast<unsigned>(h), stride};
                renderer.present_framebuffer(view, dw, dh);
            } else {
                renderer.present_framebuffer({}, dw, dh, true);
            }
            gx_cpu_ns += thread_cpu_ns() - t;
        });
        // Frame pacing: fields between new display copies, as the screen shows them.
        static std::uint64_t paced_copies = 0, paced_field = 0;
        static unsigned slowest = 0, late = 0, paced = 0;
        if (const std::uint64_t copies = gx_stats().display_copies; copies != paced_copies) {
            if (paced_field) {
                const unsigned span = static_cast<unsigned>(field - paced_field);
                slowest = std::max(slowest, span);
                late += span > 2;
                ++paced;
            }
            paced_copies = copies;
            paced_field = field;
        }
        std::uint64_t now = now_ns();
        if (now - stats.wall >= 5000000000ull) {
            if (paced) log("pacing: slowest frame %u fields (%.1f fps), %u of %u frames over 2 fields\n", slowest,
                           60.0 / std::max(1u, slowest), late, paced);
            slowest = late = paced = 0;
            const auto& s = gx_stats();
            std::uint64_t cpu = thread_cpu_ns();
            double secs = (now - stats.wall) / 1e9;
            double frames = static_cast<double>(s.display_copies - stats.copies);
            const std::uint64_t gx_now = gx_cpu_ns;
            double gx_ms = (gx_now - stats.gx) / 1e6;
            double guest_ms = (cpu - stats.cpu) / 1e6 - (gx_thread ? 0.0 : gx_ms);
            const std::uint64_t process = process_cpu_ns();
            const double process_ms = (process - stats.process) / 1e6;
            const IdleTime idle = idle_time();
            const double idle_ms = (idle.total_ns - stats.idle.total_ns) / 1e6, idle_gx_ms = (idle.gx_busy_ns - stats.idle.gx_busy_ns) / 1e6;
            const std::uint64_t gl = gl_cpu_ns();
            const sg::GlesStats r = renderer.stats();
            const double f = frames ? frames : 1.0;
            log("perf: %.1f fps, %.1f fields/s; per frame: guest thread %.2f ms, GX %.2f ms (%s), GL thread %.2f ms, guest idle "
                "%.2f ms (%.2f while GX busy); process %.2f ms; draws %llu\n",
                frames / secs, (field - stats.fields) / secs, frames ? guest_ms / frames : 0.0, frames ? gx_ms / frames : 0.0,
                gx_thread ? "own thread" : "guest thread", (gl - stats.gl) / 1e6 / f, frames ? idle_ms / frames : 0.0,
                frames ? idle_gx_ms / frames : 0.0, frames ? process_ms / frames : 0.0, static_cast<unsigned long long>(s.draws));
            const auto& d = r.direct;
            const auto& o = stats.gles.direct;
            log("gl: per frame %.0f draws (%.0f batched, %.0f culled) in %.0f GL draw calls, %.1f segments, %.0f KB vertices, %.0f uniform blocks, "
                "%.1f texture uploads, %.1f texture copies (%.1f invalidations), %.1f EFB peeks "
                "(%.1f live, %.1f waited), %.1f shader compiles, %.1f format conversions; geometry %.0f hits, %.0f misses, %.0f KB compared; "
                "%.1f RAM write faults\n",
                (d.draws - o.draws) / f, (d.batched_draws - o.batched_draws) / f, (d.culled_draws - o.culled_draws) / f,
                (d.submissions - o.submissions) / f, (d.segments - o.segments) / f, (d.vertex_bytes - o.vertex_bytes) / 1024.0 / f, (d.uniform_blocks - o.uniform_blocks) / f,
                (d.texture_uploads - o.texture_uploads) / f, (r.texture_copies - stats.gles.texture_copies) / f,
                (r.invalidations - stats.gles.invalidations) / f,
                (r.peeks - stats.gles.peeks) / f, (r.live_peeks - stats.gles.live_peeks) / f,
                (r.peek_waits - stats.gles.peek_waits) / f, (r.shader_compiles - stats.gles.shader_compiles) / f,
                (r.format_conversions - stats.gles.format_conversions) / f, (r.direct.geometry_hits - stats.gles.direct.geometry_hits) / f,
                (r.direct.geometry_misses - stats.gles.direct.geometry_misses) / f,
                (r.direct.geometry_validated_bytes - stats.gles.direct.geometry_validated_bytes) / 1024.0 / f,
                (ram_tracking_faults() - stats.faults) / f);
            if (const auto misses = r.region_misses - stats.gles.region_misses)
                log("snapshots: %.2f reads per frame outside the read-back region\n", misses / f);
            if (const auto unready = r.direct.unready_draws - stats.gles.direct.unready_draws)
                log("unready: %llu draws skipped while their program built\n", static_cast<unsigned long long>(unready));
            if (const auto links = r.direct.fallback_links - stats.gles.direct.fallback_links)
                log("programs: %llu linked on the GL thread after their draws waited (longest link %llu ms)\n",
                    static_cast<unsigned long long>(links), static_cast<unsigned long long>(r.direct.fallback_link_max_ms));
            static GxSyncStats last_sync;
            const GxSyncStats sync = gx_sync_stats();
            if (sync.snapshot_tokens != last_sync.snapshot_tokens)
                log("sync: per frame %.1f snapshot tokens (%.1f forced, %.1f late), %.1f snapshot peeks (%.1f waited)\n",
                    (sync.snapshot_tokens - last_sync.snapshot_tokens) / f, (sync.forced_tokens - last_sync.forced_tokens) / f,
                    (sync.late_tokens - last_sync.late_tokens) / f,
                    (sync.snapshot_peeks - last_sync.snapshot_peeks) / f, (sync.waited_peeks - last_sync.waited_peeks) / f);
            last_sync = sync;
            stats = {now, cpu, gx_now, s.display_copies, field, process, idle, gl, r, ram_tracking_faults(), stats.start, stats.captures};
        }
        if (seconds > 0 && now - stats.start >= static_cast<std::uint64_t>(seconds * 1e9)) {
            if (profile) {
                sampler_report(40);
                if (!capture.empty()) sampler_dump(capture + "/samples.txt");
            }
            log("%s: stopping after %.0f s\n", game.title, seconds);
            starlet::host::exit_now(0);
        }
    };
    std::string error;
    // Input scripts and route plans count from the boot, after translation and shader preparation.
    pads.start = now_ns();
    boot(options, error);
    log("%s: %s\n", game.title, error.c_str());
    return 1;
}

} // namespace gcrt

// SDL2 + OpenGL ES 3.1 frontend shared by the recompiled ports.
#pragma once

#include "gcrt/program.hpp"

#include <cstdint>

namespace gcrt {

struct GameInfo {
    const char* title;      // window title and log name
    int efb_height;         // the game's EFB frame height (render mode efbHeight)
    Program program;        // the game's code (linked_program() in compiled builds)
    // Without program: the function list and configuration the frontend translates the
    // disc's executable with at first launch (paths relative to the working directory).
    const char* symbols = nullptr;
    const char* config = nullptr;
    // Game thread, at every retrace after input is polled; guest RAM is consistent here.
    void (*retrace)(std::uint64_t field) = nullptr;
    // Once the overlay is up, on the GL thread with the overlay's lock held: sbc-port-overlay
    // calls such as spo_screen_open are allowed. Not called without the overlay.
    void (*overlay_ready)() = nullptr;
};

// Command line:
//   DISC SAVES SHADER_CACHE [--seconds S] [--capture DIR] [--capture-every N] [--windowed]
//   [--input SCRIPT] [--profile]
// Returns the process exit status.
int run_frontend(const GameInfo&, int argc, char** argv);

} // namespace gcrt

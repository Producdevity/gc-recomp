// Entry point for ports: boots a recompiled DOL from a disc image.
#pragma once

#include "gcrt/devices.hpp"
#include "gcrt/gx.hpp"
#include "gcrt/program.hpp"

#include <string>

namespace gcrt {

struct BootOptions {
    std::string disc;
    Program program;   // the game's code, made from the disc's DOL
    // Memory card A as a Dolphin GCI folder under saves (<saves>/USA/Card A); empty: no card.
    std::string saves;
    GxBackend* gx = nullptr;
    bool gx_thread = false;   // run the GX backend on its own thread
    RetraceHook retrace;
    PadSource pads;
    AudioSink audio;
};

// Loads the DOL, sets up low memory as the IPL and apploader would and runs the game on
// the calling thread. Returns only on failure, with the reason in error.
bool boot(const BootOptions&, std::string& error);

} // namespace gcrt

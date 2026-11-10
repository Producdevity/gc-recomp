// Host models of the JAudio DSP microcode family (runtime/src/jaudio_dsp.cpp).
#pragma once

#include "gcrt/dsp.hpp"

namespace gcrt {

// Luigi's Mansion's early JAudio microcode (runtime/src/jaudio.cpp).
Microcode* create_jaudio_early(DspApi&);
// JASystem microcode as Super Mario Sunshine uploads it (runtime/src/jasystem.cpp).
Microcode* create_jasystem(DspApi&);

} // namespace gcrt

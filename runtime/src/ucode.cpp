// Host models of DSP microcode.
#include "gcrt/dsp.hpp"
#include "gcrt/jaudio.hpp"
#include "gcrt/runtime.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace gcrt {
Microcode* create_microcode(std::uint32_t hash, DspApi& api) {
    switch (hash) {
    case 0x64d75375: return create_jaudio_early(api);   // Luigi's Mansion (GLME01)
    case 0x9a483303: return create_jasystem(api);       // Super Mario Sunshine (GMSE01)
    default: return nullptr;
    }
}

} // namespace gcrt

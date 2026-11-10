// Device modules and their host-side hooks.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace gcrt {

// Called on the guest thread when a display interrupt fires: physical address of the
// top-field framebuffer, and a running field count.
using RetraceHook = std::function<void(std::uint32_t framebuffer, std::uint64_t field)>;
void vi_init(RetraceHook);
std::uint32_t vi_framebuffer();
bool vi_enabled();

// Controller state for one port, in the bit layout the SI returns.
struct PadState {
    bool connected = false;
    std::uint16_t buttons = 0;       // GC order: bit 0 left ... bit 12 start (see si.cpp)
    std::uint8_t stick_x = 128, stick_y = 128, cstick_x = 128, cstick_y = 128;
    std::uint8_t trigger_l = 0, trigger_r = 0;
};
using PadSource = std::function<PadState(int port)>;
void si_init(PadSource);

// The disc image the drive reads.
bool di_init(const std::string& image_path, std::string& error);
bool disc_read(std::uint64_t offset, void* destination, std::size_t size);

// Memory cards in slots A and B (empty path: no card) and the IPL/RTC/SRAM chip.
// Card image paths for slots A and B; empty leaves the slot empty.
void exi_init(const std::string& card_a, const std::string& card_b);
// Writes card images with unsaved changes now (they are otherwise written shortly after
// the last change).
void memory_cards_flush();

// DSP mailboxes and microcode, ARAM DMA and the audio interface.
using AudioSink = std::function<void(const std::int16_t* stereo, std::size_t frames, unsigned rate)>;
void dsp_init(AudioSink);

} // namespace gcrt

// Controller input helpers shared by frontends and test runners.
#pragma once

#include "gcrt/devices.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace gcrt {

// GameCube controller button bits, as PADStatus reports them.
enum PadButton : std::uint16_t {
    PAD_LEFT = 0x0001, PAD_RIGHT = 0x0002, PAD_DOWN = 0x0004, PAD_UP = 0x0008, PAD_Z = 0x0010, PAD_R = 0x0020,
    PAD_L = 0x0040, PAD_A = 0x0100, PAD_B = 0x0200, PAD_X = 0x0400, PAD_Y = 0x0800, PAD_START = 0x1000,
};

// Timed button presses for automated runs: "SECONDS:buttons[,SECONDS:buttons...]", buttons
// joined by '+' from a b x y start z l r up down left right, the main stick pushed fully
// with sleft sright sup sdown, or '-' for none. Each entry holds from its time until the next.
class InputScript {
public:
    bool parse(const std::string& text, std::string& error);
    bool active() const { return !steps_.empty(); }
    // Buttons at the given time since boot, in seconds.
    std::uint16_t buttons(double seconds) const;
    // Adds the scripted buttons and, while the script pushes it, the main stick.
    void apply(double seconds, PadState& pad) const;

private:
    struct Step {
        double at;
        std::uint16_t buttons;
        std::uint8_t stick_x = 128, stick_y = 128, cstick_x = 128, cstick_y = 128;
    };
    const Step* at(double seconds) const;
    std::vector<Step> steps_;
};

} // namespace gcrt

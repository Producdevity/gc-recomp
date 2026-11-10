#include "gcrt/input.hpp"

#include <cstdlib>
#include <sstream>

namespace gcrt {

bool InputScript::parse(const std::string& text, std::string& error) {
    steps_.clear();
    std::stringstream entries(text);
    std::string entry;
    while (std::getline(entries, entry, ',')) {
        auto colon = entry.find(':');
        if (colon == std::string::npos) {
            error = "input script entry without ':' : " + entry;
            return false;
        }
        Step step{std::atof(entry.substr(0, colon).c_str()), 0};
        static const struct { const char* name; std::uint8_t x, y; bool c; } sticks[] = {
            {"sleft", 28, 128, false}, {"sright", 228, 128, false}, {"sup", 128, 228, false}, {"sdown", 128, 28, false},
            {"cleft", 28, 128, true}, {"cright", 228, 128, true}, {"cup", 128, 228, true}, {"cdown", 128, 28, true},
        };
        std::stringstream names(entry.substr(colon + 1));
        std::string name;
        while (std::getline(names, name, '+')) {
            static const struct { const char* name; std::uint16_t bit; } table[] = {
                {"a", PAD_A}, {"b", PAD_B}, {"x", PAD_X}, {"y", PAD_Y}, {"start", PAD_START}, {"z", PAD_Z},
                {"l", PAD_L}, {"r", PAD_R}, {"up", PAD_UP}, {"down", PAD_DOWN}, {"left", PAD_LEFT},
                {"right", PAD_RIGHT}, {"-", 0},
            };
            bool found = false;
            for (const auto& s : sticks)
                if (name == s.name) {
                    std::uint8_t& x = s.c ? step.cstick_x : step.stick_x;
                    std::uint8_t& y = s.c ? step.cstick_y : step.stick_y;
                    if (s.x != 128) x = s.x;
                    if (s.y != 128) y = s.y;
                    found = true;
                }
            for (const auto& t : table)
                if (name == t.name) {
                    step.buttons |= t.bit;
                    found = true;
                }
            if (!found) {
                error = "unknown button in input script: " + name;
                return false;
            }
        }
        steps_.push_back(step);
    }
    return true;
}

const InputScript::Step* InputScript::at(double seconds) const {
    const Step* current = nullptr;
    for (const auto& s : steps_)
        if (seconds >= s.at) current = &s;
    return current;
}

std::uint16_t InputScript::buttons(double seconds) const {
    const Step* s = at(seconds);
    return s ? s->buttons : 0;
}

void InputScript::apply(double seconds, PadState& pad) const {
    const Step* s = at(seconds);
    if (!s) return;
    pad.buttons |= s->buttons;
    if (s->stick_x != 128) pad.stick_x = s->stick_x;
    if (s->stick_y != 128) pad.stick_y = s->stick_y;
    if (s->cstick_x != 128) pad.cstick_x = s->cstick_x;
    if (s->cstick_y != 128) pad.cstick_y = s->cstick_y;
}

} // namespace gcrt

// Input scripts (runtime/src/input.cpp): buttons, main stick and C-stick directions.
#include "gcrt/devices.hpp"
#include "gcrt/input.hpp"

#include <cstdio>
#include <string>

int main() {
    gcrt::InputScript script;
    std::string error;
    if (!script.parse("1:a+sleft+cup,2:cright,3:-", error)) {
        std::printf("FAIL parse: %s\n", error.c_str());
        return 1;
    }
    unsigned failures = 0;
    const auto check = [&failures](bool ok, const char* what) { if (!ok) { std::printf("FAIL %s\n", what); ++failures; } };
    gcrt::PadState p;
    script.apply(1.5, p);
    check(p.buttons == gcrt::PAD_A && p.stick_x == 28 && p.stick_y == 128 && p.cstick_x == 128 && p.cstick_y == 228,
          "buttons, stick and C-stick up");
    gcrt::PadState q;
    script.apply(2.5, q);
    check(q.buttons == 0 && q.stick_x == 128 && q.cstick_x == 228 && q.cstick_y == 128, "C-stick right alone");
    std::string bad;
    check(!script.parse("1:cnorth", bad), "unknown directions are refused");
    if (failures) return 1;
    std::printf("input: ok\n");
    return 0;
}

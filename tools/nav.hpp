// Development aids for headless runs that read the game's state from guest RAM.
//
//   GCRT_RAM_DUMP=DIR:FROM:STEP:TO   main RAM (24 MiB, big-endian) to DIR/ram-<ms>.bin at
//                                    guest seconds FROM, FROM+STEP, ... TO
//   GCRT_WATCH=EXPR[,EXPR...]        prints the values every GCRT_WATCH_EVERY fields (60)
//   GCRT_NAV=PLAN                    drives port 1 from a route plan (see Controller)
//   GCRT_NAV_RECORD=FILE             writes the presses it makes as a GCRT_INPUT script
//
// An EXPR is an address with an optional pointer walk and type: 0x80401234,
// [0x80401234]+0x10:f (word at the address, plus 0x10, read as a float). Types: w (word,
// default), h (halfword), b (byte), f (float), s (signed word), x (word, printed in hex),
// t (text: up to 32 printable characters, for logs).
#pragma once

#include "gcrt/devices.hpp"
#include "gcrt/input.hpp"
#include "gcrt/runtime.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace gcnav {

inline bool ram_address(std::uint32_t a, std::uint32_t size = 4) {
    return a >= 0x80000000u && a + size <= 0x81800000u;
}

inline float as_float(std::uint32_t w) {
    float f;
    std::memcpy(&f, &w, 4);
    return f;
}

// One RAM location, reached through zero or more pointers.
struct Expr {
    std::uint32_t base = 0;
    std::vector<std::uint32_t> offsets;   // after each dereference
    int derefs = 0;
    char type = 'w';
    std::string text;

    bool parse(const std::string& s) {
        text = s;
        std::string body = s;
        auto colon = body.rfind(':');
        if (colon != std::string::npos) {
            type = body[colon + 1];
            body = body.substr(0, colon);
        }
        // Leading '[' count is the number of dereferences: [[A]+o1]+o2.
        std::size_t i = 0;
        while (i < body.size() && body[i] == '[') ++derefs, ++i;
        char* end = nullptr;
        base = static_cast<std::uint32_t>(std::strtoul(body.c_str() + i, &end, 0));
        const char* p = end;
        for (int d = 0; d < derefs; ++d) {
            if (*p != ']') return false;
            ++p;
            std::uint32_t off = 0;
            if (*p == '+' || *p == '-') {
                bool neg = *p == '-';
                off = static_cast<std::uint32_t>(std::strtoul(p + 1, &end, 0));
                if (neg) off = 0u - off;
                p = end;
            }
            offsets.push_back(off);
        }
        return *p == 0 && std::strchr("whbfsxt", type);
    }
    // Address of the value, 0 when a pointer on the way is not in RAM.
    std::uint32_t address() const {
        std::uint32_t a = base;
        for (int d = 0; d < derefs; ++d) {
            if (!ram_address(a)) return 0;
            a = gcrt::read32(a) + offsets[d];
        }
        return a;
    }
    std::uint32_t size() const { return type == 'h' ? 2 : type == 'b' ? 1 : 4; }
    std::uint32_t raw() const {
        std::uint32_t a = address();
        if (!ram_address(a, size())) return 0;
        return type == 'h' ? gcrt::read16(a) : type == 'b' ? gcrt::read8(a) : gcrt::read32(a);
    }
    // Stores the raw value (a float's bits for type f); false when the address is not in RAM.
    bool store(std::uint32_t v) const {
        std::uint32_t a = address();
        if (!ram_address(a, size())) return false;
        if (type == 'h') gcrt::write16(a, v);
        else if (type == 'b') gcrt::write8(a, v);
        else gcrt::write32(a, v);
        return true;
    }
    double value() const {
        std::uint32_t r = raw();
        switch (type) {
        case 'f': return as_float(r);
        case 's': return static_cast<std::int32_t>(r);
        default: return r;
        }
    }
    // Up to 32 characters at the address; line breaks become spaces.
    std::string read_text() const {
        std::string t;
        std::uint32_t a = address();
        if (!a) return t;
        for (int i = 0; i < 32 && ram_address(a + i, 1); ++i) {
            char c = static_cast<char>(gcrt::read8(a + i));
            if (c == 0) break;
            t += c >= 32 && c < 127 ? c : c == '\n' || c == '\r' ? ' ' : '?';
        }
        return t;
    }
    std::string format() const {
        char b[48];
        if (!address()) return "?";
        if (type == 't') return "'" + read_text() + "'";
        if (type == 'f') std::snprintf(b, sizeof b, "%.2f", value());
        else if (type == 's') std::snprintf(b, sizeof b, "%d", static_cast<int>(value()));
        else if (type == 'x') std::snprintf(b, sizeof b, "%x", raw());
        else std::snprintf(b, sizeof b, "%u", raw());
        return b;
    }
};

inline std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= s.size()) {
        auto end = s.find(sep, start);
        if (end == std::string::npos) end = s.size();
        if (end > start) out.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    return out;
}

// GCRT_RAM_DUMP and GCRT_WATCH, called on every retrace.
class Probe {
public:
    bool init() {
        if (const char* d = std::getenv("GCRT_RAM_DUMP")) {
            auto parts = split(d, ':');
            if (parts.size() != 4) return fail("GCRT_RAM_DUMP=DIR:FROM:STEP:TO");
            dir_ = parts[0];
            next_ = std::atof(parts[1].c_str());
            step_ = std::atof(parts[2].c_str());
            to_ = std::atof(parts[3].c_str());
        }
        if (const char* w = std::getenv("GCRT_WATCH")) {
            for (auto& e : split(w, ',')) {
                Expr x;
                if (!x.parse(e)) return fail("bad GCRT_WATCH entry " + e);
                watch_.push_back(x);
            }
        }
        if (const char* e = std::getenv("GCRT_WATCH_EVERY")) every_ = std::strtoull(e, nullptr, 0);
        return true;
    }
    void retrace(double seconds, std::uint64_t field) {
        if (!dir_.empty() && step_ > 0 && seconds >= next_ && next_ <= to_ + 1e-9) {
            char name[512];
            std::snprintf(name, sizeof name, "%s/ram-%07.0f.bin", dir_.c_str(), next_ * 1000);
            if (std::FILE* f = std::fopen(name, "wb")) {
                std::fwrite(gcrt::host(0x80000000u), 1, gcrt::kRamSize, f);
                std::fclose(f);
            }
            next_ += step_;
        }
        if (!watch_.empty() && field % every_ == 0) {
            std::string line;
            for (auto& w : watch_) line += " " + w.format();
            std::printf("watch %.3f f%llu:%s\n", seconds, static_cast<unsigned long long>(field), line.c_str());
        }
    }

private:
    bool fail(const std::string& why) {
        std::fprintf(stderr, "headless: %s\n", why.c_str());
        return false;
    }
    std::string dir_;
    double next_ = 0, step_ = 0, to_ = 0;
    std::vector<Expr> watch_;
    std::uint64_t every_ = 60;
};

// State-driven input for port 1. A plan is a list of variables and steps, one per line
// ('#' starts a comment):
//
//   var NAME EXPR                  a RAM value (EXPR as for GCRT_WATCH)
//   player X Z ANGLE               variables holding the player's position on the ground
//                                  plane and facing angle (0-65536, 0 = +Z, 16384 = +X)
//   pause_when COND                while COND holds (a cutscene, say), goto steps release
//                                  the stick and their timeouts stop counting
//   wait SECONDS                   nothing pressed
//   until COND [timeout S]         nothing pressed until COND holds
//   hold INPUT for SECONDS         INPUT: buttons, stick (sleft...) and C-stick (cleft, cright,
//                                  cup, cdown) directions joined by '+'
//   hold INPUT until COND [timeout S]
//   tap INPUT [times N] [every S] [with INPUT]
//                                  press for 0.1 s, release, N times (1) every S (0.3)
//   mash INPUT until COND [every S] [timeout S] [with INPUT]
//   goto X Z [radius R] [timeout S] [with INPUT]   (with: held throughout the step)
//                                  steers the stick towards (X, Z) until within R (40)
//   log TEXT                       prints the time, step and variables
//   check COND                     stops the run when COND does not hold
//   poke TARGET VALUE              stores VALUE at TARGET, a variable or an EXPR; its type
//                                  sets the width (b, h, else a word; f stores a float)
//   setbits TARGET MASK            sets MASK's bits at TARGET
//   clearbits TARGET MASK          clears MASK's bits at TARGET
//   dump FILE                      writes main RAM (24 MiB, big-endian) to FILE
//   exit                           writes memory cards and ends the run
//
// Steps run inside the game's controller read, so a store lands between two game frames.
//
// Steps with a condition or a target take timeout S (the run stops there) or limit S (the
// step ends and the plan goes on), and may end with unless COND (skipped when COND holds as
// the step is reached).
//
// COND: TERM [and TERM...], TERM: NAME OP NUMBER with OP one of == != < > <= >= & !&
// (& holds when any of NUMBER's bits are set). NAME is a variable, t (guest seconds) or
// step_t (seconds in the current step); numbers may be hex. A text variable (type t) takes
// NAME has WORD or NAME !has WORD.
//
// A failed check or timeout stops the run and prints the state, so a route never goes on
// blind. Steering picks the closest of the eight full stick directions that input scripts
// can express, so a recorded route replays exactly through GCRT_INPUT in the same build
// and time mode.
class Controller {
public:
    using Exit = std::function<void(int status)>;

    bool load(const std::string& path, std::string& error) {
        std::ifstream in(path);
        if (!in) {
            error = "cannot read " + path;
            return false;
        }
        std::string line;
        int number = 0;
        while (std::getline(in, line)) {
            ++number;
            auto hash = line.find('#');
            if (hash != std::string::npos) line = line.substr(0, hash);
            std::stringstream words(line);
            std::vector<std::string> w;
            for (std::string x; words >> x;) w.push_back(x);
            if (w.empty()) continue;
            std::string why;
            if (!parse_line(w, line, why)) {
                error = path + ":" + std::to_string(number) + ": " + why;
                return false;
            }
        }
        return true;
    }
    bool record(const std::string& path) {
        record_ = std::fopen(path.c_str(), "w");
        return record_ != nullptr;
    }
    void on_exit(Exit e) { exit_ = std::move(e); }

    // Called for each controller read of port 1 at guest time `seconds`.
    void apply(double seconds, gcrt::PadState& pad) {
        if (seconds < last_call_) seconds = last_call_;
        Output out = decide(seconds);
        pad.buttons = out.buttons;
        pad.stick_x = out.x;
        pad.stick_y = out.y;
        pad.cstick_x = out.cx;
        pad.cstick_y = out.cy;
        emit(seconds, out);
        last_call_ = seconds;
    }

private:
    struct Output {
        std::uint16_t buttons = 0;
        std::uint8_t x = 128, y = 128, cx = 128, cy = 128;
        bool operator==(const Output& o) const {
            return buttons == o.buttons && x == o.x && y == o.y && cx == o.cx && cy == o.cy;
        }
    };
    struct Term {
        std::string name;
        std::string op;
        double value = 0;
        std::string text;   // for has / !has
    };
    struct Step {
        std::string kind, text, input_text;
        Expr target;          // poke, setbits, clearbits
        std::uint32_t value = 0;
        Output input, with;   // with: held for the whole step (goto, mash, tap)
        std::vector<Term> cond;
        double seconds = 0, timeout = 0, every = 0.3, x = 0, z = 0, radius = 40;
        int times = 1;
        int line = 0;
        bool soft = false;   // limit: the step ends at the timeout instead of failing
        std::vector<Term> unless;   // skipped when this holds as the step starts
    };

    // Button names as gcrt::InputScript spells them.
    struct ButtonName {
        const char* name;
        std::uint16_t bit;
    };
    static constexpr ButtonName kButtons[] = {
        {"a", gcrt::PAD_A}, {"b", gcrt::PAD_B}, {"x", gcrt::PAD_X}, {"y", gcrt::PAD_Y},
        {"start", gcrt::PAD_START}, {"z", gcrt::PAD_Z}, {"l", gcrt::PAD_L}, {"r", gcrt::PAD_R},
        {"up", gcrt::PAD_UP}, {"down", gcrt::PAD_DOWN}, {"left", gcrt::PAD_LEFT}, {"right", gcrt::PAD_RIGHT},
    };
    static bool parse_input(const std::string& text, Output& out) {
        out = {};
        for (auto& name : split(text, '+')) {
            bool found = name == "-";
            for (auto& b : kButtons)
                if (name == b.name) out.buttons |= b.bit, found = true;
            if (name == "sleft") out.x = 28, found = true;
            if (name == "sright") out.x = 228, found = true;
            if (name == "sup") out.y = 228, found = true;
            if (name == "sdown") out.y = 28, found = true;
            if (name == "cleft") out.cx = 28, found = true;
            if (name == "cright") out.cx = 228, found = true;
            if (name == "cup") out.cy = 228, found = true;
            if (name == "cdown") out.cy = 28, found = true;
            if (!found) return false;
        }
        return true;
    }
    static Output merge(Output a, const Output& b) {
        a.buttons |= b.buttons;
        if (b.x != 128) a.x = b.x;
        if (b.y != 128) a.y = b.y;
        if (b.cx != 128) a.cx = b.cx;
        if (b.cy != 128) a.cy = b.cy;
        return a;
    }
    static std::string format_input(const Output& o) {
        std::string s;
        auto add = [&](const char* n) { s += s.empty() ? n : std::string("+") + n; };
        if (o.y > 128) add("sup");
        if (o.y < 128) add("sdown");
        if (o.x < 128) add("sleft");
        if (o.x > 128) add("sright");
        if (o.cy > 128) add("cup");
        if (o.cy < 128) add("cdown");
        if (o.cx < 128) add("cleft");
        if (o.cx > 128) add("cright");
        for (auto& b : kButtons)
            if (o.buttons & b.bit) add(b.name);
        return s.empty() ? "-" : s;
    }
    static bool number(const std::string& s, double& v) {
        char* end = nullptr;
        if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) v = static_cast<double>(std::strtoul(s.c_str(), &end, 16));
        else v = std::strtod(s.c_str(), &end);
        return end && *end == 0 && !s.empty();
    }
    bool parse_cond(const std::vector<std::string>& w, std::size_t& i, std::vector<Term>& cond, std::string& why) {
        while (i + 3 <= w.size()) {
            Term t{w[i], w[i + 1], 0, w[i + 2]};
            static const char* ops[] = {"==", "!=", "<", ">", "<=", ">=", "&", "!&", "has", "!has"};
            bool op = false;
            for (auto o : ops) op = op || t.op == o;
            const bool text_op = t.op == "has" || t.op == "!has";
            if (!op || (!text_op && !number(w[i + 2], t.value)) || (text_op && (!vars_.count(t.name) || vars_.at(t.name).type != 't'))) {
                why = "bad condition at " + w[i];
                return false;
            }
            if (t.name != "t" && t.name != "step_t" && !vars_.count(t.name)) {
                why = "unknown variable " + t.name;
                return false;
            }
            cond.push_back(t);
            i += 3;
            if (i < w.size() && w[i] == "and") ++i;
            else break;
        }
        if (cond.empty()) why = "missing condition";
        return !cond.empty();
    }
    bool parse_options(const std::vector<std::string>& w, std::size_t i, Step& s, std::string& why) {
        for (; i < w.size(); i += 2) {
            if (w[i] == "unless") {
                std::size_t j = i + 1;
                if (!parse_cond(w, j, s.unless, why)) return false;
                if (j != w.size()) return why = "unless must end the line", false;
                return true;
            }
            if (i + 1 >= w.size()) {
                why = "option " + w[i] + " without value";
                return false;
            }
            double v = 0;
            if (w[i] == "with") {
                if (!parse_input(w[i + 1], s.with)) return why = "bad input " + w[i + 1], false;
                continue;
            }
            if (!number(w[i + 1], v)) return why = "bad number " + w[i + 1], false;
            if (w[i] == "timeout") s.timeout = v;
            else if (w[i] == "limit") s.timeout = v, s.soft = true;
            else if (w[i] == "radius") s.radius = v;
            else if (w[i] == "every") s.every = v;
            else if (w[i] == "times") s.times = static_cast<int>(v);
            else return why = "unknown option " + w[i], false;
        }
        return true;
    }
    bool parse_line(const std::vector<std::string>& w, const std::string& line, std::string& why) {
        const std::string& k = w[0];
        if (k == "var") {
            Expr e;
            if (w.size() != 3 || !e.parse(w[2])) return why = "var NAME EXPR", false;
            vars_[w[1]] = e;
            order_.push_back(w[1]);
            return true;
        }
        if (k == "pause_when") {
            std::size_t i = 1;
            return parse_cond(w, i, pause_, why);
        }
        if (k == "player") {
            if (w.size() != 4) return why = "player X Z ANGLE", false;
            for (int i = 1; i < 4; ++i)
                if (!vars_.count(w[i])) return why = "unknown variable " + w[i], false;
            px_ = w[1], pz_ = w[2], pa_ = w[3];
            return true;
        }
        Step s;
        s.kind = k;
        s.line = static_cast<int>(steps_.size());
        std::size_t i = 1;
        if (k == "wait") {
            if (w.size() != 2 || !number(w[1], s.seconds)) return why = "wait SECONDS", false;
        } else if (k == "until") {
            if (!parse_cond(w, i, s.cond, why) || !parse_options(w, i, s, why)) return false;
        } else if (k == "hold" || k == "tap" || k == "mash") {
            if (w.size() < 2 || !parse_input(w[1], s.input)) return why = "bad input", false;
            s.input_text = w[1];
            i = 2;
            if (k == "hold") {
                if (w.size() >= 4 && w[2] == "for") {
                    if (!number(w[3], s.seconds)) return why = "hold INPUT for SECONDS", false;
                    i = 4;
                } else if (w.size() >= 3 && w[2] == "until") {
                    i = 3;
                    if (!parse_cond(w, i, s.cond, why)) return false;
                } else return why = "hold INPUT (for S | until COND)", false;
            } else if (k == "mash") {
                if (w.size() < 3 || w[2] != "until") return why = "mash INPUT until COND", false;
                i = 3;
                if (!parse_cond(w, i, s.cond, why)) return false;
                s.every = 0.5;
            }
            if (!parse_options(w, i, s, why)) return false;
        } else if (k == "goto") {
            if (w.size() < 3 || !number(w[1], s.x) || !number(w[2], s.z)) return why = "goto X Z", false;
            if (px_.empty()) return why = "goto needs a player line", false;
            if (!parse_options(w, 3, s, why)) return false;
        } else if (k == "log") {
            s.text = line.substr(line.find("log") + 3);
        } else if (k == "check") {
            if (!parse_cond(w, i, s.cond, why)) return false;
        } else if (k == "poke" || k == "setbits" || k == "clearbits") {
            if (w.size() != 3) return why = k + " TARGET VALUE", false;
            if (vars_.count(w[1])) s.target = vars_.at(w[1]);
            else if (!s.target.parse(w[1])) return why = "bad target " + w[1], false;
            if (s.target.type == 't') return why = "cannot store text", false;
            double v = 0;
            if (!number(w[2], v)) return why = "bad number " + w[2], false;
            if (s.target.type == 'f' && k == "poke") {
                float f = static_cast<float>(v);
                std::memcpy(&s.value, &f, 4);
            } else {
                s.value = static_cast<std::uint32_t>(static_cast<std::int64_t>(v));
            }
        } else if (k == "dump") {
            if (w.size() != 2) return why = "dump FILE", false;
            s.input_text = w[1];
        } else if (k == "exit") {
        } else {
            return why = "unknown step " + k, false;
        }
        s.text = s.text.empty() ? line : s.text;
        steps_.push_back(s);
        return true;
    }

    double var(const std::string& name, double seconds) const {
        if (name == "t") return seconds;
        if (name == "step_t") return seconds - step_start_;
        return vars_.at(name).value();
    }
    bool holds(const std::vector<Term>& cond, double seconds) const {
        for (auto& t : cond) {
            if (t.op == "has" || t.op == "!has") {
                bool found = vars_.at(t.name).read_text().find(t.text) != std::string::npos;
                if (found != (t.op == "has")) return false;
                continue;
            }
            double v = var(t.name, seconds);
            const auto bits = static_cast<std::uint32_t>(v) & static_cast<std::uint32_t>(t.value);
            bool ok = t.op == "==" ? v == t.value : t.op == "!=" ? v != t.value : t.op == "<" ? v < t.value
                    : t.op == ">" ? v > t.value : t.op == "<=" ? v <= t.value : t.op == "&" ? bits != 0
                    : t.op == "!&" ? bits == 0 : v >= t.value;
            if (!ok) return false;
        }
        return true;
    }
    std::string state(double seconds) const {
        std::string s;
        char b[64];
        std::snprintf(b, sizeof b, "[%.3f]", seconds);
        s = b;
        for (auto& n : order_) s += " " + n + "=" + vars_.at(n).format();
        return s;
    }
    void stop(double seconds, const std::string& why, int status, bool quit = true) {
        std::printf("nav: %s at step %zu (%s): %s\n", status ? "FAILED" : "done", index_ + 1,
                    index_ < steps_.size() ? steps_[index_].text.c_str() : "end", why.c_str());
        std::printf("nav: %s\n", state(seconds).c_str());
        finished_ = true;
        if (record_) std::fflush(record_);
        if (exit_ && quit) exit_(status);
    }
    void next(double seconds) {
        ++index_;
        started_ = false;
        step_start_ = seconds;
        moved_at_ = seconds;
        tries_ = 0;
        best_dist_ = -1;
        held_ = -1;
        slide_until_ = 0;
        if (std::getenv("GCRT_NAV_TRACE") && index_ < steps_.size())
            std::printf("nav: step %zu %s %s\n", index_ + 1, steps_[index_].text.c_str(), state(seconds).c_str());
    }

    // Stick direction k (0 = up, clockwise in eighths) as an input.
    static Output direction(int k) {
        static const std::uint8_t xs[8] = {128, 228, 228, 228, 128, 28, 28, 28};
        static const std::uint8_t ys[8] = {228, 228, 128, 28, 28, 28, 128, 228};
        Output o;
        o.x = xs[k & 7];
        o.y = ys[k & 7];
        return o;
    }
    static double wrap(double units) {
        units = std::fmod(units, 65536.0);
        if (units > 32768) units -= 65536;
        if (units < -32768) units += 65536;
        return units;
    }
    Output steer(const Step& s, double seconds) {
        double x = var(px_, seconds), z = var(pz_, seconds), facing = var(pa_, seconds);
        double dx = s.x - x, dz = s.z - z;
        // Facing follows the stick through the camera: learn the camera's turn while the
        // stick is held in one direction.
        if (held_ >= 0 && seconds - held_since_ > 0.25) {
            double expect = 32768 - held_ * 8192;
            double off = wrap(facing - expect);
            if (std::fabs(off) < 6000) camera_ = camera_ + 0.2 * wrap(off - camera_);
        }
        double want = std::atan2(dx, dz) * 65536 / (2 * M_PI);
        auto error = [&](int k) { return std::fabs(wrap(32768 - k * 8192 + camera_ - want)); };
        int best = 0;
        for (int k = 1; k < 8; ++k)
            if (error(k) < error(best)) best = k;
        // Keep the held direction while it is within 30 degrees: fewer, longer presses.
        if (held_ >= 0 && seconds >= slide_until_ && error(held_) < 5461) best = held_;
        // Blocked: no progress for 0.6 s. Leave at 45, 90 and then 135 degrees from the
        // target direction (backing off around a corner), alternating sides, for 0.5 s
        // each before going straight again.
        double dist = std::hypot(dx, dz);
        if (dist < best_dist_ - 8) best_dist_ = dist, moved_at_ = seconds, tries_ = 0;
        if (seconds - moved_at_ > 0.6) {
            ++tries_;
            moved_at_ = seconds;
            slide_until_ = seconds + 0.5;
            slide_ = (tries_ & 1 ? 1 : -1) * (1 + (tries_ - 1) / 2 % 3);
        }
        int k = seconds < slide_until_ ? best + slide_ : best;
        k &= 7;
        if (k != held_) held_ = k, held_since_ = seconds;
        Output o = direction(k);
        o.buttons |= s.with.buttons;
        return o;
    }

    Output decide(double seconds) {
        Output none;
        if (finished_) return none;
        if (index_ == static_cast<std::size_t>(-1)) next(seconds);
        while (index_ < steps_.size()) {
            const Step& s = steps_[index_];
            if (!started_ && !s.unless.empty() && holds(s.unless, seconds)) {
                next(seconds);
                continue;
            }
            started_ = true;
            if (s.kind == "goto" && !pause_.empty() && holds(pause_, seconds)) {
                // The step's clock stands still while paused.
                step_start_ += seconds - last_call_;
                moved_at_ += seconds - last_call_;
                held_ = -1;
                return none;
            }
            double in = seconds - step_start_;
            if (s.timeout > 0 && in > s.timeout) {
                if (!s.soft) {
                    stop(seconds, "timeout", 3);
                    return none;
                }
                std::printf("nav: limit reached: %s %s\n", s.text.c_str(), state(seconds).c_str());
                next(seconds);
                continue;
            }
            if (s.kind == "wait") {
                if (in < s.seconds) return none;
            } else if (s.kind == "until") {
                if (!holds(s.cond, seconds)) return none;
            } else if (s.kind == "hold") {
                bool done = s.cond.empty() ? in >= s.seconds : holds(s.cond, seconds);
                if (!done) return s.input;
            } else if (s.kind == "tap") {
                double period = std::max(s.every, 0.2);
                int n = static_cast<int>(in / period);
                if (n < s.times) return in - n * period < 0.1 ? merge(s.input, s.with) : s.with;
            } else if (s.kind == "mash") {
                if (!holds(s.cond, seconds)) {
                    double period = std::max(s.every, 0.2);
                    double phase = in - std::floor(in / period) * period;
                    return phase < 0.1 ? merge(s.input, s.with) : s.with;
                }
            } else if (s.kind == "goto") {
                if (best_dist_ < 0) best_dist_ = 1e30;
                double d = std::hypot(s.x - var(px_, seconds), s.z - var(pz_, seconds));
                if (d > s.radius) return steer(s, seconds);
            } else if (s.kind == "log") {
                std::printf("nav: %s %s\n", s.text.c_str(), state(seconds).c_str());
            } else if (s.kind == "check") {
                if (!holds(s.cond, seconds)) {
                    stop(seconds, "check failed", 4);
                    return none;
                }
                std::printf("nav: ok %s %s\n", s.text.c_str(), state(seconds).c_str());
            } else if (s.kind == "poke" || s.kind == "setbits" || s.kind == "clearbits") {
                std::uint32_t old = s.target.raw();
                std::uint32_t v = s.kind == "poke" ? s.value : s.kind == "setbits" ? old | s.value : old & ~s.value;
                if (!s.target.store(v)) {
                    stop(seconds, "target not in RAM", 5);
                    return none;
                }
                std::printf("nav: %s %08x: %x -> %x\n", s.text.c_str(), s.target.address(), old, s.target.raw());
            } else if (s.kind == "dump") {
                std::FILE* f = std::fopen(s.input_text.c_str(), "wb");
                bool ok = f && std::fwrite(gcrt::host(0x80000000u), 1, gcrt::kRamSize, f) == gcrt::kRamSize;
                if (f) std::fclose(f);
                if (!ok) {
                    stop(seconds, "cannot write " + s.input_text, 5);
                    return none;
                }
                std::printf("nav: dump %s %s\n", s.input_text.c_str(), state(seconds).c_str());
            } else if (s.kind == "exit") {
                stop(seconds, "exit", 0);
                return none;
            }
            next(seconds);
        }
        stop(seconds, "end of plan", 0, false);
        return none;
    }

    // Each change of the output becomes a script entry between the previous read and this
    // one, so GCRT_INPUT replays it at exactly the same reads.
    void emit(double seconds, const Output& out) {
        if (!record_ || (have_last_ && out == last_out_)) return;
        double at = have_last_ ? (last_call_ + seconds) / 2 : seconds;
        char b[32];
        if (have_last_ && seconds - last_call_ > 0.002) std::snprintf(b, sizeof b, "%.3f", std::round(at * 1000) / 1000);
        else std::snprintf(b, sizeof b, "%.6f", at);
        std::fprintf(record_, "%s%s:%s", entries_++ ? "," : "", b, format_input(out).c_str());
        std::fflush(record_);
        last_out_ = out;
        have_last_ = true;
    }

    std::map<std::string, Expr> vars_;
    std::vector<std::string> order_;
    std::string px_, pz_, pa_;
    std::vector<Term> pause_;
    std::vector<Step> steps_;
    std::size_t index_ = static_cast<std::size_t>(-1);
    double step_start_ = 0, last_call_ = 0;
    bool finished_ = false, started_ = false;
    // goto state
    double best_dist_ = -1, moved_at_ = 0, slide_until_ = 0, held_since_ = 0, camera_ = 0;
    int held_ = -1, tries_ = 0, slide_ = 0;
    // recording
    std::FILE* record_ = nullptr;
    Output last_out_;
    bool have_last_ = false;
    int entries_ = 0;
    Exit exit_;
};

} // namespace gcnav

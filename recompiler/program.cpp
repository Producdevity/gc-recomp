#include "program.hpp"

#include "natives.hpp"

#include "ppc.hpp"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <map>

namespace gcr {
namespace {


void add_call_targets(const Image& image, std::vector<Function>& functions) {
    for (int round = 0; round < 4; ++round) {
        std::sort(functions.begin(), functions.end(), [](const Function& a, const Function& b) { return a.start < b.start; });
        std::map<std::uint32_t, std::size_t> starts;
        for (std::size_t n = 0; n < functions.size(); ++n) starts[functions[n].start] = n;
        std::map<std::uint32_t, std::uint32_t> added;
        for (const auto& f : functions) {
            for (std::uint32_t pc = f.start; pc < f.end; pc += 4) {
                Insn i{image.read32(pc)};
                Op op = decode(i);
                std::uint32_t target = 0;
                if (op == Op::b) target = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
                else if (op == Op::bc) target = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
                else continue;
                bool external = target < f.start || target >= f.end;
                if (!(i.lk() || external) || (i.lk() && target == pc + 4)) continue;
                if (starts.count(target) || !image.in_text(target)) continue;
                // Inside another function: a second entry running to that function's end.
                auto next = starts.upper_bound(target);
                std::uint32_t end = next == starts.end() ? target + 4 : next->first;
                if (next != starts.begin()) {
                    const auto& prev = functions[std::prev(next)->second];
                    if (target < prev.end) end = prev.end;
                }
                added[target] = end;
            }
        }
        if (added.empty()) return;
        for (auto& [start, end] : added) {
            Function f;
            f.start = start;
            f.end = end;
            char name[32];
            std::snprintf(name, sizeof name, "entry_%08X", start);
            f.name = name;
            functions.push_back(f);
        }
    }
}

} // namespace

std::uint64_t fnv1a(const std::vector<std::uint8_t>& bytes) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (auto b : bytes) {
        h ^= b;
        h *= 0x100000001b3ull;
    }
    return h;
}

bool build_program(const Image& image, const std::vector<FunctionSymbol>& symbols, const Config& config,
                   const ProgramOptions& options, Program& program, std::string& error) {
    std::map<std::uint32_t, Function> by_start;
    for (const auto& s : symbols) {
        if (!s.size || !image.in_text(s.address)) continue;
        Function f{s.address, s.address + s.size, s.name, {}, {}, false};
        by_start[s.address] = f;
    }
    for (const auto& s : config.functions) {
        Function f{s.address, s.address + s.size, s.name, {}, {}, false};
        by_start[s.address] = f;
    }
    for (auto a : config.skip) by_start.erase(a.first);

    program.image = &image;
    program.check_returns = options.check_returns;
    program.functions.clear();
    for (auto& [a, f] : by_start) program.functions.push_back(f);
    add_call_targets(image, program.functions);
    std::sort(program.functions.begin(), program.functions.end(),
              [](const Function& a, const Function& b) { return a.start < b.start; });
    for (std::size_t n = 0; n < program.functions.size(); ++n) {
        auto& f = program.functions[n];
        program.by_start[f.start] = n;
        if (auto h = config.hle.find(f.start); h != config.hle.end()) f.hle = h->second;
        else if (const char* name = options.natives ? native_for(image, f.start, f.end) : nullptr) f.hle = name;
        if (auto w = config.wrap.find(f.start); w != config.wrap.end()) f.wrap = w->second;
        f.exact = options.exact || config.exact.count(f.start);
    }
    for (auto& [a, h] : config.hle)
        if (!program.by_start.count(a)) {
            error = "hle target is not a function: " + h;
            return false;
        }
    for (auto& [a, v] : config.save_context) program.save_context.insert(a);

    // Data words that point into text: jump-table and callback candidates.
    std::uint32_t text_lo = 0xFFFFFFFF, text_hi = 0;
    for (const auto& s : image.sections())
        if (s.text) {
            text_lo = std::min(text_lo, s.address);
            text_hi = std::max(text_hi, s.address + s.size);
        }
    for (const auto& s : image.sections()) {
        if (s.text) continue;
        for (std::uint32_t a = s.address; a + 4 <= s.address + s.size; a += 4) {
            std::uint32_t v = image.read32(a);
            if (v >= text_lo && v < text_hi && !(v & 3) && image.in_text(v)) program.code_pointers.push_back(v);
        }
    }
    std::sort(program.code_pointers.begin(), program.code_pointers.end());
    program.code_pointers.erase(std::unique(program.code_pointers.begin(), program.code_pointers.end()),
                                program.code_pointers.end());
    return true;
}

} // namespace gcr

// gcrecomp: statically recompiles a GameCube DOL to C.
//
//   gcrecomp (--dol main.dol | --disc game.iso) --symbols symbols.txt [--config game.cfg] --out DIR
//            [--files N] [--check-returns] [--exact]

#include "emit.hpp"
#include "image.hpp"
#include "natives.hpp"
#include "ppc.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <string>

using namespace gcr;

namespace {

[[noreturn]] void die(const std::string& message) {
    std::fprintf(stderr, "gcrecomp: %s\n", message.c_str());
    std::exit(1);
}

std::uint64_t fnv1a(const std::vector<std::uint8_t>& bytes) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (auto b : bytes) {
        h ^= b;
        h *= 0x100000001b3ull;
    }
    return h;
}

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

int main(int argc, char** argv) {
    std::string dol, disc, symbols, config_path, out;
    int files = 64;
    bool check_returns = false, exact = false;
    for (int n = 1; n < argc; ++n) {
        std::string a = argv[n];
        auto value = [&]() -> std::string {
            if (n + 1 >= argc) die("missing value for " + a);
            return argv[++n];
        };
        if (a == "--dol") dol = value();
        else if (a == "--disc") disc = value();
        else if (a == "--symbols") symbols = value();
        else if (a == "--config") config_path = value();
        else if (a == "--out") out = value();
        else if (a == "--files") files = std::atoi(value().c_str());
        else if (a == "--check-returns") check_returns = true;
        else if (a == "--exact") exact = true;
        else die("unknown argument " + a);
    }
    if ((dol.empty() && disc.empty()) || symbols.empty() || out.empty())
        die("usage: gcrecomp (--dol DOL | --disc ISO) --symbols SYMBOLS [--config CFG] --out DIR");

    std::string error;
    Image image;
    if (!(disc.empty() ? image.load_dol(dol, error) : image.load_disc(disc, error))) die(error);
    std::vector<FunctionSymbol> syms;
    if (!load_dtk_symbols(symbols, syms, error)) die(error);
    Config config;
    if (!config_path.empty() && !load_config(config_path, config, error)) die(error);

    std::map<std::uint32_t, Function> by_start;
    for (const auto& s : syms) {
        if (!s.size || !image.in_text(s.address)) continue;
        Function f{s.address, s.address + s.size, s.name, {}, {}, false};
        by_start[s.address] = f;
    }
    for (const auto& s : config.functions) {
        Function f{s.address, s.address + s.size, s.name, {}, {}, false};
        by_start[s.address] = f;
    }
    for (auto a : config.skip) by_start.erase(a.first);

    Program program;
    program.image = &image;
    program.check_returns = check_returns;
    for (auto& [a, f] : by_start) program.functions.push_back(f);
    add_call_targets(image, program.functions);
    std::sort(program.functions.begin(), program.functions.end(),
              [](const Function& a, const Function& b) { return a.start < b.start; });
    for (std::size_t n = 0; n < program.functions.size(); ++n) {
        auto& f = program.functions[n];
        program.by_start[f.start] = n;
        if (auto h = config.hle.find(f.start); h != config.hle.end()) f.hle = h->second;
        else if (const char* name = gcr::native_for(image, f.start, f.end)) f.hle = name;
        if (auto w = config.wrap.find(f.start); w != config.wrap.end()) f.wrap = w->second;
        f.exact = exact || config.exact.count(f.start);
    }
    for (auto& [a, h] : config.hle)
        if (!program.by_start.count(a)) die("hle target is not a function: " + h);
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

    // Prototypes.
    {
        std::ofstream h(out + "/recomp_funcs.h");
        if (!h) die("cannot write to " + out);
        h << "#pragma once\n#include \"gcrt/recomp.h\"\n";
        for (const auto& f : program.functions) {
            char line[64];
            std::snprintf(line, sizeof line, "void f_%08X(gc_ctx*);\n", f.start);
            h << line;
        }
        std::set<std::string> hle;
        for (const auto& f : program.functions)
            if (!f.hle.empty()) hle.insert(f.hle);
        for (const auto& name : hle) h << "void hle_" << name << "(gc_ctx*);\n";
        std::set<std::string> wraps;
        for (const auto& f : program.functions)
            if (!f.wrap.empty()) wraps.insert(f.wrap);
        for (const auto& name : wraps) h << "void wrap_" << name << "(gc_ctx*, gc_func);\n";
    }
    EmitStats stats;
    std::size_t total = program.functions.size();
    std::size_t per_file = (total + files - 1) / files;
    for (int file = 0; file < files; ++file) {
        char name[64];
        std::snprintf(name, sizeof name, "/recomp_%03d.c", file);
        std::ofstream c(out + name);
        c << "#include \"recomp_funcs.h\"\n#include <math.h>\n\n";
        for (std::size_t n = file * per_file; n < std::min(total, (file + 1) * per_file); ++n)
            c << emit_function(program, program.functions[n], stats) << "\n";
    }
    {
        std::ofstream t(out + "/recomp_table.c");
        t << "#include \"recomp_funcs.h\"\n\nconst gc_func_entry gc_func_table[] = {\n";
        for (const auto& f : program.functions) {
            char line[96];
            std::snprintf(line, sizeof line, "    {0x%08Xu, 0x%08Xu, f_%08X},\n", f.start, f.end, f.start);
            t << line;
        }
        char tail[256];
        std::snprintf(tail, sizeof tail,
                      "};\nconst unsigned gc_func_count = %zu;\nconst unsigned long long gc_dol_hash = 0x%016llxull;\n",
                      total, static_cast<unsigned long long>(fnv1a(image.bytes())));
        t << tail;
    }
    std::printf("functions %zu, instructions %zu, invalid %zu, indirect jumps %zu, unresolved %zu, spin loops %zu\n",
                total, stats.instructions, stats.invalid, stats.indirect_jumps, stats.unresolved, stats.spin_loops);
    return 0;
}

#include "a64_translate.hpp"

#include "image.hpp"
#include "program.hpp"

#include <cstdio>

namespace gcbe {

bool translate_files(const Request& request, Stats& stats, const Progress& progress, std::string& error) {
    gcr::Image image;
    if (!(!request.dol_bytes.empty() ? image.load_dol_bytes(request.dol_bytes, error)
          : request.disc.empty()     ? image.load_dol(request.dol, error)
                                     : image.load_disc(request.disc, error)))
        return false;
    std::vector<gcr::FunctionSymbol> symbols;
    if (!gcr::load_dtk_symbols(request.symbols, symbols, error)) return false;
    gcr::Config config;
    if (!request.config.empty() && !gcr::load_config(request.config, config, error)) return false;
    gcr::Program program;
    gcr::ProgramOptions po;
    po.exact = request.exact;
    po.natives = request.natives;
    if (!gcr::build_program(image, symbols, config, po, program, error)) return false;
    std::vector<std::uint8_t> blob;
    if (!translate(program, request.options, blob, stats, progress, error)) return false;
    std::string temp = request.out + ".tmp";
    std::FILE* f = std::fopen(temp.c_str(), "wb");
    if (!f) {
        error = "cannot write " + temp;
        return false;
    }
    bool ok = std::fwrite(blob.data(), 1, blob.size(), f) == blob.size();
    ok = std::fclose(f) == 0 && ok;
    if (!ok || std::rename(temp.c_str(), request.out.c_str()) != 0) {
        std::remove(temp.c_str());
        error = "cannot write " + request.out;
        return false;
    }
    return true;
}

} // namespace gcbe

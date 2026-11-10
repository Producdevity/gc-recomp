// GameCube disc image readers.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace gcrt {

class Disc {
public:
    virtual ~Disc() = default;
    virtual std::uint64_t size() const = 0;
    virtual bool read(std::uint64_t offset, void* destination, std::size_t size) = 0;
};

// ISO/GCM, or RVZ (uncompressed groups) by its magic.
std::unique_ptr<Disc> open_disc(const std::string& path, std::string& error);

} // namespace gcrt

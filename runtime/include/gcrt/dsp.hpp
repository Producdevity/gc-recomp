// Host models of DSP microcode, selected by a hash of the uploaded IRAM image.
#pragma once

#include <cstdint>

namespace gcrt {

class DspApi {
public:
    virtual ~DspApi() = default;
    // Queue a mail for the CPU, optionally raising the DSP interrupt.
    virtual void send_mail(std::uint32_t mail, bool interrupt) = 0;
    // Raise the DSP interrupt without a mail.
    virtual void raise_interrupt() = 0;
    virtual std::uint8_t* main_memory(std::uint32_t address, std::uint32_t size) = 0;
    virtual std::uint8_t* aram_memory() = 0;
    virtual unsigned output_rate() = 0;
};

class Microcode {
public:
    virtual ~Microcode() = default;
    virtual const char* name() const = 0;
    virtual void start() = 0;
    virtual void mail(std::uint32_t) = 0;
    virtual void cpu_interrupt() {}
};

std::uint32_t ucode_hash(const std::uint8_t* data, std::uint32_t size);
// Returns nullptr for an unknown microcode.
Microcode* create_microcode(std::uint32_t hash, DspApi&);

} // namespace gcrt

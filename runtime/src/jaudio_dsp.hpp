// Signal path of the JAudio DSP microcode family (runtime/src/jaudio_dsp.cpp). The
// microcode front ends (jaudio.cpp, jasystem.cpp) drive it block by block.
#pragma once

#include "gcrt/dsp.hpp"

#include <cstdint>

namespace gcrt {

class JaudioDsp {
public:
    // Early: Luigi's Mansion's microcode. Jasystem: Super Mario Sunshine's.
    enum class Variant { Early, Jasystem };

    JaudioDsp(DspApi&, Variant);
    // Boot: data memory cleared.
    void reset();
    // Command 0x81: voice count, voice table, render tables, ADPCM coefficients and the
    // four effect lines' parameter blocks, all in main memory.
    void setup(std::uint16_t voices, std::uint32_t voice_table, std::uint32_t tables, std::uint32_t adpcm,
               std::uint32_t effects);
    // Command 0x82: output buffers and Q4.12 mixer level; effect parameters are read here.
    void begin_frame(std::uint32_t left, std::uint32_t right, std::uint16_t level);
    // Per 80-sample block: buses and effect lines before the CPU releases the voices,
    // then the voices, then the mix written to the outputs (which then advance).
    void prepare_block();
    void render_voices();
    void finish_block();
    // Jasystem: the 64-voice mask sent with each release (voice 0 in bit 63).
    void set_active_mask(std::uint32_t high, std::uint32_t low);

private:
    using u8 = std::uint8_t;
    using u16 = std::uint16_t;
    using s16 = std::int16_t;
    using u32 = std::uint32_t;
    using s32 = std::int32_t;

    bool early() const { return variant_ == Variant::Early; }
    s16 rs(u32 a) const { return static_cast<s16>(m_[a & 0xFFF]); }
    void ws(u32 a, s32 v) { m_[a & 0xFFF] = static_cast<u16>(v); }
    u32 r32(u32 a) const { return (u32(m_[a & 0xFFF]) << 16) | m_[(a + 1) & 0xFFF]; }
    void w32(u32 a, u32 v) {
        m_[a & 0xFFF] = static_cast<u16>(v >> 16);
        m_[(a + 1) & 0xFFF] = static_cast<u16>(v);
    }
    void dma_in(u32 mram, u32 dram, u32 words);
    void dma_out(u32 mram, u32 dram, u32 words);
    void copy_words(u32 from, u32 to, u32 count);
    void report(const char* what, u32 value);
    u8 accel8();
    s16 accel16();

    void copy_scaled(u16 src, u16 dst, int count, s16 gain);
    void add_scaled(u16 src, u16 dst, int count, s16 gain);
    s16 mix_ramp(u16 src, u16 dst, s16 gain, s32 delta);
    void master_gain(u16 bus);
    void fir(u16 coefs, u16 data, int taps);

    void load_fx_line(int line);
    u32 fx_ring_address(int line);
    void prepare_effects();
    void finish_effects();
    void update_patterns();
    void init_modulated_delay();
    void modulated_delay();

    void render_voice(u32 address);
    void zero_fill(int count);
    void copy_run(u32 from, int count);
    void decode_afc(u32 dst);
    void compute_afc_position();
    void decode_afc_buffered();
    void afc_silence();
    void afc_block_boundary();
    void copy_afc_retained(int& n, bool bounded);
    void afc_buffered(int n);
    void compute_afc_hq_position();
    void afc_aligned(int n);
    void compute_pcm_position(bool wide);
    void read_pcm(int count, bool wide);
    void pcm_aram(int n, bool wide);
    void mram_samples(u16 count);
    void pcm_mram(int n);
    void generate(u16 type);
    u16 pattern(u16 table, u16 phase, u16 pitch, bool modulated);
    void resample();
    void filter_voice();
    void mix_sends();
    void mix_positional();

    DspApi& api_;
    Variant variant_;
    u16 m_[0x1000] = {};
    u32 voice_table_ = 0, fx_table_ = 0, out_left_ = 0, out_right_ = 0, voice_count_ = 0;
    s16 master_gain_ = 0;
    u32 accel_ = 0;
    const u8* aram_ = nullptr;   // ARAM, allocated once at startup
    u16 ar3_ = 0, filtered_ = 0;
    bool configured_ = false;
    int reports_ = 0;
    u16 logged_fx_[4][16] = {};
};

} // namespace gcrt

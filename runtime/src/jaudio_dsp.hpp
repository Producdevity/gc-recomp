// Signal path of the JAudio DSP microcode family (runtime/src/jaudio_dsp.cpp). The
// microcode front ends (jaudio.cpp, jasystem.cpp) drive it block by block.
#pragma once

#include "gcrt/dsp.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

namespace gcrt {

class JaudioDsp {
public:
    // Early: Luigi's Mansion's microcode. Jasystem: Super Mario Sunshine's.
    enum class Variant { Early, Jasystem };

    // Jasystem's split of a block between two threads (jasystem.cpp). The voice side runs
    // where the block runs: voice fetch, decoding, filters, send gains and the voice
    // write-back, so everything the CPU reads back is ready with the block's mails. The bus
    // side (buses, effect lines, mixing the voices into the buses, master level, output and
    // effect rings) is recorded into a Synthesis, with a copy of every voice's samples as
    // the mixer reads them, and synthesize() runs it later on another thread. A voice
    // without filters resamples there too, from a copy of its raw buffer; the voice side
    // computes only the samples its write-back holds (held sample, delay history).
    // The sides own disjoint data memory: the voice side 0x000-0x37F, 0x3F8-0x8FF and
    // 0xB00-0xBFF, the bus side the rest. A voice or a frame whose addresses would cross
    // that line ends the recording: drain, run the recorded part, continue unsplit.
    struct Synthesis {
        struct Mix {
            std::uint32_t samples;   // index of 80 samples in `samples`
            std::uint16_t bus;
            std::int16_t gain;
            std::int32_t delta;
        };
        // A voice's output, resampled from a raw buffer copy into `samples`.
        struct Resample {
            std::uint32_t raw, out;
            std::uint16_t phase, pitch;
        };
        bool frame = false, prepared = false, finished = false;
        std::uint16_t fx[0x40] = {};   // frame: effect parameters, outputs and level
        std::uint32_t left = 0, right = 0;
        std::int16_t level = 0;
        bool coefs_taken = false;
        std::int16_t coefs[0x100] = {};   // the resampling table, with the first Resample
        std::vector<Resample> resamples;
        std::vector<Mix> mixes;
        std::vector<std::int16_t> samples;
    };

    JaudioDsp(DspApi&, Variant);
    // Boot: data memory cleared.
    void reset();
    // Command 0x81: voice count, voice table, render tables, ADPCM coefficients and the
    // four effect lines' parameter blocks, all in main memory.
    void setup(std::uint16_t voices, std::uint32_t voice_table, std::uint32_t tables, std::uint32_t adpcm,
               std::uint32_t effects);
    // Command 0x82: output buffers and Q4.12 mixer level; effect parameters are read here.
    // With a job, recording starts with the frame when its effect lines allow the split.
    void begin_frame(std::uint32_t left, std::uint32_t right, std::uint16_t level, Synthesis* job = nullptr);
    // Per 80-sample block: buses and effect lines before the CPU releases the voices,
    // then the voices, then the mix written to the outputs (which then advance).
    void prepare_block();
    void render_voices();
    void finish_block();
    // Jasystem: the 64-voice mask sent with each release (voice 0 in bit 63).
    void set_active_mask(std::uint32_t high, std::uint32_t low);

    // Waits until the other thread has run every job handed to it.
    void set_drain(std::function<void()>);
    // Records the next block into the job (no effect while one is recorded or when the
    // frame was not split).
    void defer(Synthesis*);
    // The recorded job once its block has ended; recording stops.
    Synthesis* take_finished();
    // Drains, runs the recorded part here and stops recording.
    void undefer();
    // The bus side of a recorded job (any thread; one at a time, in order).
    void synthesize(Synthesis&);

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
    void report(const char* what, u32 value) const;
    u8 accel8();
    s16 accel16();

    void copy_scaled(u16 src, u16 dst, int count, s16 gain);
    void add_scaled(u16 src, u16 dst, int count, s16 gain);
    s16 mix_ramp(u16 src, u16 dst, s16 gain, s32 delta);
    s16 mix_samples(const s16* in, u16 dst, s16 gain, s32 delta);
    s16 mix(u16 src, u16 dst, s16 gain, s32 delta);
    u32 capture(u16 src);
    s16 record(u32 samples, u16 dst, s16 gain, s32 delta);
    void master_gain(u16 bus);
    void fir(u16 coefs, u16 data, int taps, bool scratch = true);

    void prepare_buses();
    void finish_buses();
    bool separable_voice() const;
    static bool separable_bus(u16 bus);

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
    static s16 interpolate(const s16* coefs, const s16* raw, u16 phase, u16 pitch, int from, int to, s16* out);
    void end_resample(u16 phase, u16 pitch, s16 last);
    void resample_voice();
    bool writes_voice_out() const;
    void materialize();
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
    mutable std::atomic<int> reports_{0};
    u16 logged_fx_[4][16] = {};
    Synthesis* job_ = nullptr;
    bool frame_split_ = true;
    // The voice being rendered resamples on the DSP thread: its output words' index in
    // the job, after a prefix of the words before them.
    static constexpr u32 kNone = ~0u;
    u32 voice_out_ = kNone;
    u16 voice_prefix_ = 0;
    bool prefix_taken_ = false;
    // The output words in data memory are not yet written (see materialize).
    bool pending_ = false;
    u16 pending_phase_ = 0, pending_pitch_ = 0;
    std::function<void()> drain_;
};

} // namespace gcrt

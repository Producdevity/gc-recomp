// Signal path of the JAudio DSP microcode family: Luigi's Mansion's early microcode
// (IRAM hash 0x64d75375) and Super Mario Sunshine's JASystem one (0x9a483303).
//
// Behaviour follows Sunshine's microcode as the doldecomp/sms annotated source documents
// it (libs/JSystem/src/sms_jdsp.ds assembles to the bytes in the DOL). Luigi's Mansion's
// upload shares that code word for word for decoding, resampling, filters, the six-send
// mixer, master level, effect lines and the modulated delay; where it differs (listed
// at Variant::Early below) its own words were read through the same mnemonics.
//
// The model keeps the microcode's 4K-word data memory: buses, the voice block, effect
// lines and scratch sit at their DSP addresses, so sends reach whatever address the CPU
// names. Arithmetic is the DSP's: 16x16 products (doubled where the microcode selects
// that), 40-bit accumulators, round-half-even where a product moves to an accumulator's
// high part, saturation where the microcode runs in 40-bit mode (voice mixing, decoding,
// resampling, master level) and wrap-around where it runs in 16-bit mode (bus folds,
// effect lines, FIR filters).
//
// Variant::Early differs from Sunshine's: no automatic (positional) mixer, so every
// voice uses its six sends; each block starts the fronts as the previous block's rear
// left and its complement (no rear-right or rear-effect folds); the modulated delay
// reads bus 0x0B00 (ring 0x0B60) and main-memory PCM goes through scratch 0x0A00; setup
// reads 0x200 table words (no sine table); sources 0x05 and 0x10 generate nothing; the
// general AFC reader always loops; every voice is fetched (no active mask).
#include "jaudio_dsp.hpp"

#include "gcrt/runtime.hpp"

#include <algorithm>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif
#include <cstdio>
#include <cstdlib>

namespace gcrt {
namespace {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using s16 = std::int16_t;
using u32 = std::uint32_t;
using s32 = std::int32_t;
using u64 = std::uint64_t;
using s64 = std::int64_t;

// DSP data memory, word addresses.
enum : u16 {
    RESAMPLE_COEFS = 0x000, PATTERN_0 = 0x100, PATTERN_1 = 0x140, PATTERN_2 = 0x180, PATTERN_3 = 0x1C0,
    SINE_TABLE = 0x200, AFC_COEFS = 0x300, FX_LINE = 0x390, FX_RING_INDEX = 0x3A4, FX_PARAMS = 0x3A8,
    COEF_SCRATCH = 0x3E8, MD_WRITE = 0x3F0, MD_READ = 0x3F1, MD_DIRECTION = 0x3F2, MD_THRESHOLD_0 = 0x3F3,
    MD_THRESHOLD_1 = 0x3F4, MD_STEP = 0x3F5, MD_FRACTION = 0x3F6, MD_STEP_DELTA = 0x3F7, SAVED_HELD_SAMPLE = 0x3F8,
    // Voice block, DMA'd from the CPU's 0x180-byte channel; the first 0x80 words go back.
    VOICE = 0x400, V_ENABLED = 0x400, V_DONE = 0x401, V_PITCH = 0x402, V_RESET = 0x404, V_END_REACHED = 0x405,
    V_PAUSED = 0x406, V_DELAY_COUNT = 0x407, V_SENDS = 0x408, V_PAN = 0x428, V_FX = 0x429, V_AM_CURRENT = 0x42A,
    V_AM_TARGET = 0x42B, V_AUTO_MIXER = 0x42C, V_AUTO_MIXER_COPY = 0x42D, V_PHASE = 0x430, V_AFC_INDEX = 0x431,
    V_AFC_COUNT = 0x432, V_HELD = 0x433, V_POSITION = 0x434, V_BLOCK = 0x436, V_ARAM = 0x438, V_REMAINING = 0x43A,
    V_RESAMPLE_HISTORY = 0x43C, V_FIR_HISTORY = 0x440, V_IIR_HISTORY = 0x454, V_AFC_DECODED = 0x458,
    V_AFC_OLDER = 0x466, V_AFC_NEWER = 0x467, V_DELAY_HISTORY_END = 0x477, V_DIST_Y = 0x478, V_DIST_X = 0x479,
    V_TYPE = 0x480, V_LOOP = 0x481, V_LOOP_NEWER = 0x482, V_LOOP_OLDER = 0x483, V_FILTER = 0x484, V_STOP = 0x485,
    V_LOOP_START = 0x488, V_END = 0x48A, V_BASE = 0x48C, V_FIR_COEFS = 0x490, V_IIR_COEFS = 0x4A4,
    V_DIST_COEF = 0x4A8,
    ACTIVE_MASK = 0x4FC, VOICE_OUT = 0x520, RAW = 0x580,
    BUS_AUX_CENTER = 0x9A0, BUS_REAR_LEFT_FX = 0xC00, BUS_REAR_RIGHT_FX = 0xC50, BUS_REAR_RIGHT = 0xCA0,
    BUS_FRONT_LEFT = 0xD00, BUS_FRONT_RIGHT = 0xD60, BUS_FX_0 = 0xDC0, BUS_FRONT_LEFT_FX = 0xE80,
    BUS_FRONT_RIGHT_FX = 0xEE0, BUS_REAR_LEFT = 0xF40, BUS_AUX_RIGHT_SUBTRACT = 0xFA0,
    // Sunshine: modulated-delay bus, its ring, and positional gains / main-memory PCM scratch.
    // Early: the modulated delay uses 0x0B00 and main-memory PCM 0x0A00.
    SCRATCH_A00 = 0xA00, SCRATCH_B00 = 0xB00,
};

enum : u16 {
    SOURCE_AFC_LQ = 0x05, SOURCE_PCM8_ARAM = 0x08, SOURCE_AFC_HQ = 0x09, SOURCE_PCM16_ARAM = 0x10,
    SOURCE_PCM16_MRAM_DIRECT = 0x20, SOURCE_PCM16_MRAM_RESAMPLED = 0x21,
};

constexpr int kBlock = 80;
constexpr s16 kUnity = 0x7FFF, kRearCross = static_cast<s16>(0xB820);   // Dolby surround fold-down
constexpr u16 kPositionalDestinations[8] = {
    BUS_FRONT_LEFT, BUS_FRONT_RIGHT, BUS_REAR_LEFT, BUS_REAR_RIGHT,
    BUS_FRONT_LEFT_FX, BUS_FRONT_RIGHT_FX, BUS_REAR_LEFT_FX, BUS_REAR_RIGHT_FX,
};

// 40-bit accumulator values are kept sign-extended in an int64.
inline s64 wrap40(s64 v) { return static_cast<s64>(static_cast<u64>(v) << 24) >> 24; }
// Middle word as the register holds it.
inline s16 mid(s64 acc) { return static_cast<s16>(static_cast<u16>(static_cast<u64>(acc) >> 16)); }
// Middle word as a 40-bit-mode read gives it: clamped when the value exceeds 32 bits.
inline s16 mid_sat(s64 acc) {
    acc = wrap40(acc);
    if (acc > INT32_MAX) return 0x7FFF;
    if (acc < INT32_MIN) return -0x8000;
    return static_cast<s16>(acc >> 16);
}
// Product moved to the high part with rounding: nearest, ties to even.
inline s64 round_mid(s64 p) { return (p + ((p & 0x10000) ? 0x8000 : 0x7FFF)) & ~static_cast<s64>(0xFFFF); }

} // namespace

JaudioDsp::JaudioDsp(DspApi& api, Variant variant) : api_(api), variant_(variant) {}

void JaudioDsp::reset() {
    for (auto& w : m_) w = 0;
}

// ---- Data memory and DMA ----------------------------------------------------------------
void JaudioDsp::dma_in(u32 mram, u32 dram, u32 words) {
    const u8* p = api_.main_memory(mram & 0x01FFFFFFu, words * 2);
    for (u32 i = 0; i < words; ++i) m_[(dram + i) & 0xFFF] = p ? static_cast<u16>((p[i * 2] << 8) | p[i * 2 + 1]) : 0;
    if (!p) report("DMA from %08x outside RAM", mram);
}
void JaudioDsp::dma_out(u32 mram, u32 dram, u32 words) {
    u8* p = api_.main_memory(mram & 0x01FFFFFFu, words * 2);
    if (!p) return report("DMA to %08x outside RAM", mram);
    for (u32 i = 0; i < words; ++i) {
        u16 w = m_[(dram + i) & 0xFFF];
        p[i * 2] = static_cast<u8>(w >> 8);
        p[i * 2 + 1] = static_cast<u8>(w);
    }
}
void JaudioDsp::copy_words(u32 from, u32 to, u32 count) {
    for (u32 i = 0; i < count; ++i) m_[(to + i) & 0xFFF] = m_[(from + i) & 0xFFF];
}
// Anomalies the microcode would not survive cleanly; logged a few times.
void JaudioDsp::report(const char* what, u32 value) {
    if (reports_ >= 16) return;
    ++reports_;
    char text[128];
    std::snprintf(text, sizeof text, what, value);
    log("DSP: JAudio: %s\n", text);
}
// Sample accelerator reading ARAM sequentially from a byte address.
JaudioDsp::u8 JaudioDsp::accel8() {
    if (!aram_) aram_ = api_.aram_memory();
    return aram_[accel_++ & (kAramSize - 1)];
}
JaudioDsp::s16 JaudioDsp::accel16() {
    u32 a = accel_ & (kAramSize - 2);
    accel_ = a + 2;
    if (!aram_) aram_ = api_.aram_memory();
    return static_cast<s16>((aram_[a] << 8) | aram_[a + 1]);
}

// ---- Commands and blocks ----------------------------------------------------------------
void JaudioDsp::setup(u16 voices, u32 voice_table, u32 tables, u32 adpcm, u32 effects) {
    voice_table_ = voice_table;
    fx_table_ = effects;
    dma_in(tables, RESAMPLE_COEFS, early() ? 0x200 : 0x280);
    dma_in(adpcm, AFC_COEFS, 0x20);
    voice_count_ = voices;
    init_modulated_delay();
}

void JaudioDsp::begin_frame(u32 left, u32 right, u16 level) {
    out_left_ = left;
    out_right_ = right;
    master_gain_ = static_cast<s16>(level);
    dma_in(fx_table_, FX_PARAMS, 0x40);
    if (verbosity > 2)
        for (int line = 0; line < 4; ++line) {
            const u16* p = &m_[FX_PARAMS + line * 16];
            if (std::equal(p, p + 16, logged_fx_[line])) continue;
            std::copy(p, p + 16, logged_fx_[line]);
            log("DSP: JAudio effect line %d: flags %x ring %u blocks at %08x, sends %04x:%04x %04x:%04x, "
                "FIR %04x %04x %04x %04x %04x %04x %04x %04x\n",
                line, p[0], p[1], (u32(p[2]) << 16) | p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12],
                p[13], p[14], p[15]);
        }
}

void JaudioDsp::set_active_mask(u32 high, u32 low) {
    m_[ACTIVE_MASK] = static_cast<u16>(high >> 16);
    m_[ACTIVE_MASK + 1] = static_cast<u16>(high);
    m_[ACTIVE_MASK + 2] = static_cast<u16>(low >> 16);
    m_[ACTIVE_MASK + 3] = static_cast<u16>(low);
}

void JaudioDsp::prepare_block() {
    if (early()) {
        // Fronts start as the previous block's rear left and its complement.
        for (int n = 0; n < kBlock; ++n) {
            u16 x = m_[BUS_REAR_LEFT + n];
            m_[BUS_FRONT_LEFT + n] = x;
            m_[BUS_FRONT_RIGHT + n] = static_cast<u16>(~x);
        }
        for (u16 bus : {BUS_REAR_RIGHT, BUS_REAR_LEFT, BUS_AUX_RIGHT_SUBTRACT, SCRATCH_B00, BUS_AUX_CENTER})
            for (int n = 0; n < kBlock; ++n) m_[bus + n] = 0;
    } else {
        // Fronts start as the previous block's rear left (delayed 80 samples) and the
        // second half of its rear right (delayed 40), matrixed.
        copy_scaled(BUS_REAR_LEFT, BUS_FRONT_LEFT, kBlock, kUnity);
        copy_scaled(BUS_REAR_LEFT, BUS_FRONT_RIGHT, kBlock, kRearCross);
        add_scaled(BUS_REAR_RIGHT + 40, BUS_FRONT_LEFT, 40, kRearCross);
        add_scaled(BUS_REAR_RIGHT + 40, BUS_FRONT_RIGHT, 40, kUnity);
        for (u16 bus : {BUS_REAR_RIGHT, BUS_REAR_LEFT, BUS_AUX_RIGHT_SUBTRACT, SCRATCH_A00, BUS_AUX_CENTER})
            for (int n = 0; n < kBlock; ++n) m_[bus + n] = 0;
    }
    prepare_effects();
    update_patterns();
}

// GCRT_NOMIX=1 skips the voices (diagnostics: the buses and effect lines still run).
void JaudioDsp::render_voices() {
    static const bool disabled = std::getenv("GCRT_NOMIX") != nullptr;
    if (disabled) return;
    for (u32 i = 0; i < voice_count_; ++i) {
        if (!early()) {
            u16 mask = m_[(ACTIVE_MASK + (i >> 4)) & 0xFFF];
            if (!((mask << (i & 15)) & 0x8000)) continue;
        }
        render_voice(voice_table_ + i * 0x180);
    }
}

void JaudioDsp::finish_block() {
    modulated_delay();
    add_scaled(BUS_AUX_CENTER, BUS_FRONT_LEFT, kBlock, 0x5A82);
    add_scaled(BUS_AUX_CENTER, BUS_FRONT_RIGHT, kBlock, 0x5A82);
    if (!early()) {
        add_scaled(BUS_REAR_RIGHT, BUS_FRONT_LEFT + 40, 40, kRearCross);
        add_scaled(BUS_REAR_RIGHT, BUS_FRONT_RIGHT + 40, 40, kUnity);
    }
    master_gain(BUS_FRONT_LEFT);
    dma_out(out_left_, BUS_FRONT_LEFT, kBlock);
    add_scaled(BUS_AUX_RIGHT_SUBTRACT, BUS_FRONT_RIGHT, kBlock, static_cast<s16>(0x8000));
    master_gain(BUS_FRONT_RIGHT);
    dma_out(out_right_, BUS_FRONT_RIGHT, kBlock);
    out_left_ += 0xA0;
    out_right_ += 0xA0;
    finish_effects();
}

// ---- Bus arithmetic ------------------------------------------------------------------------
// dst = gain * src, doubled product, rounded; 16-bit mode.
void JaudioDsp::copy_scaled(u16 src, u16 dst, int count, s16 gain) {
    for (int n = 0; n < count; ++n) ws(dst + n, mid(round_mid(2 * s64(gain) * rs(src + n))));
}
// dst += gain * src, doubled product, rounded; 16-bit mode, so the sum wraps.
void JaudioDsp::add_scaled(u16 src, u16 dst, int count, s16 gain) {
    s16 in[kBlock];
    for (int n = 0; n < count; ++n) in[n] = rs(src + n);
    for (int n = 0; n < count; ++n) ws(dst + n, mid(round_mid(2 * s64(gain) * in[n]) + (s64(rs(dst + n)) << 16)));
}
// dst += gain * src / 65536 with saturation. The gain moves by delta (a 16.16 step)
// once, after the 65th sample. Returns the new gain.
JaudioDsp::s16 JaudioDsp::mix_ramp(u16 src, u16 dst, s16 gain, s32 delta) {
    s64 g = s64(gain) << 16;
    if (g == 0 && delta == 0) return gain;
    s16 gm = gain;
#if defined(__aarch64__)
    // (dst << 16) + gm * src stays inside 40 bits, so the saturated middle word is
    // dst + (gm * src >> 16) saturated to 16 bits: eight samples at a time.
    if ((src & 0xFFF) + kBlock <= 0x1000 && (dst & 0xFFF) + kBlock <= 0x1000) {
        const s16* in = reinterpret_cast<const s16*>(&m_[src & 0xFFF]);
        s16* out = reinterpret_cast<s16*>(&m_[dst & 0xFFF]);
        const auto run = [&](int from, int to, s16 k) {
            const int16x4_t kk = vdup_n_s16(k);
            int n = from;
            for (; n + 8 <= to; n += 8) {
                const int16x8_t x = vld1q_s16(in + n);
                const int16x4_t lo = vshrn_n_s32(vmull_s16(vget_low_s16(x), kk), 16);
                const int16x4_t hi = vshrn_n_s32(vmull_s16(vget_high_s16(x), kk), 16);
                vst1q_s16(out + n, vqaddq_s16(vld1q_s16(out + n), vcombine_s16(lo, hi)));
            }
            for (; n < to; ++n) out[n] = mid_sat((s64(out[n]) << 16) + s64(k) * in[n]);
        };
        run(0, 65, gm);
        g = wrap40(g + delta);
        run(65, kBlock, mid(g));
        return mid_sat(g);
    }
#endif
    for (int n = 0; n < kBlock; ++n) {
        if (n == 65) {
            g = wrap40(g + delta);
            gm = mid(g);
        }
        ws(dst + n, mid_sat((s64(rs(dst + n)) << 16) + s64(gm) * rs(src + n)));
    }
    return mid_sat(g);
}
// Q4.12 master level with saturation.
void JaudioDsp::master_gain(u16 bus) {
    for (int n = 0; n < kBlock; ++n) ws(bus + n, mid_sat(wrap40((s64(rs(bus + n)) * master_gain_) << 4)));
}
// In-place FIR: data[i] = sum c[k] * data[i + k]; doubled products, rounded, 16-bit mode.
void JaudioDsp::fir(u16 coefs, u16 data, int taps) {
    copy_words(coefs, COEF_SCRATCH, taps);
    if (taps != 2 && taps != 4 && taps != 8) report("FIR with %u taps (coefficient wrap unverified)", taps);
    for (int i = 0; i < kBlock; ++i) {
        s64 p = 0;
        for (int k = 0; k < taps; ++k) p += 2 * s64(rs(COEF_SCRATCH + k)) * rs(data + i + k);
        ws(data + i, mid(round_mid(p)));
    }
}

// ---- Effect lines --------------------------------------------------------------------------
// Each line: 16 parameter words (flags, ring length in blocks, ring address in main
// memory, two sends {bus, gain}, 8 FIR coefficients) and a work buffer of 8 history
// samples plus 80, 0x60 words apart from 0x0DC0. The ring block read here is written
// back with the voices' sends added at the end of the block.
void JaudioDsp::load_fx_line(int line) { copy_words(FX_PARAMS + line * 16, FX_LINE, 16); }
JaudioDsp::u32 JaudioDsp::fx_ring_address(int line) {
    return r32(FX_LINE + 2) + u32(s32(rs(FX_RING_INDEX + line)) * 0xA0);
}

void JaudioDsp::prepare_effects() {
    for (int line = 0; line < 4; ++line) {
        u16 work = static_cast<u16>(BUS_FX_0 + line * 0x60);
        load_fx_line(line);
        u16 flags = m_[FX_LINE];
        if (!flags) continue;
        copy_words(work + kBlock, work, 8);
        dma_in(fx_ring_address(line), work + 8, kBlock);
        if (flags & 1) fir(FX_LINE + 8, work, 8);
        if (u16 bus = m_[FX_LINE + 4]) add_scaled(work, bus, kBlock, rs(FX_LINE + 5));
        if (u16 bus = m_[FX_LINE + 6]) add_scaled(work, bus, kBlock, rs(FX_LINE + 7));
        if (flags & 2) fir(FX_LINE + 8, work, 8);
    }
    if (early()) return;
    add_scaled(BUS_REAR_LEFT_FX, BUS_FRONT_LEFT_FX, kBlock, kUnity);
    add_scaled(BUS_REAR_LEFT_FX, BUS_FRONT_RIGHT_FX, kBlock, kRearCross);
    add_scaled(BUS_REAR_RIGHT_FX + 40, BUS_FRONT_LEFT_FX, 40, kRearCross);
    add_scaled(BUS_REAR_RIGHT_FX + 40, BUS_FRONT_RIGHT_FX, 40, kUnity);
    for (int n = 0; n < kBlock; ++n) m_[BUS_REAR_LEFT_FX + n] = m_[BUS_REAR_RIGHT_FX + n] = 0;
}

void JaudioDsp::finish_effects() {
    if (!early()) {
        add_scaled(BUS_REAR_RIGHT_FX, BUS_FRONT_LEFT_FX + 40, 40, kRearCross);
        add_scaled(BUS_REAR_RIGHT_FX, BUS_FRONT_RIGHT_FX + 40, 40, kUnity);
    }
    for (int line = 0; line < 4; ++line) {
        u16 work = static_cast<u16>(BUS_FX_0 + line * 0x60);
        load_fx_line(line);
        if (!m_[FX_LINE]) continue;
        u32 address = fx_ring_address(line);
        u16 next = static_cast<u16>(m_[FX_RING_INDEX + line] + 1);
        if (s32(next) >= s32(rs(FX_LINE + 1))) next = 0;
        m_[FX_RING_INDEX + line] = next;
        dma_out(address, work, kBlock);
    }
}

// Patterns 2 and 3 evolve once per block (sources 0x0B and 0x0C read them). This
// follows the microcode's register flow: both accumulators enter with only their middle
// words loaded in 16-bit mode. Sunshine clears them just before; the early microcode
// leaves earlier values in their high and low parts, taken here as cleared.
void JaudioDsp::update_patterns() {
    s16 a = rs(PATTERN_2 + 62), b = rs(PATTERN_2 + 63);
    u16 rd = PATTERN_2, wr = PATTERN_2;
    s64 p = s64(a) * b;
    s64 acc0 = s64(m_[rd++]) << 16;
    s64 acc1 = acc0;
    s16 ax0h = b;
    for (int i = 0; i < 32; ++i) {
        acc0 = wrap40(-acc0);
        acc0 = wrap40(acc0 + p);
        p = s64(mid(acc1)) * ax0h;
        ax0h = rs(rd++);
        ws(wr++, mid(acc0));
        acc1 = s64(ax0h) << 16;
        acc1 = wrap40(acc1 + p);
        p = s64(mid(acc0)) * ax0h;
        ax0h = rs(rd++);
        acc1 = wrap40(acc1 << 1);
        ws(wr++, mid(acc1));
        acc0 = s64(ax0h) << 16;
    }
    // Pattern 3: a ramp whose slope and direction come from its last two words.
    s16 h = rs(PATTERN_3 + 62), l = rs(PATTERN_3 + 63);
    acc1 = wrap40(static_cast<s64>((static_cast<u64>(acc1) & ~u64(0xFFFF0000)) | (u64(u16(l)) << 16)));
    p = s64(l) * h;
    acc0 = wrap40(round_mid(p) + (s64(h) << 16));
    s16 step = static_cast<s16>((u16(mid(acc0)) & 0x1FF) | 0x2000);
    if (acc1 > 0) step = static_cast<s16>(-step);
    acc1 = wrap40(acc1 + (s64(step) << 16));
    for (int i = 0; i < 64; ++i) {
        ws(PATTERN_3 + i, mid(acc1));
        acc1 = wrap40(acc1 + (s64(step) << 16));
    }
}

// Modulated delay: the bus is written into a 160-sample ring in alternating halves and
// read back at a step of 1 +- 1/128 that flips when the read point nears the write
// point (thresholds 0x30 and 0x10), then added to both fronts at half level. The ring
// ends on a 256-word boundary, where the DSP's address wrap takes the read point back.
void JaudioDsp::init_modulated_delay() {
    m_[MD_DIRECTION] = 0xFFFF;
    m_[MD_WRITE] = 0;
    m_[MD_FRACTION] = 0;
    m_[MD_STEP_DELTA] = 0x100;
    m_[MD_STEP] = static_cast<u16>(0x8000 - 0x100);
    m_[MD_THRESHOLD_0] = 0x30;
    m_[MD_THRESHOLD_1] = 0x10;
    m_[MD_READ] = 0x96;
}
void JaudioDsp::modulated_delay() {
    u16 bus = early() ? SCRATCH_B00 : SCRATCH_A00, ring = static_cast<u16>(bus + 0x60);
    u16 write = m_[MD_WRITE];
    m_[MD_WRITE] = write ? 0 : 0x50;
    copy_words(bus, ring + write, kBlock);
    u16 read = m_[MD_READ];
    s32 gap = 0xA0 - s32(read);
    if (gap >= 0x50) gap -= 0x50;
    s16 direction = rs(MD_DIRECTION);
    if (direction < 0 ? s32(rs(MD_THRESHOLD_0)) - gap <= 0 : gap - s32(rs(MD_THRESHOLD_1)) <= 0) {
        m_[MD_STEP] = static_cast<u16>(direction < 0 ? 0x8000 + m_[MD_STEP_DELTA] : 0x8000 - m_[MD_STEP_DELTA]);
        ws(MD_DIRECTION, -direction);
    }
    u32 step = u32(m_[MD_STEP]) << 1, frac = m_[MD_FRACTION];
    for (int n = 0; n < kBlock; ++n) {
        s16 x = rs(ring + read);
        frac += step;
        read = static_cast<u16>((read + (frac >> 16)) % 0xA0);
        frac &= 0xFFFF;
        ws(bus + n, x);
    }
    m_[MD_READ] = read;
    m_[MD_FRACTION] = static_cast<u16>(frac);
    add_scaled(bus, BUS_FRONT_RIGHT, kBlock, 0x3FFF);
    add_scaled(bus, BUS_FRONT_LEFT, kBlock, 0x3FFF);
}

// ---- Voices --------------------------------------------------------------------------------
void JaudioDsp::render_voice(u32 address) {
    dma_in(address, VOICE, 0xC0);
    if (!m_[V_ENABLED] || m_[V_DONE]) return;
    if (verbosity > 2 && m_[V_RESET])
        log("DSP: JAudio voice %08x start: type %02x pitch %04x base %08x end %08x loop %d/%08x filter %02x "
            "dist %04x mixer %s sends %04x:%04x %04x:%04x %04x:%04x delays %u: %04x %04x %04x iir %04x %04x %04x %04x\n",
            address, m_[V_TYPE], m_[V_PITCH], r32(V_BASE), r32(V_END), m_[V_LOOP], r32(V_LOOP_START), m_[V_FILTER],
            m_[V_DIST_COEF], (!early() && m_[V_AUTO_MIXER]) ? "auto" : "sends", m_[V_SENDS], m_[V_SENDS + 1],
            m_[V_SENDS + 4], m_[V_SENDS + 5], m_[V_SENDS + 8], m_[V_SENDS + 9], m_[V_DELAY_COUNT], m_[V_SENDS + 3],
            m_[V_SENDS + 7], m_[V_SENDS + 11], m_[V_IIR_COEFS], m_[V_IIR_COEFS + 1], m_[V_IIR_COEFS + 2],
            m_[V_IIR_COEFS + 3]);
    m_[SAVED_HELD_SAMPLE] = m_[V_HELD];
    if (m_[V_PAUSED]) {
        for (int n = 0; n < kBlock; ++n) m_[VOICE_OUT + n] = m_[V_HELD];
    } else {
        u16 type = m_[V_TYPE];
        u16 needed = static_cast<u16>((u32(m_[V_PHASE]) + 80u * m_[V_PITCH]) >> 12);
        ar3_ = RAW;
        bool afc = type == SOURCE_AFC_HQ || (!early() && type == SOURCE_AFC_LQ);
        bool pcm = type == SOURCE_PCM8_ARAM || (!early() && type == SOURCE_PCM16_ARAM);
        if (afc) {
            if (type == SOURCE_AFC_LQ || (m_[V_LOOP] && ((m_[V_LOOP_START + 1] | m_[V_END + 1]) & 0xF)))
                afc_buffered(needed);
            else
                afc_aligned(needed);
            resample();
        } else if (pcm) {
            pcm_aram(needed, type == SOURCE_PCM16_ARAM);
            resample();
        } else if (type == SOURCE_PCM16_MRAM_DIRECT) {
            ar3_ = VOICE_OUT;
            pcm_mram(kBlock);
        } else if (type == SOURCE_PCM16_MRAM_RESAMPLED) {
            pcm_mram(needed);
            resample();
        } else {
            generate(type);
        }
    }
    filter_voice();
    if (early()) {
        mix_sends();
    } else {
        m_[V_AUTO_MIXER_COPY] = m_[V_AUTO_MIXER];
        if (m_[V_AUTO_MIXER]) mix_positional();
        else mix_sends();
    }
    if (!m_[V_PAUSED]) m_[V_RESET] = 0;
    dma_out(address, VOICE, 0x80);
}

void JaudioDsp::zero_fill(int count) {
    for (int n = 0; n < count; ++n) ws(ar3_++, 0);
    m_[V_DONE] = 1;
}
void JaudioDsp::copy_run(u32 from, int count) {
    for (int n = 0; n < count; ++n) m_[ar3_++ & 0xFFF] = m_[(from + n) & 0xFFF];
}

// AFC: 16-sample blocks of 9 bytes (4-bit residuals) or 5 bytes (2-bit), a header byte
// of scale and predictor index into the 16 coefficient pairs. Each sample uses the
// previous accumulator's middle word as the processor holds it and the one before as
// it was stored (saturated).
void JaudioDsp::decode_afc(u32 dst) {
    u8 header = accel8();
    int shift = 17 - (header >> 4);
    s16 c1 = rs(AFC_COEFS + (header & 15) * 2), c2 = rs(AFC_COEFS + (header & 15) * 2 + 1);
    s16 residual[16];
    if (m_[V_TYPE] == SOURCE_AFC_LQ && !early()) {
        for (int b = 0; b < 4; ++b) {
            u8 x = accel8();
            for (int j = 0; j < 4; ++j) residual[b * 4 + j] = static_cast<s16>(((x << (2 * j)) & 0xC0) << 8);
        }
    } else {
        for (int b = 0; b < 8; ++b) {
            u8 x = accel8();
            residual[b * 2] = static_cast<s16>((x & 0xF0) << 8);
            residual[b * 2 + 1] = static_cast<s16>((x & 0x0F) << 12);
        }
    }
    s16 last_raw = rs(V_AFC_NEWER), last = last_raw, before = rs(V_AFC_OLDER);
    for (int k = 0; k < 16; ++k) {
        s64 acc = (s64(residual[k]) << 16) >> shift;
        acc = wrap40((acc + s64(c1) * last_raw + s64(c2) * before) << 5);
        s16 out = mid_sat(acc);
        ws(dst + k, out);
        before = last;
        last = out;
        last_raw = mid(acc);
    }
}

// General AFC reader (5-byte sources, and 9-byte ones with unaligned loops): the
// position in samples is authoritative and blocks decode into the voice's 16-sample
// buffer, from which runs are copied out.
void JaudioDsp::compute_afc_position() {
    u32 pos = r32(V_POSITION);
    u32 block = static_cast<u32>(s32(pos) >> 4);
    w32(V_BLOCK, block);
    u16 type = m_[V_TYPE];
    s64 offset = s64(s16(block >> 16)) * type * 65536 + s64(u16(block)) * type;
    w32(V_ARAM, static_cast<u32>(offset + s32(r32(V_BASE))));
    m_[V_AFC_INDEX] = pos & 0xF;
    w32(V_REMAINING, r32(V_END) - pos);
}
void JaudioDsp::decode_afc_buffered() {
    if (!configured_) {
        configured_ = true;
        accel_ = r32(V_ARAM);
    }
    decode_afc(V_AFC_DECODED);
}
void JaudioDsp::afc_silence() {
    for (int k = 0; k < 16; ++k) m_[V_AFC_DECODED + k] = 0;
    m_[V_DONE] = 1;
    m_[V_AFC_COUNT] = 16;
    m_[V_AFC_INDEX] = 0;
}
void JaudioDsp::afc_block_boundary() {
    if (!early() && m_[V_DONE]) return afc_silence();
    u32 end = r32(V_END);
    if (s64(end >> 4) - s32(r32(V_BLOCK)) != 0) {
        decode_afc_buffered();
        m_[V_AFC_COUNT] = 16;
        m_[V_AFC_INDEX] = 0;
        return;
    }
    if (s64(r32(V_POSITION)) - s32(end) < 0) {   // last, partial block
        decode_afc_buffered();
        m_[V_AFC_COUNT] = m_[V_REMAINING + 1];
        m_[V_AFC_INDEX] = 0;
        return;
    }
    if (!early() && !m_[V_LOOP]) return afc_silence();
    w32(V_POSITION, r32(V_LOOP_START));
    m_[V_AFC_COUNT] = 16;
    compute_afc_position();
    m_[V_AFC_NEWER] = m_[V_LOOP_NEWER];
    m_[V_AFC_OLDER] = m_[V_LOOP_OLDER];
    configured_ = false;
    decode_afc_buffered();
}
// Copies decoded samples from the buffer index on and advances the position.
void JaudioDsp::copy_afc_retained(int& n, bool bounded) {
    u16 index = m_[V_AFC_INDEX];
    int k = u16(m_[V_AFC_COUNT] - index);
    if (bounded && k >= n) k = n;
    copy_run(V_AFC_DECODED + index, k);
    w32(V_POSITION, r32(V_POSITION) + u32(k));
    n -= k;
}
void JaudioDsp::afc_buffered(int n) {
    compute_afc_position();
    configured_ = false;
    for (int guard = 0; guard < 4096; ++guard) {
        if (r32(V_REMAINING) == 0) {
            afc_block_boundary();
            if (m_[V_AFC_INDEX] == 0) {
                copy_afc_retained(n, true);
                if (n <= 0) return;
                compute_afc_position();
            }
        }
        if (s64(r32(V_REMAINING)) - n < 0) {
            if (m_[V_AFC_INDEX] == 0) {
                afc_block_boundary();
                if (!early() && m_[V_DONE]) {
                    for (int k = 0; k < n; ++k) ws(ar3_++, 0);
                    return;
                }
            }
            copy_afc_retained(n, false);
            if (n <= 0) return report("AFC reader overran its count", 0);
            compute_afc_position();
            continue;
        }
        for (; guard < 4096; ++guard) {
            if (m_[V_AFC_INDEX] == 0) afc_block_boundary();
            copy_afc_retained(n, true);
            if (n <= 0) return;
            compute_afc_position();
        }
    }
    report("AFC reader made no progress", 0);
}

// Aligned 9-byte AFC: decodes whole blocks straight into the raw buffer and counts
// blocks to the end; the position only moves on reset and loop.
void JaudioDsp::compute_afc_hq_position() {
    u32 pos = r32(V_POSITION);
    s64 diff = s64(r32(V_END)) - s32(pos);
    w32(V_REMAINING, static_cast<u32>(diff >> 4));
    u32 block = (pos >> 4) | (diff < 0 ? 0xF0000000u : 0);
    u16 type = m_[V_TYPE];
    s64 offset = s64(s16(block >> 16)) * type * 65536 + s64(s16(block)) * type;
    w32(V_ARAM, static_cast<u32>(offset + s32(r32(V_BASE))));
    m_[V_END_REACHED] = 0;
    m_[V_AFC_INDEX] = 0;
}
void JaudioDsp::afc_aligned(int n) {
    if (m_[V_DONE]) return zero_fill(n);
    if (m_[V_RESET]) {
        w32(V_POSITION, 0);
        m_[V_AFC_OLDER] = m_[V_AFC_NEWER] = 0;
        compute_afc_hq_position();
    }
    if (u16 index = m_[V_AFC_INDEX]) {
        int available = 16 - index;
        copy_run(V_AFC_DECODED + index, available);
        if (n < available) {
            m_[V_AFC_INDEX] = static_cast<u16>(index + n);
            return;
        }
        if (n == available) {
            m_[V_AFC_INDEX] = 0;
            return;
        }
        n -= available;
    }
    bool at_end = m_[V_END_REACHED] != 0;
    for (int guard = 0; guard < 4096; ++guard) {
        if (at_end) {
            m_[V_END_REACHED] = 0;
            if (!m_[V_LOOP]) return zero_fill(n);
            w32(V_POSITION, r32(V_LOOP_START));
            compute_afc_hq_position();
            m_[V_AFC_NEWER] = m_[V_LOOP_NEWER];
            m_[V_AFC_OLDER] = m_[V_LOOP_OLDER];
        }
        accel_ = r32(V_ARAM);
        u32 consumed = 0;
        for (;;) {
            decode_afc(ar3_);
            m_[V_AFC_OLDER] = m_[(ar3_ + 14) & 0xFFF];
            m_[V_AFC_NEWER] = m_[(ar3_ + 15) & 0xFFF];
            ar3_ += 16;
            consumed += m_[V_TYPE];
            u32 remaining = r32(V_REMAINING) - 1;
            if (remaining == 0) m_[V_END_REACHED] = 1;
            w32(V_REMAINING, remaining);
            if (n > 16) {
                n -= 16;
                if (m_[V_END_REACHED]) break;
                continue;
            }
            m_[V_AFC_INDEX] = static_cast<u16>(n == 16 ? 0 : n);
            copy_words(ar3_ - 16, V_AFC_DECODED, 16);
            w32(V_ARAM, r32(V_ARAM) + u16(consumed));
            return;
        }
        at_end = true;
    }
    report("AFC reader made no progress", 0);
}

// PCM8 and PCM16 in ARAM: remaining length and ARAM address advance; the position only
// moves on reset and loop.
void JaudioDsp::compute_pcm_position(bool wide) {
    u32 pos = r32(V_POSITION);
    w32(V_REMAINING, r32(V_END) - pos);
    w32(V_ARAM, r32(V_BASE) + (wide ? pos * 2 : pos));
    m_[V_END_REACHED] = 0;
}
void JaudioDsp::read_pcm(int count, bool wide) {
    for (int k = 0; k < count; ++k) ws(ar3_++, wide ? accel16() : static_cast<s16>(accel8() << 8));
}
void JaudioDsp::pcm_aram(int n, bool wide) {
    if (m_[V_DONE]) return zero_fill(n);
    if (m_[V_RESET]) {
        w32(V_POSITION, 0);
        compute_pcm_position(wide);
    }
    bool at_end = m_[V_END_REACHED] != 0;
    for (int guard = 0; guard < 4096; ++guard) {
        if (at_end) {
            m_[V_END_REACHED] = 0;
            if (!m_[V_LOOP]) return zero_fill(n);
            w32(V_POSITION, r32(V_LOOP_START));
            compute_pcm_position(wide);
        }
        accel_ = r32(V_ARAM);
        if (wide) accel_ &= ~1u;
        u32 remaining = r32(V_REMAINING);
        if ((remaining >> 16) == 0 && s32(remaining & 0xFFFF) - n < 0) {
            int k = remaining & 0xFFFF;
            read_pcm(k, wide);
            n -= k;
            m_[V_END_REACHED] = 1;
            at_end = true;
            continue;
        }
        read_pcm(n, wide);
        remaining -= u32(n);
        if (remaining == 0) m_[V_END_REACHED] = 1;
        w32(V_REMAINING, remaining);
        w32(V_ARAM, r32(V_ARAM) + u32(wide ? 2 * n : n));
        return;
    }
    report("PCM reader made no progress", 0);
}

// PCM16 in main memory (CPU-decoded streams): a ring of END.high samples from BASE;
// the high word of the position is the index into it. At the ring's end the reader
// continues from the loop address.
void JaudioDsp::mram_samples(u16 count) {
    if (!count) return;
    u16 scratch = early() ? SCRATCH_A00 : SCRATCH_B00;
    u16 index = m_[V_POSITION];
    u16 odd = index & 1;
    dma_in(r32(V_BASE) + u32(index) * 2 - 2u * odd, scratch, u32(count) + 2);
    m_[V_POSITION] = static_cast<u16>(index + count);
    copy_run(scratch + odd, count);
}
void JaudioDsp::pcm_mram(int n) {
    u32 remaining = r32(V_REMAINING);
    if (s64(remaining) - n < 0) {
        u16 k = static_cast<u16>(remaining);
        mram_samples(k);
        s16 last = rs(ar3_ - 1);
        for (int i = 0; i < u16(n - k); ++i) ws(ar3_++, last);
        m_[V_DONE] = 1;
        m_[V_REMAINING + 1] = 0;
        return;
    }
    w32(V_REMAINING, remaining - u32(n));
    s32 to_wrap = s32(m_[V_END]) - s32(m_[V_POSITION]);
    m_[V_BLOCK] = static_cast<u16>(to_wrap);
    if (to_wrap - n >= 0) return mram_samples(static_cast<u16>(n));
    mram_samples(static_cast<u16>(to_wrap));
    u16 rest = static_cast<u16>(n - rs(V_BLOCK));
    m_[V_POSITION] = 0;
    w32(V_BASE, r32(V_LOOP_START));
    mram_samples(rest);
}

// Oscillators and table patterns write 80 samples directly; the phase is 16-bit.
void JaudioDsp::generate(u16 type) {
    u16 pitch = m_[V_PITCH];
    u16 phase = m_[V_PHASE];
    u16 out = VOICE_OUT;
    switch (type & 0x3F) {
    case 0x00:
    case 0x02: {   // square; 0x02 adds the 0x0CA0 bus to the step and clears it
        u32 acc = u32(phase) << 1;
        for (int n = 0; n < kBlock; ++n) {
            ws(out + n, ((acc >> 16) & 1) ? 0x4000 : -0x4000);
            u32 step = pitch;
            if ((type & 0x3F) == 0x02) {
                step = (m_[BUS_REAR_RIGHT + n] + u32(pitch)) & 0xFFFF;
                m_[BUS_REAR_RIGHT + n] = 0;
            }
            acc += step;
        }
        phase = static_cast<u16>(acc >> 1);
        break;
    }
    case 0x03: {   // quarter-duty square
        u32 acc = u32(phase) << 2;
        for (int n = 0; n < kBlock; ++n) {
            ws(out + n, ((acc >> 16) & 3) == 3 ? 0x4000 : -0x4000);
            acc += u32(pitch) << 1;
        }
        phase = static_cast<u16>(acc >> 2);
        break;
    }
    case 0x01:   // saw: the phase itself
        for (int n = 0; n < kBlock; ++n) {
            m_[out + n] = phase;
            phase = static_cast<u16>(phase + (pitch >> 1));
        }
        break;
    case 0x06:   // constant: the pitch word
        for (int n = 0; n < kBlock; ++n) m_[out + n] = pitch;
        break;
    case 0x04: phase = pattern(PATTERN_1, phase, pitch, false); break;
    case 0x07: phase = pattern(PATTERN_0, phase, pitch, false); break;
    case 0x0A: phase = pattern(PATTERN_0, phase, pitch, true); break;
    case 0x0B: phase = pattern(PATTERN_2, phase, pitch, false); break;
    case 0x0C: phase = pattern(PATTERN_3, phase, pitch, false); break;
    case 0x05: case 0x08: case 0x09: case 0x0D: case 0x0E: case 0x0F: case 0x10:
        break;
    default:
        report("source type %04x not modelled", type);
        break;
    }
    m_[V_PHASE] = phase;
}
// 64-entry table stepped by a 6.16 phase (6.10 when stored); the modulated form adds
// pitch * bus 0x0CA0 / 1024 per sample.
JaudioDsp::u16 JaudioDsp::pattern(u16 table, u16 phase, u16 pitch, bool modulated) {
    u32 index = (phase >> 10) & 63;
    s64 frac = u16(phase << 6);
    s64 step = s64(pitch) << 5;
    for (int n = 0; n < kBlock; ++n) {
        s16 value = rs(table + index);
        s64 sum = frac;
        if (modulated) sum = ((frac << 10) + s64(s16(pitch)) * rs(BUS_REAR_RIGHT + n)) >> 10;
        sum += step;
        index = (index + u32(sum >> 16)) & 63;
        frac = sum & 0xFFFF;
        ws(VOICE_OUT + n, value);
    }
    return static_cast<u16>((index << 10) | (u16(frac) >> 6));
}

// 64-phase 4-tap interpolation of the raw buffer into 80 samples, four history samples
// prepended; at a step of 4.0 or more the oldest tap is taken unfiltered.
void JaudioDsp::resample() {
    copy_words(V_RESAMPLE_HISTORY, RAW - 4, 4);
    u16 pitch = m_[V_PITCH];
    u32 step = u32(pitch) << 4;
    u32 pos = (u32(RAW - 4) << 16) | u16(m_[V_PHASE] << 4);
    s16 last = 0;
    if ((pitch >> 12) >= 4) {
        for (int n = 0; n < kBlock; ++n) {
            last = rs(pos >> 16);
            ws(VOICE_OUT + n, last);
            pos += step;
        }
    } else {
#if defined(__aarch64__)
        // Taps, coefficients and output inside data memory without wrapping: a 4-lane
        // product summed in 64 bits, the same value as the scalar loop.
        const u32 last_at = (pos + step * (kBlock - 1)) >> 16;
        if (last_at + 3 < 0x1000 && (RESAMPLE_COEFS & 0xFFF) + 256 <= 0x1000 && (VOICE_OUT & 0xFFF) + kBlock <= 0x1000) {
            const s16* m = reinterpret_cast<const s16*>(m_);
            s16* out = reinterpret_cast<s16*>(&m_[VOICE_OUT & 0xFFF]);
            for (int n = 0; n < kBlock; ++n) {
                const int32x4_t products = vmull_s16(vld1_s16(m + (RESAMPLE_COEFS & 0xFFF) + ((pos >> 10) & 63) * 4),
                                                     vld1_s16(m + (pos >> 16)));
                const s64 acc = round_mid(2 * vaddvq_s64(vpaddlq_s32(products)));
                out[n] = mid_sat(acc);
                last = mid(acc);
                pos += step;
            }
            ws(V_HELD, last);
            m_[V_PHASE] = static_cast<u16>((pos & 0xFFFF) >> 4);
            copy_words(pos >> 16, V_RESAMPLE_HISTORY, 4);
            return;
        }
#endif
        for (int n = 0; n < kBlock; ++n) {
            u32 taps = ((pos >> 10) & 63) * 4;
            u32 at = pos >> 16;
            s64 p = 0;
            for (int t = 0; t < 4; ++t) p += 2 * s64(rs(RESAMPLE_COEFS + taps + t)) * rs(at + t);
            s64 acc = round_mid(p);
            ws(VOICE_OUT + n, mid_sat(acc));
            last = mid(acc);
            pos += step;
        }
    }
    ws(V_HELD, last);
    m_[V_PHASE] = static_cast<u16>((pos & 0xFFFF) >> 4);
    copy_words(pos >> 16, V_RESAMPLE_HISTORY, 4);
}

// ---- Voice filters ---------------------------------------------------------------------------
void JaudioDsp::filter_voice() {
    if (s16 k = rs(V_DIST_COEF)) {
        // y[n] = y[n-1] + k * (x[n] - x[n-1]) / 128
        if (m_[V_RESET]) m_[V_DIST_Y] = m_[V_DIST_X] = 0;
        s16 x_prev = rs(V_DIST_X), y = rs(V_DIST_Y);
        for (int n = 0; n < kBlock; ++n) {
            s16 x = rs(VOICE_OUT + n);
            s16 d = static_cast<s16>(x - x_prev);
            y = mid_sat(((2 * s64(d) * k) << 8) + (s64(y) << 16));
            ws(VOICE_OUT + n, y);
            x_prev = x;
        }
        ws(V_DIST_Y, y);
        ws(V_DIST_X, x_prev);
    }
    filtered_ = VOICE_OUT;
    if (int taps = m_[V_FILTER] & 0x1F) {
        filtered_ = static_cast<u16>(VOICE_OUT - taps);
        copy_words(V_FIR_HISTORY, filtered_, taps);
        fir(V_FIR_COEFS, filtered_, taps);
        copy_words(filtered_ + kBlock, V_FIR_HISTORY, taps);
    }
    if ((m_[V_FILTER] & 0x20) && !(m_[V_IIR_COEFS + 3] == 0 && m_[V_IIR_COEFS + 2] == 0 &&
                                   m_[V_IIR_COEFS + 1] == 0 && m_[V_IIR_COEFS] == 0x7FFF)) {
        // y[n] = 2 * (c0 x[n-1] + c1 x[n-2] + c2 y[n-1] + c3 y[n-2]) / 65536, saturated.
        s16 c[4];
        for (int i = 0; i < 4; ++i) c[i] = rs(V_IIR_COEFS + i);
        s16 x1 = rs(V_IIR_HISTORY), x2 = rs(V_IIR_HISTORY + 1);
        s16 y1 = rs(V_IIR_HISTORY + 2), y2 = rs(V_IIR_HISTORY + 3);
        for (int n = 0; n < kBlock; ++n) {
            s16 x = rs(filtered_ + n);
            s16 y = mid_sat(2 * (s64(c[0]) * x1 + s64(c[1]) * x2 + s64(c[2]) * y1 + s64(c[3]) * y2));
            ws(filtered_ + n, y);
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;
        }
        ws(V_IIR_HISTORY, x1);
        ws(V_IIR_HISTORY + 1, x2);
        ws(V_IIR_HISTORY + 2, y1);
        ws(V_IIR_HISTORY + 3, y2);
    }
}

// ---- Voice mixers ------------------------------------------------------------------------------
// Six routed sends {bus, target, current, delay word}. The current gain moves 1/32 of
// the way to the target per block; the delay word's low byte (samples taken from the
// previous block's tail) steps once per block toward its high byte. A stop request
// halves the targets and ends the voice once every current gain is zero.
void JaudioDsp::mix_sends() {
    u16 kept = m_[V_DELAY_COUNT];
    for (int j = 0; j < kept; ++j) m_[(filtered_ - 1 - j) & 0xFFF] = m_[(V_DELAY_HISTORY_END - j) & 0xFFF];
    if (m_[V_STOP]) {
        bool any = false;
        for (int k = 0; k < 6; ++k) {
            u16 current = m_[V_SENDS + k * 4 + 2];
            m_[V_SENDS + k * 4 + 1] = current >> 1;
            any |= current != 0;
        }
        if (!any) m_[V_DONE] = 1;
    }
    for (int k = 0; k < 6; ++k) {
        u16 send = static_cast<u16>(V_SENDS + k * 4);
        u16 bus = m_[send];
        s16 target = rs(send + 1), current = rs(send + 2);
        s32 delta = static_cast<s32>((s64(target - current) << 16) >> 5);
        u16 word = m_[send + 3];
        u16 delay = word & 0xFF;
        u16 wanted = (word & 0x8000) ? (0xFF00 | (word >> 8)) : (word >> 8);
        if (wanted != delay) ws(send + 3, mid_sat((s64(s16(word)) + (wanted > delay ? 1 : -1)) << 16));
        if (bus) current = mix_ramp(static_cast<u16>(filtered_ - delay), bus, current, delta);
        ws(send + 2, current);
    }
    for (int j = 0; j < kept; ++j) m_[(V_DELAY_HISTORY_END - j) & 0xFFF] = m_[(filtered_ + 79 - j) & 0xFFF];
}

// Automatic mixer (Sunshine): pan (high byte) and front/rear position (low byte) index a
// quarter sine table into four dry and four effect gains; its volume jumps to the
// target each block after a single 1/32 step.
void JaudioDsp::mix_positional() {
    if (m_[V_STOP]) {
        u16 current = m_[V_AM_CURRENT];
        m_[V_AM_TARGET] = current >> 1;
        if (current == 0) m_[V_DONE] = 1;
    }
    u16 word = m_[V_PAN];
    u16 pan = (word & 0x8000) ? (0xFF00 | (word >> 8)) : (word >> 8);
    u16 rear = word & 0x7F;
    auto sine = [&](u16 i) { return rs(SINE_TABLE + i); };
    s64 left = sine(pan ^ 0x7F), right = sine(pan), front = sine(rear ^ 0x7F), back = sine(rear);
    s16 g[4] = {mid_sat(left * front), mid_sat(right * front), mid_sat(back * left), mid_sat(right * back)};
    for (int i = 0; i < 4; ++i) ws(SCRATCH_B00 + i, g[i]);
    s16 current = rs(V_AM_CURRENT), fx = rs(V_FX);
    s16 fx_current = mid_sat(2 * s64(fx) * current);
    s16 target = rs(V_AM_TARGET);
    ws(V_AM_CURRENT, target);
    s16 delta = mid_sat((s64(target) - current) << 16);
    s16 gain[8], step[8];
    for (int i = 0; i < 4; ++i) {
        gain[i] = mid_sat(s64(g[i]) * current);
        gain[4 + i] = mid_sat(s64(gain[i]) * fx_current);
        step[i] = mid_sat(s64(u16(g[i])) * delta);
        step[4 + i] = mid_sat(s64(step[i]) * fx);
    }
    for (int i = 0; i < 8; ++i) {
        ws(SCRATCH_B00 + 4 + i, gain[i]);
        ws(SCRATCH_B00 + 12 + i, step[i]);
    }
    for (int i = 0; i < 8; ++i)
        mix_ramp(filtered_, kPositionalDestinations[i], gain[i], static_cast<s32>((s64(step[i]) << 16) >> 5));
}

} // namespace gcrt

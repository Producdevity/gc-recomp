// Mailbox protocol of the early JAudio DSP microcode that Luigi's Mansion uploads
// (GLME01, 6112 bytes, IRAM hash 0x64d75375); the signal path is jaudio_dsp.cpp.
//
// Commands are a header mail and operands; the CPU then polls for one reply mail,
// skipping 0x88881357: 0x80 start (1 word), 0x81 setup (5: voice count, voice table,
// render tables, ADPCM coefficients, effect lines), 0x82 frame (3: sub-frames and Q4.12
// mixer level, left and right PCM buffers), 0x8B table load (2).
// A bare 0 mail renders the next 80-sample sub-frame of the current frame and raises the
// DSP interrupt without a mail; the CPU answers with the next 0 after updating its voices.
// The render runs on a DSP thread, in parallel with the CPU as on the console, and the
// interrupt follows its end; with virtual time it runs inline. A frame's sub-frames come
// back to back, so each side spins briefly before blocking and wakes the other only when
// it is blocked: a futex wake per sub-frame costs the CPU thread more than the render.
#include "jaudio_dsp.hpp"

#include "gcrt/dsp.hpp"
#include "gcrt/jaudio.hpp"
#include "gcrt/runtime.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace gcrt {
namespace {

class JaudioEarly final : public Microcode {
public:
    explicit JaudioEarly(DspApi& api) : api_(api), dsp_(api, JaudioDsp::Variant::Early) {}
    const char* name() const override { return "Jaudio (early)"; }
    void start() override {
        wait_idle();
        dsp_.reset();
        api_.send_mail(0x88880000u, false);
    }
    void mail(std::uint32_t value) override {
        wait_idle();
        if (words_.empty() && value == 0) {
            subframe();
            return;
        }
        if (words_.empty()) expected_ = length(value >> 24);
        words_.push_back(value);
        if (words_.size() < expected_) return;
        execute();
        words_.clear();
    }

private:
    static std::size_t length(std::uint32_t opcode) {
        switch (opcode) {
        case 0x81: return 5;
        case 0x82: return 3;
        case 0x8B: return 2;
        default: return 1;
        }
    }
    void execute() {
        std::uint32_t opcode = words_[0] >> 24;
        switch (opcode) {
        case 0x81: dsp_.setup(words_[0] & 0xFFFF, words_[1], words_[2], words_[3], words_[4]); break;
        case 0x82:
            subframes_ = (words_[0] >> 16) & 0xFF;
            next_ = 0;
            dsp_.begin_frame(words_[1], words_[2], words_[0] & 0xFFFF);
            break;
        case 0x80:
        case 0x8B:
            break;
        default:
            if (!reported_) log("DSP: Jaudio command %08x not modelled\n", words_[0]);
            reported_ = true;
            break;
        }
        api_.send_mail(0x88880000u | opcode, false);
    }
    void render() {
        dsp_.prepare_block();
        dsp_.render_voices();
        dsp_.finish_block();
    }
    void subframe() {
        if (next_ >= subframes_) return;
        ++next_;
        if (virtual_time) {
            render();
            api_.raise_interrupt();
            return;
        }
        if (!worker_started_) {
            worker_started_ = true;
            std::thread([this] { work(); }).detach();
        }
        job_.store(true);
        if (worker_blocked_.load()) {
            std::lock_guard<std::mutex> lock(mutex_);
            wake_.notify_one();
        }
    }
    void work() {
        for (;;) {
            if (!spin([this] { return job_.load(); })) {
                std::unique_lock<std::mutex> lock(mutex_);
                worker_blocked_.store(true);
                wake_.wait(lock, [this] { return job_.load(); });
                worker_blocked_.store(false);
            }
            render();
            schedule_at(now_ns(), [this] { api_.raise_interrupt(); });
            wake_guest();
            job_.store(false);
            if (cpu_blocked_.load()) {
                std::lock_guard<std::mutex> lock(mutex_);
                idle_.notify_all();
            }
        }
    }
    // The voice and frame state belongs to the render until it ends.
    void wait_idle() {
        if (!job_.load() || spin([this] { return !job_.load(); })) return;
        std::unique_lock<std::mutex> lock(mutex_);
        cpu_blocked_.store(true);
        idle_.wait(lock, [this] { return !job_.load(); });
        cpu_blocked_.store(false);
    }
    // Polls ready() for up to 200 us.
    template <class Ready> static bool spin(Ready ready) {
        const std::uint64_t until = now_ns() + 200000;
        for (unsigned n = 0;; ++n) {
            if (ready()) return true;
            if ((n & 63) == 63 && now_ns() >= until) return false;
#if defined(__aarch64__)
            asm volatile("yield");
#endif
        }
    }
    DspApi& api_;
    std::mutex mutex_;
    std::condition_variable wake_, idle_;
    // Sequentially consistent: a side sets its blocked flag, then checks the other's flag
    // under the mutex, so a wake is never lost between a spin and a block.
    std::atomic<bool> job_{false}, worker_blocked_{false}, cpu_blocked_{false};
    bool worker_started_ = false;
    JaudioDsp dsp_;
    std::vector<std::uint32_t> words_;
    std::size_t expected_ = 1;
    std::uint32_t subframes_ = 0, next_ = 0;
    bool reported_ = false;
};

} // namespace

Microcode* create_jaudio_early(DspApi& api) { return new JaudioEarly(api); }

} // namespace gcrt

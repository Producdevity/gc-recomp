// Mailbox protocol of the JASystem DSP microcode that Super Mario Sunshine uploads
// (GMSE01, 7392 bytes, IRAM hash 0x9a483303); the signal path is jaudio_dsp.cpp.
// Behaviour follows the microcode (doldecomp/sms libs/JSystem/src/sms_jdsp.ds) and its
// CPU side (dspproc.c, osdsp_task.c, dsptask.c, JASAudioThread.cpp, JASDSPBuf.cpp).
//
// DSP to CPU: task mails DCD1xxxx raise the DSP interrupt (0000 booted, 0004 sync, 0005
// frame end); 0000 and 0004 are followed by an F355xxxx mail without one.
// CPU to DSP: a mail N > 0 is followed by N command words, queued and run in order; a
// mail 0 is followed by two words, the 64-voice active mask, and opens the voice gate.
// CDD1000x answers a frame-end mail (3: continue with the command queue).
// Commands, by the high byte of the first word: 0x81 setup (voice count; voice table,
// render tables, ADPCM coefficients, effect lines), 0x82 render (sub-frames and Q4.12
// mixer level; left and right PCM outputs). Every command but render is acknowledged
// with DCD10004 and F355 plus the command's high half.
// Render, per 80-sample block: prepare buses and effect lines, wait for the gate, render
// the active voices, close the gate and report DCD10004 / F355FF<block>, then mix and
// write the block. After the last block: DCD10005, then wait for the task action.
//
// Blocks are split between two threads (JaudioDsp::Synthesis). The voice side runs on the
// guest thread when the voice mask arrives, before the block's mails: voice positions,
// flags, decoder, resampler and filter state, delay history and send gains reach the voice
// table in main memory as the microcode writes them back. The rest (resampling the other
// output samples of voices without filters, buses, effect lines, mixing, master level,
// writing the outputs and effect rings to main memory) runs on a DSP thread from copies
// taken on the guest thread, one block behind or more. The guest waits for it before the
// frame-end mail DCD10005, and before any other command or a restart. That barrier covers
// every main-memory write of the DSP thread: the microcode sends a block's mails before it
// mixes and writes that block, so only DCD10005 tells the CPU that outputs and rings are
// complete. GCRT_DSP_INLINE=1 runs both sides on the guest thread.
//
// Not modelled: the utility commands 0x84-0x89 (their CPU wrappers are empty) and task
// switching (Sunshine adds no other DSP task).
#include "jaudio_dsp.hpp"

#include "gcrt/dsp.hpp"
#include "gcrt/jaudio.hpp"
#include "gcrt/runtime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace gcrt {
namespace {

class Jasystem final : public Microcode {
public:
    explicit Jasystem(DspApi& api)
        : api_(api), split_(!std::getenv("GCRT_DSP_INLINE")), dsp_(api, JaudioDsp::Variant::Jasystem) {
        dsp_.set_drain([this] { drain(); });
    }
    const char* name() const override { return "JASystem"; }

    void start() override {
        drain();
        dsp_.reset();
        gate_ = true;
        state_ = State::Idle;
        api_.send_mail(0xDCD10000u, true);
        api_.send_mail(0xF3551111u, false);
    }

    // Interrupt-driven transfer: a mail's low half is the word count that follows, 0 for
    // the voice mask (two words); CDD1xxxx is a task action.
    void mail(std::uint32_t value) override {
        if (left_) {
            words_.push_back(value);
            if (--left_ == 0) timed_apply(mask_ ? Unit::Mask : Unit::Command);
            return;
        }
        if ((value >> 16) == 0xCDD1) {
            words_.assign(1, value);
            timed_apply(Unit::Task);
            return;
        }
        const std::uint16_t count = value & 0xFFFF;
        words_.clear();
        mask_ = count == 0;
        left_ = count ? count : 2;
    }

private:
    enum class State { Idle, WaitGate, WaitTaskAction };
    enum class Unit { Mask, Command, Task };

    void timed_apply(Unit unit) {
        if (verbosity < 1) return apply(unit);
        const std::uint64_t cpu = thread_cpu_ns();
        apply(unit);
        report_cost(thread_cpu_ns() - cpu);
    }

    void apply(Unit unit) {
        switch (unit) {
        case Unit::Mask:
            dsp_.set_active_mask(words_[0], words_[1]);
            gate_ = false;
            break;
        case Unit::Command:
            commands_.push_back(words_);
            break;
        case Unit::Task:
            if (state_ != State::WaitTaskAction) {
                if (reported_++ < 8) log("DSP: JASystem task mail %08x outside a frame end\n", words_[0]);
                return;
            }
            task_action(words_[0] & 0xFFFF);
            return;
        }
        run();
    }

    void send_task(std::uint16_t low) { api_.send_mail(0xDCD10000u | low, true); }
    void send_audio(std::uint16_t low) { api_.send_mail(0xF3550000u | low, false); }

    void task_action(std::uint16_t action) {
        if (action == 3) {
            state_ = State::Idle;
            run();
            return;
        }
        if (reported_++ < 8) log("DSP: JASystem task action %u not modelled\n", action);
    }

    void run() {
        for (;;) {
            if (state_ == State::Idle) {
                if (commands_.empty()) return;
                std::vector<std::uint32_t> words = std::move(commands_.front());
                commands_.pop_front();
                execute(words);
            } else if (state_ == State::WaitGate) {
                if (gate_) return;
                block();
            } else {
                return;
            }
        }
    }

    void execute(const std::vector<std::uint32_t>& w) {
        drain();
        std::uint16_t high = static_cast<std::uint16_t>(w[0] >> 16);
        std::uint16_t extra = static_cast<std::uint16_t>(w[0]);
        auto arg = [&](std::size_t i) { return i < w.size() ? w[i] : 0u; };
        if (verbosity > 1) log("DSP: JASystem command %08x (%zu words)\n", w[0], w.size());
        switch ((high >> 8) & 0x3F) {
        case 1:   // setup
            dsp_.setup(extra, arg(1), arg(2), arg(3), arg(4));
            break;
        case 2:   // render
            ++frames_;
            blocks_left_ = high & 0xFF;
            block_index_ = 0;
            dsp_.begin_frame(arg(1), arg(2), extra, split_ ? slot() : nullptr);
            begin_block();
            return;
        case 0: case 3: case 10: case 11: case 12: case 13: case 14: case 15:
            break;
        default:
            if (reported_++ < 8) log("DSP: JASystem command %08x not modelled\n", w[0]);
            break;
        }
        send_task(4);
        send_audio(high);
    }

    void begin_block() {
        if (blocks_left_ == 0) {
            dsp_.undefer();
            drain();
            send_task(5);
            state_ = State::WaitTaskAction;
            return;
        }
        if (split_) dsp_.defer(slot());
        dsp_.prepare_block();
        state_ = State::WaitGate;
    }

    void block() {
        ++block_index_;
        dsp_.render_voices();
        gate_ = true;
        send_task(4);
        send_audio(static_cast<std::uint16_t>(0xFF00 | block_index_));
        --blocks_left_;
        dsp_.finish_block();
        ++blocks_;
        if (dsp_.take_finished()) {
            ++split_blocks_;
            submit();
        }
        begin_block();
    }

    // ---- DSP thread: recorded blocks in order, from a ring the guest fills ----------------
    static constexpr std::uint32_t kJobs = 16;

    // The job the next block records into; waits for a free one.
    JaudioDsp::Synthesis* slot() {
        const std::uint32_t n = submitted_.load(std::memory_order_relaxed);
        wait_guest([&] { return n - completed_.load() < kJobs; });
        return &jobs_[n % kJobs];
    }
    void submit() {
        if (!worker_started_) {
            worker_started_ = true;
            std::thread([this] { work(); }).detach();
        }
        submitted_.fetch_add(1);
        if (worker_blocked_.load()) {
            std::lock_guard<std::mutex> lock(mutex_);
            wake_.notify_one();
        }
    }
    void drain() {
        const std::uint32_t n = submitted_.load(std::memory_order_relaxed);
        if (completed_.load() == n) return;
        const auto since = std::chrono::steady_clock::now();
        wait_guest([&] { return completed_.load() == n; });
        if (verbosity >= 1) wait_ns_ += std::chrono::nanoseconds(std::chrono::steady_clock::now() - since).count();
    }
    template <class Ready> void wait_guest(Ready ready) {
        if (ready() || spin(ready)) return;
        std::unique_lock<std::mutex> lock(mutex_);
        guest_blocked_.store(true);
        idle_.wait(lock, ready);
        guest_blocked_.store(false);
    }
    void work() {
        for (;;) {
            auto ready = [this] { return completed_.load() != submitted_.load(); };
            if (!spin(ready)) {
                std::unique_lock<std::mutex> lock(mutex_);
                worker_blocked_.store(true);
                wake_.wait(lock, ready);
                worker_blocked_.store(false);
            }
            const std::uint32_t n = completed_.load(std::memory_order_relaxed);
            const std::uint64_t cpu = verbosity >= 1 ? thread_cpu_ns() : 0;
            dsp_.synthesize(jobs_[n % kJobs]);
            if (verbosity >= 1) worker_ns_.fetch_add(thread_cpu_ns() - cpu, std::memory_order_relaxed);
            completed_.store(n + 1);
            if (guest_blocked_.load()) {
                std::lock_guard<std::mutex> lock(mutex_);
                idle_.notify_all();
            }
        }
    }
    // Polls ready() for up to 200 us of host time (virtual time stands still meanwhile).
    template <class Ready> static bool spin(Ready ready) {
        const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(200);
        for (unsigned n = 0;; ++n) {
            if (ready()) return true;
            if ((n & 63) == 63 && std::chrono::steady_clock::now() >= until) return false;
#if defined(__aarch64__)
            asm volatile("yield");
#endif
        }
    }

    static std::uint64_t thread_cpu_ns() {
        timespec t{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
        return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
    }
    // GCRT_VERBOSE=1: the model's CPU time per audio frame on each thread, and the guest's
    // waits for the DSP thread, every 10 s.
    void report_cost(std::uint64_t ns) {
        cost_ns_ += ns;
        const std::uint64_t now = now_ns();
        if (!cost_since_) cost_since_ = now;
        if (now - cost_since_ < 10000000000ull) return;
        if (frames_)
            log("DSP: JASystem %.3f ms CPU per audio frame on the guest thread, %.3f ms on the DSP thread, "
                "%.3f ms waiting; %u frames, %u of %u blocks split in 10 s\n",
                cost_ns_ / 1e6 / frames_, worker_ns_.exchange(0) / 1e6 / frames_, wait_ns_ / 1e6 / frames_, frames_,
                split_blocks_, blocks_);
        cost_ns_ = 0;
        split_blocks_ = 0;
        blocks_ = 0;
        wait_ns_ = 0;
        frames_ = 0;
        cost_since_ = now;
    }

    DspApi& api_;
    const bool split_;
    JaudioDsp dsp_;
    JaudioDsp::Synthesis jobs_[kJobs];
    // Sequentially consistent: a side sets its blocked flag, then checks the other's
    // counter under the mutex, so a wake is never lost between a spin and a block.
    std::atomic<std::uint32_t> submitted_{0}, completed_{0};
    std::atomic<bool> worker_blocked_{false}, guest_blocked_{false};
    std::mutex mutex_;
    std::condition_variable wake_, idle_;
    bool worker_started_ = false;
    std::atomic<std::uint64_t> worker_ns_{0};
    std::uint64_t cost_ns_ = 0, cost_since_ = 0, wait_ns_ = 0;
    std::uint32_t frames_ = 0, blocks_ = 0, split_blocks_ = 0;
    State state_ = State::Idle;
    bool gate_ = true;
    std::deque<std::vector<std::uint32_t>> commands_;
    std::vector<std::uint32_t> words_;   // the unit being received
    std::uint32_t left_ = 0;
    bool mask_ = false;
    std::uint32_t blocks_left_ = 0, block_index_ = 0;
    int reported_ = 0;
};

} // namespace

Microcode* create_jasystem(DspApi& api) { return new Jasystem(api); }

} // namespace gcrt

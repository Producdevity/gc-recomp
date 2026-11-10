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
// The model runs on the guest thread when a transfer completes. GCRT_DSP_THREAD=1 runs it
// on a DSP thread instead, beside the CPU as on the console: complete transfers queue to it
// and its mails reach the mailbox at the guest's next safepoint (not with virtual time).
//
// Not modelled: the utility commands 0x84-0x89 (their CPU wrappers are empty) and task
// switching (Sunshine adds no other DSP task).
#include "jaudio_dsp.hpp"

#include "gcrt/dsp.hpp"
#include "gcrt/jaudio.hpp"
#include "gcrt/runtime.hpp"

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
        : api_(api), inline_(virtual_time || !std::getenv("GCRT_DSP_THREAD")), dsp_(api, JaudioDsp::Variant::Jasystem) {}
    const char* name() const override { return "JASystem"; }

    void start() override {
        std::unique_lock<std::mutex> lock(mutex_);
        idle_.wait(lock, [this] { return inbox_.empty() && !busy_; });
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
            if (--left_ == 0) submit({mask_ ? Unit::Mask : Unit::Command, std::move(words_)});
            return;
        }
        if ((value >> 16) == 0xCDD1) {
            submit({Unit::Task, {value}});
            return;
        }
        const std::uint16_t count = value & 0xFFFF;
        words_.clear();
        mask_ = count == 0;
        left_ = count ? count : 2;
    }

private:
    enum class State { Idle, WaitGate, WaitTaskAction };
    struct Unit {
        enum Kind { Mask, Command, Task } kind;
        std::vector<std::uint32_t> words;
    };

    void submit(Unit unit) {
        if (inline_) {
            timed_apply(unit);
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_started_) {
            worker_started_ = true;
            std::thread([this] { work(); }).detach();
        }
        inbox_.push_back(std::move(unit));
        wake_.notify_one();
    }

    void work() {
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
            wake_.wait(lock, [this] { return !inbox_.empty(); });
            Unit unit = std::move(inbox_.front());
            inbox_.pop_front();
            busy_ = true;
            lock.unlock();
            timed_apply(unit);
            lock.lock();
            busy_ = false;
            if (inbox_.empty()) idle_.notify_all();
        }
    }

    void timed_apply(const Unit& unit) {
        if (verbosity < 1) return apply(unit);
        const std::uint64_t cpu = thread_cpu_ns();
        apply(unit);
        report_cost(thread_cpu_ns() - cpu);
    }

    void apply(const Unit& unit) {
        switch (unit.kind) {
        case Unit::Mask:
            dsp_.set_active_mask(unit.words[0], unit.words[1]);
            gate_ = false;
            break;
        case Unit::Command:
            commands_.push_back(unit.words);
            break;
        case Unit::Task:
            if (state_ != State::WaitTaskAction) {
                if (reported_++ < 8) log("DSP: JASystem task mail %08x outside a frame end\n", unit.words[0]);
                return;
            }
            task_action(unit.words[0] & 0xFFFF);
            return;
        }
        run();
    }

    // Mails to the CPU, in order. From the DSP thread they reach the mailbox on the guest
    // thread, one event per batch.
    void post(std::uint32_t value, bool interrupt) {
        if (inline_) {
            api_.send_mail(value, interrupt);
            return;
        }
        bool first;
        {
            std::lock_guard<std::mutex> lock(outbox_mutex_);
            first = outbox_.empty();
            outbox_.emplace_back(value, interrupt);
        }
        if (!first) return;
        schedule_at(now_ns(), [this] { deliver(); });
        wake_guest();
    }
    void deliver() {
        std::deque<std::pair<std::uint32_t, bool>> mails;
        {
            std::lock_guard<std::mutex> lock(outbox_mutex_);
            mails.swap(outbox_);
        }
        for (const auto& [value, interrupt] : mails) api_.send_mail(value, interrupt);
    }

    static std::uint64_t thread_cpu_ns() {
        timespec t{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
        return std::uint64_t(t.tv_sec) * 1000000000ull + std::uint64_t(t.tv_nsec);
    }
    // GCRT_VERBOSE=1: the model's CPU time per audio frame, every 10 s.
    void report_cost(std::uint64_t ns) {
        cost_ns_ += ns;
        const std::uint64_t now = now_ns();
        if (!cost_since_) cost_since_ = now;
        if (now - cost_since_ < 10000000000ull) return;
        if (frames_)
            log("DSP: JASystem %.2f ms CPU per audio frame, %u frames in 10 s (%s)\n", cost_ns_ / 1e6 / frames_, frames_,
                inline_ ? "guest thread" : "DSP thread");
        cost_ns_ = 0;
        frames_ = 0;
        cost_since_ = now;
    }

    void send_task(std::uint16_t low) { post(0xDCD10000u | low, true); }
    void send_audio(std::uint16_t low) { post(0xF3550000u | low, false); }

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
            dsp_.begin_frame(arg(1), arg(2), extra);
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
            send_task(5);
            state_ = State::WaitTaskAction;
            return;
        }
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
        begin_block();
    }

    DspApi& api_;
    const bool inline_;
    std::mutex mutex_, outbox_mutex_;
    std::condition_variable wake_, idle_;
    std::deque<Unit> inbox_;
    std::deque<std::pair<std::uint32_t, bool>> outbox_;
    bool busy_ = false, worker_started_ = false;
    std::uint64_t cost_ns_ = 0, cost_since_ = 0;
    std::uint32_t frames_ = 0;
    JaudioDsp dsp_;
    State state_ = State::Idle;
    bool gate_ = true;
    std::deque<std::vector<std::uint32_t>> commands_;
    std::vector<std::uint32_t> words_;   // CPU side: the unit being received
    std::uint32_t left_ = 0;
    bool mask_ = false;
    std::uint32_t blocks_left_ = 0, block_index_ = 0;
    int reported_ = 0;
};

} // namespace

Microcode* create_jasystem(DspApi& api) { return new Jasystem(api); }

} // namespace gcrt

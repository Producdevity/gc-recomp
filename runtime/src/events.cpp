// Host-time event queue. Callbacks run on the guest thread from safepoints; a timer
// thread raises gc_attn when the earliest event falls due so running guest code stops
// at its next loop back-edge.
#include "gcrt/runtime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace gcrt {
namespace {
struct Pending {
    EventId id;
    std::function<void()> fn;
};
std::mutex mu;
std::condition_variable guest_cv, timer_cv;
std::multimap<std::uint64_t, Pending> queue;
std::unordered_map<EventId, std::multimap<std::uint64_t, Pending>::iterator> by_id;
EventId next_id = 1;
bool woken = false;
bool running = false;
std::thread timer;

void timer_main() {
    std::unique_lock<std::mutex> lock(mu);
    while (running) {
        if (queue.empty()) {
            timer_cv.wait(lock);
            continue;
        }
        std::uint64_t when = queue.begin()->first, now = now_ns();
        if (now >= when) {
            raise_attention();
            guest_cv.notify_all();
            // The guest consumes due events at its next safepoint; check again shortly.
            timer_cv.wait_for(lock, std::chrono::microseconds(500));
        } else {
            timer_cv.wait_for(lock, std::chrono::nanoseconds(when - now));
        }
    }
}
} // namespace

EventId schedule_at(std::uint64_t when_ns, std::function<void()> fn) {
    std::lock_guard<std::mutex> lock(mu);
    EventId id = next_id++;
    bool earliest = queue.empty() || when_ns < queue.begin()->first;
    auto it = queue.emplace(when_ns, Pending{id, std::move(fn)});
    by_id[id] = it;
    if (earliest) timer_cv.notify_all();
    if (earliest) set_virtual_deadline(when_ns);
    return id;
}

EventId schedule_in(std::uint64_t delay_ns, std::function<void()> fn) { return schedule_at(now_ns() + delay_ns, std::move(fn)); }

void cancel(EventId id) {
    std::lock_guard<std::mutex> lock(mu);
    auto it = by_id.find(id);
    if (it == by_id.end()) return;
    queue.erase(it->second);
    by_id.erase(it);
}

bool run_due_events() {
    bool any = false;
    struct Rearm {
        ~Rearm() {
            std::lock_guard<std::mutex> lock(mu);
            set_virtual_deadline(queue.empty() ? ~0ull : queue.begin()->first);
        }
    } rearm;
    for (;;) {
        std::function<void()> fn;
        {
            std::lock_guard<std::mutex> lock(mu);
            if (queue.empty() || queue.begin()->first > now_ns()) break;
            auto it = queue.begin();
            fn = std::move(it->second.fn);
            by_id.erase(it->second.id);
            queue.erase(it);
        }
        fn();
        any = true;
    }
    return any;
}

namespace {
bool time_polled = false;
}

void note_time_poll() { time_polled = true; }

void wait_for_event(std::uint64_t max_ns) {
    if (time_polled) {
        time_polled = false;
        if (virtual_time) advance_virtual_time(now_ns() + 200);
        return;
    }
    if (virtual_time) {
        // Every guest thread waits: jump to the next event.
        std::uint64_t to = now_ns() + max_ns;
        {
            std::lock_guard<std::mutex> lock(mu);
            if (!queue.empty()) to = std::min(to, queue.begin()->first);
        }
        advance_virtual_time(to);
        return;
    }
    std::unique_lock<std::mutex> lock(mu);
    if (woken) {
        woken = false;
        return;
    }
    std::uint64_t deadline = now_ns() + max_ns;
    if (!queue.empty() && queue.begin()->first < deadline) deadline = queue.begin()->first;
    std::uint64_t now = now_ns();
    if (deadline <= now) return;
    guest_cv.wait_for(lock, std::chrono::nanoseconds(deadline - now), [] { return woken; });
    woken = false;
}

void wake_guest() {
    {
        std::lock_guard<std::mutex> lock(mu);
        woken = true;
    }
    raise_attention();
    guest_cv.notify_all();
}

void events_start() {
    std::lock_guard<std::mutex> lock(mu);
    if (running || virtual_time) return;
    running = true;
    timer = std::thread(timer_main);
}

void events_stop() {
    {
        std::lock_guard<std::mutex> lock(mu);
        running = false;
        timer_cv.notify_all();
    }
    if (timer.joinable()) timer.join();
}

} // namespace gcrt

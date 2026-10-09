#pragma once
#include <chrono>
#include <thread>

namespace aviator {
// One pacer per arm. Limit callback returns, including repeated holding commands.
class CommandPacer {
    using Clock = std::chrono::steady_clock;
    Clock::time_point next_{};
public:
    void reset() { next_ = {}; }
    template<class Now, class Sleep>
    Clock::time_point wait(Now now, Sleep sleep_until) {
        auto sent = now();
        if (sent < next_) {
            sleep_until(next_);
            sent = now();
        }
        // Base the next deadline on actual release time, never catch up missed ticks.
        next_ = sent + std::chrono::milliseconds(1);
        return sent;
    }
    Clock::time_point wait() {
        return wait(Clock::now, [](Clock::time_point deadline) { std::this_thread::sleep_until(deadline); });
    }
};
}

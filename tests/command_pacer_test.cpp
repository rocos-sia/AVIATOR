#include "CommandPacer.hpp"
#include <iostream>
#include <stdexcept>
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main() try {
    aviator::CommandPacer pacer;
    Clock::time_point time{};
    int waits = 0;
    auto now = [&] { return time; };
    auto sleep = [&](Clock::time_point deadline) { ++waits; time = deadline; };
    // Model a 2 kHz SDK issuing identical hold commands. No target update is needed
    // to build a controller queue, so every callback return must be paced.
    auto previous = pacer.wait(now, sleep);
    for (int k = 0; k < 2000; ++k) {
        time += 500us;
        const auto sent = pacer.wait(now, sleep);
        check(sent - previous == 1ms, "2 kHz callback was not limited to 1 kHz");
        previous = sent;
    }
    check(time == Clock::time_point{} + 2s, "unexpected output rate");
    // A normal/slower SDK is not delayed by another full period.
    const int before = waits;
    time += 1ms;
    pacer.wait(now, sleep);
    time += 2ms;
    pacer.wait(now, sleep);
    check(waits == before, "normal SDK pacing was slowed down");
    // Scheduling delay must not produce a burst of catch-up commands.
    time += 20ms;
    previous = pacer.wait(now, sleep);
    check(pacer.wait(now, sleep) - previous == 1ms, "late callback caused catch-up burst");
    time += 500us;
    previous = pacer.wait(now, [&](Clock::time_point deadline) { time = deadline + 50us; });
    check(pacer.wait(now, sleep) - previous == 1ms, "late wake-up shortened next interval");
    // Each arm and each startLoop epoch has an independent initial release.
    aviator::CommandPacer other;
    const auto resumed = time;
    check(other.wait(now, sleep) == resumed, "one arm delayed the other");
    pacer.reset();
    check(pacer.wait(now, sleep) == resumed, "resume inherited the previous deadline");
    check(pacer.wait(now, sleep) - resumed == 1ms, "resume lost subsequent pacing");
    // Exercise the production steady_clock/sleep_until path without robot hardware.
    aviator::CommandPacer real;
    previous = real.wait();
    for (int k = 0; k < 20; ++k) {
        const auto sent = real.wait();
        check(sent - previous >= 1ms, "real callback release was too early");
        previous = sent;
    }
    std::cout << "PASS command pacing: 2 kHz, normal rate, late wake-up, stall, reset, dual arms\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

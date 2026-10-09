#include "TargetTrace.hpp"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
using aviator::TargetTrace;
using aviator::TargetTraceFrame;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
TargetTraceFrame frame(uint64_t i) {
    TargetTraceFrame f{i, i, i / 1000};
    f.target.fill(double(i)); f.measured.fill(double(i) + .1); f.velocity.fill(double(i) + .2);
    return f;
}
int main() try {
    auto trace = std::make_unique<TargetTrace>();
    for (uint64_t i=0; i<300; ++i) trace->push(frame(i));
    trace->poll();
    std::array<double,7> expected{}; expected.fill(299);
    check(trace->begin(300, expected), "Failed to arm trace");
    check(trace->frames().size()==256 && trace->frames().front().callback==44, "Pretrigger history not retained");
    for (uint64_t i=300; i<320; ++i) trace->push(frame(i));
    trace->poll();
    check(trace->frames().back().target[6]==319, "Returned callback target corrupted");
    check(!trace->begin(320, {}), "Existing evidence overwritten");
    trace->push(frame(1000301)); trace->poll();
    check(trace->frames().size()==276, "Capture exceeded one-second window");
    trace->reset();
    for (uint64_t i=0; i<TargetTrace::queue_capacity+10; ++i) trace->push(frame(i));
    check(trace->dropped()==10, "Overflow was not reported");
    trace->begin(4096, {});
    check(trace->frames().front().callback==3840 && trace->frames().back().callback==4095,
          "Overflow overwrote unread records");
    trace->reset(); trace->begin(0, {});
    std::thread producer([&] {
        for (uint64_t i=1; i<=200000; ++i) trace->push(frame(i));
    });
    for (int i=0; i<200000; ++i) trace->poll();
    producer.join(); trace->poll();
    uint64_t previous=0;
    for (const auto& f : trace->frames()) {
        check(f.callback>previous, "Queue ordering violated");
        for (int j=0;j<7;++j) {
            check(f.target[j]==double(f.callback) && f.measured[j]==double(f.callback)+.1 &&
                  f.velocity[j]==double(f.callback)+.2, "Torn realtime frame");
        }
        check(f.loop_epoch==f.callback/1000, "Loop epoch corrupted");
        previous=f.callback;
    }
    check(!trace->frames().empty(), "Concurrent capture empty");
    std::cout << "PASS target trace: history, trigger, overflow, bounded capture, concurrent frame integrity\n";
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

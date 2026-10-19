#include "PhotoelectricEmergency.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace aviator;
using namespace std::chrono_literals;
void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
int main() try {
    zmq::context_t context(1);
    zmq::socket_t pub(context, zmq::socket_type::pub);
    configure(pub, {256, 256, 0}); pub.bind("tcp://127.0.0.1:*");
    MotionConfig config;
    config.subscribe = pub.get(zmq::sockopt::last_endpoint);
    config.photoelectric.enabled = true;
    config.photoelectric.timeout_us = 150000;
    for (const auto mode : {"zero", "invalid", "timeout", "same_sample", "startup_zero", "bad_value"}) {
        PhotoelectricEmergency input(config);
        std::this_thread::sleep_for(100ms);
        check(!input.snapshot().healthy && !input.snapshot().latched, "startup allowed without evidence");
        auto m = motionMessage(Topic::photoelectric_state, "aviator_photoelectric", "test-instance", 1);
        m.body = {{"channel", "X1"}, {"value", 1}};
        auto send = [&] { publishMessage(pub, m); std::this_thread::sleep_for(5ms); };
        m.header.publisher_id = "wrong"; send();
        check(!input.snapshot().healthy, "wrong publisher accepted");
        m.header.publisher_id = "aviator_photoelectric";
        if (std::string(mode) == "startup_zero") m.body["value"] = 0;
        send();
        if (std::string(mode) != "startup_zero") {
            check(input.snapshot().healthy, "fresh 1 not accepted");
            m.body["value"] = 0; send(); // Duplicate sequence must not trigger.
            check(!input.snapshot().latched, "duplicate sequence accepted");
            m.body["value"] = 1;
            ++m.header.sequence;
            if (std::string(mode) == "zero") {
                m.body["value"] = 0;
                publishMessage(pub, m);
                ++m.header.sequence; m.body["value"] = 1; send();
            } else if (std::string(mode) == "invalid") {
                m.header.valid = false; send();
            } else if (std::string(mode) == "bad_value") {
                m.body["value"] = true; send();
            } else {
                for (int i = 0; i < 35; ++i) {
                    if (std::string(mode) == "same_sample") { ++m.header.sequence; send(); }
                    else std::this_thread::sleep_for(5ms);
                }
            }
        }
        check(input.snapshot().latched && !input.snapshot().healthy, "emergency not latched");
        const auto reason = input.snapshot().reason;
        m.body["value"] = 1; m.header.valid = true; ++m.header.sequence;
        m.header.sample_mono_us = monotonic_us(); send();
        check(input.snapshot().latched && input.snapshot().reason == reason, "1 cleared emergency");
    }
    std::cout << "PASS photoelectric startup, identity, zero burst, invalid, stale, sticky latch\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

#include "monitor.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
aviator::Message message() {
    aviator::Message m;
    m.topic = aviator::Topic::flight_state;
    m.header = {"1.0", 1, 1, 1000000, "clock", "aviator_core",
        "11111111-1111-4111-8111-111111111111", true};
    m.body = {{"system", {{"state", "CONTROL"}, {"control_source", "JOYSTICK"},
        {"current_error_code", 0}, {"last_error_code", 0}}},
        {"freshness", {{"arm", {{"valid", true}, {"age_ms", 4}}}}}};
    return m;
}
std::string encoded(const aviator::Message& m) {
    std::string payload, error;
    check(aviator::encode(m, payload, error), "encode fixture");
    return payload;
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--publish") {
            zmq::context_t context{1}; zmq::socket_t pub(context, zmq::socket_type::pub);
            aviator::configure(pub); pub.bind(argv[2]);
            auto m = message();
            m.header.clock_id = aviator::local_clock_id();
            for (unsigned i = 1; i <= 500; ++i) {
                m.header.sequence = i; m.header.sample_mono_us = aviator::monotonic_us();
                m.header.timestamp = aviator::utc_us();
                aviator::send(pub, "flight.state", encoded(m));
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            return 0;
        }
        monitor::State state;
        check(state.snapshot(1000000, "clock")["streams"].empty(), "empty snapshot");
        auto m = message();
        state.ingest("flight.state", encoded(m), 1000000);
        auto snap = state.snapshot(1010000, "clock");
        check(snap["streams"][0]["status"] == "FRESH", "fresh state");
        check(snap["streams"][0]["summary"]["arm"]["age_ms"] == 14, "aggregate age advances");
        check(state.snapshot(1100000, "clock")["streams"][0]["status"] == "STALE", "timeout without messages");
        check(state.snapshot(1010000, "other")["streams"][0]["age_ms"].is_null(), "cross-clock age not fabricated");
        state.ingest("flight.state", encoded(m), 1020000);
        check(state.snapshot(1020000, "clock")["streams"][0]["duplicates"] == 1, "duplicate counted");
        m.header.sequence = 4;
        state.ingest("flight.state", encoded(m), 1030000);
        check(state.snapshot(1030000, "clock")["streams"][0]["gaps"] == 2, "gap accounting");
        check(state.snapshot(1200000, "clock")["streams"][0]["status"] == "STALE", "new sequence cannot hide old sample");
        m.header.sequence = 5; m.header.valid = false;
        state.ingest("flight.state", encoded(m), 1040000);
        check(state.snapshot(1040000, "clock")["streams"][0]["status"] == "INVALID", "invalid status");
        check(state.detail(1) == encoded(m) && state.detail(999).empty(), "raw payload and missing ID");
        state.ingest("flight.state.debug", "{}", 1040000);
        check(state.snapshot(1040000, "clock")["rejected"] == 1, "precise topics");
        for (unsigned i = 0; i < 65; ++i) {
            m.header.publisher_id = "source-" + std::to_string(i);
            state.ingest("flight.state", encoded(m), 1050000 + i);
        }
        snap = state.snapshot(1060000, "clock");
        check(snap["streams"].size() == 64 && snap["evicted"] == 2, "bounded stream cache");
        std::cout << "monitor tests passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

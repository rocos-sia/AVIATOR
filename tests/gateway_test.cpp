#include "gateway.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
input_event event(unsigned type, unsigned code, int value, std::uint64_t time) {
    input_event result{};
    result.type = type; result.code = code; result.value = value;
    result.input_event_sec = time / 1000000;
    result.input_event_usec = time % 1000000;
    return result;
}
flight_gateway::JoystickSample stick() {
    return {{ABS_X, -32768, 32767, 0}, {ABS_Y, 0, 65534, 32767}};
}
}
int main() {
    try {
        auto sample = stick();
        check(!sample.fresh(1000000, 100000), "no sample on startup");
        sample.update(event(EV_ABS, ABS_X, -32768, 1000000), 1000000);
        check(!sample.valid, "axis update is not a complete report");
        sample.update(event(EV_ABS, ABS_Y, 65534, 1000000), 1000000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000);
        check(sample.roll_value == -1 && sample.pitch_value == 1, "full-range normalization");
        check(sample.fresh(1099999, 100000) && !sample.fresh(1100000, 100000), "timeout boundary");
        const auto session = aviator::new_session_id();
        auto first = flight_gateway::command(sample, session, "boot", 1, 1010000, 123, 100000);
        auto repeat = flight_gateway::command(sample, session, "boot", 2, 1020000, 456, 100000);
        check(first.header.sample_mono_us == repeat.header.sample_mono_us &&
              first.header.timestamp != repeat.header.timestamp && repeat.header.sequence == 2,
              "publishing cannot refresh original sample");
        auto stale = flight_gateway::command(sample, session, "boot", 3, 1100000, 789, 100000);
        check(!stale.header.valid && stale.body == first.body, "stale keeps display value but is invalid");
        std::string payload, error;
        aviator::Message decoded;
        check(aviator::encode(first, payload, error) &&
              aviator::decode("flight.command", payload, decoded, error), "gateway common codec");
        check(decoded.body.at("source") == "JOYSTICK", "source identity");
        sample.roll.inverted = true;
        sample.update(event(EV_ABS, ABS_X, 32767, 1020000), 1020000);
        sample.update(event(EV_ABS, ABS_Y, 32767, 1020000), 1020000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1020000), 1020000);
        check(sample.roll_value == -1 && sample.pitch_value == 0, "inversion and unsigned center");
        sample.update(event(EV_ABS, ABS_X, 0, 1030000), 1030000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1030000), 1030000);
        check(sample.roll_value == 0, "signed zero preserved");
        sample.update(event(EV_SYN, SYN_DROPPED, 0, 1040000), 1040000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1050000), 1050000);
        check(sample.failed && !sample.valid, "overflow cannot auto re-enable");
        sample = stick();
        sample.update(event(EV_SYN, SYN_REPORT, 0, 2000000), 1000000);
        check(sample.failed, "future event rejected");
        sample = stick();
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 999999), 1000000);
        check(sample.failed, "regressing event rejected");
        sample = stick();
        sample.update(event(EV_ABS, ABS_X, 40000, 1000000), 1000000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000);
        check(sample.failed, "out-of-range axis is not silently clamped");
        sample = stick(); sample.failed = true;
        check(!flight_gateway::command(sample, session, "boot", 4, 1100000, 1, 100000).header.valid,
              "disconnect invalidates command");

        aviator::Message state;
        state.topic = aviator::Topic::flight_state;
        state.header = {"1.0", 1, 1, 1000000, "boot", "aviator_core", session, true};
        state.body = {{"system", {{"state", "CONTROL"}, {"current_error_code", 0}, {"last_error_code", 8194}}}};
        check(flight_gateway::valid_state_summary(state), "valid state summary");
        aviator::InputPolicy policy;
        policy.topic = aviator::Topic::flight_state; policy.publisher_id = "aviator_core";
        policy.session_id = session; policy.clock_id = "boot";
        aviator::InputGuard feedback(policy);
        check(feedback.accept(state, 1000000, error), "authorized Core feedback");
        check(feedback.expired(1100000), "Core silence becomes stale");
        state.header.sequence = 2; state.header.session_id = aviator::new_session_id();
        check(!feedback.accept(state, 1000001, error), "Core restart requires new session authorization");
        state.body["system"]["state"] = "UNKNOWN";
        check(!flight_gateway::valid_state_summary(state), "unknown state rejected");
        state.body["system"]["state"] = "CONTROL";
        state.body["system"]["current_error_code"] = -1;
        check(!flight_gateway::valid_state_summary(state), "invalid error code rejected");
        std::cout << "gateway tests passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

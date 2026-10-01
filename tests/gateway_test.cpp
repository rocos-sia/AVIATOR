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
        auto idle = stick();
        idle.initializePosition(0, 32767, 1000000);
        idle.deviceChecked(1000000);
        check(idle.fresh(1000000, 100000) && idle.roll_value == 0 && idle.pitch_value == 0,
              "stationary startup gets a valid queried position without an evdev event");
        const auto startup_command = flight_gateway::command(idle, aviator::new_session_id(), "boot", 1,
                                                              1000000, 1, 100000);
        aviator::InputPolicy startup_policy;
        startup_policy.publisher_id = "flight_gateway";
        startup_policy.session_id = startup_command.header.session_id;
        startup_policy.clock_id = "boot";
        startup_policy.source = "JOYSTICK";
        startup_policy.allow_joystick_position_hold = true;
        aviator::InputGuard startup_guard(startup_policy);
        std::string startup_error;
        check(startup_guard.accept(startup_command, 1000000, startup_error),
              "Core can bind a stationary Gateway before the first button press");
        idle.initializePosition(32767, 65534, 1020000);
        idle.deviceChecked(1020000);
        check(idle.sample_us == 1000000 && idle.roll_value == 0,
              "periodic checks do not fabricate a new position sample");
        idle.update(event(EV_ABS, ABS_X, 123, 999999), 1020000);
        idle.update(event(EV_SYN, SYN_REPORT, 0, 999999), 1020000);
        check(!idle.failed && idle.roll_value == 0 && idle.roll.value == 0,
              "queued pre-snapshot reports cannot roll back initial position");
        idle.update(event(EV_ABS, ABS_X, 32767, 1030000), 1030000);
        idle.update(event(EV_SYN, SYN_REPORT, 0, 1030000), 1030000);
        idle.deviceChecked(1030000);
        check(idle.fresh(1030000, 100000) && idle.roll_value == 1, "events replace initial snapshot");
        idle.invalidate();
        idle.initializePosition(0, 32767, 1040000);
        check(idle.failed && !idle.valid, "snapshot cannot revive a failed device");
        idle = stick();
        idle.update(event(EV_ABS, ABS_X, 1, 1000000), 1000000);
        idle.initializePosition(0, 32767, 1000000);
        check(!idle.valid, "snapshot cannot complete a partial report");
        idle = stick();
        idle.initializePosition(40000, 32767, 1000000);
        check(idle.failed, "invalid initial axis snapshot rejected");
        sample.update(event(EV_ABS, ABS_X, -32768, 1000000), 1000000);
        check(!sample.valid, "axis update is not a complete report");
        sample.update(event(EV_ABS, ABS_Y, 65534, 1000000), 1000000);
        sample.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000);
        check(!sample.fresh(1000000, 100000), "report alone does not renew device check");
        sample.deviceChecked(1000000);
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
        auto held = sample;
        for (std::uint64_t now = 1020000; now <= 6000000; now += 20000) {
            held.deviceChecked(now);
            const auto holding = flight_gateway::command(held, session, "boot", 10, now, 456, 100000);
            check(holding.header.valid && holding.header.sample_mono_us == 1000000 &&
                  holding.body.at("input_state").at("checked_mono_us") == now,
                  "static position stays valid without inventing new axis events");
        }
        check(!held.fresh(6100000, 100000), "publication cannot renew stopped device checks");
        held.update(event(EV_ABS, ABS_X, 100, 6020000), 6020000);
        held.deviceChecked(6020000);
        check(held.checked_us == 6000000, "partial report cannot renew lease");
        held.update(event(EV_SYN, SYN_REPORT, 0, 6020000), 6020000);
        held.deviceChecked(6020000);
        check(held.fresh(6020000, 100000), "completed report renews lease");
        held.invalidate();
        held.deviceChecked(6040000);
        check(!held.fresh(6040000, 100000) && !held.device_connected,
              "device fault cannot be cleared by another successful query");
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
        for (const char* name : {"INITIALIZING", "RELEASING", "READY", "HOMING"}) {
            state.body["system"]["state"] = name;
            check(flight_gateway::valid_state_summary(state), "new Core transition state rejected");
        }
        state.body["system"]["state"] = "STOPPING";
        check(!flight_gateway::valid_state_summary(state), "retired STOPPING accepted");
        state.body["system"]["state"] = "CONTROL";
        flight_gateway::CoreFeedback discovered("", "boot");
        check(discovered.session().empty() && discovered.expired(1000000), "discovery starts unbound");
        auto bad = state;
        bad.header.valid = false;
        check(!discovered.accept(bad, 1000000, error) && discovered.session().empty(), "invalid status bound Core");
        bad = state; bad.header.clock_id = "other-boot";
        check(!discovered.accept(bad, 1000000, error) && discovered.session().empty(), "foreign clock bound Core");
        bad = state; bad.header.publisher_id = "other";
        check(!discovered.accept(bad, 1000000, error) && discovered.session().empty(), "foreign publisher bound Core");
        check(!discovered.accept(state, 1100000, error) && discovered.session().empty(), "stale status bound Core");
        check(discovered.accept(state, 1000000, error) && discovered.session() == session, "automatic Core binding failed");
        check(!discovered.requestsReady(1000000), "Core discovery alone cannot authorize a button");
        check(!discovered.accept(state, 1000001, error), "replayed Core status accepted");
        bad = state; bad.header.sequence = 1; bad.header.session_id = "restarted-core";
        bad.header.sample_mono_us = 1000001;
        check(discovered.accept(bad, 1000001, error) && discovered.session() == "restarted-core", "Core restart rejected");
        check(discovered.expired(1100001), "silent Core remained fresh");
        check(!discovered.accept(bad, 1100001, error) && discovered.session() == "restarted-core", "expiry unbound Core session");
        auto authorized = state;
        authorized.header.sequence = 2;
        authorized.header.sample_mono_us = 1100000;
        authorized.body["system"]["source_authorized"] = true;
        check(discovered.accept(authorized, 1100000, error) && discovered.requestsReady(1100000),
              "fresh Core binding confirmation enables buttons");
        check(!discovered.requestsReady(1200000), "stale Core feedback disables button submission");
        authorized.header.sequence = 3;
        authorized.header.sample_mono_us = 1200000;
        authorized.body["system"]["source_authorized"] = false;
        check(discovered.accept(authorized, 1200000, error) && !discovered.requestsReady(1200000),
              "revoked source disables button submission");
        flight_gateway::CoreFeedback pinned(session, "boot");
        check(pinned.accept(bad, 1000001, error), "legacy manual Core pin was enforced");
        aviator::InputPolicy policy;
        policy.topic = aviator::Topic::flight_state; policy.publisher_id = "aviator_core";
        policy.session_id = session; policy.clock_id = "boot";
        aviator::InputGuard feedback(policy);
        check(feedback.accept(state, 1000000, error), "authorized Core feedback");
        check(feedback.expired(1100000), "Core silence becomes stale");
        state.header.sequence = 1; state.header.session_id = "another-core-instance";
        state.header.sample_mono_us = 1000001;
        check(feedback.accept(state, 1000001, error), "Core restart blocked by session authorization");
        state.body["system"]["state"] = "UNKNOWN";
        check(!flight_gateway::valid_state_summary(state), "unknown state rejected");
        state.body["system"]["state"] = "CONTROL";
        state.body["system"]["current_error_code"] = -1;
        check(!flight_gateway::valid_state_summary(state), "invalid error code rejected");
        std::cout << "gateway tests passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

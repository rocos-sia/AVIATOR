#include "gateway.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace flight_gateway {
namespace {
bool axis_valid(const Axis& axis) {
    return axis.minimum < axis.maximum && axis.value >= axis.minimum && axis.value <= axis.maximum;
}
double normalized(const Axis& axis) {
    // Preserve zero for signed axes; unsigned ranges use their midpoint.
    const double center = axis.minimum < 0 && axis.maximum > 0 ? 0.0 :
        (static_cast<double>(axis.minimum) + axis.maximum) / 2.0;
    const double value = axis.value >= center ? (axis.value - center) / (axis.maximum - center) :
        (axis.value - center) / (center - axis.minimum);
    return axis.inverted ? -value : value;
}
}
std::vector<unsigned> JoystickButtons::update(const input_event& event, std::uint64_t now,
                                              bool valid) {
    std::vector<unsigned> pressed;
    if (!valid || (event.type == EV_SYN && event.code == SYN_DROPPED)) {
        pending.fill(0);
        return pressed;
    }
    if (event.type == EV_KEY) {
        for (unsigned i = 0; i < codes.size(); ++i) {
            if (!codes[i] || codes[i] != event.code)
                continue;
            if (event.value == 0)
                held[i] = false;
            if (event.value == 1 && !held[i]) {
                held[i] = true;
                if (event.input_event_sec >= 0 && event.input_event_usec >= 0 &&
                    event.input_event_usec < 1000000 &&
                    static_cast<std::uint64_t>(event.input_event_sec) <=
                        aviator::max_json_integer / 1000000) {
                    const auto time = static_cast<std::uint64_t>(event.input_event_sec) * 1000000 +
                                      event.input_event_usec;
                    if (time <= now && now - time < 100000)
                        pending[i] = time;
                }
            }
        }
    }
    if (event.type == EV_SYN && event.code == SYN_REPORT) {
        const auto time =
            static_cast<std::uint64_t>(event.input_event_sec) * 1000000 + event.input_event_usec;
        for (unsigned i = 0; i < pending.size(); ++i)
            if (pending[i] && pending[i] <= now && now - pending[i] < 100000 &&
                time <= now && now - time < 100000)
                pressed.push_back(i);
        pending.fill(0);
    }
    return pressed;
}
void JoystickSample::update(const input_event& event, std::uint64_t now) {
    if (failed) return;
    if (event.type == EV_SYN && event.code == SYN_DROPPED) {
        invalidate(); return;
    }
    if (event.type == EV_ABS) {
        report_pending = true;
        if (event.code == roll.code) roll.value = event.value;
        if (event.code == pitch.code) pitch.value = event.value;
    }
    if (event.type != EV_SYN || event.code != SYN_REPORT) return;
    if (!axis_valid(roll) || !axis_valid(pitch) || event.input_event_sec < 0 ||
        event.input_event_usec < 0 || event.input_event_usec >= 1000000 ||
        static_cast<std::uint64_t>(event.input_event_sec) > aviator::max_json_integer / 1000000) {
        invalidate(); return;
    }
    const auto time = static_cast<std::uint64_t>(event.input_event_sec) * 1000000 + event.input_event_usec;
    if (time > now || time < sample_us || time > aviator::max_json_integer) {
        invalidate(); return;
    }
    roll_value = normalized(roll);
    pitch_value = normalized(pitch);
    sample_us = time;
    valid = true;
    report_pending = false;
}
void JoystickSample::deviceChecked(std::uint64_t now) {
    if (failed || report_pending) return;
    device_connected = true;
    checked_us = now;
}
void JoystickSample::invalidate() {
    failed = true;
    valid = false;
    device_connected = false;
}
bool JoystickSample::fresh(std::uint64_t now, std::uint64_t timeout) const {
    return valid && !failed && device_connected && sample_us > 0 && checked_us >= sample_us &&
           now >= checked_us && now - checked_us < timeout;
}
aviator::Message command(const JoystickSample& sample, const std::string& session,
                         const std::string& clock, std::uint64_t sequence,
                         std::uint64_t now, std::uint64_t utc, std::uint64_t timeout) {
    aviator::Message result;
    result.topic = aviator::Topic::flight_command;
    result.header = {"1.0", sequence, utc, sample.sample_us, clock, "flight_gateway",
                     session, sample.fresh(now, timeout)};
    result.body = {{"source", "JOYSTICK"},
        {"input_state", {{"mode", "POSITION_HOLD"}, {"device_connected", sample.device_connected},
                         {"checked_mono_us", sample.checked_us}}},
        {"control", {{"roll", sample.roll_value}, {"pitch", sample.pitch_value}}}};
    return result;
}
bool valid_state_summary(const aviator::Message& message) {
    if (message.topic != aviator::Topic::flight_state) return false;
    try {
        const auto& system = message.body.at("system");
        const auto state = system.at("state").get<std::string>();
        constexpr std::array<const char*, 10> states{
            "INIT",      "INITIALIZING", "STANDBY", "GRASPING", "RELEASING",
            "FOLLOWING", "CONTROL",      "SAFE",    "ERROR",    "EMERGENCY_STOP"};
        if (std::find(states.begin(), states.end(), state) == states.end()) return false;
        for (const char* key : {"current_error_code", "last_error_code"}) {
            const auto& value = system.at(key);
            if (!value.is_number_integer() ||
                (!value.is_number_unsigned() && value.get<std::int64_t>() < 0) ||
                value.get<std::uint64_t>() > aviator::max_json_integer) return false;
        }
        return true;
    } catch (const nlohmann::json::exception&) { return false; }
}
} // namespace flight_gateway

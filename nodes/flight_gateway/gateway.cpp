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
    // A report queued during the initial ioctl snapshot may predate that snapshot.
    if (initial_snapshot_us && event.input_event_sec >= 0 && event.input_event_usec >= 0 &&
        event.input_event_usec < 1000000 &&
        static_cast<std::uint64_t>(event.input_event_sec) <= aviator::max_json_integer / 1000000 &&
        static_cast<std::uint64_t>(event.input_event_sec) * 1000000 + event.input_event_usec < initial_snapshot_us)
        return;
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
void JoystickSample::initializePosition(int roll_raw, int pitch_raw, std::uint64_t now) {
    if (valid || failed || report_pending) return;
    roll.value = roll_raw;
    pitch.value = pitch_raw;
    if (!axis_valid(roll) || !axis_valid(pitch) || !now || now > aviator::max_json_integer) {
        invalidate(); return;
    }
    roll_value = normalized(roll);
    pitch_value = normalized(pitch);
    initial_snapshot_us = sample_us = now;
    valid = true;
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
void KeyboardInput::suppressHeld(unsigned code) {
    for (unsigned i = 0; i < keyboard_arrows.size(); ++i)
        if (keyboard_arrows[i] == code) suppressed_[i] = true;
}
void KeyboardInput::setAxis(unsigned axis, double value) {
    if (axis == 0)
        roll_ = std::clamp(sample_.roll.inverted ? -value : value, -config_.roll_limit, config_.roll_limit);
    else
        pitch_ = std::clamp(sample_.pitch.inverted ? -value : value, -config_.pitch_limit, config_.pitch_limit);
}
void KeyboardInput::advance(std::uint64_t now) {
    if (integrated_us_ && now > integrated_us_) {
        const double dt = (now - integrated_us_) / 1000000.0;
        const int roll_direction = int(held_[1]) - int(held_[0]);
        const int pitch_direction = int(held_[3]) - int(held_[2]);
        roll_ = roll_direction ? std::clamp(roll_ + roll_direction * config_.roll_speed * dt,
                                           -config_.roll_limit, config_.roll_limit) : roll_;
        pitch_ = pitch_direction ? std::clamp(pitch_ + pitch_direction * config_.pitch_speed * dt,
                                             -config_.pitch_limit, config_.pitch_limit) : pitch_;
    }
    integrated_us_ = std::max(integrated_us_, now);
}
void KeyboardInput::update(const input_event& event, std::uint64_t now) {
    if (sample_.failed) return;
    if (event.type == EV_SYN && event.code == SYN_DROPPED) {
        sample_.invalidate(); return;
    }
    if (event.type != EV_KEY && !(event.type == EV_SYN && event.code == SYN_REPORT)) return;
    if (event.input_event_sec < 0 || event.input_event_usec < 0 || event.input_event_usec >= 1000000 ||
        static_cast<std::uint64_t>(event.input_event_sec) > aviator::max_json_integer / 1000000) {
        sample_.invalidate(); return;
    }
    const auto time = static_cast<std::uint64_t>(event.input_event_sec) * 1000000 + event.input_event_usec;
    if (!time || time > now || time < event_us_ || now - time >= 100000) {
        sample_.invalidate(); return;
    }
    event_us_ = time;
    if (event.type == EV_KEY) {
        if (event.value < 0 || event.value > 2) { sample_.invalidate(); return; }
        sample_.report_pending = true;
        if (event.code == KEY_0 && event.value == 1) center_pending_ = true;
        for (unsigned i = 0; i < keyboard_arrows.size(); ++i) {
            if (event.code != keyboard_arrows[i]) continue;
            if (event.value == 0) { suppressed_[i] = false; pending_[i] = false; }
            if (event.value == 1 && !suppressed_[i]) pending_[i] = true;
        }
        return;
    }
    advance(time);
    if (center_pending_) {
        roll_ = pitch_ = 0;
        for (unsigned i = 0; i < held_.size(); ++i)
            suppressed_[i] = suppressed_[i] || held_[i] || pending_[i];
        pending_.fill(false);
        center_pending_ = false;
    }
    held_ = pending_;
    sample_.report_pending = false;
}
void KeyboardInput::deviceChecked(std::uint64_t now) {
    if (sample_.failed || sample_.report_pending) return;
    advance(now);
    sample_.roll_value = sample_.roll.inverted ? -roll_ : roll_;
    sample_.pitch_value = sample_.pitch.inverted ? -pitch_ : pitch_;
    // This is a newly integrated software target, based on checked keyboard state.
    sample_.sample_us = now;
    sample_.valid = true;
    sample_.deviceChecked(now);
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
CoreFeedback::CoreFeedback(const std::string&, const std::string& clock) {
    policy_.topic = aviator::Topic::flight_state;
    policy_.publisher_id = "aviator_core";
    policy_.clock_id = clock;
}
bool CoreFeedback::accept(const aviator::Message& message, std::uint64_t now, std::string& error) {
    if (!valid_state_summary(message)) { error = "invalid Core system summary"; return false; }
    const auto& system = message.body.at("system");
    if (system.contains("source_authorized") && !system.at("source_authorized").is_boolean()) {
        error = "invalid Core source_authorized"; return false;
    }
    if (guard_) {
        if (!guard_->accept(message, now, error)) return false;
        policy_.session_id = message.header.session_id;
        source_authorized_ = system.value("source_authorized", false);
        return true;
    }
    auto candidate_policy = policy_;
    candidate_policy.session_id = message.header.session_id;
    aviator::InputGuard candidate(candidate_policy);
    if (!candidate.accept(message, now, error)) return false;
    policy_ = std::move(candidate_policy);
    guard_ = std::move(candidate);
    source_authorized_ = system.value("source_authorized", false);
    return true;
}
bool valid_state_summary(const aviator::Message& message) {
    if (message.topic != aviator::Topic::flight_state) return false;
    try {
        const auto& system = message.body.at("system");
        const auto state = system.at("state").get<std::string>();
        constexpr std::array<const char*, 12> states{
            "INIT", "STANDBY", "GRASPING", "FOLLOWING", "CONTROL", "SAFE", "ERROR", "EMERGENCY_STOP",
            "INITIALIZING", "RELEASING", "READY", "HOMING"};
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

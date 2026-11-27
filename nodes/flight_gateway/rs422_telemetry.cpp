#include "rs422_runtime.hpp"
#include <cmath>
namespace flight_gateway {
namespace {
void wheelPosition(double angle, double displacement, int16_t& roll, int16_t& pitch) {
    const double r = aviator::wheel::roll(angle), p = aviator::wheel::pitch(displacement);
    // Only tolerate floating-point roundoff, never clip actual out-of-range measurements.
    if (std::isfinite(r) && std::abs(r) <= 50.0 + 1e-9)
        roll = rs422::encodeRoll(std::clamp(r, -50.0, 50.0));
    if (std::isfinite(p) && p >= -80.0 - 1e-9 && p <= 70.0 + 1e-9)
        pitch = rs422::encodePitch(std::clamp(p, -80.0, 70.0));
}
aviator::InputPolicy cameraPolicy(std::string clock, const std::string& publisher) {
    aviator::InputPolicy p;
    p.topic = aviator::Topic::camera_detection;
    p.publisher_id = publisher;
    p.clock_id = std::move(clock);
    p.timeout_us = 100000;
    return p;
}
aviator::InputPolicy photoelectricPolicy(std::string clock, const std::string& publisher) {
    aviator::InputPolicy p;
    p.topic = aviator::Topic::photoelectric_state;
    p.publisher_id = publisher;
    p.clock_id = std::move(clock);
    p.timeout_us = 100000;
    return p;
}
} // namespace
PhotoelectricTelemetry::PhotoelectricTelemetry(std::string clock, std::string publisher)
    : publisher_(std::move(publisher)), guard_(photoelectricPolicy(std::move(clock), publisher_)) {}
bool PhotoelectricTelemetry::accept(const aviator::Message& message, uint64_t now) {
    if (message.topic != aviator::Topic::photoelectric_state || message.header.publisher_id != publisher_ ||
        !message.body.is_object() || !message.body.contains("channel") || message.body["channel"] != "X1")
        return false;
    // A bad observation from this source must not preserve an older healthy handbrake value.
    brake_ = rs422::unknown;
    std::string error;
    if (!guard_.accept(message, now, error) || !message.header.sample_mono_us ||
        message.header.sample_mono_us < sample_)
        return false;
    const auto value = message.body.find("value");
    if (value == message.body.end() || !value->is_number_integer() || (*value != 0 && *value != 1))
        return false;
    const int current = value->get<int>();
    if (message.header.sample_mono_us == sample_ && current != value_)
        return false;
    sample_ = message.header.sample_mono_us;
    value_ = current;
    // Wire byte 12 is the logical trigger, not the raw electrical level.
    brake_ = current == 0 ? 0x01 : 0x00;
    return true;
}
void PhotoelectricTelemetry::populate(rs422::Uplink& up, uint64_t now) const {
    up.brake = guard_.expired(now) ? rs422::unknown : brake_;
}
CameraTelemetry::CameraTelemetry(std::string clock, std::string camera_id, std::string publisher)
    : camera_id_(std::move(camera_id)), publisher_(std::move(publisher)),
      guard_(cameraPolicy(std::move(clock), publisher_)) {}
bool CameraTelemetry::accept(const aviator::Message& message, uint64_t now) {
    if (message.topic != aviator::Topic::camera_detection || message.header.publisher_id != publisher_ ||
        !message.body.is_object() || !message.body.contains("camera_id") ||
        message.body["camera_id"] != camera_id_) return false;
    // A rejected observation from our source must not keep an older healthy value alive.
    roll_ = pitch_ = rs422::invalid_position;
    try {
        std::string error;
        auto envelope = message;
        envelope.header.valid = true;
        if (!guard_.accept(envelope, now, error) || message.header.sample_mono_us <= sample_) return false;
        sample_ = message.header.sample_mono_us;
        const auto& wheel = message.body.at("steering_wheel");
        if (!message.header.valid || wheel.at("valid") != true) return false;
        const auto& theta = wheel.at("theta_rad");
        const auto& travel = wheel.at("translation_along_axis_m");
        if (!theta.is_number() || !travel.is_number()) return false;
        wheelPosition(-theta.get<double>(), -travel.get<double>() - .080, roll_, pitch_);
        return roll_ != rs422::invalid_position || pitch_ != rs422::invalid_position;
    } catch (const nlohmann::json::exception&) { return false; }
}
void CameraTelemetry::populate(rs422::Uplink& up, uint64_t now) const {
    up.roll = up.pitch = rs422::invalid_position;
    if (!guard_.expired(now)) {
        up.roll = roll_;
        up.pitch = pitch_;
    }
}
void populateTelemetry(rs422::Uplink& up, const aviator::Message& state, uint64_t now) {
    up.joint_roll = up.joint_pitch = rs422::invalid_position;
    if (!state.header.valid || now < state.header.sample_mono_us ||
        now - state.header.sample_mono_us >= 100000)
        return;
    try {
        const auto& sys = state.body.at("system");
        up.state = rs422::stateCode(sys.at("state").get<std::string>());
        const auto fault = [&](const char* key) {
            const auto& v = sys.at(key);
            if (!v.is_number_integer() || v.get<int64_t>() < 0 || v.get<uint64_t>() > 65535)
                return rs422::unknown_fault;
            const auto code = v.get<uint16_t>();
            return rs422::knownFault(code) ? code : rs422::unknown_fault;
        };
        up.fault = fault("current_error_code");
        up.last_fault = fault("last_error_code");
        up.severity = up.fault == rs422::unknown_fault ? rs422::unknown
                      : up.fault                       ? uint8_t(((up.fault / 100) % 10 + 1) * 16)
                                                       : 0;
        const auto fresh = [&](const nlohmann::json& item) {
            const auto t = item.at("sample_mono_us").get<uint64_t>();
            return item.at("valid") == true && t && t <= now && now - t < 100000;
        };
        // FK feedback has its own validity/time. Simulation wheel truth and commanded
        // wheel_reference are not joint measurements and cannot populate these bytes.
        if (state.body.contains("wheel_joint_measurement")) {
            try {
                const auto& m = state.body.at("wheel_joint_measurement");
                if (state.body.value("software_lock", false) && m.at("source") == "joint_fk" && fresh(m))
                    wheelPosition(m.at("angle").get<double>(), m.at("displacement").get<double>(),
                                  up.joint_roll, up.joint_pitch);
            } catch (const nlohmann::json::exception&) { /* Keep unrelated feedback available. */ }
        }
        if (state.body.contains("discrete_inputs")) {
            const auto& inputs = state.body.at("discrete_inputs");
            if (fresh(inputs)) {
                if (inputs.contains("rear_arrived") && inputs["rear_arrived"].is_boolean())
                    up.rear = inputs["rear_arrived"].get<bool>();
            }
        }
    } catch (const nlohmann::json::exception&) {
        // Malformed optional data cannot fabricate healthy measurements.
    }
}
} // namespace flight_gateway

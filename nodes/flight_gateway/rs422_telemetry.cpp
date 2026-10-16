#include "rs422_runtime.hpp"
#include <cmath>
namespace flight_gateway {
void populateTelemetry(rs422::Uplink& up, const aviator::Message& state, uint64_t now) {
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
        if (state.body.contains("wheel_measurement")) {
            const auto& m = state.body.at("wheel_measurement");
            if (fresh(m)) {
                // Internal coordinates: angle=-ICD roll; displacement=(pitch_mm-170)/1000.
                const double roll =
                    -m.at("angle").get<double>() / (50 * 3.14159265358979323846 / 180);
                const double pitch = (m.at("displacement").get<double>() + .085) / .085;
                if (std::isfinite(roll) && std::abs(roll) <= 1.04)
                    up.roll = rs422::quantize(roll, 1.04);
                if (std::isfinite(pitch) && std::abs(pitch) <= 90.0 / 85)
                    up.pitch = rs422::quantize(pitch, 90.0 / 85);
            }
        }
        if (state.body.contains("discrete_inputs")) {
            const auto& inputs = state.body.at("discrete_inputs");
            if (fresh(inputs)) {
                if (inputs.contains("rear_arrived") && inputs["rear_arrived"].is_boolean())
                    up.rear = inputs["rear_arrived"].get<bool>();
                if (inputs.contains("emergency_brake") && inputs["emergency_brake"].is_boolean())
                    up.brake = inputs["emergency_brake"].get<bool>();
            }
        }
    } catch (const nlohmann::json::exception&) {
        // Malformed optional data cannot fabricate healthy measurements.
    }
}
} // namespace flight_gateway

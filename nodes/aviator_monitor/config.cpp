#include "config.hpp"
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
namespace monitor {
Json default_config() {
    Json arms = Json::object();
    for (const auto* side : {"left", "right"}) {
        Json joints = Json::array();
        for (unsigned i = 1; i <= 7; ++i)
            joints.push_back({{"name", std::string("AR5-5_07") + (side[0] == 'l' ? "L" : "R") +
                                           "-W4C4A2_joint_" + std::to_string(i)},
                              {"sign", 1},
                              {"offset_rad", 0}});
        arms[side] = joints;
    }
    return {{"version", 1},
            {"sources",
             {{"flight.command", "flight_gateway"},
              {"flight.state", "aviator_core"},
              {"arm.state", "manipulator"},
              {"hand.state", "inspire_hand"},
              {"camera.detection", "camera"}}},
            {"timeouts_ms",
             {{"flight.command", 100},
              {"flight.state", 100},
              {"arm.command", 50},
              {"arm.state", 50},
              {"hand.command", 100},
              {"hand.state", 300},
              {"camera.command", 200},
              {"camera.detection", 200}}},
            {"preview",
             {{"endpoint", "tcp://127.0.0.1:5561"},
              {"camera_id", "cockpit"},
              {"publisher_id", "camera"},
              {"timeout_ms", 500}}},
            {"arm_joints", arms},
            {"hand_calibration", nullptr},
            {"yoke_calibration", nullptr}};
}
namespace {
void require(bool ok, const std::string& message) {
    if (!ok)
        throw std::runtime_error("monitor config: " + message);
}
bool finite(const Json& j) { return j.is_number() && std::isfinite(j.get<double>()); }
void vector(const Json& j, unsigned size) {
    require(j.is_array() && j.size() == size, "invalid vector length");
    for (const auto& v : j)
        require(finite(v), "non-finite vector");
}
void pose(const Json& j) {
    vector(j.at("position_m"), 3);
    vector(j.at("quaternion_xyzw"), 4);
    double norm = 0;
    for (const auto& q : j.at("quaternion_xyzw"))
        norm += q.get<double>() * q.get<double>();
    require(std::abs(norm - 1) < 1e-5, "quaternion must be unit xyzw");
}
} // namespace
void validate_config(const Json& c) {
    require(c.is_object() && c.at("version") == 1, "version must be 1");
    const auto defaults = default_config();
    require(c.at("sources").is_object(), "sources must be object");
    for (const auto& [key, value] : defaults.at("sources").items())
        require(c.at("sources").contains(key), "source cannot be null/deleted: " + key);
    for (const auto& [key, value] : defaults.at("timeouts_ms").items())
        require(c.at("timeouts_ms").contains(key), "timeout cannot be null/deleted: " + key);
    for (const auto& [topic, value] : c.at("sources").items()) {
        require(defaults.at("sources").contains(topic), "unknown overview topic " + topic);
        if (value.is_string()) {
            require(value.get<std::string>().size() <= 128, "invalid publisher");
        } else {
            require(value.is_object() && value.size() <= 2 &&
                        value.at("publisher_id").is_string() &&
                        !value.at("publisher_id").get<std::string>().empty() &&
                        value.at("publisher_id").get<std::string>().size() <= 128,
                    "source must be publisher string or publisher/session selector");
            // Legacy session selectors are accepted but no longer filter sources.
        }
    }
    for (const auto& [topic, value] : c.at("timeouts_ms").items()) {
        require(defaults.at("timeouts_ms").contains(topic), "unknown timeout topic " + topic);
        require(finite(value) && value.get<double>() >= 1 && value.get<double>() <= 10000,
                "invalid timeout");
    }
    const auto& p = c.at("preview");
    require(p.at("endpoint").is_string(), "preview endpoint must be string");
    const auto endpoint = p.at("endpoint").get<std::string>();
    require(endpoint.empty() || endpoint.rfind("tcp://", 0) == 0, "preview endpoint must use TCP");
    require(p.at("camera_id").is_string() && !p.at("camera_id").get<std::string>().empty() &&
                p.at("camera_id").get<std::string>().size() <= 64,
            "invalid camera id");
    require(p.at("publisher_id").is_string() && !p.at("publisher_id").get<std::string>().empty(),
            "invalid preview publisher");
    require(finite(p.at("timeout_ms")) && p.at("timeout_ms").get<double>() >= 1 &&
                p.at("timeout_ms").get<double>() <= 10000,
            "invalid preview timeout");
    std::set<std::string> names;
    for (const auto* side : {"left", "right"}) {
        const auto& joints = c.at("arm_joints").at(side);
        require(joints.is_array() && joints.size() == 7, "arm mapping must contain seven joints");
        for (const auto& joint : joints) {
            require(joint.at("name").is_string() &&
                        names.insert(joint.at("name").get<std::string>()).second,
                    "duplicate arm joint");
            require(finite(joint.at("sign")) && std::abs(joint.at("sign").get<double>()) == 1 &&
                        finite(joint.at("offset_rad")),
                    "invalid arm sign/offset");
        }
    }
    const auto& hand = c.at("hand_calibration");
    if (!hand.is_null()) {
        require(hand.at("id").is_string() && !hand.at("id").get<std::string>().empty(),
                "hand calibration id required");
        for (const auto* side : {"left", "right"}) {
            const auto& curves = hand.at(side);
            require(curves.is_array() && curves.size() == 6, "six hand curves required per side");
            for (const auto& curve : curves) {
                require(curve.at("joint").is_string() &&
                            names.insert(curve.at("joint").get<std::string>()).second,
                        "duplicate calibrated joint");
                const auto& knots = curve.at("knots");
                require(knots.is_array() && knots.size() >= 2 && knots.size() <= 32,
                        "invalid hand knots");
                double previous = -1;
                for (const auto& knot : knots) {
                    vector(knot, 2);
                    double x = knot[0].get<double>();
                    require(x >= 0 && x <= 1 && x > previous, "knots must increase within [0,1]");
                    previous = x;
                }
                require(knots.front()[0] == 0 && knots.back()[0] == 1,
                        "hand curves must cover [0,1]");
            }
        }
    }
    const auto& y = c.at("yoke_calibration");
    if (!y.is_null()) {
        require(y.at("id").is_string() && !y.at("id").get<std::string>().empty(),
                "yoke calibration id required");
        for (const auto* key : {"aircraft_camera", "tag_yoke", "aircraft_yoke_zero"})
            pose(y.at(key));
        for (const auto* key : {"roll_axis", "pitch_axis"}) {
            vector(y.at(key), 3);
            double norm = 0;
            for (const auto& v : y.at(key))
                norm += v.get<double>() * v.get<double>();
            require(std::abs(norm - 1) < 1e-5, "motion axes must be unit vectors");
        }
        for (const auto* key : {"pitch_zero_mm", "min_confidence", "max_rotation_residual_deg",
                                "max_translation_residual_mm"})
            require(finite(y.at(key)), "invalid yoke threshold");
        require(y.at("pitch_zero_mm") >= -5 && y.at("pitch_zero_mm") <= 175 &&
                    y.at("min_confidence") >= 0 && y.at("min_confidence") <= 1 &&
                    y.at("max_rotation_residual_deg") > 0 &&
                    y.at("max_translation_residual_mm") > 0,
                "yoke thresholds out of range");
        const auto& display = y.at("model");
        for (const auto* key : {"roll_sign", "roll_offset_rad", "pitch_sign", "pitch_offset_m"})
            require(finite(display.at(key)), "invalid display mapping");
        require(std::abs(display.at("roll_sign").get<double>()) == 1 &&
                    std::abs(display.at("pitch_sign").get<double>()) == 1,
                "display sign must be +/-1");
    }
}
Json load_config(const std::string& path) {
    Json c = default_config();
    if (!path.empty()) {
        std::ifstream file(path);
        if (!file)
            throw std::runtime_error("cannot read monitor config: " + path);
        Json overrides;
        file >> overrides;
        require(overrides.is_object(), "root must be object");
        for (const auto& [key, value] : overrides.items())
            require(c.contains(key), "unknown key " + key);
        c.merge_patch(overrides);
        // JSON merge-patch deletes null fields; calibration null explicitly disables it.
        for (const auto* key : {"hand_calibration", "yoke_calibration"})
            if (!c.contains(key))
                c[key] = nullptr;
    }
    validate_config(c);
    return c;
}
} // namespace monitor

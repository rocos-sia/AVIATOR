#include "monitor.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace monitor {
namespace {
constexpr double pi = 3.14159265358979323846;
bool finite(const Json& j) { return j.is_number() && std::isfinite(j.get<double>()); }
bool vector(const Json& j, unsigned count) {
    return j.is_array() && j.size() == count && std::all_of(j.begin(), j.end(), finite);
}
bool stamp(const Json& j) {
    return j.is_number_integer() && j.get<double>() >= 0 &&
           j.get<double>() <= aviator::max_json_integer;
}
const Json& at(const Json& j, const std::string& key) {
    static const Json empty;
    return j.is_object() && j.contains(key) ? j.at(key) : empty;
}
bool yes(const Json& j) { return j.is_boolean() && j.get<bool>(); }
Json source(const Stream* s) {
    if (!s)
        return nullptr;
    const auto& h = s->message.header;
    return {{"topic", aviator::topic_name(s->message.topic)},
            {"publisher_id", h.publisher_id},
            {"session_id", h.session_id},
            {"sequence", h.sequence},
            {"message_id", s->id}};
}
const Stream* select(const State& state, aviator::Topic topic, std::uint64_t now, bool& conflict) {
    const auto name = std::string(aviator::topic_name(topic));
    const auto rule = state.config.at("sources").value(name, Json(""));
    const auto publisher =
        rule.is_string() ? rule.get<std::string>() : rule.at("publisher_id").get<std::string>();
    const Stream* last = nullptr;
    unsigned live = 0;
    for (const auto& s : state.streams) {
        if (s.message.topic != topic ||
            (!publisher.empty() && s.message.header.publisher_id != publisher))
            continue;
        if (now >= s.received_us && now - s.received_us < state.timeout_us(topic)) {
            ++live;
            last = &s;
        }
    }
    conflict = live > 1;
    if (live == 0) {
        for (const auto& s : state.streams)
            if (s.message.topic == topic &&
                (publisher.empty() || s.message.header.publisher_id == publisher) &&
                (!last || s.received_us > last->received_us))
                last = &s;
    }
    return conflict ? nullptr : last;
}
Json group(const Stream* s, bool conflict, std::uint64_t now, const std::string& clock,
           std::uint64_t limit, bool respect_valid = true) {
    Json g{{"source", source(s)},
           {"receive_state", "UNAVAILABLE"},
           {"measurement_state", conflict ? "SOURCE_CONFLICT" : "UNAVAILABLE"},
           {"reason", conflict ? "multiple_recent_sources" : "no_sample"},
           {"sample_age_ms", nullptr},
           {"receive_age_ms", nullptr},
           {"fresh_for_ms", nullptr},
           {"calibration_id", nullptr},
           {"current", nullptr},
           {"last_valid", nullptr}};
    if (!s)
        return g;
    const auto& h = s->message.header;
    std::uint64_t effective_sample = h.sample_mono_us;
    if (s->message.topic == aviator::Topic::flight_command &&
        s->message.body.contains("input_state")) {
        std::string reason;
        bool connected = false;
        std::uint64_t checked = 0;
        if (aviator::read_position_hold(s->message, checked, connected, reason) && connected)
            effective_sample = checked;
    }
    const auto received = now >= s->received_us ? now - s->received_us : limit;
    // Camera display follows received detections. Producer clocks and capture
    // latency must not freeze a valid observation while messages keep arriving.
    const bool receive_based = s->message.topic == aviator::Topic::camera_detection;
    g["receive_age_ms"] = received / 1000.0;
    g["receive_state"] = received < limit ? "FRESH" : "STALE";
    if (!receive_based && h.clock_id != clock) {
        g["measurement_state"] = "CLOCK_UNKNOWN";
        g["reason"] = "clock_domain_mismatch";
    } else if (!receive_based && effective_sample > now) {
        g["measurement_state"] = "FUTURE";
        g["reason"] = "future_sample";
    } else {
        const auto age = receive_based ? received : now - effective_sample;
        if (h.clock_id == clock && h.sample_mono_us <= now)
            g["sample_age_ms"] = (now - h.sample_mono_us) / 1000.0;
        g["effective_age_ms"] = age / 1000.0;
        if (received >= limit || (respect_valid && age >= limit)) {
            g["measurement_state"] = "STALE";
            g["reason"] = "sample_or_receive_timeout";
        } else if (respect_valid && !h.valid) {
            g["measurement_state"] = "INVALID";
            g["reason"] = "producer_invalid";
        } else {
            g["measurement_state"] = "VALID";
            g["reason"] = "valid_sample";
            g["fresh_for_ms"] =
                (limit - (respect_valid ? std::max(received, age) : received)) / 1000.0;
        }
    }
    return g;
}
bool valid(const Json& g) { return g.at("measurement_state") == "VALID"; }
void invalidate(Json& g, const char* state, const char* reason) {
    g["measurement_state"] = state;
    g["reason"] = reason;
    g["current"] = nullptr;
    g["fresh_for_ms"] = nullptr;
}
void side_age(Json& g, const Json& body, std::uint64_t now, std::uint64_t limit) {
    const auto& time = at(body, "sample_mono_us");
    if (!stamp(time)) {
        invalidate(g, "UNAVAILABLE", "side_sample_time_missing");
        return;
    }
    const auto sampled = time.get<std::uint64_t>();
    if (sampled > now) {
        invalidate(g, "FUTURE", "future_side_sample");
        return;
    }
    double age = (now - sampled) / 1000.0;
    const auto& reported = at(body, "feedback_age_ms");
    if (finite(reported)) {
        if (reported.get<double>() < 0) {
            invalidate(g, "INVALID", "negative_feedback_age");
            return;
        }
        // Reported age was measured at the outer sample time; always advance it.
        age = std::max(age, reported.get<double>() + g["sample_age_ms"].get<double>());
    }
    g["sample_age_ms"] = age;
    if (age * 1000 >= limit)
        invalidate(g, "STALE", "side_feedback_timeout");
    else
        g["fresh_for_ms"] = std::min(g["fresh_for_ms"].get<double>(), limit / 1000.0 - age);
}
using V = std::array<double, 3>;
using Q = std::array<double, 4>;
V add(V a, V b) {
    for (int i = 0; i < 3; ++i)
        a[i] += b[i];
    return a;
}
V neg(V a) {
    for (auto& v : a)
        v = -v;
    return a;
}
double dot(V a, V b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Q mul(Q a, Q b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}
Q inverse(Q q) { return {-q[0], -q[1], -q[2], q[3]}; }
V rotate(Q q, V v) {
    Q r = mul(mul(q, {v[0], v[1], v[2], 0}), inverse(q));
    return {r[0], r[1], r[2]};
}
struct Pose {
    V p;
    Q q;
};
Pose compose(Pose a, Pose b) { return {add(a.p, rotate(a.q, b.p)), mul(a.q, b.q)}; }
Pose configured_pose(const Json& j) {
    return {j.at("position_m").get<V>(), j.at("quaternion_xyzw").get<Q>()};
}
bool camera_pose(const Json& j, Pose& result) {
    const auto& p = at(j, "position");
    const auto& q = at(j, "orientation");
    for (auto* key : {"x", "y", "z"})
        if (!finite(at(p, key)))
            return false;
    for (auto* key : {"qx", "qy", "qz", "qw"})
        if (!finite(at(q, key)))
            return false;
    result = {{p["x"], p["y"], p["z"]}, {q["qx"], q["qy"], q["qz"], q["qw"]}};
    double norm = 0;
    for (double v : result.q)
        norm += v * v;
    if (std::abs(norm - 1) > 1e-3)
        return false;
    for (auto& v : result.q)
        v /= std::sqrt(norm);
    return true;
}
void yoke(Json& g, const Json& b, const Json& c, const std::string& camera_id) {
    // New camera publishers already resolve motion against their calibration.
    // Never fall back to raw pose when a supplied observation is invalid.
    if (b.contains("steering_wheel")) {
        const auto& wheel = at(b, "steering_wheel");
        g["calibration_id"] = at(wheel, "calibration_id");
        const auto& theta = at(wheel, "theta_rad");
        const auto& translation = at(wheel, "translation_along_axis_m");
        if (!yes(at(wheel, "valid")) || !finite(theta) || !finite(translation)) {
            invalidate(g, "INVALID", "invalid_steering_wheel");
            return;
        }
        const double roll = theta.get<double>() * 180 / pi;
        const double travel = translation.get<double>();
        g["current"] = {
            {"roll_deg", roll},
            {"pitch_mm", (travel + .085) * 1000},
            {"roll_percent", roll * 2},
            {"pitch_percent", travel / .085 * 100},
            {"confidence", at(b, "confidence")},
            {"camera_id", at(b, "camera_id")},
            {"frame_id", at(b, "frame_id")},
            {"pose_mapping", "CAMERA_STEERING_WHEEL"},
            {"model_joints", {{"roll_input_joint", -theta.get<double>()},
                              {"pitch_input_joint", -travel - .085}}}};
        return;
    }
    // Legacy raw poses still need the configured camera's geometry. Calibrated
    // steering_wheel observations above are independent of RGB preview settings.
    if (at(b, "camera_id") != camera_id) {
        invalidate(g, "INVALID", "camera_identity_mismatch");
        return;
    }
    if (at(b, "status") != "TRACKING") {
        invalidate(g, "INVALID", "target_not_tracking");
        return;
    }
    if (c.is_null()) {
        invalidate(g, "UNCALIBRATED", "yoke_calibration_missing");
        return;
    }
    g["calibration_id"] = c.at("id");
    const auto& confidence = at(b, "confidence");
    if (!finite(confidence) || confidence.get<double>() > 1 ||
        confidence.get<double>() < c.at("min_confidence").get<double>()) {
        invalidate(g, "INVALID", "detection_confidence_low");
        return;
    }
    Pose observed;
    if (!camera_pose(at(b, "pose"), observed)) {
        invalidate(g, "INVALID", "invalid_target_pose");
        return;
    }
    Pose zero = configured_pose(c.at("aircraft_yoke_zero"));
    Pose absolute = compose(compose(configured_pose(c.at("aircraft_camera")), observed),
                            configured_pose(c.at("tag_yoke")));
    Pose relative = compose({rotate(inverse(zero.q), neg(zero.p)), inverse(zero.q)}, absolute);
    V rollaxis = c.at("roll_axis").get<V>(), pitchaxis = c.at("pitch_axis").get<V>();
    double projection = dot({relative.q[0], relative.q[1], relative.q[2]}, rollaxis);
    double norm = std::hypot(projection, relative.q[3]);
    if (norm < 1e-8) {
        invalidate(g, "INVALID", "ambiguous_rotation");
        return;
    }
    Q twist = {rollaxis[0] * projection / norm, rollaxis[1] * projection / norm,
               rollaxis[2] * projection / norm, relative.q[3] / norm};
    Q residual = mul(relative.q, inverse(twist));
    double rotation_error = 2 * std::acos(std::clamp(std::abs(residual[3]), 0.0, 1.0)) * 180 / pi;
    pitchaxis =
        rotate(twist, pitchaxis); // Prismatic motion follows the roll joint in the URDF chain.
    double displacement = dot(relative.p, pitchaxis);
    V remainder = relative.p;
    for (int i = 0; i < 3; ++i)
        remainder[i] -= pitchaxis[i] * displacement;
    double translation_error = std::sqrt(dot(remainder, remainder)) * 1000;
    if (rotation_error > c.at("max_rotation_residual_deg").get<double>() ||
        translation_error > c.at("max_translation_residual_mm").get<double>()) {
        invalidate(g, "INVALID", "kinematic_residual_exceeded");
        return;
    }
    double roll = std::remainder(2 * std::atan2(projection, relative.q[3]), 2 * pi) * 180 / pi;
    double pitch = displacement * 1000 + c.at("pitch_zero_mm").get<double>();
    if (roll < -52 - 1e-6 || roll > 52 + 1e-6 || pitch < -5 - 1e-6 || pitch > 175 + 1e-6) {
        invalidate(g, "INVALID", "physical_feedback_out_of_range");
        return;
    }
    const auto& model = c.at("model");
    g["current"] = {{"roll_deg", roll},
                    {"pitch_mm", pitch},
                    {"roll_percent", roll * 2},
                    {"pitch_percent", (pitch - 85) / 85 * 100},
                    {"confidence", confidence},
                    {"camera_id", at(b, "camera_id")},
                    {"frame_id", at(b, "frame_id")},
                    {"rotation_residual_deg", rotation_error},
                    {"translation_residual_mm", translation_error},
                    {"model_joints",
                     {{"roll_input_joint", model.at("roll_sign").get<double>() * roll * pi / 180 +
                                               model.at("roll_offset_rad").get<double>()},
                      {"pitch_input_joint", model.at("pitch_sign").get<double>() * pitch / 1000 +
                                                model.at("pitch_offset_m").get<double>()}}}};
}
} // namespace
Json State::overview(std::uint64_t now, const std::string& clock) {
    std::lock_guard<std::mutex> lock(mutex);
    Json result{{"schema_version", 1},
                {"monitor_session_id", session_id},
                {"snapshot_revision", ++snapshot_revision},
                {"snapshot_generated_mono_us", now},
                {"arms", Json::object()},
                {"hands", Json::object()},
                {"publishers", Json::array()}};
    bool conflict = false;
    auto choose = [&](aviator::Topic t, bool respect = true) {
        const Stream* s = select(*this, t, now, conflict);
        return std::make_pair(s, group(s, conflict, now, clock, timeout_us(t), respect));
    };
    auto [system, sg] = choose(aviator::Topic::flight_state);
    if (valid(sg)) {
        const auto& b = at(system->message.body, "system");
        if (!at(b, "state").is_string() || !at(b, "control_source").is_string())
            invalidate(sg, "INVALID", "system_fields_missing");
        else
            sg["current"] = {{"state", b["state"]},
                             {"control_source", b["control_source"]},
                             {"current_error_code", at(b, "current_error_code")},
                             {"last_error_code", at(b, "last_error_code")}};
    }
    result["system"] = sg;
    auto [command, cg] = choose(aviator::Topic::flight_command);
    if (valid(cg)) {
        const auto& b = at(command->message.body, "control");
        const auto& roll = at(b, "roll");
        const auto& pitch = at(b, "pitch");
        if (!finite(roll) || !finite(pitch) || std::abs(roll.get<double>()) > 1 ||
            std::abs(pitch.get<double>()) > 1)
            invalidate(cg, "INVALID", "command_out_of_range");
        else
            cg["current"] = {{"roll_normalized", roll},
                             {"pitch_normalized", pitch},
                             {"roll_percent", roll.get<double>() * 100},
                             {"pitch_percent", pitch.get<double>() * 100},
                             {"roll_deg", roll.get<double>() * 50},
                             {"pitch_mm", 85 + pitch.get<double>() * 85},
                             {"control_source", at(command->message.body, "source")}};
    }
    result["flight_command"] = cg;
    for (auto topic : {aviator::Topic::arm_state, aviator::Topic::hand_state}) {
        auto [stream, base] = choose(topic, false);
        bool hand = topic == aviator::Topic::hand_state;
        for (const auto* side : {"left", "right"}) {
            Json g = base;
            if (valid(g)) {
                const auto& b = at(at(stream->message.body, hand ? "hands" : "arms"), side);
                side_age(g, b, now, timeout_us(topic));
                if (valid(g) && !yes(at(b, "valid")))
                    invalidate(g, "INVALID", "side_feedback_invalid");
                if (valid(g) && hand) {
                    const auto& raw = at(b, "drive_position_raw");
                    const auto& values = at(b, "drive_position_normalized");
                    bool ok =
                        yes(at(b, "feedback_available")) && vector(raw, 6) && vector(values, 6);
                    if (ok)
                        for (int i = 0; i < 6; ++i)
                            ok = ok && raw[i].is_number_integer() && raw[i] >= 0 &&
                                 raw[i] <= 1000 && values[i] >= 0 && values[i] <= 1 &&
                                 std::abs(raw[i].get<double>() / 1000 - values[i].get<double>()) <
                                     1e-6;
                    if (!ok)
                        invalidate(g, "INVALID", "invalid_drive_feedback");
                    else {
                        Json joints = Json::object();
                        const auto& calibration = config.at("hand_calibration");
                        if (!calibration.is_null()) {
                            g["calibration_id"] = calibration.at("id");
                            const auto& curves = calibration.at(side);
                            for (int i = 0; i < 6; ++i) {
                                const auto& curve = curves[i];
                                const auto& knots = curve.at("knots");
                                double x = values[i].get<double>();
                                for (unsigned k = 1; k < knots.size(); ++k)
                                    if (x <= knots[k][0].get<double>()) {
                                        double fraction = (x - knots[k - 1][0].get<double>()) /
                                                          (knots[k][0].get<double>() -
                                                           knots[k - 1][0].get<double>());
                                        joints[curve.at("joint").get<std::string>()] =
                                            knots[k - 1][1].get<double>() +
                                            fraction * (knots[k][1].get<double>() -
                                                        knots[k - 1][1].get<double>());
                                        break;
                                    }
                            }
                        }
                        g["current"] = {
                            {"drive_position_raw", raw},
                            {"drive_position_normalized", values},
                            {"commanded_drive_position_normalized",
                             at(b, "commanded_drive_position_normalized")},
                            {"status", at(b, "status")},
                            {"enabled", at(b, "enabled")},
                            {"error_code", at(b, "error_code")},
                            {"grasp_verified", at(b, "grasp_verified")},
                            {"sample_time_basis", at(b, "sample_time_basis")},
                            {"pose_state", "ESTIMATED"},
                            {"pose_mapping", calibration.is_null() ? "URDF_LIMITS" : "CALIBRATED"},
                            {"model_joints", joints}};
                    }
                } else if (valid(g)) {
                    const auto& q = at(b, "joint_position");
                    if (!vector(q, 7))
                        invalidate(g, "INVALID", "invalid_joint_positions");
                    else {
                        Json joints = Json::object();
                        const auto& mapping = config.at("arm_joints").at(side);
                        for (int i = 0; i < 7; ++i)
                            joints[mapping[i].at("name").get<std::string>()] =
                                q[i].get<double>() * mapping[i].at("sign").get<double>() +
                                mapping[i].at("offset_rad").get<double>();
                        g["current"] = {{"joint_position_rad", q},
                                        {"model_joints", joints},
                                        {"status", at(b, "status")},
                                        {"enabled", at(b, "enabled")},
                                        {"error_code", at(b, "error_code")}};
                    }
                }
            }
            result[hand ? "hands" : "arms"][side] = g;
        }
    }
    auto [camera, yg] = choose(aviator::Topic::camera_detection);
    if (valid(yg))
        yoke(yg, camera->message.body, config.at("yoke_calibration"),
             config.at("preview").at("camera_id").get<std::string>());
    result["yoke_observation"] = yg;
    const std::pair<const char*, aviator::Topic> roles[] = {
        {"Flight Gateway", aviator::Topic::flight_command},
        {"Core", aviator::Topic::flight_state},
        {"Manipulator", aviator::Topic::arm_state},
        {"Inspire Hand", aviator::Topic::hand_state},
        {"Camera", aviator::Topic::camera_detection}};
    for (const auto& [name, t] : roles) {
        auto [s, g] = choose(t, false);
        result["publishers"].push_back(
            {{"name", name},
             {"topic", aviator::topic_name(t)},
             {"source", g["source"]},
             {"state", g["measurement_state"] == "SOURCE_CONFLICT" ? Json("SOURCE_CONFLICT")
                                                                   : g["receive_state"]},
             {"receive_age_ms", g["receive_age_ms"]},
             {"timeout_ms", timeout_us(t) / 1000.0}});
    }
    for (const auto* name : {"Logger", "Bus"})
        result["publishers"].push_back(
            {{"name", name}, {"state", "UNOBSERVED"}, {"source", nullptr}});
    return result;
}
} // namespace monitor

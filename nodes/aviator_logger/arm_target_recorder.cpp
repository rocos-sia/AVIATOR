#include "arm_target_recorder.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aviator {
namespace {
uint64_t integer(const Json& j) {
    if (!j.is_number_integer() || (j.is_number_integer() && !j.is_number_unsigned() && j.get<int64_t>() < 0) ||
        j.get<uint64_t>() > max_json_integer) throw std::invalid_argument("invalid integer");
    return j.get<uint64_t>();
}
Json identity(const Header& h) {
    return {{"publisher_id", h.publisher_id}, {"session_id", h.session_id},
            {"sequence", h.sequence}, {"sample_mono_us", h.sample_mono_us}};
}
std::vector<double> values(const Json& j, size_t n) {
    if (!j.is_array() || j.size() != n) throw std::invalid_argument("invalid vector");
    std::vector<double> v;
    for (const auto& x : j) {
        if (!x.is_number() || !std::isfinite(x.get<double>())) throw std::invalid_argument("invalid number");
        v.push_back(x.get<double>());
    }
    return v;
}
}
void ArmTargetRecorder::command(std::string_view payload) {
    ++commands_;
    Message m; std::string error;
    if (!decode("arm.command", payload, m, error) || !m.header.valid) { ++malformed_; return; }
    try {
        if (!((m.body.at("mode") == "JOINT_TRAJECTORY" && m.body.at("execution") == "SYNCHRONIZED_TICKS") ||
              (m.body.at("mode") == "WHEEL_SERVO" && m.body.at("execution") == "LATEST_TARGET")))
            throw std::invalid_argument("unsupported command");
        const auto epoch = m.body.at("control_epoch").get<std::string>();
        for (auto& w : windows_) {
            if (w.header.publisher_id == m.header.publisher_id &&
                w.header.clock_id == m.header.clock_id && w.header.sequence == m.header.sequence && w.epoch == epoch) {
                // Whitespace differences do not create a conflicting trajectory.
                if (Json::parse(w.payload) != Json::parse(payload)) w.conflict = true;
                return;
            }
        }
        if (windows_.size() == 64) { windows_.pop_front(); ++evicted_; }
        windows_.push_back({m.header, epoch, std::string(payload), false});
    } catch (const std::exception&) { ++malformed_; }
}
std::optional<Json> ArmTargetRecorder::state(std::string_view payload) {
    Message m; std::string error;
    if (!decode("arm.state", payload, m, error)) { ++malformed_; return {}; }
    Json out = {{"msg_type", "RecordedArmTarget"}, {"version", "1.0"},
        {"publisher_id", "aviator_logger"}, {"session_id", session_}, {"sequence", ++sequence_},
        {"timestamp", m.header.timestamp}, {"sample_mono_us", uint64_t(0)},
        {"clock_id", m.header.clock_id}, {"valid", false}, {"reason", "unmatched"},
        {"time_basis", "unavailable"}, {"source_state", identity(m.header)},
        {"source_command", nullptr}, {"trajectory_id", nullptr}, {"tick", nullptr},
        {"joint_position", nullptr}, {"joint_velocity", nullptr}, {"joint_acceleration", nullptr},
        {"derivatives_available", false}, {"streaming", nullptr}};
    const auto missing = [&](const char* reason) -> std::optional<Json> {
        out["reason"] = reason; ++missing_; return out;
    };
    try {
        const auto key = m.header.publisher_id + "/" + m.header.session_id + "/" + m.header.clock_id;
        auto it = std::find_if(states_.begin(), states_.end(), [&](const auto& s) { return s.first == key; });
        if (it != states_.end()) {
            if (m.header.sequence <= it->second) return missing("duplicate_or_out_of_order_state");
            it->second = m.header.sequence;
        } else {
            if (states_.size() == 64) states_.pop_front();
            states_.emplace_back(key, m.header.sequence);
        }
        // Execution cursor and measured-joint sample have different time bases.
        if (!m.body.contains("status_mono_us")) return missing("missing_execution_timestamp");
        const auto observed = integer(m.body.at("status_mono_us"));
        out["sample_mono_us"] = observed;
        out["time_basis"] = "arm.state.status_mono_us";
        const auto& e = m.body.at("execution");
        const auto id = integer(e.at("trajectory_id")), tick = integer(e.at("tick"));
        out["trajectory_id"] = id; out["tick"] = tick;
        if (!m.header.valid) return missing("invalid_state");
        if (e.at("fault").get<bool>()) return missing("executor_fault");
        if (e.at("stopping").get<bool>()) return missing("executor_stopping");
        const auto& ack = m.body.at("accepted_command");
        if (ack.is_null()) return missing("no_accepted_command");
        out["source_command"] = ack;
        const auto seq = integer(ack.at("sequence"));
        const Window* found = nullptr;
        for (const auto& w : windows_) {
            if (w.header.publisher_id == ack.at("publisher_id") &&
                w.header.sequence == seq && w.epoch == ack.at("control_epoch") && w.header.clock_id == m.header.clock_id) {
                found = &w; break;
            }
        }
        if (!found) return missing("accepted_window_not_cached");
        if (found->conflict) return missing("conflicting_command_identity");
        const auto b = Json::parse(found->payload);
        if (b.at("config_id") != m.body.at("config_id") || integer(b.at("trajectory_id")) != id)
            return missing("trajectory_or_config_mismatch");
        if (found->header.sample_mono_us > observed) return missing("command_newer_than_state");
        if (b.at("execution") == "LATEST_TARGET") {
            // No joint trajectory is sent on this channel. Record the device's
            // reported planned position explicitly, never reconstruct it from
            // the wheel goal or substitute measured joint velocities.
            out["joint_position"] = values(e.at("target"), 14);
            out["streaming"] = true;
            out["source_command"]["sample_mono_us"] = found->header.sample_mono_us;
            out["valid"] = true;
            out["reason"] = "device_reported_latest_target";
            ++matched_;
            return out;
        }
        const auto first = integer(b.at("first_tick")), total = integer(b.at("total_ticks"));
        if (tick < first || tick > total) return missing("cursor_outside_window");
        const bool streaming = b.value("streaming", false);
        out["streaming"] = streaming;
        const uint64_t stride = streaming ? 1 : 2, offset = tick - first;
        std::vector<double> q, dq, ddq;
        size_t count = 0;
        for (const char* side : {"left", "right"}) {
            const auto& points = b.at("arms").at(side).at("points");
            if (!points.is_array() || points.size() < 2 || points.size() > (streaming ? 51u : 32u) ||
                (count && count != points.size())) return missing("invalid_window");
            count = points.size();
            if (offset > stride * (count - 1)) return missing("cursor_outside_window");
            for (size_t k = 0; k < count; ++k)
                if (integer(points[k].at("time_from_start_us")) != k * stride * 1000) return missing("invalid_time_grid");
            const auto k = offset / stride;
            auto pos = values(points[k].at("joint_position"), 7);
            if (offset % stride) {
                const auto next = values(points[k + 1].at("joint_position"), 7);
                for (size_t j = 0; j < 7; ++j) pos[j] = (pos[j] + next[j]) * .5;
            }
            q.insert(q.end(), pos.begin(), pos.end());
            if (streaming) {
                const auto vel = values(points[k].at("joint_velocity"), 7);
                const auto acc = values(points[k].at("joint_acceleration"), 7);
                dq.insert(dq.end(), vel.begin(), vel.end()); ddq.insert(ddq.end(), acc.begin(), acc.end());
            }
        }
        const auto target = values(e.at("target"), 14);
        for (size_t j = 0; j < 14; ++j)
            if (std::abs(target[j] - q[j]) > 1e-7) return missing("execution_target_mismatch");
        out["joint_position"] = q;
        // Ordinary trajectory windows contain no acceleration samples. Never invent them.
        if (streaming) { out["joint_velocity"] = dq; out["joint_acceleration"] = ddq; }
        out["derivatives_available"] = streaming;
        out["source_command"]["sample_mono_us"] = found->header.sample_mono_us;
        out["valid"] = true; out["reason"] = streaming ? "matched" : "matched_position_only";
        ++matched_;
        return out;
    } catch (const std::exception&) { return missing("malformed_state_or_window"); }
}
Json ArmTargetRecorder::summary() const {
    return {{"mode", "compact"}, {"commands_omitted", commands_}, {"matched_targets", matched_},
            {"unmatched_targets", missing_}, {"invalid_inputs", malformed_}, {"cache_evictions", evicted_},
            {"cache_windows", 64}, {"joint_order", "left.J1..J7,right.J1..J7"}};
}
} // namespace aviator

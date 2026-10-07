#include "HandControl.hpp"
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace aviator {
void HandControl::configure(const std::filesystem::path& path) {
    const auto node = YAML::LoadFile(path.string())["core_hand"];
    enabled_ = node && node["enabled"].as<bool>(false);
    if (!enabled_) return;
    publisher_ = node["publisher_id"].as<std::string>("inspire_hand");
    const int timeout = node["completion_timeout_ms"].as<int>(5000);
    const int feedback = node["feedback_timeout_ms"].as<int>(500);
    tolerance_ = node["open_tolerance"].as<double>(.03);
    if (publisher_.empty() || timeout < 500 || timeout > 30000 || feedback < 200 || feedback > 1000 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0 || tolerance_ > 1.1)
        throw std::runtime_error("Invalid core_hand timing/publisher/open_tolerance");
    timeout_ = uint64_t(timeout) * 1000;
    feedback_timeout_ = uint64_t(feedback) * 1000;
    auto pose = [&](const YAML::Node& p, Pose& out) {
        for (int side = 0; side < 2; ++side) {
            const auto values = p[side ? "right" : "left"];
            if (!values.IsSequence() || values.size() != 6)
                throw std::runtime_error("core_hand pose requires left/right arrays of six normalized values");
            for (int j = 0; j < 6; ++j) {
                const auto value = values[j].as<double>();
                if (!std::isfinite(value) || value < 0 || value > 1)
                    throw std::runtime_error("core_hand positions must be in [0,1]");
                out[side][j] = value;
            }
        }
    };
    if (node["open"]) pose(node["open"], open_);
    if (node["close"] && !node["close"].IsNull()) { pose(node["close"], close_); has_close_ = true; }
}
void HandControl::request(bool close, uint64_t now) {
    if (close && !has_close_)
        throw std::runtime_error("Missing core_hand.close: configure calibrated left/right normalized targets in system.yaml");
    active_ = true;
    synchronized_ = endpoint_ = endpoint_acknowledged_ = false;
    closing_ = close;
    target_ = close ? close_ : open_;
    requested_ = now;
    first_sequence_ = command_sequence_ + 1;
    next_ = 0;
    accepted_at_ = 0;
    acknowledged_ = false;
    error_.clear();
}
void HandControl::beginApproach(uint64_t now) {
    if (!has_close_)
        throw std::runtime_error("Missing core_hand.close for synchronized grasp");
    request(false, now);
    synchronized_ = true;
    closing_ = true;
}
void HandControl::approachProgress(const std::array<double, 2>& progress, uint64_t now) {
    if (!active_ || !synchronized_) throw std::runtime_error("No active synchronized hand trajectory");
    for (auto u : progress)
        if (!std::isfinite(u) || u < 0 || u > 1)
            throw std::runtime_error("Invalid hand trajectory progress");
    for (int side = 0; side < 2; ++side)
        for (int j = 0; j < 6; ++j)
            target_[side][j] = open_[side][j] + progress[side] * (close_[side][j] - open_[side][j]);
    if (!endpoint_ && progress[0] == 1 && progress[1] == 1) {
        endpoint_ = true;
        requested_ = now;
        first_sequence_ = command_sequence_ + 1; // A partial-target ACK cannot complete the grasp.
        next_ = 0; // Publish the exact endpoint without waiting for the next 50 Hz slot.
    }
}
void HandControl::revoke() {
    active_ = false; // Protective revocation is not itself a new device fault.
}
void HandControl::fail(const std::string& reason) {
    active_ = false;
    error_ = reason;
}
bool HandControl::fresh(uint64_t now) const {
    return valid_ && received_ && now >= received_ && now >= sample_ &&
           now - received_ < feedback_timeout_ && now - sample_ < feedback_timeout_;
}
std::string HandControl::messageFault(uint64_t now) const {
    // Message presence is independent of hand validity, ACKs and active targets.
    // Revoking motion on ERROR must not clear this fault before messages resume.
    const auto last = received_ ? received_ : monitor_started_;
    if (enabled_ && monitor_started_ && now > last && now - last > 1000000)
        return "hand.state message timeout (>1000 ms): publisher=" + publisher_;
    return {};
}
void HandControl::receive(const Message& m, uint64_t now, const std::string&) {
    if (m.topic != Topic::hand_state || m.header.publisher_id != publisher_ ||
        m.header.clock_id != local_clock_id() || m.header.sample_mono_us > now ||
        now - m.header.sample_mono_us >= feedback_timeout_) return;
    const bool restarted = m.header.session_id != node_session_;
    if ((!restarted && m.header.sequence <= state_sequence_) ||
        (restarted && state_sequence_ && m.header.sample_mono_us <= sample_)) return;
    try {
        // Validate before consuming the sequence or binding the feedback session.
        const bool readonly = m.body.at("feedback_only").get<bool>();
        const bool accepted = m.body.at("command_valid").get<bool>();
        const auto& ack = m.body.at("accepted_command");
        const auto publisher = ack.at("publisher_id").get<std::string>();
        const auto seq = ack.at("sequence").get<uint64_t>();
        const auto stamp = ack.at("sample_mono_us").get<uint64_t>();
        bool valid = m.header.valid;
        for (const char* side : {"left", "right"}) {
            const auto& hand = m.body.at("hands").at(side);
            if (!hand.at("valid").get<bool>()) { valid = false; continue; }
            const auto sample = hand.at("sample_mono_us").get<uint64_t>();
            if (sample > now || now - sample >= feedback_timeout_) valid = false;
            const auto values = hand.at("drive_position_normalized").get<std::array<double, 6>>();
            for (auto v : values) if (!std::isfinite(v) || v < 0 || v > 1) return;
        }
        node_session_ = m.header.session_id;
        state_sequence_ = m.header.sequence;
        received_ = now;
        sample_ = m.header.sample_mono_us;
        valid_ = valid;
        body_ = m.body;
        if (!active_) return;
        if (readonly) { fail("aviator_hand is feedback-only; restart it in control mode"); return; }
        if (accepted && publisher != "aviator_core") {
            fail("aviator_hand bound to another publisher; restart aviator_hand and Core"); return;
        }
        if (accepted && publisher == "aviator_core" &&
            seq >= first_sequence_ && seq <= command_sequence_ && stamp >= requested_ &&
            stamp <= now && now - stamp < 100000) {
            accepted_at_ = now;
            acknowledged_ = true;
            if (synchronized_ && endpoint_) endpoint_acknowledged_ = true;
        }
    } catch (const Json::exception&) { return; }
}
bool HandControl::complete(uint64_t now) const {
    if (!active_ || !acknowledged_ || !fresh(now) || now - accepted_at_ >= feedback_timeout_) return false;
    if (synchronized_ && (!endpoint_ || !endpoint_acknowledged_))
        return false;
    if (closing_) return true; // CAN write acknowledgement, never a claim of physical grasp.
    for (int side = 0; side < 2; ++side) {
        const auto& hand = body_.at("hands").at(side ? "right" : "left");
        if (hand.at("sample_mono_us").get<uint64_t>() < requested_) return false;
        const auto actual = hand.at("drive_position_normalized").get<std::array<double, 6>>();
        for (int j = 0; j < 6; ++j) if (std::abs(actual[j] - target_[side][j]) > tolerance_) return false;
    }
    return true;
}
std::string HandControl::fault(uint64_t now) const {
    if (!error_.empty()) return error_;
    if (!active_) return {};
    if (acknowledged_ && (!fresh(now) || now - accepted_at_ >= feedback_timeout_))
        return "hand.state feedback/command acknowledgement expired";
    if ((!synchronized_ || endpoint_ || !acknowledged_) && !complete(now) && now - requested_ >= timeout_)
        return "Hand target timeout: check CAN, pose, feedback and publisher binding";
    return {};
}
std::optional<Message> HandControl::command(uint64_t now, uint64_t heartbeat, const std::string& session) {
    if (!enabled_ || !active_) return {};
    const auto reason = fault(now);
    if (!reason.empty()) { fail(reason); return {}; }
    if (!heartbeat) { fail("Core heartbeat missing"); return {}; }
    if (now < heartbeat) {
        fail("Core heartbeat clock order invalid: ahead_us=" + std::to_string(heartbeat - now)); return {};
    }
    if (now - heartbeat >= 100000) {
        fail("Core heartbeat expired: age_us=" + std::to_string(now - heartbeat) +
             " limit_us=100000; hand watchdog applies safe_pose"); return {};
    }
    if (now < next_) return {};
    next_ = now + 20000; // 50 Hz, including arm planning/wait/ordinary Stop.
    auto m = motionMessage(Topic::hand_command, "aviator_core", session, ++command_sequence_);
    m.header.sample_mono_us = now;
    m.body = {{"control_epoch", epoch_}, {"mode", "NORMALIZED_POSITION"},
              {"origin", {{"publisher_id", "aviator_core"}, {"session_id", session},
                          {"sequence", command_sequence_}, {"sample_mono_us", heartbeat},
                          {"clock_id", local_clock_id()}}}};
    for (int side = 0; side < 2; ++side)
        m.body["hands"][side ? "right" : "left"]["drive_position_normalized"] = target_[side];
    return m;
}
} // namespace aviator

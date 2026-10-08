#include "simulation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace simulation {
namespace {
void require(bool value, const std::string& error) { if (!value) throw std::runtime_error(error); }
int id(mjModel* m, mjtObj kind, const std::string& name) {
    int value = mj_name2id(m, kind, name.c_str());
    if (value < 0) throw std::runtime_error("missing model object: " + name);
    return value;
}
double number(const Json& j) {
    require(j.is_number(), "expected number");
    double v = j.get<double>(); require(std::isfinite(v), "non-finite number"); return v;
}
}
Simulation::Simulation(const std::string& path, const Authorization& auth, bool managed)
    : session_(aviator::new_instance_id()), clock_(aviator::local_clock_id()) {
    char error[2048]{};
    model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(error);
    auto* m = model();
    const int camera = mj_name2id(m, mjOBJ_CAMERA, "cockpit_apriltag");
    if (camera >= 0 && m->cam_resolution[2*camera] > 1 && m->cam_resolution[2*camera+1] > 1) {
        camera_resolution_ = {m->cam_resolution[2*camera], m->cam_resolution[2*camera+1]};
        require(cameraWidth() <= 8192 && cameraHeight() <= 8192, "camera resolution too large");
    }
    require(m->nu == 0, "model must use external torque control (nu=0)");
    require(m->opt.timestep > 0 && m->opt.timestep <= .01, "invalid model timestep");
    require(!managed || std::abs(m->opt.timestep - .001) < 1e-12, "device protocol requires 1 ms physics timestep");
    data_.reset(mj_makeData(m)); require(data() != nullptr, "mj_makeData failed");
    mj_resetDataKeyframe(m, data(), id(m, mjOBJ_KEY, "aviator_home"));
    for (int s = 0; s < 2; ++s) {
        const std::string side = s == 0 ? "left" : "right";
        auto add = [&](std::vector<Joint>& group, const std::string& name, double damping) {
            int j = id(m, mjOBJ_JOINT, name);
            require(m->jnt_type[j] == mjJNT_HINGE && m->jnt_limited[j], "expected limited hinge");
            int q = m->jnt_qposadr[j], v = m->jnt_dofadr[j];
            group.push_back({q, v, m->jnt_range[2*j], m->jnt_range[2*j+1], data()->qpos[q]});
            m->dof_damping[v] += damping;
        };
        for (int j = 1; j <= 7; ++j)
            add(arms_[s], std::string("AR5-5_07") + (s == 0 ? "L" : "R") + "-W4C4A2_joint_" + std::to_string(j), managed ? 0 : 80);
        for (const char* joint : {"thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "little_1"})
            add(hands_[s], side + "_" + joint + "_joint", .15);
        tcp_[s] = id(m, mjOBJ_SITE, side + "_tcp");
        data()->eq_active[id(m, mjOBJ_EQUALITY, side + "_grasp")] = 0;
    }
    roll_ = id(m, mjOBJ_JOINT, "roll_input_joint");
    pitch_ = id(m, mjOBJ_JOINT, "pitch_input_joint");
    mj_forward(m, data());
    if (!auth.epoch.empty()) {
        for (int i = 0; i < 3; ++i) {
            aviator::InputPolicy p;
            p.topic = i == 0 ? aviator::Topic::arm_command : i == 1 ? aviator::Topic::hand_command : aviator::Topic::camera_command;
            p.publisher_id = auth.publisher; p.session_id = auth.session; p.clock_id = clock_;
            p.timeout_us = i == 2 ? auth.camera_timeout_us : 50000;
            p.control_epoch = auth.epoch; p.origin_publisher_id = auth.origin_publisher;
            p.origin_session_id = auth.origin_session;
            guards_[i] = std::make_unique<aviator::InputGuard>(p);
        }
    }
}
bool Simulation::command(const aviator::Message& message, std::uint64_t now, std::string& error) {
    try {
        const int slot = message.topic == aviator::Topic::arm_command ? 0 :
            message.topic == aviator::Topic::hand_command ? 1 : message.topic == aviator::Topic::camera_command ? 2 : -1;
        require(slot >= 0, "unknown command topic");
        require(bool(guards_[slot]), "commands disabled: configure authorization");
        const auto& b = message.body;
        if (b.contains("config_id")) require(b.at("config_id") == "aviator-mjcf-v1", "config mismatch");
        std::array<std::vector<double>, 2> targets;
        if (slot < 2) {
            require(!b.at("origin").contains("topic") || b.at("origin").at("topic") == "flight.command", "unsupported origin topic");
            const auto mode = b.at("mode").get<std::string>();
            const bool normalized = slot == 1 && mode == "NORMALIZED_POSITION";
            require(mode == "JOINT_POSITION" || normalized, "unsupported command mode");
            const auto& group = b.at(slot == 0 ? "arms" : "hands");
            for (int s = 0; s < 2; ++s) {
                const auto& joints = slot == 0 ? arms_[s] : hands_[s];
                const auto& side = group.at(s == 0 ? "left" : "right");
                require(!side.contains("grasp") && !side.contains("points") &&
                    !side.contains(normalized ? "joint_position" : "drive_position_normalized"), "conflicting targets");
                const auto& values = side.at(normalized ? "drive_position_normalized" : "joint_position");
                require(values.is_array() && values.size() == joints.size(), "wrong joint count");
                for (std::size_t j = 0; j < joints.size(); ++j) {
                    double v = number(values[j]);
                    if (normalized) { require(v >= 0 && v <= 1, "normalized target out of range"); v = joints[j].low + (1-v)*(joints[j].high-joints[j].low); }
                    require(v >= joints[j].low && v <= joints[j].high, "joint target out of range");
                    targets[s].push_back(v);
                }
            }
        } else {
            require(b.at("camera_id") == camera_id_ && b.at("target") == "YOKE", "unknown camera/target");
            require(b.at("tracking_enabled").is_boolean(), "tracking_enabled must be boolean");
            const double confidence = number(b.at("min_confidence"));
            require(confidence >= 0 && confidence <= 1, "bad confidence");
            const auto& roi = b.at("roi");
            if (!roi.is_null()) {
                for (auto key : {"x", "y", "width", "height"}) {
                    require(roi.at(key).is_number_integer(), "ROI must contain integers");
                    require(number(roi.at(key)) >= 0, "ROI out of range");
                }
                require(number(roi.at("width")) > 0 && number(roi.at("height")) > 0 &&
                    number(roi.at("x")) + number(roi.at("width")) <= cameraWidth() &&
                    number(roi.at("y")) + number(roi.at("height")) <= cameraHeight(), "ROI outside image");
            }
        }
        if (!guards_[slot]->accept(message, now, error)) return false;
        references_[slot] = {{"publisher_id", message.header.publisher_id}, {"session_id", message.header.session_id}, {"sequence", message.header.sequence}};
        if (slot < 2) {
            references_[slot]["control_epoch"] = b.at("control_epoch");
            for (int s = 0; s < 2; ++s) {
                auto& joints = slot == 0 ? arms_[s] : hands_[s];
                for (std::size_t j = 0; j < joints.size(); ++j) joints[j].target = targets[s][j];
            }
            active_[slot] = true;
        } else camera_command_ = b;
        error.clear(); return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}
void Simulation::setInitialWheel(double angle, double displacement) {
    data()->qpos[model()->jnt_qposadr[roll_]] = angle;
    data()->qpos[model()->jnt_qposadr[pitch_]] = displacement;
    // Older/custom models may omit the second wheel.
    for (const auto& [name, value] : {std::pair{"roll_input_joint_2", angle},
                                     std::pair{"pitch_input_joint_2", displacement}}) {
        const int joint = mj_name2id(model(), mjOBJ_JOINT, name);
        if (joint >= 0) data()->qpos[model()->jnt_qposadr[joint]] = value;
    }
    mj_forward(model(), data());
}
bool Simulation::handCommand(const aviator::Message& message, std::uint64_t now, std::string& error) {
    try {
        require(message.topic == aviator::Topic::hand_command, "expected hand.command");
        const auto& b = message.body;
        std::array<std::array<double, 6>, 2> targets{};
        if (message.header.valid) {
            const std::string mode = b.at("mode");
            require(mode == "NORMALIZED_POSITION" || mode == "GRASP_SETPOINT", "unsupported hand mode");
            for (int side = 0; side < 2; ++side) {
                const auto& hand = b.at("hands").at(side ? "right" : "left");
                require(!hand.contains("joint_position"), "mixed hand target modes");
                if (mode == "NORMALIZED_POSITION") {
                    require(!hand.contains("grasp"), "mixed hand target modes");
                    const auto& values = hand.at("drive_position_normalized");
                    require(values.is_array() && values.size() == 6, "expected six hand positions");
                    for (int j = 0; j < 6; ++j) targets[side][j] = number(values[j]);
                } else {
                    require(!hand.contains("drive_position_normalized"), "mixed hand target modes");
                    targets[side].fill(1 - number(hand.at("grasp").at("closure")));
                }
                for (auto& v : targets[side]) {
                    require(v >= 0 && v <= 1, "hand target outside [0,1]");
                    v = std::round(v * 1000) / 1000; // RH56FTP register resolution.
                }
            }
        }
        aviator::InputPolicy policy;
        policy.topic = aviator::Topic::hand_command;
        policy.publisher_id = message.header.publisher_id;
        policy.clock_id = clock_;
        policy.control_epoch = b.at("control_epoch");
        policy.origin_publisher_id = b.at("origin").at("publisher_id");
        policy.timeout_us = policy.origin_timeout_us = 100000;
        // RH56FTP binds publisher/epoch/origin on the first accepted command.
        auto candidate = hand_guard_ ? *hand_guard_ : aviator::InputGuard(policy);
        auto checked = message;
        checked.body["origin"].erase("topic"); // RH56FTP does not constrain this extension.
        const bool accepted = candidate.accept(checked, now, error);
        require(accepted || (!message.header.valid && error == "invalid business data"), error);
        hand_guard_ = std::make_unique<aviator::InputGuard>(std::move(candidate));
        hand_valid_ = message.header.valid;
        for (int side = 0; side < 2; ++side)
            for (int j = 0; j < 6; ++j) {
                auto& joint = hands_[side][j];
                auto& hold = closing_holds_[side][j];
                const int requested = static_cast<int>(std::lround(targets[side][j] * 1000));
                const bool changed = hold.requested != requested;
                const bool release = hold.held >= 0 &&
                    ((hold.held_at_target && changed) || hold.residual_direction * (requested - hold.held) > 0);
                if (!hand_valid_ || release || now < hold.last_command || now - hold.last_command >= 100000)
                    hold = {};
                if (changed) hold.samples.clear();
                hold.requested = hand_valid_ ? requested : -1;
                hold.last_command = now;
                const double effective = hold.held >= 0 ? hold.held / 1000.0 : targets[side][j];
                joint.target = joint.low + (1 - (hand_valid_ ? effective : 1)) * (joint.high - joint.low);
            }
        hand_ack_ = {{"publisher_id", message.header.publisher_id}, {"session_id", message.header.session_id},
                     {"sequence", message.header.sequence}, {"sample_mono_us", message.header.sample_mono_us},
                     {"control_epoch", b.at("control_epoch")}};
        error.clear();
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}
void Simulation::applyHands(std::uint64_t now) {
    if (hand_valid_ && hand_guard_->expired(now)) {
        hand_valid_ = false;
        closing_holds_ = {};
        for (auto& side : hands_) for (auto& joint : side) joint.target = joint.low; // safe_pose: all open
    }
    for (int side = 0; side < 2; ++side) for (int channel = 0; channel < 6; ++channel) {
        auto& j = hands_[side][channel];
        auto& hold = closing_holds_[side][channel];
        // Same ten-second residual OR stable-position policy as RH56FTP defaults.
        if (hand_valid_ && hold.held < 0) {
            const int actual = static_cast<int>(std::lround(1000 * std::clamp(
                (j.high - data()->qpos[j.q]) / (j.high - j.low), 0.0, 1.0)));
            const int residual = actual - hold.requested;
            const int direction = residual < 0 ? -1 : 1;
            if (direction != hold.residual_direction) hold.residual_since = 0;
            hold.residual_direction = direction;
            if (std::abs(residual) <= 10) hold.residual_since = 0;
            else if (!hold.residual_since) hold.residual_since = now;
            if (hold.samples.empty() || now - hold.samples.back().first >= 10000) {
                hold.samples.emplace_back(now, actual);
                while (hold.samples.size() > 1 && now - hold.samples[1].first >= 10000000)
                    hold.samples.pop_front();
                int low = actual, high = actual;
                for (const auto& sample : hold.samples) { low = std::min(low, sample.second); high = std::max(high, sample.second); }
                const bool stable = now - hold.samples.front().first >= 10000000 && high - low < 10;
                const bool residual_held = hold.residual_since && now - hold.residual_since >= 10000000;
                if (stable || residual_held) {
                    hold.held = actual;
                    hold.held_at_target = std::abs(residual) <= 10;
                    j.target = j.low + (1 - actual / 1000.0) * (j.high - j.low);
                }
            }
        }
        data()->qfrc_applied[j.v] = std::clamp(3 * (j.target - data()->qpos[j.q]) + data()->qfrc_bias[j.v], -1.0, 1.0);
    }
}
aviator::Message Simulation::handState(std::uint64_t now, const std::string& publisher) {
    auto out = state(true, now);
    out.header.publisher_id = publisher;
    out.body.erase("config_id");
    out.body["accepted_command"] = hand_ack_;
    out.body["command_valid"] = hand_valid_;
    out.body["feedback_only"] = false;
    for (const char* name : {"left", "right"}) {
        auto& h = out.body["hands"][name];
        h["status"] = hand_valid_ ? "ACTIVE" : "READY";
        h.erase("enabled");
        h["requested_drive_position_normalized"] = Json::array();
        h["closing_hold_active"] = Json::array();
        h["closing_hold_position_normalized"] = Json::array();
        for (const auto& hold : closing_holds_[std::string(name) == "left" ? 0 : 1]) {
            if (hand_valid_) h["requested_drive_position_normalized"].push_back(hold.requested / 1000.0);
            h["closing_hold_active"].push_back(hold.held >= 0);
            h["closing_hold_position_normalized"].push_back(hold.held >= 0 ? Json(hold.held / 1000.0) : Json(nullptr));
        }
        for (const char* field : {"error_codes", "status_codes"})
            h[field] = std::array<int, 6>{};
        // Hardware-only telemetry is unavailable; never invent physical sensor readings.
        for (const char* field : {"force", "current", "temperature"}) h[field] = Json::array();
        h["feedback_samples"] = out.header.sequence;
        h["feedback_io_errors"] = 0;
        h["feedback_last_error"] = "";
    }
    return out;
}
void Simulation::step(std::uint64_t now) {
    auto* m = model(); auto* d = data();
    for (int group = 0; group < 2; ++group) {
        const bool fresh = guards_[group] && !guards_[group]->expired(now);
        auto& sides = group == 0 ? arms_ : hands_;
        if (active_[group] && !fresh) {
            for (auto& joints : sides) for (auto& j : joints) j.target = std::clamp(d->qpos[j.q], j.low, j.high);
            active_[group] = false;
        }
        for (auto& joints : sides) for (const auto& j : joints) {
            // Arm gains follow AviatorRobot_simple; hand gains suit the small finger inertias.
            const double kp = group == 0 ? 1000 : 3;
            const double limit = group == 0 ? 150 : 1;
            d->qfrc_applied[j.v] = std::clamp(kp*(j.target-d->qpos[j.q]) + d->qfrc_bias[j.v], -limit, limit);
        }
    }
    mj_step(m, d);
    // mj_step's derived poses otherwise describe the pre-integration position.
    mj_forward(m, d);
    for (int i = 0; i < m->nq; ++i) require(std::isfinite(d->qpos[i]), "non-finite simulation position");
    for (int i = 0; i < m->nv; ++i) require(std::isfinite(d->qvel[i]), "non-finite simulation velocity");
}
aviator::Message Simulation::envelope(aviator::Topic topic, int slot, std::uint64_t now, bool valid) {
    aviator::Message m; m.topic = topic;
    m.header = {"1.0", ++sequences_[slot], aviator::utc_us(), now, clock_, "simulation", session_, valid};
    m.body["config_id"] = "aviator-mjcf-v1"; return m;
}
aviator::Message Simulation::state(bool hand, std::uint64_t now) {
    int group = hand ? 1 : 0;
    auto out = envelope(hand ? aviator::Topic::hand_state : aviator::Topic::arm_state, group, now, true);
    for (int s = 0; s < 2; ++s) {
        const auto& joints = hand ? hands_[s] : arms_[s];
        Json side = {{"valid", true}, {"enabled", true}, {"error_code", 0}, {"sample_mono_us", now},
            {"status", active_[group] ? "ACTIVE" : references_[group].is_null() ? "READY" : "SAFE"}};
        side["joint_position"] = Json::array(); side["joint_velocity"] = Json::array();
        for (const auto& j : joints) { side["joint_position"].push_back(data()->qpos[j.q]); side["joint_velocity"].push_back(data()->qvel[j.v]); }
        if (!hand) {
            const auto* p = data()->site_xpos + 3*tcp_[s]; mjtNum q[4];
            mju_mat2Quat(q, data()->site_xmat + 9*tcp_[s]);
            side["tcp_pose"] = {{"frame_id", "mujoco_world"}, {"position", {{"x",p[0]}, {"y",p[1]}, {"z",p[2]}}},
                {"orientation", {{"qx",q[1]}, {"qy",q[2]}, {"qz",q[3]}, {"qw",q[0]}}}};
        } else {
            // Emulate the hardware drive channels: 1=open, 0=closed.
            // Quantize once so raw counts and normalized feedback agree exactly.
            side["joint_position"] = nullptr;
            side["joint_velocity"] = nullptr;
            side["feedback_available"] = true;
            side["position_source"] = "mujoco_joint_position";
            side["sample_time_basis"] = "host_simulation_sample";
            side["feedback_age_ms"] = 0.0;
            side["drive_position_raw"] = Json::array();
            side["drive_position_normalized"] = Json::array();
            side["commanded_drive_position_normalized"] = Json::array();
            for (const auto& j : joints) {
                const double range = j.high - j.low;
                const int raw = static_cast<int>(std::lround(1000 *
                    std::clamp((j.high-data()->qpos[j.q])/range, 0.0, 1.0)));
                side["drive_position_raw"].push_back(raw);
                side["drive_position_normalized"].push_back(raw / 1000.0);
                side["commanded_drive_position_normalized"].push_back(
                    std::clamp((j.high-j.target)/range, 0.0, 1.0));
            }
            side["grasp_verified"] = false;
        }
        out.body[hand ? "hands" : "arms"][s == 0 ? "left" : "right"] = side;
    }
    out.body["accepted_command"] = references_[group]; return out;
}
void Simulation::setCameraId(const std::string& id) {
    require(!id.empty() && id.size() <= 80 && id.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos,
        "invalid camera id");
    camera_id_ = id;
}
aviator::Message Simulation::detection(std::uint64_t now, bool captured, bool in_roi,
                                      const mjData* snapshot) {
    const auto* frame = snapshot ? snapshot : data();
    if (captured) ++frame_;
    // Standalone camera acquisition is active at startup, like nodes/camera.
    // Once a command is accepted, preserve its tracking switch and watchdog.
    const bool tracking = references_[2].is_null() || (guards_[2] && !guards_[2]->expired(now) &&
        camera_command_.value("tracking_enabled", false));
    bool valid = captured && in_roi && tracking;
    auto out = envelope(aviator::Topic::camera_detection, 2, now, valid);
    out.body.update({{"camera_id", camera_id_}, {"frame_id", captured ? Json(frame_) : Json(nullptr)},
        {"image_width", cameraWidth()}, {"image_height",cameraHeight()}, {"status", valid ? "TRACKING" : captured ? "SEARCHING" : "OFFLINE"},
        {"confidence", valid ? 1.0 : 0.0}, {"command_ref", references_[2]},
        {"detector", "mujoco_ground_truth"}, {"pose", nullptr},
        {"steering_wheel", {{"valid", valid}, {"reason", valid ? "" : "target_not_tracking"},
            {"theta_rad", nullptr}, {"translation_along_axis_m", nullptr},
            {"calibration_id", "mujoco_ground_truth"}}},
        {"yoke", {{"detected",valid}, {"roll",nullptr}, {"pitch",nullptr}}}});
    if (valid) {
        const int camera = mj_name2id(model(), mjOBJ_CAMERA, "cockpit_apriltag");
        const int tag = mj_name2id(model(), mjOBJ_SITE, "yoke_apriltag_center");
        if (camera >= 0 && tag >= 0) {
            // Target-to-camera transform, meters; optical axes right/down/forward.
            mjtNum world_to_optical[9], rotation[9], offset[3], position[3], q[4];
            mju_transpose(world_to_optical, frame->cam_xmat + 9*camera, 3, 3);
            for (int i = 3; i < 9; ++i) world_to_optical[i] = -world_to_optical[i];
            mju_sub3(offset, frame->site_xpos + 3*tag, frame->cam_xpos + 3*camera);
            mju_mulMatVec3(position, world_to_optical, offset);
            mju_mulMatMat(rotation, world_to_optical, frame->site_xmat + 9*tag, 3, 3, 3);
            mju_mat2Quat(q, rotation);
            out.body["pose"] = {{"position", {{"x",position[0]}, {"y",position[1]}, {"z",position[2]}}},
                {"orientation", {{"qx",q[1]}, {"qy",q[2]}, {"qz",q[3]}, {"qw",q[0]}}}};
            out.body["tag_id"] = 0;
            out.body["target_frame"] = "yoke_apriltag_center";
        }
        // Inverse of Monitor's camera-to-model convention, using measured joints.
        out.body["steering_wheel"]["theta_rad"] = -frame->qpos[model()->jnt_qposadr[roll_]];
        out.body["steering_wheel"]["translation_along_axis_m"] =
            -frame->qpos[model()->jnt_qposadr[pitch_]] - .085;
        // Synthetic calibration: model lower/upper joint limits map to -1/+1.
        for (const auto& entry : {std::pair<const char*,int>{"roll",roll_}, {"pitch",pitch_}}) {
            int j = entry.second; double lo=model()->jnt_range[2*j], hi=model()->jnt_range[2*j+1];
            out.body["yoke"][entry.first] = std::clamp(2*(frame->qpos[model()->jnt_qposadr[j]]-lo)/(hi-lo)-1, -1.0, 1.0);
        }
    }
    return out;
}
}

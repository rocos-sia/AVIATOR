#include "rocos_mujoco/mujoco_simulator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace rocos_mujoco {
bool MujocoSimulator::initializeAviator() {
    try {
        const auto config = YAML::LoadFile(hw_config_path_)["aviator_grasp"];
        if (!config || !config["enabled"].as<bool>(false))
            return true;
        auto id = [&](mjtObj type, const std::string &name) {
            const int value = mj_name2id(m_, type, name.c_str());
            if (value < 0)
                throw std::runtime_error("Missing AVIATOR object: " + name);
            return value;
        };
        for (int side = 0; side < 2; ++side) {
            const std::string name = side == 0 ? "left" : "right";
            aviator_tcp_[side] = id(mjOBJ_SITE, name + "_tcp");
            aviator_handle_[side] = id(mjOBJ_SITE, name + "_handle");
            aviator_weld_[side] = id(mjOBJ_EQUALITY, name + "_grasp");
            const int weld = aviator_weld_[side];
            if (m_->eq_type[weld] != mjEQ_WELD || m_->eq_objtype[weld] != mjOBJ_SITE ||
                m_->eq_obj1id[weld] != aviator_tcp_[side] || m_->eq_obj2id[weld] != aviator_handle_[side])
                throw std::runtime_error("AVIATOR requires matching site welds");
            d_->eq_active[weld] = 0;
        }
        aviator_wheel_[0] = id(mjOBJ_JOINT, "roll_input_joint");
        aviator_wheel_[1] = id(mjOBJ_JOINT, "pitch_input_joint");
        aviator_position_tol_ = config["position_tolerance"].as<double>();
        aviator_rotation_tol_ = config["rotation_tolerance"].as<double>();
        aviator_speed_tol_ = config["speed_tolerance"].as<double>();
        aviator_stable_time_ = config["stable_time"].as<double>();
        for (double value :
             {aviator_position_tol_, aviator_rotation_tol_, aviator_speed_tol_, aviator_stable_time_})
            if (!std::isfinite(value) || value <= 0)
                throw std::runtime_error("Invalid AVIATOR grasp tolerance");
        aviator_channel_ = std::make_unique<aviator::Channel>(aviator_bus_id_, true);
        updateAviator();
        std::cout << "[AVIATOR] Passive wheel feedback and two grasp welds ready\n";
        return true;
    } catch (const std::exception &error) {
        std::cerr << "[AVIATOR] " << error.what() << '\n';
        return false;
    }
}

void MujocoSimulator::updateAviator() {
    if (!aviator_channel_)
        return;
    mj_forward(m_, d_);
    aviator::Channel::Guard lock(*aviator_channel_);
    auto &shared = aviator_channel_->data();
    auto &f = shared.feedback;
    const uint32_t actual_locks =
        (d_->eq_active[aviator_weld_[0]] ? 1u : 0u) | (d_->eq_active[aviator_weld_[1]] ? 2u : 0u);
    if (f.locked && f.locked != actual_locks)
        f.fault = 1;
    f.locked = actual_locks;
    f.heartbeat = aviator::monotonicTime();
    f.time = d_->time;
    f.angle = d_->qpos[m_->jnt_qposadr[aviator_wheel_[0]]];
    f.displacement = d_->qpos[m_->jnt_qposadr[aviator_wheel_[1]]];
    if (joints_.size() == 14)
        for (int i = 0; i < 14; ++i)
            f.joints[i] = d_->qpos[joints_[i].mj_joint_qpos];
    bool aligned = true;
    for (int side = 0; side < 2; ++side) {
        f.velocity[side] = d_->qvel[m_->jnt_dofadr[aviator_wheel_[side]]];
        const int tcp = aviator_tcp_[side], target = aviator_handle_[side];
        mjtNum delta[3], qa[4], qb[4], rotation[3], va[6], vb[6];
        mju_sub3(delta, d_->site_xpos + 3 * tcp, d_->site_xpos + 3 * target);
        mju_mat2Quat(qa, d_->site_xmat + 9 * tcp);
        mju_mat2Quat(qb, d_->site_xmat + 9 * target);
        mju_subQuat(rotation, qa, qb);
        mj_objectVelocity(m_, d_, mjOBJ_SITE, tcp, va, 0);
        mj_objectVelocity(m_, d_, mjOBJ_SITE, target, vb, 0);
        mju_sub(va, va, vb, 6);
        f.position_error[side] = mju_norm3(delta);
        f.rotation_error[side] = mju_norm3(rotation);
        f.relative_speed[side] = std::max(mju_norm3(va + 3), mju_norm3(va));
        aligned = aligned && f.position_error[side] < aviator_position_tol_ &&
                  f.rotation_error[side] < aviator_rotation_tol_ &&
                  f.relative_speed[side] < aviator_speed_tol_;
    }
    const bool enabled =
        joints_.size() == 14 &&
        std::all_of(joints_.begin(), joints_.end(), [](const auto &joint) { return joint.cia.isEnabled(); });
    aviator_stable_ = aligned ? aviator_stable_ + m_->opt.timestep : 0;
    f.ready = !f.fault && enabled && aviator_stable_ >= aviator_stable_time_;
    const bool lost =
        f.locked && (!enabled || f.position_error[0] > .02 || f.position_error[1] > .02 ||
                     f.rotation_error[0] > .15 || f.rotation_error[1] > .15 ||
                     (shared.controller_pid && f.heartbeat - shared.controller_heartbeat > 2.0));
    aviator_bad_time_ = lost ? aviator_bad_time_ + m_->opt.timestep : 0;
    if (aviator_bad_time_ > .1)
        f.fault = 1;
    if (shared.request != f.ack) {
        f.result = aviator::Result::Ok;
        switch (shared.command) {
        case aviator::Command::Lock:
            if (f.fault)
                f.result = aviator::Result::Fault;
            else if (!enabled)
                f.result = aviator::Result::NotEnabled;
            else if (!f.ready)
                f.result = aviator::Result::NotAligned;
            else {
                for (int w : aviator_weld_)
                    d_->eq_active[w] = 1;
                f.locked = 3;
                std::cout << "[AVIATOR] Both handles locked\n";
            }
            break;
        case aviator::Command::Unlock:
            for (int w : aviator_weld_)
                d_->eq_active[w] = 0;
            f.locked = 0;
            break;
        case aviator::Command::ResetFault:
            if (!f.locked) {
                f.fault = 0;
                aviator_bad_time_ = 0;
            } else
                f.result = aviator::Result::Fault;
            break;
        case aviator::Command::None:
            break;
        }
        f.ack = shared.request;
    }
}
} // namespace rocos_mujoco

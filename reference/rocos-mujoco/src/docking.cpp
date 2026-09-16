// SPDX-License-Identifier: GPL-3.0-or-later
#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/docking_protocol.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace rocos_mujoco {
namespace {
struct Error { double position, rotation, linear_speed, angular_speed; };
Error siteError(const mjModel* m, const mjData* d, int site, int target) {
    mjtNum delta[3], qa[4], qb[4], rotation[3], velocity[6];
    mju_sub3(delta, d->site_xpos + 3 * site, d->site_xpos + 3 * target);
    mju_mat2Quat(qa, d->site_xmat + 9 * site);
    mju_mat2Quat(qb, d->site_xmat + 9 * target);
    mju_subQuat(rotation, qa, qb);
    mj_objectVelocity(m, d, mjOBJ_SITE, site, velocity, 0);
    return {mju_norm3(delta), mju_norm3(rotation), mju_norm3(velocity + 3), mju_norm3(velocity)};
}
void markLocked(mjModel* model, int site) {
    const float color[] = {0, 1, 1, 1};  // Cyan target = confirmed attachment.
    std::memcpy(model->site_rgba + 4 * site, color, sizeof(color));
}
}

bool MujocoSimulator::initializeDocking() {
    try {
        const auto config = YAML::LoadFile(hw_config_path_)["docking"];
        if (!config || !config["enabled"].as<bool>(false)) return true;
        auto& s = docking_;
        s.enabled = true;
        s.slave = config["io_slave"].as<int>();
        auto site = [&](const char* key) {
            const int id = mj_name2id(m_, mjOBJ_SITE, config[key].as<std::string>().c_str());
            if (id < 0) throw std::runtime_error(std::string("Missing docking site: ") + key);
            return id;
        };
        auto weld = [&](const char* key, int anchor, int target) {
            const int id = mj_name2id(m_, mjOBJ_EQUALITY, config[key].as<std::string>().c_str());
            if (id < 0 || m_->eq_type[id] != mjEQ_WELD || m_->eq_objtype[id] != mjOBJ_SITE ||
                m_->eq_obj1id[id] != anchor || m_->eq_obj2id[id] != target)
                throw std::runtime_error(std::string("Expected site-based docking weld: ") + key);
            if (m_->site_bodyid[target] != 0) throw std::runtime_error("Docking requires fixed world targets");
            return id;
        };
        s.site_a = site("anchor_a"); s.target_a = site("target_a");
        s.site_b = site("anchor_b"); s.target_b = site("target_b");
        s.weld_a = weld("weld_a", s.site_a, s.target_a);
        s.weld_b = weld("weld_b", s.site_b, s.target_b);
        auto positive = [&](const char* key) {
            const double value = config[key].as<double>();
            if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("Invalid docking parameter: ") + key);
            return value;
        };
        s.position_tolerance = positive("position_tolerance");
        s.rotation_tolerance = positive("rotation_tolerance");
        s.linear_speed_tolerance = positive("linear_speed_tolerance");
        s.angular_speed_tolerance = positive("angular_speed_tolerance");
        s.stable_time = positive("stable_time");
        // With the original 200 Nms/rad damping, the floating-base weld and
        // implicit integration develop a transient oscillation at 1 kHz.
        // Use a separately configured simulation value for this docking demo.
        if (config["joint_damping"]) {
            const double damping = positive("joint_damping");
            for (const auto& joint : joints_) m_->dof_damping[joint.mj_joint_dof] = damping;
        }
        bool found = false;
        for (const auto& joint : joints_) if (joint.slave_id == s.slave) {
            found = joint.pdo.findOffset("digital_inputs", true) >= 0 &&
                    joint.pdo.findOffset("digital_outputs", false) >= 0;
        }
        if (!found) throw std::runtime_error("Docking requires YAML-mapped digital I/O on the selected drive");
        mj_forward(m_, d_);
        const auto a = siteError(m_, d_, s.site_a, s.target_a);
        if (a.position > s.position_tolerance || a.rotation > s.rotation_tolerance)
            throw std::runtime_error("A must start aligned with its target; refusing to snap from a distance");
        d_->eq_active[s.weld_a] = 1;
        d_->eq_active[s.weld_b] = 0;
        s.status = docking::Active;
        std::cout << "[Docking] Step 1: securing A; B waits for a lock request and geometric alignment." << std::endl;
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[Docking] " << error.what() << '\n';
        return false;
    }
}

void MujocoSimulator::updateDocking() {
    auto& s = docking_;
    if (!s.enabled) return;
    // Run on the physics thread, under the same viewer lock as mj_step.
    mj_forward(m_, d_);
    const auto a = siteError(m_, d_, s.site_a, s.target_a);
    const auto b = siteError(m_, d_, s.site_b, s.target_b);
    auto settled = [&](const Error& error) {
        return error.position <= s.position_tolerance && error.rotation <= s.rotation_tolerance &&
               error.linear_speed <= s.linear_speed_tolerance && error.angular_speed <= s.angular_speed_tolerance;
    };
    const double dt = cycle_time_us_ / 1e6;
    const bool released = (s.status & docking::ReleasedA) != 0;
    if (!released && (!d_->eq_active[s.weld_a] || a.position > 5 * s.position_tolerance ||
        a.rotation > 5 * s.rotation_tolerance)) {
        if (!(s.status & docking::Fault))
            std::cerr << "[Docking] A constraint fault: position_m=" << a.position
                      << " rotation_rad=" << a.rotation << " B_active=" << s.b_activated << '\n';
        s.status |= docking::Fault;
        s.status &= ~(docking::LockedA | docking::ReadyB);
        return;
    }
    s.a_stable = settled(a) ? s.a_stable + dt : 0;
    if (!released && !(s.status & docking::LockedA) && s.a_stable >= s.stable_time) {
        s.status |= docking::LockedA;
        markLocked(m_, s.target_a);
        std::cout << "[Docking] A locked at world_origin." << std::endl;
    }
    if (s.status & docking::Fault) return;
    s.b_stable = settled(b) ? s.b_stable + dt : 0;
    s.status &= ~docking::ReadyB;
    if (s.b_stable >= s.stable_time) s.status |= docking::ReadyB;
    int32_t command = 0;
    for (const auto& joint : joints_) if (joint.slave_id == s.slave) {
        const int offset = joint.pd_output_base + joint.pdo.findOffset("digital_outputs", false);
        std::memcpy(&command, static_cast<const char*>(shm_->pdOutputPtr) + offset, sizeof(command));
    }
    const bool release_request = (command & docking::RequestReleaseA) &&
                                !(s.previous_command & docking::RequestReleaseA);
    s.previous_command = command;
    if (!s.b_activated && (command & docking::RequestLockB) &&
        (s.status & docking::LockedA) && s.a_stable >= s.stable_time && (s.status & docking::ReadyB)) {
        d_->eq_active[s.weld_b] = 1;
        s.b_activated = true;
        s.status |= docking::ClosingB;
        s.b_stable = 0; // Require a second stable interval after activating the constraint.
        s.status &= ~docking::ReadyB;
        std::cout << "[Docking] B weld activated: position_error_m=" << b.position
                  << " rotation_error_rad=" << b.rotation << std::endl;
    }
    if (s.b_activated && !d_->eq_active[s.weld_b]) {
        s.status |= docking::Fault;
        s.status &= ~docking::LockedB;
    } else if (s.b_activated && s.b_stable >= s.stable_time) {
        s.status |= docking::LockedB;
        s.status &= ~docking::ClosingB;
        if (!s.b_reported) {
            markLocked(m_, s.target_b);
            std::cout << "[Docking] STEP1_COMPLETE A_error_m=" << a.position
                      << " B_error_m=" << b.position << " B_rotation_rad=" << b.rotation
                      << "; both ends remain locked." << std::endl;
            s.b_reported = true;
        }
    } else if (s.b_reported && (b.position > 5 * s.position_tolerance || b.rotation > 5 * s.rotation_tolerance)) {
        std::cerr << "[Docking] B constraint fault: position_m=" << b.position
                  << " rotation_rad=" << b.rotation << '\n';
        s.status |= docking::Fault;
        s.status &= ~docking::LockedB;
    }
    if (s.status & docking::Fault) return;
    if (!released && release_request && (s.status & docking::LockedB) &&
        d_->eq_active[s.weld_b] && s.b_stable >= s.stable_time) {
        d_->eq_active[s.weld_a] = 0;
        s.status &= ~docking::LockedA;
        s.status |= docking::ReleasedA;
        const float yellow[] = {1, 1, 0, 1};
        std::memcpy(m_->site_rgba + 4 * s.target_a, yellow, sizeof(yellow));
        std::cout << "[Docking] A_RELEASED: B remains locked; world_origin is yellow again." << std::endl;
    }
    if (s.status & docking::ReleasedA) {
        if (d_->eq_active[s.weld_a]) {
            s.status |= docking::Fault; // An external edit must not silently change the support.
            return;
        }
        s.report_elapsed += dt;
        if (s.report_elapsed >= .25) {
            s.report_elapsed = 0;
            const auto* p = d_->site_xpos + 3 * s.site_a;
            // Independent world-frame feedback from MuJoCo, also used by the integration test.
            std::cout << "[Docking] SUPPORT_B A_world_m=" << p[0] << ',' << p[1] << ',' << p[2]
                      << " A_rotation_rad=" << a.rotation << " A_speed_m_s=" << a.linear_speed
                      << " B_error_m=" << b.position << " B_rotation_rad=" << b.rotation << std::endl;
        }
    }
}
} // namespace rocos_mujoco

// SPDX-License-Identifier: GPL-3.0-or-later
#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/crawling_protocol.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace rocos_mujoco {
namespace {
struct Error { double position, rotation, speed, angular_speed; };
Error error(const mjModel* m, const mjData* d, int site, int target) {
    mjtNum dp[3], qa[4], qb[4], dr[3], velocity[6];
    mju_sub3(dp, d->site_xpos + 3 * site, d->site_xpos + 3 * target);
    mju_mat2Quat(qa, d->site_xmat + 9 * site); mju_mat2Quat(qb, d->site_xmat + 9 * target);
    mju_subQuat(dr, qa, qb); mj_objectVelocity(m, d, mjOBJ_SITE, site, velocity, 0);
    return {mju_norm3(dp), mju_norm3(dr), mju_norm3(velocity + 3), mju_norm3(velocity)};
}
}

bool MujocoSimulator::initializeCrawling() {
    try {
        const auto c = YAML::LoadFile(hw_config_path_)["crawling"];
        if (!c || !c["enabled"].as<bool>(false)) return true;
        if (docking_.enabled) throw std::runtime_error("Select either docking or crawling mode");
        if (c["anchors"].size() != 6) throw std::runtime_error("Crawling requires six anchors");
        auto& s = crawling_; s.enabled = true; s.slave = c["io_slave"].as<int>();
        auto positive = [&](const char* key) {
            const double v = c[key].as<double>();
            if (!std::isfinite(v) || v <= 0) throw std::runtime_error(std::string("Invalid crawling parameter: ") + key);
            return v;
        };
        s.position_tolerance = positive("position_tolerance"); s.rotation_tolerance = positive("rotation_tolerance");
        s.linear_speed_tolerance = positive("linear_speed_tolerance"); s.angular_speed_tolerance = positive("angular_speed_tolerance");
        s.stable_time = positive("stable_time");
        const double damping = positive("joint_damping");
        bool io_found = false;
        for (const auto& j : joints_) {
            m_->dof_damping[j.mj_joint_dof] = damping;
            if (j.slave_id == s.slave) io_found = j.pdo.findOffset("digital_inputs", true) >= 0 && j.pdo.findOffset("digital_outputs", false) >= 0;
        }
        if (!io_found) throw std::runtime_error("Crawling requires YAML-mapped digital I/O");
        auto id = [&](int type, const std::string& name) {
            const int value = mj_name2id(m_, type, name.c_str());
            if (value < 0) throw std::runtime_error("Missing crawling object: " + name);
            return value;
        };
        for (int e = 0; e < 2; ++e) {
            auto& end = s.ends[e]; end.site = id(mjOBJ_SITE, "anchor_" + std::to_string(e + 1));
            for (int i = 0; i < 6; ++i) {
                s.markers[i] = id(mjOBJ_SITE, "hex_" + std::to_string(i));
                end.targets[i] = id(mjOBJ_SITE, "hex_" + std::to_string(i) + "_" + std::to_string(e));
                end.welds[i] = id(mjOBJ_EQUALITY, "crawl_" + std::to_string(e) + "_" + std::to_string(i));
                const int w = end.welds[i], target = end.targets[i];
                if (m_->eq_type[w] != mjEQ_WELD || m_->eq_objtype[w] != mjOBJ_SITE ||
                    m_->eq_obj1id[w] != end.site || m_->eq_obj2id[w] != target || m_->site_bodyid[target] != 0)
                    throw std::runtime_error("Expected a site-based weld to a fixed world anchor");
                const auto p = c["anchors"][i]["position"].as<std::vector<double>>();
                const double yaw = c["anchors"][i]["yaw"].as<double>();
                if (p.size() != 3 || !std::isfinite(yaw)) throw std::runtime_error("Invalid anchor pose");
                const mjtNum expected[4] = {e ? 0 : std::cos(yaw / 2), e ? std::cos(yaw / 2) : 0,
                                           e ? std::sin(yaw / 2) : 0, e ? 0 : std::sin(yaw / 2)};
                mjtNum dr[3]; mju_subQuat(dr, m_->site_quat + 4 * target, expected);
                if (mju_norm3(dr) > 1e-8) throw std::runtime_error("Anchor orientation differs between XML and YAML");
                for (int k = 0; k < 3; ++k)
                    if (!std::isfinite(p[k]) || std::abs(m_->site_pos[3 * target + k] - p[k]) > 1e-8 ||
                        std::abs(m_->site_pos[3 * s.markers[i] + k] - p[k]) > 1e-8)
                        throw std::runtime_error("Anchor position differs between XML and YAML");
                d_->eq_active[w] = 0;
            }
        }
        // Startup posture only; running support switches never overwrite qpos.
        const auto initial_q = c["initial_joint_positions"].as<std::vector<double>>();
        if (initial_q.size() != 7 || joints_.size() != 7) throw std::runtime_error("Expected seven initial joint angles");
        const auto hardware = YAML::LoadFile(hw_config_path_)["hardware"];
        for (size_t i = 0; i < joints_.size(); ++i) {
            const auto limits = hardware[i]["limit"];
            if (!std::isfinite(initial_q[i]) || initial_q[i] < limits["lower"].as<double>() || initial_q[i] > limits["upper"].as<double>())
                throw std::runtime_error("Initial posture outside YAML limits");
            d_->qpos[joints_[i].mj_joint_qpos] = initial_q[i];
        }
        mj_forward(m_, d_);
        const auto initial = error(m_, d_, s.ends[0].site, s.ends[0].targets[0]);
        if (initial.position > s.position_tolerance || initial.rotation > s.rotation_tolerance)
            throw std::runtime_error("A must start aligned with anchor 0");
        s.ends[0].attached = 0; d_->eq_active[s.ends[0].welds[0]] = 1;
        s.status = crawling::Active | (1 << 8);
        std::cout << "[Crawl] Six anchors loaded; securing A at 0." << std::endl;
        return true;
    } catch (const std::exception& e) { std::cerr << "[Crawl] " << e.what() << std::endl; return false; }
}

void MujocoSimulator::updateCrawling() {
    auto& s = crawling_; if (!s.enabled) return;
    mj_forward(m_, d_); // Physics thread; same lock as the viewer and mj_step.
    const double dt = cycle_time_us_ / 1e6;
    auto settled = [&](const Error& e) {
        return e.position <= s.position_tolerance && e.rotation <= s.rotation_tolerance &&
               e.speed <= s.linear_speed_tolerance && e.angular_speed <= s.angular_speed_tolerance;
    };
    auto fault = [&](const char* reason) {
        if (!(s.status & crawling::Fault)) std::cerr << "[Crawl] FAULT: " << reason << std::endl;
        s.status |= crawling::Fault; s.pending = 0;
    };
    s.self_check_elapsed += dt;
    if (s.self_check_elapsed >= .01) {
        s.self_check_elapsed = 0;
        for (int i = 0; i < d_->ncon; ++i) {
            const auto& contact = d_->contact[i];
            const int a = contact.geom[0], b = contact.geom[1];
            if (m_->geom_bodyid[a] <= 0 || m_->geom_bodyid[b] <= 0 ||
                m_->geom_type[a] != mjGEOM_MESH || m_->geom_type[b] != mjGEOM_MESH) continue;
            const double clearance = contact.dist;
            s.min_self_clearance = std::min(s.min_self_clearance, clearance);
            if (!std::isfinite(clearance) || clearance < .0005) fault("robot mesh self-collision");
        }
    }
    for (int e = 0; e < 2; ++e) {
        auto& end = s.ends[e];
        for (int i = 0; i < 6; ++i)
            if (bool(d_->eq_active[end.welds[i]]) != (end.attached == i)) fault("unexpected weld activation/deactivation");
        if (end.attached < 0) continue;
        const auto actual = error(m_, d_, end.site, end.targets[end.attached]);
        s.max_position_error = std::max(s.max_position_error, actual.position);
        s.max_rotation_error = std::max(s.max_rotation_error, actual.rotation);
        if (!std::isfinite(actual.position) || !std::isfinite(actual.rotation) ||
            actual.position > 5 * s.position_tolerance || actual.rotation > 5 * s.rotation_tolerance)
            fault("support position/orientation error");
        end.stable = settled(actual) ? end.stable + dt : 0;
        if (!end.confirmed && end.stable >= s.stable_time && !(s.status & crawling::Fault)) {
            end.confirmed = true;
            std::cout << "[Crawl] LOCKED end=" << e << " anchor=" << end.attached
                      << " error_m=" << actual.position << " rotation_rad=" << actual.rotation << std::endl;
        }
    }
    if (s.ends[0].attached < 0 && s.ends[1].attached < 0) fault("both ends free");
    int32_t command = 0;
    for (const auto& j : joints_) if (j.slave_id == s.slave)
        std::memcpy(&command, static_cast<const char*>(shm_->pdOutputPtr) + j.pd_output_base +
                    j.pdo.findOffset("digital_outputs", false), sizeof(command));
    const int seq = (command >> 8) & 255;
    // Clearing a command cancels a pending pre-lock request; activated welds stay held.
    if (!command) { s.pending = 0; s.candidate_stable = 0; }
    if (seq && seq != s.seen && !(s.status & crawling::Fault)) {
        s.seen = seq; s.pending = command; s.candidate_stable = 0; s.status &= ~crawling::Rejected;
    }
    auto acknowledge = [&](bool rejected) {
        s.ack = (s.pending >> 8) & 255;
        if (rejected) s.status |= crawling::Rejected;
        s.pending = 0; s.candidate_stable = 0;
    };
    if (s.pending && !(s.status & crawling::Fault)) {
        const int op = s.pending & 15, e = (s.pending >> 4) & 1, target = (s.pending >> 5) & 7;
        auto& end = s.ends[e]; auto& other = s.ends[1 - e];
        if (target >= 6 || (s.status & crawling::Done)) acknowledge(true);
        else if (op == crawling::Lock) {
            if (end.attached >= 0) {
                if (end.attached != target) acknowledge(true);
                else if (end.confirmed) acknowledge(false);
            } else if (!other.confirmed || other.attached == target) acknowledge(true);
            else {
                const auto actual = error(m_, d_, end.site, end.targets[target]);
                s.candidate_stable = settled(actual) && other.stable >= s.stable_time ? s.candidate_stable + dt : 0;
                if (s.candidate_stable >= s.stable_time) {
                    end.attached = target; end.confirmed = false; end.stable = 0;
                    d_->eq_active[end.welds[target]] = 1; // Static site weld: never teleport or edit qpos/eq_data.
                    std::cout << "[Crawl] WELD_ON end=" << e << " anchor=" << target << std::endl;
                }
            }
        } else if (op == crawling::Release) {
            if (end.attached != target || !end.confirmed || !other.confirmed) acknowledge(true);
            else if (other.stable >= s.stable_time) {
                d_->eq_active[end.welds[target]] = 0;
                end.attached = -1; end.confirmed = false; end.stable = 0;
                std::cout << "[Crawl] RELEASED end=" << e << " anchor=" << target
                          << " support=" << 1 - e << " at=" << other.attached << std::endl;
                acknowledge(false);
            }
        } else if (op == crawling::Finish) {
            if (!s.ends[0].confirmed || !s.ends[1].confirmed) acknowledge(true);
            else {
                s.status |= crawling::Done;
                std::cout << "[Crawl] FINISHED max_support_error_m=" << s.max_position_error
                          << " max_support_rotation_rad=" << s.max_rotation_error
                          << " min_self_clearance_m=" << s.min_self_clearance << std::endl;
                acknowledge(false);
            }
        } else acknowledge(true);
    }
    s.status &= ~(crawling::LockedA | crawling::LockedB | crawling::Busy | (63 << 8) | (255 << 16));
    for (int e = 0; e < 2; ++e) {
        if (s.ends[e].confirmed) s.status |= 1 << e;
        s.status |= (s.ends[e].attached + 1) << (8 + 3 * e);
    }
    s.status |= s.ack << 16;
    if (s.pending) s.status |= crawling::Busy;
    for (int i = 0; i < 6; ++i) {
        bool occupied = false;
        for (const auto& end : s.ends) occupied |= end.attached == i && end.confirmed;
        const float color[] = {occupied ? 0.f : 1.f, 1.f, occupied ? 1.f : 0.f, 1.f};
        std::memcpy(m_->site_rgba + 4 * s.markers[i], color, sizeof(color));
    }
    s.report_elapsed += dt;
    if (s.report_elapsed >= .25) {
        s.report_elapsed = 0;
        const auto* a = d_->site_xpos + 3 * s.ends[0].site; const auto* b = d_->site_xpos + 3 * s.ends[1].site;
        std::cout << "[Crawl] POSE A=" << a[0] << ',' << a[1] << ',' << a[2]
                  << " B=" << b[0] << ',' << b[1] << ',' << b[2]
                  << " anchors=" << s.ends[0].attached << ',' << s.ends[1].attached
                  << " inward_z=" << d_->site_xmat[9 * s.ends[0].site + 8]
                  << ',' << -d_->site_xmat[9 * s.ends[1].site + 8] << " q=";
        for(size_t i=0;i<joints_.size();++i)
            std::cout << (i ? "," : "") << d_->qpos[joints_[i].mj_joint_qpos];
        std::cout << " dq=";
        for(size_t i=0;i<joints_.size();++i)
            std::cout << (i ? "," : "") << d_->qvel[joints_[i].mj_joint_dof];
        std::cout << std::endl;
    }
}
} // namespace rocos_mujoco

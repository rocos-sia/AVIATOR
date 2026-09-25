#include "aviator/Aviator.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mujoco/mujoco.h>
#include <mutex>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/robot.h>
#include <semaphore.h>
#include <signal.h>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace aviator {
namespace ipc = rocos_mujoco::aviator;
namespace fs = std::filesystem;
using Joints = std::array<double, 14>;
using Feedback = ipc::Feedback;
namespace {
void require(bool value, const std::string &error) {
    if (!value)
        throw std::runtime_error(error);
}
double smooth(double u) { return u * u * u * (10 + u * (-15 + 6 * u)); }
constexpr double radians = 3.14159265358979323846 / 180.0;
KDL::Frame frame(const YAML::Node &node) {
    const auto p = node["position"].as<std::vector<double>>();
    const auto q = node["quaternion"].as<std::vector<double>>();
    require(p.size() == 3 && q.size() == 4, "Invalid grasp frame dimensions");
    double norm = 0;
    for (double value : p)
        require(std::isfinite(value), "Non-finite grasp position");
    for (double value : q) {
        require(std::isfinite(value), "Non-finite grasp quaternion");
        norm += value * value;
    }
    require(std::abs(norm - 1) < 1e-6, "Grasp quaternion must be normalized (wxyz)");
    return {KDL::Rotation::Quaternion(q[1], q[2], q[3], q[0]), KDL::Vector(p[0], p[1], p[2])};
}
} // namespace

class Aviator::Impl {
  public:
    Impl(const std::string &path, int id) : config_path(fs::absolute(path)), bus(id) {}
    ~Impl() {
        cancel = true;
        running = false;
        if (heartbeat_thread.joinable())
            heartbeat_thread.join();
        if (channel && claimed) {
            try {
                ipc::Channel::Guard guard(*channel);
                channel->data().controller_pid = 0;
            } catch (...) {
            }
        }
        if (tick != SEM_FAILED)
            sem_close(tick);
    }
    fs::path config_path;
    int bus;
    YAML::Node config;
    std::unique_ptr<ipc::Channel> channel;
    std::unique_ptr<rocos::Hardware> hardware;
    std::unique_ptr<rocos::Robot> arms[2];
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
    std::mutex operation;
    std::atomic<bool> cancel{false}, running{false};
    std::atomic<int> phase{0}; // disconnected, idle, enabled, approaching, aligned, locked, moving, fault
    std::thread heartbeat_thread;
    bool claimed = false;
    sem_t *tick = SEM_FAILED;
    int joint_ids[14]{};
    int wheel_ids[2]{};
    KDL::Frame handles[2], tool;
    KDL::Frame wheel_origin;
    double approach_distance = .05, plan_dt = .02, max_speed = .7, tracking = .12;
    double joint2_min = 85 * radians, joint2_max = 95 * radians, joint2_margin = .5 * radians;
    Joints home{}, approach_seed{};
    Joints last_target{};

    void checkElbows(const Joints &q) const {
        for (int side = 0; side < 2; ++side)
            require(std::isfinite(q[7 * side + 1]) && q[7 * side + 1] >= joint2_min &&
                        q[7 * side + 1] <= joint2_max,
                    std::string(side == 0 ? "Left" : "Right") +
                        " joint 2 outside elbow-down range [85, 95] deg");
    }

    Feedback status() {
        require(channel != nullptr, "Call Init first");
        auto f = channel->read();
        require(ipc::monotonicTime() - f.heartbeat < .5, "Simulator feedback timeout");
        return f;
    }
    Joints measured() {
        Joints q{};
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis)
                q[side * 7 + axis] = arms[side]->getJointPosition(axis);
        return q;
    }
    void hold() {
        if (!hardware || !arms[0] || !arms[1])
            return;
        auto q = measured();
        ipc::Channel::Guard guard(*channel);
        for (int i = 0; i < 14; ++i)
            arms[i / 7]->setJointPosition(i % 7, q[i]);
        last_target = q;
    }
    void waitTick() {
        timespec deadline{};
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_nsec += 100000000;
        if (deadline.tv_nsec >= 1000000000) {
            ++deadline.tv_sec;
            deadline.tv_nsec -= 1000000000;
        }
        while (sem_timedwait(tick, &deadline) != 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("Simulator cycle timeout");
        }
    }
    void command(ipc::Command command) {
        uint64_t sequence;
        {
            ipc::Channel::Guard guard(*channel);
            auto &shared = channel->data();
            shared.command = command;
            sequence = ++shared.request;
        }
        const double until = ipc::monotonicTime() + 2;
        while (ipc::monotonicTime() < until) {
            const auto f = status();
            if (f.ack == sequence) {
                require(f.result == ipc::Result::Ok, "Simulator rejected grasp command (result=" +
                                                         std::to_string(static_cast<int>(f.result)) + ")");
                return;
            }
            waitTick();
        }
        throw std::runtime_error("Grasp command acknowledgement timeout");
    }
    KDL::Frame wheel(double angle, double translation) const {
        return wheel_origin * KDL::Frame(KDL::Rotation::RotZ(angle), KDL::Vector(0, 0, translation));
    }
    KDL::Frame target(int side, double angle, double translation, double retreat = 0) const {
        auto flange = wheel(angle, translation) * handles[side] * tool.Inverse();
        // Withdraw along the mounting stem, perpendicular to the cylinder axis.
        flange.p = flange.p - flange.M * KDL::Vector(0, 0, retreat);
        return flange;
    }
    Joints solve(const std::array<KDL::Frame, 2> &targets, const Joints &seed) {
        Joints result{};
        for (int side = 0; side < 2; ++side) {
            require(!cancel, "Motion cancelled while solving IK");
            KDL::JntArray initial(7), output(7);
            for (int axis = 0; axis < 7; ++axis)
                initial(axis) = seed[side * 7 + axis];
            require(arms[side]->kinematics_.CartToJnt(initial, targets[side], output) >= 0,
                    std::string(side == 0 ? "Left" : "Right") +
                        " arm IK failed within elbow-down joint 2 limits; target/path is unreachable");
            for (int axis = 0; axis < 7; ++axis)
                result[side * 7 + axis] = output(axis);
        }
        return result;
    }
    void check(const Joints &q, double angle, double translation, bool raw = false) {
        checkElbows(q);
        auto *m = model.get();
        auto *d = data.get();
        for (int i = 0; i < 14; ++i) {
            const int j = joint_ids[i];
            require(std::isfinite(q[i]) && q[i] >= m->jnt_range[2 * j] && q[i] <= m->jnt_range[2 * j + 1],
                    "Joint limit violation on axis " + std::to_string(i));
            d->qpos[m->jnt_qposadr[j]] = q[i];
        }
        d->qpos[m->jnt_qposadr[wheel_ids[0]]] = angle;
        d->qpos[m->jnt_qposadr[wheel_ids[1]]] = translation;
        if (raw) return; // raw mode: keep isfinite/joint-limit, skip collision detection
        mj_forward(m, d);
        for (int c = 0; c < d->ncon; ++c) {
            const auto &contact = d->contact[c];
            if (contact.dist < -.001) {
                auto name = [&](int geom) {
                    const char *value = mj_id2name(m, mjOBJ_GEOM, geom);
                    return value ? std::string(value) : "geom#" + std::to_string(geom);
                };
                throw std::runtime_error("Planned collision: " + name(contact.geom1) +
                                         " / " + name(contact.geom2));
            }
        }
    }
    struct Sample {
        Joints q;
        double angle, translation;
    };
    using Path = std::vector<Sample>;
    void validate(Path &path, double duration) {
        require(path.size() >= 2, "Empty trajectory");
        const double dt = duration / (path.size() - 1);
        for (size_t k = 0; k < path.size(); ++k) {
            require(!cancel, "Motion cancelled while planning");
            check(path[k].q, path[k].angle, path[k].translation);
            if (k == 0)
                continue;
            for (int axis = 0; axis < 14; ++axis)
                require(std::abs(path[k].q[axis] - path[k - 1].q[axis]) / dt <=
                            std::min(max_speed, arms[axis / 7]->getJntVelLimit(axis % 7)),
                        "Planned joint speed exceeds limit on axis " + std::to_string(axis) + " at sample " +
                            std::to_string(k) + "; increase duration or choose another target");
            // Check between IK knots as well as at knots.
            Joints mid{};
            for (int axis = 0; axis < 14; ++axis)
                mid[axis] = .5 * (path[k].q[axis] + path[k - 1].q[axis]);
            check(mid, .5 * (path[k].angle + path[k - 1].angle),
                  .5 * (path[k].translation + path[k - 1].translation));
        }
    }
    void execute(const Path &path, double duration, bool locked) {
        const double start = status().time;
        try {
            for (;;) {
                waitTick();
                require(!cancel, "Motion stopped");
                const auto f = status();
                require(!f.fault, "Simulator grasp fault");
                require(!locked || f.locked == 3, "Both handles must remain locked");
                require(arms[0]->IsEnabled() && arms[1]->IsEnabled(), "A drive is no longer enabled");
                const Joints actual = measured();
                checkElbows(actual);
                for (int i = 0; i < 14; ++i)
                    require(std::abs(actual[i] - last_target[i]) < tracking,
                            "Tracking error on axis " + std::to_string(i));
                const double u = std::clamp((f.time - start) / duration, 0., 1.);
                const double coordinate = u * (path.size() - 1);
                const auto k = std::min(static_cast<size_t>(coordinate), path.size() - 2);
                const double fraction = coordinate - k;
                Joints next{};
                for (int i = 0; i < 14; ++i)
                    next[i] = path[k].q[i] + fraction * (path[k + 1].q[i] - path[k].q[i]);
                {
                    ipc::Channel::Guard guard(*channel);
                    for (int i = 0; i < 14; ++i)
                        arms[i / 7]->setJointPosition(i % 7, next[i]);
                }
                last_target = next;
                if (u >= 1)
                    break;
            }
        } catch (...) {
            hold();
            phase = 7;
            throw;
        }
    }
    void settle(bool require_ready, double angle = 0, double translation = 0) {
        const double until = ipc::monotonicTime() + 5;
        double stable = 0;
        while (ipc::monotonicTime() < until) {
            waitTick();
            require(!cancel, "Motion stopped");
            const auto f = status();
            require(!f.fault, "Grasp fault while settling");
            checkElbows(measured());
            bool good = require_ready
                            ? f.ready != 0
                            : std::abs(f.angle - angle) < config["wheel_angle_tolerance"].as<double>() &&
                                  std::abs(f.displacement - translation) <
                                      config["wheel_translation_tolerance"].as<double>() &&
                                  f.position_error[0] < .003 && f.position_error[1] < .003 &&
                                  f.rotation_error[0] < .035 && f.rotation_error[1] < .035 &&
                                  std::abs(f.velocity[0]) < .01 && std::abs(f.velocity[1]) < .002;
            if (good) {
                if (stable == 0)
                    stable = f.time;
                if (f.time - stable > .15)
                    return;
            } else
                stable = 0;
        }
        const auto f = status();
        throw std::runtime_error("Settle timeout: wheel=" + std::to_string(f.angle) + "," +
                                 std::to_string(f.displacement) +
                                 " grasp errors=" + std::to_string(f.position_error[0]) + "," +
                                 std::to_string(f.position_error[1]));
    }
};

Aviator::Aviator(const std::string &config, int bus) : impl_(std::make_unique<Impl>(config, bus)) {}
Aviator::~Aviator() = default;
void Aviator::Init() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    require(!p.channel, "Already initialized");
    p.config = YAML::LoadFile(p.config_path.string());
    auto resolve = [&](const char *key) {
        return p.config_path.parent_path() / p.config[key].as<std::string>();
    };
    const auto grasp = YAML::LoadFile(resolve("grasp").string());
    const auto posture = YAML::LoadFile(resolve("posture").string());
    const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
    require(limits.size() == 2 && limits[0] == 85 && limits[1] == 95,
            "Expected elbow-down joint 2 limits [85, 95] deg");
    p.joint2_min = limits[0] * radians;
    p.joint2_max = limits[1] * radians;
    p.joint2_margin = posture["joint2_planning_margin_deg"].as<double>() * radians;
    require(std::isfinite(p.joint2_margin) && p.joint2_margin > 0 && p.joint2_margin < 5 * radians,
            "Invalid joint 2 planning margin");
    for (int side = 0; side < 2; ++side) {
        const auto values = posture[side == 0 ? "left_home_deg" : "right_home_deg"].as<std::vector<double>>();
        require(values.size() == 7, "Expected seven home angles per arm");
        for (int axis = 0; axis < 7; ++axis) {
            require(std::isfinite(values[axis]), "Non-finite home angle");
            p.home[7 * side + axis] = values[axis] * radians;
        }
        const auto seed = posture[side == 0 ? "left_approach_seed_deg" : "right_approach_seed_deg"]
                              .as<std::vector<double>>();
        require(seed.size() == 7, "Expected seven approach seed angles per arm");
        for (int axis = 0; axis < 7; ++axis) {
            require(std::isfinite(seed[axis]), "Non-finite approach seed");
            p.approach_seed[7 * side + axis] = seed[axis] * radians;
        }
    }
    p.checkElbows(p.home);
    p.checkElbows(p.approach_seed);
    p.handles[0] = frame(grasp["left"]);
    p.handles[1] = frame(grasp["right"]);
    p.tool = frame(grasp["tool"]);
    const double cylinder_radius = grasp["tool"]["radius"].as<double>();
    const double cylinder_length = grasp["tool"]["length"].as<double>();
    require(std::isfinite(cylinder_radius) && cylinder_radius > 0 && std::isfinite(cylinder_length) &&
                cylinder_length > 0,
            "Invalid grasp cylinder dimensions");
    for (int side = 0; side < 2; ++side) {
        auto geometry = grasp["handle_geometry"][side == 0 ? "left" : "right"];
        auto center = geometry["center"].as<std::vector<double>>();
        auto direction = geometry["axis"].as<std::vector<double>>();
        require(center.size() == 3 && direction.size() == 3, "Invalid handle geometry");
        KDL::Vector axis(direction[0], direction[1], direction[2]);
        require(std::abs(axis.Norm() - 1) < 1e-6, "Handle axis must be normalized");
        const auto delta = p.handles[side].p - KDL::Vector(center[0], center[1], center[2]);
        const double offset = KDL::dot(delta, axis);
        require((delta - offset * axis).Norm() < 1e-6 &&
                    std::abs(KDL::dot(p.handles[side].M.UnitZ(), axis)) > 1 - 1e-6 &&
                    std::abs(offset) + cylinder_length / 2 <= geometry["half_length"].as<double>(),
                "Cylinder must be coaxial with and contained in the handle grasp segment");
    }
    p.approach_distance = grasp["approach_distance"].as<double>();
    p.plan_dt = p.config["planning_period"].as<double>();
    p.max_speed = p.config["joint_speed"].as<double>();
    p.tracking = p.config["tracking_tolerance"].as<double>();
    require(std::isfinite(p.plan_dt) && p.plan_dt > 0 && p.plan_dt <= .05 && p.max_speed > 0 &&
                p.tracking > 0,
            "Invalid planning parameters");
    auto model_path = resolve("model");
    if (!fs::is_regular_file(model_path))
        model_path = p.config_path.parent_path().parent_path() / "model/aviator.xml";
    char error[1024]{};
    p.model.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    require(p.model != nullptr, std::string("Cannot load planning model: ") + error);
    p.data.reset(mj_makeData(p.model.get()));
    require(p.data != nullptr, "Cannot allocate planning data");
    auto *m = p.model.get();
    auto *d = p.data.get();
    require(m->nq == 16 && m->nv == 16, "Expected AVIATOR 16-DOF model");
    for (int i = 0; i < m->neq; ++i)
        d->eq_active[i] = 0;
    mj_forward(m, d);
    for (int i = 0; i < 14; ++i) {
        const std::string name =
            std::string("AR5-5_07") + (i < 7 ? "L" : "R") + "-W4C4A2_joint_" + std::to_string(i % 7 + 1);
        p.joint_ids[i] = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        require(p.joint_ids[i] >= 0, "Missing arm joint " + name);
    }
    const int home_key = mj_name2id(m, mjOBJ_KEY, "aviator_home");
    require(home_key >= 0, "Missing aviator_home keyframe; regenerate assets");
    for (int i = 0; i < 14; ++i)
        require(std::abs(m->key_qpos[home_key * m->nq + m->jnt_qposadr[p.joint_ids[i]]] - p.home[i]) < 1e-8,
                "Home config differs from MJCF; regenerate assets");
    p.wheel_ids[0] = mj_name2id(m, mjOBJ_JOINT, "roll_input_joint");
    p.wheel_ids[1] = mj_name2id(m, mjOBJ_JOINT, "pitch_input_joint");
    const int body = mj_name2id(m, mjOBJ_BODY, "steering_wheel");
    require(body >= 0 && p.wheel_ids[0] >= 0 && p.wheel_ids[1] >= 0, "Missing passive wheel");
    const auto *r = d->xmat + 9 * body;
    const auto *x = d->xpos + 3 * body;
    p.wheel_origin = KDL::Frame(KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
                                KDL::Vector(x[0], x[1], x[2]));
    for (int side = 0; side < 2; ++side) {
        const auto name = std::string(side == 0 ? "left" : "right");
        const int site = mj_name2id(m, mjOBJ_SITE, (name + "_handle").c_str());
        require(site >= 0, "Missing handle site");
        KDL::Frame actual;
        const auto *a = d->site_xmat + 9 * site;
        const auto *b = d->site_xpos + 3 * site;
        actual = KDL::Frame(KDL::Rotation(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]),
                            KDL::Vector(b[0], b[1], b[2]));
        const auto difference = KDL::diff(actual, p.wheel_origin * p.handles[side]);
        require(difference.vel.Norm() < 1e-8 && difference.rot.Norm() < 1e-8,
                "Grasp config differs from MJCF; regenerate assets");
        const int flange = mj_name2id(
            m, mjOBJ_BODY, (std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2_flan_link").c_str());
        const int tcp = mj_name2id(m, mjOBJ_SITE, (name + "_tcp").c_str());
        const int cylinder = mj_name2id(m, mjOBJ_GEOM, (name + "_grasp_cylinder").c_str());
        require(flange >= 0 && tcp >= 0 && cylinder >= 0, "Missing cylinder tool");
        auto pose = [](const mjtNum *r, const mjtNum *p) {
            return KDL::Frame(KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
                              KDL::Vector(p[0], p[1], p[2]));
        };
        auto actual_tool = pose(d->xmat + 9 * flange, d->xpos + 3 * flange).Inverse() *
                           pose(d->site_xmat + 9 * tcp, d->site_xpos + 3 * tcp);
        auto tool_error = KDL::diff(actual_tool, p.tool);
        require(tool_error.vel.Norm() < 1e-8 && tool_error.rot.Norm() < 1e-8 &&
                    m->geom_type[cylinder] == mjGEOM_CYLINDER &&
                    std::abs(m->geom_size[3 * cylinder] - cylinder_radius) < 1e-9 &&
                    std::abs(m->geom_size[3 * cylinder + 1] - cylinder_length / 2) < 1e-9,
                "Cylinder tool config differs from MJCF; regenerate assets");
    }
    p.channel = std::make_unique<ipc::Channel>(p.bus);
    p.status();
    {
        ipc::Channel::Guard guard(*p.channel);
        auto &shared = p.channel->data();
        require(shared.controller_pid == 0 || (kill(shared.controller_pid, 0) != 0 && errno == ESRCH),
                "Another AVIATOR controller owns this simulator");
        shared.controller_pid = getpid();
        shared.controller_heartbeat = ipc::monotonicTime();
        p.claimed = true;
    }
    auto *bus_config = rocos::SharedMemoryConfig::getInstance(p.bus);
    require(bus_config->getSlaveNum() == 14, "Expected 14 drive slaves");
    require(bus_config->getDt() == 1000, "Controller expects a 1000 us simulation cycle");
    p.hardware = std::make_unique<rocos::Hardware>(resolve("urdf").string(), p.bus);
    for (int side = 0; side < 2; ++side) {
        const std::string tip = std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2_flan_link";
        p.arms[side] =
            std::make_unique<rocos::Robot>(p.hardware.get(), resolve("urdf").string(), "aircraft", tip, true);
        require(p.arms[side]->getJointNum() == 7 && p.arms[side]->GetRobotState() != "ERROR_STATE",
                "Robot initialization failed");
        KDL::JntArray lower(7), upper(7);
        for (int axis = 0; axis < 7; ++axis) {
            const int joint = p.joint_ids[side * 7 + axis];
            lower(axis) = m->jnt_range[2 * joint];
            upper(axis) = m->jnt_range[2 * joint + 1];
        }
        // Rebuild TRAC-IK with the task limits, retaining the physical URDF limits.
        // The inward margin leaves room for servo tracking error at the bounds.
        lower(1) = p.joint2_min + p.joint2_margin;
        upper(1) = p.joint2_max - p.joint2_margin;
        p.arms[side]->kinematics_.setPosLimits(lower, upper);
        p.arms[side]->kinematics_.Initialize(true);
    }
    p.tick = sem_open(("/sync" + std::to_string(p.bus) + "_8").c_str(), 0);
    require(p.tick != SEM_FAILED, "Cannot open simulation cycle semaphore");
    p.running = true;
    p.heartbeat_thread = std::thread([&p] {
        while (p.running) {
            try {
                ipc::Channel::Guard guard(*p.channel);
                p.channel->data().controller_heartbeat = ipc::monotonicTime();
            } catch (...) {
                p.cancel = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    p.last_target = p.measured();
    p.phase = 1;
}
void Aviator::Enable() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    p.status();
    p.cancel = false;
    p.checkElbows(p.measured());
    p.hold();
    for (int side = 0; side < 2; ++side) {
        if (!p.arms[side]->IsEnabled())
            require(p.arms[side]->SetEnabled() == 0, "Failed to enable arm");
    }
    require(p.arms[0]->IsEnabled() && p.arms[1]->IsEnabled(), "Drive enable acknowledgement failed");
    p.phase = p.status().locked ? 5 : 2;
}
void Aviator::Disable() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    require(p.status().locked == 0, "Unlock handles before disabling drives");
    require(p.arms[0]->SetDisabled() == 0 && p.arms[1]->SetDisabled() == 0, "Failed to disable arms");
    p.phase = 1;
}
void Aviator::ApproachHandles() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    const auto f = p.status();
    require(f.locked == 0 && !f.fault && p.arms[0]->IsEnabled() && p.arms[1]->IsEnabled(),
            "Enable both arms and unlock before approach");
    p.cancel = false;
    p.phase = 3;
    try {
        auto start = p.measured();
        p.checkElbows(start);
        Joints seed = p.approach_seed;
        const auto goal = p.solve({p.target(0, f.angle, f.displacement, p.approach_distance),
                                   p.target(1, f.angle, f.displacement, p.approach_distance)},
                                  seed);
        double duration = p.config["approach_duration"].as<double>();
        require(std::isfinite(duration) && duration >= 1, "Invalid approach duration");
        for (int i = 0; i < 14; ++i)
            duration = std::max(duration, 2.0 * std::abs(goal[i] - start[i]) / p.max_speed);
        size_t steps = static_cast<size_t>(std::ceil(duration / p.plan_dt));
        Impl::Path path;
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            Joints q{};
            for (int i = 0; i < 14; ++i)
                q[i] = start[i] + s * (goal[i] - start[i]);
            path.push_back({q, f.angle, f.displacement});
        }
        p.validate(path, duration);
        p.execute(path, duration, false);
        const auto now = p.status();
        duration = p.config["final_approach_duration"].as<double>();
        require(std::isfinite(duration) && duration >= 1, "Invalid final approach duration");
        steps = static_cast<size_t>(std::ceil(duration / p.plan_dt));
        path.clear();
        seed = p.measured();
        KDL::Frame from[2];
        for (int side = 0; side < 2; ++side) {
            KDL::JntArray q(7);
            for (int j = 0; j < 7; ++j)
                q(j) = seed[side * 7 + j];
            require(p.arms[side]->kinematics_.JntToCart(q, from[side]) >= 0, "FK failed");
        }
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            std::array<KDL::Frame, 2> targets;
            for (int side = 0; side < 2; ++side) {
                const auto end = p.target(side, now.angle, now.displacement);
                const auto twist = KDL::diff(from[side], end);
                targets[side] = KDL::addDelta(from[side], twist, s);
            }
            if (k)
                seed = p.solve(targets, seed);
            path.push_back({seed, now.angle, now.displacement});
        }
        p.validate(path, duration);
        p.execute(path, duration, false);
        p.settle(true);
        p.phase = 4;
    } catch (...) {
        p.hold();
        p.phase = 7;
        throw;
    }
}
void Aviator::LockHandles() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    const auto f = p.status();
    p.checkElbows(p.measured());
    if (f.locked == 3 && !f.fault)
        return;
    require(!f.fault && f.position_error[0] < .003 && f.position_error[1] < .003 &&
                f.rotation_error[0] < .035 && f.rotation_error[1] < .035,
            "Move both TCPs to their handle frames before locking");
    p.cancel = false;
    p.settle(true);
    p.command(ipc::Command::Lock);
    require(p.status().locked == 3, "Both welds were not acknowledged");
    p.phase = 5;
}
void Aviator::MoveWheel(double angle, double translation, double duration) {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    const auto f = p.status();
    require(f.locked == 3 && !f.fault, "Lock both handles before MoveWheel");
    require(std::isfinite(angle) && angle >= -.87266 && angle <= .87266 && std::isfinite(translation) &&
                translation >= -.170 && translation <= 0 && std::isfinite(duration) && duration >= .5 &&
                duration <= 120,
            "Invalid wheel target or duration");
    p.cancel = false;
    bool started = false;
    try {
        const size_t steps = static_cast<size_t>(std::ceil(duration / p.plan_dt));
        Impl::Path path;
        auto seed = p.measured();
        p.checkElbows(seed);
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            const double a = f.angle + s * (angle - f.angle),
                         t = f.displacement + s * (translation - f.displacement);
            if (k)
                seed = p.solve({p.target(0, a, t), p.target(1, a, t)}, seed);
            path.push_back({seed, a, t});
        }
        p.validate(path, duration); // All IK/limit/collision checks precede any target write.
        p.phase = 6;
        started = true;
        p.execute(path, duration, true);
        p.settle(false, angle, translation);
        p.phase = 5;
    } catch (...) {
        if (started)
            p.hold();
        p.phase = 7;
        throw;
    }
}
void Aviator::RunJointFeedback(
    double duration,
    const std::function<std::optional<JointTarget>(const rocos_mujoco::aviator::Feedback &)> &controller) {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    require(std::isfinite(duration) && controller,
            "Invalid joint-feedback run arguments");
    auto first = p.status();
    require(first.locked == 3 && !first.fault && p.arms[0]->IsEnabled() && p.arms[1]->IsEnabled(),
            "Both arms must be enabled and handles locked");
    p.cancel = false;
    p.phase = 6;
    try {
        std::ofstream send_trace;
        if (const char* file = std::getenv("AVIATOR_JOINT_SEND_TRACE_FILE")) {
            send_trace.open(file);
            send_trace << "feedback_sim_time,monotonic_time,q0_target\n";
        }
        constexpr double period = .01;
        const bool raw = std::getenv("AVIATOR_T6_RAW") != nullptr;
        const bool infinite = duration <= 0;
        const long long cycles = infinite ? 0 : static_cast<long long>(std::ceil(duration / period));
        for (long long cycle = 0; infinite || cycle <= cycles; ++cycle) {
            const double expected = first.time + cycle * period;
            auto feedback = p.status();
            while (feedback.time + 1e-6 < expected) {
                p.waitTick();
                feedback = p.status();
            }
            require(!p.cancel, "Joint-feedback run was stopped");
            const double lateness = feedback.time - expected;
            require(lateness <= .005,
                    "Joint-feedback cycle missed its 10 ms simulation deadline: cycle=" +
                        std::to_string(cycle) + ", late=" +
                        std::to_string(lateness * 1000) + " ms");
            require(!feedback.fault && feedback.locked == 3 &&
                        p.arms[0]->IsEnabled() && p.arms[1]->IsEnabled(),
                    "Grasp or drive fault during joint-feedback run");
            Joints actual{};
            std::copy(std::begin(feedback.joints), std::end(feedback.joints), actual.begin());
            p.checkElbows(actual);
            if (!raw) {
                for (int axis = 0; axis < 14; ++axis)
                    require(std::abs(actual[axis] - p.last_target[axis]) < p.tracking,
                            "Tracking error on axis " + std::to_string(axis));
            }
            const auto target = controller(feedback);
            if (!target) {
                p.hold();
                continue;
            }
            for (int axis = 0; axis < 14; ++axis) {
                require(std::isfinite(target->q[axis]),
                        "Non-finite joint target on axis " + std::to_string(axis));
                if (!raw)
                    require(std::abs(target->q[axis] - actual[axis]) / period <=
                                std::min(p.max_speed, p.arms[axis / 7]->getJntVelLimit(axis % 7)),
                            "Joint-feedback speed limit on axis " + std::to_string(axis));
            }
            p.check(target->q, target->next_task[0], target->next_task[1], raw);
            Joints midpoint{};
            for (int axis = 0; axis < 14; ++axis)
                midpoint[axis] = .5 * (actual[axis] + target->q[axis]);
            p.check(midpoint, .5 * (feedback.angle + target->next_task[0]),
                    .5 * (feedback.displacement + target->next_task[1]), raw);
            {
                ipc::Channel::Guard guard(*p.channel);
                for (int axis = 0; axis < 14; ++axis)
                    p.arms[axis / 7]->setJointPosition(axis % 7, target->q[axis]);
            }
            if (send_trace)
                send_trace << std::setprecision(17) << feedback.time << ','
                           << ipc::monotonicTime() << ',' << target->q[0] << '\n';
            p.last_target = target->q;
        }
        p.hold();
        p.phase = 5;
    } catch (...) {
        p.hold();
        p.phase = 7;
        throw;
    }
}
void Aviator::UnlockHandles() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    p.status();
    p.hold();
    p.command(ipc::Command::Unlock);
    p.phase = 2;
}
void Aviator::ResetFault() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    p.command(ipc::Command::ResetFault);
    p.cancel = false;
    p.phase = p.arms[0]->IsEnabled() && p.arms[1]->IsEnabled() ? 2 : 1;
}
void Aviator::Stop() noexcept { impl_->cancel = true; }
Feedback Aviator::GetStatus() { return impl_->status(); }
std::string Aviator::GetState() const {
    static const char *names[] = {"disconnected", "idle",   "enabled", "approaching",
                                  "aligned",      "locked", "moving",  "stopped/error"};
    return names[impl_->phase.load()];
}
} // namespace aviator

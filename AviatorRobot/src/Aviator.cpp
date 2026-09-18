#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/DataLink.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/backend.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <mutex>
#include <stdexcept>
#include <time.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace aviator {
namespace fs = std::filesystem;
using Joints = std::array<double, 14>;
namespace {
void require(bool value, const std::string &error) {
    if (!value)
        throw std::runtime_error(error);
}
double smooth(double u) { return u * u * u * (10 + u * (-15 + 6 * u)); }
constexpr double radians = 3.14159265358979323846 / 180.0;
// 与协议/仿真一致的单调时钟(CLOCK_MONOTONIC 秒),用于心跳与开环定时。
double steady_now() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
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
    ~Impl() = default;
    fs::path config_path;
    int bus;
    YAML::Node config;
    std::unique_ptr<DataLink> link;
    std::unique_ptr<Kinematics> kinematics;
    std::unique_ptr<CollisionChecker> collision;
    std::mutex operation;
    std::atomic<bool> cancel{false};
    std::atomic<int> phase{0}; // disconnected, idle, enabled, approaching, aligned, locked, moving, fault
    KDL::Frame handles[2], tool;
    KDL::Frame wheel_origin;
    double approach_distance = .05, plan_dt = .02, max_speed = .7, tracking = .12;
    double settle_duration = 1.5;
    double joint2_min = 85 * radians, joint2_max = 95 * radians, joint2_margin = .5 * radians;
    Joints home{}, approach_seed{};
    Joints last_target{};
    // 轮盘目标(角度,深度)是 MoveWheel 的输入,算法内部跟踪:
    // MoveWheel 完成后更新为目标;接近完成后同步到后端实测位形。
    // 初始为 Home 位形 (0,0),与 MJCF 关键帧一致。当前"实测"位形经 grasp() 获取,
    // 用于接近对准与开环轨迹,而不是用来读回目标。
    double wheel_angle = 0, wheel_displacement = 0;

    void checkElbows(const Joints &q) const {
        for (int side = 0; side < 2; ++side)
            require(std::isfinite(q[7 * side + 1]) && q[7 * side + 1] >= joint2_min &&
                        q[7 * side + 1] <= joint2_max,
                    std::string(side == 0 ? "Left" : "Right") +
                        " joint 2 outside elbow-down range [85, 95] deg");
    }

    GraspState grasp() {
        require(link != nullptr, "Call Init first");
        const auto state = link->graspState();
        require(steady_now() - state.heartbeat < .5, "Backend feedback timeout");
        return state;
    }
    Status status() {
        const auto g = grasp();
        Status s;
        s.angle = wheel_angle;
        s.displacement = wheel_displacement;
        s.position_error[0] = g.position_error[0];
        s.position_error[1] = g.position_error[1];
        s.rotation_error[0] = g.rotation_error[0];
        s.rotation_error[1] = g.rotation_error[1];
        s.locked = g.locked;
        s.ready = g.ready;
        s.fault = g.fault;
        s.ack = g.ack;
        s.result = g.result;
        return s;
    }
    Joints measured() {
        Joints q{};
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis)
                q[side * 7 + axis] = link->getJointPosition(static_cast<Side>(side), axis);
        return q;
    }
    void hold() {
        if (!link)
            return;
        auto q = measured();
        link->setJointPositions(q);
        last_target = q;
    }
    void waitTick() { link->waitTick(); }
    void command(GraspCommand command) {
        const uint64_t sequence = link->sendGraspCommand(command);
        const double until = steady_now() + 2;
        while (steady_now() < until) {
            const auto g = grasp();
            if (g.ack == sequence) {
                require(g.result == GraspResult::Ok, "Backend rejected grasp command (result=" +
                                                         std::to_string(static_cast<int>(g.result)) + ")");
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
            std::array<double, 7> initial{}, output{};
            for (int axis = 0; axis < 7; ++axis)
                initial[axis] = seed[side * 7 + axis];
            require(kinematics->solveIk(static_cast<Side>(side), initial, targets[side], output),
                    std::string(side == 0 ? "Left" : "Right") +
                        " arm IK failed within elbow-down joint 2 limits; target/path is unreachable");
            for (int axis = 0; axis < 7; ++axis)
                result[side * 7 + axis] = output[axis];
        }
        return result;
    }
    void check(const Joints &q, double angle, double translation) {
        checkElbows(q);
        for (int i = 0; i < 14; ++i)
            require(std::isfinite(q[i]) &&
                        q[i] >= kinematics->jointLower(static_cast<Side>(i / 7), i % 7) &&
                        q[i] <= kinematics->jointUpper(static_cast<Side>(i / 7), i % 7),
                    "Joint limit violation on axis " + std::to_string(i));
        collision->check(q, angle, translation);
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
                            std::min(max_speed, link->jointVelLimit(static_cast<Side>(axis / 7), axis % 7)),
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
        const double start = steady_now();
        try {
            for (;;) {
                waitTick();
                require(!cancel, "Motion stopped");
                const auto g = grasp();
                require(!g.fault, "Backend grasp fault");
                require(!locked || g.locked == 3, "Both handles must remain locked");
                require(link->isEnabled(Side::Left) && link->isEnabled(Side::Right),
                        "A drive is no longer enabled");
                const Joints actual = measured();
                checkElbows(actual);
                for (int i = 0; i < 14; ++i)
                    require(std::abs(actual[i] - last_target[i]) < tracking,
                            "Tracking error on axis " + std::to_string(i));
                const double u = std::clamp((steady_now() - start) / duration, 0., 1.);
                const double coordinate = u * (path.size() - 1);
                const auto k = std::min(static_cast<size_t>(coordinate), path.size() - 2);
                const double fraction = coordinate - k;
                Joints next{};
                for (int i = 0; i < 14; ++i)
                    next[i] = path[k].q[i] + fraction * (path[k + 1].q[i] - path[k].q[i]);
                link->setJointPositions(next);
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
    // 等待抓取就绪(接近后的对准确认)。稳定窗口用本地单调时钟,不再依赖仿真时钟。
    void settleReady() {
        const double until = steady_now() + 5;
        double stable = 0;
        while (steady_now() < until) {
            waitTick();
            require(!cancel, "Motion stopped");
            const auto g = grasp();
            require(!g.fault, "Grasp fault while settling");
            checkElbows(measured());
            if (g.ready) {
                const double now = steady_now();
                if (stable == 0)
                    stable = now;
                if (now - stable > .15)
                    return;
            } else
                stable = 0;
        }
        throw std::runtime_error("Grasp did not become ready");
    }
    // 开环驻留:按时间等待运动收尾,统一仿真与真机。期间保留取消/故障/肘部安全检查。
    void dwell(double seconds) {
        const double until = steady_now() + seconds;
        while (steady_now() < until) {
            waitTick();
            require(!cancel, "Motion stopped");
            const auto g = grasp();
            require(!g.fault, "Grasp fault while settling");
            checkElbows(measured());
        }
    }
};

Aviator::Aviator(const std::string &config, int bus) : impl_(std::make_unique<Impl>(config, bus)) {}
Aviator::~Aviator() = default;
void Aviator::Init() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    require(!p.link, "Already initialized");
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
    p.wheel_origin = frame(grasp["wheel_origin"]);
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
    p.settle_duration = p.config["settle_duration"].as<double>();
    require(std::isfinite(p.plan_dt) && p.plan_dt > 0 && p.plan_dt <= .05 && p.max_speed > 0 &&
                p.tracking > 0 && std::isfinite(p.settle_duration) && p.settle_duration >= 0,
            "Invalid planning parameters");
    auto collision_urdf = resolve("collision_urdf");
    if (!fs::is_regular_file(collision_urdf))
        collision_urdf = p.config_path.parent_path().parent_path() / "model/aviator_collision.urdf";

    // 后端工厂:算法只拿到抽象接口,各后端符号隔离在 backend 实现文件里。
    // 由 config 的 backend 字段选择:mujoco(仿真,默认)/ rokae(真机)。
    const std::string backend = p.config["backend"].as<std::string>("mujoco");
    if (backend == "mujoco") {
        p.link = makeMuJoCoDataLink(resolve("urdf").string(), p.bus);
    } else if (backend == "rokae") {
#ifdef AVIATOR_HAVE_ROKAE
        const auto rk = p.config["rokae"];
        p.link = makeRokaeDataLink(resolve("urdf").string(),
                                   rk["left_ip"].as<std::string>(),
                                   rk["right_ip"].as<std::string>(),
                                   rk["local_ip"].as<std::string>());
#else
        throw std::runtime_error(
            "Backend 'rokae' requested but this build has no Rokae xCore SDK; rebuild with the "
            "SDK headers and libraries present (see CMakeLists.txt)");
#endif
    } else {
        throw std::runtime_error("Unknown backend: " + backend);
    }
    const double ik_lo = p.joint2_min + p.joint2_margin;
    const double ik_hi = p.joint2_max - p.joint2_margin;
    p.kinematics = makeTracIkKinematics(resolve("urdf").string(), ik_lo, ik_hi);
    ModelGeometry geometry;
    geometry.handles = {p.handles[0], p.handles[1]};
    geometry.tool = p.tool;
    geometry.wheel_origin = p.wheel_origin;
    geometry.cylinder_radius = cylinder_radius;
    geometry.cylinder_length = cylinder_length;
    geometry.home = p.home;
    p.collision = makePinocchioCollisionChecker(collision_urdf.string(), geometry);
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
        if (!p.link->isEnabled(static_cast<Side>(side)))
            p.link->enable(static_cast<Side>(side));
    }
    require(p.link->isEnabled(Side::Left) && p.link->isEnabled(Side::Right),
            "Drive enable acknowledgement failed");
    p.phase = p.status().locked ? 5 : 2;
}
void Aviator::Disable() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    require(p.status().locked == 0, "Unlock handles before disabling drives");
    p.link->disable(Side::Left);
    p.link->disable(Side::Right);
    p.phase = 1;
}
void Aviator::ApproachHandles() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    const auto f = p.status();
    require(f.locked == 0 && !f.fault && p.link->isEnabled(Side::Left) && p.link->isEnabled(Side::Right),
            "Enable both arms and unlock before approach");
    p.cancel = false;
    p.phase = 3;
    try {
        auto start = p.measured();
        p.checkElbows(start);
        // 接近时对准轮盘"当前"把手位置:轮盘是被动件,仿真里会因重力分量缓慢漂移,
        // 真机上也可能不在理想零点,故取后端实测位形而非内部目标(0,0)。
        const auto wheel0 = p.grasp();
        Joints seed = p.approach_seed;
        const auto goal = p.solve({p.target(0, wheel0.angle, wheel0.displacement, p.approach_distance),
                                   p.target(1, wheel0.angle, wheel0.displacement, p.approach_distance)},
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
            path.push_back({q, wheel0.angle, wheel0.displacement});
        }
        p.validate(path, duration);
        p.execute(path, duration, false);
        // 第二阶段开始前重新读取轮盘位形(第一阶段期间轮盘仍在缓慢漂移)。
        const auto wheel1 = p.grasp();
        duration = p.config["final_approach_duration"].as<double>();
        require(std::isfinite(duration) && duration >= 1, "Invalid final approach duration");
        steps = static_cast<size_t>(std::ceil(duration / p.plan_dt));
        path.clear();
        seed = p.measured();
        KDL::Frame from[2];
        for (int side = 0; side < 2; ++side) {
            std::array<double, 7> q{};
            for (int j = 0; j < 7; ++j)
                q[j] = seed[side * 7 + j];
            require(p.kinematics->solveFk(static_cast<Side>(side), q, from[side]), "FK failed");
        }
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            std::array<KDL::Frame, 2> targets;
            for (int side = 0; side < 2; ++side) {
                const auto end = p.target(side, wheel1.angle, wheel1.displacement);
                const auto twist = KDL::diff(from[side], end);
                targets[side] = KDL::addDelta(from[side], twist, s);
            }
            if (k)
                seed = p.solve(targets, seed);
            path.push_back({seed, wheel1.angle, wheel1.displacement});
        }
        p.validate(path, duration);
        p.execute(path, duration, false);
        p.settleReady();
        // 接近完成后,把内部跟踪的轮盘目标同步到实测位形,使后续 MoveWheel 从真实位形出发。
        {
            const auto w = p.grasp();
            p.wheel_angle = w.angle;
            p.wheel_displacement = w.displacement;
        }
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
    p.settleReady();
    p.command(GraspCommand::Lock);
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
            const double a = p.wheel_angle + s * (angle - p.wheel_angle),
                         t = p.wheel_displacement + s * (translation - p.wheel_displacement);
            if (k)
                seed = p.solve({p.target(0, a, t), p.target(1, a, t)}, seed);
            path.push_back({seed, a, t});
        }
        p.validate(path, duration); // All IK/limit/collision checks precede any target write.
        p.phase = 6;
        started = true;
        p.execute(path, duration, true);
        p.dwell(p.settle_duration);
        p.wheel_angle = angle;
        p.wheel_displacement = translation;
        p.phase = 5;
    } catch (...) {
        if (started)
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
    p.command(GraspCommand::Unlock);
    p.phase = 2;
}
void Aviator::ResetFault() {
    auto &p = *impl_;
    std::lock_guard<std::mutex> operation(p.operation);
    p.command(GraspCommand::ResetFault);
    p.cancel = false;
    p.phase = p.link->isEnabled(Side::Left) && p.link->isEnabled(Side::Right) ? 2 : 1;
}
void Aviator::Stop() noexcept { impl_->cancel = true; }
Status Aviator::GetStatus() { return impl_->status(); }
std::string Aviator::GetState() const {
    static const char *names[] = {"disconnected", "idle",   "enabled", "approaching",
                                  "aligned",      "locked", "moving",  "stopped/error"};
    return names[impl_->phase.load()];
}
} // namespace aviator

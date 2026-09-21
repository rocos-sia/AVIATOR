#include "aviator/Aviator.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/backend.hpp"
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>

namespace aviator {

using Joints = std::array<double, 14>;

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }

void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}

double smooth(double u) { return u * u * u * (10 + u * (-15 + 6 * u)); }

constexpr double radians = M_PI / 180.0;

struct ServoTimeout : std::runtime_error {
    ServoTimeout() : std::runtime_error("Servo command timeout; holding position") {}
};

KDL::Frame parseFrame(const YAML::Node &node) {
    const auto p = node["position"].as<std::vector<double>>();
    const auto q = node["quaternion"].as<std::vector<double>>();
    require(p.size() == 3 && q.size() == 4, "Invalid frame dimensions");
    for (double value : p) require(std::isfinite(value), "Nonfinite frame position");
    double norm = 0;
    for (double val : q) norm += val * val;
    require(std::abs(norm - 1) < 1e-6, "Quaternion must be normalized (wxyz)");
    return {KDL::Rotation::Quaternion(q[1], q[2], q[3], q[0]),
            KDL::Vector(p[0], p[1], p[2])};
}

} // namespace

// Aviator 实现类
class Aviator::Impl {
  public:
    Impl(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
         std::unique_ptr<CollisionChecker> collision_checker, const std::string &config_file)
        : datalink_(std::move(datalink)), kinematics_(std::move(kinematics)),
          collision_checker_(std::move(collision_checker)), config_file_(config_file) {}

    // 后端由配置选择时使用这个构造：Init() 里再调工厂。
    // sim 句柄仅在 backend: mujoco 时被用到。
    Impl(BackendContext context, const std::string &config_file)
        : backend_context_(context), config_file_(config_file) {}

    ~Impl() {
        cancel_ = true;
        shutdown_ = true;
        servo_cv_.notify_all();
        if (servo_thread_.joinable()) servo_thread_.join();
    }

    void Init() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");

        require(GetState() == "UNINITIALIZED", "Init may only be called once");
        // 加载配置
        YAML::Node config = YAML::LoadFile(config_file_);

        approach_duration_ = config["approach_duration"].as<double>(4.0);
        final_approach_duration_ = config["final_approach_duration"].as<double>(3.0);
        planning_period_ = config["planning_period"].as<double>(0.02);
        joint_speed_ = config["joint_speed"].as<double>(1.9);
        tracking_tolerance_ = config["tracking_tolerance"].as<double>(0.12);
        settle_duration_ = config["settle_duration"].as<double>(1.5);
        wheel_angular_speed_ = config["wheel_angular_speed"].as<double>(0.4);
        wheel_linear_speed_ = config["wheel_linear_speed"].as<double>(0.08);
        servo_period_ = config["servo_period"].as<double>(0.02);
        servo_timeout_ = config["servo_timeout"].as<double>(0.25);

        // 加载抓取配置
        std::string config_dir = std::filesystem::absolute(config_file_).parent_path().string();
        YAML::Node grasp = YAML::LoadFile(resolvePath(config, "grasp", config_dir));

        handles_[0] = parseFrame(grasp["left"]);
        handles_[1] = parseFrame(grasp["right"]);
        tool_ = parseFrame(grasp["tool"]);
        wheel_origin_ = parseFrame(grasp["wheel_origin"]);
        approach_distance_ = grasp["approach_distance"].as<double>();

        // 加载姿态配置
        YAML::Node posture = YAML::LoadFile(resolvePath(config, "posture", config_dir));

        auto left_home = posture["left_home_deg"].as<std::vector<double>>();
        auto right_home = posture["right_home_deg"].as<std::vector<double>>();
        require(left_home.size() == 7 && right_home.size() == 7, "Invalid home angles");

        for (int i = 0; i < 7; ++i) {
            home_[i] = left_home[i] * radians;
            home_[7 + i] = right_home[i] * radians;
        }

        auto left_seed = posture["left_approach_seed_deg"].as<std::vector<double>>();
        auto right_seed = posture["right_approach_seed_deg"].as<std::vector<double>>();
        require(left_seed.size() == 7 && right_seed.size() == 7, "Invalid approach seed angles");
        for (int i = 0; i < 7; ++i) {
            approach_seed_[i] = left_seed[i] * radians;
            approach_seed_[7 + i] = right_seed[i] * radians;
        }

        auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        require(limits.size() == 2, "joint2_limits_deg must contain two values");
        joint2_min_ = limits[0] * radians;
        joint2_max_ = limits[1] * radians;
        joint2_margin_ = posture["joint2_planning_margin_deg"].as<double>() * radians;

        require(std::isfinite(planning_period_) && planning_period_ > 0 && planning_period_ <= 0.05,
                "planning_period must be in (0, 0.05]");
        require(std::isfinite(approach_duration_) && approach_duration_ >= 1 &&
                std::isfinite(final_approach_duration_) && final_approach_duration_ >= 1 &&
                std::isfinite(joint_speed_) && joint_speed_ > 0 &&
                std::isfinite(tracking_tolerance_) && tracking_tolerance_ > 0 &&
                std::isfinite(settle_duration_) && settle_duration_ >= 0,
                "Invalid duration, joint speed or tracking tolerance");
        require(std::isfinite(joint2_margin_) && joint2_margin_ >= 0 &&
                joint2_min_ + joint2_margin_ < joint2_max_ - joint2_margin_, "Invalid J2 limits/margin");
        require(std::isfinite(wheel_angular_speed_) && wheel_angular_speed_ > 0 &&
                std::isfinite(wheel_linear_speed_) && wheel_linear_speed_ > 0 &&
                std::isfinite(servo_period_) && servo_period_ >= 0.01 && servo_period_ <= 0.05 &&
                std::isfinite(servo_timeout_) && servo_timeout_ >= 2 * servo_period_,
                "Invalid wheel speed or servo period/timeout");
        checkElbows(home_);
        checkElbows(approach_seed_);

        // 配置驱动的后端创建。已注入后端（测试用）时跳过。
        if (!datalink_) {
            const std::string urdf = resolvePath(config, "urdf", config_dir);
            applyRokaeOverrides(config);

            GraspGeometry geometry;
            geometry.wheel_origin = wheel_origin_;
            geometry.handles[0] = handles_[0];
            geometry.handles[1] = handles_[1];
            geometry.tool = tool_;
            geometry.approach_distance = approach_distance_;

            const std::string backend = config["backend"].as<std::string>("mujoco");
            datalink_ = makeDataLink(backend, backend_context_, urdf, geometry, rokae_);
        }

        // 运动学与碰撞检测统一在这里创建（与后端选择无关）。
        // 已注入时跳过，便于测试替换。
        if (!kinematics_) {
            const std::string urdf = resolvePath(config, "urdf", config_dir);
            const double ik_lo = joint2_min_ + joint2_margin_;
            const double ik_hi = joint2_max_ - joint2_margin_;
            kinematics_ = makeTracIkKinematics(urdf, ik_lo, ik_hi);
        }
        if (!collision_checker_) {
            std::string collision_urdf = resolvePath(config, "collision_urdf", config_dir);
            // 构建目录运行时，配置里的相对路径可能不存在，回退到 bin/model/
            if (!std::filesystem::is_regular_file(collision_urdf)) {
                const std::string fallback = config_dir + "/../model/aviator_collision.urdf";
                if (std::filesystem::is_regular_file(fallback))
                    collision_urdf = fallback;
            }
            std::string srdf = collision_urdf;
            const auto dot = srdf.find_last_of('.');
            if (dot != std::string::npos)
                srdf = srdf.substr(0, dot) + ".srdf";

            GraspCylinder cylinder;
            cylinder.radius = grasp["tool"]["radius"].as<double>(0.028);
            cylinder.length = grasp["tool"]["length"].as<double>(0.060);
            collision_checker_ = makePinocchioCollisionChecker(collision_urdf, srdf, cylinder);
        }

        last_target_ = measured();
        setState("INITIALIZED");
        servo_thread_ = std::thread([this] { servoLoop(); });
    }

    void Enable() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "INITIALIZED" || GetState() == "DISABLED", "Invalid state for enable");

        cancel_ = false;
        checkElbows(measured());
        hold();

        try {
            datalink_->enable(Side::Left);
            datalink_->enable(Side::Right);
        } catch (...) {
            auto error = std::current_exception();
            try { datalink_->disable(Side::Right); } catch (...) {}
            try { datalink_->disable(Side::Left); } catch (...) {}
            std::rethrow_exception(error);
        }

        require(datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right),
                "Drive enable failed");

        setState(grasp().locked ? "LOCKED" : "ENABLED");
    }

    void Disable() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(grasp().locked == 0, "Unlock handles before disabling");

        datalink_->disable(Side::Left);
        datalink_->disable(Side::Right);

        setState("DISABLED");
    }

    void ApproachHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        auto f = grasp();
        require(f.locked == 0 && !f.fault && datalink_->isEnabled(Side::Left) &&
                    datalink_->isEnabled(Side::Right),
                "Enable both arms and unlock before approach");

        cancel_ = false;
        setState("APPROACHING");

        try {
            auto start = measured();
            checkElbows(start);

            // 第一阶段：关节空间运动到预接近位置
            auto wheel0 = grasp();
            Joints seed = approach_seed_;
            auto goal = solve({target(0, wheel0.angle, wheel0.displacement, approach_distance_),
                              target(1, wheel0.angle, wheel0.displacement, approach_distance_)},
                             seed);

            double duration = approach_duration_;
            for (int i = 0; i < 14; ++i)
                duration = std::max(duration, 2.0 * std::abs(goal[i] - start[i]) / joint_speed_);

            size_t steps = static_cast<size_t>(std::ceil(duration / planning_period_));
            Path path;
            for (size_t k = 0; k <= steps; ++k) {
                double s = smooth(double(k) / steps);
                Joints q{};
                for (int i = 0; i < 14; ++i)
                    q[i] = start[i] + s * (goal[i] - start[i]);
                path.push_back({q, wheel0.angle, wheel0.displacement});
            }

            validate(path, duration);
            execute(path, duration, false);

            // 第二阶段：笛卡尔空间精确对准
            auto wheel1 = grasp();
            duration = final_approach_duration_;
            steps = static_cast<size_t>(std::ceil(duration / planning_period_));
            path.clear();

            seed = measured();
            KDL::Frame from[2];
            for (int side = 0; side < 2; ++side) {
                std::array<double, 7> q{};
                for (int j = 0; j < 7; ++j)
                    q[j] = seed[side * 7 + j];
                require(kinematics_->solveFk(static_cast<Side>(side), q, from[side]), "FK failed");
            }

            for (size_t k = 0; k <= steps; ++k) {
                double s = smooth(double(k) / steps);
                std::array<KDL::Frame, 2> targets;
                for (int side = 0; side < 2; ++side) {
                    auto end = target(side, wheel1.angle, wheel1.displacement);
                    auto twist = KDL::diff(from[side], end);
                    targets[side] = KDL::addDelta(from[side], twist, s);
                }
                if (k)
                    seed = solve(targets, seed);
                path.push_back({seed, wheel1.angle, wheel1.displacement});
            }

            validate(path, duration);
            execute(path, duration, false);
            settleReady();

            // 同步轮盘位形
            auto w = grasp();
            wheel_angle_ = w.angle;
            wheel_displacement_ = w.displacement;

            setState("APPROACHED");
        } catch (...) {
            hold();
            setState("FAULT");
            throw;
        }
    }

    void LockHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        auto f = grasp();
        checkElbows(measured());

        if (f.locked == 3 && !f.fault)
            return;

        require(!f.fault && f.position_error[0] < 0.003 && f.position_error[1] < 0.003 &&
                    f.rotation_error[0] < 0.035 && f.rotation_error[1] < 0.035,
                "Move both TCPs to handle frames before locking");

        cancel_ = false;
        settleReady();
        command(GraspCommand::Lock);
        require(grasp().locked == 3, "Both handles not acknowledged");

        setState("LOCKED");
    }

    // Plan once, stretch time to respect wheel AND joint velocity limits, then execute.
    void MoveWheel(double angle, double displacement, double v) {
        validateWheelInput(angle, displacement, v);
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "LOCKED", "Lock handles before MoveWheel; clear faults first");
        requireWheelReady();
        cancel_ = false;
        setMotionError("");
        setState("MOVING");
        try {
            const double a0 = wheel_angle_, d0 = wheel_displacement_;
            // Sample by geometry at full speed; reducing v must not multiply memory/IK work.
            const double nominal = std::max({0.5, 1.875 * std::abs(angle - a0) / wheel_angular_speed_,
                1.875 * std::abs(displacement - d0) / wheel_linear_speed_});
            const size_t steps = static_cast<size_t>(std::ceil(nominal / planning_period_));
            Path path{{measured(), a0, d0}};
            for (size_t k = 1; k <= steps; ++k) {
                const double u = smooth(double(k) / steps);
                const double a = a0 + u * (angle - a0), d = d0 + u * (displacement - d0);
                path.push_back({solve({target(0, a, d), target(1, a, d)}, path.back().q), a, d});
            }
            double duration = nominal / v;
            for (size_t k = 1; k < path.size(); ++k)
                duration = std::max(duration, jointTravelTime(path[k - 1].q, path[k].q, v) * steps);
            duration *= 1.001; // leave room for floating-point roundoff at the limit
            require(std::isfinite(duration), "Speed ratio is too small for a finite trajectory");
            validate(path, duration, v);
            execute(path, duration, true);
            dwell(settle_duration_);
            setState("LOCKED");
        } catch (const std::exception &error) {
            setMotionError(error.what());
            hold();
            setState("FAULT");
            throw;
        }
    }

    // Only validate/publish a target here. IK, collision checks and execution run in servoLoop.
    void ServoWheel(double angle, double displacement, double v) {
        validateWheelInput(angle, displacement, v);
        std::lock_guard<std::mutex> mailbox(servo_mutex_);
        if (!servo_active_) {
            std::unique_lock<std::mutex> control(mutex_, std::try_to_lock);
            require(control.owns_lock(), "Another control operation is running");
            require(GetState() == "LOCKED", "Lock handles before ServoWheel; clear faults first");
            requireWheelReady();
            cancel_ = false;
            setMotionError("");
            servo_active_ = true;
            setState("SERVO");
        } else {
            require(!cancel_, "Servo is stopping; wait for LOCKED before restarting");
        }
        servo_target_ = {angle, displacement, v, std::chrono::steady_clock::now()};
        servo_cv_.notify_one();
    }

    void UnlockHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(grasp().locked != 0, "Must be locked before unlocking");

        command(GraspCommand::Unlock);
        setState("ENABLED");
    }

    void ResetFault() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        command(GraspCommand::ResetFault);
        cancel_ = false;
        setMotionError("");
        const bool enabled = datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right);
        setState(enabled ? (grasp().locked == 3 ? "LOCKED" : "ENABLED") : "DISABLED");
    }

    void SetRealTime(bool enabled) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(datalink_ != nullptr, "Call Init first");
        datalink_->setRealTime(enabled);
    }

    std::mutex *PhysicsMutex() {
        require(datalink_ != nullptr, "Call Init first");
        return datalink_->physicsMutex();
    }

    void Stop() noexcept {
        cancel_ = true;
    }

    Status GetStatus() {
        auto gs = grasp();
        Status status;
        status.open_loop = gs.open_loop;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status.motion_error = motion_error_;
        }
        status.angle = wheel_angle_;
        status.displacement = wheel_displacement_;
        status.position_error[0] = gs.position_error[0];
        status.position_error[1] = gs.position_error[1];
        status.rotation_error[0] = gs.rotation_error[0];
        status.rotation_error[1] = gs.rotation_error[1];
        status.locked = gs.locked;
        status.ready = gs.ready;
        status.fault = gs.fault;
        status.ack = gs.ack;
        status.result = gs.result;
        return status;
    }

    std::string GetState() const { std::lock_guard<std::mutex> lock(status_mutex_); return state_; }
    void setState(const std::string &value) { std::lock_guard<std::mutex> lock(status_mutex_); state_ = value; }

  private:
    struct Sample {
        Joints q;
        double angle, translation;
    };
    using Path = std::vector<Sample>;
    struct ServoTarget {
        double angle = 0, displacement = 0, v = 0.5;
        std::chrono::steady_clock::time_point received;
    };

    static void validateWheelInput(double angle, double displacement, double v) {
        require(std::isfinite(angle) && angle >= -0.87266 && angle <= 0.87266 &&
                std::isfinite(displacement) && displacement >= -0.170 && displacement <= 0 &&
                std::isfinite(v) && v > 0 && v <= 1,
                "Wheel target out of range or speed ratio v not in (0, 1]");
    }

    void requireWheelReady() {
        const auto g = grasp();
        require(g.locked == 3 && !g.fault && datalink_->isEnabled(Side::Left) &&
                datalink_->isEnabled(Side::Right), "Enable and lock both handles before wheel motion");
    }

    void setMotionError(const std::string &error) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        motion_error_ = error;
    }

    double jointTravelTime(const Joints &from, const Joints &to, double v) const {
        double duration = 0;
        for (int i = 0; i < 14; ++i) {
            const double limit = v * std::min(joint_speed_,
                datalink_->jointVelLimit(static_cast<Side>(i / 7), i % 7));
            duration = std::max(duration, std::abs(to[i] - from[i]) / limit);
        }
        return duration;
    }

    bool servoExpired() {
        std::lock_guard<std::mutex> lock(servo_mutex_);
        return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            servo_target_.received).count() > servo_timeout_;
    }

    // Each short segment has zero endpoint velocity/acceleration (quintic easing).
    // Candidate distance is reduced until both arm joint speed limits also fit the segment.
    Path planServoSegment(const ServoTarget &command) {
        const Sample start{last_target_, wheel_angle_, wheel_displacement_};
        const double da = command.angle - start.angle, dd = command.displacement - start.translation;
        double fraction = 1;
        if (da != 0) fraction = std::min(fraction, command.v * wheel_angular_speed_ * servo_period_ / (1.875 * std::abs(da)));
        if (dd != 0) fraction = std::min(fraction, command.v * wheel_linear_speed_ * servo_period_ / (1.875 * std::abs(dd)));
        for (int attempt = 0; attempt < 10; ++attempt) {
            const double angle = start.angle + fraction * da;
            const double displacement = start.translation + fraction * dd;
            const auto q = (da == 0 && dd == 0) ? start.q :
                solve({target(0, angle, displacement), target(1, angle, displacement)}, start.q);
            const double time = 1.875 * jointTravelTime(start.q, q, command.v);
            if (time <= servo_period_) {
                Path segment{start, {q, angle, displacement}};
                // Check endpoints and interior interpolation (not just the new target).
                for (int k = 0; k <= 4; ++k) {
                    const double u = k / 4.0;
                    Joints intermediate{};
                    for (int i = 0; i < 14; ++i) intermediate[i] = start.q[i] + u * (q[i] - start.q[i]);
                    check(intermediate, start.angle + u * (angle - start.angle),
                          start.translation + u * (displacement - start.translation));
                }
                return segment;
            }
            fraction *= std::min(0.5, 0.9 * servo_period_ / time);
        }
        fail("Servo IK cannot satisfy joint velocity limits");
    }

    void servoLoop() {
        for (;;) {
            {
                std::unique_lock<std::mutex> mailbox(servo_mutex_);
                servo_cv_.wait(mailbox, [this] { return shutdown_ || servo_active_; });
                if (shutdown_) return;
            }
            std::unique_lock<std::mutex> control(mutex_);
            std::string next_state = "LOCKED";
            try {
                while (!cancel_ && !shutdown_) {
                    if (servoExpired()) throw ServoTimeout();
                    ServoTarget command;
                    { std::lock_guard<std::mutex> lock(servo_mutex_); command = servo_target_; }
                    requireWheelReady();
                    auto segment = planServoSegment(command);
                    execute(segment, servo_period_, true, true);
                }
                hold();
            } catch (const ServoTimeout &error) {
                setMotionError(error.what());
                try { hold(); } catch (...) { next_state = "FAULT"; }
            } catch (const std::exception &error) {
                setMotionError(error.what());
                // Stop and stale commands hold in the current impedance mode. Other failures latch FAULT.
                next_state = cancel_ ? "LOCKED" : "FAULT";
                try { hold(); } catch (...) { next_state = "FAULT"; }
            }
            std::lock_guard<std::mutex> mailbox(servo_mutex_);
            std::lock_guard<std::mutex> status(status_mutex_);
            servo_active_ = false;
            state_ = next_state;
            control.unlock();
        }
    }

    // 配置里的相对路径按 YAML 所在目录解析（与原项目一致）。
    static std::string resolvePath(const YAML::Node &config, const char *key,
                                   const std::string &base_dir) {
        const std::string value = config[key].as<std::string>();
        if (!value.empty() && value[0] == '/')
            return value;
        return base_dir + "/" + value;
    }

    // backend: rokae 时读取 IP 等参数。
    void applyRokaeOverrides(const YAML::Node &config) {
        const YAML::Node rk = config["rokae"];
        if (!rk)
            return;
        rokae_.left_ip = rk["left_ip"].as<std::string>("");
        rokae_.right_ip = rk["right_ip"].as<std::string>("");
        rokae_.local_ip = rk["local_ip"].as<std::string>("");
        rokae_.grasp_mode = rk["grasp_mode"].as<std::string>("open_loop");
        if (rk["joint_stiffness"]) {
            const auto stiffness = rk["joint_stiffness"].as<std::vector<double>>();
            require(stiffness.size() == 7, "rokae.joint_stiffness requires 7 values");
            std::copy(stiffness.begin(), stiffness.end(), rokae_.joint_stiffness.begin());
        }
    }

    void checkElbows(const Joints &q) const {
        for (int side = 0; side < 2; ++side) {
            const double value = q[7 * side + 1];
            if (!(std::isfinite(value) && value >= joint2_min_ && value <= joint2_max_))
                fail(std::string(side == 0 ? "Left" : "Right") + " joint 2 outside elbow-down range: " +
                     std::to_string(value * 180 / M_PI) + " deg, expected [" +
                     std::to_string(joint2_min_ * 180 / M_PI) + ", " +
                     std::to_string(joint2_max_ * 180 / M_PI) + "] deg");
        }
    }

    GraspState grasp() {
        require(datalink_ != nullptr, "Call Init first");
        auto state = datalink_->graspState();
        require(datalink_->time() - state.heartbeat < 0.5, "Backend feedback timeout");
        return state;
    }

    Joints measured() {
        Joints q{};
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis)
                q[side * 7 + axis] = datalink_->getJointPosition(static_cast<Side>(side), axis);
        return q;
    }

    void hold() {
        if (!datalink_)
            return;
        auto q = measured();
        datalink_->setJointPositions(q);
        last_target_ = q;
    }

    void command(GraspCommand cmd) {
        uint64_t seq = datalink_->sendGraspCommand(cmd);
        double until = datalink_->time() + 2;
        while (datalink_->time() < until) {
            auto g = grasp();
            if (g.ack == seq) {
                require(g.result == GraspResult::Ok, "Backend rejected grasp command");
                return;
            }
            datalink_->waitTick();
        }
        fail("Grasp command acknowledgement timeout");
    }

    KDL::Frame wheel(double angle, double translation) const {
        return wheel_origin_ *
               KDL::Frame(KDL::Rotation::RotZ(angle), KDL::Vector(0, 0, translation));
    }

    KDL::Frame target(int side, double angle, double translation, double retreat = 0) const {
        auto flange = wheel(angle, translation) * handles_[side] * tool_.Inverse();
        flange.p = flange.p - flange.M * KDL::Vector(0, 0, retreat);
        return flange;
    }

    Joints solve(const std::array<KDL::Frame, 2> &targets, const Joints &seed) {
        Joints result{};
        for (int side = 0; side < 2; ++side) {
            require(!cancel_, "Motion cancelled while solving IK");
            std::array<double, 7> initial{}, output{};
            for (int axis = 0; axis < 7; ++axis)
                initial[axis] = seed[side * 7 + axis];
            require(kinematics_->solveIk(static_cast<Side>(side), initial, targets[side], output),
                    std::string(side == 0 ? "Left" : "Right") + " arm IK failed");
            for (int axis = 0; axis < 7; ++axis)
                result[side * 7 + axis] = output[axis];
        }
        return result;
    }

    void check(const Joints &q, double angle, double translation) {
        checkElbows(q);
        for (int i = 0; i < 14; ++i)
            require(std::isfinite(q[i]) &&
                        q[i] >= kinematics_->jointLower(static_cast<Side>(i / 7), i % 7) &&
                        q[i] <= kinematics_->jointUpper(static_cast<Side>(i / 7), i % 7),
                    "Joint limit violation on axis " + std::to_string(i));
        collision_checker_->check(q, angle, translation);
    }

    void validate(const Path &path, double duration, double v = 1.0) {
        require(path.size() >= 2, "Empty trajectory");
        double dt = duration / (path.size() - 1);
        for (size_t k = 0; k < path.size(); ++k) {
            require(!cancel_, "Motion cancelled while planning");
            check(path[k].q, path[k].angle, path[k].translation);
            if (k == 0)
                continue;
            for (int axis = 0; axis < 14; ++axis)
                require(
                    std::abs(path[k].q[axis] - path[k - 1].q[axis]) / dt <=
                        v * std::min(joint_speed_,
                                 datalink_->jointVelLimit(static_cast<Side>(axis / 7), axis % 7)),
                    "Planned joint speed exceeds limit on axis " + std::to_string(axis));

            Joints mid{};
            for (int axis = 0; axis < 14; ++axis)
                mid[axis] = 0.5 * (path[k].q[axis] + path[k - 1].q[axis]);
            check(mid, 0.5 * (path[k].angle + path[k - 1].angle),
                  0.5 * (path[k].translation + path[k - 1].translation));
        }
    }

    void execute(const Path &path, double duration, bool locked, bool servo_segment = false) {
        double start = datalink_->time();
        try {
            for (;;) {
                datalink_->waitTick();
                require(!cancel_, "Motion stopped");
                if (servo_segment && servoExpired()) throw ServoTimeout();
                auto g = grasp();
                require(!g.fault, "Backend grasp fault");
                require(!locked || g.locked == 3, "Both handles must remain locked");
                require(datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right),
                        "A drive is no longer enabled");

                Joints actual = measured();
                checkElbows(actual);
                for (int i = 0; i < 14; ++i)
                    require(std::abs(actual[i] - last_target_[i]) < tracking_tolerance_,
                            "Tracking error on axis " + std::to_string(i));

                double u = std::clamp((datalink_->time() - start) / duration, 0.0, 1.0);
                if (servo_segment) u = smooth(u);
                double coordinate = u * (path.size() - 1);
                auto k = std::min(static_cast<size_t>(coordinate), path.size() - 2);
                double fraction = coordinate - k;

                Joints next{};
                for (int i = 0; i < 14; ++i)
                    next[i] = path[k].q[i] + fraction * (path[k + 1].q[i] - path[k].q[i]);

                const double angle = path[k].angle + fraction * (path[k + 1].angle - path[k].angle);
                const double displacement = path[k].translation +
                    fraction * (path[k + 1].translation - path[k].translation);
                datalink_->setWheelReference(angle, displacement);
                // Preserve the latest reference if Stop interrupts the trajectory.
                wheel_angle_ = angle;
                wheel_displacement_ = displacement;
                datalink_->setJointPositions(next);
                last_target_ = next;

                if (u >= 1)
                    break;
            }
        } catch (...) {
            // Servo owns its asynchronous stop/timeout state transition in servoLoop.
            if (!servo_segment) { hold(); setState("FAULT"); }
            throw;
        }
    }

    void settleReady() {
        double until = datalink_->time() + 5;
        double stable = 0;
        while (datalink_->time() < until) {
            datalink_->waitTick();
            require(!cancel_, "Motion stopped");
            auto g = grasp();
            require(!g.fault, "Grasp fault while settling");
            checkElbows(measured());

            if (g.ready) {
                double now = datalink_->time();
                if (stable == 0)
                    stable = now;
                if (now - stable > 0.15)
                    return;
            } else
                stable = 0;
        }
        fail("Grasp did not become ready");
    }

    void dwell(double seconds) {
        double until = datalink_->time() + seconds;
        while (datalink_->time() < until) {
            datalink_->waitTick();
            require(!cancel_, "Motion stopped");
            auto g = grasp();
            require(!g.fault, "Grasp fault while settling");
            checkElbows(measured());
        }
    }

    std::unique_ptr<DataLink> datalink_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<CollisionChecker> collision_checker_;

    // backend 由配置选择时，Init() 里用这些创建具体后端
    BackendContext backend_context_;
    RokaeConfig rokae_;

    std::string config_file_;
    mutable std::mutex status_mutex_;
    std::string state_ = "UNINITIALIZED";

    double approach_duration_;
    double final_approach_duration_;
    double planning_period_;
    double joint_speed_;
    double tracking_tolerance_;
    double settle_duration_;
    double wheel_angular_speed_, wheel_linear_speed_;
    double servo_period_, servo_timeout_;
    double approach_distance_;

    double joint2_min_, joint2_max_, joint2_margin_;

    KDL::Frame handles_[2], tool_, wheel_origin_;
    Joints home_{}, approach_seed_{};
    Joints last_target_{};

    std::atomic<double> wheel_angle_{0.0};
    std::atomic<double> wheel_displacement_{0.0};

    // One control owner; one small mailbox for nonblocking streaming commands.
    std::mutex servo_mutex_;
    std::condition_variable servo_cv_;
    ServoTarget servo_target_;
    std::thread servo_thread_;
    std::atomic<bool> servo_active_{false}, shutdown_{false};
    std::string motion_error_;
    std::atomic<bool> cancel_{false};
    mutable std::mutex mutex_;
};

// Aviator 外部接口实现
Aviator::Aviator(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
                 std::unique_ptr<CollisionChecker> collision_checker,
                 const std::string &config_file)
    : impl_(std::make_unique<Impl>(std::move(datalink), std::move(kinematics),
                                   std::move(collision_checker), config_file)) {}

// 配置驱动：后端、运动学、碰撞检测都在 Init() 里按 config 创建。
// model/data 仅在 backend: mujoco 时被使用，真机传 nullptr。
Aviator::Aviator(mjModel *model, mjData *data, const std::string &config_file)
    : impl_(std::make_unique<Impl>(BackendContext{model, data}, config_file)) {}

Aviator::~Aviator() = default;

void Aviator::Init() { impl_->Init(); }
void Aviator::Enable() { impl_->Enable(); }
void Aviator::Disable() { impl_->Disable(); }
void Aviator::ApproachHandles() { impl_->ApproachHandles(); }
void Aviator::LockHandles() { impl_->LockHandles(); }
void Aviator::MoveWheel(double angle_rad, double displacement_m, double v) {
    impl_->MoveWheel(angle_rad, displacement_m, v);
}
void Aviator::ServoWheel(double angle_rad, double displacement_m, double v) {
    impl_->ServoWheel(angle_rad, displacement_m, v);
}
void Aviator::UnlockHandles() { impl_->UnlockHandles(); }
void Aviator::ResetFault() { impl_->ResetFault(); }
void Aviator::SetRealTime(bool enabled) { impl_->SetRealTime(enabled); }
std::mutex *Aviator::PhysicsMutex() { return impl_->PhysicsMutex(); }
void Aviator::Stop() noexcept { impl_->Stop(); }
Status Aviator::GetStatus() { return impl_->GetStatus(); }
std::string Aviator::GetState() const { return impl_->GetState(); }

} // namespace aviator

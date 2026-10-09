#include "Logger.hpp"
#include "aviator/Aviator.hpp"
#include "ServoPlanner.hpp"
#include "ServoBufferConfig.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/backend.hpp"
#include "aviator/GraspTools.hpp"
#include <yaml-cpp/yaml.h>
#include "aviator/Pose.hpp"
#include <ruckig/ruckig.hpp>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>
#include <vector>
#include <future>

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
constexpr double command_period = 0.001;

struct ServoTimeout : std::runtime_error {
    ServoTimeout() : std::runtime_error("Servo command timeout; holding position") {}
};

pinocchio::SE3 parseFrame(const YAML::Node &node) {
    const auto p = node["position"].as<std::vector<double>>();
    const auto q = node["quaternion"].as<std::vector<double>>();
    require(p.size() == 3 && q.size() == 4, "Invalid frame dimensions");
    for (double value : p) require(std::isfinite(value), "Nonfinite frame position");
    double norm = 0;
    for (double val : q) norm += val * val;
    require(std::abs(norm - 1) < 1e-6, "Quaternion must be normalized (wxyz)");
    return {Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized(),
            Eigen::Vector3d(p[0], p[1], p[2])};
}

} // namespace

// Aviator 实现类
class Aviator::Impl {
  public:
    Impl(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
         std::unique_ptr<CollisionChecker> collision_checker, const std::string &config_file)
        : datalink_(std::move(datalink)), kinematics_(std::move(kinematics)),
          collision_checker_(std::move(collision_checker)), config_file_(config_file) {}

    ~Impl() {
        cancel_ = true;
        shutdown_ = true;
        servo_cv_.notify_all();
        if (servo_thread_.joinable()) servo_thread_.join();
    }

    void init() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");

        require(GetState() == "UNINITIALIZED", "Init may only be called once");
        // 加载配置
        YAML::Node config = YAML::LoadFile(config_file_);
        collision_check_enabled_ = config["collision_check_enabled"].as<bool>(true);
        aviator::Logger::info("Planning collision checks: {}", (collision_check_enabled_ ? "enabled" : "disabled"));

        home_position_tolerance_ = config["home_position_tolerance"].as<double>(0.02);
        require(std::isfinite(home_position_tolerance_) && home_position_tolerance_ > 0 && home_position_tolerance_ <= 0.15,
                "Invalid home_position_tolerance (rad): expected (0, 0.15]");
        home_speed_ = config["home_speed"].as<double>(0.1);
        approach_speed_ = config["approach_speed"].as<double>(0.1);
        joint_acceleration_ = config["joint_acceleration"].as<double>(0.2);
        joint_jerk_ = config["joint_jerk"].as<double>(1.0);
        final_approach_duration_ = config["final_approach_duration"].as<double>(3.0);
        planning_period_ = config["planning_period"].as<double>(0.02);
        joint_speed_ = config["joint_speed"].as<double>(1.9);
        stop_acceleration_ = config["stop_acceleration"].as<double>(2.0);
        settle_duration_ = config["settle_duration"].as<double>(1.5);
        wheel_angular_speed_ = config["wheel_angular_speed"].as<double>(0.4);
        wheel_linear_speed_ = config["wheel_linear_speed"].as<double>(0.08);
        servo_period_ = config["servo_period"].as<double>(0.02);
        servo_timeout_ = config["servo_timeout"].as<double>(0.25);
        wheel_acceleration_ = {config["wheel_angular_acceleration"].as<double>(0.05),
                               config["wheel_linear_acceleration"].as<double>(0.005)};
        wheel_jerk_ = {config["wheel_angular_jerk"].as<double>(0.05),
                       config["wheel_linear_jerk"].as<double>(0.005)};
        for (int i = 0; i < 2; ++i)
            require(std::isfinite(wheel_acceleration_[i]) && wheel_acceleration_[i] > 0 &&
                    std::isfinite(wheel_jerk_[i]) && wheel_jerk_[i] > 0, "Invalid wheel acceleration/jerk");
        require(std::abs(servo_period_ * 1000 - std::round(servo_period_ * 1000)) < 1e-8,
                "servo_period must be an integer number of milliseconds");
        servo_buffer_ = ServoBufferConfig::load(config, servo_period_);
        aviator::Logger::info("Servo buffer: prefill_ms={} effective_prefill_ms={} lookahead_ms={} block_ms={}",
            servo_buffer_.prefill_ms, servo_buffer_.effective_prefill_ms,
            servo_buffer_.lookahead_ms, std::llround(servo_period_ * 1000));

        // 加载抓取配置
        std::string config_dir = std::filesystem::absolute(config_file_).parent_path().string();
        YAML::Node grasp = YAML::LoadFile(resolvePath(config, "grasp", config_dir));

        handles_[0] = parseFrame(grasp["left"]);
        handles_[1] = parseFrame(grasp["right"]);
        for (int side = 0; side < 2; ++side)
            tools_[side] = parseFrame(toolFrameConfig(grasp, side));
        wheel_origin_ = parseFrame(grasp["wheel_origin"]);
        approach_distance_ = grasp["approach_distance"].as<double>(); // Release retreat only.
        hand_closing_distance_ = grasp["hand_closing_distance"].as<double>(0.07);
        require(std::isfinite(hand_closing_distance_) && hand_closing_distance_ > 0,
                "hand_closing_distance must be positive and finite (m)");

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
        require(std::isfinite(home_speed_) && home_speed_ > 0 &&
                std::isfinite(approach_speed_) && approach_speed_ > 0 &&
                std::isfinite(joint_acceleration_) && joint_acceleration_ > 0 &&
                std::isfinite(joint_jerk_) && joint_jerk_ > 0 &&
                std::isfinite(final_approach_duration_) && final_approach_duration_ >= 1 &&
                std::isfinite(joint_speed_) && joint_speed_ > 0 &&
                std::isfinite(stop_acceleration_) && stop_acceleration_ > 0 &&
                std::isfinite(settle_duration_) && settle_duration_ >= 0,
                "Invalid duration, joint speed, acceleration or jerk");
        require(std::isfinite(joint2_margin_) && joint2_margin_ >= 0 &&
                joint2_min_ + joint2_margin_ < joint2_max_ - joint2_margin_, "Invalid J2 limits/margin");
        require(std::isfinite(wheel_angular_speed_) && wheel_angular_speed_ > 0 &&
                std::isfinite(wheel_linear_speed_) && wheel_linear_speed_ > 0 &&
                std::isfinite(servo_period_) && servo_period_ >= 0.01 && servo_period_ <= 0.05 &&
                std::isfinite(servo_timeout_) && servo_timeout_ >= 2 * servo_period_,
                "Invalid wheel speed or servo period/timeout");
        checkElbows(home_);
        checkElbows(approach_seed_);

        require(datalink_ != nullptr, "Core requires a ZMQ device connection");

        // 运动学与碰撞检测统一在这里创建（与后端选择无关）。
        // 已注入时跳过，便于测试替换。
        if (!kinematics_) {
            const std::string urdf = resolvePath(config, "urdf", config_dir);
            const double ik_lo = joint2_min_ + joint2_margin_;
            const double ik_hi = joint2_max_ - joint2_margin_;
            kinematics_ = makePinIkKinematics(urdf, ik_lo, ik_hi);
        }
        if (collision_check_enabled_ && !collision_checker_) {
            collision_checker_ = makePinocchioCollisionChecker(resolvePath(config, "urdf", config_dir));
        }

        last_target_ = measured();
        setState("INITIALIZED");
        servo_thread_ = std::thread([this] { servoLoop(); });
    }

    void enable() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "INITIALIZED" || GetState() == "DISABLED", "Invalid state for enable");

        cancel_ = false;
        backend_stop_requested_ = false;
        checkElbows(measured());

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

        // 后端首帧已对齐实测值，此后只沿指令轨迹衔接，不重新赋值为反馈位置。
        default_impedance_confirmed_ = true;
        last_target_ = datalink_->jointTargets();
        last_velocity_ = {};
        last_angle_velocity_ = last_displacement_velocity_ = 0;

        setState(grasp().locked ? "LOCKED" : "ENABLED");
    }

    void disable() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        if (grasp().locked || GetState() == "APPROACHED") command(GraspCommand::Unlock);

        stopDrives();

        setState("DISABLED");
    }

    void homeMotion(bool verify_position) {
        moveJoints(home_, home_speed_);
        const double deadline = datalink_->time() + 5.0;
        double stable_since = -1;
        double longest_stable = 0;
        Joints actual{}, velocity{};
        bool stopped = false;
        while (datalink_->time() < deadline) {
            datalink_->waitTick();
            require(!cancel_, "Motion stopped");
            requireBackend(!grasp().fault, "Backend grasp fault while homing");
            require(datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right),
                    "A drive is no longer enabled");
            actual = measured();
            checkElbows(actual);
            bool settled = true;
            for (int i = 0; i < 14; ++i) {
                require(std::isfinite(actual[i]), "Nonfinite joint feedback while homing");
                velocity[i] = datalink_->getJointVelocity(static_cast<Side>(i / 7), i % 7);
                require(std::isfinite(velocity[i]), "Nonfinite joint velocity while homing");
                settled = settled && std::abs(velocity[i]) < 0.02 &&
                    (!verify_position || std::abs(actual[i] - home_[i]) <= home_position_tolerance_);
            }
            const double now = datalink_->time();
            if (!settled) stable_since = -1;
            else if (stable_since < 0) stable_since = now;
            if (stable_since >= 0) longest_stable = std::max(longest_stable, now - stable_since);
            if (stable_since >= 0 && now - stable_since >= 0.15) {
                stopped = true;
                break;
            }
        }
        if (!stopped) {
            std::ostringstream detail;
            detail << std::fixed << std::setprecision(6)
                   << "Home settling timeout after 5 s: position_check=" << verify_position
                   << " position_tolerance_rad=" << home_position_tolerance_
                   << " speed_limit_rad_s=0.020000 required_stable_ms=150 longest_stable_ms="
                   << longest_stable * 1000 << "; final joint feedback:";
            // Include every joint: the final sample can pass even though the stable window did not.
            for (int i = 0; i < 14; ++i) {
                const double error = actual[i] - home_[i];
                detail << '\n' << (i < 7 ? "left" : "right") << ".J" << i % 7 + 1
                       << " target_rad=" << home_[i] << " actual_rad=" << actual[i]
                       << " error_rad=" << error << " velocity_rad_s=" << velocity[i]
                       << " position_ok=" << (!verify_position || std::abs(error) <= home_position_tolerance_)
                       << " speed_ok=" << (std::abs(velocity[i]) < 0.02);
            }
            requireBackend(false, detail.str().c_str());
        }

    }

    void moveHome() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "ENABLED" && !grasp().locked &&
                datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right),
                "Home requires enabled, unlocked arms");
        cancel_ = false;
        setState("HOMING");
        try {
            homeMotion(true);
            setState("ENABLED");
        } catch (const std::exception& e) {
            setMotionError(std::string("MoveHome: ") + e.what());
            stopSafely(); setState("FAULT"); throw;
        } catch (...) { stopSafely(); setState("FAULT"); throw; }
    }

    void approachHandles(bool from_home = false) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        auto f = grasp();
        requireBackend(!f.fault, "Backend grasp fault before ApproachHandles");
        require(f.locked == 0 && !f.fault && datalink_->isEnabled(Side::Left) &&
                    datalink_->isEnabled(Side::Right),
                "Enable both arms and unlock before approach");

        cancel_ = false;
        setState("APPROACHING");

        const char *phase = "open hands";
        try {
            command(GraspCommand::Unlock); // Also reopen after an unlatched/aborted approach.
            phase = "home trajectory";
            if (!from_home) homeMotion(false); // Preserve the Direct demo home stage.

            // One synchronized joint trajectory from home to the final grasp pose.
            phase = "synchronized approach";
            const auto wheel0 = grasp();
            const auto goal = solve({target(0, wheel0.angle, wheel0.displacement),
                                     target(1, wheel0.angle, wheel0.displacement)}, approach_seed_);
            moveJoints(goal, approach_speed_, true);
            phase = "approach complete";

            // 同步轮盘位形
            auto w = grasp();
            wheel_angle_ = w.angle;
            wheel_displacement_ = w.displacement;

            setState("APPROACHED");
        } catch (const std::exception &e) {
            const std::string error = std::string("ApproachHandles [") + phase + "]: " + e.what();
            setMotionError(error);
            stopSafely();
            setState("FAULT");
            throw std::runtime_error(error);
        } catch (...) {
            stopSafely();
            setState("FAULT");
            throw;
        }
    }

    void prepareGrasp() {
        if (GetState() == "INITIALIZED" || GetState() == "DISABLED") {
            enable(); // RemoteLink opens and confirms hands before enabling.
        } else {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            require(lock.owns_lock() && !servo_active_ && GetState() == "ENABLED",
                    "Grasp preparation requires a stable enabled executor");
            command(GraspCommand::Unlock); // Re-grasp after release: confirm actual hands open too.
        }
    }

    void lockHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "APPROACHED" || GetState() == "LOCKED", "Approach before locking");
        command(GraspCommand::Lock); // RemoteLink also waits for physical hand command acceptance when configured.
        setState("LOCKED");
    }

    // Plan once, stretch time to respect wheel AND joint velocity limits, then execute.
    void moveWheel(double angle, double displacement, double v) {
        validateWheelInput(angle, displacement, v);
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "LOCKED", "Lock handles before MoveWheel; clear faults first");
        cancel_ = false;
        setMotionError("");
        setState("MOVING");
        try {
            const double a0 = wheel_angle_, d0 = wheel_displacement_;
            // Sample by geometry at full speed; reducing v must not multiply memory/IK work.
            const double nominal = std::max({0.5, 1.875 * std::abs(angle - a0) / wheel_angular_speed_,
                1.875 * std::abs(displacement - d0) / wheel_linear_speed_});
            const size_t steps = static_cast<size_t>(std::ceil(nominal / planning_period_));
            Path path{{planningStart(), a0, d0}};
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
            stopSafely();
            setState("FAULT");
            throw;
        }
    }

    // Only validate/publish a target here. IK, collision checks and execution run in servoLoop.
    void servoWheel(double angle, double displacement, double v) {
        validateWheelInput(angle, displacement, v);
        std::lock_guard<std::mutex> mailbox(servo_mutex_);
        if (!servo_active_) {
            std::unique_lock<std::mutex> control(mutex_, std::try_to_lock);
            require(control.owns_lock(), "Another control operation is running");
            require(GetState() == "LOCKED", "Lock handles before ServoWheel; clear faults first");
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

    void unlockHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "LOCKED" || GetState() == "APPROACHED" || GetState() == "FAULT",
                "No completed approach or software lock to release");

        command(GraspCommand::Unlock);
        setState("ENABLED");
    }

    void releaseHandles() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        require(GetState() == "LOCKED" || GetState() == "ENABLED", "Release requires a stable enabled executor");
        requireBackend(!grasp().fault, "Backend fault before release");
        cancel_ = false;
        setMotionError("");
        setState("RELEASING");
        const char* phase = "planning start";
        try {
            auto seed = planningStart();
            auto wheel = grasp();
            phase = "forward kinematics";
            std::array<pinocchio::SE3, 2> from;
            for (int side = 0; side < 2; ++side) {
                std::array<double, 7> q{};
                std::copy_n(seed.begin() + side * 7, 7, q.begin());
                require(kinematics_->solveFk(static_cast<Side>(side), q, from[side]), "Release FK failed");
            }
            Path path;
            phase = "retreat inverse kinematics";
            const auto steps = static_cast<size_t>(std::ceil(final_approach_duration_ / planning_period_));
            for (size_t k = 0; k <= steps; ++k) {
                require(!cancel_, "Release cancelled");
                std::array<pinocchio::SE3, 2> targets;
                for (int side = 0; side < 2; ++side)
                    targets[side] = interpolatePose(from[side], target(side, wheel.angle, wheel.displacement,
                                                                            approach_distance_), smooth(double(k) / steps));
                if (k) seed = solve(targets, seed);
                path.push_back({seed, wheel.angle, wheel.displacement});
            }
            phase = "retreat validation";
            validate(path, final_approach_duration_);
            require(!cancel_, "Release cancelled");
            phase = "open hands";
            command(GraspCommand::Unlock);
            phase = "retreat execution";
            execute(path, final_approach_duration_, false);
            setState("ENABLED");
        } catch (const std::exception& e) {
            const auto error = std::string("ReleaseHandles [") + phase + "]: " + e.what();
            // Publish the cause before FAULT becomes observable. Otherwise the
            // owner can enter SAFE and discard this worker's failed generation.
            setMotionError(error);
            stopSafely(); setState("FAULT");
            throw std::runtime_error(error);
        } catch (...) {
            setMotionError(std::string("ReleaseHandles [") + phase + "]: unknown exception");
            stopSafely(); setState("FAULT"); throw;
        }
    }

    void resetFault() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Another control operation is running");
        command(GraspCommand::ResetFault);
        backend_stop_requested_ = false;
        cancel_ = false;
        setMotionError("");
        const bool enabled = datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right);
        setState(enabled ? (grasp().locked == 3 ? "LOCKED" : "ENABLED") : "DISABLED");
    }

    void acknowledgeFault() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Executor has not stopped");
        const auto feedback = grasp();
        require(!feedback.fault, "Device fault is still present");
        setMotionError("");
        if (GetState() == "FAULT") {
            const bool enabled = datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right);
            setState(enabled ? (feedback.locked == 3 ? "LOCKED" : "ENABLED") : "DISABLED");
        }
        // The Core owns subsequent motion authorization; do not restart a trajectory here.
    }

    void setImpedanceProfile(bool following, const std::atomic<bool>& cancelled) {
        if (!following && default_impedance_confirmed_) return;
        // EXIT_CONTROL already requested the normal Servo deceleration. Wait for it
        // without blocking the Managed owner, which continues protection/heartbeats.
        while (GetState() == "SERVO") {
            require(!cancelled, "Impedance task cancelled while waiting for Servo stop");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        require(lock.owns_lock() && !servo_active_, "Executor busy during impedance update");
        require(!cancelled, "Impedance task cancelled");
        const auto previous = GetState();
        require(previous == "LOCKED" || previous == "ENABLED", "Impedance update requires stable enabled executor");
        setState("STIFFNESS");
        try {
            // A cancelled request may have reached the device: require a confirmed
            // default restoration before the next home/release even if this call fails.
            default_impedance_confirmed_ = false;
            std::ostringstream target_log;
            target_log << std::setprecision(17) << "Core impedance target profile=" << (following ? "following" : "default")
                       << " target_rad=[";
            const auto held = datalink_->jointTargets();
            for (size_t j = 0; j < held.size(); ++j) target_log << (j ? "," : "") << held[j];
            target_log << ']';
            Logger::info("{}", target_log.str());
            datalink_->setImpedanceProfile(following);
            if (cancelled) throw MotionCancelled();
            default_impedance_confirmed_ = !following;
            setState(previous);
        } catch (const MotionCancelled&) {
            setState(previous); // The Managed protection path owns the stop/revoke.
            throw;
        } catch (const std::exception& e) {
            setMotionError(e.what());
            setState("FAULT");
            throw;
        }
    }

    void stop() noexcept {
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
        status.angle = gs.angle;
        status.displacement = gs.displacement;
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
    void setState(const std::string &value) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (state_ != value) aviator::Logger::info("Core executor from={} to={}", state_, value);
        state_ = value;
    }

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

    void setMotionError(const std::string &error) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (!error.empty() && motion_error_ != error)
            aviator::Logger::error("Core executor state={} motion failed: {}", state_, error);
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

    void runServo() {
        JointFrame last{last_target_, wheel_angle_, wheel_displacement_};
        ServoPlanner planner(*kinematics_,
            [this](int side, double a, double d) { return target(side, a, d); },
            [this](const JointFrame &f) { check(f.q, f.angle, f.displacement); },
            wheel_origin_, last, servo_period_,
            {wheel_angular_speed_, wheel_linear_speed_}, wheel_acceleration_, wheel_jerk_);
        // Immutable future samples; the executor advances one shared dual-arm tick.
        // Refill below the configured lead; each append contributes one whole block.
        std::vector<JointFrame> prefill{last};
        bool started = false, stopping = false, timed_out = false;
        for (;;) {
            if (!stopping && (cancel_ || shutdown_ || servoExpired())) {
                stopping = true;
                timed_out = !cancel_ && !shutdown_;
            }
            if (started && datalink_->streamAhead() >= servo_buffer_.lookahead_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            ServoTarget command;
            { std::lock_guard<std::mutex> lock(servo_mutex_); command = servo_target_; }
            auto frames = planner.advance(command.angle, command.displacement, command.v, stopping);
            last = frames.back();
            if (started) datalink_->appendStream(frames);
            else {
                prefill.insert(prefill.end(), frames.begin() + 1, frames.end());
                if (prefill.size() >= servo_buffer_.effective_prefill_ms + 1) {
                    datalink_->beginStream(prefill);
                    started = true;
                }
            }
            if (stopping && planner.stopped() && started) {
                datalink_->finishStream();
                last_target_ = last.q;
                last_velocity_ = {};
                wheel_angle_ = last.angle;
                wheel_displacement_ = last.displacement;
                if (timed_out) setMotionError(ServoTimeout().what());
                return;
            }
        }
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
                runServo();
            } catch (const MotionCancelled& error) {
                setMotionError(error.what());
                if (stopSafely()) setMotionError("");
                else next_state = "FAULT";
            } catch (const std::exception &error) {
                setMotionError(error.what());
                // Stop and stale commands hold in the current impedance mode. Other failures latch FAULT.
                next_state = "FAULT";
                stopSafely();
            }
            std::lock_guard<std::mutex> mailbox(servo_mutex_);
            std::lock_guard<std::mutex> status(status_mutex_);
            servo_active_ = false;
            if (state_ != next_state)
                aviator::Logger::info("Core Servo from={} to={} reason={}", state_, next_state, motion_error_);
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
        const bool fresh = std::isfinite(state.heartbeat) && datalink_->time() - state.heartbeat < 0.5;
        if (!fresh || state.fault) backend_stop_requested_ = true;
        requireBackend(fresh, "Backend feedback timeout (>500 ms)");
        return state;
    }

    void requireBackend(bool ok, const char *message) {
        if (ok) return;
        std::string details;
        try { if (datalink_) details = datalink_->diagnostics(); }
        catch (const std::exception &e) { details = std::string("Diagnostics unavailable: ") + e.what(); }
        fail(std::string(message) + (details.empty() ? "" : "\n" + details));
    }

    Joints measured() {
        Joints q{};
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis)
                q[side * 7 + axis] = datalink_->getJointPosition(static_cast<Side>(side), axis);
        return q;
    }

    void checkMotionState() {
        try {
            require(!backend_stop_requested_, "Backend stop required after an earlier fault");
            requireBackend(!grasp().fault, "Backend grasp fault");
            require(datalink_->isEnabled(Side::Left) && datalink_->isEnabled(Side::Right),
                    "A drive is no longer enabled");
            const auto actual = measured();
            checkElbows(actual);
            for (int i = 0; i < 14; ++i)
                if (!std::isfinite(actual[i])) fail("Nonfinite joint feedback on axis " + std::to_string(i));
        } catch (...) {
            backend_stop_requested_ = true;
            throw;
        }
    }

    Joints planningStart() {
        checkMotionState();
        return last_target_;
    }

    void stopDrives() {
        std::exception_ptr error;
        for (Side side : {Side::Left, Side::Right}) {
            try { datalink_->disable(side); }
            catch (...) { if (!error) error = std::current_exception(); }
        }
        if (error) std::rethrow_exception(error);
    }

    // 按共同的速度比例制动双臂，保持最终指令；绝不跳回实测位置。
    // 反馈故障或制动路径不安全时，停止两臂 SDK 控制，不继续生成位置指令。
    void hold() {
        datalink_->stopTrajectory();
        last_target_ = datalink_->jointTargets();
        last_velocity_ = {};
        last_angle_velocity_ = last_displacement_velocity_ = 0;
    }

    bool stopSafely() {
        try { hold(); return true; }
        catch (const std::exception &e) {
            std::lock_guard<std::mutex> lock(status_mutex_);
            motion_error_ += std::string("; smooth stop unavailable: ") + e.what();
            return false;
        }
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

    pinocchio::SE3 wheel(double angle, double translation) const {
        return wheel_origin_ *
               pinocchio::SE3(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                              Eigen::Vector3d(0, 0, translation));
    }

    pinocchio::SE3 target(int side, double angle, double translation, double retreat = 0) const {
        auto flange = wheel(angle, translation) * handles_[side] * tools_[side].inverse();
        flange.translation() -= flange.rotation() * Eigen::Vector3d(0, 0, retreat);
        return flange;
    }

    Joints solve(const std::array<pinocchio::SE3, 2> &targets, const Joints &seed) {
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
        if (collision_check_enabled_)
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

    // 两臂共用一条 Ruckig 轨迹。先完整规划/检查，运行时只按 1 ms 取样。
    void moveJoints(const Joints &goal, double speed, bool close_hands = false) {
        ruckig::Ruckig<14> planner{command_period};
        ruckig::InputParameter<14> input;
        input.current_position = planningStart();
        input.current_velocity = last_velocity_;
        input.current_acceleration.fill(0); // 此接口只在使能保持或上一段停稳后调用。
        input.target_position = goal;
        input.target_velocity.fill(0);
        input.target_acceleration.fill(0);
        input.max_acceleration.fill(joint_acceleration_);
        input.max_jerk.fill(joint_jerk_);
        input.synchronization = ruckig::Synchronization::Time;
        input.duration_discretization = ruckig::DurationDiscretization::Discrete;
        for (int i = 0; i < 14; ++i)
            input.max_velocity[i] = std::min({speed, joint_speed_,
                datalink_->jointVelLimit(static_cast<Side>(i / 7), i % 7)});

        ruckig::Trajectory<14> trajectory;
        const auto result = planner.calculate(input, trajectory);
        require(result >= 0, "Ruckig joint planning failed: " + std::to_string(int(result)));
        const double duration = trajectory.get_duration();
        require(std::isfinite(duration) && duration >= 0, "Invalid Ruckig duration");
        const auto w = grasp();
        // 沿实际曲线检查，包括原规划间隔的中点，不能用端点间直线替代。
        const size_t checks = std::max<size_t>(1, std::ceil(duration / (planning_period_ * 0.5)));
        Joints q{}, velocity{}, acceleration{};
        for (size_t k = 0; k <= checks; ++k) {
            require(!cancel_, "Motion cancelled while planning");
            trajectory.at_time(duration * double(k) / checks, q, velocity, acceleration);
            check(q, w.angle, w.displacement);
        }
        const size_t ticks = static_cast<size_t>(std::llround(duration / command_period));
        require(ticks <= 3600000, "Trajectory exceeds one hour budget");
        std::vector<JointFrame> frames;
        frames.reserve(ticks + 1);
        for (size_t k = 0; k <= ticks; ++k) {
            trajectory.at_time(std::min(k * command_period, duration), q, velocity, acceleration);
            frames.push_back({q, w.angle, w.displacement});
        }
        if (close_hands) {
            const auto wheel_pose = wheel(w.angle, w.displacement);
            planHandClosure(frames, hand_closing_distance_, [&](int side, const JointFrame& frame) {
                require(!cancel_, "Motion cancelled while planning hand closure");
                std::array<double, 7> joints{};
                std::copy_n(frame.q.begin() + side * 7, 7, joints.begin());
                pinocchio::SE3 flange;
                require(kinematics_->solveFk(static_cast<Side>(side), joints, flange), "FK failed");
                return ((flange * tools_[side]).translation() -
                        (wheel_pose * handles_[side]).translation()).norm();
            });
        }
        datalink_->runTrajectory(frames, cancel_);
        last_target_ = frames.back().q;
        last_velocity_ = {};
    }

    void execute(const Path &path, double duration, bool) {
        for (int i = 0; i < 14; ++i)
            require(std::abs(path.front().q[i] - last_target_[i]) < 1e-10,
                    "Trajectory start differs from last command");
        const size_t ticks = std::max<size_t>(1, std::ceil(duration / command_period));
        require(ticks <= 3600000, "Trajectory exceeds one hour budget");
        std::vector<JointFrame> frames;
        frames.reserve(ticks + 1);
        for (size_t tick = 0; tick <= ticks; ++tick) {
            double u = double(tick) / ticks;
            const double coordinate = u * (path.size() - 1);
            const size_t k = std::min(size_t(coordinate), path.size() - 2);
            const double f = coordinate - k;
            JointFrame frame;
            for (int i = 0; i < 14; ++i) frame.q[i] = path[k].q[i] + f * (path[k + 1].q[i] - path[k].q[i]);
            frame.angle = path[k].angle + f * (path[k + 1].angle - path[k].angle);
            frame.displacement = path[k].translation + f * (path[k + 1].translation - path[k].translation);
            frames.push_back(frame);
        }
        datalink_->runTrajectory(frames, cancel_);
        last_target_ = frames.back().q;
        wheel_angle_ = frames.back().angle;
        wheel_displacement_ = frames.back().displacement;
        last_velocity_ = {};
        last_angle_velocity_ = last_displacement_velocity_ = 0;
    }

    void dwell(double seconds) {
        double until = datalink_->time() + seconds;
        while (datalink_->time() < until) {
            datalink_->waitTick();
            require(!cancel_, "Motion stopped");
            auto g = grasp();
            requireBackend(!g.fault, "Grasp fault while settling");
            checkElbows(measured());
        }
    }

    std::unique_ptr<DataLink> datalink_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    bool default_impedance_confirmed_ = true; // Accessed only by the serialized executor worker.
    bool collision_check_enabled_ = true;

    std::string config_file_;
    mutable std::mutex status_mutex_;
    double home_position_tolerance_ = 0.02;
    std::string state_ = "UNINITIALIZED";

    double home_speed_, approach_speed_, joint_acceleration_, joint_jerk_;
    double final_approach_duration_;
    double planning_period_;
    double joint_speed_;
    double stop_acceleration_;
    double settle_duration_;
    double wheel_angular_speed_, wheel_linear_speed_;
    double servo_period_, servo_timeout_;
    ServoBufferConfig servo_buffer_;
    std::array<double, 2> wheel_acceleration_, wheel_jerk_;
    double approach_distance_, hand_closing_distance_;

    double joint2_min_, joint2_max_, joint2_margin_;

    pinocchio::SE3 handles_[2]{pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
    std::array<pinocchio::SE3, 2> tools_{{pinocchio::SE3::Identity(), pinocchio::SE3::Identity()}};
    pinocchio::SE3 wheel_origin_ = pinocchio::SE3::Identity();
    Joints home_{}, approach_seed_{};
    Joints last_target_{};
    Joints last_velocity_{};
    double last_angle_velocity_ = 0, last_displacement_velocity_ = 0;

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
    std::atomic<bool> backend_stop_requested_{false};
    mutable std::mutex mutex_;
};

namespace {
uint64_t managedNow() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

// The FSM is owned by Aviator, not by the console or a worker thread.
class Aviator::Managed {
public:
    Managed(Impl& executor, ManagedOptions options)
        : executor_(executor), options_(std::move(options)), owner_(std::this_thread::get_id()) {
        require(options_.snapshot && options_.allow_motion && options_.heartbeat && options_.request_brake,
                "Managed Aviator requires live evidence, motion gate, heartbeat and brake adapters");
        options_.allow_motion(false);
    }
    ~Managed() {
        cancelled_ = true;
        executor_.stop();
        try { options_.allow_motion(false); } catch (...) {}
        while (worker_.valid() && worker_.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
            try { options_.heartbeat(); } catch (...) {}
        }
        if (worker_.valid()) { try { worker_.get(); } catch (...) {} }
    }
    void owner() const {
        require(owner_ == std::this_thread::get_id(), "Managed Aviator API must run on its owner thread");
    }
    fsm::Snapshot snapshot() const {
        auto s = options_.snapshot();
        s.executor_idle = !worker_.valid();
        const auto phase = executor_.GetState();
        s.settled = s.settled && s.executor_idle && phase != "SERVO" && phase != "MOVING" &&
                    phase != "APPROACHING" && phase != "RELEASING" && phase != "HOMING";
        if (phase == "FAULT") {
            s.ready = false;
            // Maintenance may have removed the device fault; ResetError acknowledges the local fault.
            if (std::string(machine_.state()) != "ERROR" && std::string(machine_.state()) != "SAFE") {
                s.fault = executor_.GetStatus().motion_error;
                if (s.fault.empty()) s.fault = "Executor entered FAULT without a diagnostic";
            }
        }
        if (machine_.acceptsControl() && !restoring_default_) {
            const auto now = managedNow();
            const auto last = input_at_ ? input_at_ : std::max(machine_.controlSince(), control_ready_at_);
            s.input_ready = s.input_ready && now >= last && now - last < 100000;
        }
        return s;
    }
    SystemStatus status() const {
        owner();
        auto conditions = snapshot();
        // A transition is published before its worker starts. Pending tasks must
        // never briefly advertise a settled FOLLOWING state to service clients.
        conditions.executor_idle = conditions.executor_idle && machine_.job() == fsm::Job::none;
        conditions.settled = conditions.settled && conditions.executor_idle;
        return {machine_.state(), machine_.currentError(), machine_.lastError(), machine_.stateCode(),
                machine_.generation(), machine_.acceptsControl() && !restoring_default_, brake_requested_, conditions};
    }
    void init() {
        owner();
        require(!booted_, "Managed Init may only be called once; use EnterStandby to recover");
        booted_ = true;
        options_.heartbeat();
        machine_.boot(snapshot(), managedNow());
        effects();
    }
    void update() {
        owner();
        require(booted_, "Call managed Init first");
        options_.heartbeat();
        machine_.supervise(snapshot(), managedNow()); // Fault/emergency precedes completion.
        if (worker_.valid() && worker_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            if (restoring_default_) {
                try {
                    worker_.get();
                    if (restore_generation_ == machine_.generation() && machine_.acceptsControl()) {
                        control_ready_at_ = managedNow();
                        input_at_ = 0;
                    }
                } catch (const std::exception& e) {
                    if (restore_generation_ == machine_.generation()) machine_.fault(e.what());
                } catch (...) {
                    if (restore_generation_ == machine_.generation()) machine_.fault("Default impedance restoration failed");
                }
                restoring_default_ = false;
            } else {
                try { worker_.get(); completion_ = active_; }
                catch (const std::exception& e) { machine_.failed(active_->generation, e.what()); }
                catch (...) { machine_.failed(active_->generation, "Unknown executor failure"); }
                active_.reset();
            }
        }
        if (completion_) {
            if (completion_->generation != machine_.generation()) completion_.reset();
            else {
                auto s = snapshot();
                if (s.settled || completion_->job == fsm::Job::grasp) {
                    machine_.done(completion_->generation, s, managedNow());
                    completion_.reset();
                }
            }
        }
        effects();
    }
    fsm::Reply request(fsm::Operation operation) {
        update();
        const auto reply = machine_.request(operation, snapshot(), managedNow());
        if (reply == fsm::Reply::completed && operation == fsm::Operation::start_control) {
            // Keep START_CONTROL's original direct state transition. Restore the
            // profile on the existing executor slot; the owner keeps supervising
            // input, protection and services while targets are temporarily gated.
            input_at_ = 0;
            timing_input_at_ = 0;
            timing_inputs_ = 0;
            restoring_default_ = true;
            restore_generation_ = machine_.generation();
            cancelled_ = false;
            try {
                worker_ = std::async(std::launch::async, [this] {
                    executor_.setImpedanceProfile(false, cancelled_);
                });
            } catch (const std::exception& e) {
                restoring_default_ = false;
                machine_.fault(e.what());
            }
        }
        if (reply == fsm::Reply::completed && operation == fsm::Operation::reset_error) {
            try { executor_.acknowledgeFault(); }
            catch (const std::exception& e) { machine_.fault(e.what()); }
        }
        effects();
        if ((reply == fsm::Reply::accepted ||
             (reply == fsm::Reply::completed && (operation == fsm::Operation::reset_error ||
                                               operation == fsm::Operation::start_control))) &&
            std::string(machine_.state()) == "ERROR") return fsm::Reply::capability_unavailable;
        return reply;
    }
    bool servo(double angle, double displacement, double speed, uint64_t sample) {
        update();
        const auto now = managedNow();
        if (!machine_.acceptsControl() || restoring_default_ ||
            sample < std::max(machine_.controlSince(), control_ready_at_) || sample > now ||
            now - sample >= 100000 || (input_at_ && sample <= input_at_)) return false;
        try {
            executor_.servoWheel(angle, displacement, speed);
            input_at_ = sample; // Never refresh the lease with a replayed old target.
            ++timing_inputs_;
            if (!timing_input_at_) timing_input_at_ = now;
            if (now - timing_input_at_ >= 1000000) {
                const auto actual = executor_.GetStatus();
                Logger::info("Control timing input_hz={:.1f} input_age_ms={:.2f} "
                    "requested_angle_rad={:.5f} reference_angle_rad={:.5f} "
                    "requested_displacement_m={:.5f} reference_displacement_m={:.5f} speed_ratio={:.3f}",
                    timing_inputs_ * 1e6 / (now - timing_input_at_), (now - sample) / 1000.0,
                    angle, actual.angle, displacement, actual.displacement, speed);
                timing_inputs_ = 0;
                timing_input_at_ = now;
            }
            return true;
        } catch (const std::exception& e) {
            machine_.fault(e.what()); effects(); return false;
        }
    }
    void emergency(const std::string& reason) {
        owner(); machine_.emergency(reason); effects();
    }
private:
    void report() { if (options_.report) options_.report(status()); }
    void protection() {
        if (machine_.takeStop()) {
            cancelled_ = true;
            executor_.stop();
            input_at_ = 0;
            if (std::string(machine_.state()) != "FOLLOWING") options_.allow_motion(false);
        }
        if (machine_.takeBrake()) {
            brake_requested_ = true;
            try { options_.request_brake(); }
            catch (const std::exception& e) { machine_.fault(e.what()); }
        }
    }
    void effects() {
        protection();
        report(); // Publish the transition before submitting a blocking executor task.
        // EXIT_CONTROL can cancel restoration and queue Following while it drains.
        if (worker_.valid()) return;
        if (auto task = machine_.takeTask()) {
            machine_.supervise(snapshot(), managedNow());
            if (task->generation != machine_.generation()) { protection(); report(); return; }
            if (worker_.valid()) { machine_.failed(task->generation, "Executor slot occupied"); protection(); report(); return; }
            active_ = task;
            cancelled_ = false;
            try {
                options_.allow_motion(true);
                worker_ = std::async(std::launch::async, [this, job = task->job] {
                    const auto check = [&] { require(!cancelled_, "Managed task cancelled"); };
                    check();
                    if (job == fsm::Job::initialize) {
                        if (executor_.GetState() == "UNINITIALIZED") executor_.init();
                        check();
                        executor_.enable(); // Rokae prepare/start configures joint impedance and holds current pose.
                    } else if (job == fsm::Job::home) {
                        if (executor_.GetState() == "INITIALIZED" || executor_.GetState() == "DISABLED") {
                            executor_.enable(); check();
                        }
                        executor_.setImpedanceProfile(false, cancelled_); check();
                        executor_.moveHome();
                    } else if (job == fsm::Job::grasp) {
                        executor_.prepareGrasp(); check();
                        executor_.approachHandles(true); check();
                        executor_.lockHandles();
                    } else if (job == fsm::Job::following_impedance) {
                        executor_.setImpedanceProfile(true, cancelled_);
                    } else if (job == fsm::Job::release) {
                        executor_.setImpedanceProfile(false, cancelled_); check();
                        executor_.releaseHandles(); check();
                        executor_.moveHome(); // STANDBY always means home has been reached.
                    }
                });
            } catch (const std::exception& e) {
                active_.reset(); machine_.failed(task->generation, e.what()); protection(); report();
            }
        }
    }
    Impl& executor_;
    ManagedOptions options_;
    const std::thread::id owner_;
    fsm::RobotStateMachine machine_;
    std::future<void> worker_;
    std::optional<fsm::Task> active_, completion_;
    std::atomic<bool> cancelled_{false};
    bool booted_ = false, brake_requested_ = false;
    bool restoring_default_ = false;
    uint64_t restore_generation_ = 0, control_ready_at_ = 0;
    uint64_t input_at_ = 0;
    uint64_t timing_input_at_ = 0, timing_inputs_ = 0;
};

Aviator::Aviator(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
                 std::unique_ptr<CollisionChecker> collision_checker, const std::string& config_file,
                 std::optional<ManagedOptions> options)
    : impl_(std::make_unique<Impl>(std::move(datalink), std::move(kinematics),
                                   std::move(collision_checker), config_file)) {
    if (options) managed_ = std::make_unique<Managed>(*impl_, std::move(*options));
}
Aviator::~Aviator() = default;
void Aviator::requireDirect() const { require(!managed_, "Direct action is disabled on a managed Aviator"); }
Aviator::Managed& Aviator::managed() const {
    require(bool(managed_), "State-machine operation requires ManagedOptions");
    managed_->owner();
    return *managed_;
}
void Aviator::Init() { managed().init(); }
void Aviator::Update() { managed().update(); }
fsm::Reply Aviator::EnterStandby() { return managed().request(fsm::Operation::enter_standby); }
fsm::Reply Aviator::GraspWheel() { return managed().request(fsm::Operation::grasp_wheel); }
fsm::Reply Aviator::StartControl() { return managed().request(fsm::Operation::start_control); }
fsm::Reply Aviator::ExitControl() { return managed().request(fsm::Operation::exit_control); }
fsm::Reply Aviator::LeaveWheel() { return managed().request(fsm::Operation::leave_wheel); }
fsm::Reply Aviator::ResetError() { return managed().request(fsm::Operation::reset_error); }
bool Aviator::ServoWheel(double angle, double displacement, double speed, uint64_t sample) {
    return managed().servo(angle, displacement, speed, sample);
}
void Aviator::EmergencyStop(const std::string& reason) { managed().emergency(reason); }
SystemStatus Aviator::GetSystemStatus() const { return managed().status(); }
std::string Aviator::GetSystemState() const { return GetSystemStatus().state; }
void Aviator::init() { requireDirect(); impl_->init(); }
void Aviator::enable() { requireDirect(); impl_->enable(); }
void Aviator::disable() { requireDirect(); impl_->disable(); }
void Aviator::approachHandles() { requireDirect(); impl_->approachHandles(); }
void Aviator::lockHandles() { requireDirect(); impl_->lockHandles(); }
void Aviator::moveWheel(double a, double d, double v) { requireDirect(); impl_->moveWheel(a, d, v); }
void Aviator::servoWheel(double a, double d, double v) { requireDirect(); impl_->servoWheel(a, d, v); }
void Aviator::unlockHandles() { requireDirect(); impl_->unlockHandles(); }
void Aviator::releaseHandles() { requireDirect(); impl_->releaseHandles(); }
void Aviator::resetFault() { requireDirect(); impl_->resetFault(); }
void Aviator::acknowledgeFault() { requireDirect(); impl_->acknowledgeFault(); }
void Aviator::stop() { requireDirect(); impl_->stop(); }
Status Aviator::GetStatus() { return impl_->GetStatus(); }
std::string Aviator::GetState() const { return impl_->GetState(); }
} // namespace aviator

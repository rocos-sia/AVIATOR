#include "aviator/Aviator.hpp"
#include "aviator/Kinematics.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace aviator {

namespace {
void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::runtime_error(message);
}

void fail(const std::string &message) { throw std::runtime_error(message); }

double smooth(double t) {
    // 5阶多项式平滑函数: s(t) = 6t^5 - 15t^4 + 10t^3
    return t * t * t * (10.0 + t * (-15.0 + t * 6.0));
}

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

} // namespace

class Aviator::Impl {
  public:
    Impl(const std::string &config_path);
    ~Impl();

    void Enable();
    void Disable();
    void Initialize();
    void ApproachHandles();
    void LockHandles();
    void MoveWheel(double angle_rad, double displacement_m, double duration_s);
    void UnlockHandles();
    void ResetFault();
    void Stop() noexcept;
    Status GetStatus();
    std::string GetState() const;

  private:
    enum class State { Idle, Busy, Stopped };

    struct Waypoint {
        Joints q;
        double angle;
        double displacement;
    };

    void checkStopped();
    Joints measured();
    void hold();
    void checkElbows(const Joints &q);
    pinocchio::SE3 wheel(double angle, double displacement);
    pinocchio::SE3 target(int side, double angle, double displacement);
    Joints solve(const std::array<pinocchio::SE3, 2> &targets, const Joints &seed);
    void validate(const std::vector<Waypoint> &path, double duration);
    void execute(const std::vector<Waypoint> &path, double duration, bool wait);
    void dwell(double duration);
    Status graspState();

    // 配置
    YAML::Node config_;
    std::string urdf_path_;
    std::unique_ptr<Kinematics> kinematics_;

    // 把手和工具位姿
    std::array<pinocchio::SE3, 2> handles_;
    pinocchio::SE3 tool_;

    // J2 肘部约束
    double joint2_min_;
    double joint2_max_;

    // 规划参数
    double plan_dt_;
    double settle_duration_;

    // 状态
    mutable std::mutex state_mutex_;
    State state_;
    std::atomic<bool> cancel_;
    int phase_;
    double wheel_angle_;
    double wheel_displacement_;
};

Aviator::Impl::Impl(const std::string &config_path)
    : state_(State::Idle), cancel_(false), phase_(0), wheel_angle_(0.0), wheel_displacement_(0.0) {

    // 加载配置文件
    config_ = YAML::LoadFile(config_path);

    // 加载 URDF
    urdf_path_ = config_["urdf"].as<std::string>();
    std::cout << "[Aviator] Loading URDF: " << urdf_path_ << std::endl;

    // 加载把手和工具配置
    auto grasp_config = config_["grasp"];

    // 解析工具变换 (tool frame)
    auto tool_pos = grasp_config["tool"]["position"].as<std::vector<double>>();
    auto tool_quat = grasp_config["tool"]["quaternion"].as<std::vector<double>>();
    Eigen::Quaterniond tool_q(tool_quat[0], tool_quat[1], tool_quat[2], tool_quat[3]);
    Eigen::Vector3d tool_p(tool_pos[0], tool_pos[1], tool_pos[2]);
    tool_ = pinocchio::SE3(tool_q.toRotationMatrix(), tool_p);

    // 解析左右把手位姿
    for (int side = 0; side < 2; ++side) {
        std::string key = (side == 0) ? "left" : "right";
        auto handle_pos = grasp_config[key]["position"].as<std::vector<double>>();
        auto handle_quat = grasp_config[key]["quaternion"].as<std::vector<double>>();
        Eigen::Quaterniond handle_q(handle_quat[0], handle_quat[1], handle_quat[2], handle_quat[3]);
        Eigen::Vector3d handle_p(handle_pos[0], handle_pos[1], handle_pos[2]);
        handles_[side] = pinocchio::SE3(handle_q.toRotationMatrix(), handle_p);
    }

    // 加载 J2 约束
    auto posture_config = config_["posture"];
    joint2_min_ = posture_config["joint2_min"].as<double>() * kDegToRad;
    joint2_max_ = posture_config["joint2_max"].as<double>() * kDegToRad;

    // 规划参数
    plan_dt_ = config_["planning"]["dt"].as<double>(0.01);
    settle_duration_ = config_["planning"]["settle_duration"].as<double>(0.5);

    // 创建运动学求解器
    kinematics_ = makeTracIkKinematics(urdf_path_, joint2_min_, joint2_max_);

    std::cout << "[Aviator] Initialized successfully" << std::endl;
}

Aviator::Impl::~Impl() { Stop(); }

void Aviator::Impl::Enable() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cout << "[Aviator] Enable called" << std::endl;
}

void Aviator::Impl::Disable() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cout << "[Aviator] Disable called" << std::endl;
}

void Aviator::Impl::Initialize() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    require(state_ == State::Idle, "Initialize: system is busy");
    state_ = State::Busy;
    cancel_ = false;

    try {
        std::cout << "[Aviator] Initializing..." << std::endl;

        // TODO: 移动到 home 位置
        // 这里需要实现移动到初始位置的逻辑

        phase_ = 1;
        state_ = State::Idle;
        std::cout << "[Aviator] Initialization complete" << std::endl;
    } catch (...) {
        hold();
        phase_ = 7;
        state_ = State::Idle;
        throw;
    }
}

void Aviator::Impl::ApproachHandles() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    require(state_ == State::Idle, "ApproachHandles: system is busy");
    state_ = State::Busy;
    cancel_ = false;

    try {
        auto wheel0 = graspState();
        wheel_angle_ = wheel0.angle;
        wheel_displacement_ = wheel0.displacement;

        // 第一阶段：粗接近，从 home 到后撤位置
        double duration = config_["approach"]["coarse_duration"].as<double>(4.0);
        const double retreat = config_["geometry"]["approach_distance"].as<double>(0.06);

        // 计算后撤目标位姿
        std::array<pinocchio::SE3, 2> retreat_targets;
        for (int side = 0; side < 2; ++side) {
            auto tcp = target(side, wheel0.angle, wheel0.displacement);
            // 沿 TCP 的 Z 轴（安装杆方向）后撤
            Eigen::Vector3d retreat_offset = tcp.rotation().col(2) * retreat;
            retreat_targets[side] = pinocchio::SE3(tcp.rotation(), tcp.translation() - retreat_offset);
        }

        // 求解 IK
        Joints seed = measured();
        Joints goal = solve(retreat_targets, seed);

        // 生成轨迹
        std::vector<Waypoint> path;
        size_t steps = static_cast<size_t>(std::ceil(duration / plan_dt_));
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            Joints q;
            for (int i = 0; i < kTotalJoints; ++i)
                q[i] = seed[i] + s * (goal[i] - seed[i]);
            path.push_back({q, wheel0.angle, wheel0.displacement});
        }

        validate(path, duration);
        execute(path, duration, true);

        // 第二阶段：精确接近，到达把手位置
        duration = config_["approach"]["fine_duration"].as<double>(2.0);
        auto wheel1 = graspState();

        seed = measured();
        std::array<pinocchio::SE3, 2> handle_targets;
        for (int side = 0; side < 2; ++side)
            handle_targets[side] = target(side, wheel1.angle, wheel1.displacement);

        goal = solve(handle_targets, seed);

        path.clear();
        steps = static_cast<size_t>(std::ceil(duration / plan_dt_));
        for (size_t k = 0; k <= steps; ++k) {
            const double s = smooth(double(k) / steps);
            Joints q;
            for (int i = 0; i < kTotalJoints; ++i)
                q[i] = seed[i] + s * (goal[i] - seed[i]);
            path.push_back({q, wheel1.angle, wheel1.displacement});
        }

        validate(path, duration);
        execute(path, duration, false);
        dwell(settle_duration_);

        phase_ = 4;
        state_ = State::Idle;
    } catch (...) {
        hold();
        phase_ = 7;
        state_ = State::Idle;
        throw;
    }
}

void Aviator::Impl::LockHandles() {
    std::cout << "[Aviator] LockHandles called" << std::endl;
    phase_ = 5;
}

void Aviator::Impl::MoveWheel(double angle_rad, double displacement_m, double duration_s) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    require(state_ == State::Idle, "MoveWheel: system is busy");
    state_ = State::Busy;
    cancel_ = false;

    try {
        std::cout << "[Aviator] Moving wheel: angle=" << angle_rad << " rad, displacement="
                  << displacement_m << " m, duration=" << duration_s << " s" << std::endl;

        // TODO: 实现轮盘运动逻辑

        wheel_angle_ = angle_rad;
        wheel_displacement_ = displacement_m;

        state_ = State::Idle;
    } catch (...) {
        hold();
        state_ = State::Idle;
        throw;
    }
}

void Aviator::Impl::UnlockHandles() {
    std::cout << "[Aviator] UnlockHandles called" << std::endl;
    phase_ = 6;
}

void Aviator::Impl::ResetFault() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cout << "[Aviator] ResetFault called" << std::endl;
}

void Aviator::Impl::Stop() noexcept {
    cancel_ = true;
    std::cout << "[Aviator] Stop requested" << std::endl;
}

Status Aviator::Impl::GetStatus() {
    Status status;
    status.angle = wheel_angle_;
    status.displacement = wheel_displacement_;
    status.locked = (phase_ >= 5);
    status.ready = (phase_ >= 4);
    status.fault = (phase_ == 7);
    return status;
}

std::string Aviator::Impl::GetState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    switch (state_) {
    case State::Idle:
        return "idle";
    case State::Busy:
        return "busy";
    case State::Stopped:
        return "stopped";
    default:
        return "unknown";
    }
}

void Aviator::Impl::checkStopped() {
    if (cancel_)
        throw std::runtime_error("Motion cancelled");
}

Joints Aviator::Impl::measured() {
    // TODO: 从真实硬件或仿真读取当前关节角
    // 暂时返回 home 位置
    Joints q{};
    auto posture_config = config_["posture"];
    auto home = posture_config["home"].as<std::vector<double>>();
    for (int i = 0; i < kTotalJoints && i < static_cast<int>(home.size()); ++i)
        q[i] = home[i] * kDegToRad;
    return q;
}

void Aviator::Impl::hold() {
    std::cout << "[Aviator] Holding position" << std::endl;
    // TODO: 实现保持当前位置的逻辑
}

void Aviator::Impl::checkElbows(const Joints &q) {
    // 检查 J2 (肘部) 是否在允许范围内
    for (int side = 0; side < 2; ++side) {
        double j2 = q[side * 7 + 1];
        require(j2 >= joint2_min_ && j2 <= joint2_max_,
                "Elbow joint J2 out of range [" + std::to_string(joint2_min_ * kRadToDeg) + ", " +
                    std::to_string(joint2_max_ * kRadToDeg) + "] deg");
    }
}

pinocchio::SE3 Aviator::Impl::wheel(double angle, double displacement) {
    // 轮盘变换: rotation around X-axis + translation along X-axis
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitX());
    Eigen::Vector3d translation(displacement, 0, 0);
    return pinocchio::SE3(rotation.toRotationMatrix(), translation);
}

pinocchio::SE3 Aviator::Impl::target(int side, double angle, double displacement) {
    // TCP 目标位姿 = wheel * handle * tool^-1
    return wheel(angle, displacement) * handles_[side] * tool_.inverse();
}

Joints Aviator::Impl::solve(const std::array<pinocchio::SE3, 2> &targets, const Joints &seed) {
    Joints result = seed;
    for (int side = 0; side < 2; ++side) {
        std::array<double, 7> arm_seed{}, arm_result{};
        for (int j = 0; j < 7; ++j)
            arm_seed[j] = seed[side * 7 + j];
        if (!kinematics_->solveIk(static_cast<Side>(side), arm_seed, targets[side], arm_result))
            fail(std::string(side == 0 ? "Left" : "Right") + " arm IK failed");
        for (int j = 0; j < 7; ++j)
            result[side * 7 + j] = arm_result[j];
    }
    checkElbows(result);
    return result;
}

void Aviator::Impl::validate(const std::vector<Waypoint> &path, double duration) {
    require(!path.empty(), "Empty path");
    const double dt = duration / (path.size() - 1);
    for (size_t k = 0; k < path.size(); ++k) {
        checkStopped();
        checkElbows(path[k].q);
        // TODO: 检查关节限位、速度限制等
    }
}

void Aviator::Impl::execute(const std::vector<Waypoint> &path, double duration, bool wait) {
    std::cout << "[Aviator] Executing path with " << path.size() << " waypoints over " << duration
              << " seconds" << std::endl;
    // TODO: 发送轨迹到硬件或仿真
    if (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
    }
}

void Aviator::Impl::dwell(double duration) {
    std::cout << "[Aviator] Dwelling for " << duration << " seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
}

Status Aviator::Impl::graspState() {
    // TODO: 从传感器读取当前把手状态
    // 暂时返回零位
    Status status;
    status.angle = wheel_angle_;
    status.displacement = wheel_displacement_;
    status.locked = false;
    status.ready = false;
    status.fault = false;
    return status;
}

// Aviator 公共接口实现
Aviator::Aviator(const std::string &config_path) : impl_(std::make_unique<Impl>(config_path)) {}

Aviator::~Aviator() = default;

void Aviator::Enable() { impl_->Enable(); }
void Aviator::Disable() { impl_->Disable(); }
void Aviator::Initialize() { impl_->Initialize(); }
void Aviator::ApproachHandles() { impl_->ApproachHandles(); }
void Aviator::LockHandles() { impl_->LockHandles(); }
void Aviator::MoveWheel(double angle_rad, double displacement_m, double duration_s) {
    impl_->MoveWheel(angle_rad, displacement_m, duration_s);
}
void Aviator::UnlockHandles() { impl_->UnlockHandles(); }
void Aviator::ResetFault() { impl_->ResetFault(); }
void Aviator::Stop() noexcept { impl_->Stop(); }
Status Aviator::GetStatus() { return impl_->GetStatus(); }
std::string Aviator::GetState() const { return impl_->GetState(); }

} // namespace aviator

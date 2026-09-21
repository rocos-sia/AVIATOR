#include "aviator/backend.hpp"

#include "rokae_sdk.hpp"
#include "OpenLoopGrasp.hpp"
#include <atomic>
#include <cmath>
#include <limits>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <time.h>
#include <vector>

namespace aviator {

namespace {
constexpr std::chrono::microseconds kTick{1000};

double monotonic() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}

// URDF 四元数 [x,y,z,w] → KDL::Rotation
KDL::Rotation rotationFromUrdf(const urdf::Rotation &r) {
    return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}
KDL::Vector vectorFromUrdf(const urdf::Vector3 &v) { return KDL::Vector(v.x, v.y, v.z); }

// 在 URDF 里沿固定关节链求 link 在 root 系下的位姿。
// 只用于推算臂基座 → 轮盘的安装变换（中间全是 fixed 关节或已知转角为 0）。
// 这里只支持"纯固定关节"链，遇到可动关节就报错，避免静默算错。
KDL::Frame fixedChainToRoot(const urdf::ModelInterfaceSharedPtr &model,
                            const std::string &link, const std::string &root) {
    std::vector<const urdf::Joint *> chain;
    std::string current = link;
    while (current != root) {
        const auto child_link = model->getLink(current);
        require(child_link != nullptr, "URDF 缺少 link: " + current);
        const auto joint = child_link->parent_joint;
        require(joint != nullptr, "link " + current + " 没有父关节，无法连到 " + root);
        require(joint->type == urdf::Joint::FIXED,
                "安装链上出现非固定关节 " + joint->name + "，无法用固定变换推算");
        chain.push_back(joint.get());
        current = joint->parent_link_name;
        require(!current.empty(), "父链在到达 " + root + " 前断开");
    }

    KDL::Frame result; // 单位阵
    // 从 root 往下累乘
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const auto *joint = *it;
        const KDL::Frame local(rotationFromUrdf(joint->parent_to_joint_origin_transform.rotation),
                               vectorFromUrdf(joint->parent_to_joint_origin_transform.position));
        result = result * local;
    }
    return result;
}

} // namespace

// 珞石 xMateErPro 真机后端（双臂）。
//
// 与仿真后端的差异（同一条 DataLink 接口，语义对齐）：
//  * 时间源：waitTick 按 1 ms 墙钟节拍，time() 返回墙钟 CLOCK_MONOTONIC。
//    仿真返回 mjData::time（可加速），真机不可能快于真实时间。
//  * 关节伺服：命令下发交给 SDK 的周期调度（RT 线程按 1 ms 回调 JointPosition），
//    而非本线程忙等 sendCommand——后者的节拍抖动会触发伺服报错或通信丢包。
//    算法线程只走 waitTick 的 1 ms 节拍；setJointPositions 写入目标后由 RT 线程取走。
//  * 抓取对齐：轮盘是外部硬件，Rokae SDK 不提供其编码器。对齐误差用双臂实测 TCP
//    位姿（RtSupportedFields::tcpPose_m）与"轮盘位形 (θ, d) 下把手的期望位姿"比较。
//    期望位姿 = mounting[side] × wheel(θ,d) × handle[side]，
//    其中 mounting 由 URDF 固定链推算，handle/tool 来自 grasp.json。
//  * 开环锁定/解锁：软件阶段标记，无外部夹爪或轮盘传感器反馈。
//
// SDK 通过独立共享库隔离内嵌 KDL，接口只传标准数组。
class RokaeDataLink final : public DataLink {
  public:
    RokaeDataLink(const std::string &urdf_path, const RokaeConfig &config,
                  const GraspGeometry &geometry)
        : geometry_(geometry) {
        require(!config.left_ip.empty() && !config.right_ip.empty(),
                "rokae.left_ip / rokae.right_ip 未配置");
        require(config.grasp_mode == "open_loop", "当前真机仅支持 rokae.grasp_mode: open_loop");
        require(!config.local_ip.empty(), "rokae.local_ip 未配置");
        require(config.left_ip != config.right_ip && config.local_ip != config.left_ip &&
                    config.local_ip != config.right_ip, "双臂与上位机 IP 必须不同");

        for (int i = 0; i < 7; ++i)
            require(std::isfinite(config.joint_stiffness[i]) && config.joint_stiffness[i] > 0 &&
                    config.joint_stiffness[i] <= (i < 4 ? 3000 : 300),
                    "Invalid xMateErPro joint stiffness (Nm/rad)");
        const auto model = urdf::parseURDFFile(urdf_path);
        require(model != nullptr, "无法解析 URDF: " + urdf_path);
        loadVelocityLimits(model);

        // 臂基座 → 轮盘 的安装变换：由 URDF 的固定关节链推算，
        // 与仿真用同一份模型定义，避免两处各写一份常量而漂移。
        for (int side = 0; side < 2; ++side) {
            const std::string base_link =
                std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2_base";
            // aircraft → base_link 的逆 × aircraft → steering_wheel
            const KDL::Frame base_in_root = fixedChainToRoot(model, base_link, "aircraft");
            const KDL::Frame wheel_in_root = geometry.wheel_origin;
            mounting_[side] = base_in_root.Inverse() * wheel_in_root;
        }

        const char *ip[2] = {config.left_ip.c_str(), config.right_ip.c_str()};
        for (int side = 0; side < 2; ++side) {
            arms_[side] = std::make_unique<RokaeArm>(ip[side], config.local_ip,
                                                     frameToRowMajor(geometry.tool), config.joint_stiffness);
            joint_pos_[side] = arms_[side]->position();
            for (int axis = 0; axis < 7; ++axis)
                target_[7 * side + axis] = joint_pos_[side][axis];
        }

        // 初始轮盘参考为回中位置；随后随共用轨迹更新。
    }

    ~RokaeDataLink() override {
        // Stop callbacks before any state they capture is destroyed.
        for (int i = 1; i >= 0; --i)
            if (arms_[i]) { try { arms_[i]->stop(); } catch (...) {} }
        for (auto &arm : arms_) arm.reset();
    }

    // —— 臂 IO ——

    double getJointPosition(Side side, int axis) const override {
        const int i = static_cast<int>(side);
        if (!powered_[i]) return arms_[i]->position()[axis];
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_pos_[i][axis];
    }

    // 关节速度。实时模式已订阅 jointVel_m，RT 回调里一并缓存；
    // 未收到有效实时速度时显式报错。
    double getJointVelocity(Side side, int axis) const override {
        const int i = static_cast<int>(side);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (joint_vel_valid_[i])
                return joint_vel_[i][axis];
        }
        throw std::runtime_error("Rokae joint velocity feedback unavailable");
    }

    void setJointPositions(const std::array<double, 14> &q) override {
        // 目标只在此处更新；由 setControlLoop 的回调（SDK RT 线程）按 1 ms 取走下发。
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (double value : q) require(std::isfinite(value), "Nonfinite joint target");
        target_ = q;
    }

    double jointVelLimit(Side side, int axis) const override {
        return vel_limit_[static_cast<int>(side)][axis];
    }

    bool isEnabled(Side side) const override { return powered_[static_cast<int>(side)]; }

    void enable(Side side) override {
        const int i = static_cast<int>(side);
        if (powered_[i]) return;
        // Refresh after a disabled interval: never replay the construction-time pose.
        const auto q = arms_[i]->position();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            joint_pos_[i] = q;
            std::copy(q.begin(), q.end(), target_.begin() + 7 * i);
            tcp_pose_valid_[i] = joint_vel_valid_[i] = false;
            feedback_time_[i] = 0;
        }
        arms_[i]->start([this, i](const RokaeSample &sample) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (double value : sample.position) require(std::isfinite(value), "Invalid joint feedback");
            for (double value : sample.velocity) require(std::isfinite(value), "Invalid velocity feedback");
            for (double value : sample.tcp) require(std::isfinite(value), "Invalid TCP feedback");
            if (!tcp_pose_valid_[i])
                std::copy(sample.position.begin(), sample.position.end(), target_.begin() + 7 * i);
            joint_pos_[i] = sample.position;
            joint_vel_[i] = sample.velocity;
            tcp_pose_[i] = sample.tcp;
            tcp_pose_valid_[i] = joint_vel_valid_[i] = true;
            feedback_time_[i] = monotonic();
            std::array<double, 7> q{};
            std::copy_n(target_.begin() + 7 * i, 7, q.begin());
            return q;
        });
        powered_[i] = true;
        const double deadline = monotonic() + 0.5;
        for (;;) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (tcp_pose_valid_[i]) break;
            }
            if (monotonic() > deadline) {
                disable(side);
                fail("Rokae realtime feedback timeout during enable");
            }
            std::this_thread::sleep_for(kTick);
        }
    }

    void disable(Side side) override {
        const int i = static_cast<int>(side);
        arms_[i]->stop();
        powered_[i] = false;
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_vel_valid_[i] = tcp_pose_valid_[i] = false;
    }

    // Only arm feedback is available: lock/unlock acknowledge a software control phase.
    GraspState graspState() const override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return updateGraspLocked();
    }

    uint64_t sendGraspCommand(GraspCommand command) override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        updateGraspLocked();
        return grasp_.command(command);
    }

    void setWheelReference(double angle, double displacement) override {
        require(std::isfinite(angle) && std::isfinite(displacement), "Invalid wheel reference");
        std::lock_guard<std::mutex> lock(state_mutex_);
        grasp_.reference(angle, displacement);
    }

    // —— 周期同步 ——

    void waitTick() override {
        if (next_tick_ == std::chrono::steady_clock::time_point{})
            next_tick_ = std::chrono::steady_clock::now();
        const auto now = std::chrono::steady_clock::now();
        if (next_tick_ < now - kTick) next_tick_ = now;
        next_tick_ += kTick;
        std::this_thread::sleep_until(next_tick_);
        // 命令下发与状态读取已在 RT 回调里完成，此处只提供算法侧 1 ms 节拍。
    }

    double time() const override { return monotonic(); }

  private:
    GraspState updateGraspLocked() const {
        const double now = monotonic();
        double feedback = now, speed = 0;
        double position[2], rotation[2];
        const auto reference = grasp_.state();
        for (int side = 0; side < 2; ++side) {
            if (powered_[side]) feedback = std::min(feedback, feedback_time_[side]);
            position[side] = rotation[side] = std::numeric_limits<double>::infinity();
            if (tcp_pose_valid_[side]) {
                const auto error = KDL::diff(frameFromRowMajor(tcp_pose_[side]),
                    desiredCylinderPose(side, reference.angle, reference.displacement));
                position[side] = error.vel.Norm();
                rotation[side] = error.rot.Norm();
            }
            for (double value : joint_vel_[side]) speed = std::max(speed, std::abs(value));
        }
        return grasp_.update(now, feedback, powered_[0] && powered_[1], position, rotation, speed);
    }
    // KDL::Frame → SDK 的 4x4 行优先数组（Frame::pos 的布局）
    static std::array<double, 16> frameToRowMajor(const KDL::Frame &f) {
        std::array<double, 16> m{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[4 * i + j] = f.M(i, j);
        for (int i = 0; i < 3; ++i) {
            m[4 * i + 3] = f.p(i);
            m[12 + i] = 0.0;
        }
        m[15] = 1.0;
        return m;
    }

    // 行优先 4x4 齐次矩阵（SDK tcpPose_m 的布局）→ KDL::Frame
    static KDL::Frame frameFromRowMajor(const std::array<double, 16> &m) {
        const KDL::Rotation R(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
        return KDL::Frame(R, KDL::Vector(m[3], m[7], m[11]));
    }

    // 目标"抓取圆柱中心"位姿 = 臂基座 → 轮盘 × 轮盘(θ, d) × handle
    //
    // 注意：因为 configureToolset() 已把控制器末端坐标系设为 grasp.json 的 tool
    // （法兰 → 圆柱中心），所以 tcpPose_m 直接给出圆柱中心的位姿，
    // 期望值里不再需要乘 tool⁻¹。
    KDL::Frame desiredCylinderPose(int side, double angle, double displacement) const {
        const KDL::Frame wheel =
            mounting_[side] *
            KDL::Frame(KDL::Rotation::RotZ(angle), KDL::Vector(0, 0, displacement));
        return wheel * geometry_.handles[side];
    }

    void loadVelocityLimits(const urdf::ModelInterfaceSharedPtr &model) {
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis) {
                const std::string name = std::string("AR5-5_07") + (side == 0 ? "L" : "R") +
                                         "-W4C4A2_joint_" + std::to_string(axis + 1);
                const auto joint = model->getJoint(name);
                require(joint && joint->limits, "URDF 缺少关节或速度限位: " + name);
                vel_limit_[side][axis] = joint->limits->velocity;
            }
    }

    std::unique_ptr<RokaeArm> arms_[2];

    std::array<double, 7> vel_limit_[2]{};
    std::array<double, 7> joint_pos_[2]{};
    std::array<double, 7> joint_vel_[2]{};
    bool joint_vel_valid_[2] = {false, false};
    std::array<double, 16> tcp_pose_[2]{};
    bool tcp_pose_valid_[2] = {false, false};
    std::array<double, 14> target_{};

    GraspGeometry geometry_;
    KDL::Frame mounting_[2];

    std::atomic<bool> powered_[2]{{false}, {false}};
    double feedback_time_[2]{};
    mutable OpenLoopGrasp grasp_;

    std::chrono::steady_clock::time_point next_tick_{};

    // 保护 joint_pos_ / tcp_pose_ / target_ 跨线程访问：
    // 算法线程写 target_、读 joint_pos_；SDK RT 线程（回调）读 target_、写 joint_pos_。
    mutable std::mutex state_mutex_;
};

std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const RokaeConfig &config,
                                            const GraspGeometry &geometry) {
    return std::make_unique<RokaeDataLink>(urdf_path, config, geometry);
}

} // namespace aviator

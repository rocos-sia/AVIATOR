#include "aviator/backend.hpp"

#include "rokae_sdk.hpp"
#include "OpenLoopGrasp.hpp"
#include "CommandContinuity.hpp"
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
#include <iomanip>
#include <sstream>

namespace aviator {

namespace {
constexpr std::chrono::microseconds kTick{1000};

double monotonic() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool ok, const char *message) {
    if (!ok)
        fail(message);
}

void require(bool ok, const std::string &message) {
    if (!ok) fail(message);
}

// 在 URDF 里沿固定关节链求 link 在 root 系下的位姿。
// 只用于推算臂基座 → 轮盘的安装变换（中间全是 fixed 关节或已知转角为 0）。
// 这里只支持"纯固定关节"链，遇到可动关节就报错，避免静默算错。
pinocchio::SE3 fixedChainToRoot(const urdf::ModelInterfaceSharedPtr &model,
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

    auto result = pinocchio::SE3::Identity();
    // 从 root 往下累乘
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const auto *joint = *it;
        const auto &r = joint->parent_to_joint_origin_transform.rotation;
        const auto &p = joint->parent_to_joint_origin_transform.position;
        const pinocchio::SE3 local(Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized(),
                                   Eigen::Vector3d(p.x, p.y, p.z));
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
    std::atomic<double> command_deadline_{0};
    RokaeDataLink(const std::string &urdf_path, const RokaeConfig &config,
                  const GraspGeometry &geometry)
        : geometry_(geometry) {
        require(!config.left_ip.empty() && !config.right_ip.empty(),
                "rokae.left_ip / rokae.right_ip 未配置");
        require(config.grasp_mode == "open_loop", "当前真机仅支持 rokae.grasp_mode: open_loop");
        require(!config.left_local_ip.empty(), "rokae.left_local_ip 未配置");
        require(!config.right_local_ip.empty(), "rokae.right_local_ip 未配置");
        require(config.left_ip != config.right_ip, "左右臂控制器 IP 必须不同");
        for (const auto &local_ip : {config.left_local_ip, config.right_local_ip})
            require(local_ip != config.left_ip && local_ip != config.right_ip,
                    "上位机 IP 不能与机械臂控制器 IP 相同");

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
            const pinocchio::SE3 base_in_root = fixedChainToRoot(model, base_link, "aircraft");
            const pinocchio::SE3 wheel_in_root = geometry.wheel_origin;
            mounting_[side] = base_in_root.inverse() * wheel_in_root;
        }

        const char *ip[2] = {config.left_ip.c_str(), config.right_ip.c_str()};
        const std::string local_ip[2] = {config.left_local_ip, config.right_local_ip};
        for (int side = 0; side < 2; ++side) {
            endpoints_[side] = std::string(side == 0 ? "left" : "right") +
                " robot=" + ip[side] + " local=" + local_ip[side];
            try {
                arms_[side] = std::make_unique<RokaeArm>(ip[side], local_ip[side],
                                                         poseToRowMajor(geometry.tools[side]), config.joint_stiffness);
            } catch (const std::exception &e) {
                throw std::runtime_error("Rokae connect/configure " + endpoints_[side] + ": " + e.what());
            }
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

    ArmFeedback armFeedback() const override {
        const auto s = captureState();
        ArmFeedback f; f.target=s.target;
        for(int side=0;side<2;++side) {
            f.enabled[side]=s.powered[side];
            f.valid[side]=s.tcp_valid[side] && s.velocity_valid[side];
            f.sample_time[side]=s.feedback_time[side];
            for(int j=0;j<7;++j){ f.q[side*7+j]=s.position[side][j];f.dq[side*7+j]=s.velocity[side][j]; }
            if(f.valid[side]) {
                const auto pose=poseFromRowMajor(s.tcp[side]);
                Eigen::Quaterniond q(pose.rotation());q.normalize();
                for(int j=0;j<3;++j)f.tcp[side*7+j]=pose.translation()[j];
                f.tcp[side*7+3]=q.x();f.tcp[side*7+4]=q.y();f.tcp[side*7+5]=q.z();f.tcp[side*7+6]=q.w();
            }
        }
        return f;
    }

    void commandDeadline(double deadline) override { command_deadline_.store(deadline); }

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
        for (double value : q) require(std::isfinite(value), "Nonfinite joint target");
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int side = 0; side < 2; ++side)
            require(!powered_[side] || consumed_sequence_[side] == command_sequence_,
                    "Previous dual-arm command has not been consumed");
        target_ = q;
        ++command_sequence_;
    }

    std::array<double, 14> jointTargets() const override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return target_;
    }

    double jointVelLimit(Side side, int axis) const override {
        return vel_limit_[static_cast<int>(side)][axis];
    }

    bool isEnabled(Side side) const override { return powered_[static_cast<int>(side)]; }

    void enable(Side side) override {
        const int i = static_cast<int>(side);
        if (powered_[i]) return;
        if (!powered_[0] && !powered_[1]) {
            callback_fault_ = false;
            // 两臂的阻塞初始化均在第一个周期线程启动前完成。
            try { for (auto &arm : arms_) arm->prepare(); }
            catch (...) {
                auto error = std::current_exception();
                for (auto &arm : arms_) { try { arm->stop(); } catch (...) {} }
                std::rethrow_exception(error);
            }
        }
        // Refresh after a disabled interval: never replay the construction-time pose.
        const auto q = arms_[i]->position();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            joint_pos_[i] = q;
            std::copy(q.begin(), q.end(), target_.begin() + 7 * i);
            tcp_pose_valid_[i] = joint_vel_valid_[i] = false;
            feedback_time_[i] = 0;
            feedback_count_[i] = 0;
            max_feedback_gap_[i] = 0;
        }
        arms_[i]->start([this, i](const RokaeSample &sample) {
          try {
            require(!callback_fault_, "Dual-arm realtime command fault");
            const double deadline = command_deadline_.load();
            require(deadline <= 0 || monotonic() < deadline, "Local RT command watchdog expired");
            for (double value : sample.position) require(std::isfinite(value), "Invalid joint feedback");
            for (double value : sample.velocity) require(std::isfinite(value), "Invalid velocity feedback");
            for (double value : sample.tcp) require(std::isfinite(value), "Invalid TCP feedback");
            // 周期线程不等待业务线程释放锁；竞争时重发上一条已发送目标。
            // 不确认新序号，轨迹线程会继续等待，不能因此覆盖/跳过目标。
            std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
            if (!lock.owns_lock()) return sent_[i];
            if (!tcp_pose_valid_[i]) {
                std::copy(sample.position.begin(), sample.position.end(), target_.begin() + 7 * i);
                sent_[i] = sample.position;
            }
            joint_pos_[i] = sample.position;
            joint_vel_[i] = sample.velocity;
            tcp_pose_[i] = sample.tcp;
            tcp_pose_valid_[i] = joint_vel_valid_[i] = true;
            const double now = monotonic();
            if (feedback_time_[i] > 0)
                max_feedback_gap_[i] = std::max(max_feedback_gap_[i], now - feedback_time_[i]);
            feedback_time_[i] = now;
            ++feedback_count_[i];
            std::array<double, 7> q{};
            std::copy_n(target_.begin() + 7 * i, 7, q.begin());
            const int bad = excessiveJointStep(sent_[i], q, vel_limit_[i]);
            if (bad >= 0)
                fail("Rejected discontinuous realtime command: J" + std::to_string(bad + 1) +
                     " previous=" + std::to_string(sent_[i][bad]) + " next=" + std::to_string(q[bad]) +
                     " max_step_rad=" + std::to_string(vel_limit_[i][bad] * 0.001));
            sent_[i] = q;
            consumed_sequence_[i] = command_sequence_;
            return q;
          } catch (...) {
            callback_fault_ = true; // 一臂出错时，另一臂也拒绝后续位置指令。
            throw;
          }
        }, [this, i](const std::array<double, 7> &q) {
            // startMove 前使用刚刷新的关节位置初始化保持目标。
            // 不标记实时反馈已就绪；使能仍需等待 startLoop 后的第一帧。
            std::lock_guard<std::mutex> lock(state_mutex_);
            joint_pos_[i] = sent_[i] = q;
            std::copy(q.begin(), q.end(), target_.begin() + 7 * i);
        });
        powered_[i] = true;
        const double deadline = monotonic() + 0.5;
        for (;;) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (tcp_pose_valid_[i]) break;
            }
            if (arms_[i]->motionFailed()) {
                const auto details = diagnostics();
                disable(side);
                fail("Rokae realtime startup failed\n" + details);
            }
            if (monotonic() > deadline) {
                disable(side);
                fail("Rokae realtime feedback timeout during enable");
            }
            std::this_thread::sleep_for(kTick);
        }
        if (powered_[0] && powered_[1]) {
            const auto snapshot = captureState();
            for (int side = 0; side < 2; ++side)
                if (arms_[side]->motionFailed() || monotonic() - snapshot.feedback_time[side] > 0.1)
                    fail("Rokae dual-arm startup health check failed\n" + diagnostics());
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
        std::lock_guard<std::mutex> lock(grasp_mutex_);
        return updateGraspLocked();
    }

    std::string diagnostics() const override {
        std::string result;
        {
            std::lock_guard<std::mutex> lock(grasp_mutex_);
            result = fault_snapshot_.empty() ? formatSnapshot(monotonic(), grasp_.state(), captureState()) : fault_snapshot_;
        }
        // 不持有反馈锁调用 SDK；首次故障快照与当前 SDK 错误状态分开标注。
        result += "\nCurrent SDK status:";
        for (int side = 0; side < 2; ++side)
            if (arms_[side]) result += "\n  " + endpoints_[side] + " " + arms_[side]->diagnostics();
        return result;
    }

    uint64_t sendGraspCommand(GraspCommand command) override {
        std::lock_guard<std::mutex> lock(grasp_mutex_);
        updateGraspLocked();
        return grasp_.command(command);
    }

    void setWheelReference(double angle, double displacement) override {
        require(std::isfinite(angle) && std::isfinite(displacement), "Invalid wheel reference");
        std::lock_guard<std::mutex> lock(grasp_mutex_);
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
        // 两个独立 SDK 回调都取走上一条目标后才允许推进，不能覆盖尚未消费的目标。
        // 这里只约束指令顺序，不宣称两台控制器有硬件时钟同步。
        const double deadline = monotonic() + 0.1;
        for (;;) {
            for (int side = 0; side < 2; ++side) {
                if (powered_[side] && arms_[side]->motionFailed()) callback_fault_ = true;
            }
            if (callback_fault_) fail("Rokae realtime command failed\n" + diagnostics());
            bool consumed = true;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int side = 0; side < 2; ++side)
                    consumed = consumed && (!powered_[side] || consumed_sequence_[side] == command_sequence_);
            }
            if (consumed) return;
            if (monotonic() >= deadline) {
                callback_fault_ = true;
                fail("Rokae command acknowledgement timeout (>100 ms)\n" + diagnostics());
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    double time() const override { return monotonic(); }

  private:
    struct StateSnapshot {
        std::array<double, 7> position[2], velocity[2];
        std::array<double, 16> tcp[2];
        std::array<double, 14> target;
        bool powered[2], tcp_valid[2], velocity_valid[2];
        double feedback_time[2], max_gap[2];
        uint64_t callbacks[2];
    };
    StateSnapshot captureState() const {
        StateSnapshot snapshot{};
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int side = 0; side < 2; ++side) {
            snapshot.position[side] = joint_pos_[side];
            snapshot.velocity[side] = joint_vel_[side];
            snapshot.tcp[side] = tcp_pose_[side];
            snapshot.powered[side] = powered_[side];
            snapshot.tcp_valid[side] = tcp_pose_valid_[side];
            snapshot.velocity_valid[side] = joint_vel_valid_[side];
            snapshot.feedback_time[side] = feedback_time_[side];
            snapshot.max_gap[side] = max_feedback_gap_[side];
            snapshot.callbacks[side] = feedback_count_[side];
        }
        snapshot.target = target_;
        return snapshot;
    }

    GraspState updateGraspLocked() const {
        const auto snapshot = captureState();
        const double now = monotonic();
        double feedback = now, speed = 0;
        double position[2], rotation[2];
        const auto reference = grasp_.state();
        for (int side = 0; side < 2; ++side) {
            if (snapshot.powered[side]) feedback = std::min(feedback, snapshot.feedback_time[side]);
            position[side] = rotation[side] = std::numeric_limits<double>::infinity();
            if (snapshot.tcp_valid[side]) {
                const auto actual = poseFromRowMajor(snapshot.tcp[side]);
                const auto desired = desiredCylinderPose(side, reference.angle, reference.displacement);
                position[side] = (actual.translation() - desired.translation()).norm();
                rotation[side] = rotationError(actual, desired);
            }
            for (double value : snapshot.velocity[side]) speed = std::max(speed, std::abs(value));
        }
        auto state = grasp_.update(now, feedback, snapshot.powered[0] && snapshot.powered[1], position, rotation, speed);
        if (!reference.fault && state.fault)
            fault_snapshot_ = "First fault snapshot:\n" + formatSnapshot(now, state, snapshot);
        if (!state.fault) fault_snapshot_.clear();
        return state;
    }

    // 在反馈锁外计算/格式化；SDK 周期线程不访问 grasp_mutex_。
    std::string formatSnapshot(double now, const GraspState &state, const StateSnapshot &snapshot) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(4)
            << "Rokae reason=" << (state.fault_reason.empty() ? "none latched" : state.fault_reason)
            << " time_s=" << now << " fault=" << state.fault << " locked=" << state.locked
            << " ready=" << state.ready << " wheel_ref_rad=" << state.angle
            << " wheel_ref_m=" << state.displacement;
        auto values = [&](const auto &q) { for (double x : q) out << ' ' << x; };
        for (int side = 0; side < 2; ++side) {
            out << "\n  " << endpoints_[side] << " enabled=" << snapshot.powered[side]
                << " tcp_valid=" << snapshot.tcp_valid[side] << " velocity_valid=" << snapshot.velocity_valid[side]
                << " callbacks=" << snapshot.callbacks[side] << " feedback_age_ms=";
            if (snapshot.feedback_time[side] > 0) out << (now-snapshot.feedback_time[side])*1000;
            else out << "no feedback";
            out << " max_callback_gap_ms=" << snapshot.max_gap[side]*1000
                << " TCP_position_error_mm=" << state.position_error[side]*1000
                << " TCP_rotation_error_rad=" << state.rotation_error[side];
            out << "\n    q_rad:"; values(snapshot.position[side]);
            out << "\n    dq_rad_s:"; values(snapshot.velocity[side]);
            out << "\n    target_rad:";
            for (int j = 0; j < 7; ++j) out << ' ' << snapshot.target[7*side+j];
            if (snapshot.tcp_valid[side]) {
                const auto desired = desiredCylinderPose(side, state.angle, state.displacement);
                out << "\n    TCP_actual_xyz_m:";
                for (int j = 0; j < 3; ++j) out << ' ' << snapshot.tcp[side][4*j+3];
                out << " TCP_expected_xyz_m:";
                for (int j = 0; j < 3; ++j) out << ' ' << desired.translation()[j];
            }
        }
        return out.str();
    }
    // 目标"抓取圆柱中心"位姿 = 臂基座 → 轮盘 × 轮盘(θ, d) × handle
    //
    // 注意：控制器末端坐标系已设为 grasp.json 的 tool.left / tool.right
    // （法兰 → 圆柱中心），所以 tcpPose_m 直接给出圆柱中心的位姿，
    // 期望值里不再需要乘 tool⁻¹。
    pinocchio::SE3 desiredCylinderPose(int side, double angle, double displacement) const {
        const pinocchio::SE3 wheel =
            mounting_[side] *
            pinocchio::SE3(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                              Eigen::Vector3d(0, 0, displacement));
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
    std::array<double, 7> sent_[2]{};
    uint64_t command_sequence_ = 0, consumed_sequence_[2]{};
    std::atomic<bool> callback_fault_{false};

    GraspGeometry geometry_;
    pinocchio::SE3 mounting_[2]{pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};

    std::atomic<bool> powered_[2]{{false}, {false}};
    double feedback_time_[2]{};
    uint64_t feedback_count_[2]{};
    double max_feedback_gap_[2]{};
    std::string endpoints_[2];
    mutable std::string fault_snapshot_;
    mutable OpenLoopGrasp grasp_;

    std::chrono::steady_clock::time_point next_tick_{};

    // 保护 joint_pos_ / tcp_pose_ / target_ 跨线程访问：
    // 算法线程写 target_、读 joint_pos_；SDK RT 线程（回调）读 target_、写 joint_pos_。
    mutable std::mutex state_mutex_;
    mutable std::mutex grasp_mutex_; // 状态计算、日志及软件锁定，RT 回调不获取此锁。
};

std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const RokaeConfig &config,
                                            const GraspGeometry &geometry) {
    return std::make_unique<RokaeDataLink>(urdf_path, config, geometry);
}

} // namespace aviator

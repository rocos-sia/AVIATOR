#pragma once
#include "aviator/DataLink.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace aviator {

// MuJoCo 直接访问的 DataLink 实现（单进程，无 IPC）。
//
// 物理语义与原 rocos_mujoco 仿真器保持一致：
//  * MJCF 没有 actuator（nu == 0）。位置伺服通过 qfrc_applied 施加
//    "重力前馈 + 比例力矩" 实现，对应原 MujocoSimulator::applyControl 的 CSP 分支，
//    并按原实现在无内置 actuator 时补上关节阻尼抑制超调。
//  * 抓取 weld 的运行时开关在 mjData::eq_active；mjModel::eq_active0 只是
//    mj_resetData 使用的初始值，运行期修改它不会生效。
//  * 必须显式载入 aviator_home keyframe（默认 qpos 全零不是可用姿态）。
//
// 时序：本后端持有**独立物理线程**，按 1 ms 节拍持续推进物理（等价于原架构里
// 仿真器进程的角色）。控制器的 waitTick() 只做"等待下一个节拍完成"，
// 与原实现的信号量同步语义一致。这样控制器空闲时物理仍在推进，
// 操纵盘不会因失去支撑而下垂。
class MuJoCoDirectDataLink final : public DataLink {
  public:
    MuJoCoDirectDataLink(mjModel *model, mjData *data, const std::string &urdf_path);
    ~MuJoCoDirectDataLink() override;

    // 臂 IO
    double getJointPosition(Side side, int axis) const override;
    double getJointVelocity(Side side, int axis) const override;
    void setJointPositions(const std::array<double, 14> &q) override;
    double jointVelLimit(Side side, int axis) const override;
    bool isEnabled(Side side) const override;
    void enable(Side side) override;
    void disable(Side side) override;

    // 抓取 IO
    GraspState graspState() const override;
    uint64_t sendGraspCommand(GraspCommand command) override;

    // 等待下一个物理节拍完成（物理线程负责推进，这里只同步）
    void waitTick() override;

    // 仿真时间（秒），由物理线程推进
    double time() const override;

    // 渲染/实测采样时与完整物理状态更新互斥。
    std::mutex *physicsMutex() override { return &mutex_; }

    // true：物理按 1 ms 墙钟节拍（仿真时间 ≈ 真实时间，用于可视化）
    // false：与 waitTick 严格锁步，尽快推进（无头验证用，可数十倍加速且可复现）
    void setRealTime(bool enabled) override;

    uint64_t stepCount() const;
    // Minimum/maximum measured J2 over every physics step, both arms (radians).
    std::array<double, 2> elbowRange() const;

  private:
    void initJointMapping(const std::string &urdf_path);
    // 每步更新对齐误差、稳定窗口与故障标志（对应原 updateAviator 的职责）
    // 调用方必须持有 mutex_。
    void updateAlignmentLocked();
    // 物理线程主循环
    void physicsLoop();

    static int sideOffset(Side side) { return (side == Side::Left) ? 0 : 7; }

    mjModel *model_;
    mjData *data_;

    std::array<int, 14> joint_qpos_adr_{};
    std::array<int, 14> joint_dof_adr_{};
    std::array<double, 14> joint_vel_limit_{};

    int wheel_qpos_adr_[2]{};
    int wheel_dof_adr_[2]{};

    int tcp_site_[2]{};
    int handle_site_[2]{};
    int weld_id_[2]{};

    bool enabled_[2] = {false, false};
    std::array<double, 14> target_{}; // 下发的关节目标
    std::array<double, 14> hold_{};   // 未使能时的保持位形

    uint64_t command_seq_ = 0;
    uint64_t ack_seq_ = 0;
    GraspResult last_result_ = GraspResult::Ok;
    uint32_t fault_ = 0;

    // 对齐状态（物理线程每步更新，供 graspState 读取）
    double position_error_[2]{};
    double rotation_error_[2]{};
    double stable_time_ = 0.0; // 连续对齐累计时长
    double bad_time_ = 0.0;    // 锁定后连续失稳累计时长
    uint32_t ready_ = 0;

    // 节拍同步
    mutable std::mutex mutex_;
    std::condition_variable tick_cv_;
    uint64_t tick_ = 0;      // 已完成步数
    uint64_t consumed_ = 0;  // waitTick 已消费到的步数
    double sim_step_span_ = 0.0; // 本步跨过的仿真时间
    bool real_time_ = true;
    bool running_ = false;
    std::thread physics_thread_;
    std::chrono::steady_clock::time_point next_tick_;

    std::array<double, 2> elbow_range_{{1e100, -1e100}};
    std::array<double, 14> original_damping_{};
};

} // namespace aviator

#pragma once
#include "aviator/DataLink.hpp"
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace aviator {

// 前向声明
class Kinematics;
class CollisionChecker;

// 公共状态：抓取状态 + 轮盘位形
struct Status {
    std::string motion_error; // 异步 Servo 故障/超时原因；新运动开始时清空
    bool open_loop = false; // 真机开环阶段标记，locked 不代表外部机构反馈
    double angle = 0;              // 轮盘转角 (rad)
    double displacement = 0;       // 轮盘推拉位移 (m)
    double position_error[2]{};    // 左右抓取位置误差 (m)
    double rotation_error[2]{};    // 左右抓取旋转误差 (rad)
    uint32_t locked = 0;           // 是否锁定
    uint32_t ready = 0;            // 是否就绪
    uint32_t fault = 0;            // 是否故障
    uint64_t ack = 0;              // 命令确认序号
    GraspResult result = GraspResult::Ok;
};

// Aviator 双臂控制器
// 除 ServoWheel/Stop/状态读取外，动作接口阻塞到完成；失败抛 std::runtime_error
// Stop 可由另一线程调用；GetStatus 可在运动期间读取
class Aviator {
  public:
    // 构造函数（依赖注入）：直接传入数据链接、运动学和碰撞检测器。
    // 运行时注入 RemoteLink；运动学和碰撞检查器为空时按 robot.yaml 创建。
    Aviator(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
            std::unique_ptr<CollisionChecker> collision_checker, const std::string &config_file);

    ~Aviator();

    Aviator(const Aviator &) = delete;
    Aviator &operator=(const Aviator &) = delete;

    // 初始化（加载配置）
    void Init();

    // 使能/失能双臂
    void Enable();
    void Disable();

    // 接近把手（当前位置 → home 并停稳 → 预接近 → 精确对准）
    void ApproachHandles();

    // 锁定抓取
    void LockHandles();

    // 操纵轮盘
    // angle_rad: 目标转角 (rad)
    // displacement_m: 目标推拉位移 (m)
    // v: 速度比例 (0, 1]，同时约束轮盘转速、推拉速度与各关节速度。
    void MoveWheel(double angle_rad, double displacement_m, double v = 0.5);

    // 发布最新绝对目标，立即返回；建议每 20 ms 更新，超过 servo_timeout 则减速保持。
    // 先锁定把手；异步错误见 Status.motion_error / GetState()。
    // Stop 后等待状态离开 SERVO，再执行其他动作。
    void ServoWheel(double angle_rad, double displacement_m, double v = 0.5);

    // 解锁抓取
    void UnlockHandles();
    // Explicit release program: unlock, retreat along the approach offset; never home.
    void ReleaseHandles();

    // 重置故障
    void ResetFault();
    // Acknowledge a fault already cleared by maintenance; no device reset/unlock/enable.
    void AcknowledgeFault();

    // 请求减速停止（可从另一线程调用）；反馈/控制故障时转为停止后端控制。
    void Stop() noexcept;

    // 获取状态
    Status GetStatus();

    // 获取控制器状态字符串
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace aviator

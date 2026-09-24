#pragma once
#include "aviator/DataLink.hpp"
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// MuJoCo 句柄前向声明（仅 backend: mujoco 时需要；见 aviator/backend.hpp）
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

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
    // 测试与自定义后端用；后端由配置选择时用下面的重载。
    Aviator(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
            std::unique_ptr<CollisionChecker> collision_checker, const std::string &config_file);

    // 构造函数（配置驱动）：后端、运动学、碰撞检测都在 Init() 里按 config 创建。
    //   backend: mujoco → 需要 model/data（由 aviator 入口加载模型后传入）
    //   backend: rokae  → 需要编译期存在 xCore SDK；model/data 传 nullptr
    // 这是"仿真 / 真机只差一个配置字段"的入口。
    Aviator(mjModel *model, mjData *data, const std::string &config_file);

    ~Aviator();

    Aviator(const Aviator &) = delete;
    Aviator &operator=(const Aviator &) = delete;

    // 初始化（加载配置）
    void Init();

    // 使能/失能双臂
    void Enable();
    void Disable();

    // 接近把手（预接近 + 精确对准）
    void ApproachHandles();

    // 锁定抓取
    void LockHandles();

    // 操纵轮盘
    // angle_rad: 目标转角 (rad)
    // displacement_m: 目标推拉位移 (m)
    // v: 速度比例 (0, 1]，同时约束轮盘转速、推拉速度与各关节速度。
    void MoveWheel(double angle_rad, double displacement_m, double v = 0.5);

    // 发布最新绝对目标，立即返回；建议每 20 ms 更新，超过 servo_timeout 则保持。
    // 先锁定把手；异步错误见 Status.motion_error / GetState()。
    // Stop 后等待状态离开 SERVO，再执行其他动作。
    void ServoWheel(double angle_rad, double displacement_m, double v = 0.5);

    // 解锁抓取
    void UnlockHandles();

    // 重置故障
    void ResetFault();

    // 停止运动（可从另一线程调用）
    void Stop() noexcept;

    // 是否按真实时间节拍推进（转发给后端）。
    // 仿真可置 false 让物理尽快推进；真机后端忽略此设置（不可能快于真实时间）。
    // 必须在 Init() 之后调用。
    void SetRealTime(bool enabled);

    // 渲染线程用：与物理步进互斥的锁；后端不支持时返回 nullptr（无需加锁）。
    // 仿真后端持有独立物理线程，渲染时必须持此锁，否则会边步进边读几何。
    // 必须在 Init() 之后调用。
    std::mutex *PhysicsMutex();

    // 获取状态
    Status GetStatus();

    // 获取控制器状态字符串
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace aviator

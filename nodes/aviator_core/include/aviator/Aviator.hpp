#pragma once
#include "aviator/DataLink.hpp"
#include "aviator/RobotStateMachine.hpp"
#include <functional>
#include <optional>
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

// Read-only whole-robot status, distinct from the existing actuator Status/GetState().
struct SystemStatus {
    std::string state, current_error, last_error;
    unsigned state_code = 0;
    uint64_t generation = 0;
    bool accepts_control = false, brake_requested = false;
    fsm::Snapshot conditions;
};
// Supplied by a trusted same-host runtime; no simulated defaults in managed mode.
struct ManagedOptions {
    std::function<fsm::Snapshot()> snapshot;
    std::function<void(bool)> allow_motion;
    std::function<void()> heartbeat;
    std::function<void()> request_brake;
    std::function<void(const SystemStatus&)> report;
};

// Lowercase: direct executor API. Uppercase operations: managed FSM API.
// Managed Init enables/configures impedance and ends in READY; EnterStandby homes before STANDBY.
// Managed API/queries must be called by the creating thread; Update() runs every ~5 ms.
// Blocking executor tasks use one worker; it never calls the state machine.
class Aviator {
  public:
    // 构造函数（依赖注入）：直接传入数据链接、运动学和碰撞检测器。
    // 运行时注入 RemoteLink；运动学和碰撞检查器为空时按 robot.yaml 创建。
    Aviator(std::unique_ptr<DataLink> datalink, std::unique_ptr<Kinematics> kinematics,
            std::unique_ptr<CollisionChecker> collision_checker, const std::string &config_file,
            std::optional<ManagedOptions> managed = std::nullopt);

    ~Aviator();

    Aviator(const Aviator &) = delete;
    Aviator &operator=(const Aviator &) = delete;

    // Managed lifecycle: Boot is internal, not a seventh external operation.
    void Init();
    void Update();
    fsm::Reply EnterStandby();
    fsm::Reply GraspWheel();
    fsm::Reply StartControl();
    fsm::Reply ExitControl();
    fsm::Reply LeaveWheel();
    fsm::Reply ResetError();
    // sample_mono_us must come from the validated input, in local monotonic time.
    // POSITION_HOLD adapters supply the validated device-check time, not a fabricated local refresh.
    bool ServoWheel(double angle_rad, double displacement_m, double v, uint64_t sample_mono_us);
    void EmergencyStop(const std::string& reason); // Internal safety input, not a normal operation.
    SystemStatus GetSystemStatus() const;
    std::string GetSystemState() const;

    // Direct mode only; same algorithms and blocking behavior as the original API.
    void init();
    void enable();
    void disable();
    void approachHandles();
    void lockHandles();
    void moveWheel(double angle_rad, double displacement_m, double v = 0.5);
    void servoWheel(double angle_rad, double displacement_m, double v = 0.5);
    void unlockHandles();
    void releaseHandles();
    void resetFault();
    void acknowledgeFault();
    void stop(); // Direct mode cancellation, callable from another thread.

    // 获取状态
    Status GetStatus();

    // 获取控制器状态字符串
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    class Managed;
    std::unique_ptr<Managed> managed_; // Destroy worker before executor.
    void requireDirect() const;
    Managed& managed() const;
};

} // namespace aviator

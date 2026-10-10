#pragma once
#include <array>
#include <functional>
#include <cmath>
#include <atomic>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <mutex>
#include <string>

namespace aviator {

inline void validateJointStiffness(const std::array<double, 7>& values) {
    for (size_t j = 0; j < values.size(); ++j)
        if (!std::isfinite(values[j]) || values[j] <= 0 || values[j] > (j < 4 ? 3000 : 300))
            throw std::runtime_error("Invalid joint stiffness on J" + std::to_string(j + 1));
}

enum class Side { Left = 0, Right = 1 };
enum class GraspCommand { Lock, Unlock, ResetFault };
enum class GraspResult { Ok, NotAligned, NotEnabled, Fault };

struct MotionCancelled : std::runtime_error {
    MotionCancelled() : std::runtime_error("Core motion authorization revoked") {}
};

// 抓取状态
struct GraspState {
    bool open_loop = false; // true: wheel pose/latch state are software references, not feedback
    double heartbeat = 0;              // 后端存活时间戳
    uint32_t locked = 0;               // bit0=左侧，bit1=右侧；两侧锁定为 3
    uint32_t ready = 0;                // 0=未就绪, 1=就绪
    uint32_t fault = 0;                // 0=正常, 1=故障
    std::string fault_reason;          // 保留首次故障原因，ResetFault 时清除
    double position_error[2]{};        // 左右抓取位置误差 (m)
    double rotation_error[2]{};        // 左右抓取旋转误差 (rad)
    double angle = 0;                  // 轮盘当前转角 (rad)
    double displacement = 0;           // 轮盘当前推拉位移 (m)
    uint64_t ack = 0;                  // 命令确认序号
    GraspResult result = GraspResult::Ok;
};

struct ArmFeedback {
    std::array<double, 14> q{}, dq{}, target{}, tcp{};
    std::array<double, 2> sample_time{};
    std::array<bool, 2> enabled{}, valid{};
};

struct JointFrame {
    std::array<double, 14> q{};
    double angle = 0, displacement = 0;
    std::array<double, 14> dq{}, ddq{};
    // Core-local schedule, excluded from the arm wire protocol; negative means keep target.
    std::array<double, 2> hand_closure{{-1, -1}};
};

struct ServoGoal {
    double angle = 0, displacement = 0, speed_ratio = 1;
    uint64_t input_sample = 0; // Original Core input time; retransmission must not refresh it.
    bool stop = false;
};

// 数据链接抽象接口：臂IO + 抓取IO + 周期同步
class DataLink {
  public:
    virtual ~DataLink() = default;
    // Non-RT Core execution API. The network adapter streams bounded windows;
    // hardware backends only implement the device IO below.
    virtual void runTrajectory(const std::vector<JointFrame>&, const std::atomic<bool>&) {
        throw std::runtime_error("Trajectory streaming requires the Core network adapter");
    }
    virtual void stopTrajectory() {}
    virtual bool latestServoSupported() const { return false; }
    // False means the device already stopped this run; finish before starting a new ID.
    virtual bool latestServo(const ServoGoal&) { throw std::runtime_error("Latest servo unavailable"); }
    virtual void finishLatestServo() { throw std::runtime_error("Latest servo unavailable"); }
    virtual void beginStream(const std::vector<JointFrame>&) { throw std::runtime_error("Stream unavailable"); }
    virtual void appendStream(const std::vector<JointFrame>&) { throw std::runtime_error("Stream unavailable"); }
    virtual size_t streamAhead() const { throw std::runtime_error("Stream unavailable"); }
    virtual void finishStream() { throw std::runtime_error("Stream unavailable"); }
    virtual ArmFeedback armFeedback() const { throw std::runtime_error("Feedback snapshot unavailable"); }
    virtual void commandDeadline(double) {} // CLOCK_MONOTONIC seconds, checked in SDK callback.


    // 臂IO
    virtual double getJointPosition(Side side, int axis) const = 0;
    virtual double getJointVelocity(Side side, int axis) const = 0;
    virtual void setJointPositions(const std::array<double, 14> &q) = 0;
    // 使能后读取后端实际保持的目标，不能用实测反馈替代上一条指令。
    virtual std::array<double, 14> jointTargets() const = 0;
    virtual double jointVelLimit(Side side, int axis) const = 0;
    virtual bool isEnabled(Side side) const = 0;
    virtual void enable(Side side) = 0;   // 失败抛 std::runtime_error
    virtual void disable(Side side) = 0;  // 失败抛 std::runtime_error

    // Non-RT, dual-arm lifecycle operation. check() must run between SDK stages.
    // Rebase the hold to fresh measured joints before changing stiffness; read back jointTargets().
    // force_reapply executes the lifecycle even when the requested values are unchanged.
    virtual void setJointStiffness(const std::array<double, 7>&, const std::function<void()>&,
                                   bool force_reapply = false) {
        throw std::runtime_error("Runtime joint stiffness unavailable");
    }
    virtual void setImpedanceProfile(bool /*following*/) {
        throw std::runtime_error("Impedance profiles require the Core network adapter");
    }

    // 抓取IO
    virtual GraspState graspState() const = 0;
    // Acquire a new camera observation on demand, in control wheel coordinates (rad, m).
    // check() keeps cancellation and backend supervision active while waiting.
    virtual std::array<double, 2> readCameraWheel(const std::string&, uint64_t, uint64_t,
                                                const std::function<void()>&) {
        throw std::runtime_error("Camera grasp input requires the Core network adapter");
    }
    // 仅在错误路径调用；不能在实时回调中进行格式化/打印。
    virtual std::string diagnostics() const { return {}; }
    virtual uint64_t sendGraspCommand(GraspCommand command) = 0; // 返回请求序号

    // 周期同步
    // Update the wheel reference alongside joint targets; simulations keep reading the physical wheel.
    virtual void setWheelReference(double /*angle*/, double /*displacement*/) {}

    virtual void waitTick() = 0; // 推进/等待一个控制周期（1 ms）

    // 控制用的单调时间源（秒）。仿真是仿真时间，真机是墙钟时间。
    // 用于反馈时效和驻留；轨迹每次 waitTick 最多推进 1 ms，不追赶漏掉的周期。
    virtual double time() const = 0;

    // 是否按真实时间节拍推进。
    //
    // 仿真可以置 false 让物理尽快推进（无头验证时快几十倍）；真机不可能快于
    // 真实时间，因此默认实现是空操作——后端各自决定是否支持加速。
    virtual void setRealTime(bool /*enabled*/) {}

    // 与"物理推进"互斥的锁，供渲染线程使用（持锁后再读几何/渲染）。
    //
    // 只有自带物理线程的后端需要它（仿真）。真机没有物理步进，
    // 返回 nullptr 表示"无需加锁"，调用方据此跳过加锁。
    virtual std::mutex *physicsMutex() { return nullptr; }
};

} // namespace aviator

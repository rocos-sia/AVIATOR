#pragma once
#include <array>
#include <cstdint>
#include <mutex>

namespace aviator {

enum class Side { Left = 0, Right = 1 };
enum class GraspCommand { Lock, Unlock, ResetFault };
enum class GraspResult { Ok, NotAligned, NotEnabled, Fault };

// 抓取状态
struct GraspState {
    bool open_loop = false; // true: wheel pose/latch state are software references, not feedback
    double heartbeat = 0;              // 后端存活时间戳
    uint32_t locked = 0;               // bit0=左侧，bit1=右侧；两侧锁定为 3
    uint32_t ready = 0;                // 0=未就绪, 1=就绪
    uint32_t fault = 0;                // 0=正常, 1=故障
    double position_error[2]{};        // 左右抓取位置误差 (m)
    double rotation_error[2]{};        // 左右抓取旋转误差 (rad)
    double angle = 0;                  // 轮盘当前转角 (rad)
    double displacement = 0;           // 轮盘当前推拉位移 (m)
    uint64_t ack = 0;                  // 命令确认序号
    GraspResult result = GraspResult::Ok;
};

// 数据链接抽象接口：臂IO + 抓取IO + 周期同步
class DataLink {
  public:
    virtual ~DataLink() = default;

    // 臂IO
    virtual double getJointPosition(Side side, int axis) const = 0;
    virtual double getJointVelocity(Side side, int axis) const = 0;
    virtual void setJointPositions(const std::array<double, 14> &q) = 0;
    virtual double jointVelLimit(Side side, int axis) const = 0;
    virtual bool isEnabled(Side side) const = 0;
    virtual void enable(Side side) = 0;   // 失败抛 std::runtime_error
    virtual void disable(Side side) = 0;  // 失败抛 std::runtime_error

    // 抓取IO
    virtual GraspState graspState() const = 0;
    virtual uint64_t sendGraspCommand(GraspCommand command) = 0; // 返回请求序号

    // 周期同步
    // Update the wheel reference alongside joint targets; simulations keep reading the physical wheel.
    virtual void setWheelReference(double /*angle*/, double /*displacement*/) {}

    virtual void waitTick() = 0; // 推进/等待一个控制周期（1 ms）

    // 控制用的单调时间源（秒）。仿真是仿真时间，真机是墙钟时间。
    // 轨迹插值、驻留以它为准；Servo 输入超时单独使用墙钟，监测真实输入流。
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

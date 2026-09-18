#pragma once
#include <array>
#include <cstdint>

namespace aviator {

enum class Side { Left = 0, Right = 1 };
enum class GraspCommand { Lock, Unlock, ResetFault };
enum class GraspResult { Ok, NotAligned, NotEnabled, Fault };

// 抓取状态:夹爪(或仿真 weld)的对齐/锁定/故障与命令确认。
// angle/displacement 是轮盘当前实测位形(接近时用于对准当前把手位置);
// 注意:轮盘的"目标"角/位移是 MoveWheel 的输入、由算法内部跟踪,不从硬件读回。
struct GraspState {
    double heartbeat = 0; // 后端存活时间戳(CLOCK_MONOTONIC 秒)
    uint32_t locked = 0, ready = 0, fault = 0;
    double position_error[2]{}, rotation_error[2]{};
    double angle = 0, displacement = 0; // 轮盘当前实测转角/推拉位移(rad / m)
    uint64_t ack = 0;
    GraspResult result = GraspResult::Ok;
};

// 算法唯一的数据出入口:臂 IO + 抓取 IO + 周期同步。
// 仿真与真机各自实现后端,算法只依赖本接口,因此算法源文件不引用任何后端私有类型。
class DataLink {
  public:
    virtual ~DataLink() = default;
    // —— 臂 IO ——
    virtual double getJointPosition(Side side, int axis) const = 0;
    virtual void setJointPositions(const std::array<double, 14> &q) = 0;
    virtual double jointVelLimit(Side side, int axis) const = 0;
    virtual bool isEnabled(Side side) const = 0;
    virtual void enable(Side side) = 0;   // 失败抛 std::runtime_error
    virtual void disable(Side side) = 0;  // 失败抛 std::runtime_error
    // —— 抓取 IO ——
    virtual GraspState graspState() const = 0;
    virtual uint64_t sendGraspCommand(GraspCommand command) = 0; // 返回请求序号
    // —— 周期同步 ——
    virtual void waitTick() = 0;
};

} // namespace aviator

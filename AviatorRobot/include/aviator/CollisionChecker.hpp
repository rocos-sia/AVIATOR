#pragma once
#include <array>

namespace aviator {

// 离线碰撞检查:给定双臂关节角与轮盘角度/位移,若穿透超过阈值则抛 std::runtime_error。
// 仿真实现基于 MuJoCo 前向动力学;Phase 2 替换为 Pinocchio。
class CollisionChecker {
  public:
    virtual ~CollisionChecker() = default;
    virtual void check(const std::array<double, 14> &q, double wheel_angle,
                       double wheel_displacement) = 0;
};

} // namespace aviator

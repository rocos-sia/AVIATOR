#pragma once
#include <array>

namespace aviator {

// 碰撞检测接口：给定双臂关节角与轮盘位形，检查碰撞。
class CollisionChecker {
  public:
    virtual ~CollisionChecker() = default;

    // 碰撞检查：若检测到碰撞则抛 std::runtime_error。
    // q: 14 轴关节角（左臂 0-6，右臂 7-13）
    // wheel_angle: 轮盘转角 (rad)
    // wheel_displacement: 轮盘推拉位移 (m)
    virtual void check(const std::array<double, 14> &q, double wheel_angle,
                       double wheel_displacement) = 0;
};

} // namespace aviator

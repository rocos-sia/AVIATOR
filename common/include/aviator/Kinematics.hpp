#pragma once
#include "aviator/DataLink.hpp"
#include "aviator/Pose.hpp"
#include <array>

namespace aviator {

// 双臂运动学接口 (FK/IK)
class Kinematics {
  public:
    virtual ~Kinematics() = default;

    // 逆运动学：给定目标位姿和种子，求解关节角
    // 返回 true 表示求解成功
    virtual bool solveIk(Side side, const std::array<double, 7> &seed,
                         const pinocchio::SE3 &target, std::array<double, 7> &out) = 0;

    // 正运动学：给定关节角，求解末端位姿
    virtual bool solveFk(Side side, const std::array<double, 7> &q, pinocchio::SE3 &out) = 0;

    // 关节限位 (来自 URDF)
    virtual double jointLower(Side side, int axis) const = 0;
    virtual double jointUpper(Side side, int axis) const = 0;
};

} // namespace aviator

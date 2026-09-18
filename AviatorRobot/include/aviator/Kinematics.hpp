#pragma once
#include "aviator/DataLink.hpp"
#include <array>
#include <kdl/frames.hpp>

namespace aviator {

// 双臂运动学(FK/IK),与数据源无关。当前实现为 TRAC-IK(复用后端提供的运动学组件)。
class Kinematics {
  public:
    virtual ~Kinematics() = default;
    virtual bool solveIk(Side side, const std::array<double, 7> &seed, const KDL::Frame &target,
                         std::array<double, 7> &out) = 0;
    virtual bool solveFk(Side side, const std::array<double, 7> &q, KDL::Frame &out) = 0;
    virtual double jointLower(Side side, int axis) const = 0; // 物理限位(来自 URDF)
    virtual double jointUpper(Side side, int axis) const = 0;
};

} // namespace aviator

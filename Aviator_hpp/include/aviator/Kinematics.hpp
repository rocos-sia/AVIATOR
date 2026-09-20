#pragma once

#include <array>
#include <memory>
#include <string>
#include <pinocchio/spatial/se3.hpp>

namespace aviator {

constexpr int kArmJoints = 7;

enum class Side { Left = 0, Right = 1 };

class Kinematics {
  public:
    virtual ~Kinematics() = default;

    // 逆运动学：给定目标位姿和种子关节角，求解关节角
    // target: 目标位姿 (相对于 aircraft 坐标系)
    // seed: 种子关节角 (7 个手臂关节)
    // out: 输出关节角 (7 个手臂关节)
    // 返回: 成功返回 true，失败返回 false
    virtual bool solveIk(Side side, const std::array<double, 7> &seed,
                         const pinocchio::SE3 &target, std::array<double, 7> &out) = 0;

    // 正运动学：给定关节角，求解末端位姿
    // q: 关节角 (7 个手臂关节)
    // out: 输出位姿 (相对于 aircraft 坐标系)
    // 返回: 成功返回 true，失败返回 false
    virtual bool solveFk(Side side, const std::array<double, 7> &q, pinocchio::SE3 &out) = 0;

    // 关节限位
    virtual double jointLower(Side side, int axis) const = 0;
    virtual double jointUpper(Side side, int axis) const = 0;
};

// 创建 TRAC-IK 运动学求解器
std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path,
                                                  double joint2_min, double joint2_max);

} // namespace aviator

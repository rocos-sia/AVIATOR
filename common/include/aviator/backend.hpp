#pragma once
#include "aviator/CollisionChecker.hpp"
#include "aviator/DataLink.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/Pose.hpp"
#include <memory>
#include <string>

namespace aviator {

// 抓取工具圆柱的期望尺寸，与 config/grasp.json 一致。
// 碰撞后端用它和碰撞 URDF 中实际的圆柱对比，防止两侧配置漂移。
struct GraspCylinder {
    double radius = 0.028; // m
    double length = 0.060; // m
};

// 抓取几何，全部来自 config/grasp.json，由算法层解析后传给后端，
// 避免后端各自重复读取配置导致漂移。
//
//   wheel_origin  轮盘坐标系在 aircraft 系下的位姿
//   handles[2]    handle site 相对轮盘坐标系的位姿（左/右）
//   tools[2]      左/右法兰 → 各自抓取圆柱中心的刚体变换
//   approach_dist 预接近距离（沿法兰 -Z 后退量），供真机做同样的接近动作
struct GraspGeometry {
    pinocchio::SE3 wheel_origin = pinocchio::SE3::Identity();
    pinocchio::SE3 handles[2]{pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
    std::array<pinocchio::SE3, 2> tools{{pinocchio::SE3::Identity(), pinocchio::SE3::Identity()}};
    double approach_distance = 0.06;
};

// Rokae hardware connection settings.
struct RokaeConfig {
    std::string left_ip;
    std::string right_ip;
    std::string left_local_ip;
    std::string right_local_ip;
    std::string grasp_mode = "open_loop";
    std::array<double, 7> joint_stiffness{{500, 500, 500, 500, 50, 50, 50}}; // Nm/rad
    // 关节跟踪容差等在真机上可另行收紧，这里只放连接与安全相关项。
};

// 珞石 xMateErPro 真机后端。由启用 AVIATOR_BUILD_XCORE_SDK 的构建提供。
// 臂基座 → 轮盘的安装变换由 URDF 的固定关节推算，因此只需传入 grasp.json 的几何。
std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const RokaeConfig &config,
                                            const GraspGeometry &geometry);

// PIN-IK / Pinocchio 运动学后端。joint2_min/max 为 J2 规划限位，弧度。
std::unique_ptr<Kinematics> makePinIkKinematics(const std::string &urdf_path, double joint2_min,
                                                 double joint2_max);

// Pinocchio + hpp-fcl/coal 碰撞后端。
// 碰撞 URDF 由 scripts/generate_aviator.py 导出（分片凸网格 + 圆柱工具 + SRDF 排除对）。
std::unique_ptr<CollisionChecker> makePinocchioCollisionChecker(const std::string &urdf_path,
                                                               const std::string &srdf_path,
                                                               const GraspCylinder &cylinder,
                                                               const std::array<pinocchio::SE3, 2> &tools);

} // namespace aviator

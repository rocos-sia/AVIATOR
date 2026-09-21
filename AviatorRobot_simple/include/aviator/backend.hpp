#pragma once
#include "aviator/CollisionChecker.hpp"
#include "aviator/DataLink.hpp"
#include "aviator/Kinematics.hpp"
#include <kdl/frames.hpp>
#include <memory>
#include <string>

// MuJoCo 类型前向声明，避免公共头文件暴露第三方头。
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

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
//   tool          法兰 → 抓取圆柱中心（连接杆 + 圆柱）的刚体变换
//   approach_dist 预接近距离（沿法兰 -Z 后退量），供真机做同样的接近动作
struct GraspGeometry {
    KDL::Frame wheel_origin;
    KDL::Frame handles[2];
    KDL::Frame tool;
    double approach_distance = 0.06;
};

// 后端构造上下文。
//
// 仿真需要 mjModel/mjData 句柄，真机不需要——两者通过同一个工厂创建，
// 由 config 的 backend 字段分发，因此这里把"可能为空的仿真句柄"显式建模。
struct BackendContext {
    mjModel *model = nullptr; // 仅 backend: mujoco 需要
    mjData *data = nullptr;   // 同上

    bool hasSimulation() const { return model != nullptr && data != nullptr; }
};

// 真机后端参数（backend: rokae 时从 config 的 rokae 段读取）。
struct RokaeConfig {
    std::string left_ip;
    std::string right_ip;
    std::string local_ip;
    std::string grasp_mode = "open_loop";
    std::array<double, 7> joint_stiffness{{500, 500, 500, 500, 50, 50, 50}}; // Nm/rad
    // 关节跟踪容差等在真机上可另行收紧，这里只放连接与安全相关项。
};

// —— 后端工厂 ——
// 算法层只通过这些工厂拿到抽象接口，各后端的私有类型（mujoco / rokae）只出现在
// 对应的实现文件里，因此算法源文件不引用任何后端头文件。

// 按配置创建数据链接。
//   backend: mujoco → MuJoCoDirectDataLink（要求 context.hasSimulation()）
//   backend: rokae  → RokaeDataLink（要求 AVIATOR_WITH_ROKAE=ON，否则抛错）
std::unique_ptr<DataLink> makeDataLink(const std::string &backend, const BackendContext &context,
                                       const std::string &urdf_path,
                                       const GraspGeometry &geometry,
                                       const RokaeConfig &rokae = {});

// MuJoCo 单进程后端：直接读写 mjModel/mjData，物理步进由 waitTick() 驱动。
std::unique_ptr<DataLink> makeMuJoCoDirectDataLink(mjModel *model, mjData *data,
                                                   const std::string &urdf_path);

// 珞石 xMateErPro 真机后端。仅当编译期定义了 AVIATOR_HAVE_ROKAE 时可用。
// 臂基座 → 轮盘的安装变换由 URDF 的固定关节推算，因此只需传入 grasp.json 的几何。
#ifdef AVIATOR_HAVE_ROKAE
std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const RokaeConfig &config,
                                            const GraspGeometry &geometry);
#endif

// TRAC-IK 运动学后端。joint2_min/max 为 J2（肘部）的规划限位，弧度。
std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path, double joint2_min,
                                                 double joint2_max);

// Pinocchio + hpp-fcl/coal 碰撞后端。
// 碰撞 URDF 由 scripts/generate_aviator.py 导出（分片凸网格 + 圆柱工具 + SRDF 排除对）。
std::unique_ptr<CollisionChecker> makePinocchioCollisionChecker(const std::string &urdf_path,
                                                               const std::string &srdf_path,
                                                               const GraspCylinder &cylinder);

} // namespace aviator

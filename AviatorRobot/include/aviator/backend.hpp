#pragma once
#include "aviator/CollisionChecker.hpp"
#include "aviator/DataLink.hpp"
#include "aviator/Kinematics.hpp"
#include <array>
#include <kdl/frames.hpp>
#include <memory>
#include <string>

namespace aviator {

// 任务几何:从 config 解析,传给碰撞后端做一致性校验(与碰撞模型对账)。
struct ModelGeometry {
    std::array<KDL::Frame, 2> handles;
    KDL::Frame tool;
    KDL::Frame wheel_origin;
    double cylinder_radius = 0, cylinder_length = 0;
    std::array<double, 14> home;
};

// 后端工厂。算法只通过这些工厂拿到抽象接口,不接触后端具体类型。
// 这些工厂是唯一允许包含后端头文件(mujoco / rocos_app / rocos_mujoco)的入口。
std::unique_ptr<DataLink> makeMuJoCoDataLink(const std::string &urdf_path, int bus);
// Rokae 真机后端仅在编译期检测到 xCoreSDK(定义 AVIATOR_HAVE_ROKAE)时可用。
#ifdef AVIATOR_HAVE_ROKAE
std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const std::string &left_ip,
                                            const std::string &right_ip,
                                            const std::string &local_ip);
#endif
std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path, double joint2_min,
                                                 double joint2_max);
std::unique_ptr<CollisionChecker> makePinocchioCollisionChecker(const std::string &urdf_path,
                                                                const ModelGeometry &geometry);

} // namespace aviator

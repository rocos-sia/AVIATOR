#include "aviator/CollisionChecker.hpp"
#include "aviator/backend.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <coal/shape/geometric_shapes.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace aviator {

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}
} // namespace

// Pinocchio 碰撞后端。
//
// 碰撞 URDF 里两侧法兰各带一个抓取圆柱（left/right_grasp_cylinder），它按设计会与
// 对应握持外壳的有意重叠，因此必须从碰撞对里剔除"恰好一侧是抓取圆柱"的所有组合。
// 这是原实现里由 MuJoCo 的 conaffinity 位掩码表达的语义（圆柱只与另一圆柱碰撞），
// 在 Pinocchio 侧删除所有“恰有一侧为抓取圆柱”的碰撞对，与原项目一致。
// 因此圆柱只与另一圆柱检查；机械臂、安装柱等其他几何仍按 SRDF 检查。
class PinocchioCollisionChecker final : public CollisionChecker {
  public:
    PinocchioCollisionChecker(const std::string &urdf_path, const std::string &srdf_path,
                              const GraspCylinder &expected) {
        const std::string package_dir = parentDir(urdf_path);

        pinocchio::urdf::buildModel(urdf_path, model_);
        require(model_.nq == 16, "Expected the AVIATOR 16-DOF collision model, got nq = " +
                                     std::to_string(model_.nq));
        data_ = pinocchio::Data(model_);

        pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_,
                                   package_dir);
        geom_model_.addAllCollisionPairs();

        // SRDF 已含父子 FILTERPARENT 排除 + 轮盘轴承 / 腕部壳体排除
        pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_path);

        maskGraspCylinders(expected);
        bindJointIndices();

        geom_data_ = pinocchio::GeometryData(geom_model_);
    }

    void check(const std::array<double, 14> &q, double wheel_angle,
               double wheel_displacement) override {
        pinocchio::Model::ConfigVectorType configuration = pinocchio::neutral(model_);
        for (int i = 0; i < 14; ++i)
            configuration[joint_ids_[i]] = q[i];
        configuration[wheel_ids_[0]] = wheel_angle;
        configuration[wheel_ids_[1]] = wheel_displacement;

        pinocchio::computeCollisions(model_, data_, geom_model_, geom_data_, configuration, false);

        for (pinocchio::GeomIndex k = 0; k < geom_model_.collisionPairs.size(); ++k) {
            if (geom_data_.collisionResults[k].isCollision()) {
                const auto &pair = geom_model_.collisionPairs[k];
                throw std::runtime_error("Planned collision: " +
                                         geom_model_.geometryObjects[pair.first].name + " / " +
                                         geom_model_.geometryObjects[pair.second].name);
            }
        }
    }

  private:
    static std::string parentDir(const std::string &path) {
        const auto slash = path.find_last_of('/');
        if (slash == std::string::npos)
            return ".";
        return slash == 0 ? "/" : path.substr(0, slash);
    }

    static std::string withExtension(const std::string &path, const std::string &extension) {
        const auto slash = path.find_last_of('/');
        const auto dot = path.find_last_of('.');
        if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
            return path + extension;
        return path.substr(0, dot) + extension;
    }

    // 校验两个抓取圆柱的尺寸与 grasp.json 一致，并删除"恰好一侧是圆柱"的碰撞对。
    void maskGraspCylinders(const GraspCylinder &expected) {
        std::vector<char> is_cylinder(geom_model_.ngeoms, 0);
        int cylinder_count = 0;
        for (pinocchio::GeomIndex i = 0; i < geom_model_.ngeoms; ++i) {
            const auto *cyl =
                dynamic_cast<const coal::Cylinder *>(geom_model_.geometryObjects[i].geometry.get());
            if (!cyl)
                continue;
            is_cylinder[i] = 1;
            ++cylinder_count;
            require(std::abs(cyl->radius - expected.radius) < 1e-9 &&
                        std::abs(2 * cyl->halfLength - expected.length) < 1e-9,
                    "Grasp cylinder in " + geom_model_.geometryObjects[i].name +
                        " differs from grasp.json; regenerate assets");
        }
        require(cylinder_count == 2,
                "Expected exactly two grasp cylinders, found " + std::to_string(cylinder_count));

        std::vector<pinocchio::CollisionPair> to_remove;
        for (const auto &pair : geom_model_.collisionPairs)
            if (is_cylinder[pair.first] != is_cylinder[pair.second])
                to_remove.push_back(pair);
        for (const auto &pair : to_remove)
            geom_model_.removeCollisionPair(pair);
    }

    void bindJointIndices() {
        // 按名称查找，不依赖 URDF 解析顺序
        for (int i = 0; i < 14; ++i) {
            const std::string name = std::string("AR5-5_07") + (i < 7 ? "L" : "R") +
                                     "-W4C4A2_joint_" + std::to_string(i % 7 + 1);
            require(model_.existJointName(name), "Missing arm joint " + name);
            joint_ids_[i] = static_cast<int>(model_.joints[model_.getJointId(name)].idx_q());
        }
        for (int i = 0; i < 2; ++i) {
            const std::string name = i == 0 ? "roll_input_joint" : "pitch_input_joint";
            require(model_.existJointName(name), "Missing passive wheel joint " + name);
            wheel_ids_[i] = static_cast<int>(model_.joints[model_.getJointId(name)].idx_q());
        }
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;
    int joint_ids_[14]{};
    int wheel_ids_[2]{};
};

std::unique_ptr<CollisionChecker> makePinocchioCollisionChecker(const std::string &urdf_path,
                                                               const std::string &srdf_path,
                                                               const GraspCylinder &cylinder) {
    return std::make_unique<PinocchioCollisionChecker>(urdf_path, srdf_path, cylinder);
}

} // namespace aviator

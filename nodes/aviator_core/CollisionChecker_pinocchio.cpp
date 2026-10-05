#include "aviator/CollisionChecker.hpp"
#include "aviator/backend.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <array>
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

// Collision geometry comes from the same complete URDF as kinematics.
class PinocchioCollisionChecker final : public CollisionChecker {
  public:
    explicit PinocchioCollisionChecker(const std::string &urdf_path) {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_,
                                   parentDir(urdf_path));
        geom_model_.addAllCollisionPairs();
        // Exclude rigidly connected geometry and directly connected URDF links.
        // No exclusions from the obsolete cylinder model are carried forward.
        std::vector<pinocchio::CollisionPair> excluded;
        for (const auto &pair : geom_model_.collisionPairs) {
            const auto &a = geom_model_.geometryObjects[pair.first];
            const auto &b = geom_model_.geometryObjects[pair.second];
            if (a.parentJoint == b.parentJoint || parentLink(a.parentFrame) == b.parentFrame ||
                parentLink(b.parentFrame) == a.parentFrame)
                excluded.push_back(pair);
        }
        for (const auto &pair : excluded) geom_model_.removeCollisionPair(pair);
        bindJointIndices();
        geom_data_ = pinocchio::GeometryData(geom_model_);
    }

    void check(const std::array<double, 14> &q, double wheel_angle,
               double wheel_displacement) override {
        // The interface supplies arms/wheel only; fingers stay at URDF zero pose.
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

    pinocchio::FrameIndex parentLink(pinocchio::FrameIndex link) const {
        auto parent = model_.frames[link].parentFrame;
        while (parent != 0 && model_.frames[parent].type != pinocchio::BODY)
            parent = model_.frames[parent].parentFrame;
        return parent;
    }

    int configurationIndex(const std::string &name) const {
        require(model_.existJointName(name), "Missing collision joint " + name);
        const auto &joint = model_.joints[model_.getJointId(name)];
        require(joint.nq() == 1, "Expected one configuration value for " + name);
        return static_cast<int>(joint.idx_q());
    }

    void bindJointIndices() {
        // 按名称查找，不依赖 URDF 解析顺序
        for (int i = 0; i < 14; ++i) {
            const std::string name = std::string("AR5-5_07") + (i < 7 ? "L" : "R") +
                                     "-W4C4A2_joint_" + std::to_string(i % 7 + 1);
            joint_ids_[i] = configurationIndex(name);
        }
        for (int i = 0; i < 2; ++i) {
            const std::string name = i == 0 ? "roll_input_joint" : "pitch_input_joint";
            wheel_ids_[i] = configurationIndex(name);
        }
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;
    int joint_ids_[14]{};
    int wheel_ids_[2]{};
};

std::unique_ptr<CollisionChecker> makePinocchioCollisionChecker(const std::string &urdf_path) {
    return std::make_unique<PinocchioCollisionChecker>(urdf_path);
}

} // namespace aviator

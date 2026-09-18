#include "aviator/backend.hpp"

#include <rocos_app/kinematics.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <stdexcept>
#include <string>

namespace aviator {

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}
// 从 URDF 运动链提取 7 个关节的物理限位(基座→末端顺序)。
void chainJointLimits(const KDL::Chain &chain, const urdf::ModelInterfaceSharedPtr &model,
                      std::array<double, 7> &lower, std::array<double, 7> &upper) {
    int idx = 0;
    for (unsigned s = 0; s < chain.getNrOfSegments(); ++s) {
        const auto &joint = chain.getSegment(s).getJoint();
        if (joint.getType() == KDL::Joint::None)
            continue;
        require(idx < 7, "More than 7 revolute joints in chain");
        const auto uj = model->getJoint(joint.getName());
        require(uj && uj->limits, "URDF joint without limits: " + joint.getName());
        lower[idx] = uj->limits->lower;
        upper[idx] = uj->limits->upper;
        ++idx;
    }
    require(idx == 7, "Expected 7 revolute joints, got " + std::to_string(idx));
}
} // namespace

// TRAC-IK 后端:把 rocos::Kinematics 从 rocos::Robot 中剥离,独立构链。
class TracIkKinematics final : public Kinematics {
  public:
    TracIkKinematics(const std::string &urdf, double joint2_min, double joint2_max) {
        auto model = urdf::parseURDFFile(urdf);
        require(model != nullptr, "Cannot parse URDF " + urdf);
        KDL::Tree tree;
        require(kdl_parser::treeFromFile(urdf, tree), "Cannot build KDL tree from " + urdf);
        for (int side = 0; side < 2; ++side) {
            const std::string tip =
                std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2_flan_link";
            require(solvers_[side].setChain(tree, "aircraft", tip), "Cannot extract chain " + tip);
            chainJointLimits(solvers_[side].getChain(), model, lower_[side], upper_[side]);
            KDL::JntArray lo(7), hi(7);
            for (int axis = 0; axis < 7; ++axis) {
                lo(axis) = lower_[side][axis];
                hi(axis) = upper_[side][axis];
            }
            // 用任务限位重建 TRAC-IK,保留 URDF 物理限位;向内余量容纳边界处的伺服跟踪误差。
            lo(1) = joint2_min;
            hi(1) = joint2_max;
            solvers_[side].setPosLimits(lo, hi);
            solvers_[side].Initialize(true);
        }
    }

    bool solveIk(Side side, const std::array<double, 7> &seed, const KDL::Frame &target,
                 std::array<double, 7> &out) override {
        KDL::JntArray init(7), result(7);
        for (int axis = 0; axis < 7; ++axis)
            init(axis) = seed[axis];
        if (solvers_[static_cast<int>(side)].CartToJnt(init, target, result) < 0)
            return false;
        for (int axis = 0; axis < 7; ++axis)
            out[axis] = result(axis);
        return true;
    }
    bool solveFk(Side side, const std::array<double, 7> &q, KDL::Frame &out) override {
        KDL::JntArray jq(7);
        for (int axis = 0; axis < 7; ++axis)
            jq(axis) = q[axis];
        return solvers_[static_cast<int>(side)].JntToCart(jq, out) >= 0;
    }
    double jointLower(Side side, int axis) const override {
        return lower_[static_cast<int>(side)][axis];
    }
    double jointUpper(Side side, int axis) const override {
        return upper_[static_cast<int>(side)][axis];
    }

  private:
    rocos::Kinematics solvers_[2];
    std::array<double, 7> lower_[2], upper_[2];
};

std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path, double joint2_min,
                                                 double joint2_max) {
    return std::make_unique<TracIkKinematics>(urdf_path, joint2_min, joint2_max);
}

} // namespace aviator

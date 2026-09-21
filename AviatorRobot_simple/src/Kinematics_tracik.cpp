#include "aviator/Kinematics.hpp"
#include "aviator/backend.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace aviator {

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }

void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}

constexpr double kTwoPi = 2.0 * M_PI;

// 把角度平移到 [lower, upper] 所在的那个等价区间（按 2π 的整数倍）。
//
// 需要这一步是因为 TRAC-IK 的 normalizeAngle(val, min, max) 在下溢时写成
// `min - fmod(min - val, 2π) + 2π`：当 val 只比 min 小一点点时，结果约为 min + 2π，
// 即整整超出一圈。J2 的规划区间只有 9°（85.5°–94.5°），远小于一圈，于是解恰好落在
// 下界附近时就会得到 445.5° 这种值，被后续的肘部检查判为越界。
// 这里取离区间中心最近的等价角：真在区间内的解会被还原，本就超限的解保持超限，
// 仍会被上层拒绝，不会掩盖真正的不可达情况。
double wrapIntoRange(double value, double lower, double upper) {
    if (!std::isfinite(value))
        return value;
    const double center = 0.5 * (lower + upper);
    return value - kTwoPi * std::round((value - center) / kTwoPi);
}

// 从 URDF 运动链提取 7 个关节的物理限位（基座→末端顺序），
// 与原后端走同一份模型定义，保证限位来源一致。
void chainJointLimits(const KDL::Chain &chain, const urdf::ModelInterfaceSharedPtr &model,
                      std::array<double, 7> &lower, std::array<double, 7> &upper) {
    int idx = 0;
    for (unsigned s = 0; s < chain.getNrOfSegments(); ++s) {
        const auto &joint = chain.getSegment(s).getJoint();
        if (joint.getType() == KDL::Joint::None)
            continue;
        require(idx < 7, "More than 7 revolute joints in chain");
        const auto urdf_joint = model->getJoint(joint.getName());
        require(urdf_joint && urdf_joint->limits, "URDF joint without limits: " + joint.getName());
        lower[idx] = urdf_joint->limits->lower;
        upper[idx] = urdf_joint->limits->upper;
        ++idx;
    }
    require(idx == 7, "Expected 7 revolute joints, got " + std::to_string(idx));
}

} // namespace

// TRAC-IK 运动学后端。
//
// 与原实现保持同一组求解参数：
//   * 用 KDL::Chain + 限位构造（而不是 URDF 文件构造），以便把 J2 的任务限位
//     直接作为求解边界传入；
//   * eps = 7e-7、SolveType::Distance（"靠近种子解"），与原 rocos::Kinematics
//     的 Initialize(prefer_near_seed = true) 一致。
// 关节的物理限位仍来自 URDF，只有 J2 被收紧到任务限位（85.5°–94.5°）。
class TracIkKinematics final : public Kinematics {
  public:
    TracIkKinematics(const std::string &urdf_path, double joint2_min, double joint2_max) {
        const auto model = urdf::parseURDFFile(urdf_path);
        require(model != nullptr, "Cannot parse URDF " + urdf_path);

        KDL::Tree tree;
        require(kdl_parser::treeFromFile(urdf_path, tree),
                "Cannot build KDL tree from " + urdf_path);

        for (int side = 0; side < 2; ++side) {
            const std::string tip = std::string("AR5-5_07") + (side == 0 ? "L" : "R") +
                                    "-W4C4A2_flan_link";
            require(tree.getChain("aircraft", tip, chains_[side]),
                    "Cannot extract chain " + tip);
            require(chains_[side].getNrOfJoints() == 7, "Expected a 7-DOF chain for " + tip);

            chainJointLimits(chains_[side], model, lower_[side], upper_[side]);

            KDL::JntArray lo(7), hi(7);
            for (int axis = 0; axis < 7; ++axis) {
                lo(axis) = lower_[side][axis];
                hi(axis) = upper_[side][axis];
            }
            // J2 收紧到任务限位；其余轴保留 URDF 物理限位
            lo(1) = joint2_min;
            hi(1) = joint2_max;

            fk_[side] = std::make_unique<KDL::ChainFkSolverPos_recursive>(chains_[side]);
            ik_[side] = std::make_unique<TRAC_IK::TRAC_IK>(chains_[side], lo, hi, 0.005, 7e-7,
                                                           TRAC_IK::Distance);
        }
    }

    bool solveIk(Side side, const std::array<double, 7> &seed, const KDL::Frame &target,
                 std::array<double, 7> &out) override {
        const int index = static_cast<int>(side);

        KDL::JntArray initial(7), result(7);
        for (int axis = 0; axis < 7; ++axis)
            initial(axis) = seed[axis];

        if (ik_[index]->CartToJnt(initial, target, result) < 0)
            return false;

        // 折回任务区间后再交给上层校验，避免 TRAC-IK 的整圈回绕值被当成越界。
        for (int axis = 0; axis < 7; ++axis)
            out[axis] = wrapIntoRange(result(axis), lower_[index][axis], upper_[index][axis]);
        return true;
    }

    bool solveFk(Side side, const std::array<double, 7> &q, KDL::Frame &out) override {
        KDL::JntArray joint(7);
        for (int axis = 0; axis < 7; ++axis)
            joint(axis) = q[axis];
        return fk_[static_cast<int>(side)]->JntToCart(joint, out) >= 0;
    }

    double jointLower(Side side, int axis) const override {
        return lower_[static_cast<int>(side)][axis];
    }

    double jointUpper(Side side, int axis) const override {
        return upper_[static_cast<int>(side)][axis];
    }

  private:
    KDL::Chain chains_[2];
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_[2];
    std::unique_ptr<TRAC_IK::TRAC_IK> ik_[2];
    std::array<double, 7> lower_[2]{}, upper_[2]{};
};

std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path, double joint2_min,
                                                 double joint2_max) {
    return std::make_unique<TracIkKinematics>(urdf_path, joint2_min, joint2_max);
}

} // namespace aviator

#include "aviator/Kinematics.hpp"
#include "aviator/backend.hpp"

#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace aviator {
namespace {
void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}
} // namespace

// Each arm owns a chain, FK workspace and solver. Calls are serialized by Aviator.
class PinIkKinematics final : public Kinematics {
  public:
    PinIkKinematics(const std::string &urdf_path, double joint2_min, double joint2_max) {
        std::ifstream file(urdf_path);
        require(bool(file), "Cannot read URDF " + urdf_path);
        const std::string xml((std::istreambuf_iterator<char>(file)), {});
        for (int side = 0; side < 2; ++side) {
            const std::string prefix = std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2";
            PIN_IK::loadURDFModel(xml, "aircraft", prefix + "_flan_link", model_[side],
                                 lower_[side], upper_[side], tip_[side]);
            const auto &model = model_[side];
            require(model.nq == 7 && model.nv == 7 && model.njoints == 8,
                    "Expected seven bounded single-axis joints for " + prefix);
            for (int axis = 0; axis < 7; ++axis) {
                require(model.names[axis + 1] == prefix + "_joint_" + std::to_string(axis + 1) &&
                        model.joints[axis + 1].idx_q() == axis && model.joints[axis + 1].idx_v() == axis,
                        "Unexpected joint order for " + prefix);
            }
            require(std::isfinite(joint2_min) && std::isfinite(joint2_max) &&
                    joint2_min < joint2_max && joint2_min >= lower_[side][1] &&
                    joint2_max <= upper_[side][1], "Invalid J2 planning limits");
            ik_lower_[side] = lower_[side];
            ik_upper_[side] = upper_[side];
            ik_lower_[side][1] = joint2_min;
            ik_upper_[side][1] = joint2_max;
            data_[side] = std::make_unique<pinocchio::Data>(model);
            ik_[side] = std::make_unique<PIN_IK::PIN_IK>(model, ik_lower_[side], ik_upper_[side],
                                                       tip_[side], 0.005, 7e-7, PIN_IK::Distance);
        }
    }

    bool solveIk(Side side, const std::array<double, 7> &seed, const pinocchio::SE3 &target,
                 std::array<double, 7> &out) override {
        const int i = static_cast<int>(side);
        const Eigen::Map<const Eigen::Matrix<double, 7, 1>> initial(seed.data());
        if (!initial.allFinite() || !target.translation().allFinite() || !target.rotation().allFinite())
            return false;
        Eigen::VectorXd result;
        if (ik_[i]->CartToJnt(initial, target, result) < 0 || result.size() != 7 || !result.allFinite())
            return false;
        if ((result.array() < ik_lower_[i].array()).any() ||
            (result.array() > ik_upper_[i].array()).any()) return false;

        std::array<double, 7> candidate;
        for (int axis = 0; axis < 7; ++axis) candidate[axis] = result[axis];
        pinocchio::SE3 actual;
        if (!solveFk(side, candidate, actual)) return false;
        if ((actual.translation() - target.translation()).norm() > 1e-6 ||
            rotationError(actual, target) > 1e-6) return false;
        out = candidate;
        return true;
    }

    bool solveFk(Side side, const std::array<double, 7> &q, pinocchio::SE3 &out) override {
        const int i = static_cast<int>(side);
        const Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(q.data());
        if (!angles.allFinite()) return false;
        pinocchio::forwardKinematics(model_[i], *data_[i], angles);
        pinocchio::updateFramePlacements(model_[i], *data_[i]);
        out = data_[i]->oMf[tip_[i]];
        return true;
    }

    // Physical limits remain distinct from the task's tighter J2 planning limits.
    double jointLower(Side side, int axis) const override { return lower_[static_cast<int>(side)][axis]; }
    double jointUpper(Side side, int axis) const override { return upper_[static_cast<int>(side)][axis]; }

  private:
    pinocchio::Model model_[2];
    std::unique_ptr<pinocchio::Data> data_[2];
    pinocchio::FrameIndex tip_[2]{};
    Eigen::VectorXd lower_[2], upper_[2], ik_lower_[2], ik_upper_[2];
    std::unique_ptr<PIN_IK::PIN_IK> ik_[2];
};

std::unique_ptr<Kinematics> makePinIkKinematics(const std::string &urdf_path, double joint2_min,
                                              double joint2_max) {
    return std::make_unique<PinIkKinematics>(urdf_path, joint2_min, joint2_max);
}
} // namespace aviator

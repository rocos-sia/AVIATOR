#include "aviator/Kinematics.hpp"
#include "aviator/backend.hpp"

#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/QR>
#include <Eigen/Cholesky>
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

    bool jointDerivatives(Side side, const std::array<double, 7> &q,
                          const Eigen::Matrix<double, 6, 1> &twist,
                          const Eigen::Matrix<double, 6, 1> &acceleration,
                          std::array<double, 7> &dq, std::array<double, 7> &ddq) override {
        const int i = static_cast<int>(side);
        const Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(q.data());
        auto &data = *data_[i];
        Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero(), dJ = J;
        pinocchio::computeJointJacobians(model_[i], data, angles);
        pinocchio::getFrameJacobian(model_[i], data, tip_[i], pinocchio::LOCAL_WORLD_ALIGNED, J);
        // J2 has a narrow task envelope. Penalize its redundant motion instead of
        // letting an unconstrained minimum-norm solution drift toward the limit.
        constexpr double elbow_weight = .01;
        J.col(1) *= elbow_weight;
        auto inverse = J.completeOrthogonalDecomposition();
        const Eigen::Matrix<double, 7, 1> scaled_velocity = inverse.solve(twist);
        Eigen::Matrix<double, 7, 1> velocity = scaled_velocity;
        velocity[1] *= elbow_weight;
        pinocchio::computeJointJacobiansTimeVariation(model_[i], data, angles, velocity);
        pinocchio::getFrameJacobianTimeVariation(model_[i], data, tip_[i], pinocchio::LOCAL_WORLD_ALIGNED, dJ);
        dJ.col(1) *= elbow_weight;
        // Include the derivative of the minimum-norm inverse (redundant seventh joint).
        const Eigen::Matrix<double, 6, 1> lambda = (J * J.transpose()).ldlt().solve(twist);
        const Eigen::Matrix<double, 7, 1> redundant = dJ.transpose() * lambda;
        const Eigen::Matrix<double, 7, 1> scaled_accel = inverse.solve(acceleration - dJ * scaled_velocity) +
                                                      redundant - inverse.solve(J * redundant);
        Eigen::Matrix<double, 7, 1> accel = scaled_accel;
        accel[1] *= elbow_weight;
        if (!velocity.allFinite() || !accel.allFinite() ||
            (J * scaled_velocity - twist).norm() > 1e-7 ||
            (J * scaled_accel + dJ * scaled_velocity - acceleration).norm() > 1e-7) return false;
        std::copy_n(velocity.data(), 7, dq.begin());
        std::copy_n(accel.data(), 7, ddq.begin());
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

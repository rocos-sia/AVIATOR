#pragma once
#include "aviator/Kinematics.hpp"
#include <functional>
#include <ruckig/ruckig.hpp>

namespace aviator {

// One persistent wheel trajectory; both arms share its angle and translation.
// Planning runs outside the device's RT callback. Each block inherits q/dq/ddq.
class ServoPlanner {
  public:
    using Target = std::function<pinocchio::SE3(int, double, double)>;
    using Check = std::function<void(const JointFrame&)>;
    ServoPlanner(Kinematics& kinematics, Target target, Check check,
                 const pinocchio::SE3& wheel_origin, const JointFrame& start, double period,
                 std::array<double, 2> speed, std::array<double, 2> acceleration,
                 std::array<double, 2> jerk);
    std::vector<JointFrame> advance(double angle, double displacement, double speed_ratio,
                                    bool stop);
    bool stopped() const;

  private:
    JointFrame endpoint(const std::array<double, 2>& p, const std::array<double, 2>& v,
                        const std::array<double, 2>& a);
    Kinematics& kinematics_;
    Target target_;
    Check check_;
    Eigen::Vector3d axis_, center_;
    JointFrame last_;
    double period_;
    std::array<double, 2> speed_;
    ruckig::Ruckig<2> planner_;
    ruckig::InputParameter<2> input_;
};
} // namespace aviator

#pragma once
#include "aviator/Kinematics.hpp"
#include <functional>
#include <ruckig/ruckig.hpp>
#include <limits>

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
                 std::array<double, 2> jerk,
                 double joint_speed_limit = std::numeric_limits<double>::max());
    std::vector<JointFrame> advance(double angle, double displacement, double speed_ratio,
                                    bool stop);
    bool stopped() const;

  private:
    JointFrame endpoint(const std::array<double, 2>& p, const std::array<double, 2>& v,
                        const std::array<double, 2>& a, bool takeover = false);
    std::vector<JointFrame> align(double angle, double displacement, double speed_ratio, bool stop);
    void checkGeometry(const JointFrame&, double position_tolerance, double rotation_tolerance);
    Kinematics& kinematics_;
    Target target_;
    Check check_;
    Eigen::Vector3d axis_, center_;
    JointFrame last_;
    double period_;
    std::array<double, 2> speed_;
    ruckig::Ruckig<2> planner_;
    ruckig::InputParameter<2> input_;
    double takeover_speed_;
    bool aligning_ = false;
    bool alignment_goal_valid_ = false;
    std::array<double, 2> alignment_goal_{};
    ruckig::Ruckig<14> alignment_planner_;
    ruckig::InputParameter<14> alignment_input_;
};
} // namespace aviator

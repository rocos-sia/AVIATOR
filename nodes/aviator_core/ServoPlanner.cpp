#include "ServoPlanner.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aviator {
namespace {
void require(bool ok, const std::string& message) {
    if (!ok)
        throw std::runtime_error(message);
}
} // namespace
ServoPlanner::ServoPlanner(Kinematics& kinematics, Target target, Check check,
                           const pinocchio::SE3& wheel_origin, const JointFrame& start,
                           double period, std::array<double, 2> speed,
                           std::array<double, 2> acceleration, std::array<double, 2> jerk)
    : kinematics_(kinematics), target_(std::move(target)), check_(std::move(check)),
      axis_(wheel_origin.rotation() * Eigen::Vector3d::UnitZ()),
      center_(wheel_origin.translation()), last_(start), period_(period), speed_(speed),
      planner_(period) {
    // Preserve the sub-micrometre residual of the accepted starting IK solution.
    // Correcting that residual in the first 20 ms would create a jerk spike.
    // This uses the last COMMAND's FK, never measured TCP/grasp feedback.
    std::array<pinocchio::SE3, 2> offsets;
    for (int side = 0; side < 2; ++side) {
        std::array<double, 7> q;
        std::copy_n(start.q.begin() + 7 * side, 7, q.begin());
        pinocchio::SE3 actual;
        require(kinematics_.solveFk(static_cast<Side>(side), q, actual), "Servo initial FK failed");
        const auto desired = target_(side, start.angle, start.displacement);
        require((actual.translation() - desired.translation()).norm() <= 2e-6 &&
                    rotationError(actual, desired) <= 2e-6,
                "Servo initial command is inconsistent with wheel reference");
        offsets[side] = desired.inverse() * actual;
    }
    const auto original_target = target_;
    target_ = [original_target, offsets](int side, double angle, double displacement) {
        return original_target(side, angle, displacement) * offsets[side];
    };
    input_.current_position = {start.angle, start.displacement};
    input_.current_velocity = {0, 0};
    input_.current_acceleration = {0, 0};
    input_.target_velocity = {0, 0};
    input_.target_acceleration = {0, 0};
    input_.max_velocity = speed;
    input_.max_acceleration = acceleration;
    input_.max_jerk = jerk;
    input_.synchronization = ruckig::Synchronization::Time;
}

JointFrame ServoPlanner::endpoint(const std::array<double, 2>& p, const std::array<double, 2>& v,
                                  const std::array<double, 2>& a) {
    JointFrame frame;
    frame.angle = p[0];
    frame.displacement = p[1];
    for (int side = 0; side < 2; ++side) {
        const auto arm = static_cast<Side>(side);
        const auto desired = target_(side, p[0], p[1]);
        std::array<double, 7> seed{}, q{}, dq{}, ddq{};
        for (int j = 0; j < 7; ++j) {
            int i = side * 7 + j;
            seed[j] = last_.q[i] + period_ * last_.dq[i] + .5 * period_ * period_ * last_.ddq[i];
        }
        // Start Newton correction from the extrapolated continuous branch. A global IK
        // restart can change the redundant posture and inject high-frequency joint motion.
        q = seed;
        bool solved = false;
        for (int iteration = 0; iteration < 8; ++iteration) {
            pinocchio::SE3 actual;
            require(kinematics_.solveFk(arm, q, actual), "Servo FK failed");
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = desired.translation() - actual.translation();
            Eigen::AngleAxisd rotation(desired.rotation() * actual.rotation().transpose());
            error.tail<3>() = rotation.axis() * rotation.angle();
            if (error.norm() < 1e-12) {
                solved = true;
                break;
            }
            require(kinematics_.jointDerivatives(arm, q, error, Eigen::Matrix<double, 6, 1>::Zero(),
                                                 dq, ddq),
                    "Servo differential IK singular");
            for (int j = 0; j < 7; ++j)
                q[j] += dq[j];
        }
        require(solved, "Servo local IK failed to converge");
        // Flange velocity = angular velocity cross radius + axial translation.
        const Eigen::Vector3d radial = desired.translation() - center_;
        Eigen::Matrix<double, 6, 1> twist, accel;
        twist.head<3>() = axis_.cross(radial) * v[0] + axis_ * v[1];
        twist.tail<3>() = axis_ * v[0];
        accel.head<3>() = axis_.cross(radial) * a[0] +
                          axis_.cross(axis_.cross(radial)) * v[0] * v[0] + axis_ * a[1];
        accel.tail<3>() = axis_ * a[0];
        require(kinematics_.jointDerivatives(arm, q, twist, accel, dq, ddq),
                "Servo differential IK singular");
        for (int j = 0; j < 7; ++j) {
            int i = side * 7 + j;
            frame.q[i] = q[j];
            frame.dq[i] = dq[j];
            frame.ddq[i] = ddq[j];
        }
    }
    return frame;
}

std::vector<JointFrame> ServoPlanner::advance(double angle, double displacement, double ratio,
                                              bool stop) {
    input_.control_interface =
        stop ? ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    input_.target_position = {angle, displacement};
    // Lowering the speed limit does not reset the current velocity or acceleration.
    for (int i = 0; i < 2; ++i)
        input_.max_velocity[i] = speed_[i] * ratio;
    ruckig::Trajectory<2> trajectory;
    auto result = planner_.calculate(input_, trajectory);
    require(result >= 0, "Servo Ruckig failed: " + std::to_string(int(result)));
    std::array<double, 2> p{}, v{}, a{};
    trajectory.at_time(period_, p, v, a);
    JointFrame end = endpoint(p, v, a);
    // Quintic Hermite interpolation matches position, velocity and acceleration at
    // both ends. Unlike the previous easing, derivatives are NOT reset per block.
    std::array<std::array<double, 6>, 14> coeff{};
    for (int j = 0; j < 14; ++j) {
        auto& c = coeff[j];
        c[0] = last_.q[j];
        c[1] = last_.dq[j] * period_;
        c[2] = .5 * last_.ddq[j] * period_ * period_;
        double P = end.q[j] - c[0] - c[1] - c[2];
        double V = end.dq[j] * period_ - c[1] - 2 * c[2];
        double A = end.ddq[j] * period_ * period_ - 2 * c[2];
        c[3] = 10 * P - 4 * V + .5 * A;
        c[4] = -15 * P + 7 * V - A;
        c[5] = 6 * P - 3 * V + .5 * A;
    }
    const size_t ticks = std::llround(period_ / .001);
    std::vector<JointFrame> frames;
    frames.reserve(ticks + 1);
    for (size_t k = 0; k <= ticks; ++k) {
        double u = double(k) / ticks;
        JointFrame f;
        trajectory.at_time(k * .001, p, v, a);
        f.angle = p[0];
        f.displacement = p[1];
        for (int j = 0; j < 14; ++j) {
            const auto& c = coeff[j];
            f.q[j] = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
            f.dq[j] =
                (c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) / period_;
            f.ddq[j] =
                (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (period_ * period_);
            require(std::isfinite(f.q[j]) && std::isfinite(f.dq[j]) && std::isfinite(f.ddq[j]),
                    "Nonfinite Servo joint command");
        }
        check_(f);
        // Planned geometry validation, not a measured grasp-error gate.
        for (int side = 0; side < 2; ++side) {
            std::array<double, 7> q;
            std::copy_n(f.q.begin() + 7 * side, 7, q.begin());
            pinocchio::SE3 actual;
            require(kinematics_.solveFk(static_cast<Side>(side), q, actual), "Servo FK failed");
            auto desired = target_(side, f.angle, f.displacement);
            require((actual.translation() - desired.translation()).norm() <= 1e-5 &&
                        rotationError(actual, desired) <= 1e-4,
                    "Servo interpolated wheel geometry error");
        }
        frames.push_back(f);
    }
    frames.front() = last_;
    frames.back() = end;
    last_ = end;
    trajectory.at_time(period_, input_.current_position, input_.current_velocity,
                       input_.current_acceleration);
    return frames;
}
bool ServoPlanner::stopped() const {
    for (int i = 0; i < 2; ++i)
        if (std::abs(input_.current_velocity[i]) > 1e-10 ||
            std::abs(input_.current_acceleration[i]) > 1e-10)
            return false;
    return true;
}
} // namespace aviator

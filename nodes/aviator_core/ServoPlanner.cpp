#include "ServoPlanner.hpp"
#include "Logger.hpp"
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
                           std::array<double, 2> acceleration, std::array<double, 2> jerk,
                           double joint_speed_limit)
    : kinematics_(kinematics), target_(std::move(target)), check_(std::move(check)),
      axis_(wheel_origin.rotation() * Eigen::Vector3d::UnitZ()),
      center_(wheel_origin.translation()), last_(start), period_(period), speed_(speed),
      planner_(period), takeover_speed_(std::min(speed[0], joint_speed_limit)), alignment_planner_(period) {
    require(std::isfinite(joint_speed_limit) && joint_speed_limit > 0,
            "Invalid Servo joint speed limit");
    for (const auto& limits : {speed, acceleration, jerk})
        for (double value : limits)
            require(std::isfinite(value) && value > 0, "Invalid Servo response limits");
    check_(start);
    // Wheel metadata can be stale after compliant following. It is never an
    // offset for the absolute destination, nor a reason to reject the hold.
    for (int side = 0; side < 2; ++side) {
        std::array<double, 7> q;
        std::copy_n(start.q.begin() + 7 * side, 7, q.begin());
        pinocchio::SE3 actual;
        require(kinematics_.solveFk(Side(side), q, actual), "Servo hold FK failed");
        const auto desired = target_(side, start.angle, start.displacement);
        aligning_ = aligning_ || (actual.translation() - desired.translation()).norm() > 1e-9 ||
                    rotationError(actual, desired) > 1e-9;
    }
    // Delay IK until the first valid goal: no intermediate move to the stale
    // wheel reference. The accepted measured joint hold remains the first frame.
    alignment_input_.current_position = start.q;
    alignment_input_.current_velocity = start.dq;
    alignment_input_.current_acceleration = start.ddq;
    alignment_input_.target_position = start.q;
    alignment_input_.synchronization = ruckig::Synchronization::Time;
    alignment_input_.target_velocity.fill(0);
    alignment_input_.target_acceleration.fill(0);
    // Share CONTROL's angular response settings, expressed in joint radians
    // during takeover. Linear wheel limits (metres) cannot be joint limits.
    // Preserve the configured/URDF joint velocity cap without a slow-entry cap.
    alignment_input_.max_velocity.fill(takeover_speed_);
    alignment_input_.max_acceleration.fill(acceleration[0]);
    alignment_input_.max_jerk.fill(jerk[0]);
    for (int j = 0; j < 14; ++j) {
        require(std::abs(start.dq[j]) < 1e-10 && std::abs(start.ddq[j]) < 1e-10,
                "Servo takeover requires a stationary hold");
    }
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
                                  const std::array<double, 2>& a, bool takeover) {
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
        for (int iteration = 0; iteration < (takeover ? 120 : 8); ++iteration) {
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
            double scale = 1;
            if (takeover) {
                double largest = 0;
                for (double value : dq) largest = std::max(largest, std::abs(value));
                if (largest > .1) scale = .1 / largest;
            }
            for (int j = 0; j < 7; ++j)
                q[j] += scale * dq[j];
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
    if (aligning_) return align(angle, displacement, ratio, stop);
    input_.control_interface =
        stop ? ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    input_.target_position = {angle, displacement};
    // Lowering the speed limit does not reset the current velocity or acceleration.
    for (int i = 0; i < 2; ++i)
        input_.max_velocity[i] = speed_[i] * ratio;
    ruckig::Trajectory<2> trajectory;
    auto result = planner_.calculate(input_, trajectory);
    if (result == ruckig::Result::ErrorSynchronizationCalculation &&
        std::isfinite(trajectory.get_duration()) && trajectory.get_duration() >= 0) {
        // Step 2 can fail numerically at an extremal synchronization time,
        // including near-rest residuals. Retry once with one command block of
        // slack, preserving the current state, constraints and Time synchronization.
        auto retry = input_;
        retry.minimum_duration = trajectory.get_duration() + period_;
        result = planner_.calculate(retry, trajectory);
        if (result >= 0)
            Logger::warn("Servo Ruckig synchronization recovered with minimum_duration={} s input:{}",
                         *retry.minimum_duration, retry.to_string());
    }
    if (result < 0)
        throw std::runtime_error("Servo Ruckig failed: " + std::to_string(int(result)) +
                                 " period=" + std::to_string(period_) +
                                 " stop=" + std::to_string(stop) + " input:" + input_.to_string());
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
        checkGeometry(f, 1e-5, 1e-4);
        frames.push_back(f);
    }
    frames.front() = last_;
    frames.back() = end;
    last_ = end;
    trajectory.at_time(period_, input_.current_position, input_.current_velocity,
                       input_.current_acceleration);
    return frames;
}
void ServoPlanner::checkGeometry(const JointFrame& f, double position_tolerance,
                                  double rotation_tolerance) {
    for (int side = 0; side < 2; ++side) {
        std::array<double, 7> q;
        std::copy_n(f.q.begin() + 7 * side, 7, q.begin());
        pinocchio::SE3 actual;
        require(kinematics_.solveFk(static_cast<Side>(side), q, actual), "Servo FK failed");
        const auto desired = target_(side, f.angle, f.displacement);
        const double position_error = (actual.translation() - desired.translation()).norm();
        const double rotation_error = rotationError(actual, desired);
        if (!(position_error <= position_tolerance && rotation_error <= rotation_tolerance))
            throw std::runtime_error("Servo wheel geometry residual side=" + std::to_string(side) +
                " position_m=" + std::to_string(position_error) +
                " rotation_rad=" + std::to_string(rotation_error) +
                " limits=" + std::to_string(position_tolerance) + "," + std::to_string(rotation_tolerance));
    }
}

std::vector<JointFrame> ServoPlanner::align(double angle, double displacement, double ratio, bool stop) {
    const std::array<double, 2> goal{angle, displacement};
    if (!stop && (!alignment_goal_valid_ || alignment_goal_ != goal)) {
        const auto destination = endpoint(goal, {0, 0}, {0, 0}, true);
        check_(destination);
        alignment_input_.target_position = destination.q;
        if (!alignment_goal_valid_)
            Logger::info("Servo joint takeover from held joints to absolute angle_rad={} displacement_m={}",
                         angle, displacement);
        alignment_goal_ = goal;
        alignment_goal_valid_ = true;
    }
    alignment_input_.control_interface = stop ? ruckig::ControlInterface::Velocity :
                                               ruckig::ControlInterface::Position;
    alignment_input_.max_velocity.fill(takeover_speed_ * ratio);
    ruckig::Trajectory<14> trajectory;
    auto result = alignment_planner_.calculate(alignment_input_, trajectory);
    if (result == ruckig::Result::ErrorSynchronizationCalculation &&
        std::isfinite(trajectory.get_duration()) && trajectory.get_duration() >= 0) {
        // As in wheel planning, a near-rest axis can put synchronized duration
        // on a numerical boundary. Add one block without resetting q/dq/ddq.
        auto retry = alignment_input_;
        retry.minimum_duration = trajectory.get_duration() + period_;
        result = alignment_planner_.calculate(retry, trajectory);
        if (result >= 0)
            Logger::warn("Servo takeover synchronization recovered with minimum_duration={} s",
                         *retry.minimum_duration);
    }
    if (result < 0)
        throw std::runtime_error("Servo takeover Ruckig failed: " + std::to_string(int(result)) +
                                 " input:" + alignment_input_.to_string());
    const size_t ticks = std::llround(period_ / .001);
    std::vector<JointFrame> frames;
    frames.reserve(ticks + 1);
    for (size_t k = 0; k <= ticks; ++k) {
        JointFrame frame = last_;
        trajectory.at_time(k * .001, frame.q, frame.dq, frame.ddq);
        check_(frame);
        // Intermediate joint poses need not fit a rigid wheel. The stored wheel
        // reference is metadata, not a measurement during joint takeover.
        frames.push_back(frame);
    }
    frames.front() = last_;
    last_ = frames.back();
    alignment_input_.current_position = last_.q;
    alignment_input_.current_velocity = last_.dq;
    alignment_input_.current_acceleration = last_.ddq;
    if (!stop && trajectory.get_duration() <= period_) {
        last_.angle = alignment_goal_[0];
        last_.displacement = alignment_goal_[1];
        checkGeometry(last_, 1e-5, 1e-4);
        frames.back() = last_;
        input_.current_position = alignment_goal_;
        input_.current_velocity = {0, 0};
        input_.current_acceleration = {0, 0};
        aligning_ = false;
        Logger::info("Servo joint takeover complete angle_rad={} displacement_m={}",
                     last_.angle, last_.displacement);
    }
    return frames;
}

bool ServoPlanner::stopped() const {
    if (aligning_) {
        for (int j = 0; j < 14; ++j)
            if (std::abs(last_.dq[j]) > 1e-10 || std::abs(last_.ddq[j]) > 1e-10) return false;
        return true;
    }
    for (int i = 0; i < 2; ++i)
        if (std::abs(input_.current_velocity[i]) > 1e-10 ||
            std::abs(input_.current_acceleration[i]) > 1e-10)
            return false;
    return true;
}
} // namespace aviator

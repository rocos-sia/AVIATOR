#include "JointWheelFeedback.hpp"
#include <algorithm>
#include <cmath>

namespace aviator {
nlohmann::json JointWheelFeedback::measure(const ArmFeedback& feedback, bool locked, uint64_t now) {
    nlohmann::json result{{"source", "joint_fk"}, {"valid", false}, {"sample_mono_us", 0},
                          {"angle", nullptr}, {"displacement", nullptr}};
    if (!locked) return result;
    for (int side = 0; side < 2; ++side) {
        const double sample = feedback.sample_time[side];
        if (!feedback.valid[side] || !feedback.enabled[side] || !std::isfinite(sample) || sample <= 0 ||
            sample > double(now) / 1e6 || double(now) / 1e6 - sample >= .1) return result;
    }
    // The oldest real sample determines freshness; publishing must not refresh it.
    result["sample_mono_us"] = uint64_t(std::min(feedback.sample_time[0], feedback.sample_time[1]) * 1e6);
    if (std::abs(feedback.sample_time[0] - feedback.sample_time[1]) > .005) return result;
    try {
        WheelPoseEstimate poses[2];
        for (int side = 0; side < 2; ++side) {
            std::array<double, 7> q;
            std::copy_n(feedback.q.begin() + side * 7, 7, q.begin());
            for (int axis = 0; axis < 7; ++axis)
                if (!std::isfinite(q[axis]) || q[axis] < kinematics_->jointLower(Side(side), axis) ||
                    q[axis] > kinematics_->jointUpper(Side(side), axis)) return result;
            pinocchio::SE3 flange;
            if (!kinematics_->solveFk(Side(side), q, flange)) return result;
            poses[side] = estimate_.fromFlange(side, flange);
        }
        const double angle = meanWheelAngle(poses[0].angle, poses[1].angle);
        const double displacement = (poses[0].displacement + poses[1].displacement) / 2;
        if (!std::isfinite(angle) || !std::isfinite(displacement)) return result;
        result["angle"] = angle;
        result["displacement"] = displacement;
        result["valid"] = true;
    } catch (const std::exception&) {
        // Unobservable/non-finite FK does not become a controller fault or a reference value.
    }
    return result;
}
} // namespace aviator

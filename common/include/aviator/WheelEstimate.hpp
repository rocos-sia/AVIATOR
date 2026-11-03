#pragma once
#include "aviator/GraspTools.hpp"
#include "aviator/Pose.hpp"
#include <cmath>

namespace aviator {
inline pinocchio::SE3 wheelEstimateFrame(const YAML::Node& n) {
    const auto p = n["position"].as<std::vector<double>>();
    const auto q = n["quaternion"].as<std::vector<double>>();
    if (p.size() != 3 || q.size() != 4) throw std::runtime_error("Invalid grasp frame");
    for (double x : p) if (!std::isfinite(x)) throw std::runtime_error("Invalid frame position");
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1) > 1e-6)
        throw std::runtime_error("Invalid frame quaternion");
    return {rotation.normalized(), Eigen::Vector3d(p[0], p[1], p[2])};
}
struct WheelPoseEstimate {
    double angle, displacement, transverse_error, tilt_error;
};
class WheelEstimate {
    pinocchio::SE3 origin_, handles_[2], tools_[2];
public:
    explicit WheelEstimate(const YAML::Node& grasp) : origin_(wheelEstimateFrame(grasp["wheel_origin"])) {
        for (int s = 0; s < 2; ++s) {
            handles_[s] = wheelEstimateFrame(grasp[s ? "right" : "left"]);
            tools_[s] = wheelEstimateFrame(aviator::toolFrameConfig(grasp, s));
        }
    }
    WheelPoseEstimate fromFlange(int side, const pinocchio::SE3& flange) const {
        // Invert the existing Core geometry: F = O * Rz(angle)Tz(d) * H * inv(T).
        const auto wheel = origin_.inverse() * flange * tools_[side] * handles_[side].inverse();
        const auto& r = wheel.rotation();
        const double x = r(0, 0) + r(1, 1), y = r(1, 0) - r(0, 1);
        if (!wheel.translation().allFinite() || !r.allFinite() || std::hypot(x, y) < 1e-8)
            throw std::runtime_error("Wheel angle is not observable from flange pose");
        const double angle = std::atan2(y, x);
        const Eigen::Matrix3d projected = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return {angle, wheel.translation().z(), wheel.translation().head<2>().norm(),
                Eigen::Quaterniond(projected).angularDistance(Eigen::Quaterniond(r))};
    }
};
inline double wheelAngleDifference(double a, double b) { return std::atan2(std::sin(a-b), std::cos(a-b)); }
inline double meanWheelAngle(double a, double b) { return a + wheelAngleDifference(b, a) / 2; }
} // namespace aviator

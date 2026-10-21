#pragma once
#include "aviator/GraspTools.hpp"
#include "aviator/Kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace wheel_bandwidth {
constexpr double pi = 3.14159265358979323846;
inline pinocchio::SE3 frame(const YAML::Node& n) {
    const auto p = n["position"].as<std::vector<double>>();
    const auto q = n["quaternion"].as<std::vector<double>>();
    if (p.size() != 3 || q.size() != 4) throw std::runtime_error("Invalid grasp frame");
    for (double x : p) if (!std::isfinite(x)) throw std::runtime_error("Invalid frame position");
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1) > 1e-6)
        throw std::runtime_error("Invalid frame quaternion");
    return {rotation.normalized(), Eigen::Vector3d(p[0], p[1], p[2])};
}
struct Estimate {
    double angle, displacement, transverse_error, tilt_error;
};
class WheelEstimate {
    pinocchio::SE3 origin_, handles_[2], tools_[2];
public:
    explicit WheelEstimate(const YAML::Node& grasp) : origin_(frame(grasp["wheel_origin"])) {
        for (int s = 0; s < 2; ++s) {
            handles_[s] = frame(grasp[s ? "right" : "left"]);
            tools_[s] = frame(aviator::toolFrameConfig(grasp, s));
        }
    }
    Estimate fromFlange(int side, const pinocchio::SE3& flange) const {
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
inline double difference(double a, double b) { return std::atan2(std::sin(a-b), std::cos(a-b)); }
inline double meanAngle(double a, double b) { return a + difference(b, a) / 2; }
inline double smooth(double u) { u = std::clamp(u, 0., 1.); return u*u*u*(10+u*(-15+6*u)); }
struct Point { int segment; double frequency, local_time, offset; bool measure; };
struct Target { double angle, displacement; };
struct Sweep {
    std::vector<double> frequencies{.1, .2, .3, .5, .7, 1.};
    double amplitude = 10*pi/180;
    bool translation = false;
    int settle = 2, measure = 5, ramp = 1;
    void validateMotion(double center) const {
        if (!std::isfinite(center) || center < -.170 || center > 0 ||
            !std::isfinite(amplitude) || amplitude <= 0)
            throw std::runtime_error("Invalid sweep center or amplitude");
        if (translation) {
            if (center-amplitude < -.170 || center+amplitude > 0)
                throw std::runtime_error("Translation center +/- amplitude must be within [-170,0] mm");
        } else if (amplitude > 10*pi/180) {
            throw std::runtime_error("Rotation amplitude must be (0,10] deg");
        }
    }
    Target target(double offset, double center) const {
        return translation ? Target{0., center+offset} : Target{offset, center};
    }
    double duration() const {
        double t = 0; for (double f : frequencies) t += (2*ramp+settle+measure)/f; return t;
    }
    Point at(double t) const {
        for (size_t i = 0; i < frequencies.size(); ++i) {
            const double f = frequencies[i], cycles = t*f, count = 2*ramp+settle+measure;
            if (cycles < count && t >= 0) {
                const double envelope = smooth(cycles/ramp)*smooth((count-cycles)/ramp);
                return {int(i), f, t, amplitude*envelope*std::sin(2*pi*cycles),
                    cycles >= ramp+settle && cycles < ramp+settle+measure};
            }
            t -= count/f;
        }
        return {-1, 0, 0, 0, false};
    }
};
} // namespace wheel_bandwidth

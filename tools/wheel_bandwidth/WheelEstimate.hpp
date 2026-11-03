#pragma once
#include "aviator/WheelEstimate.hpp"
#include "aviator/Kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace wheel_bandwidth {
constexpr double pi = 3.14159265358979323846;
using aviator::WheelEstimate;
using Estimate = aviator::WheelPoseEstimate;
inline pinocchio::SE3 frame(const YAML::Node& n) { return aviator::wheelEstimateFrame(n); }
inline double difference(double a, double b) { return aviator::wheelAngleDifference(a, b); }
inline double meanAngle(double a, double b) { return aviator::meanWheelAngle(a, b); }
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

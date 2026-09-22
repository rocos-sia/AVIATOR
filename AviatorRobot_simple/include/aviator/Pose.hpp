#pragma once
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Geometry>
#include <array>

namespace aviator {

// Straight translation and shortest-path rotation; s is the smoothed progress [0,1].
inline pinocchio::SE3 interpolatePose(const pinocchio::SE3 &from,
                                     const pinocchio::SE3 &to, double s) {
    const Eigen::Quaterniond a(from.rotation()), b(to.rotation());
    return {a.slerp(s, b).normalized(),
            ((1 - s) * from.translation() + s * to.translation()).eval()};
}

inline double rotationError(const pinocchio::SE3 &a, const pinocchio::SE3 &b) {
    return Eigen::Quaterniond(a.rotation()).angularDistance(Eigen::Quaterniond(b.rotation()));
}

// xCore uses row-major 4x4 matrices. Copy entries explicitly, independent of Eigen storage.
inline std::array<double, 16> poseToRowMajor(const pinocchio::SE3 &pose) {
    std::array<double, 16> values{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) values[4 * row + col] = pose.rotation()(row, col);
        values[4 * row + 3] = pose.translation()[row];
    }
    values[15] = 1;
    return values;
}

inline pinocchio::SE3 poseFromRowMajor(const std::array<double, 16> &values) {
    auto pose = pinocchio::SE3::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) pose.rotation()(row, col) = values[4 * row + col];
        pose.translation()[row] = values[4 * row + 3];
    }
    return pose;
}

} // namespace aviator

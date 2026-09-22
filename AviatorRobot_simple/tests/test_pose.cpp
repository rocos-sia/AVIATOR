// 位姿约定：已知 SDK 矩阵、工具变换和接近插值边界，不连接真机。
#include "aviator/Pose.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }

int main() {
    try {
        const pinocchio::SE3 pose(Eigen::Quaterniond(std::sqrt(.5),0,0,std::sqrt(.5)), Eigen::Vector3d(1,2,3));
        const std::array<double,16> expected{0,-1,0,1, 1,0,0,2, 0,0,1,3, 0,0,0,1};
        const auto encoded = aviator::poseToRowMajor(pose);
        for (int i = 0; i < 16; ++i) check(std::abs(encoded[i]-expected[i]) < 1e-14, "SDK row-major encoding mismatch");
        const auto decoded = aviator::poseFromRowMajor(expected);
        check((decoded.translation()-pose.translation()).norm() < 1e-14 &&
              aviator::rotationError(decoded,pose) < 1e-14, "SDK row-major decoding mismatch");
        const pinocchio::SE3 tool(Eigen::Matrix3d::Identity(), Eigen::Vector3d(.1,0,0));
        check(((pose*tool).translation()-Eigen::Vector3d(1,2.1,3)).norm() < 1e-14, "Tool multiplication order changed");
        check(((pose*tool.inverse()).translation()-Eigen::Vector3d(1,1.9,3)).norm() < 1e-14, "Flange target offset changed");

        const pinocchio::SE3 start(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitY()).toRotationMatrix(), Eigen::Vector3d(1,2,3));
        for (double angle : {0., 1e-9, .6, M_PI-1e-9, M_PI, M_PI+1e-9, 2*M_PI-.2}) {
            const pinocchio::SE3 end(start.rotation()*Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                     Eigen::Vector3d(-2,4,1));
            for (double s : {0., .25, .5, .75, 1.}) {
                const auto point = aviator::interpolatePose(start,end,s);
                check((point.translation()-((1-s)*start.translation()+s*end.translation())).norm() < 1e-14,
                      "Approach translation is not straight");
                check(std::abs(aviator::rotationError(start,point)-s*aviator::rotationError(start,end)) < 1e-12,
                      "Approach rotation is not shortest-path interpolation");
                check(point.rotation().isUnitary(1e-12) && point.rotation().determinant()>0, "Invalid interpolated rotation");
                if (s == 1.) check(aviator::rotationError(point,end) < 1e-12, "Interpolation missed endpoint");
            }
        }
        std::cout << "Pose conventions, SDK matrices, tool offsets and interpolation boundaries passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

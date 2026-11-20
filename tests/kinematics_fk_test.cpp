#include "aviator/backend.hpp"
#include <pin_ik/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <yaml-cpp/yaml.h>
using namespace aviator;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main(int argc, char** argv) {
    try {
        check(argc == 2, "expected project root");
        const auto config_dir = std::filesystem::path(argv[1]) / "config";
        const auto robot = YAML::LoadFile((config_dir / "robot.yaml").string());
        const auto posture = YAML::LoadFile((config_dir / robot["posture"].as<std::string>()).string());
        const double margin = posture["joint2_planning_margin_deg"].as<double>() * M_PI / 180;
        const double lo = posture["joint2_limits_deg"][0].as<double>() * M_PI / 180 + margin;
        const double hi = posture["joint2_limits_deg"][1].as<double>() * M_PI / 180 - margin;
        const auto path = (config_dir / robot["urdf"].as<std::string>()).string();
        auto kin = makePinIkKinematics(path, lo, hi), clean = makePinIkKinematics(path, lo, hi);
        std::ifstream file(path);
        const std::string xml((std::istreambuf_iterator<char>(file)), {});
        std::mt19937 random(1505);
        double max_position = 0, max_rotation = 0;
        unsigned derivatives = 0;
        std::vector<double> all_times, tip_times;
        for (int side = 0; side < 2; ++side) {
            pinocchio::Model model;
            Eigen::VectorXd lower, upper;
            pinocchio::FrameIndex tip;
            const auto prefix = std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2";
            PIN_IK::loadURDFModel(xml, "aircraft", prefix + "_flan_link", model, lower, upper, tip);
            pinocchio::Data data(model);
            for (int sample = 0; sample < 502; ++sample) {
                std::array<double, 7> q;
                for (int axis = 0; axis < 7; ++axis)
                    q[axis] = sample == 0 ? lower[axis] : sample == 1 ? upper[axis] :
                        std::uniform_real_distribution<double>(lower[axis], upper[axis])(random);
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(q.data());
                pinocchio::SE3 actual;
                check(kin->solveFk(Side(side), q, actual), "finite FK rejected");
                pinocchio::forwardKinematics(model, data, angles);
                pinocchio::updateFramePlacements(model, data);
                max_position = std::max(max_position, (actual.translation() - data.oMf[tip].translation()).norm());
                max_rotation = std::max(max_rotation, (actual.rotation() - data.oMf[tip].rotation()).norm());
                Eigen::Matrix<double, 6, 1> twist, acceleration;
                twist << .001, -.002, .003, -.004, .005, -.006;
                acceleration = twist * .1;
                std::array<double, 7> dq{}, ddq{}, clean_dq{}, clean_ddq{};
                const bool a = kin->jointDerivatives(Side(side), q, twist, acceleration, dq, ddq);
                const bool b = clean->jointDerivatives(Side(side), q, twist, acceleration, clean_dq, clean_ddq);
                check(a == b, "FK workspace must not change derivative acceptance");
                if (a) {
                    ++derivatives;
                    for (int axis = 0; axis < 7; ++axis)
                        check(std::abs(dq[axis] - clean_dq[axis]) < 1e-12 &&
                              std::abs(ddq[axis] - clean_ddq[axis]) < 1e-12,
                              "FK workspace must not change derivatives");
                }
                // Compare the old and new frame-update work on the same model/q.
                for (int mode = 0; mode < 2; ++mode) {
                    const auto began = std::chrono::steady_clock::now();
                    for (int repeat = 0; repeat < 50; ++repeat) {
                        pinocchio::forwardKinematics(model, data, angles);
                        if (mode == 0) pinocchio::updateFramePlacements(model, data);
                        else pinocchio::updateFramePlacement(model, data, tip);
                    }
                    const double us = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - began).count() / 50;
                    (mode == 0 ? all_times : tip_times).push_back(us);
                }
            }
            std::array<double, 7> bad{}; bad[0] = std::numeric_limits<double>::quiet_NaN();
            pinocchio::SE3 sentinel = pinocchio::SE3::Identity();
            check(!kin->solveFk(Side(side), bad, sentinel) && sentinel.isIdentity(), "invalid FK preserves output");
        }
        check(max_position < 1e-12 && max_rotation < 1e-12 && derivatives > 900,
              "random/boundary FK and derivative regression");
        std::sort(all_times.begin(), all_times.end()); std::sort(tip_times.begin(), tip_times.end());
        std::cout << "1004 FK poses: max_position=" << max_position << " max_rotation=" << max_rotation
                  << " derivatives=" << derivatives << " all_frames_p50_us=" << all_times[all_times.size()/2]
                  << " tip_p50_us=" << tip_times[tip_times.size()/2] << '\n';
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

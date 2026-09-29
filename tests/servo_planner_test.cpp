#include "ServoPlanner.hpp"
#include "aviator/backend.hpp"
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>
using namespace aviator;
pinocchio::SE3 frame(const YAML::Node& n) {
    auto p = n["position"].as<std::vector<double>>(), q = n["quaternion"].as<std::vector<double>>();
    return {Eigen::Quaterniond(q[0], q[1], q[2], q[3]), Eigen::Vector3d(p[0], p[1], p[2])};
}
int main(int argc, char** argv) {
    try {
        if (argc != 2)
            throw std::runtime_error("Expected project root");
        std::filesystem::path root = argv[1];
        auto config = YAML::LoadFile((root / "config/robot.yaml").string());
        auto posture =
            YAML::LoadFile((root / "config" / config["posture"].as<std::string>()).string());
        auto grasp = YAML::LoadFile((root / "config" / config["grasp"].as<std::string>()).string());
        double lo = 85 * M_PI / 180, hi = 94 * M_PI / 180;
        auto kin = makePinIkKinematics(
            (root / "config" / config["urdf"].as<std::string>()).string(), lo, hi);
        auto origin = frame(grasp["wheel_origin"]), tool = frame(grasp["tool"]);
        pinocchio::SE3 handles[] = {frame(grasp["left"]), frame(grasp["right"])};
        auto target = [&](int side, double a, double d) {
            return origin *
                   pinocchio::SE3(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                  Eigen::Vector3d(0, 0, d)) *
                   handles[side] * tool.inverse();
        };
        JointFrame start;
        for (int side = 0; side < 2; ++side) {
            auto seed = posture[side ? "right_approach_seed_deg" : "left_approach_seed_deg"]
                            .as<std::vector<double>>();
            std::array<double, 7> q{}, out{};
            for (int j = 0; j < 7; ++j)
                q[j] = seed[j] * M_PI / 180;
            if (!kin->solveIk(static_cast<Side>(side), q, target(side, 0, 0), out))
                throw std::runtime_error("Initial IK");
            std::copy(out.begin(), out.end(), start.q.begin() + side * 7);
        }
        double peak_v = 0, peak_a = 0, peak_j = 0;
        auto check = [&](const JointFrame& f) {
            if (f.q[1] < lo || f.q[1] > hi || f.q[8] < lo || f.q[8] > hi)
                throw std::runtime_error("J2 limit: " + std::to_string(f.q[1] * 180 / M_PI) + ", " +
                                         std::to_string(f.q[8] * 180 / M_PI) +
                                         " wheel=" + std::to_string(f.angle));
        };
        ServoPlanner planner(*kin, target, check, origin, start, .02, {.4, .04}, {.05, .005},
                             {.05, .005});
        JointFrame previous = start;
        // A held step, target reversal before arrival, full combined range, then braking.
        for (int k = 0; k < 3000; ++k) {
            double a = k < 140    ? .43633
                       : k < 350  ? -.174532
                       : k < 1300 ? .87266
                       : k < 2350 ? -.87266
                                  : .5;
            double d = k < 140 ? 0 : k < 350 ? -.017 : k < 1300 ? -.170 : 0;
            bool stop = k >= 2500;
            auto block = planner.advance(a, d, 1, stop);
            for (int j = 0; j < 14; ++j)
                if (std::abs(block.front().q[j] - previous.q[j]) > 1e-12 ||
                    std::abs(block.front().dq[j] - previous.dq[j]) > 1e-12 ||
                    std::abs(block.front().ddq[j] - previous.ddq[j]) > 1e-12)
                    throw std::runtime_error("Block boundary discontinuity");
            for (size_t i = 1; i < block.size(); ++i)
                for (int j = 0; j < 14; ++j) {
                    peak_v = std::max(peak_v, std::abs(block[i].dq[j]));
                    peak_a = std::max(peak_a, std::abs(block[i].ddq[j]));
                    peak_j =
                        std::max(peak_j, std::abs(block[i].ddq[j] - block[i - 1].ddq[j]) / .001);
                    if (std::abs((block[i].q[j] - block[i - 1].q[j]) / .001 -
                                 .5 * (block[i].dq[j] + block[i - 1].dq[j])) > 1e-6 ||
                        std::abs((block[i].dq[j] - block[i - 1].dq[j]) / .001 -
                                 .5 * (block[i].ddq[j] + block[i - 1].ddq[j])) > 1e-4)
                        throw std::runtime_error(
                            "Sample derivatives disagree with joint positions");
                }
            previous = block.back();
            if (stop && planner.stopped())
                break;
        }
        if (!planner.stopped())
            throw std::runtime_error("Did not stop");
        std::cout << "Servo step/reversal/braking C2 passed; joint peaks v=" << peak_v
                  << " a=" << peak_a << " j=" << peak_j << '\n';
        // User's faster wheel parameters: small initial offset, then a rapid reversal.
        ServoPlanner fast(*kin, target, check, origin, start, .02, {.8, .08}, {1.0, .1}, {.5, .05});
        previous = start;
        peak_a = peak_j = 0;
        for (int k = 0; k < 800; ++k) {
            double a = k < 60 ? .00085304 : k < 180 ? -.87266 : k < 360 ? .87266 : .0332686;
            double d = k < 60 ? -.00116325 : k < 180 ? -.170 : 0;
            const bool stop = k >= 700;
            auto block = fast.advance(a, d, 1, stop);
            if (block.front().q != previous.q || block.front().dq != previous.dq || block.front().ddq != previous.ddq)
                throw std::runtime_error("Fast Servo block boundary discontinuity");
            for (size_t i = 1; i < block.size(); ++i)
                for (int j = 0; j < 14; ++j) {
                    peak_a = std::max(peak_a, std::abs(block[i].ddq[j]));
                    peak_j = std::max(peak_j, std::abs(block[i].ddq[j] - block[i-1].ddq[j]) / .001);
                }
            previous = block.back();
            if (stop && fast.stopped()) break;
        }
        if (!fast.stopped() || (peak_a <= .5 && peak_j <= 1))
            throw std::runtime_error("Fast scenario: stopped=" + std::to_string(fast.stopped()) + " a=" + std::to_string(peak_a) + " j=" + std::to_string(peak_j));
        std::cout << "Fast Servo passed beyond old dynamic caps; joint peaks a=" << peak_a << " j=" << peak_j << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

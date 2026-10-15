#include "ServoPlanner.hpp"
#include "DeviceServo.hpp"
#include <chrono>
#include "aviator/backend.hpp"
#include "aviator/GraspTools.hpp"
#include <filesystem>
#include <iostream>
#include <random>
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
        const double margin = posture["joint2_planning_margin_deg"].as<double>() * M_PI / 180;
        double lo = posture["joint2_limits_deg"][0].as<double>() * M_PI / 180 + margin;
        double hi = posture["joint2_limits_deg"][1].as<double>() * M_PI / 180 - margin;
        auto kin = makePinIkKinematics(
            (root / "config" / config["urdf"].as<std::string>()).string(), lo, hi);
        auto origin = frame(grasp["wheel_origin"]);
        pinocchio::SE3 tools[] = {frame(toolFrameConfig(grasp, 0)), frame(toolFrameConfig(grasp, 1))};
        pinocchio::SE3 handles[] = {frame(grasp["left"]), frame(grasp["right"])};
        auto target = [&](int side, double a, double d) {
            return origin *
                   pinocchio::SE3(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                  Eigen::Vector3d(0, 0, d)) *
                   handles[side] * tools[side].inverse();
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
            for (int j = 0; j < 14; ++j)
                if (!std::isfinite(f.q[j]) || f.q[j] < kin->jointLower(Side(j / 7), j % 7) ||
                    f.q[j] > kin->jointUpper(Side(j / 7), j % 7))
                    throw std::runtime_error("Physical joint limit at axis " + std::to_string(j));
            if (f.q[1] < lo || f.q[1] > hi || f.q[8] < lo || f.q[8] > hi)
                throw std::runtime_error("J2 limit: " + std::to_string(f.q[1] * 180 / M_PI) + ", " +
                                         std::to_string(f.q[8] * 180 / M_PI) +
                                         " wheel=" + std::to_string(f.angle));
        };
        DeviceSettings settings(loadMotionConfig(root / "config/system.yaml"));
        DeviceServerOptions options;
        configureDeviceServo(options, settings);
        // solveIk fixtures have finite numerical residuals. Finish a stationary
        // takeover first when a scenario specifically exercises wheel tracking.
        const auto nominal = [&](JointFrame f) {
            auto servo = options.servo(f);
            for (int tick = 0; tick < 1000; ++tick)
                f = servo->step({f.angle, f.displacement, 1., 1, false});
            return f;
        };
        start = nominal(start);
        const auto pose = [&](double angle, double displacement) {
            JointFrame f = start;
            f.angle = angle; f.displacement = displacement;
            for (int side = 0; side < 2; ++side) {
                std::array<double, 7> seed{}, q{};
                std::copy_n(start.q.begin() + side * 7, 7, seed.begin());
                if (!kin->solveIk(Side(side), seed, target(side, angle, displacement), q))
                    throw std::runtime_error("Takeover fixture IK failed");
                std::copy(q.begin(), q.end(), f.q.begin() + side * 7);
            }
            return f;
        };
        // Repeat entry from opposite sides with stale wheel metadata: the
        // destination must remain absolute without accumulating pose offsets.
        for (double angle : {.18, -.14}) {
            auto held = pose(angle, -.11);
            // A stale software reference and compliant asymmetric posture must
            // not redefine the absolute target or reject joint-space takeover.
            held.q[10] -= .02;
            held.angle = 0; held.displacement = -.085;
            const auto unchanged = held.q;
            ServoPlanner takeover(*kin, target, check, origin, held, .02,
                                  {.4, .04}, {.05, .005}, {.05, .005});
            if (held.q != unchanged) throw std::runtime_error("Takeover replaced measured hold with IK");
            JointFrame previous = held;
            for (int k = 0; k < 600; ++k) {
                const auto block = takeover.advance(0, -.085, 1, false);
                if (block.front().q != previous.q || block.front().dq != previous.dq ||
                    block.front().ddq != previous.ddq)
                    throw std::runtime_error("Takeover C2 boundary discontinuity");
                for (size_t i = 1; i < block.size(); ++i) {
                    for (int j = 0; j < 14; ++j) {
                        const auto& f = block[i]; const auto& p = block[i-1];
                        if (std::abs(f.q[j] - p.q[j] - .0005 * (f.dq[j] + p.dq[j])) > 1e-7 ||
                            std::abs(f.dq[j] - p.dq[j] - .0005 * (f.ddq[j] + p.ddq[j])) > 1.35e-7)
                            throw std::runtime_error("Takeover per-tick derivative discontinuity");
                        if (k == 0 && (std::abs(f.dq[j]) > .4 + 1e-8 ||
                            std::abs(f.ddq[j]) > .05 + 1e-8 ||
                            std::abs(f.ddq[j] - p.ddq[j]) / .001 > .05 + 1e-6))
                            throw std::runtime_error("First takeover block exceeded alignment limits");
                    }
                }
                previous = block.back();
            }
            if (std::abs(previous.angle) > 1e-9 || std::abs(previous.displacement + .085) > 1e-9)
                throw std::runtime_error("Takeover did not reach absolute center");
            for (int side = 0; side < 2; ++side) {
                std::array<double, 7> q; pinocchio::SE3 actual;
                std::copy_n(previous.q.begin() + side * 7, 7, q.begin());
                if (!kin->solveFk(Side(side), q, actual) ||
                    (actual.translation() - target(side, 0, -.085).translation()).norm() > 1e-7 ||
                    rotationError(actual, target(side, 0, -.085)) > 1e-7)
                    throw std::runtime_error("Takeover retained an offset at physical center");
            }
        }
        auto rebased = pose(.1, -.10);
        rebased.q[10] -= .002;
        ServoPlanner interrupted(*kin, target, check, origin, rebased, .001,
                                {.4, .04}, {.05, .005}, {.05, .005});
        for (int i = 0; i < 20; ++i) interrupted.advance(0, -.085, 1, false);
        for (int i = 0; i < 1000 && !interrupted.stopped(); ++i) interrupted.advance(0, -.085, 1, true);
        if (!interrupted.stopped()) throw std::runtime_error("Takeover did not stop on cancellation");
        const auto stopped = interrupted.advance(0, -.085, 1, true).back();
        const auto still = interrupted.advance(.5, 0, 1, true).back();
        if (still.q != stopped.q || still.angle != rebased.angle || still.displacement != rebased.displacement)
            throw std::runtime_error("Cancelled takeover consumed a new wheel target");
        std::cout << "Absolute takeover / C2 alignment / cancellation passed\n";
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
                                 .5 * (block[i].ddq[j] + block[i - 1].ddq[j])) > 1.35e-4)
                        throw std::runtime_error(
                            "Sample derivatives disagree with joint positions k=" + std::to_string(k) +
                            " i=" + std::to_string(i) + " j=" + std::to_string(j));
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
        // Keyboard targets ramp at 50 Hz, then remain held. With the configured
        // limits, seed 98 reaches a deceleration synchronization failure at tick 790
        // in Ruckig 0.15.3 without the synchronization retry.
        start.displacement = -.085;
        for (int side = 0; side < 2; ++side) {
            std::array<double, 7> q{}, out{};
            std::copy_n(start.q.begin() + side * 7, 7, q.begin());
            if (!kin->solveIk(static_cast<Side>(side), q, target(side, 0, start.displacement), out))
                throw std::runtime_error("Keyboard initial IK");
            std::copy(out.begin(), out.end(), start.q.begin() + side * 7);
        }
        start = nominal(start);
        ServoPlanner keyboard(*kin, target, check, origin, start, .02,
                              {1.8, .5}, {2., 1.}, {4., 2.});
        std::mt19937 random(98);
        std::array<int, 2> direction{};
        double a = 0, d = start.displacement;
        previous = start;
        for (int k = 0; k < 950; ++k) {
            if (k % 50 == 0)
                for (auto& value : direction) value = int(random() % 3) - 1;
            a = std::clamp(a + direction[0] * (-.87266 * .02), -.87266, .87266);
            d = std::clamp(d + direction[1] * .085 * .02, -.170, 0.);
            const auto block = keyboard.advance(a, d, 1, false);
            if (block.front().q != previous.q || block.front().dq != previous.dq ||
                block.front().ddq != previous.ddq)
                throw std::runtime_error("Keyboard Servo block boundary discontinuity");
            previous = block.back();
        }
        for (int k = 0; k < 200 && !keyboard.stopped(); ++k)
            keyboard.advance(a, d, 1, true);
        if (!keyboard.stopped()) throw std::runtime_error("Keyboard scenario did not stop");
        std::cout << "Keyboard held-target synchronization regression passed\n";
        // Production factory: 1 ms planning, current robot constraints, rapid target
        // replacement, continuous q/dq/ddq, input-speed changes and controlled stop.
        auto latest = options.servo(start);
        previous = start;
        double total_us = 0, max_us = 0;
        unsigned count = 0;
        for (int k = 0; k < 4000; ++k) {
            const bool stopping = k >= 1600;
            ServoGoal goal{(k / 80) % 2 ? -.08 : .08, -.085, k < 800 ? 1. : .5, 1, stopping};
            const auto began = std::chrono::steady_clock::now();
            const auto current = latest->step(goal);
            const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - began).count();
            total_us += us; max_us = std::max(max_us, us); ++count;
            for (int j = 0; j < 14; ++j) {
                if (std::abs(current.q[j] - previous.q[j] - .0005 * (current.dq[j] + previous.dq[j])) > 1e-7 ||
                    std::abs(current.dq[j] - previous.dq[j] - .0005 * (current.ddq[j] + previous.ddq[j])) > 1e-5)
                    throw std::runtime_error("Latest 1 ms Servo derivative discontinuity");
            }
            check(current);
            previous = current;
            if (stopping && latest->stopped()) break;
        }
        if (!latest->stopped()) throw std::runtime_error("Latest Servo did not stop");
        std::cout << "Latest 1 ms Servo passed: steps=" << count << " mean_us=" << total_us/count
                  << " max_us=" << max_us << " (wall time includes scheduler jitter)\n";
        // Captured after compliant grasp: using the old wheel reference (0, -.085)
        // adds its drift to every goal; an unweighted wrist also hits +/-60 degrees.
        JointFrame compliant;
        compliant.angle = .15734908702110342;
        compliant.displacement = -.08848852357910701;
        compliant.q = {-.2441592211, 1.5483223663, -1.4718264843, 1.5707879272,
                       -.1045421521, .1352043964, .1238566744,
                       .2951912808, 1.5256245130, 1.2800535572, 1.7130233472,
                       -.0972914518, -.0946703438, -.4992555569};
        // Verify acceleration against a central difference of the weighted
        // velocity field, independently of Ruckig's within-tick jerk switches.
        std::array<double, 7> q{}, dq{}, ddq{}, plus{}, minus{}, unused{};
        std::copy_n(compliant.q.begin(), 7, q.begin());
        Eigen::Matrix<double, 6, 1> twist, acceleration;
        twist << .1, .2, .3, .4, .5, .6;
        acceleration << -.1, .2, -.3, .4, -.5, .6;
        if (!kin->jointDerivatives(Side::Left, q, twist, acceleration, dq, ddq))
            throw std::runtime_error("Weighted derivatives failed");
        auto forward = q, backward = q;
        constexpr double epsilon = 1e-5;
        for (int j = 0; j < 7; ++j) {
            forward[j] += epsilon * dq[j]; backward[j] -= epsilon * dq[j];
        }
        if (!kin->jointDerivatives(Side::Left, forward, twist + epsilon * acceleration, acceleration, plus, unused) ||
            !kin->jointDerivatives(Side::Left, backward, twist - epsilon * acceleration, acceleration, minus, unused))
            throw std::runtime_error("Weighted derivative difference failed");
        for (int j = 0; j < 7; ++j)
            if (std::abs(ddq[j] - (plus[j] - minus[j]) / (2 * epsilon)) > 1e-6)
                throw std::runtime_error("Weighted acceleration disagrees with velocity derivative");
        // Real FOLLOWING -> CONTROL failure from 2026-10-15: approximately 4 cm
        // rigid-geometry residual. Plan directly from the accepted measured hold.
        compliant.angle = 0;
        compliant.displacement = -.085;
        compliant.q = {-.25096584822178886, 1.5382299945629172, -1.5837275451522743,
                       1.6235650262136645, -.25219075584936534, -.082772644576316778, .019892375235903266,
                       .40945282837933089, 1.5105374468628645, 1.2883907034296482,
                       1.8268081329130856, -.3000940996410566, -.29067593697243499, -.18303722413999388};
        auto captured = options.servo(compliant);
        previous = compliant;
        const double takeover_v = std::min(config["wheel_angular_speed"].as<double>(),
            *std::min_element(settings.speed.begin(), settings.speed.end()));
        const double takeover_a = config["wheel_angular_acceleration"].as<double>();
        const double takeover_j = config["wheel_angular_jerk"].as<double>();
        int arrived_tick = -1;
        double takeover_peak_v = 0;
        // Replace the gateway target during takeover; the final target is NOT
        // center. Response must use current CONTROL settings, without .05 rad/s.
        constexpr double final_angle = -.07, final_displacement = -.095;
        for (int tick = 0; tick < 4000; ++tick) {
            const auto current = captured->step({tick < 200 ? .1 : final_angle,
                tick < 200 ? -.085 : final_displacement, 1., 1, false});
            for (int j = 0; j < 14; ++j) {
                takeover_peak_v = std::max(takeover_peak_v, std::abs(current.dq[j]));
                // A jerk switch inside a sample bounds the trapezoid integration
                // error by J*dt^2/4; retain the same physical continuity check.
                if (std::abs(current.q[j] - previous.q[j] - .0005 * (current.dq[j] + previous.dq[j])) > 1e-7 ||
                    std::abs(current.dq[j] - previous.dq[j] - .0005 * (current.ddq[j] + previous.ddq[j])) > takeover_j * .001 * .001 / 4 + 1e-9 ||
                    std::abs(current.dq[j]) > takeover_v + 1e-8 || std::abs(current.ddq[j]) > takeover_a + 1e-8 ||
                    std::abs(current.ddq[j] - previous.ddq[j]) / .001 > takeover_j + 1e-6)
                    throw std::runtime_error("Captured hold takeover violated continuous joint limits");
                if (tick == 0 && std::abs(current.q[j] - compliant.q[j]) > takeover_j * 1e-9 / 6 + 1e-12)
                    throw std::runtime_error("Takeover first frame jumped away from measured hold");
            }
            check(current);
            previous = current;
            if (tick >= 200 && arrived_tick < 0 && captured->stopped() &&
                std::abs(current.angle - final_angle) < 1e-9 &&
                std::abs(current.displacement - final_displacement) < 1e-9) arrived_tick = tick;
        }
        if (arrived_tick < 0 || arrived_tick >= 2500 || takeover_peak_v <= .1)
            throw std::runtime_error("Takeover retained slow response: arrival_ms=" +
                std::to_string(arrived_tick + 1) + " peak_velocity=" + std::to_string(takeover_peak_v));
        for (int side = 0; side < 2; ++side) {
            pinocchio::SE3 actual;
            std::copy_n(previous.q.begin() + side * 7, 7, q.begin());
            if (!kin->solveFk(Side(side), q, actual) ||
                (actual.translation() - target(side, final_angle, final_displacement).translation()).norm() > 1e-7 ||
                rotationError(actual, target(side, final_angle, final_displacement)) > 1e-7)
                throw std::runtime_error("Captured hold failed to reach latest nonzero gateway target");
        }
        std::cout << "Captured hold / CONTROL response / nonzero target passed: arrival_ms="
                  << arrived_tick + 1 << " peak_velocity=" << takeover_peak_v << '\n';
        compliant = nominal(pose(compliant.angle, compliant.displacement));
        auto sweep = options.servo(compliant);
        previous = compliant;
        for (double displacement : {-.09949, -.085, -.17, 0.}) {
            for (double angle : {-.87266, .87266}) {
                for (int tick = 0; tick < 2000; ++tick) {
                    const auto current = sweep->step({angle, displacement, 1., 1, false});
                    check(current);
                    previous = current;
                }
                if (std::abs(previous.angle - angle) > 1e-8 ||
                    std::abs(previous.displacement - displacement) > 1e-8)
                    throw std::runtime_error("Compliant Servo did not reach full-range goal");
            }
        }
        std::cout << "Compliant grasp +/-50 degree and full pitch sweep passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

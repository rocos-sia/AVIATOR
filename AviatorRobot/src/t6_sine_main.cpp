#include "aviator/Aviator.hpp"
#include "t6.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
constexpr double dt = .01;
constexpr double wheel_command_period = .02;
constexpr double pi = 3.14159265358979323846;

struct WheelSample {
    aviator::t6::Vec2 x{}, xdot{};
    int knot = 0;
};

// The wheel planner publishes position and velocity at 50 Hz. Cubic Hermite
// interpolation supplies the 100 Hz policy loop and matches both quantities
// exactly at every planner knot.
struct SineWheelReference {
    aviator::t6::Vec2 center{}, amplitude{};
    double frequency = .1;

    WheelSample sample(double time) const {
        const int knot = static_cast<int>(std::floor((time + 1e-10) / wheel_command_period));
        const double t0 = knot * wheel_command_period;
        const double u = std::clamp((time - t0) / wheel_command_period, 0., 1.);
        const double u2 = u * u, u3 = u2 * u;
        const double h00 = 2 * u3 - 3 * u2 + 1;
        const double h10 = u3 - 2 * u2 + u;
        const double h01 = -2 * u3 + 3 * u2;
        const double h11 = u3 - u2;
        const double dh00 = (6 * u2 - 6 * u) / wheel_command_period;
        const double dh10 = 3 * u2 - 4 * u + 1;
        const double dh01 = (-6 * u2 + 6 * u) / wheel_command_period;
        const double dh11 = 3 * u2 - 2 * u;
        WheelSample out;
        out.knot = knot;
        for (int axis = 0; axis < 2; ++axis) {
            const double omega = 2 * pi * frequency;
            const double y0 = center[axis] + amplitude[axis] * std::sin(omega * t0);
            const double y1 = center[axis] + amplitude[axis] *
                std::sin(omega * (t0 + wheel_command_period));
            const double v0 = amplitude[axis] * omega * std::cos(omega * t0);
            const double v1 = amplitude[axis] * omega *
                std::cos(omega * (t0 + wheel_command_period));
            out.x[axis] = h00 * y0 + h10 * wheel_command_period * v0 +
                          h01 * y1 + h11 * wheel_command_period * v1;
            out.xdot[axis] = dh00 * y0 + dh10 * v0 + dh01 * y1 + dh11 * v1;
        }
        return out;
    }
};

void verify_wheel_reference(const SineWheelReference& reference, double duration) {
    constexpr double epsilon = 1e-7;
    for (int knot = 1; knot <= static_cast<int>(std::ceil(duration / wheel_command_period)); ++knot) {
        const double time = knot * wheel_command_period;
        const auto left = reference.sample(time - epsilon);
        const auto right = reference.sample(time + epsilon);
        for (int axis = 0; axis < 2; ++axis) {
            if (std::abs((left.x[axis] + epsilon * left.xdot[axis]) -
                         (right.x[axis] - epsilon * right.xdot[axis])) > 1e-8 ||
                std::abs(left.xdot[axis] - right.xdot[axis]) > 1e-6)
                throw std::runtime_error("Wheel reference position/velocity is discontinuous");
        }
    }
}

struct Metrics {
    int samples = 0;
    double theta_sq = 0, slide_sq = 0, joint_sq = 0;
    double theta_max = 0, slide_max = 0, joint_max = 0;
    double residual_max = 0, min_clearance = 1e9;
    double solve_ms_sum = 0, solve_ms_max = 0;
    std::vector<double> solve_ms;
    void add(double theta_error, double slide_error, double joint_error,
             double residual, double clearance, double compute_ms) {
        ++samples;
        theta_sq += theta_error * theta_error;
        slide_sq += slide_error * slide_error;
        joint_sq += joint_error * joint_error;
        theta_max = std::max(theta_max, std::abs(theta_error));
        slide_max = std::max(slide_max, std::abs(slide_error));
        joint_max = std::max(joint_max, joint_error);
        residual_max = std::max(residual_max, residual);
        min_clearance = std::min(min_clearance, clearance);
        solve_ms_sum += compute_ms;
        solve_ms_max = std::max(solve_ms_max, compute_ms);
        solve_ms.push_back(compute_ms);
    }
};

double number(const char* value, const char* name, double lo, double hi) {
    const double parsed = std::stod(value);
    if (!std::isfinite(parsed) || parsed < lo || parsed > hi)
        throw std::invalid_argument(std::string("Invalid ") + name);
    return parsed;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 4 || argc > 10) {
        std::cerr << "usage: aviatorT6Sine MODEL.onnx LUT_DIRECTORY OUTPUT.csv "
                     "[duration_s=10] [theta_amp_rad=0.1] [slide_amp_m=0.01] "
                     "[frequency_hz=0.1] [bus=0] [max_joint_speed=1.5]\n";
        return 2;
    }
    std::signal(SIGINT, [](int) { interrupted = 1; });
    std::signal(SIGTERM, [](int) { interrupted = 1; });
    try {
        const double duration = argc > 4 ? number(argv[4], "duration", .1, 60) : 10;
        const double angle_amp = argc > 5 ? number(argv[5], "theta amplitude", -.872665, .872665) : .1;
        const double slide_amp = argc > 6 ? number(argv[6], "slide amplitude", -.08, .08) : .01;
        const double frequency = argc > 7 ? number(argv[7], "frequency", .01, 1) : .1;
        const int bus = argc > 8 ? std::stoi(argv[8]) : 0;
        if (bus < 0) throw std::invalid_argument("Invalid bus");
        const double max_joint_speed = argc > 9 ?
            number(argv[9], "joint speed limit", .1, 6.28) : 1.5;
        aviator::t6::FeedbackLimits limits;
        limits.max_rms_joint_error = .02;
        limits.max_joint_error = .05;
        limits.max_phase_change = .025;
        limits.continuity_weight = .001;
        limits.max_joint_speed = max_joint_speed;
        limits.min_clearance = .005;
        limits.time_tolerance = .002;
        aviator::t6::T6 policy(argv[1], argv[2], limits);
        aviator::Aviator robot(std::string(AVIATOR_CONFIG_DIR) + "/aviator.yaml", bus);
        robot.Init();
        std::atomic<bool> monitoring{true};
        std::thread monitor([&] {
            while (monitoring) {
                if (interrupted) robot.Stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        std::vector<std::string> rows;
        rows.reserve(static_cast<std::size_t>(duration / dt) + 2);
        Metrics metrics;
        std::string failure;
        double start_time = -1;
        aviator::t6::Vec2 center{};
        aviator::t6::Joints last_command{};
        bool have_last_command = false;
        try {
            robot.Enable();
            robot.ApproachHandles();
            robot.LockHandles();
            robot.MoveWheel(0, -.08, 4);
            const auto settled = robot.GetStatus();
            center = {0., -.08};
            const SineWheelReference wheel_reference{center, {angle_amp, slide_amp}, frequency};
            verify_wheel_reference(wheel_reference, duration);
            std::cout << "sin center theta=" << center[0] << " s=" << center[1]
                      << "; initial feedback=" << settled.angle << "," << settled.displacement
                      << "; amp=" << angle_amp << "," << slide_amp
                      << "; frequency=" << frequency << " Hz"
                      << "; wheel_command_frequency=50 Hz"
                      << "; max_joint_speed=" << max_joint_speed << " rad/s\n";
            robot.RunJointFeedback(duration, [&](const rocos_mujoco::aviator::Feedback& f)
                                       -> std::optional<aviator::JointTarget> {
                if (interrupted) throw std::runtime_error("Interrupted");
                if (start_time < 0) start_time = f.time;
                const double t = f.time - start_time;
                const auto wheel_ref = wheel_reference.sample(t);
                const auto wheel_next = wheel_reference.sample(t + dt);
                const auto x_ref = wheel_ref.x;
                const auto x_next = wheel_next.x;
                aviator::t6::Feedback measured;
                measured.x = {f.angle, f.displacement};
                measured.xdot = {f.velocity[0], f.velocity[1]};
                measured.time = f.time;
                std::copy(std::begin(f.joints), std::end(f.joints), measured.q.begin());
                const auto compute_start = std::chrono::steady_clock::now();
                const auto result = policy(measured, x_next);
                const double compute_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - compute_start).count();
                double joint_error = 0;
                if (have_last_command)
                    for (int j = 0; j < 14; ++j)
                        joint_error = std::max(joint_error,
                                               std::abs(measured.q[j] - last_command[j]));
                const double residual = std::max(result.estimate.rms_joint_error[0],
                                                 result.estimate.rms_joint_error[1]);
                const double theta_error = measured.x[0] - x_ref[0];
                const double slide_error = measured.x[1] - x_ref[1];
                metrics.add(theta_error, slide_error, joint_error, residual,
                            result.clearance, compute_ms);
                std::ostringstream row;
                row << std::setprecision(10) << t << ',' << f.time << ','
                    << x_ref[0] << ',' << x_ref[1] << ','
                    << wheel_ref.xdot[0] << ',' << wheel_ref.xdot[1] << ','
                    << wheel_ref.knot << ','
                    << measured.x[0] << ',' << measured.x[1] << ','
                    << measured.xdot[0] << ',' << measured.xdot[1] << ','
                    << result.estimate.phi[0] << ',' << result.estimate.phi[1] << ','
                    << result.estimate.rms_joint_error[0] << ','
                    << result.estimate.rms_joint_error[1] << ','
                    << result.action[0] << ',' << result.action[1] << ','
                    << result.executed_action[0] << ',' << result.executed_action[1] << ','
                    << result.intervened << ','
                    << result.candidate_phi[0] << ',' << result.candidate_phi[1] << ','
                    << result.clearance << ',' << result.max_joint_speed << ',' << compute_ms << ','
                    << joint_error << ',' << f.position_error[0] << ',' << f.position_error[1]
                    << ',' << f.rotation_error[0] << ',' << f.rotation_error[1] << ','
                    << aviator::t6::status_name(result.status) << ','
                    << static_cast<int>(result.stage) << ',' << result.has_command;
                for (double q : measured.q) row << ',' << q;
                for (double q : result.q) row << ',' << q;
                rows.push_back(row.str());
                if (result.status != aviator::t6::Status::ok)
                    throw std::runtime_error(std::string("T6 rejected step at t=") +
                                             std::to_string(t) + " s: " +
                                             aviator::t6::status_name(result.status));
                if (!result.has_command) return std::nullopt; // initial phase acquisition
                last_command = result.q;
                have_last_command = true;
                return aviator::JointTarget{result.q, x_next};
            });
            robot.UnlockHandles();
            robot.Disable();
        } catch (const std::exception& e) {
            failure = e.what();
            robot.Stop();
            try { robot.UnlockHandles(); robot.Disable(); } catch (...) {}
        }
        monitoring = false;
        monitor.join();
        std::ofstream csv(argv[3]);
        if (!csv) throw std::runtime_error("Cannot open result CSV");
        csv << "t,sim_time,theta_ref,s_ref,theta_dot_ref,s_dot_ref,wheel_knot,"
               "theta_meas,s_meas,theta_dot,s_dot,"
               "phi_L,phi_R,phase_rms_L,phase_rms_R,action_L,action_R,"
               "executed_action_L,executed_action_R,intervened,"
               "candidate_phi_L,candidate_phi_R,lut_clearance,max_joint_speed,compute_ms,"
               "joint_tracking_max,grasp_position_L,grasp_position_R,"
               "grasp_rotation_L,grasp_rotation_R,status,stage,has_command";
        for (int j = 0; j < 14; ++j) csv << ",q_meas_" << j;
        for (int j = 0; j < 14; ++j) csv << ",q_cmd_" << j;
        csv << '\n';
        for (const auto& row : rows) csv << row << '\n';
        csv.close();
        std::cout << "samples=" << metrics.samples << " csv=" << argv[3] << '\n';
        if (metrics.samples) {
            std::sort(metrics.solve_ms.begin(), metrics.solve_ms.end());
            const std::size_t p99 = static_cast<std::size_t>(
                std::ceil(.99 * metrics.solve_ms.size())) - 1;
            std::cout << std::setprecision(6)
                      << "theta_rms=" << std::sqrt(metrics.theta_sq / metrics.samples)
                      << " rad theta_max=" << metrics.theta_max << " rad\n"
                      << "slide_rms=" << std::sqrt(metrics.slide_sq / metrics.samples)
                      << " m slide_max=" << metrics.slide_max << " m\n"
                      << "joint_tracking_rms=" << std::sqrt(metrics.joint_sq / metrics.samples)
                      << " rad joint_tracking_max=" << metrics.joint_max << " rad\n"
                      << "phase_reconstruction_max=" << metrics.residual_max
                      << " rad lut_clearance_min=" << metrics.min_clearance << " m\n"
                      << "t6_compute_mean=" << metrics.solve_ms_sum / metrics.samples
                      << " ms t6_compute_p99=" << metrics.solve_ms[p99]
                      << " ms t6_compute_max=" << metrics.solve_ms_max << " ms\n";
        }
        if (!failure.empty()) {
            std::cerr << "TEST STOPPED: " << failure << '\n';
            return 1;
        }
        std::cout << "TEST COMPLETED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "TEST ERROR: " << e.what() << '\n';
        return 1;
    }
}

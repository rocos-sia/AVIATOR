#include "aviator/Aviator.hpp"
#include "joystick.hpp"
#include "t6.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
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

// ---- joystick -> wheel mapping (locked spec) -------------------------------
constexpr double theta_max = 0.872665; // rad, axis 0 full range -> ±50°
constexpr double s_neutral = -0.080;   // m, axis 1 centre -> −80 mm
constexpr double s_full = -0.159;      // m, axis 1 negative limit -> −159 mm (1 mm LUT margin)
constexpr double s_push = -0.001;      // m, axis 1 positive limit -> −1 mm (1 mm LUT margin)
constexpr double theta_rate = 1.5;     // rad/s, slew limit on θ_ref
constexpr double deadzone = 0.05;      // fraction of full axis range
constexpr double ema_alpha = 0.5;      // per-knot EMA smoothing coefficient (0..1)
constexpr int axis_theta = 0;          // joystick axis index -> θ
constexpr int axis_s = 1;              // joystick axis index -> s
constexpr double axis_full = 32767.0;  // full-scale axis magnitude

struct WheelSample {
    aviator::t6::Vec2 x{}, xdot{};
    int knot = 0;
    int raw_theta = 0, raw_s = 0;
};

struct TaskTarget {
    aviator::t6::Vec2 x{};
    double scale = 0;
};

// Limit the requested wheel step using the current measured joints and LUT.
// The actor may still change phase, so T6 performs the final safety check.
TaskTarget feasible_task_target(const aviator::t6::Lookup& lookup,
                                const aviator::t6::Feedback& measured,
                                aviator::t6::Vec2 requested,
                                const aviator::t6::FeedbackLimits& limits) {
    aviator::t6::PhaseEstimate phase;
    try {
        phase = lookup.estimate_phase(measured.x, measured.q);
    } catch (const std::out_of_range&) {
        return {measured.x, 0};
    }
    constexpr double scales[] = {1, .5, .25, .125, .0625, .03125,
                                 .015625, .0078125, .00390625, 0};
    for (double scale : scales) {
        const aviator::t6::Vec2 x{
            measured.x[0] + scale * (requested[0] - measured.x[0]),
            measured.x[1] + scale * (requested[1] - measured.x[1])};
        aviator::t6::LookupResult sample;
        try {
            sample = lookup.query(x, phase.phi);
        } catch (const std::out_of_range&) {
            continue;
        }
        if (sample.branch >= 2 || sample.clearance < limits.min_clearance ||
            sample.joint_margin < 0 ||
            sample.margin_minus[0] < 0 || sample.margin_minus[1] < 0 ||
            sample.margin_plus[0] < 0 || sample.margin_plus[1] < 0)
            continue;
        double speed = 0;
        for (int joint = 0; joint < 14; ++joint)
            speed = std::max(speed,
                             std::abs(sample.q[joint] - measured.q[joint]) / dt);
        if (speed <= .8 * limits.max_joint_speed)
            return {x, scale};
    }
    return {measured.x, 0};
}

// Deadzone + renormalise: raw in [-axis_full, axis_full], dead in the same
// units; output is [-1, 1] with the dead band mapped to 0.
double deadzone_normalize(double raw, double dead) {
    const double a = std::abs(raw);
    if (a <= dead) return 0.0;
    return std::copysign((a - dead) / (axis_full - dead), raw);
}

const char* stage_name(aviator::t6::CheckStage stage) {
    switch (stage) {
    case aviator::t6::CheckStage::none: return "none";
    case aviator::t6::CheckStage::reset: return "reset";
    case aviator::t6::CheckStage::feedback: return "feedback";
    case aviator::t6::CheckStage::target: return "target";
    case aviator::t6::CheckStage::policy_command: return "policy_command";
    }
    return "unknown";
}

std::string joint_name(int j) {
    if (j < 0 || j >= 14) return "?";
    return std::string(j < 7 ? "L" : "R") + std::to_string(j % 7 + 1);
}

// Detailed rejection dump: the RL proposal was infeasible. Print every field
// the fail-closed check populated so the reason is unambiguous (throttled by
// the caller to avoid flooding while a fault persists).
void print_rejection(const aviator::t6::StepResult& result,
                     const aviator::t6::Feedback& measured,
                     const aviator::t6::Vec2& x_ref, double t) {
    std::cerr << "[t6] REJECTED t=" << t << " s: "
              << aviator::t6::status_name(result.status)
              << " (stage=" << stage_name(result.stage) << ")\n";
    std::cerr << "  x_meas=(" << measured.x[0] << ", " << measured.x[1] << ")"
              << "  x_ref=(" << x_ref[0] << ", " << x_ref[1] << ")";
    if (result.stage == aviator::t6::CheckStage::policy_command)
        std::cerr << "  candidate_x=(" << result.candidate_x[0] << ", "
                  << result.candidate_x[1] << ")";
    std::cerr << '\n';
    std::cerr << "  phi=(" << result.phi[0] << ", " << result.phi[1] << ")";
    if (result.stage == aviator::t6::CheckStage::policy_command)
        std::cerr << "  candidate_phi=(" << result.candidate_phi[0] << ", "
                  << result.candidate_phi[1] << ")";
    std::cerr << '\n';
    if (result.has_action)
        std::cerr << "  action=(" << result.action[0] << ", " << result.action[1] << ")"
                  << "  executed=(" << result.executed_action[0] << ", "
                  << result.executed_action[1] << ")  intervened=" << result.intervened << '\n';
    std::cerr << "  clearance=" << result.clearance << " m";
    if (result.max_joint_speed > 0)
        std::cerr << "  max_joint_speed=" << result.max_joint_speed << " rad/s"
                  << "  limiting_joint=" << joint_name(result.limiting_joint);
    std::cerr << '\n';
    if (result.has_evaluated_sample) {
        const auto& s = result.evaluated_sample;
        std::cerr << "  sample: clearance=" << s.clearance << " m  branch=" << int(s.branch)
                  << "  joint_margin=" << s.joint_margin
                  << "  phi_margin_minus=(" << s.margin_minus[0] << ", " << s.margin_minus[1] << ")"
                  << "  phi_margin_plus=(" << s.margin_plus[0] << ", " << s.margin_plus[1] << ")\n";
    }
    std::cerr << "  estimate: rms=(" << result.estimate.rms_joint_error[0] << ", "
              << result.estimate.rms_joint_error[1] << ")  max=("
              << result.estimate.max_joint_error[0] << ", "
              << result.estimate.max_joint_error[1] << ")\n";
}

// Position-mapped, rate-limited joystick reference. Publishes one knot every
// 20 ms; the 100 Hz policy loop linearly continues each knot at the slew
// velocity (the slew, not a Hermite look-ahead, is the smoothing: a joystick
// has no future value to interpolate toward).
struct JoystickWheelReference {
    joystick::Device device;
    double s_rate;
    bool raw; // AVIATOR_JOYSTICK_RAW: bypass EMA/deadzone/slew, reference = joystick position
    aviator::t6::Vec2 knot_pos{{0., s_neutral}};
    aviator::t6::Vec2 knot_vel{};
    int last_knot = -1;
    double ema_theta = 0., ema_s = 0.;
    int raw_theta = 0, raw_s = 0;
    bool warned_disconnect = false;

    JoystickWheelReference(std::string path, double s_rate_)
        : device(std::move(path), false), s_rate(s_rate_) {
        const char *raw_env = std::getenv("AVIATOR_JOYSTICK_RAW");
        raw = raw_env && raw_env[0] != '\0' && raw_env[0] != '0';
    }

    void update_knot() {
        const auto& state = device.poll();
        aviator::t6::Vec2 desired{0., s_neutral};
        if (state.connected &&
            state.axes.size() > static_cast<std::size_t>(std::max(axis_theta, axis_s))) {
            raw_theta = state.axes[axis_theta];
            raw_s = state.axes[axis_s];
            if (raw) {
                // Direct passthrough: raw axis -> position, no EMA/deadzone.
                desired[0] = std::clamp(raw_theta / axis_full, -1., 1.) * theta_max;
                desired[1] = s_neutral +
                             std::clamp(raw_s / axis_full, -1., 1.) * (s_push - s_neutral);
            } else {
                ema_theta = ema_alpha * state.axes[axis_theta] + (1. - ema_alpha) * ema_theta;
                ema_s = ema_alpha * state.axes[axis_s] + (1. - ema_alpha) * ema_s;
                const double n_theta = deadzone_normalize(ema_theta, deadzone * axis_full);
                const double n_s = deadzone_normalize(ema_s, deadzone * axis_full);
                desired[0] = std::clamp(n_theta, -1., 1.) * theta_max;
                desired[1] = s_neutral + std::clamp(n_s, -1., 1.) * (s_push - s_neutral);
            }
        } else if (!warned_disconnect) {
            std::cerr << "[joystick] not connected (" << state.error
                      << "); holding neutral θ=0, s=" << s_neutral << " m\n";
            warned_disconnect = true;
        }
        // Rate-limited slew from where the reference sits at this knot boundary.
        // In raw mode the rate limit is removed: the reference jumps straight to
        // the joystick position (still linear-interpolated over one 20 ms knot).
        const aviator::t6::Vec2 current{knot_pos[0] + knot_vel[0] * wheel_command_period,
                                        knot_pos[1] + knot_vel[1] * wheel_command_period};
        aviator::t6::Vec2 next{};
        if (raw) {
            next = desired;
        } else {
            const double max_theta = theta_rate * wheel_command_period;
            const double max_s = s_rate * wheel_command_period;
            next[0] = current[0] + std::clamp(desired[0] - current[0], -max_theta, max_theta);
            next[1] = current[1] + std::clamp(desired[1] - current[1], -max_s, max_s);
        }
        knot_vel[0] = (next[0] - current[0]) / wheel_command_period;
        knot_vel[1] = (next[1] - current[1]) / wheel_command_period;
        knot_pos = current;
        // Belt-and-suspenders: the slew already keeps the reference inside the
        // LUT box, but clamp to the safe band with the accepted 1 mm margins.
        knot_pos[0] = std::clamp(knot_pos[0], -theta_max, theta_max);
        knot_pos[1] = std::clamp(knot_pos[1], s_full, s_push);
    }

    WheelSample sample(double time) {
        const int knot = static_cast<int>(std::floor((time + 1e-10) / wheel_command_period));
        if (knot != last_knot) {
            update_knot();
            last_knot = knot;
        }
        const double t0 = knot * wheel_command_period;
        const double u = std::clamp((time - t0) / wheel_command_period, 0., 1.);
        WheelSample out;
        out.knot = knot;
        out.raw_theta = raw_theta;
        out.raw_s = raw_s;
        for (int axis = 0; axis < 2; ++axis) {
            out.x[axis] = knot_pos[axis] + knot_vel[axis] * u * wheel_command_period;
            out.xdot[axis] = knot_vel[axis];
        }
        return out;
    }
};

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
    if (argc < 4 || argc > 9) {
        std::cerr << "usage: aviatorT6Joystick MODEL.onnx LUT_DIRECTORY OUTPUT.csv "
                     "[duration_s=60, 0=infinite] [joystick_path=/dev/input/js0] [bus=0] "
                     "[max_joint_speed=1.5] [s_rate_m_s=0.04]\n";
        return 2;
    }
    std::signal(SIGINT, [](int) { interrupted = 1; });
    std::signal(SIGTERM, [](int) { interrupted = 1; });
    try {
        const double duration = argc > 4 ? number(argv[4], "duration", 0, 1e9) : 60;
        const std::string joystick_path = argc > 5 ? argv[5] : "/dev/input/js0";
        const int bus = argc > 6 ? std::stoi(argv[6]) : 0;
        if (bus < 0) throw std::invalid_argument("Invalid bus");
        const double max_joint_speed = argc > 7 ?
            number(argv[7], "joint speed limit", .1, 6.28) : 1.5;
        const double s_rate = argc > 8 ?
            number(argv[8], "s slew rate", .001, .5) : .04;
        aviator::t6::FeedbackLimits limits;
        limits.max_rms_joint_error = .02;
        limits.max_joint_error = .05;
        limits.max_phase_change = .025;
        limits.continuity_weight = .001;
        limits.max_joint_speed = max_joint_speed;
        limits.min_clearance = .005;
        limits.time_tolerance = .005;
        aviator::t6::T6 policy(argv[1], argv[2], limits);
        aviator::t6::Lookup target_lookup(argv[2]);
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
        rows.reserve(duration > 0 ? static_cast<std::size_t>(duration / dt) + 2
                                  : static_cast<std::size_t>(600 / dt) + 2);
        Metrics metrics;
        std::string failure;
        double start_time = -1;
        int hold_count = 0;
        bool rearm = false;
        double last_hint_time = -1e9;
        aviator::t6::Joints last_command{};
        bool have_last_command = false;
        try {
            robot.Enable();
            robot.ApproachHandles();
            robot.LockHandles();
            robot.MoveWheel(0, -.08, 4);
            const auto settled = robot.GetStatus();
            JoystickWheelReference wheel_reference{joystick_path, s_rate};
            const auto& joy = wheel_reference.device.poll();
            if (joy.connected)
                std::cout << "joystick: " << joy.name << " axes=" << joy.axes.size()
                          << " buttons=" << joy.buttons.size()
                          << "; axis " << axis_theta << " -> θ, axis " << axis_s << " -> s"
                          << "; initial raw=("
                          << (joy.axes.size() > axis_theta ? joy.axes[axis_theta] : 0) << ", "
                          << (joy.axes.size() > axis_s ? joy.axes[axis_s] : 0) << ")\n";
            else
                std::cerr << "joystick not connected (" << joy.error << "); neutral hold only\n";
            wheel_reference.knot_pos = {settled.angle, settled.displacement};
            std::cout << "initial wheel theta=" << settled.angle
                      << " s=" << settled.displacement
                      << "; neutral θ=0 s=" << s_neutral << " m, full-pull s=" << s_full
                      << " m, full-push s=" << s_push
                      << " m; slew θ≤" << theta_rate << " rad/s, s≤" << s_rate
                      << " m/s; wheel_command_frequency=50 Hz"
                      << "; max_joint_speed=" << max_joint_speed << " rad/s"
                      << (wheel_reference.raw ? " [RAW: slew/EMA/deadzone bypassed]" : "")
                      << "\n";
            robot.RunJointFeedback(duration, [&](const rocos_mujoco::aviator::Feedback& f)
                                       -> std::optional<aviator::JointTarget> {
                if (interrupted) throw std::runtime_error("Interrupted");
                if (start_time < 0) start_time = f.time;
                const double t = f.time - start_time;
                const auto wheel_ref = wheel_reference.sample(t);
                const auto wheel_next = wheel_reference.sample(t + dt);
                const auto x_ref = wheel_ref.x;
                aviator::t6::Feedback measured;
                measured.x = {f.angle, f.displacement};
                measured.xdot = {f.velocity[0], f.velocity[1]};
                measured.time = f.time;
                std::copy(std::begin(f.joints), std::end(f.joints), measured.q.begin());
                const auto compute_start = std::chrono::steady_clock::now();
                const auto task_target = feasible_task_target(
                    target_lookup, measured, wheel_next.x, limits);
                const auto x_next = task_target.x;
                const auto result = policy(measured, x_next, rearm);
                rearm = result.status != aviator::t6::Status::ok;
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
                    << wheel_ref.knot << ',' << wheel_ref.raw_theta << ',' << wheel_ref.raw_s << ','
                    << x_next[0] << ',' << x_next[1] << ',' << task_target.scale << ','
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
                // hold-last-safe: on any rejection, send no command (p.hold()) and
                // hint on screen, then re-acquire phase on the next cycle.
                if (result.status != aviator::t6::Status::ok) {
                    if (result.new_failure && f.time - last_hint_time >= 1.0) {
                        print_rejection(result, measured, x_ref, t);
                        std::cerr << "[t6] hold (rearm next cycle)\n";
                        last_hint_time = f.time;
                    }
                    ++hold_count;
                    return std::nullopt;
                }
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
        csv << "t,sim_time,theta_ref,s_ref,theta_dot_ref,s_dot_ref,wheel_knot,axis0_raw,axis1_raw,"
               "theta_cmd,s_cmd,task_scale,"
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
        std::cout << "samples=" << metrics.samples << " hold_cycles=" << hold_count
                  << " csv=" << argv[3] << '\n';
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
                      << "control_compute_mean=" << metrics.solve_ms_sum / metrics.samples
                      << " ms control_compute_p99=" << metrics.solve_ms[p99]
                      << " ms control_compute_max=" << metrics.solve_ms_max << " ms\n";
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

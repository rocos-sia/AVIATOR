#include "aviator/Aviator.hpp"
#include "t6.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>

// A small, isolated left-joint-1 step while the wheel is locked at (0, -80 mm).
// The simulator's optional 1 ms trace records when its actuator sees the target.
int main(int argc, char** argv) {
    if (argc != 4 && argc != 5 && argc != 6) {
        std::cerr << "usage: aviator_joint_step_probe CONFIG.yaml BUS OUTPUT.csv "
                     "[LUT_DIRECTORY [WHEEL_STEP_RAD]]\n";
        return 2;
    }
    try {
        const double wheel_step = argc == 6 ? std::stod(argv[5]) : .005;
        if (!(wheel_step >= .0001 && wheel_step <= .01))
            throw std::invalid_argument("wheel step must be in [0.0001, 0.01] rad");
        aviator::Aviator robot(argv[1], std::stoi(argv[2]));
        robot.Init();
        robot.Enable();
        robot.ApproachHandles();
        robot.LockHandles();
        robot.MoveWheel(0, -.08, 4);
        std::ofstream out(argv[3]);
        if (!out) throw std::runtime_error("Cannot open step probe CSV");
        out << std::setprecision(17)
            << "t,feedback_sim_time,callback_monotonic_time,q0_meas,q0_command,theta,s\n";
        double first_time = -1, base_q0 = 0;
        double base_angle = 0, base_s = 0;
        std::array<double, 14> base{};
        std::array<double, 14> stepped{};
        const bool coordinated = argc >= 5;
        std::optional<aviator::t6::Lookup> lookup;
        if (coordinated) lookup.emplace(argv[4]);
        robot.RunJointFeedback(.8, [&](const rocos_mujoco::aviator::Feedback& f)
                                          -> std::optional<aviator::JointTarget> {
            if (first_time < 0) {
                first_time = f.time;
                std::copy(std::begin(f.joints), std::end(f.joints), base.begin());
                base_q0 = base[0];
                base_angle = f.angle;
                base_s = f.displacement;
                stepped = base;
                if (coordinated) {
                    const auto phase = lookup->estimate_phase({f.angle, f.displacement}, base);
                    stepped = lookup->query({f.angle + wheel_step, f.displacement}, phase.phi).q;
                } else {
                    stepped[0] += .005;
                }
            }
            const double t = f.time - first_time;
            const auto& q = t >= .2 ? stepped : base;
            const auto x_target = t >= .2 && coordinated ? base_angle + wheel_step : f.angle;
            out << t << ',' << f.time << ',' << rocos_mujoco::aviator::monotonicTime()
                << ',' << f.joints[0] << ',' << q[0] << ','
                << f.angle << ',' << f.displacement << '\n';
            return aviator::JointTarget{q, {x_target, coordinated ? base_s : f.displacement}};
        });
        robot.UnlockHandles();
        robot.Disable();
        std::cout << "step_probe_completed base_q0=" << base_q0
                  << " target_q0=" << stepped[0]
                  << " coordinated=" << coordinated
                  << " wheel_step=" << wheel_step << " csv=" << argv[3] << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "step_probe_failed: " << e.what() << '\n';
        return 1;
    }
}

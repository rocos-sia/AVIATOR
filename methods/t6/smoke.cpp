#include "t6.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 5 && argc != 9) {
        std::cerr << "usage: t6_smoke ACTOR LUT theta s [theta_next s_next theta_dot_next s_dot_next]\n";
        return 2;
    }
    try {
        aviator::t6::Lookup lookup(argv[2]);
        aviator::t6::Actor actor(argv[1]);
        aviator::t6::Controller controller(lookup, actor);
        auto first = controller.reset({std::atof(argv[3]), std::atof(argv[4])}, {0, 0});
        std::cout << std::setprecision(17) << "reset_status=" << int(first.status)
                  << " clearance=" << first.clearance << " phi=" << first.phi[0]
                  << "," << first.phi[1] << "\n";
        if (first.status != aviator::t6::Status::ok) return 1;
        const auto action = actor.predict(controller.observation());
        std::cout << "action=" << action[0] << "," << action[1] << "\n";
        if (argc == 9) {
            const auto next = controller.step({std::atof(argv[5]), std::atof(argv[6])},
                                               {std::atof(argv[7]), std::atof(argv[8])});
            std::cout << "step_status=" << int(next.status) << " clearance=" << next.clearance
                      << " max_joint_speed=" << next.max_joint_speed << " phi=" << next.phi[0]
                      << "," << next.phi[1] << "\nq=";
            for (int i = 0; i < 14; ++i) std::cout << (i ? "," : "") << next.q[i];
            std::cout << "\n";
            if (next.status == aviator::t6::Status::ok) {
                const auto projected = lookup.estimate_phase(
                    {std::atof(argv[5]), std::atof(argv[6])}, next.q);
                std::cout << "projected_phi=" << projected.phi[0] << "," << projected.phi[1]
                          << " rms=" << projected.rms_joint_error[0] << ","
                          << projected.rms_joint_error[1] << "\n";
            }
            return next.status == aviator::t6::Status::ok ? 0 : 1;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n"; return 1;
    }
}

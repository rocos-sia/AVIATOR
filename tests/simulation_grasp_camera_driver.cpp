#include "RemoteLink.hpp"
#include <cmath>
#include <iostream>

int main(int argc, char** argv) try {
    if (argc != 2) throw std::runtime_error("Expected system config path");
    const auto config = aviator::loadMotionConfig(argv[1]);
    const auto expected = aviator::loadInitialWheel(config.robot);
    aviator::RemoteLink link(config);
    for (int attempt = 0; attempt < 2; ++attempt) {
        const auto pose = link.readCameraWheel("cockpit", 200000, 2000000, [&] { link.heartbeat(); });
        if (std::abs(pose[0] - expected.angle) > .005 ||
            std::abs(pose[1] - expected.displacement) > .005)
            throw std::runtime_error("Simulation grasp camera pose mismatch");
    }
    std::cout << "Simulation grasp camera acquired twice without a real camera\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}

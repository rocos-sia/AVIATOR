#include "aviator/GraspTools.hpp"
#include "aviator/backend.hpp"
#include "device_server.hpp"
#include <cmath>
#include <iostream>
using namespace aviator;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
void require(bool ok, const std::string& why) {
    if (!ok)
        throw std::runtime_error(why);
}
pinocchio::SE3 frame(const YAML::Node& n) {
    auto p = n["position"].as<std::vector<double>>(), q = n["quaternion"].as<std::vector<double>>();
    require(p.size() == 3 && q.size() == 4, "Invalid wxyz pose");
    for (double value : p)
        require(std::isfinite(value), "Invalid pose translation");
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    require(std::isfinite(rotation.norm()) && std::abs(rotation.norm() - 1) < 1e-6,
            "Invalid quaternion");
    return {rotation, Eigen::Vector3d(p[0], p[1], p[2])};
}
} // namespace
int main(int argc, char** argv) try {
    auto file = defaultSystemConfig();
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout
                << "manipulator [--config system.yaml]\nRokae hardware only; starts disabled.\n";
            return 0;
        }
        if (arg == "--config" && i + 1 < argc)
            file = argv[++i];
        else
            throw std::runtime_error("Unknown or incomplete option: " + arg);
    }
    const auto config = loadMotionConfig(file);
    const DeviceSettings settings(config);
    auto path = [&](const char* key) { return settings.path(key); };
    auto g = YAML::LoadFile(path("grasp").string());
    GraspGeometry geometry;
    for (int side = 0; side < 2; ++side)
        geometry.tools[side] = frame(toolFrameConfig(g, side));
    geometry.wheel_origin = frame(g["wheel_origin"]);
    geometry.handles[0] = frame(g["left"]);
    geometry.handles[1] = frame(g["right"]);
    RokaeConfig rk;
    auto r = settings.robot["rokae"];
    rk.grasp_mode = r["grasp_mode"].as<std::string>();
    rk.left_ip = r["left_ip"].as<std::string>();
    rk.right_ip = r["right_ip"].as<std::string>();
    rk.left_local_ip = r["left_local_ip"].as<std::string>();
    rk.right_local_ip = r["right_local_ip"].as<std::string>();
    auto stiffness = r["joint_stiffness"].as<std::vector<double>>();
    require(stiffness.size() == 7, "Seven stiffness values required");
    std::copy(stiffness.begin(), stiffness.end(), rk.joint_stiffness.begin());

    auto device = makeRokaeDataLink(path("urdf").string(), rk, geometry);
    device->setWheelReference(settings.initial_wheel.angle, settings.initial_wheel.displacement);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    return runDeviceServer(*device, config, settings, {}, interrupted);
} catch (const std::exception& e) {
    std::cerr << "manipulator: " << e.what() << '\n';
    return 1;
}

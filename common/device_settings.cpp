#include "device_server.hpp"
#include <cmath>
#include <urdf_parser/urdf_parser.h>
namespace aviator {
namespace {
void require(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
} // namespace
std::filesystem::path DeviceSettings::path(const char* key) const {
    const std::filesystem::path p = robot[key].as<std::string>();
    return p.is_absolute() ? p : directory / p;
}
DeviceSettings::DeviceSettings(const MotionConfig& config)
    : robot(YAML::LoadFile(config.robot.string())), directory(config.robot.parent_path()),
      initial_wheel(loadInitialWheel(config.robot)) {
    default_stiffness = robot["rokae"]["joint_stiffness"].as<std::array<double, 7>>();
    following_stiffness = robot["rokae"]["following_joint_stiffness"]
        .as<std::array<double, 7>>(default_stiffness);
    validateJointStiffness(default_stiffness);
    validateJointStiffness(following_stiffness);
    const auto urdf = urdf::parseURDFFile(path("urdf").string());
    require(bool(urdf), "Cannot load control URDF");

    for (int side = 0; side < 2; ++side)
        for (int j = 0; j < 7; ++j) {
            auto joint = urdf->getJoint(std::string("AR5-5_07") + (side ? "R" : "L") +
                                        "-W4C4A2_joint_" + std::to_string(j + 1));
            require(joint && joint->limits, "Missing joint limits");
            const int i = side * 7 + j;
            lower[i] = joint->limits->lower;
            upper[i] = joint->limits->upper;
            speed[i] = std::min(joint->limits->velocity, robot["joint_speed"].as<double>());
        }
    const auto posture = YAML::LoadFile(path("posture").string());
    for (int i : {1, 8}) {
        lower[i] = std::max(lower[i], posture["joint2_limits_deg"][0].as<double>() * M_PI / 180);
        upper[i] = std::min(upper[i], posture["joint2_limits_deg"][1].as<double>() * M_PI / 180);
    }
    braking = robot["stop_acceleration"].as<double>();
    require(std::isfinite(braking) && braking > 0, "Invalid braking acceleration");
}
} // namespace aviator

#pragma once
#include "motion.hpp"
#include <csignal>
#include <functional>
#include <yaml-cpp/yaml.h>
namespace aviator {
struct DeviceSettings {
    YAML::Node robot;
    std::filesystem::path directory;
    Joints lower{}, upper{}, speed{};
    double braking = 2;
    WheelReference initial_wheel;
    explicit DeviceSettings(const MotionConfig&);
    std::filesystem::path path(const char* key) const;
};
struct DeviceServerOptions {
    std::string name = "manipulator";
    std::array<std::string, 2> tcp_frames{{"left_base", "right_base"}};
    bool wheel_measurement = false;
    std::vector<std::string> topics;
    std::function<void(const Message&, uint64_t)> message;
    std::function<bool(zmq::socket_t&, uint64_t)> tick;
};
// Owns reliable lifecycle services, authorization, trajectory cursor, watchdog and arm feedback.
// Logical identities remain manipulator/aviator_core for interchangeable device processes.
int runDeviceServer(DataLink&, const MotionConfig&, const DeviceSettings&,
                    const DeviceServerOptions&, const volatile std::sig_atomic_t& interrupted);
} // namespace aviator

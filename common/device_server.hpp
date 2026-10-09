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
    std::array<double, 7> default_stiffness{}, following_stiffness{};
    WheelReference initial_wheel;
    explicit DeviceSettings(const MotionConfig&);
    std::filesystem::path path(const char* key) const;
};
class DeviceServo {
public:
    virtual ~DeviceServo() = default;
    virtual JointFrame step(const ServoGoal&) = 0;
    virtual bool stopped() const = 0;
};
struct DeviceServerOptions {
    std::string name = "manipulator";
    std::array<std::string, 2> tcp_frames{{"left_base", "right_base"}};
    bool wheel_measurement = false;
    std::function<std::unique_ptr<DeviceServo>(const JointFrame&)> servo;
    std::vector<std::string> topics;
    std::function<void(const Message&, uint64_t)> message;
    std::function<bool(zmq::socket_t&, uint64_t)> tick;
};
// Owns reliable lifecycle services, authorization, trajectory cursor, watchdog and arm feedback.
// Logical identities remain manipulator/aviator_core for interchangeable device processes.
int runDeviceServer(DataLink&, const MotionConfig&, const DeviceSettings&,
                    const DeviceServerOptions&, const volatile std::sig_atomic_t& interrupted);
} // namespace aviator

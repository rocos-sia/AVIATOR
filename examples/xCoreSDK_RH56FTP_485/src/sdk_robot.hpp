#pragma once
#include <rokae/robot.h>
#include <memory>
#include <mutex>
#include <string>

namespace demo {
// xCoreSDK 0.7.1 lazily registers process-wide spdlog loggers. Concurrent
// robot construction can register _rokae_log_console_ twice and abort.
// Keep this gate process-wide, including across separate Backend instances.
inline std::unique_ptr<rokae::ArRobot> makeSdkRobot(
    const std::string& robotIp = {}) {
    static std::mutex initializationMutex;
    std::lock_guard<std::mutex> lock(initializationMutex);
    auto robot = std::make_unique<rokae::ArRobot>();
    if (!robotIp.empty()) robot->connectToRobot(robotIp);
    return robot;
}
} // namespace demo

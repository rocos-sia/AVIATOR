#pragma once
#include "aviator/Kinematics.hpp"
#include "aviator/WheelEstimate.hpp"
#include <memory>
#include <nlohmann/json.hpp>

namespace aviator {
// Dedicated FK instance used only by the non-RT publication thread. This
// estimate is telemetry, never a Servo target or independent wheel sensor.
class JointWheelFeedback {
  public:
    JointWheelFeedback(std::unique_ptr<Kinematics> kinematics, const YAML::Node& grasp)
        : kinematics_(std::move(kinematics)), estimate_(grasp) {}
    nlohmann::json measure(const ArmFeedback&, bool locked, uint64_t now);
  private:
    std::unique_ptr<Kinematics> kinematics_;
    WheelEstimate estimate_;
};
} // namespace aviator

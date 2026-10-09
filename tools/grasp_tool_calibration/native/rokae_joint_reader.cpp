#include "rokae_joint_reader.hpp"

#include <rokae/robot.h>
#include <iostream>
#include <stdexcept>
#include <system_error>

namespace aviator::calibration {

class RokaeJointReader::Impl {
 public:
    rokae::xMateErProRobot robot;
    std::string endpoint;
    bool connected = false;

    ~Impl() {
        if (!connected) return;
        // The SDK documents that disconnect itself stops robot motion. Run this
        // static calibration only after ending teaching/motion and exiting the
        // control nodes. We never issue stopMove, switch modes or change power.
        std::error_code ec;
        robot.disconnectFromRobot(ec);
        if (ec)
            std::cerr << "Disconnect " << endpoint << ": " << ec.message() << '\n';
    }
};

RokaeJointReader::RokaeJointReader(const std::string& ip) : impl_(std::make_unique<Impl>()) {
    impl_->endpoint = ip;
    // No realtime reception, local-IP setup, mode changes, power changes,
    // toolset changes, or motion-controller access are needed for jointPos.
    impl_->robot.connectToRobot(ip);
    impl_->connected = true;
}

RokaeJointReader::~RokaeJointReader() = default;

std::array<double, 7> RokaeJointReader::position() {
    std::error_code ec;
    const auto state = impl_->robot.operationState(ec);
    if (ec)
        throw std::runtime_error("operationState " + impl_->endpoint + ": " + ec.message());
    if (state != rokae::OperationState::idle && state != rokae::OperationState::jog)
        throw std::runtime_error("Robot " + impl_->endpoint + " operationState=" +
            std::to_string(static_cast<int>(state)) +
            "; finish teaching/dragging and exit motion controllers before static calibration");
    ec.clear();
    const auto joints = impl_->robot.jointPos(ec); // SDK units: radians.
    if (ec)
        throw std::runtime_error("jointPos " + impl_->endpoint + ": " + ec.message() +
            " [" + ec.category().name() + ":" + std::to_string(ec.value()) + "]");
    return joints;
}

} // namespace aviator::calibration

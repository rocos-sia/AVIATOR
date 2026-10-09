#include "rokae_session.hpp"

#include <rokae/robot.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

namespace aviator::calibration {
namespace {
using Clock = std::chrono::steady_clock;

std::string operationName(rokae::OperationState state) {
    switch (state) {
    case rokae::OperationState::idle: return "idle";
    case rokae::OperationState::jog: return "jog";
    case rokae::OperationState::rtControlling: return "rtControlling";
    case rokae::OperationState::drag: return "drag";
    case rokae::OperationState::rlProgram: return "rlProgram";
    case rokae::OperationState::demo: return "demo";
    case rokae::OperationState::dynamicIdentify: return "dynamicIdentify";
    case rokae::OperationState::frictionIdentify: return "frictionIdentify";
    case rokae::OperationState::loadIdentify: return "loadIdentify";
    case rokae::OperationState::moving: return "moving";
    case rokae::OperationState::jogging: return "jogging";
    default: return "unknown";
    }
}

std::string powerName(rokae::PowerState state) {
    switch (state) {
    case rokae::PowerState::on: return "on";
    case rokae::PowerState::off: return "off";
    case rokae::PowerState::estop: return "estop";
    case rokae::PowerState::gstop: return "gstop";
    default: return "unknown";
    }
}
} // namespace

class RokaeSessionArm::Impl {
 public:
    rokae::xMateErProRobot robot;
    std::string ip;
    bool connected = false;
    bool drag_owned = false;
    const volatile std::sig_atomic_t* cancellation = nullptr;

    Impl(std::string address, const volatile std::sig_atomic_t* stop)
        : ip(std::move(address)), cancellation(stop) {}
    void checkCancellation() const {
        if (cancellation && *cancellation)
            throw std::runtime_error(ip + " session interrupted; cleaning up owned drag");
    }
    void check(const std::error_code& ec, const char* action) const {
        if (ec) throw std::runtime_error(ip + " " + action + ": " + ec.message() +
            " [" + ec.category().name() + ":" + std::to_string(ec.value()) + "]");
    }
    void requireConnected() const {
        if (!connected) throw std::runtime_error(ip + " is not connected");
    }
    rokae::OperationState operation() {
        requireConnected();
        std::error_code ec;
        const auto value = robot.operationState(ec);
        check(ec, "operationState");
        return value;
    }
    void requireIdle() {
        const auto state = operation();
        if (drag_owned || (state != rokae::OperationState::idle && state != rokae::OperationState::jog))
            throw std::runtime_error(ip + " operationState=" + operationName(state) +
                "; finish dragging/motion and exit other controllers first");
    }
    template<class Predicate> void waitFor(Predicate ready, const char* action, bool cancellable = false) {
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        do {
            // Normal transitions can be interrupted. Cleanup transitions must
            // still finish after SIGTERM, so stopDrag leaves cancellable false.
            if (cancellable) checkCancellation();
            if (ready()) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } while (Clock::now() < deadline);
        throw std::runtime_error(ip + " timed out waiting for " + action);
    }
};

RokaeSessionArm::RokaeSessionArm(std::string ip, const volatile std::sig_atomic_t* cancellation)
    : impl_(std::make_unique<Impl>(std::move(ip), cancellation)) {}

RokaeSessionArm::~RokaeSessionArm() {
    if (!impl_->connected) return;
    try { disconnect(); }
    catch (const std::exception& error) {
        std::cerr << "Session arm cleanup failed: " << error.what() << '\n';
    }
}

void RokaeSessionArm::connect() {
    impl_->checkCancellation();
    if (!impl_->connected) {
        impl_->robot.connectToRobot(impl_->ip);
        impl_->connected = true;
    }
    impl_->checkCancellation();
    // operationState is only available after connecting. Refuse taking over an
    // existing drag/controller; leave the read-only connection visible so the
    // caller sees the real state and can ask the operator to finish it.
    impl_->requireIdle();
}

SessionArmStatus RokaeSessionArm::status() {
    SessionArmStatus value;
    value.connected = impl_->connected;
    value.drag_owned = impl_->drag_owned;
    if (!value.connected) return value;
    const auto operation = impl_->operation();
    std::error_code ec;
    const auto power = impl_->robot.powerState(ec);
    impl_->check(ec, "powerState");
    value.operation_state_code = static_cast<int>(operation);
    value.operation_state = operationName(operation);
    value.dragging = operation == rokae::OperationState::drag;
    value.power_state_code = static_cast<int>(power);
    value.power_state = powerName(power);
    return value;
}

void RokaeSessionArm::startDrag() {
    impl_->checkCancellation();
    if (impl_->drag_owned && impl_->operation() == rokae::OperationState::drag) return;
    impl_->requireIdle();
    std::error_code ec;
    impl_->robot.setOperateMode(rokae::OperateMode::manual, ec);
    impl_->check(ec, "setOperateMode(manual)");
    impl_->waitFor([&] {
        const auto mode = impl_->robot.operateMode(ec);
        impl_->check(ec, "operateMode");
        return mode == rokae::OperateMode::manual;
    }, "manual mode", true);
    impl_->requireIdle();
    impl_->checkCancellation();
    impl_->robot.setPowerState(false, ec);
    impl_->check(ec, "setPowerState(false)");
    impl_->waitFor([&] {
        const auto power = impl_->robot.powerState(ec);
        impl_->check(ec, "powerState");
        if (power == rokae::PowerState::estop || power == rokae::PowerState::gstop ||
            power == rokae::PowerState::unknown)
            throw std::runtime_error(impl_->ip + " cannot start drag: powerState=" + powerName(power));
        return power == rokae::PowerState::off;
    }, "power off", true);
    impl_->requireIdle();
    impl_->checkCancellation();
    // Mark ownership before the call: an error/timeout could still leave drag
    // enabled, so the caller must be able to attempt rollback.
    impl_->drag_owned = true;
    impl_->robot.enableDrag(rokae::DragParameter::jointSpace, rokae::DragParameter::freely, ec, false);
    impl_->check(ec, "enableDrag(jointSpace, freely, button required)");
    impl_->waitFor([&] { return impl_->operation() == rokae::OperationState::drag; }, "drag enabled", true);
}

void RokaeSessionArm::stopDrag() {
    impl_->requireConnected();
    if (!impl_->drag_owned) {
        if (impl_->operation() == rokae::OperationState::drag)
            throw std::runtime_error(impl_->ip + " drag belongs to another controller; refusing to disable it");
        return;
    }
    std::error_code ec;
    impl_->robot.disableDrag(ec);
    impl_->check(ec, "disableDrag");
    impl_->waitFor([&] {
        const auto state = impl_->operation();
        return state == rokae::OperationState::idle || state == rokae::OperationState::jog;
    }, "drag disabled and arm stationary");
    impl_->drag_owned = false;
}

std::array<double, 7> RokaeSessionArm::position() {
    impl_->requireIdle();
    std::error_code ec;
    const auto joints = impl_->robot.jointPos(ec);
    impl_->check(ec, "jointPos");
    impl_->requireIdle();
    return joints;
}

void RokaeSessionArm::disconnect() {
    if (!impl_->connected) return;
    std::string errors;
    if (impl_->drag_owned) {
        try { stopDrag(); }
        catch (const std::exception& error) { errors = error.what(); }
    }
    std::error_code ec;
    impl_->robot.disconnectFromRobot(ec);
    if (ec) {
        if (!errors.empty()) errors += "; ";
        errors += impl_->ip + " disconnectFromRobot: " + ec.message();
    } else {
        impl_->connected = false;
        impl_->drag_owned = false;
    }
    if (!errors.empty()) throw std::runtime_error(errors);
}

} // namespace aviator::calibration

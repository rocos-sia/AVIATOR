#include "rokae_session.hpp"

#include <rokae/robot.h>
#include <chrono>
#include <mutex>
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
    rokae::ArRobot robot;
    std::recursive_mutex rpc_mutex;
    std::string ip;
    bool connected = false;
    bool drag_owned = false;
    bool hand_started = false;
    bool power_attempted = false;
    bool rt_mode_attempted = false;
    bool load_set = false;
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
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
    impl_->checkCancellation();
    if (!impl_->connected) {
        impl_->robot.connectToRobot(impl_->ip);
        impl_->connected = true;
    }
    impl_->checkCancellation();
    // operationState is only available after connecting. Refuse taking over an
    // existing drag/controller; leave the connection visible so the
    // caller sees the real state and can ask the operator to finish it.
    impl_->requireIdle();
    if (!impl_->load_set) {
        std::error_code ec;
        // Same physical payload as nodes/manipulator/rokae_sdk.cpp. Keep the
        // existing frames; calibration computes the flange FK independently.
        const rokae::Load load(0.71, {0.02134, -0.02368, 0.11188}, {0., 0., 0.});
        auto toolset = impl_->robot.toolset(ec);
        impl_->check(ec, "toolset");
        toolset.load = load;
        impl_->robot.setToolset(toolset, ec);
        impl_->check(ec, "setToolset(load)");
        impl_->rt_mode_attempted = true;
        impl_->robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        impl_->check(ec, "setMotionControlMode(RtCommand for payload)");
        try {
            auto rt = impl_->robot.getRtMotionController().lock();
            if (!rt) throw std::runtime_error("getRtMotionController returned null");
            rt->setLoad(load, ec);
            impl_->check(ec, "setLoad");
        } catch (...) {
            impl_->robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (!ec) impl_->rt_mode_attempted = false;
            throw;
        }
        impl_->robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
        impl_->check(ec, "setMotionControlMode(Idle after payload)");
        impl_->rt_mode_attempted = false;
        impl_->load_set = true;
        std::cerr << "Calibration " << impl_->ip
                  << " setLoad: mass=0.71 kg cog=[0.02134,-0.02368,0.11188] m inertia=[0,0,0] kg*m^2\n";
    }
}

SessionArmStatus RokaeSessionArm::status() {
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
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
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
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

void RokaeSessionArm::stopDrag(bool resume_hand) {
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
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
    if (resume_hand && impl_->hand_started) startHand();
}

std::array<double, 7> RokaeSessionArm::position() {
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
    impl_->requireIdle();
    std::error_code ec;
    const auto joints = impl_->robot.jointPos(ec);
    impl_->check(ec, "jointPos");
    impl_->requireIdle();
    return joints;
}

void RokaeSessionArm::disconnect() {
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
    if (!impl_->connected) return;
    std::string errors;
    if (impl_->drag_owned) {
        try { stopDrag(false); }
        catch (const std::exception& error) { errors = error.what(); }
    }
    std::error_code ec;
    if (impl_->rt_mode_attempted) {
        impl_->robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
        if (ec) errors += "; reset motion control: " + ec.message();
        else impl_->rt_mode_attempted = false;
    }
    if (impl_->power_attempted) {
        impl_->robot.setPowerState(false, ec);
        if (ec) errors += "; power off: " + ec.message();
        else impl_->power_attempted = false;
    }
    impl_->robot.disconnectFromRobot(ec);
    if (ec) {
        if (!errors.empty()) errors += "; ";
        errors += impl_->ip + " disconnectFromRobot: " + ec.message();
    } else {
        impl_->connected = false;
        impl_->drag_owned = false;
        impl_->hand_started = false;
        impl_->load_set = false;
    }
    if (!errors.empty()) throw std::runtime_error(errors);
}

} // namespace aviator::calibration

namespace aviator::calibration {
void RokaeSessionArm::startHand() {
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
    impl_->checkCancellation();
    impl_->requireIdle();
    std::error_code ec;
    impl_->power_attempted = true;
    impl_->robot.setPowerState(true, ec);
    impl_->check(ec, "setPowerState(true for hand)");
    impl_->hand_started = true;
    std::cerr << "Calibration " << impl_->ip << " hand power on; using current RS485 configuration and timeout\n";
}
std::vector<uint8_t> RokaeSessionArm::handTransfer(const std::vector<uint8_t>& tx, size_t count) {
    if (tx.empty() || tx.size() > 64 || count == 0 || count > 64)
        throw std::runtime_error("RS485 requires 1..64 bytes");
    std::lock_guard<std::recursive_mutex> rpc(impl_->rpc_mutex);
    impl_->requireConnected();
    std::vector<uint8_t> rx(count);
    std::error_code ec;
    const auto start = Clock::now();
    impl_->robot.XPRS485SendData(int(tx.size()), int(count), tx, rx, ec);
    impl_->check(ec, "XPRS485SendData");
    if (Clock::now() - start > std::chrono::seconds(3))
        throw std::runtime_error(impl_->ip + " RS485 transaction exceeded 3 seconds");
    return rx;
}
} // namespace aviator::calibration

#include "StateMachineRuntime.hpp"
#include "ManagedGateway.hpp"
#include "ManagedReadiness.hpp"
#include "RemoteLink.hpp"
#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/RobotStateMachine.hpp"
#include <deque>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sstream>
#include <unistd.h>

namespace aviator {
int runStateMachine(const MotionConfig& config, const volatile std::sig_atomic_t& interrupted,
                    const ManagedGatewayOptions& gateway_options) {
    std::unique_ptr<ManagedGateway> gateway;
    if (gateway_options.enabled) gateway = std::make_unique<ManagedGateway>(config, gateway_options);
    auto link = std::make_unique<RemoteLink>(config);
    auto* connection = link.get();
    std::cout << "Managed Core session=" << connection->session() << std::endl;
    Aviator* instance = nullptr;
    bool revoked = true;
    std::string last_state;
    ManagedOptions options;
    options.snapshot = [&] {
        fsm::Snapshot s;
        // This local test entry authorizes following/release. There is no external safety
        // supervisor or hardware emergency input; software EmergencyStop stays latched in SML.
        s.emergency_known = true;
        s.following_authorized = s.release_authorized = true;
        s.source_authorized = !gateway || gateway->bound();
        s.input_ready = !gateway || gateway->fresh();
        bool fresh = false, status_fresh = false;
        const auto device = connection->snapshot(fresh, &status_fresh);
        const auto phase = instance ? instance->GetState() : "UNINITIALIZED";
        s.ready = managedResourcesReady(connection->backend(), phase, device, fresh, status_fresh);
        s.settled = s.ready && !device.stopping && (!revoked || device.id == 0);
        s.fault_cleared = fresh && !device.fault;
        if (device.fault) s.fault = device.error.empty() ? "Manipulator blocking fault" : device.error;
        if (instance) {
            // Software phase/lock only; this does not measure physical wheel clearance.
            s.clear_of_wheel = s.ready && !device.locked &&
                (phase == "UNINITIALIZED" || phase == "INITIALIZED" || phase == "ENABLED" || phase == "DISABLED");
        }
        return s;
    };
    options.heartbeat = [&] { connection->heartbeat(); };
    options.allow_motion = [&](bool allowed) { revoked = !allowed; connection->allowMotion(allowed); };
    options.request_brake = [&] {
        std::cerr << "Software emergency latched in this process; physical brake adapter unavailable" << std::endl;
    };
    options.report = [&](const SystemStatus& status) {
        const auto& s = status.conditions;
        const auto source = gateway && gateway->fresh() && status.accepts_control ? "JOYSTICK" : "NONE";
        connection->report(instance->GetState(), source);
        connection->reportSystem({{"state", status.state}, {"state_code", status.state_code},
            {"ready", s.ready}, {"settled", s.settled}, {"control_source", source},
            {"input_ready", s.input_ready}, {"source_authorized", s.source_authorized},
            {"current_error_code", status.current_error.empty() ? 0 : 28673},
            {"last_error_code", status.last_error.empty() ? 0 : 28673},
            {"current_error", status.current_error}, {"last_error", status.last_error},
            {"task_phase", instance->GetState()}, {"generation", status.generation},
            {"brake_requested", status.brake_requested}, {"brake_confirmed", false},
            {"brake_status", status.brake_requested ? "UNAVAILABLE" : "NOT_REQUESTED"}});
        if (last_state != status.state) {
            last_state = status.state;
            std::cout << "system.state=" << last_state << " error=" << status.current_error << std::endl;
        }
    };
    Aviator robot(std::move(link), nullptr, nullptr, config.robot.string(), std::move(options));
    instance = &robot;
    robot.Init();
    std::cout << "Managed Aviator: ENTER_STANDBY | GRASP_WHEEL | START_CONTROL | EXIT_CONTROL | LEAVE_WHEEL | RESET_ERROR\n"
                 "status | emergency | quit\n";
    if (!gateway) std::cout << "servo <angle_rad> <displacement_m> [v] (refresh <100 ms)\n";
    std::string pending;
    uint64_t pending_mono = 0;
    std::deque<std::pair<std::string, uint64_t>> commands;
    bool quit = false;
    bool console_open = true;
    while (!quit && !interrupted) {
        if (gateway) gateway->receiveInput();
        robot.Update();
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        if (console_open && poll(&descriptor, 1, 0) > 0) {
            char buffer[512]; const auto size = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (size <= 0) {
                if (!gateway) break;
                console_open = false; // Headless gateway control survives stdin EOF.
            }
            const auto arrival = monotonic_us();
            for (int i = 0; i < size; ++i) {
                if (pending.empty()) pending_mono = arrival;
                if (buffer[i] == '\n') {
                    if (pending == "emergency") robot.EmergencyStop("Local emergency input");
                    else commands.emplace_back(std::move(pending), pending_mono);
                    pending.clear();
                }
                else pending.push_back(buffer[i]);
            }
            if (pending.size() > 4096 || commands.size() > 64) {
                pending.clear(); commands.clear(); std::cerr << "BUSY: console queue full\n";
            }
        }
        if (!commands.empty()) {
            auto [text, sample] = std::move(commands.front()); commands.pop_front();
            std::istringstream line(text);
            std::string command, extra; line >> command;
            if (command == "quit") quit = true;
            else if (command == "emergency") robot.EmergencyStop("Local emergency input");
            else if (command == "status")
                std::cout << "state=" << robot.GetSystemState() << " executor=" << robot.GetState() << std::endl;
            else if (command == "servo") {
                if (gateway) {
                    std::cout << "REJECTED: gateway mode accepts flight.command targets only\n";
                } else {
                    double a, d, v = .5;
                    if (!(line >> a >> d)) std::cout << "Invalid target\n";
                    else {
                        line >> std::ws;
                        if ((!line.eof() && !(line >> v)) || (line >> extra)) std::cout << "Invalid target arguments\n";
                        else if (!robot.ServoWheel(a, d, v, sample)) std::cout << "INVALID_STATE: inactive/stale/invalid target\n";
                    }
                }
            } else if (!command.empty()) {
                using Operation = fsm::Reply (Aviator::*)();
                const std::pair<const char*, Operation> operations[] = {
                    {"ENTER_STANDBY", &Aviator::EnterStandby}, {"GRASP_WHEEL", &Aviator::GraspWheel},
                    {"START_CONTROL", &Aviator::StartControl}, {"EXIT_CONTROL", &Aviator::ExitControl},
                    {"LEAVE_WHEEL", &Aviator::LeaveWheel}, {"RESET_ERROR", &Aviator::ResetError}};
                bool known = false;
                for (const auto& [name, operation] : operations) if (command == name) {
                    known = true;
                    if (line >> extra) std::cout << "INVALID_STATE: unexpected argument\n";
                    else {
                        const auto reply = (robot.*operation)();
                        std::cout << fsm::replyName(reply) << " " << robot.GetSystemState() << std::endl;
                    }
                }
                if (!known) std::cout << "INVALID_STATE: use the six FSM operations\n";
            }
        }
        if (gateway && !quit && !interrupted) {
            gateway->receiveRequests(robot, connection->session());
            gateway->drive(robot);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Managed destructor cancels, revokes output and joins the sole worker before destroying IO.
    return 0;
}
} // namespace aviator

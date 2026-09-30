#include "StateMachineRuntime.hpp"
#include "RemoteLink.hpp"
#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/RobotStateMachine.hpp"
#include <fcntl.h>
#include <fstream>
#include <future>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sstream>
#include <unistd.h>

namespace aviator {
namespace {
// Evidence comes from an independent same-host safety supervisor, never ordinary requests.
fsm::Snapshot evidence(const std::string& file, bool simulation, uint64_t now) {
    fsm::Snapshot s;
    if (simulation) {
        s.emergency_known = s.clear_of_wheel = s.following_authorized = true;
        s.release_authorized = s.fault_cleared = s.source_authorized = s.input_ready = true;
        return s;
    }
    try {
        if (file.empty() || std::filesystem::file_size(file) > 65536) return s;
        std::ifstream stream(file);
        const Json j = parseService(std::string(std::istreambuf_iterator<char>(stream), {}));
        if (!j.at("sample_mono_us").is_number_unsigned()) return s;
        const auto sample = j.at("sample_mono_us").get<uint64_t>();
        if (j.at("clock_id") != local_clock_id() || sample > now || now - sample >= 100000) return s;
        s.emergency_known = true;
        s.emergency_latched = j.at("emergency_latched").get<bool>();
        s.clear_of_wheel = j.at("clear_of_wheel").get<bool>();
        s.following_authorized = j.at("following_authorized").get<bool>();
        s.release_authorized = j.at("release_authorized").get<bool>();
        s.fault_cleared = j.at("fault_cleared").get<bool>();
        s.source_authorized = j.at("source_authorized").get<bool>();
        s.input_ready = j.at("input_ready").get<bool>();
    } catch (...) { s.emergency_known = false; }
    // A Core-originated emergency also survives process restart. Only maintenance removes this.
    std::error_code error;
    if (!file.empty() && (std::filesystem::exists(file + ".emergency", error) || error)) s.emergency_latched = true;
    return s;
}
void latchFile(const std::string& path) {
    const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) {
        if (errno == EEXIST) return;
        throw std::runtime_error("Cannot persist emergency request: " + path);
    }
    const char text[] = "latched\n";
    const bool ok = write(fd, text, sizeof(text) - 1) == sizeof(text) - 1 && fsync(fd) == 0;
    close(fd);
    const auto parent = std::filesystem::absolute(path).parent_path();
    const int dir = open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    const bool synced = dir >= 0 && fsync(dir) == 0;
    if (dir >= 0) close(dir);
    if (!ok || !synced) throw std::runtime_error("Emergency request persistence failed: " + path);
}
}
int runStateMachine(const MotionConfig& config, const std::string& safety_file, bool simulation,
                    const volatile std::sig_atomic_t& interrupted) {
    const auto startup = evidence(safety_file, simulation, monotonic_us());
    auto link = std::make_unique<RemoteLink>(config, startup.emergency_known && !startup.emergency_latched);
    auto* connection = link.get();
    if (simulation && connection->backend() != "mujoco")
        throw std::runtime_error("--fsm-simulation is restricted to the MuJoCo backend");
    Aviator robot(std::move(link), nullptr, nullptr, config.robot.string());
    fsm::RobotStateMachine machine;
    std::future<void> worker;
    std::optional<fsm::Task> active, completion;
    bool initialized = false, simulation_clear = true, brake = false, quitting = false, revoked = false;
    std::string pending, last_state;
    uint64_t input_at = 0, pending_mono = 0;
    const auto snapshot = [&] {
        auto s = evidence(safety_file, simulation, monotonic_us());
        bool fresh = false;
        const auto device = connection->snapshot(fresh);
        s.ready = fresh && !device.fault;
        s.executor_idle = !worker.valid();
        const auto phase = robot.GetState();
        s.settled = fresh && s.executor_idle && !device.stopping && (!revoked || device.id == 0) &&
                    phase != "SERVO" && phase != "MOVING" && phase != "APPROACHING";
        s.fault_cleared = s.fault_cleared && fresh && !device.fault;
        if (simulation) s.clear_of_wheel = simulation_clear;
        if (device.fault) s.fault = device.error.empty() ? "Manipulator blocking fault" : device.error;
        if (machine.acceptsControl()) {
            const auto now = monotonic_us();
            const auto last = input_at ? input_at : machine.controlSince();
            s.input_ready = s.input_ready && now >= last && now - last < 100000;
        }
        return s;
    };
    const auto report = [&] {
        const auto s = snapshot();
        connection->report(robot.GetState(), machine.acceptsControl() ? "LOCAL" : "NONE");
        connection->reportSystem({{"state", machine.state()}, {"state_code", machine.stateCode()},
            {"ready", s.ready}, {"settled", s.settled}, {"control_source", "NONE"},
            {"current_error_code", machine.currentError().empty() ? 0 : 28673},
            {"last_error_code", machine.lastError().empty() ? 0 : 28673},
            {"current_error", machine.currentError()}, {"last_error", machine.lastError()},
            {"task_phase", robot.GetState()}, {"generation", machine.generation()},
            {"brake_requested", brake}, {"brake_confirmed", false},
            {"brake_status", brake ? "REQUESTED_UNCONFIRMED" : "NOT_REQUESTED"}});
        if (last_state != machine.state()) {
            last_state = machine.state();
            std::cout << "system.state=" << last_state << " error=" << machine.currentError() << std::endl;
        }
    };
    const auto effects = [&] {
        if (machine.takeStop()) {
            robot.Stop(); input_at = 0;
            // EXIT_CONTROL uses the existing continuous Servo deceleration. Protective
            // transitions instead revoke all ordinary windows, including old worker output.
            if (std::string(machine.state()) != "FOLLOWING") {
                connection->allowMotion(false); revoked = true;
            }
        }
        if (machine.takeBrake()) {
            brake = true;
            if (!simulation) {
                try {
                    if (safety_file.empty()) throw std::runtime_error("No independent safety/brake channel configured");
                    latchFile(safety_file + ".emergency");
                    latchFile(safety_file + ".brake-request");
                } catch (const std::exception& e) { machine.fault(e.what()); }
            }
            std::cerr << "Emergency brake requested; physical completion is NOT confirmed" << std::endl;
        }
        report(); // Export the transition before submitting any blocking task.
        if (auto task = machine.takeTask()) {
            // Recheck emergency evidence immediately before the ordinary side effect.
            machine.supervise(snapshot(), monotonic_us());
            if (task->generation != machine.generation()) return;
            if (worker.valid()) { machine.failed(task->generation, "Executor slot occupied"); return; }
            active = task;
            revoked = false;
            connection->allowMotion(true);
            if (task->job == fsm::Job::grasp) simulation_clear = false;
            try {
                worker = std::async(std::launch::async, [&, job = task->job] {
                    if (job == fsm::Job::initialize) {
                        if (!initialized) robot.Init(); // No enable, home or implicit release.
                    } else if (job == fsm::Job::grasp) {
                        if (robot.GetState() == "INITIALIZED" || robot.GetState() == "DISABLED") robot.Enable();
                        robot.ApproachHandles();
                        robot.LockHandles();
                    } else if (job == fsm::Job::release) robot.ReleaseHandles();
                });
            } catch (const std::exception& e) { machine.failed(task->generation, e.what()); }
        }
    };
    machine.boot(snapshot(), monotonic_us());
    std::cout << "FSM: ENTER_STANDBY | GRASP_WHEEL | START_CONTROL | EXIT_CONTROL | LEAVE_WHEEL | RESET_ERROR\n"
                 "servo <angle_rad> <displacement_m> [v] (local test, refresh <100 ms) | status | emergency | quit\n";
    try {
        while (!quitting && !interrupted) {
            connection->heartbeat();
            machine.supervise(snapshot(), monotonic_us());
            if (worker.valid() && worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                try { worker.get(); completion = active; }
                catch (const std::exception& e) { machine.failed(active->generation, e.what()); }
                active.reset();
            }
            if (completion) {
                if (completion->job == fsm::Job::initialize) initialized = true;
                if (completion->generation == machine.generation()) {
                    if (completion->job == fsm::Job::release) simulation_clear = true;
                    auto s = snapshot();
                    // Completion may precede executor stopping acknowledgement; keep the
                    // same task/deadline until the actual executor is stable.
                    if (s.settled || completion->job == fsm::Job::grasp) {
                        machine.done(completion->generation, s); completion.reset();
                    }
                } else completion.reset();
            }
            effects();
            pollfd descriptor{STDIN_FILENO, POLLIN, 0};
            if (poll(&descriptor, 1, 0) > 0) {
                char buffer[512]; auto size = read(STDIN_FILENO, buffer, sizeof(buffer));
                if (size <= 0) quitting = true;
                else if (pending.size() + size <= 4096) {
                    if (pending.empty()) pending_mono = monotonic_us();
                    pending.append(buffer, size);
                }
                else { pending.clear(); std::cerr << "BUSY: console queue full\n"; }
            }
            const auto end = pending.find('\n');
            if (end != std::string::npos) {
                std::istringstream line(pending.substr(0, end)); pending.erase(0, end + 1);
                std::string command; line >> command;
                if (command == "quit") quitting = true;
                else if (command == "emergency") machine.emergency("Local emergency input");
                else if (command == "status") std::cout << "state=" << machine.state() << " executor=" << robot.GetState() << std::endl;
                else if (command == "servo") {
                    machine.supervise(snapshot(), monotonic_us());
                    double angle, displacement, v = .5;
                    if (!machine.acceptsControl() || pending_mono < machine.controlSince() ||
                        monotonic_us() - pending_mono >= 100000) std::cout << "INVALID_STATE: inactive or stale target\n";
                    else if (!(line >> angle >> displacement)) std::cout << "Invalid target\n";
                    else {
                        line >> std::ws;
                        if (!line.eof() && !(line >> v)) { std::cout << "Invalid speed\n"; continue; }
                        try { robot.ServoWheel(angle, displacement, v); input_at = monotonic_us(); }
                        catch (const std::exception& e) { machine.fault(e.what()); }
                    }
                } else if (!command.empty()) {
                    const std::pair<const char*, fsm::Operation> operations[] = {
                        {"ENTER_STANDBY", fsm::Operation::enter_standby}, {"GRASP_WHEEL", fsm::Operation::grasp_wheel},
                        {"START_CONTROL", fsm::Operation::start_control}, {"EXIT_CONTROL", fsm::Operation::exit_control},
                        {"LEAVE_WHEEL", fsm::Operation::leave_wheel}, {"RESET_ERROR", fsm::Operation::reset_error}};
                    bool known = false;
                    for (const auto& [name, op] : operations) if (command == name) {
                        known = true;
                        const auto reply = machine.request(op, snapshot(), monotonic_us());
                        if (op == fsm::Operation::reset_error && reply == fsm::Reply::completed) {
                            try { robot.AcknowledgeFault(); }
                            catch (const std::exception& e) { machine.fault(e.what()); }
                        }
                        effects();
                        std::cout << (reply == fsm::Reply::accepted && std::string(machine.state()) == "ERROR"
                                      ? "REJECTED" : fsm::replyName(reply)) << " " << machine.state() << std::endl;
                    }
                    if (!known) std::cout << "INVALID_STATE: unknown operation\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    } catch (...) {
        robot.Stop(); connection->allowMotion(false);
        if (worker.valid()) worker.wait();
        throw;
    }
    robot.Stop(); connection->allowMotion(false);
    while (worker.valid() && worker.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) connection->heartbeat();
    if (worker.valid()) { try { worker.get(); } catch (...) {} }
    // No software unlock/retreat on protection/shutdown. Physical hand commands were
    // revoked above: aviator_hand independently applies its configured timeout safe_pose.
    return 0;
}
} // namespace aviator

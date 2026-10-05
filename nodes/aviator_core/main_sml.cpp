#include "Logger.hpp"
#include "aviator/RobotStateMachine.hpp"
#include <charconv>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
namespace fsm = aviator::fsm;
using Field = std::pair<const char*, bool fsm::Snapshot::*>;
constexpr Field fields[] = {
    {"ready", &fsm::Snapshot::ready}, {"settled", &fsm::Snapshot::settled},
    {"clear_of_wheel", &fsm::Snapshot::clear_of_wheel},
    {"following_authorized", &fsm::Snapshot::following_authorized},
    {"source_authorized", &fsm::Snapshot::source_authorized},
    {"input_ready", &fsm::Snapshot::input_ready},
    {"fault_cleared", &fsm::Snapshot::fault_cleared},
    {"executor_idle", &fsm::Snapshot::executor_idle},
    {"release_authorized", &fsm::Snapshot::release_authorized},
    {"emergency_known", &fsm::Snapshot::emergency_known},
    {"emergency_latched", &fsm::Snapshot::emergency_latched}};
constexpr std::pair<const char*, fsm::Operation> operations[] = {
    {"ENTER_STANDBY", fsm::Operation::enter_standby},
    {"GRASP_WHEEL", fsm::Operation::grasp_wheel},
    {"START_CONTROL", fsm::Operation::start_control},
    {"EXIT_CONTROL", fsm::Operation::exit_control},
    {"LEAVE_WHEEL", fsm::Operation::leave_wheel},
    {"RESET_ERROR", fsm::Operation::reset_error}};

const char* jobName(fsm::Job job) {
    switch (job) {
    case fsm::Job::initialize: return "initialize";
    case fsm::Job::home: return "home";
    case fsm::Job::grasp: return "grasp";
    case fsm::Job::release: return "release";
    default: return "none";
    }
}
void help() {
    aviator::Logger::info("aviator_core_sml [--demo | --help]\n"
        "OFFLINE ONLY: no config, safety file, ZMQ, robot or hand connection.\n"
        "Actual SML transitions; all guards/actions/time below are simulated.\n"
        "ENTER_STANDBY | GRASP_WHEEL | START_CONTROL | EXIT_CONTROL | LEAVE_WHEEL | RESET_ERROR\n"
        "done: complete current mock task | fail: fail current task | late_done: replay last task "
        "completion\n"
        "fault | safety_lost | emergency: inject a simulated event\n"
        "set <guard> <0|1>: change a mock guard and immediately supervise\n"
        "advance <milliseconds>: advance virtual time (0..3600000); no real sleep\n"
        "status | help | quit\n"
        "Guards:");
    std::ostringstream names;
    for (const auto& [name, unused] : fields) names << ' ' << name;
    aviator::Logger::info("Guards:{}", names.str());
    aviator::Logger::info("\n"
        "Boot starts INITIALIZING; enter done to finish mock initialization into READY; "
        "ENTER_STANDBY then done simulates home.");
}

// Only the real transition table is shared with Core. No Aviator/RemoteLink object
// exists here. Actions are mock task registration, with explicit completion events.
class OfflineRuntime {
public:
    OfflineRuntime() {
        for (const auto& [unused, member] : fields) snapshot_.*member = true;
        snapshot_.emergency_latched = false;
        const std::string before = machine_.state();
        machine_.boot(snapshot_, now_);
        effects("Boot", before);
    }
    bool command(const std::string& text) {
        std::istringstream line(text);
        std::string command, extra;
        if (!(line >> command)) return true;
        if (command == "set") {
            std::string name, value;
            if (!(line >> name >> value) || (line >> extra) || (value != "0" && value != "1"))
                throw std::runtime_error("Expected set <guard> <0|1>");
            for (const auto& [field, member] : fields) if (name == field) {
                snapshot_.*member = value == "1";
                const std::string before = machine_.state();
                machine_.supervise(snapshot_, ++now_);
                effects(text, before);
                return true;
            }
            throw std::runtime_error("Unknown guard: " + name);
        }
        if (command == "advance") {
            std::string value;
            if (!(line >> value) || (line >> extra)) throw std::runtime_error("Expected advance <milliseconds>");
            unsigned long long milliseconds = 0;
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), milliseconds);
            if (error != std::errc{} || end != value.data() + value.size() || milliseconds > 3600000)
                throw std::runtime_error("advance must be 0..3600000 milliseconds");
            const std::string before = machine_.state();
            now_ += milliseconds * 1000;
            machine_.supervise(snapshot_, now_);
            effects(text, before);
            return true;
        }
        if (line >> extra) throw std::runtime_error("Unexpected command argument");
        if (command == "quit") return false;
        if (command == "help") { help(); return true; }
        if (command == "status") { status(); return true; }
        const std::string before = machine_.state();
        ++now_;
        for (const auto& [name, op] : operations) if (command == name) {
            aviator::Logger::info("[guards] "); guards();
            const auto reply = machine_.request(op, snapshot_, now_);
            aviator::Logger::info("[reply] {} {}", name, fsm::replyName(reply));
            effects(command, before);
            return true;
        }
        if (command == "done" || command == "fail") {
            if (!active_) {
                aviator::Logger::info("[mock] no active task");
            } else {
                const auto task = *active_;
                last_task_ = task;
                active_.reset();
                snapshot_.executor_idle = snapshot_.settled = true;
                if (command == "fail") machine_.failed(task.generation, "Injected mock task failure");
                else {
                    // This executor simulates completed retreat; other evidence remains editable.
                    if (task.job == fsm::Job::release) snapshot_.clear_of_wheel = true;
                    machine_.done(task.generation, snapshot_);
                }
            }
        } else if (command == "late_done") {
            if (last_task_) machine_.done(last_task_->generation, snapshot_);
            else aviator::Logger::info("[mock] no previous task");
        } else if (command == "fault") machine_.fault("Injected mock fault");
        else if (command == "safety_lost") machine_.safetyLost("Injected mock safety loss");
        else if (command == "emergency") machine_.emergency("Injected mock emergency");
        else throw std::runtime_error("Unknown command: " + command);
        effects(command, before);
        return true;
    }
    const char* state() const { return machine_.state(); }
private:
    void guards() const {
        std::ostringstream values;
        for (const auto& [name, member] : fields) values << name << '=' << snapshot_.*member << ' ';
        aviator::Logger::info("guards: {}", values.str());
    }
    void status() const {
        aviator::Logger::info("state={} accepts_control={} time_us={} job={} generation={} error={} last_error={}",
            machine_.state(), static_cast<int>(machine_.acceptsControl()), now_, jobName(machine_.job()), machine_.generation(),
            machine_.currentError(), machine_.lastError());
        guards();
    }
    void effects(const std::string& event, const std::string& before) {
        aviator::Logger::info("[event] {}: {} -> {}", event, before, machine_.state());
        if (machine_.takeStop()) {
            if (active_) last_task_ = active_;
            active_.reset();
            snapshot_.executor_idle = snapshot_.settled = true;
            aviator::Logger::info("[mock] stop/cancel acknowledged; no device action");
        }
        if (machine_.takeBrake()) aviator::Logger::info("[mock] brake requested; no hardware, NOT confirmed");
        if (auto task = machine_.takeTask()) {
            active_ = task;
            snapshot_.executor_idle = snapshot_.settled = false;
            if (task->job == fsm::Job::grasp) snapshot_.clear_of_wheel = false;
            aviator::Logger::info("[mock] action={} generation={} deadline_us={}; waiting for done/fail",
                jobName(task->job), task->generation, task->deadline);
        }
        status();
    }
    fsm::RobotStateMachine machine_;
    fsm::Snapshot snapshot_;
    std::optional<fsm::Task> active_, last_task_;
    std::uint64_t now_ = 1; // Virtual clock only; interactive typing never ages evidence.
};
} // namespace

int main(int argc, char** argv) try {
    bool demo = false;
    if (argc == 2 && std::string(argv[1]) == "--help") { help(); return 0; }
    if (argc == 2 && std::string(argv[1]) == "--demo") demo = true;
    else if (argc != 1) throw std::runtime_error("Only --demo/--help are supported; this executable is offline only");
    aviator::Logger::info("OFFLINE SML TEST: simulated guards, executor and clock; no device connections.");
    OfflineRuntime runtime;
    if (demo) {
        const std::pair<const char*, const char*> sequence[] = {
            {"done", "READY"}, {"ENTER_STANDBY", "HOMING"}, {"done", "STANDBY"}, {"GRASP_WHEEL", "GRASPING"}, {"done", "FOLLOWING"},
            {"START_CONTROL", "CONTROL"}, {"EXIT_CONTROL", "FOLLOWING"},
            {"LEAVE_WHEEL", "RELEASING"}, {"done", "STANDBY"}};
        for (const auto& [command, expected] : sequence) {
            runtime.command(command);
            if (std::string(runtime.state()) != expected) throw std::runtime_error("Unexpected demo state");
        }
        aviator::Logger::info("PASS offline FSM cycle");
    } else {
        help();
        for (std::string line; std::getline(std::cin, line) && runtime.command(line);) {}
    }
    return 0;
} catch (const std::exception& e) {
    aviator::Logger::error("Error: {}", e.what());
    return 1;
}

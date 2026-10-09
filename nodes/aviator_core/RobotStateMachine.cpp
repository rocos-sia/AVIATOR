#include "Logger.hpp"
#include <cstring>
#include "aviator/RobotStateMachine.hpp"
#include <limits>
#include <stdexcept>
#include <utility>

namespace aviator::fsm {
namespace {
const char* operationName(Operation op) {
    switch (op) {
    case Operation::enter_standby: return "ENTER_STANDBY";
    case Operation::grasp_wheel: return "GRASP_WHEEL";
    case Operation::start_control: return "START_CONTROL";
    case Operation::exit_control: return "EXIT_CONTROL";
    case Operation::leave_wheel: return "LEAVE_WHEEL";
    case Operation::reset_error: return "RESET_ERROR";
    }
    return "UNKNOWN";
}
// Event-scoped observation covers early returns without changing the transition table.
struct TransitionLog {
    const RobotStateMachine& machine;
    const char* event;
    const char* before = machine.state();
    ~TransitionLog() noexcept {
        try {
            if (std::strcmp(before, machine.state()) != 0)
                Logger::info("Core FSM event={} state={} -> {} generation={} error={}",
                    event, before, machine.state(), machine.generation(), machine.currentError());
        } catch (...) {}
    }
};
}

RobotStateMachine::RobotStateMachine(Timeouts t) : timeouts_(t) {
    for (auto value : {t.initialize, t.grasp, t.release, t.home})
        if (!value || value > 3600000000ULL) throw std::invalid_argument("Task timeout must be 1us..1h");
}
void RobotStateMachine::snapshot(const Snapshot& s) {
    context_.ready = s.ready;
    context_.settled = s.settled;
    context_.clear_of_wheel = s.clear_of_wheel;
    context_.following_authorized = s.following_authorized;
    context_.source_authorized = s.source_authorized;
    context_.input_ready = s.input_ready;
    context_.fault_cleared = s.fault_cleared;
    context_.executor_idle = s.executor_idle;
    context_.release_authorized = s.release_authorized;
}
void RobotStateMachine::deadline(std::uint64_t now) {
    const auto duration = (context_.job == Job::initialize || context_.job == Job::following_impedance) ? timeouts_.initialize :
                          context_.job == Job::grasp ? timeouts_.grasp :
                          context_.job == Job::home ? timeouts_.home : timeouts_.release;
    if (now > std::numeric_limits<std::uint64_t>::max() - duration) {
        fault("Monotonic deadline overflow"); return;
    }
    deadline_ = now + duration;
}
void RobotStateMachine::boot(const Snapshot& s, std::uint64_t now) {
    const TransitionLog transition{*this, "Boot"};
    supervise(s, now);
    if (machine_.process_event(Boot{})) deadline(now);
}
void RobotStateMachine::supervise(const Snapshot& s, std::uint64_t now) {
    last_now_ = now;
    snapshot(s);
    if (!s.emergency_known || s.emergency_latched) { emergency("Emergency latched or evidence unavailable"); return; }
    if (machine_.is(sml::state<EMERGENCY_STOP>)) return;
    if (!s.fault.empty()) { fault(s.fault); return; }
    if (context_.generation >= std::numeric_limits<std::uint64_t>::max() - 4) {
        emergency("Task generation exhausted; maintenance required"); return;
    }
    if (context_.job != Job::none && now >= deadline_) { fault("Task deadline exceeded"); return; }
    if (machine_.is(sml::state<INITIALIZING>) && !s.ready) { fault("Initialization resources lost"); return; }
    const bool following = machine_.is(sml::state<GRASPING>) || machine_.is(sml::state<FOLLOWING>) ||
                           machine_.is(sml::state<CONTROL>);
    if ((following && (!s.ready || !s.following_authorized)) ||
        ((machine_.is(sml::state<RELEASING>) || machine_.is(sml::state<READY>) ||
          machine_.is(sml::state<HOMING>) || machine_.is(sml::state<STANDBY>)) && !s.ready) ||
        (machine_.is(sml::state<CONTROL>) && (!s.source_authorized || !s.input_ready)))
        safetyLost("Required resource, authorization or control input lost");
}
Reply RobotStateMachine::request(Operation op, const Snapshot& s, std::uint64_t now) {
    const TransitionLog transition{*this, operationName(op)};
    const auto reply = [&](Reply result) {
        const auto level = result == Reply::accepted || result == Reply::completed
            ? spdlog::level::info : spdlog::level::warn;
        Logger::log(level, "Core FSM operation={} reply={} state={} generation={} "
            "ready={} settled={} executor_idle={} clear_of_wheel={} following_authorized={} "
            "source_authorized={} input_ready={} release_authorized={} fault_cleared={}",
            operationName(op), replyName(result), state(), generation(), s.ready, s.settled,
            s.executor_idle, s.clear_of_wheel, s.following_authorized, s.source_authorized,
            s.input_ready, s.release_authorized, s.fault_cleared);
        return result;
    };
    supervise(s, now); // Protection always precedes ordinary requests.
    if (machine_.is(sml::state<EMERGENCY_STOP>)) return reply(Reply::invalid_state);
    if (context_.job != Job::none) return reply(Reply::busy);
    bool accepted = false;
    switch (op) {
    case Operation::enter_standby: accepted = machine_.process_event(ENTER_STANDBY{}); break;
    case Operation::grasp_wheel: accepted = machine_.process_event(GRASP_WHEEL{}); break;
    case Operation::start_control: accepted = machine_.process_event(START_CONTROL{}); break;
    case Operation::exit_control: accepted = machine_.process_event(EXIT_CONTROL{}); break;
    case Operation::leave_wheel: accepted = machine_.process_event(LEAVE_WHEEL{}); break;
    case Operation::reset_error: accepted = machine_.process_event(RESET_ERROR{}); break;
    }
    if (accepted) {
        if (context_.job != Job::none) { deadline(now); return reply(Reply::accepted); }
        if (op == Operation::start_control) control_since_ = now;
        if (op == Operation::reset_error) error_.clear();
        return reply(Reply::completed);
    }
    const bool permitted =
        (machine_.is(sml::state<READY>) && op == Operation::enter_standby) ||
        (machine_.is(sml::state<STANDBY>) && op == Operation::grasp_wheel) ||
        (machine_.is(sml::state<FOLLOWING>) && (op == Operation::start_control ||
            op == Operation::leave_wheel || op == Operation::enter_standby)) ||
        (machine_.is(sml::state<SAFE>) && (op == Operation::leave_wheel || op == Operation::enter_standby)) ||
        (machine_.is(sml::state<ERROR>) && op == Operation::reset_error);
    if (!permitted) return reply(Reply::invalid_state);
    if (!s.executor_idle || !s.settled) return reply(Reply::busy);
    if (((machine_.is(sml::state<SAFE>) || machine_.is(sml::state<READY>)) && op == Operation::enter_standby && !s.clear_of_wheel) ||
        (op == Operation::reset_error && !s.fault_cleared)) return reply(Reply::invalid_state);
    return reply(Reply::capability_unavailable);
}
void RobotStateMachine::done(std::uint64_t generation, const Snapshot& s, std::uint64_t now) {
    const TransitionLog transition{*this, "Done"};
    if (!now) now = last_now_;
    last_now_ = now;
    if (generation != context_.generation || context_.job == Job::none) {
        Logger::debug("Core FSM ignored completion generation={} active_generation={} job={}",
            generation, context_.generation, static_cast<int>(context_.job));
        return;
    }
    snapshot(s);
    if (!s.emergency_known || s.emergency_latched) { emergency("Emergency during completion"); return; }
    if (!s.fault.empty()) { fault(s.fault); return; }
    if (context_.job == Job::grasp && !s.following_authorized) { safetyLost("Following authorization lost"); return; }
    const auto completed = context_.job;
    if (!machine_.process_event(Done{generation})) fault("Task completed without required postconditions");
    else {
        if (completed == Job::initialize || completed == Job::home || completed == Job::release) error_.clear();
        if (context_.job != Job::none) deadline(now);
    }
}
void RobotStateMachine::failed(std::uint64_t generation, const std::string& why) {
    if (context_.job != Job::none && generation == context_.generation) fault(why);
}
void RobotStateMachine::fault(const std::string& why) {
    const TransitionLog transition{*this, "Fault"};
    if (error_ != why || !machine_.is(sml::state<ERROR>))
        Logger::error("Core FSM fault state={} job={} generation={} deadline_us={} reason={}",
            state(), static_cast<int>(context_.job), generation(), deadline_, why);
    error_ = last_error_ = why;
    machine_.process_event(Fault{}); // ERROR updates diagnostics only; no second stop.
}
void RobotStateMachine::safetyLost(const std::string& why) {
    const TransitionLog transition{*this, "SafetyLost"};
    if (machine_.process_event(SafetyLost{})) {
        error_ = last_error_ = why;
        Logger::warn("Core FSM safety lost reason={} ready={} following_authorized={} "
            "source_authorized={} input_ready={}", why, context_.ready,
            context_.following_authorized, context_.source_authorized, context_.input_ready);
    }
}
void RobotStateMachine::emergency(const std::string& why) {
    const TransitionLog transition{*this, "Emergency"};
    if (machine_.process_event(Emergency{})) {
        error_ = last_error_ = why;
        Logger::critical("Core FSM emergency latched generation={} reason={}", generation(), why);
    }
}
std::optional<Task> RobotStateMachine::takeTask() {
    if (!context_.pending_job) return std::nullopt;
    Logger::info("Core FSM task dispatched job={} generation={} deadline_us={}",
        static_cast<int>(context_.job), context_.generation, deadline_);
    context_.pending_job = false;
    return Task{context_.job, context_.generation, deadline_};
}
bool RobotStateMachine::takeStop() { return std::exchange(context_.stop_requested, false); }
bool RobotStateMachine::takeBrake() { return std::exchange(context_.brake_requested, false); }
const char* RobotStateMachine::state() const {
#define STATE(s) if (machine_.is(sml::state<s>)) return #s
    STATE(INIT); STATE(INITIALIZING); STATE(READY); STATE(HOMING); STATE(STANDBY); STATE(GRASPING); STATE(FOLLOWING);
    STATE(CONTROL); STATE(RELEASING); STATE(SAFE); STATE(ERROR); STATE(EMERGENCY_STOP);
#undef STATE
    throw std::logic_error("Unknown SML state");
}
unsigned RobotStateMachine::stateCode() const {
    if (machine_.is(sml::state<INITIALIZING>)) return 10;
    if (machine_.is(sml::state<READY>)) return 11;
    if (machine_.is(sml::state<HOMING>)) return 12;
    const char* states[] = {"INIT", "STANDBY", "GRASPING", "FOLLOWING", "CONTROL", "SAFE", "ERROR", "EMERGENCY_STOP", "RELEASING"};
    for (unsigned i = 0; i < 9; ++i) if (std::string(state()) == states[i]) return i;
    throw std::logic_error("Unknown state code");
}
const char* replyName(Reply reply) {
    switch (reply) {
    case Reply::accepted: return "ACCEPTED";
    case Reply::completed: return "COMPLETED";
    case Reply::busy: return "BUSY";
    case Reply::invalid_state: return "INVALID_STATE";
    case Reply::capability_unavailable: return "CAPABILITY_UNAVAILABLE";
    }
    return "INVALID_STATE";
}
} // namespace aviator::fsm

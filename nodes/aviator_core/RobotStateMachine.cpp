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
    for (auto value : {t.initialize, t.grasp, t.release, t.home, t.recovery})
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
    const bool operating = !machine_.is(sml::state<INIT>) && !machine_.is(sml::state<ERROR>);
    const bool following = machine_.is(sml::state<GRASPING>) || machine_.is(sml::state<FOLLOWING>) ||
                           machine_.is(sml::state<CONTROL>);
    const bool controlling = machine_.is(sml::state<CONTROL>);
    if (controlling && s.input_timed_out) { fault("Control input deadline exceeded"); return; }
    // Preserve the state, task and generation during a bounded recoverable loss.
    // Track causes independently so alternating losses cannot renew the deadline.
    const auto expired = [&](bool lost, std::optional<std::uint64_t>& since) {
        if (!lost) { since.reset(); return false; }
        if (!since) since = now;
        return now >= *since && now - *since >= timeouts_.recovery;
    };
    const bool resource_timeout = expired(operating && !s.ready, resource_lost_);
    const bool following_timeout = expired(following && !s.following_authorized, following_lost_);
    const bool input_timeout = expired(controlling && (!s.source_authorized || !s.input_ready), input_lost_);
    if (resource_timeout) fault("Required resource recovery deadline exceeded");
    else if (following_timeout) fault("Following authorization recovery deadline exceeded");
    else if (input_timeout) fault("Control input/authorization recovery deadline exceeded");
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
    const bool leaving = (op == Operation::leave_wheel || op == Operation::enter_standby) &&
        (machine_.is(sml::state<CONTROL>) || machine_.is(sml::state<FOLLOWING>));
    if (context_.job != Job::none && !(leaving && context_.job == Job::following_impedance))
        return reply(Reply::busy);
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
        (machine_.is(sml::state<READY>) && (op == Operation::enter_standby || op == Operation::leave_wheel)) ||
        (machine_.is(sml::state<STANDBY>) && op == Operation::grasp_wheel) ||
        (machine_.is(sml::state<FOLLOWING>) && (op == Operation::start_control ||
            op == Operation::leave_wheel || op == Operation::enter_standby)) ||
        (machine_.is(sml::state<CONTROL>) && leaving) ||
        (machine_.is(sml::state<ERROR>) && op == Operation::reset_error);
    if (!permitted) return reply(Reply::invalid_state);
    if (!s.executor_idle || !s.settled) return reply(Reply::busy);
    if ((machine_.is(sml::state<READY>) && op == Operation::enter_standby && !s.clear_of_wheel) ||
        (machine_.is(sml::state<READY>) && op == Operation::leave_wheel && s.clear_of_wheel) ||
        (op == Operation::reset_error && !s.fault_cleared)) return reply(Reply::invalid_state);
    return reply(Reply::capability_unavailable);
}
bool RobotStateMachine::done(std::uint64_t generation, const Snapshot& s, std::uint64_t now) {
    const TransitionLog transition{*this, "Done"};
    if (!now) now = last_now_;
    if (generation != context_.generation || context_.job == Job::none) {
        Logger::debug("Core FSM ignored completion generation={} active_generation={} job={}",
            generation, context_.generation, static_cast<int>(context_.job));
        return true;
    }
    supervise(s, now); // Completion cannot beat an expired deadline or a blocking fault.
    if (generation != context_.generation || context_.job == Job::none) return true;
    const auto completed = context_.job;
    // Keep the completion pending while recoverable postconditions settle. The
    // original task deadline remains in force; never resubmit the physical action.
    if (!machine_.process_event(Done{generation})) return false;
    if (completed == Job::initialize || completed == Job::home || completed == Job::release) error_.clear();
    if (context_.job != Job::none) deadline(now);
    return true;
}
void RobotStateMachine::failed(std::uint64_t generation, const std::string& why) {
    if (context_.job != Job::none && generation == context_.generation) fault(why);
}
void RobotStateMachine::fault(const std::string& why) {
    const TransitionLog transition{*this, "Fault"};
    if (error_ != why || !machine_.is(sml::state<ERROR>))
        Logger::error("Core FSM fault state={} job={} generation={} deadline_us={} reason={}",
            state(), static_cast<int>(context_.job), generation(), deadline_, why);
    resource_lost_.reset(); following_lost_.reset(); input_lost_.reset();
    error_ = last_error_ = why;
    machine_.process_event(Fault{}); // ERROR updates diagnostics only; no second stop.
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
    STATE(CONTROL); STATE(RELEASING); STATE(ERROR); STATE(EMERGENCY_STOP);
#undef STATE
    throw std::logic_error("Unknown SML state");
}
unsigned RobotStateMachine::stateCode() const {
    if (machine_.is(sml::state<INITIALIZING>)) return 10;
    if (machine_.is(sml::state<READY>)) return 11;
    if (machine_.is(sml::state<HOMING>)) return 12;
    // Preserve protocol numbers: retired SAFE=5 remains an unused slot.
    const char* states[] = {"INIT", "STANDBY", "GRASPING", "FOLLOWING", "CONTROL", "", "ERROR", "EMERGENCY_STOP", "RELEASING"};
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

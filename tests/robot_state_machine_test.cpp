#include "aviator/RobotStateMachine.hpp"
#include <iostream>
#include <stdexcept>
using namespace aviator::fsm;
void check(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
Snapshot healthy() {
    Snapshot s;
    s.ready = s.settled = s.clear_of_wheel = s.following_authorized = true;
    s.source_authorized = s.input_ready = s.fault_cleared = s.executor_idle = true;
    s.release_authorized = s.emergency_known = true;
    return s;
}
void state(const RobotStateMachine& m, const char* expected) { check(std::string(m.state()) == expected, expected); }
void finish(RobotStateMachine& m, Snapshot s = healthy()) {
    auto task = m.takeTask(); check(bool(task), "missing task");
    check(!m.takeTask(), "task submitted twice"); m.done(task->generation, s);
    if (m.job() == Job::following_impedance) {
        auto profile = m.takeTask(); check(bool(profile), "missing following impedance task");
        check(!m.acceptsControl(), "following impedance enabled control");
        m.done(profile->generation, s);
    }
}
void following(RobotStateMachine& m) {
    m.boot(healthy(), 1); finish(m);
    check(m.request(Operation::enter_standby, healthy(), 2) == Reply::accepted, "home rejected");
    finish(m);
    check(m.request(Operation::grasp_wheel, healthy(), 2) == Reply::accepted, "grasp rejected");
    finish(m); state(m, "FOLLOWING");
}
int main() try {
    auto s = healthy(); RobotStateMachine m;
    state(m, "INIT"); m.boot(s, 1); state(m, "INITIALIZING"); check(m.stateCode() == 10, "init code");
    check(m.request(Operation::enter_standby, s, 2) == Reply::busy, "transition not busy");
    auto job = m.takeTask(); m.done(job->generation + 1, s); state(m, "INITIALIZING");
    m.done(job->generation, s); state(m, "READY");
    check(!m.takeTask(), "initialization automatically started home");
    check(m.request(Operation::leave_wheel, s, 2) == Reply::invalid_state, "release from clear READY accepted");
    check(m.request(Operation::grasp_wheel, s, 2) == Reply::invalid_state, "grasp before home accepted");
    check(m.request(Operation::enter_standby, s, 2) == Reply::accepted, "external home rejected");
    state(m, "HOMING"); check(m.stateCode() == 12, "home state code");
    check(m.request(Operation::grasp_wheel, s, 2) == Reply::busy, "grasp during home accepted");
    finish(m); state(m, "STANDBY");
    check(m.request(Operation::start_control, s, 3) == Reply::invalid_state, "standby control accepted");
    check(m.request(Operation::enter_standby, s, 4) == Reply::completed && !m.takeTask(), "standby idempotence");
    s.following_authorized = false;
    check(m.request(Operation::grasp_wheel, s, 5) == Reply::capability_unavailable, "guard");
    s = healthy(); m.request(Operation::grasp_wheel, s, 6); finish(m); state(m, "FOLLOWING");
    check(m.request(Operation::start_control, s, 7) == Reply::completed, "control no longer transitions directly");
    state(m, "CONTROL"); check(m.acceptsControl(), "gate disabled");
    check(!m.takeTask(), "START_CONTROL created an extra FSM task");
    check(m.controlSince() == 7, "input boundary");
    m.request(Operation::exit_control, s, 9); state(m, "FOLLOWING");
    check(!m.acceptsControl() && m.takeStop(), "exit must decelerate");
    s.settled = false;
    check(m.request(Operation::start_control, s, 10) == Reply::busy, "unsettled control");
    m.request(Operation::exit_control, s, 11); check(!m.takeStop(), "idempotent exit repeated stop");
    s = healthy(); finish(m); m.request(Operation::leave_wheel, s, 12); state(m, "RELEASING"); finish(m); state(m, "STANDBY");
    m.fault("first"); state(m, "ERROR"); check(m.takeStop(), "fault stop");
    m.fault("second"); check(!m.takeStop() && m.lastError() == "second", "fault repeat");
    s.fault_cleared = false;
    check(m.request(Operation::reset_error, s, 13) == Reply::invalid_state, "uncleared reset");
    s = healthy(); m.request(Operation::reset_error, s, 14); state(m, "READY");
    check(m.currentError().empty() && m.lastError() == "second", "error history");
    m.request(Operation::enter_standby, s, 15); state(m, "HOMING"); finish(m); state(m, "STANDBY");
    RobotStateMachine failure; failure.boot(s, 1); auto old = failure.takeTask(); failure.failed(old->generation, "submit failed");
    failure.done(old->generation, s); state(failure, "ERROR");
    RobotStateMachine timeout({10,10,10}); timeout.boot(s, 1); auto stale = timeout.takeTask();
    timeout.supervise(s, 11); timeout.done(stale->generation, s); state(timeout, "ERROR");
    // Completion evidence may recover without executing the action twice.
    RobotStateMachine post; post.boot(s, 1); auto pending = post.takeTask();
    s.clear_of_wheel = false;
    check(!post.done(pending->generation, s, 2), "invalid postcondition accepted");
    state(post, "INITIALIZING"); check(!post.takeTask(), "completion resubmitted task");
    s.clear_of_wheel = true;
    check(post.done(pending->generation, s, 3), "recovered completion lost"); state(post, "READY");
    RobotStateMachine stuck({10, 10, 10, 10}); stuck.boot(s, 1); pending = stuck.takeTask();
    s.clear_of_wheel = false; stuck.done(pending->generation, s, 2);
    stuck.done(pending->generation, healthy(), 11); state(stuck, "ERROR");
    RobotStateMachine transient; following(transient); s = healthy();
    const auto generation = transient.generation();
    s.following_authorized = false; transient.supervise(s, 10); state(transient, "FOLLOWING");
    check(!transient.takeStop() && transient.generation() == generation, "recoverable loss cancelled task");
    s.following_authorized = true; transient.supervise(s, 100009); state(transient, "FOLLOWING");
    s.following_authorized = false; transient.supervise(s, 100010);
    transient.supervise(s, 200009); state(transient, "FOLLOWING");
    transient.supervise(s, 200010); state(transient, "ERROR");
    check(transient.takeStop(), "persistent loss failed to stop");
    RobotStateMachine input_lost; following(input_lost); s = healthy();
    input_lost.request(Operation::start_control, s, 3); s.input_ready = false;
    input_lost.supervise(s, 4); state(input_lost, "CONTROL");
    s = healthy(); input_lost.supervise(s, 5); state(input_lost, "CONTROL");
    s.input_timed_out = true; input_lost.supervise(s, 6); state(input_lost, "ERROR");
    check(!input_lost.acceptsControl(), "expired input left execution enabled");
    s = healthy(); s.clear_of_wheel = false;
    check(input_lost.request(Operation::reset_error, s, 7) == Reply::completed, "grasped reset rejected");
    state(input_lost, "READY");
    check(input_lost.currentError().empty() && !input_lost.lastError().empty(), "recovery error history");
    check(input_lost.request(Operation::enter_standby, s, 8) == Reply::invalid_state, "home while grasped");
    check(input_lost.request(Operation::start_control, s, 8) == Reply::invalid_state, "reset resumed control");
    check(input_lost.request(Operation::leave_wheel, s, 9) == Reply::accepted, "READY release rejected");
    finish(input_lost); state(input_lost, "STANDBY");
    // Both operations accept one-step leave from FOLLOWING/CONTROL, including motion.
    for (bool control : {false, true}) for (auto op : {Operation::leave_wheel, Operation::enter_standby}) {
        RobotStateMachine leave; following(leave); s = healthy();
        if (control) leave.request(Operation::start_control, s, 3);
        s.settled = false; s.executor_idle = false; s.release_authorized = false;
        check(leave.request(op, s, 4) != Reply::accepted, "unauthorized release accepted");
        s.release_authorized = true;
        check(leave.request(op, s, 5) == Reply::accepted, "direct moving release rejected");
        state(leave, "RELEASING"); check(leave.takeStop() && !leave.acceptsControl(), "leave missed stop/gate");
        check(leave.request(op, s, 6) == Reply::busy, "duplicate release restarted task");
        finish(leave); state(leave, "STANDBY");
    }
    RobotStateMachine switching; following(switching); s = healthy();
    switching.request(Operation::start_control, s, 3);
    switching.request(Operation::exit_control, s, 4); auto cancelled = switching.takeTask();
    s.settled = s.executor_idle = false;
    check(switching.request(Operation::leave_wheel, s, 5) == Reply::accepted, "impedance prevented leave");
    switching.done(cancelled->generation, healthy(), 6); state(switching, "RELEASING");
    finish(switching); state(switching, "STANDBY");
    // A transient resource loss must preserve an active job and its generation.
    RobotStateMachine working; working.boot(healthy(), 1); pending = working.takeTask();
    s = healthy(); s.ready = false; working.supervise(s, 2);
    check(working.generation() == pending->generation && !working.takeStop(), "transient cancelled initialization");
    working.done(pending->generation, healthy(), 3); state(working, "READY");
    working.fault("device"); s = healthy(); s.ready = false;
    check(working.request(Operation::reset_error, s, 4) != Reply::completed, "reset without resources");
    state(working, "ERROR");
    // Every non-emergency state has an emergency edge and no normal exit afterwards.
    for (const std::string wanted : {"INIT", "INITIALIZING", "READY", "HOMING", "STANDBY", "GRASPING", "FOLLOWING", "CONTROL", "RELEASING", "ERROR"}) {
        RobotStateMachine e; s = healthy();
        if (wanted != "INIT") e.boot(s, 1);
        if (wanted != "INIT" && wanted != "INITIALIZING") finish(e);
        if (wanted != "INIT" && wanted != "INITIALIZING" && wanted != "READY") {
            e.request(Operation::enter_standby, s, 2);
            if (wanted != "HOMING") finish(e);
        }
        if (wanted == "GRASPING" || wanted == "FOLLOWING" || wanted == "CONTROL" || wanted == "RELEASING") {
            e.request(Operation::grasp_wheel, s, 3);
            if (wanted != "GRASPING") finish(e);
        }
        if (wanted == "CONTROL") { e.request(Operation::start_control, s, 4); }
        if (wanted == "RELEASING") e.request(Operation::leave_wheel, s, 4);
        if (wanted == "ERROR") e.fault("fault");
        state(e, wanted.c_str());
        auto generation = e.generation(); e.emergency("hardware latch"); state(e, "EMERGENCY_STOP");
        check(e.takeBrake(), "no independent brake request"); e.emergency("again"); check(!e.takeBrake(), "brake repeated");
        for (auto op : {Operation::enter_standby,Operation::grasp_wheel,Operation::start_control,
                        Operation::exit_control,Operation::leave_wheel,Operation::reset_error})
            check(e.request(op, s, 5) == Reply::invalid_state, "emergency escaped");
        e.done(generation,s); e.fault("brake failure"); e.boot(s,6); state(e,"EMERGENCY_STOP");
    }
    RobotStateMachine unknown; s = healthy(); s.emergency_known = false; unknown.boot(s,1);
    state(unknown,"EMERGENCY_STOP"); check(!unknown.takeTask(), "boot preceded latch evidence");
    std::cout << "Robot state machine tests passed (no hardware)\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

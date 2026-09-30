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
    m.request(Operation::start_control, s, 7); state(m, "CONTROL"); check(m.acceptsControl(), "gate disabled");
    check(m.controlSince() == 7, "input boundary");
    check(m.request(Operation::leave_wheel, s, 8) == Reply::invalid_state, "control release");
    m.request(Operation::exit_control, s, 9); state(m, "FOLLOWING");
    check(!m.acceptsControl() && m.takeStop(), "exit must decelerate");
    s.settled = false;
    check(m.request(Operation::leave_wheel, s, 10) == Reply::busy, "unsettled release");
    m.request(Operation::exit_control, s, 11); check(!m.takeStop(), "idempotent exit repeated stop");
    s = healthy(); m.request(Operation::leave_wheel, s, 12); state(m, "RELEASING"); finish(m); state(m, "STANDBY");
    m.fault("first"); state(m, "ERROR"); check(m.takeStop(), "fault stop");
    m.fault("second"); check(!m.takeStop() && m.lastError() == "second", "fault repeat");
    s.fault_cleared = false;
    check(m.request(Operation::reset_error, s, 13) == Reply::invalid_state, "uncleared reset");
    s = healthy(); m.request(Operation::reset_error, s, 14); state(m, "SAFE");
    check(m.currentError().empty() && m.lastError() == "second", "error history");
    m.request(Operation::enter_standby, s, 15); state(m, "HOMING"); finish(m); state(m, "STANDBY");
    RobotStateMachine failure; failure.boot(s, 1); auto old = failure.takeTask(); failure.failed(old->generation, "submit failed");
    failure.done(old->generation, s); state(failure, "ERROR");
    RobotStateMachine timeout({10,10,10}); timeout.boot(s, 1); auto stale = timeout.takeTask();
    timeout.supervise(s, 11); timeout.done(stale->generation, s); state(timeout, "ERROR");
    RobotStateMachine post; post.boot(s, 1); s.clear_of_wheel = false; finish(post, s); state(post, "ERROR");
    RobotStateMachine unsafe; following(unsafe); s = healthy(); s.following_authorized = false;
    unsafe.supervise(s, 10); state(unsafe, "SAFE");
    s = healthy(); s.executor_idle = false;
    check(unsafe.request(Operation::enter_standby, s, 11) == Reply::busy, "cancel not confirmed");
    RobotStateMachine input_lost; following(input_lost); s = healthy();
    input_lost.request(Operation::start_control, s, 3); s.input_ready = false;
    input_lost.supervise(s, 4); state(input_lost, "SAFE");
    check(!input_lost.acceptsControl(), "stale input left execution enabled");
    s = healthy(); input_lost.request(Operation::enter_standby, s, 5); finish(input_lost);
    check(input_lost.currentError().empty() && !input_lost.lastError().empty(), "recovery error history");
    // Every non-emergency state has an emergency edge and no normal exit afterwards.
    for (const std::string wanted : {"INIT", "INITIALIZING", "READY", "HOMING", "STANDBY", "GRASPING", "FOLLOWING", "CONTROL", "RELEASING", "SAFE", "ERROR"}) {
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
        if (wanted == "CONTROL") e.request(Operation::start_control, s, 4);
        if (wanted == "RELEASING") e.request(Operation::leave_wheel, s, 4);
        if (wanted == "SAFE" || wanted == "ERROR") e.fault("fault");
        if (wanted == "SAFE") e.request(Operation::reset_error, s, 4);
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

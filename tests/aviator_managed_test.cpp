#include "aviator/Aviator.hpp"
#include "ManagedReadiness.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/CollisionChecker.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>
using namespace aviator;
void check(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
template<class F> void rejects(F action) {
    bool rejected = false;
    try { action(); } catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "invalid API/mode/thread was accepted");
}
int main() try {
    DeviceState idle;
    for (const auto* phase : {"UNINITIALIZED", "INITIALIZED", "DISABLED"}) {
        check(managedResourcesReady("rokae", phase, idle, false, true), "Rokae pre-enable deadlock");
        check(!managedResourcesReady("rokae", phase, idle, false, false), "stale status accepted");
        check(!managedResourcesReady("mujoco", phase, idle, false, true), "invalid simulation feedback accepted");
    }
    for (const auto* phase : {"ENABLED", "APPROACHING", "LOCKED", "SERVO", "RELEASING"})
        check(!managedResourcesReady("rokae", phase, idle, false, true), "motion bypassed realtime feedback");
    idle.fault = true;
    check(!managedResourcesReady("rokae", "INITIALIZED", idle, true, true), "device fault ignored");
    idle.fault = false; idle.id = 1;
    check(!managedResourcesReady("rokae", "INITIALIZED", idle, false, true), "active trajectory bypassed feedback");
    idle.id = 0; idle.stopping = true;
    check(!managedResourcesReady("rokae", "INITIALIZED", idle, false, true), "stop in progress bypassed feedback");
    // No device object; missing configuration intentionally fails before executor IO.
    Aviator direct(nullptr, nullptr, nullptr, "/does-not-exist/aviator-managed-test.yaml");
    rejects([&] { direct.Init(); });
    rejects([&] { direct.GraspWheel(); });
    direct.stop();
    bool allowed = true;
    int heartbeats = 0, brakes = 0;
    ManagedOptions options;
    options.snapshot = [] {
        fsm::Snapshot s;
        s.ready = s.settled = s.clear_of_wheel = s.following_authorized = true;
        s.release_authorized = s.source_authorized = s.input_ready = s.fault_cleared = s.emergency_known = true;
        return s;
    };
    options.allow_motion = [&](bool value) { allowed = value; };
    options.heartbeat = [&] { ++heartbeats; };
    options.request_brake = [&] { ++brakes; };
    {
        Aviator managed(nullptr, nullptr, nullptr, "/does-not-exist/aviator-managed-test.yaml", options);
        check(!allowed, "managed constructor left output enabled");
        rejects([&] { managed.init(); });
        rejects([&] { managed.enable(); });
        rejects([&] { managed.approachHandles(); });
        rejects([&] { managed.servoWheel(0, 0); });
        rejects([&] { managed.stop(); });
        rejects([&] { managed.GraspWheel(); }); // Boot has not run.
        bool wrong_thread = false;
        std::thread foreign([&] { try { managed.Update(); } catch (const std::runtime_error&) { wrong_thread = true; } });
        foreign.join(); check(wrong_thread, "FSM accepted a foreign owner");
        managed.Init();
        rejects([&] { managed.Init(); });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (managed.GetSystemState() != "ERROR" && std::chrono::steady_clock::now() < deadline) {
            managed.Update(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        check(managed.GetSystemState() == "ERROR" && !allowed, "executor failure did not revoke output");
        check(!managed.GetSystemStatus().current_error.empty(), "executor failure lost diagnostic");
        check(!managed.ServoWheel(0, 0, .5, 1), "ERROR accepted a target");
        managed.EmergencyStop("test");
        managed.Update();
        check(managed.GetSystemState() == "EMERGENCY_STOP" && brakes == 1, "emergency not latched");
        check(managed.ResetError() == fsm::Reply::invalid_state, "emergency reset accepted");
    }
    {
        Aviator cancelled(nullptr, nullptr, nullptr, "/does-not-exist/aviator-managed-test.yaml", options);
        cancelled.Init();
        cancelled.EmergencyStop("cancel pending task");
        for (int i = 0; i < 10; ++i) { cancelled.Update(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
        check(cancelled.GetSystemState() == "EMERGENCY_STOP", "late worker result escaped emergency");
    }
    check(!allowed && heartbeats > 0, "shutdown lease behavior");
    std::cout << "PASS managed API: modes, owner thread, task failure/revoke, emergency and late completion\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

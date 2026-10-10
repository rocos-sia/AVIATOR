#include "device_server.hpp"
#include "RemoteLink.hpp"
#include "ManagedReadiness.hpp"
#include <chrono>
#include <csignal>
#include <future>
#include <iostream>
#include <thread>
using namespace aviator;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalStop(int) { interrupted = 1; }
void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
// Protocol/executor test double. Real Ruckig/IK is exercised by servo_planner_test.
class LatestProbe final : public DeviceServo {
    JointFrame frame_;
    bool stopped_ = false;
public:
    explicit LatestProbe(JointFrame f) : frame_(f) {}
    JointFrame step(const ServoGoal& g) override {
        if (g.angle == .4) {
            std::cout << "SERVO_STEP_FAILURE" << std::endl;
            throw std::runtime_error("injected servo planning failure");
        }
        stopped_ = g.stop;
        if (!stopped_) { frame_.angle = g.angle; frame_.displacement = g.displacement; }
        return frame_;
    }
    bool stopped() const override { return stopped_; }
};
// No hardware. Deliberately suspend feedback longer than the 50 ms RT watchdog.
class PausingDevice final : public DataLink {
    ArmFeedback feedback_;
    std::array<double, 7> stiffness_;
    std::string mode_;
    bool locked_ = false;
public:
    PausingDevice(const DeviceSettings& settings, const std::string& mode)
        : stiffness_(settings.default_stiffness), mode_(mode) {
        for (int j = 0; j < 14; ++j) feedback_.q[j] = feedback_.target[j] = (settings.lower[j] + settings.upper[j]) / 2;
        feedback_.tcp[6] = feedback_.tcp[13] = 1;
    }
    ArmFeedback armFeedback() const override { return feedback_; }
    double getJointPosition(Side s, int j) const override { return feedback_.q[int(s)*7+j]; }
    double getJointVelocity(Side, int) const override { return 0; }
    void setJointPositions(const Joints& q) override { feedback_.q = feedback_.target = q; }
    Joints jointTargets() const override { return feedback_.target; }
    double jointVelLimit(Side, int) const override { return 1; }
    bool isEnabled(Side s) const override { return feedback_.enabled[int(s)]; }
    void enable(Side s) override {
        feedback_.enabled[int(s)] = feedback_.valid[int(s)] = true;
        feedback_.sample_time[int(s)] = time();
    }
    void disable(Side s) override { feedback_.enabled[int(s)] = feedback_.valid[int(s)] = false; }
    GraspState graspState() const override {
        GraspState s; s.heartbeat = time(); s.locked = locked_ ? 3 : 0;
        s.angle = .15; s.displacement = -.09;
        return s;
    }
    uint64_t sendGraspCommand(GraspCommand c) override { locked_ = c == GraspCommand::Lock; return 0; }
    void waitTick() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        for (int s = 0; s < 2; ++s) if (feedback_.enabled[s]) feedback_.sample_time[s] = time();
    }
    double time() const override { return double(monotonic_us()) / 1e6; }
    void setJointStiffness(const std::array<double, 7>& stiffness, const std::function<void()>& guard,
                           bool force_reapply = false) override {
        if (!force_reapply && stiffness == stiffness_) return;
        for (int i = 0; i < (mode_ == "timeout" ? 1200 : 60); ++i) {
            guard();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (mode_ == "failure") throw std::runtime_error("injected right arm restart failure");
        guard();
        if (mode_ == "rebase") feedback_.q[10] -= .2; // Right J4 displaced during compliant hold.
        feedback_.target = feedback_.q;
        stiffness_ = stiffness;
        waitTick();
        std::cout << "RESUMED " << stiffness[6] << std::endl;
    }
};
}
int main(int argc, char** argv) try {
    check(argc == 4, "Usage: driver server|client config success|failure|cancel|heartbeat|timeout|unchanged|fallback");
    const auto config = loadMotionConfig(argv[2]);
    const std::string mode = argv[3];
    const bool after_servo = mode == "latest_impedance" || mode == "latest_heartbeat" || mode == "latest_cancel";
    const bool lose_heartbeat = mode == "heartbeat" || mode == "latest_heartbeat";
    const bool cancel_switch = mode == "cancel" || mode == "latest_cancel";
    if (std::string(argv[1]) == "server") {
        std::signal(SIGINT, signalStop); std::signal(SIGTERM, signalStop);
        DeviceSettings settings(config);
        PausingDevice device(settings, mode);
        DeviceServerOptions options;
        options.wheel_measurement = mode == "wheel_rebase";
        if (mode == "latest" || after_servo) options.servo = [](const JointFrame& f) { return std::make_unique<LatestProbe>(f); };
        return runDeviceServer(device, config, settings, options, interrupted);
    }
    RemoteLink link(config);
    link.enable(Side::Left);
    for (int i = 0; i < 30; ++i) { link.heartbeat(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    if (mode == "latest") {
        check(link.latestServoSupported(), "latest capability missing");
        auto wait = [&](auto predicate) {
            const auto deadline = monotonic_us() + 2000000;
            for (;;) {
                link.heartbeat();
                bool fresh;
                const auto state = link.snapshot(fresh);
                check(!state.fault, state.error.c_str());
                if (fresh && predicate(state)) return state;
                check(monotonic_us() < deadline, "latest state timeout");
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        // All targets change within one publish period; only the last matters.
        for (int k = 0; k < 1000; ++k)
            link.latestServo({k == 999 ? -.2 : .2, -.085, 1, monotonic_us(), false});
        const auto active = wait([](const DeviceState& s) { return s.servo_active && s.angle == -.2; });
        // Repeated publication must not refresh the original input timestamp.
        const auto stopped = wait([](const DeviceState& s) { return s.servo_stopped && !s.servo_active; });
        check(stopped.servo_input_sample == active.servo_input_sample, "heartbeat refreshed input timestamp");
        check(!link.latestServo({.3, -.05, 1, monotonic_us(), false}), "device timeout not surfaced to Core");
        for (int i = 0; i < 40; ++i) { link.heartbeat(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
        bool fresh; auto state = link.snapshot(fresh);
        check(state.servo_stopped && state.angle == -.2, "late goal restarted stopped servo ID");
        link.finishLatestServo();
        link.latestServo({.1, -.1, 1, monotonic_us(), false});
        state = wait([](const DeviceState& s) { return s.servo_active && s.angle == .1; });
        check(state.id > stopped.id, "new Servo did not get new ID");
        auto done = std::async(std::launch::async, [&] { link.finishLatestServo(); });
        while (done.wait_for(std::chrono::milliseconds(2)) != std::future_status::ready) link.heartbeat();
        done.get();
        link.latestServo({.2, -.1, 1, monotonic_us(), false});
        wait([](const DeviceState& s) { return s.servo_active; });
        // Protection stop clears latest output; allowMotion does not replay it.
        link.allowMotion(false);
        wait([](const DeviceState& s) { return !s.servo_active && !s.stopping; });
        for (int i = 0; i < 40; ++i) { link.heartbeat(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
        link.allowMotion(true);
        check(!link.snapshot(fresh).servo_active, "reauthorization replayed old servo");
        link.latestServo({.4, -.1, 1, monotonic_us(), false});
        for (int i = 0; i < 300; ++i) {
            link.heartbeat();
            state = link.snapshot(fresh);
            if (state.fault && !state.enabled[0] && !state.enabled[1]) break;
            check(i < 299, "planning fault did not disable both arms");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        check(state.error.find("injected servo planning failure") != std::string::npos, "lost planning fault");
        std::cout << "PASS latest overwrite / timeout / stop / reentry / planning fault" << std::endl;
        return 0;
    }
    if (after_servo) {
        link.latestServo({.1, -.085, 1, monotonic_us(), false});
        const auto deadline = monotonic_us() + 1000000;
        for (;;) {
            link.heartbeat();
            bool fresh;
            const auto state = link.snapshot(fresh);
            check(!state.fault, state.error.c_str());
            if (fresh && state.servo_active) break;
            check(monotonic_us() < deadline, "Servo did not start before impedance test");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        auto finish = std::async(std::launch::async, [&] { link.finishLatestServo(); });
        while (finish.wait_for(std::chrono::milliseconds(2)) != std::future_status::ready) link.heartbeat();
        finish.get();
    }
    if (mode == "wheel_rebase") link.sendGraspCommand(GraspCommand::Lock);
    const auto initial_wheel = link.graspState();
    const auto target = link.jointTargets();
    auto task = std::async(std::launch::async, [&] { link.setImpedanceProfile(true); });
    bool observed_pause = false, cancelled = false;
    auto until = monotonic_us() + 7000000;
    while (task.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
        if (!lose_heartbeat) link.heartbeat();
        bool fresh = false, status = false;
        const auto state = link.snapshot(fresh, &status);
        if (!cancelled && !fresh && status && state.impedance_switching && !state.fault) {
            observed_pause = true;
            check(managedResourcesReady("STIFFNESS", state, fresh, status), "RT pause lost readiness");
            check(!link.graspState().fault, "RT pause fabricated a feedback fault");
            check(state.target == target, "held target changed during pause");
            if (cancel_switch && !cancelled) { link.allowMotion(false); cancelled = true; }
        }
        check(monotonic_us() < until, "profile operation hung");
    }
    bool failed = false;
    try { task.get(); } catch (const std::exception& e) { failed = true; std::cout << e.what() << '\n'; }
    const bool unchanged = mode == "unchanged" || mode == "fallback";
    check(observed_pause, "Following must execute the lifecycle even with unchanged stiffness");
    const bool successful = mode == "success" || mode == "latest_impedance" || mode == "rebase" ||
                            mode == "wheel_rebase" || unchanged;
    check(failed != successful, "wrong profile result");
    if (successful) {
        bool fresh, status;
        auto state = link.snapshot(fresh, &status);
        check(fresh && !state.impedance_switching && state.impedance_profile == "following", "restart not confirmed");
        auto expected = target;
        if (mode == "rebase") expected[10] -= .2;
        check(state.target == expected && link.jointTargets() == expected, "restart did not synchronize measured hold");
        auto restore = std::async(std::launch::async, [&] { link.setImpedanceProfile(false); });
        while (restore.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) link.heartbeat();
        restore.get();
        state = link.snapshot(fresh);
        check(state.impedance_profile == "default", "default profile not restored");
        if (mode == "rebase") expected[10] -= .2;
        check(state.target == expected && link.jointTargets() == expected, "default restoration replayed old target");
        const auto wheel = link.graspState();
        check(wheel.angle == (mode == "wheel_rebase" ? .15 : initial_wheel.angle) &&
              wheel.displacement == (mode == "wheel_rebase" ? -.09 : initial_wheel.displacement),
              "Impedance hold lost measured wheel rebase or used unavailable measurement");
    } else {
        for (int i = 0; i < 200; ++i) {
            link.heartbeat();
            bool fresh; const auto state = link.snapshot(fresh);
            if (state.fault && !state.enabled[0] && !state.enabled[1]) break;
            check(i < 199, "failed update did not disable both arms");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "PASS impedance lifecycle " << mode << std::endl;
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

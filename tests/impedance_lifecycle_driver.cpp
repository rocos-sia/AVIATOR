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
        GraspState s; s.heartbeat = time(); s.locked = locked_ ? 3 : 0; return s;
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
    if (std::string(argv[1]) == "server") {
        std::signal(SIGINT, signalStop); std::signal(SIGTERM, signalStop);
        DeviceSettings settings(config);
        PausingDevice device(settings, mode);
        return runDeviceServer(device, config, settings, {}, interrupted);
    }
    RemoteLink link(config);
    link.enable(Side::Left);
    for (int i = 0; i < 30; ++i) { link.heartbeat(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    const auto target = link.jointTargets();
    auto task = std::async(std::launch::async, [&] { link.setImpedanceProfile(true); });
    bool observed_pause = false, cancelled = false;
    auto until = monotonic_us() + 7000000;
    while (task.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
        if (mode != "heartbeat") link.heartbeat();
        bool fresh = false, status = false;
        const auto state = link.snapshot(fresh, &status);
        if (!cancelled && !fresh && status && state.impedance_switching && !state.fault) {
            observed_pause = true;
            check(managedResourcesReady("STIFFNESS", state, fresh, status), "RT pause lost readiness");
            check(!link.graspState().fault, "RT pause fabricated a feedback fault");
            check(state.target == target, "held target changed during pause");
            if (mode == "cancel" && !cancelled) { link.allowMotion(false); cancelled = true; }
        }
        check(monotonic_us() < until, "profile operation hung");
    }
    bool failed = false;
    try { task.get(); } catch (const std::exception& e) { failed = true; std::cout << e.what() << '\n'; }
    const bool unchanged = mode == "unchanged" || mode == "fallback";
    check(observed_pause, "Following must execute the lifecycle even with unchanged stiffness");
    const bool successful = mode == "success" || unchanged;
    check(failed != successful, "wrong profile result");
    if (successful) {
        bool fresh, status;
        auto state = link.snapshot(fresh, &status);
        check(fresh && !state.impedance_switching && state.impedance_profile == "following", "restart not confirmed");
        check(state.target == target, "restart changed held target");
        auto restore = std::async(std::launch::async, [&] { link.setImpedanceProfile(false); });
        while (restore.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) link.heartbeat();
        restore.get();
        check(link.snapshot(fresh).impedance_profile == "default", "default profile not restored");
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

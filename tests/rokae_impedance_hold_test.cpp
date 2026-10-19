// Real RokaeDataLink lifecycle and RT callback, with an SDK-boundary test double.
#include "aviator/backend.hpp"
#include "rokae_sdk.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

namespace {
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::atomic<int> first_holds{0};
std::atomic<bool> moving_on_resume{false};
std::atomic<bool> invalid_on_resume{false};
std::atomic<bool> freeze_feedback_on_hold{false};
std::atomic<int> pause_calls{0};
std::array<std::atomic<int>, 2> emergency_calls{};
std::atomic<bool> require_parallel_emergency{false}, fail_emergency_left{false};
std::array<std::atomic<double>, 2> live_hold_since{};
double expected_hold_duration = .5;
}
namespace aviator {
class RokaeArm::Impl {
public:
    mutable std::mutex mutex;
    std::array<double, 7> q{}, last_target{};
    bool follow_target = true;
    int side = 0;
    std::function<std::array<double, 7>(const RokaeSample&)> callback;
    std::function<void(const std::array<double, 7>&)> initialize;
    std::atomic<bool> quit{false}, failed{false};
    std::thread thread;
    void stop() { quit = true; if (thread.joinable()) thread.join(); }
    void run(bool resumed) {
        quit = false;
        thread = std::thread([this, resumed] {
            bool first = true;
            try {
                while (!quit) {
                    RokaeSample sample;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        sample.position = q;
                        const double held_since = live_hold_since[side];
                        sample.received_time = freeze_feedback_on_hold && held_since > 0 ? held_since : now();
                    }
                    sample.tcp[0] = sample.tcp[5] = sample.tcp[10] = sample.tcp[15] = 1;
                    // Inject only at the first resumed callback, not into the
                    // running loop before the lifecycle's settling check.
                    if (resumed && first && moving_on_resume) sample.velocity[0] = .03;
                    if (resumed && first && invalid_on_resume)
                        sample.velocity[0] = std::numeric_limits<double>::quiet_NaN();
                    const auto target = callback(sample);
                    if (resumed && first) {
                        check(target == sample.position, "First resumed command replayed the pre-switch target");
                        ++first_holds;
                    }
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        last_target = target;
                        if (follow_target) q = target;
                        else if (target == q && live_hold_since[side] == 0)
                            live_hold_since[side] = now();
                    }
                    first = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } catch (...) { failed = true; }
        });
    }
};
RokaeArm::RokaeArm(const std::string& robot, const std::string&, const std::array<double, 16>&,
                   const std::array<double, 7>&) : impl_(std::make_unique<Impl>()) {
    impl_->side = robot == "192.0.2.2" ? 1 : 0;
}
RokaeArm::~RokaeArm() { impl_->stop(); }
std::array<double, 7> RokaeArm::position() const {
    std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->q;
}
std::string RokaeArm::diagnostics() const { return "test SDK"; }
RokaeTiming RokaeArm::timing() const { return {}; }
void RokaeArm::beginTargetTrace(const std::array<double, 7>&) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    // Compliant physical displacement while the old command remains unchanged.
    for (auto& q : impl_->q) q += .02;
    impl_->follow_target = false;
    live_hold_since[impl_->side] = 0;
}
void RokaeArm::pollTargetTrace() {}
bool RokaeArm::motionFailed() const { return impl_->failed; }
void RokaeArm::prepare(const std::function<void()>& cancelled) { cancelled(); }
void RokaeArm::start(std::function<std::array<double, 7>(const RokaeSample&)> callback,
                     std::function<void(const std::array<double, 7>&)> initialize,
                     const std::function<void()>& cancelled) {
    cancelled();
    impl_->callback = std::move(callback); impl_->initialize = std::move(initialize);
    impl_->initialize(position()); impl_->run(false);
}
void RokaeArm::pauseImpedance(const std::function<void()>& cancelled) {
    cancelled();
    impl_->stop();
    check(impl_->last_target == impl_->q, "Paused before sending the measured hold to SDK");
    const double held_since = live_hold_since[impl_->side];
    check(held_since > 0 && now() - held_since >= expected_hold_duration,
          "Paused before configured duration of live measured-position hold");
    ++pause_calls;
    for (auto& q : impl_->q) q += .01; // Drift before the lifecycle reads jointPos.
}
void RokaeArm::setPausedStiffness(const std::array<double, 7>&) {
    for (auto& q : impl_->q) q += .02; // Drift while the stiffness RPC is in flight.
}
void RokaeArm::resumeImpedance(const std::function<void()>& guard) {
    guard();
    impl_->initialize(position());
    for (auto& q : impl_->q) q += .003; // Drift between startMove and the first callback.
    impl_->follow_target = true;
    impl_->run(true);
}
void RokaeArm::stop() { impl_->stop(); }
void RokaeArm::emergencyStop() {
    ++emergency_calls[impl_->side];
    if (require_parallel_emergency && impl_->side == 0) {
        const auto deadline = now() + .5;
        while (!emergency_calls[1] && now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(emergency_calls[1] > 0, "Right power-off waited for left SDK RPC");
    }
    impl_->stop();
    if (impl_->side == 0 && fail_emergency_left.exchange(false))
        throw std::runtime_error("Injected left power-off confirmation failure");
}
}
int main(int argc, char** argv) try {
    using namespace aviator;
    check(argc == 2, "Expected URDF path");
    RokaeConfig config;
    config.left_ip = "192.0.2.1"; config.right_ip = "192.0.2.2";
    config.left_local_ip = config.right_local_ip = "192.0.2.3";
    auto device = makeRokaeDataLink(argv[1], config, GraspGeometry{});
    device->enable(Side::Left); device->enable(Side::Right);
    auto stiffness = config.joint_stiffness;
    for (int pass = 1; pass <= 2; ++pass) {
        stiffness[6] = pass == 1 ? .1 : 50;
        device->setJointStiffness(stiffness, [] {}, true);
        const auto held = device->jointTargets();
        for (double q : held) check(std::abs(q - .053 * pass) < 1e-12, "Hold missed restart drift");
        check(first_holds == 2 * pass, "Both arms must confirm their first measured hold");
    }
    moving_on_resume = true;
    device->setJointStiffness(stiffness, [] {}, true);
    check(device->isEnabled(Side::Left) && device->isEnabled(Side::Right),
          "Finite speed on restart must not reject the measured hold");
    check(first_holds == 6, "Moving restart did not synchronize both first targets");
    for (double q : device->jointTargets())
        check(std::abs(q - .159) < 1e-12, "Moving restart replayed a stale hold");

    invalid_on_resume = true;
    bool rejected = false;
    try { device->setJointStiffness(stiffness, [] {}, true); }
    catch (const std::exception&) { rejected = true; }
    check(rejected && !device->isEnabled(Side::Left) && !device->isEnabled(Side::Right),
          "Invalid feedback on restart must still disable both arms");
    check(first_holds == 6, "Invalid feedback emitted a position command");
    check(pause_calls == 8, "Expected four complete pause stages");
    device.reset();
    invalid_on_resume = false;
    moving_on_resume = false;
    // Cancellation during the 500 ms dwell must stop both arms without changing stiffness.
    device = makeRokaeDataLink(argv[1], config, GraspGeometry{});
    device->enable(Side::Left); device->enable(Side::Right);
    bool cancelled = false;
    try {
        device->setJointStiffness(stiffness, [] {
            const double began = std::max(live_hold_since[0].load(), live_hold_since[1].load());
            if (live_hold_since[0] > 0 && live_hold_since[1] > 0 && now() - began > .05)
                throw MotionCancelled();
        }, true);
    } catch (const MotionCancelled&) { cancelled = true; }
    check(cancelled && pause_calls == 8 && !device->isEnabled(Side::Left) && !device->isEnabled(Side::Right),
          "Cancelled dwell paused/restarted impedance or left an arm running");
    device.reset();
    // Feedback that stops advancing during the dwell must not qualify as a hold.
    device = makeRokaeDataLink(argv[1], config, GraspGeometry{});
    device->enable(Side::Left); device->enable(Side::Right);
    freeze_feedback_on_hold = true;
    bool stale = false;
    try { device->setJointStiffness(stiffness, [] {}, true); }
    catch (const std::exception& e) {
        stale = std::string(e.what()).find("Stale feedback during impedance pre-pause hold") != std::string::npos;
    }
    check(stale && pause_calls == 8 && !device->isEnabled(Side::Left) && !device->isEnabled(Side::Right),
          "Stale dwell feedback allowed an impedance change");
    device.reset();
    freeze_feedback_on_hold = false;
    config.impedance_hold_duration = expected_hold_duration = .7;
    device = makeRokaeDataLink(argv[1], config, GraspGeometry{});
    device->enable(Side::Left); device->enable(Side::Right);
    device->setJointStiffness(stiffness, [] {}, true);
    check(pause_calls == 10, "Configured dwell did not complete both pause stages");
    emergency_calls[0] = emergency_calls[1] = 0;
    require_parallel_emergency = fail_emergency_left = true;
    bool power_off_failed = false;
    try { device->emergencyDisable(); } catch (...) { power_off_failed = true; }
    check(power_off_failed && emergency_calls[0] == 1 && emergency_calls[1] == 1,
          "Emergency did not attempt both arms or hid failure");
    check(device->isEnabled(Side::Left) && !device->isEnabled(Side::Right),
          "Failed left confirmation was fabricated or right was left enabled");
    device->emergencyDisable();
    check(!device->isEnabled(Side::Left) && !device->isEnabled(Side::Right), "Emergency retry did not disable both arms");
    device.reset();
    for (double invalid : {0., -1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        config.impedance_hold_duration = invalid;
        bool invalid_duration = false;
        try { device = makeRokaeDataLink(argv[1], config, GraspGeometry{}); }
        catch (const std::exception& e) {
            invalid_duration = std::string(e.what()).find("impedance_hold_duration") != std::string::npos;
        }
        check(invalid_duration, "Invalid hold duration was accepted");
    }
    std::cout << "PASS default/configured live hold before pause; duration validation; drift synchronization; moving resume; invalid feedback; dwell cancellation/staleness\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }

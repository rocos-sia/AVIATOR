#include "ServoBufferConfig.hpp"
#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/Kinematics.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using namespace aviator;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {
void check(bool value, const std::string& reason) {
    if (!value) throw std::runtime_error(reason);
}

template<class Predicate>
void waitFor(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(1ms);
    check(predicate(), reason);
}

// These tests exercise scheduling, with a stationary target and no device IO.
struct StationaryKinematics final : Kinematics {
    bool solveIk(Side, const std::array<double, 7>& seed, const pinocchio::SE3&,
                 std::array<double, 7>& out) override {
        out = seed;
        return true;
    }
    bool solveFk(Side, const std::array<double, 7>&, pinocchio::SE3& out) override {
        out = pinocchio::SE3::Identity();
        return true;
    }
    bool jointDerivatives(Side, const std::array<double, 7>&,
                          const Eigen::Matrix<double, 6, 1>&,
                          const Eigen::Matrix<double, 6, 1>&,
                          std::array<double, 7>& velocity,
                          std::array<double, 7>& acceleration) override {
        velocity.fill(0);
        acceleration.fill(0);
        return true;
    }
    double jointLower(Side, int) const override { return -10; }
    double jointUpper(Side, int) const override { return 10; }
};

struct RecordingLink final : DataLink {
    RecordingLink() { q[1] = q[8] = M_PI / 2; }
    std::array<double, 14> q{};
    bool enabled[2]{};
    mutable std::atomic<unsigned> reads{0}, polls{0};
    std::atomic<size_t> ahead{0}, initial_frames{0}, appended_frames{0};
    std::atomic<unsigned> begins{0}, appends{0}, finishes{0}, forced_stops{0};
    std::atomic<bool> drain{false}, finishing{false};
    std::mutex finish_mutex;
    std::condition_variable finish_cv;
    bool allow_finish = false;

    double getJointPosition(Side side, int axis) const override {
        ++reads;
        return q[static_cast<int>(side) * 7 + axis];
    }
    double getJointVelocity(Side, int) const override { return 0; }
    void setJointPositions(const std::array<double, 14>& value) override { q = value; }
    std::array<double, 14> jointTargets() const override { return q; }
    double jointVelLimit(Side, int) const override { return 10; }
    bool isEnabled(Side side) const override { return enabled[static_cast<int>(side)]; }
    void enable(Side side) override { enabled[static_cast<int>(side)] = true; }
    void disable(Side side) override { enabled[static_cast<int>(side)] = false; }
    GraspState graspState() const override {
        GraspState state;
        state.locked = 3;
        state.ready = 1;
        return state;
    }
    uint64_t sendGraspCommand(GraspCommand) override { return 0; }
    void waitTick() override {}
    double time() const override { return 0; }

    void beginStream(const std::vector<JointFrame>& frames) override {
        initial_frames = frames.size();
        ahead = frames.size() - 1;
        ++begins;
    }
    void appendStream(const std::vector<JointFrame>& frames) override {
        appended_frames = frames.size();
        ahead += frames.size() - 1;
        ++appends;
    }
    size_t streamAhead() const override {
        ++polls;
        return drain ? 0 : ahead.load();
    }
    void finishStream() override {
        finishing = true;
        std::unique_lock<std::mutex> lock(finish_mutex);
        finish_cv.wait(lock, [&] { return allow_finish; });
        ahead = 0;
        ++finishes;
    }
    void stopTrajectory() override { ++forced_stops; }
    void release() {
        drain = true;
        {
            std::lock_guard<std::mutex> lock(finish_mutex);
            allow_finish = true;
        }
        finish_cv.notify_all();
    }
};

struct Fixture {
    fs::path dir = fs::temp_directory_path() / ("servo-buffer-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    Fixture() {
        fs::create_directory(dir);
        std::ofstream(dir / "grasp.yaml") << R"(
left: &identity {position: [0, 0, 0], quaternion: [1, 0, 0, 0]}
right: *identity
tool: *identity
wheel_origin: *identity
approach_distance: 0.05
)";
        std::ofstream(dir / "posture.yaml") << R"(
left_home_deg: &pose [0, 90, 0, 0, 0, 0, 0]
right_home_deg: *pose
left_approach_seed_deg: *pose
right_approach_seed_deg: *pose
joint2_limits_deg: [80, 100]
joint2_planning_margin_deg: 0.5
)";
    }
    ~Fixture() { fs::remove_all(dir); }
    YAML::Node config() const {
        return YAML::Load(R"(
collision_check_enabled: false
grasp: grasp.yaml
posture: posture.yaml
servo_period: 0.02
servo_timeout: 10
)");
    }
    std::string write(const YAML::Node& config) const {
        const auto path = dir / "robot.yaml";
        std::ofstream(path) << config;
        return path.string();
    }
};

void configuration(const Fixture& fixture) {
    const auto defaults = ServoBufferConfig::load(YAML::Load("{}"), .02);
    check(defaults.prefill_ms == 80 && defaults.lookahead_ms == 80 &&
              defaults.effective_prefill_ms == 80,
          "Missing buffer settings changed legacy 80 ms behavior");
    for (const auto& entry : std::vector<std::array<size_t, 4>>{
             {1, 1, 20, 20}, {20, 20, 20, 20}, {40, 40, 20, 40},
             {59, 59, 20, 60}, {60, 40, 20, 60},
             {60, 60, 20, 60}, {80, 60, 20, 80}, {81, 60, 20, 100},
             {80, 60, 30, 90}, {250, 235, 10, 250}, {200, 195, 50, 200}}) {
        auto config = fixture.config();
        config["servo_prefill_ms"] = entry[0];
        config["servo_lookahead_ms"] = entry[1];
        const auto buffer = ServoBufferConfig::load(config, entry[2] / 1000.);
        check(buffer.prefill_ms == entry[0] && buffer.lookahead_ms == entry[1] &&
                  buffer.effective_prefill_ms == entry[3],
              "Configured duration/block rounding was not preserved");
    }

    // Reject bad values before the executor reads joint feedback or starts a worker.
    for (const auto* key : {"servo_prefill_ms", "servo_lookahead_ms"}) {
        for (const auto* value : {"0", "-1", "0.5", "60.5", "251", ".nan", ".inf",
                                 "word", "[]", "{}"}) {
            auto config = fixture.config();
            config[key] = YAML::Load(value);
            auto link = std::make_unique<RecordingLink>();
            auto* device = link.get();
            Aviator core(std::move(link), std::make_unique<StationaryKinematics>(), nullptr,
                         fixture.write(config));
            bool rejected = false;
            try { core.init(); }
            catch (const std::exception&) { rejected = true; }
            check(rejected && device->reads == 0 && core.GetState() == "UNINITIALIZED",
                  std::string("Initialization accepted invalid ") + key + "=" + value);
        }
    }
    for (const auto& entry : std::vector<std::pair<const char*, size_t>>{
             {"servo_prefill_ms", 241}, {"servo_lookahead_ms", 226}}) {
        auto config = fixture.config();
        config[entry.first] = entry.second;
        bool rejected = false;
        try { ServoBufferConfig::load(config, .02); }
        catch (const std::exception&) { rejected = true; }
        check(rejected, "Accepted queue overflow after block rounding/history allowance");
    }
    for (double period : {0., .009, .0205, .051}) {
        bool rejected = false;
        try { ServoBufferConfig::load(YAML::Load("{}"), period); }
        catch (const std::exception&) { rejected = true; }
        check(rejected, "Invalid block duration accepted");
    }
}

void streaming(const Fixture& fixture, bool defaults, size_t prefill, size_t lookahead,
               size_t effective_prefill) {
    auto config = fixture.config();
    if (!defaults) {
        config["servo_prefill_ms"] = prefill;
        config["servo_lookahead_ms"] = lookahead;
    }
    auto link = std::make_unique<RecordingLink>();
    auto* device = link.get();
    Aviator core(std::move(link), std::make_unique<StationaryKinematics>(), nullptr,
                 fixture.write(config));
    // Unblock worker shutdown even when an assertion fails.
    struct Cleanup {
        Aviator& core;
        RecordingLink& device;
        ~Cleanup() { core.stop(); device.release(); }
    } cleanup{core, *device};
    core.init();
    core.enable();
    core.servoWheel(0, 0);
    waitFor([&] { return device->begins == 1; }, "Servo never began its stream");
    check(device->initial_frames == effective_prefill + 1,
          "Configured prefill did not determine beginStream frame count");

    device->ahead = lookahead;
    const auto polls = device->polls.load();
    waitFor([&] { return device->polls >= polls + 3; }, "No lookahead checks");
    check(device->appends == 0, "Servo appended at or above the lookahead threshold");
    device->ahead = lookahead - 1;
    waitFor([&] { return device->appends >= 1; }, "Servo did not refill below its lookahead threshold");
    const auto after_append = device->polls.load();
    waitFor([&] { return device->polls >= after_append + 3; }, "Servo did not wait after refilling");
    check(device->appends == 1 && device->appended_frames == 21,
          "Servo did not append exactly one 20 ms block");

    core.stop();
    device->drain = true;
    waitFor([&] { return device->finishing.load(); }, "Stop did not finish the stream");
    check(core.GetState() == "SERVO" && device->finishes == 0,
          "Core left SERVO before the submitted buffer drained");
    device->release();
    waitFor([&] { return core.GetState() != "SERVO"; }, "Servo did not exit after buffer drain");
    check(core.GetState() == "LOCKED" && device->finishes == 1 &&
              device->forced_stops == 0 && core.GetStatus().motion_error.empty(),
          "Normal buffered stop did not preserve locked, fault-free hold");
}
} // namespace

int main() try {
    Fixture fixture;
    configuration(fixture);
    streaming(fixture, true, 80, 80, 80);
    streaming(fixture, false, 1, 1, 20);
    streaming(fixture, false, 20, 20, 20);
    streaming(fixture, false, 40, 40, 40);
    streaming(fixture, false, 59, 59, 60);
    streaming(fixture, false, 60, 40, 60);
    streaming(fixture, false, 80, 60, 80);
    streaming(fixture, false, 81, 60, 100);
    std::cout << "PASS Servo buffer configuration, prefill, refill threshold and stop/drain\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}

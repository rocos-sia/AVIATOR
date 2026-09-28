#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace aviator;
namespace {
// 唯一的替身：让右臂使能失败，检查左臂是否回滚。
struct FailingEnableLink final : DataLink {
    bool enabled[2]{};
    bool disabled[2]{};
    GraspState feedback;
    mutable int diagnostic_calls = 0;
    double getJointPosition(Side, int axis) const override { return axis == 1 ? 1.5707963267948966 : 0; }
    double getJointVelocity(Side, int) const override { return 0; }
    void setJointPositions(const std::array<double, 14> &) override {}
    std::array<double, 14> jointTargets() const override { return {}; }
    double jointVelLimit(Side, int) const override { return 2; }
    bool isEnabled(Side side) const override { return enabled[int(side)]; }
    void enable(Side side) override {
        enabled[int(side)] = true;
        if (side == Side::Right) throw std::runtime_error("Injected right-arm power failure");
    }
    void disable(Side side) override { enabled[int(side)] = false; disabled[int(side)] = true; }
    GraspState graspState() const override { return feedback; }
    std::string diagnostics() const override {
        ++diagnostic_calls;
        return "right robot=192.0.2.2 feedback_age_ms=125 callbacks=42";
    }
    uint64_t sendGraspCommand(GraspCommand) override { return 0; }
    void waitTick() override {}
    double time() const override { return 0; }
};
void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
}
int main() {
    std::filesystem::path invalid;
    try {
        auto link = std::make_unique<FailingEnableLink>();
        auto *io = link.get();
        Aviator robot(std::move(link), nullptr, nullptr,
                      AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        bool rejected = false;
        try { robot.Enable(); } catch (const std::exception &) { rejected = true; }
        check(rejected, "second-arm power failure not propagated");
        check(io->disabled[0] && io->disabled[1] && !io->enabled[0] && !io->enabled[1],
              "both arms were not rolled back");
        check(io->diagnostic_calls == 0, "Diagnostics ran on a normal feedback path");
        io->feedback.fault = 1;
        std::string error;
        try { robot.ApproachHandles(); } catch (const std::exception &e) { error = e.what(); }
        check(error.find("Backend grasp fault") != std::string::npos &&
              error.find("right robot=192.0.2.2 feedback_age_ms=125") != std::string::npos,
              "Backend fault lost the per-arm diagnostics");
        io->feedback.heartbeat = -1;
        error.clear();
        try { robot.GetStatus(); } catch (const std::exception &e) { error = e.what(); }
        check(error.find("Backend feedback timeout") != std::string::npos &&
              error.find("callbacks=42") != std::string::npos,
              "Feedback timeout lost backend diagnostics");
        auto config = YAML::LoadFile(AVIATOR_CONFIG_DIR "/aviator.yaml");
        for (auto key : {"grasp", "posture"})
            config[key] = (std::filesystem::path(AVIATOR_CONFIG_DIR) / config[key].as<std::string>()).string();
        config["planning_period"] = 0;
        invalid = std::filesystem::temp_directory_path() / ("aviator-invalid-" + std::to_string(getpid()) + ".yaml");
        { std::ofstream file(invalid); file << config; }
        Aviator bad(std::make_unique<FailingEnableLink>(), nullptr, nullptr, invalid.string());
        rejected = false;
        try { bad.Init(); } catch (const std::exception &e) {
            rejected = std::string(e.what()).find("planning_period") != std::string::npos;
        }
        check(rejected, "zero planning period was not rejected");
        std::filesystem::remove(invalid);
        std::cout << "Dual-arm enable rollback, fault diagnostics and invalid planning configuration passed\n";
    } catch (const std::exception &e) {
        if (!invalid.empty()) std::filesystem::remove(invalid);
        std::cerr << e.what() << '\n'; return 1;
    }
}

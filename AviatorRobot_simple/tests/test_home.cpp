// 回 home 顺序测试：双臂到位后才能求预接近逆解；任一臂未到位必须超时停止。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace aviator;

static void check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

struct HomingLink final : DataLink {
    std::array<double, 14> home{}, actual{}, target{}, previous{};
    bool enabled[2]{};
    int blocked_side = -1;
    double now = 0, home_time = -1, peak_speed = 0;
    bool intermediate = false;
    double getJointPosition(Side s, int j) const override { return actual[int(s)*7+j]; }
    double getJointVelocity(Side s, int) const override {
        return blocked_side == 2 && s == Side::Right ? .03 : 0;
    }
    double jointVelLimit(Side, int) const override { return 0.015; }
    bool isEnabled(Side s) const override { return enabled[int(s)]; }
    void enable(Side s) override { enabled[int(s)] = true; }
    void disable(Side s) override { enabled[int(s)] = false; }
    GraspState graspState() const override {
        GraspState state;
        state.heartbeat = now;
        return state;
    }
    uint64_t sendGraspCommand(GraspCommand) override { return 0; }
    double time() const override { return now; }
    void waitTick() override {
        now += .001;
        for (int i = 0; i < 14; ++i)
            if (i / 7 != blocked_side) actual[i] = target[i];
    }
    void setJointPositions(const std::array<double, 14> &q) override {
        if (now > 0) {
            for (int i = 0; i < 14; ++i)
                peak_speed = std::max(peak_speed, std::abs(q[i]-previous[i])/.001);
            intermediate = intermediate || (q[0] > home[0]+.01 && q[0] < home[0]+.04);
        }
        bool at_home = true;
        for (int i = 0; i < 14; ++i) at_home = at_home && std::abs(q[i]-home[i]) < 1e-10;
        if (at_home && home_time < 0) home_time = now;
        previous = target = q;
    }
};

// 在第一次预接近 IK 时检查回 home 已完成，然后主动结束本专项测试。
struct ApproachProbe final : Kinematics {
    HomingLink *io;
    bool called = false;
    explicit ApproachProbe(HomingLink *link) : io(link) {}
    double jointLower(Side, int) const override { return -4; }
    double jointUpper(Side, int) const override { return 4; }
    bool solveFk(Side, const std::array<double, 7> &, pinocchio::SE3 &) override { return false; }
    bool solveIk(Side, const std::array<double, 7> &, const pinocchio::SE3 &,
                 std::array<double, 7> &) override {
        called = true;
        for (int i = 0; i < 14; ++i)
            check(std::abs(io->actual[i]-io->home[i]) < .01, "IK started before both arms reached home");
        check(io->home_time >= 0 && io->now-io->home_time >= .15, "Home settling was skipped");
        throw std::runtime_error("Home verified before approach IK");
    }
};

int main() {
    try {
        const auto posture = YAML::LoadFile(AVIATOR_CONFIG_DIR "/posture.json");
        // -1：正常跟踪；0/1：对应臂位置不跟随；2：右臂到位但仍在运动。
        for (int blocked_side : {-1, 0, 1, 2}) {
            auto link = std::make_unique<HomingLink>();
            auto *io = link.get();
            for (int side = 0; side < 2; ++side) {
                const auto q = posture[side == 0 ? "left_home_deg" : "right_home_deg"].as<std::vector<double>>();
                for (int j = 0; j < 7; ++j) io->home[side*7+j] = q[j]*M_PI/180.;
            }
            io->actual = io->home;
            io->actual[0] += .05;
            io->actual[7] += .05;
            io->previous = io->target = io->actual;
            io->blocked_side = blocked_side;
            auto probe = std::make_unique<ApproachProbe>(io);
            auto *ik = probe.get();
            Aviator robot(std::move(link), std::move(probe), nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
            robot.Init();
            robot.Enable();
            check(io->actual[0] > io->home[0]+.04, "Enable moved the robot");
            std::string error;
            try { robot.ApproachHandles(); }
            catch (const std::exception &e) { error = e.what(); }
            std::cout << "blocked_side=" << blocked_side << ": " << error << '\n';
            if (blocked_side < 0) {
                check(error == "Home verified before approach IK" && ik->called, "Home sequence failed");
                check(io->intermediate && io->peak_speed <= .015001, "Home trajectory jumped or exceeded speed limit");
                check(io->home_time > 6, "Backend velocity limit did not extend home duration");
            } else {
                check(error == "Home arrival timeout" && !ik->called, "Unsettled arm did not block approach");
                check(robot.GetState() == "FAULT", "Home failure did not enter FAULT");
            }
            robot.Disable();
        }
        std::cout << "Home trajectory, dual-arm arrival and timeout checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

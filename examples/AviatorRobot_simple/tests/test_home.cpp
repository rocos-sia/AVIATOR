// home 指令完成后仅确认停稳；位置有偏差仍进入预接近，持续运动则超时。
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
    std::array<double, 14> home{}, actual{}, target{}, previous{}, velocity{}, acceleration{};
    bool enabled[2]{};
    int blocked_side = -1;
    double now = 0, home_time = -1, peak_speed = 0, peak_acceleration = 0, peak_jerk = 0;
    double velocity_limit = .015;
    double last_motion[2]{};
    bool intermediate = false;
    double getJointPosition(Side s, int j) const override { return actual[int(s)*7+j]; }
    double getJointVelocity(Side s, int) const override {
        return blocked_side == 2 && s == Side::Right ? .03 : 0;
    }
    double jointVelLimit(Side, int) const override { return velocity_limit; }
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
    std::array<double, 14> jointTargets() const override { return target; }
    void waitTick() override {
        now += .001;
        for (int i = 0; i < 14; ++i)
            if (i / 7 != blocked_side) actual[i] = target[i];
    }
    void setJointPositions(const std::array<double, 14> &q) override {
        if (now > 0) {
            for (int i = 0; i < 14; ++i) {
                const double v = (q[i] - previous[i]) / .001;
                const double a = (v - velocity[i]) / .001;
                if (std::abs(v) > 1e-7) last_motion[i / 7] = now;
                peak_speed = std::max(peak_speed, std::abs(v));
                peak_acceleration = std::max(peak_acceleration, std::abs(a));
                peak_jerk = std::max(peak_jerk, std::abs(a - acceleration[i]) / .001);
                velocity[i] = v;
                acceleration[i] = a;
            }
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
            check(std::abs(io->target[i]-io->home[i]) < 1e-10, "IK started before home commands finished");
        check(io->home_time >= 0 && io->now-io->home_time >= .15, "Home settling was skipped");
        throw std::runtime_error("Home verified before approach IK");
    }
};

int main() {
    try {
        const auto posture = YAML::LoadFile(AVIATOR_CONFIG_DIR "/posture.json");
        // -1：正常跟踪；0/1：对应臂位置不跟随；2：右臂到位但仍在运动。
        for (int blocked_side : {-1, 0, 1, 2, 3, 4}) {
            auto link = std::make_unique<HomingLink>();
            auto *io = link.get();
            for (int side = 0; side < 2; ++side) {
                const auto q = posture[side == 0 ? "left_home_deg" : "right_home_deg"].as<std::vector<double>>();
                for (int j = 0; j < 7; ++j) io->home[side*7+j] = q[j]*M_PI/180.;
            }
            io->actual = io->home;
            // 3: 零位移；4: YAML 速度限制；0/1 的位置差 > 旧跟踪阈值，仍必须发完 home。
            const double offset = blocked_side == 3 ? 0 : (blocked_side == 0 || blocked_side == 1 || blocked_side == 4 ? .2 : .05);
            io->actual[0] += offset;
            io->actual[7] += offset / 2;
            if (blocked_side == 4) io->velocity_limit = .3;
            io->previous = io->target = io->actual;
            io->blocked_side = blocked_side >= 3 ? -1 : blocked_side;
            auto probe = std::make_unique<ApproachProbe>(io);
            auto *ik = probe.get();
            Aviator robot(std::move(link), std::move(probe), nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
            robot.Init();
            robot.Enable();
            check(io->actual == io->target, "Enable moved the robot");
            if (blocked_side == 3) io->home_time = 0;
            std::string error;
            try { robot.ApproachHandles(); }
            catch (const std::exception &e) { error = e.what(); }
            std::cout << "blocked_side=" << blocked_side << ": " << error << '\n';
            if (blocked_side != 2) {
                check(error.find("Home verified before approach IK") != std::string::npos && ik->called, "Home sequence failed");
                const double limit = std::min(.1, io->velocity_limit);
                check(io->peak_speed <= limit + 1e-6, "Home exceeded YAML/backend speed limit");
                check(io->peak_acceleration <= .2 + 1e-5, "Home acceleration limit exceeded");
                check(io->peak_jerk <= 1.0 + 1e-4, "Home jerk limit exceeded");
                check(std::abs(io->last_motion[0] - io->last_motion[1]) <= .003, "Arm trajectories did not finish together");
                if (blocked_side == 4) check(io->peak_speed > .095, "Long home did not exercise YAML speed cap");
                if (blocked_side == 3) check(io->peak_speed == 0, "Zero-distance home moved");
                else check(io->intermediate, "Home trajectory jumped");
                if (blocked_side < 0) check(io->home_time > 3.3, "Backend limit did not extend home duration");
                std::cout << "home time=" << io->home_time << " peak v/a/j=" << io->peak_speed
                          << '/' << io->peak_acceleration << '/' << io->peak_jerk << '\n';
            } else {
                check(error.find("Home settling timeout") != std::string::npos && !ik->called, "Unsettled arm did not block approach");
                check(robot.GetState() == "FAULT", "Home failure did not enter FAULT");
            }
            robot.Disable();
        }
        std::cout << "Home trajectory, position-offset tolerance and settling timeout checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

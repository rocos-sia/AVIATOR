// 注入调度延迟、稳定跟踪偏差和反馈故障，检查实际发布的指令，而非规划器内部数组。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include "../src/CommandContinuity.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace aviator;
using Joints = std::array<double, 14>;

static void check(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

struct TestLink final : DataLink {
    Joints actual{}, target{}, delta{};
    bool enabled[2]{}, stopping = false, fault = false, stale = false, fail_left_stop = false;
    double now = 0, peak_step = 0, peak_stop_acceleration = 0;
    int ticks = 0, commands = 0, stop_commands = 0, commands_after_fault = 0;
    GraspState state;
    std::function<void()> on_tick;

    TestLink() {
        const auto posture = YAML::LoadFile(AVIATOR_CONFIG_DIR "/posture.json");
        for (int side = 0; side < 2; ++side) {
            const auto q = posture[side ? "right_home_deg" : "left_home_deg"].as<std::vector<double>>();
            for (int j = 0; j < 7; ++j) target[side * 7 + j] = q[j] * M_PI / 180;
            target[side * 7] += .03;
        }
        actual = target;
        state.ready = 1;
    }
    double getJointPosition(Side s, int j) const override { return actual[int(s) * 7 + j]; }
    double getJointVelocity(Side, int) const override { return 0; }
    Joints jointTargets() const override { return target; }
    double jointVelLimit(Side, int) const override { return .2; }
    bool isEnabled(Side s) const override { return enabled[int(s)]; }
    void enable(Side s) override { enabled[int(s)] = true; }
    void disable(Side s) override {
        enabled[int(s)] = false;
        if (s == Side::Left && fail_left_stop) throw std::runtime_error("Injected left stop failure");
    }
    void setJointPositions(const Joints &q) override {
        ++commands;
        if (fault || stale) ++commands_after_fault;
        for (int i = 0; i < 14; ++i) {
            const double step = q[i] - target[i];
            peak_step = std::max(peak_step, std::abs(step));
            if (stopping)
                peak_stop_acceleration = std::max(peak_stop_acceleration, std::abs(step - delta[i]) / 1e-6);
            delta[i] = step;
        }
        if (stopping) ++stop_commands;
        target = q;
    }
    GraspState graspState() const override {
        auto result = state;
        result.heartbeat = stale ? now - 1.0 : now;
        result.fault = fault;
        return result;
    }
    uint64_t sendGraspCommand(GraspCommand c) override {
        if (c == GraspCommand::Lock) state.locked = 3;
        if (c == GraspCommand::Unlock) state.locked = 0;
        if (c == GraspCommand::ResetFault) fault = false;
        return ++state.ack;
    }
    void setWheelReference(double angle, double displacement) override {
        state.angle = angle;
        state.displacement = displacement;
    }
    void waitTick() override {
        ++ticks;
        now += ticks % 50 == 0 ? .051 : .001; // 模拟线程漏掉 50 ms，但反馈仍有效。
        actual = target;
        actual[0] -= .005; // 关节阻抗下允许的小偏差，不应导致新轨迹或停止指令跳变。
        actual[7] -= .005;
        if (on_tick) on_tick();
    }
    double time() const override { return now; }
};

int main() {
    try {
        // SDK 最后一层检查：重复保持合法，超大增量、非有限值必须拒绝。
        std::array<double, 7> previous{}, next{}, limit{};
        limit.fill(1);
        next[0] = .001;
        check(excessiveJointStep(previous, next, limit) == -1, "Legal RT step rejected");
        next[0] = .002;
        check(excessiveJointStep(previous, next, limit) == 0, "Skipped RT sample accepted");
        next[0] = std::numeric_limits<double>::quiet_NaN();
        check(excessiveJointStep(previous, next, limit) == 0, "Nonfinite RT command accepted");

        auto link = std::make_unique<TestLink>();
        auto *io = link.get();
        Aviator robot(std::move(link), nullptr, nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.Enable();
        io->state.position_error[0] = .01;
        bool rejected = false;
        try { robot.LockHandles(); } catch (const std::exception &) { rejected = true; }
        check(rejected, "Premature lock accepted");
        io->state.position_error[0] = 0; // 动作前置条件不满足，不等同于反馈/控制故障。
        robot.ApproachHandles();
        robot.LockHandles();
        robot.MoveWheel(.02, -.002, .5);
        check(io->peak_step <= .0002 + 1e-9, "Delay or stage transition caused a command jump");
        std::cout << "Delay + stage transitions: max step=" << io->peak_step << " rad\n";

        const int stop_tick = io->ticks + 1000;
        io->on_tick = [&] {
            if (io->ticks == stop_tick) {
                io->stopping = true;
                robot.Stop();
            }
        };
        std::string error;
        try { robot.MoveWheel(.5, -.05, 1); }
        catch (const std::exception &e) { error = e.what(); }
        check(error.find("Motion stopped") != std::string::npos, "Stop was not observed");
        check(io->stop_commands > 1, "Stop did not produce a braking transition");
        check(io->peak_stop_acceleration <= 2.0 + 1e-6, "Stop acceleration exceeded configured limit");
        check(io->enabled[0] && io->enabled[1], "Normal stop disabled impedance holding");
        for (double d : io->delta) check(std::abs(d) < 1e-12, "Stop did not finish at zero command velocity");
        check(io->peak_step <= .0002 + 1e-9, "Stop snapped command to feedback");
        std::cout << "Normal stop: samples=" << io->stop_commands
                  << " peak acceleration=" << io->peak_stop_acceleration << " rad/s^2\n";

        io->stopping = false;
        robot.UnlockHandles();
        robot.ResetFault();
        robot.LockHandles();
        const int fault_tick = io->ticks + 100;
        io->fail_left_stop = true;
        io->on_tick = [&] { if (io->ticks == fault_tick) io->fault = true; };
        error.clear();
        try { robot.MoveWheel(.1, -.01, .5); }
        catch (const std::exception &e) { error = e.what(); }
        check(error.find("Backend grasp fault") != std::string::npos, "Original feedback fault lost");
        check(!io->enabled[0] && !io->enabled[1], "Fault did not attempt to stop both arms");
        check(io->commands_after_fault == 0, "Position commands continued after feedback fault");
        check(robot.GetStatus().motion_error.find("Injected left stop failure") != std::string::npos,
              "SDK stop failure was lost");
        std::cout << "Feedback fault: both stops attempted; no further position commands\n";

        // 非有限反馈与过期反馈仍必须中止；不再把有限位置偏差当作故障。
        for (bool stale_feedback : {false, true}) {
            auto failing_link = std::make_unique<TestLink>();
            auto *failed = failing_link.get();
            Aviator failing(std::move(failing_link), nullptr, nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
            failing.Init();
            failing.Enable();
            if (stale_feedback) {
                // 进入执行后注入过期反馈，避免仅测试动作前置条件。
                failed->on_tick = [&] { failed->stale = true; };
            } else {
                failed->actual[0] = std::numeric_limits<double>::quiet_NaN();
            }
            error.clear();
            try { failing.ApproachHandles(); }
            catch (const std::exception &e) { error = e.what(); }
            check(error.find(stale_feedback ? "feedback timeout" : "Nonfinite joint feedback") != std::string::npos,
                  "Missing stale/nonfinite diagnostic");
            check(!failed->enabled[0] && !failed->enabled[1], "Stale/tracking fault did not stop both arms");
            check(failed->commands == 0, "Fault was hidden by rebasing commands to feedback");
        }
        std::cout << "Stale feedback and nonfinite feedback faults remain stopped\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

// ServoWheel 专项测试：连续目标 → 停止 → 超时 → 恢复 → 碰撞故障。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
using namespace aviator;
using Clock = std::chrono::steady_clock;

static void check(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

// 正常使用真实碰撞检查器；末尾打开 inject，验证异步错误能否传给调用者。
struct FaultCollision final : CollisionChecker {
    std::unique_ptr<CollisionChecker> real = makePinocchioCollisionChecker(
        AVIATOR_MODEL_DIR "/aviator_collision.urdf", AVIATOR_MODEL_DIR "/aviator_collision.srdf", {});
    std::atomic<bool> inject{false};
    void check(const std::array<double, 14> &q, double angle, double displacement) override {
        if (inject) throw std::runtime_error("Injected collision");
        real->check(q, angle, displacement);
    }
};

int main() {
    try {
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        check(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int home = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
        check(data && home >= 0, "Missing simulation data or aviator_home");
        mj_resetDataKeyframe(model.get(), data.get(), home);
        mj_forward(model.get(), data.get());
        const int roll = mj_name2id(model.get(), mjOBJ_JOINT, "roll_input_joint");
        const int slide = mj_name2id(model.get(), mjOBJ_JOINT, "pitch_input_joint");
        check(roll >= 0 && slide >= 0, "Missing wheel joints");

        auto link = makeMuJoCoDirectDataLink(model.get(), data.get(), AVIATOR_CONFIG_DIR "/aviator_control.urdf");
        auto collision = std::make_unique<FaultCollision>();
        auto *checker = collision.get();
        Aviator robot(std::move(link), nullptr, std::move(collision), AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.SetRealTime(false);
        robot.Enable();
        robot.ApproachHandles();
        robot.LockHandles();
        robot.SetRealTime(true);

        for (const auto invalid : {std::array<double, 3>{0, 0, 0}, {0, 0, -.1}, {0, 0, 1.1},
                                   {0, 0, std::numeric_limits<double>::quiet_NaN()}, {1, 0, .5}}) {
            bool rejected = false;
            try { robot.ServoWheel(invalid[0], invalid[1], invalid[2]); }
            catch (const std::exception &) { rejected = true; }
            check(rejected, "Invalid ServoWheel input accepted");
        }

        // 直接按 20 ms 周期发布；第二个目标覆盖第一个，不排队。
        for (const auto target : {std::array<double, 3>{.02, -.002, .5}, {-.015, -.001, .5}}) {
            double longest_call = 0;
            for (int tick = 0; tick < 100; ++tick) {
                const auto start = Clock::now();
                robot.ServoWheel(target[0], target[1], target[2]);
                longest_call = std::max(longest_call, std::chrono::duration<double>(Clock::now()-start).count());
                check(robot.GetState() == "SERVO", robot.GetStatus().motion_error);
                std::this_thread::sleep_until(start + std::chrono::milliseconds(20));
            }
            const auto s = robot.GetStatus();
            std::cout << "target=" << target[0] << ',' << target[1] << " reference=" << s.angle << ','
                      << s.displacement << " max_publish_ms=" << longest_call*1000 << '\n';
            check(longest_call < .1, "ServoWheel blocked on execution");
            check(std::abs(s.angle-target[0]) < .001 && std::abs(s.displacement-target[1]) < .0002,
                  "Latest target was not reached");
            std::lock_guard<std::mutex> lock(*robot.PhysicsMutex());
            check(std::abs(data->qpos[model->jnt_qposadr[roll]]-target[0]) < .005 &&
                  std::abs(data->qpos[model->jnt_qposadr[slide]]-target[1]) < .001,
                  "Physical wheel did not follow Servo target");
        }
        bool rejected = false;
        try { robot.MoveWheel(0, 0, .5); } catch (const std::exception &) { rejected = true; }
        check(rejected, "Blocking move accepted during Servo");

        robot.Stop();
        auto until = Clock::now() + std::chrono::seconds(2);
        while (robot.GetState() == "SERVO" && Clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(robot.GetState() == "LOCKED", "Stop did not hold locked");
        const double held = robot.GetStatus().angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(robot.GetStatus().angle == held, "Old target continued after Stop");

        robot.ServoWheel(.2, -.01, .5); // 故意不再更新，检查 250 ms 超时。
        until = Clock::now() + std::chrono::seconds(2);
        while (robot.GetState() == "SERVO" && Clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(robot.GetState() == "LOCKED", "Timeout did not hold locked");
        check(robot.GetStatus().motion_error.find("timeout") != std::string::npos, "Missing timeout diagnostic");
        check(robot.GetStatus().angle < .2, "Stale target completed");

        for (int tick = 0; tick < 100; ++tick) {
            robot.ServoWheel(0, -.001, .5); // 超时后发布新目标可恢复。
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(robot.GetState() == "SERVO" && std::abs(robot.GetStatus().angle) < .001, "Servo did not resume");
        robot.Stop();
        until = Clock::now() + std::chrono::seconds(2);
        while (robot.GetState() == "SERVO" && Clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(robot.GetState() == "LOCKED", "Stop after resume failed");

        checker->inject = true;
        robot.ServoWheel(.1, -.001, .5);
        until = Clock::now() + std::chrono::seconds(2);
        while (robot.GetState() == "SERVO" && Clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(robot.GetState() == "FAULT" && robot.GetStatus().motion_error == "Injected collision",
              "Asynchronous collision was not reported");
        rejected = false;
        try { robot.ServoWheel(0, 0, .5); } catch (const std::exception &) { rejected = true; }
        check(rejected, "Servo accepted while faulted");
        rejected = false;
        try { robot.MoveWheel(0, 0, .5); } catch (const std::exception &) { rejected = true; }
        check(rejected, "MoveWheel accepted while faulted");

        checker->inject = false;
        robot.UnlockHandles();
        robot.ResetFault();
        robot.Disable();
        std::cout << "Servo targets, tracking, Stop, timeout and fault recovery passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

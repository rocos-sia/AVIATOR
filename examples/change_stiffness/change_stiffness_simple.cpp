// 原地关节阻抗：运行两秒后退出并重进 RT，设置 J7 刚度后重新开始保持。
#include <rokae/robot.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace rokae;
using namespace std::chrono_literals;

namespace {
std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void requestStop(int) { interrupted.store(true); }
void check(const std::error_code &ec, const char *action = "SDK call") {
    if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message());
}
void waitUntil(std::chrono::steady_clock::time_point deadline) {
    while (!interrupted.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(10ms);
}
} // namespace

int main(int argc, char **argv) {
    const bool help = argc == 2 && std::string(argv[1]) == "--help";
    if (help || argc != 3) {
        std::cout << "Usage: change_stiffness_simple <robot_ip> <local_ip>\n"
                     "Hold for 2 seconds; leave and re-enter RT, set J7 stiffness, then hold for 8 seconds.\n";
        return help ? 0 : 2;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    xMateErProRobot robot;
    std::shared_ptr<RtMotionControlCobot<7>> rtCon;
    std::array<double, 7> jntPos{};
    std::error_code ec;
    bool connected = false, powered = false, receiving = false, moving = false, looping = false;
    int result = 0;
    try {
        robot.connectToRobot(argv[1], argv[2]);
        connected = true;
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        check(ec);
        robot.setOperateMode(OperateMode::automatic, ec);
        check(ec);
        if (interrupted.load()) throw std::runtime_error("Interrupted before power on");
        powered = true;
        robot.setPowerState(true, ec);
        check(ec);
        rtCon = robot.getRtMotionController().lock();
        if (!rtCon) throw std::runtime_error("No realtime controller");
        receiving = true;
        robot.startReceiveRobotState(1ms, {RtSupportedFields::jointPos_m});
        rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
        check(ec);

        // 只读取一次当前位置，后续始终发送该目标，不执行 MoveAbsJ 或正弦运动。
        jntPos = robot.jointPos(ec);
        check(ec);
        for (double q : jntPos)
            if (!std::isfinite(q)) throw std::runtime_error("Invalid joint position");
        std::function<JointPosition()> callback = [&jntPos] {
            JointPosition cmd(7);
            for (unsigned i = 0; i < jntPos.size(); ++i) cmd.joints[i] = jntPos[i];
            if (interrupted.load()) cmd.setFinished();
            return cmd;
        };
        rtCon->setControlLoop(callback);
        if (interrupted.load()) throw std::runtime_error("Interrupted before startMove");
        moving = true;
        rtCon->startMove(RtControllerMode::jointImpedance);
        const auto start = std::chrono::steady_clock::now();
        looping = true;
        rtCon->startLoop(false); // 非阻塞：SDK 持续发送位置，主线程等待模式切换。

        waitUntil(start + 2s);
        if (!interrupted.load()) {
            rtCon->stopLoop();
            looping = false;
            rtCon->stopMove();
            moving = false;
            robot.stopReceiveRobotState();
            receiving = false;

            rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 0.1}, ec);
            check(ec, "setJointImpedance(after re-entering RT)");
            std::cout << "J7 stiffness changed: 50 -> 0.1 Nm/rad" << std::endl;

            receiving = true;
            robot.startReceiveRobotState(1ms, {RtSupportedFields::jointPos_m});
            // rtCon->setControlLoop(callback); // 继续使用启动时读取的固定目标位置。
            if (interrupted.load()) throw std::runtime_error("Interrupted before restarting motion");
            moving = true;
            rtCon->startMove(RtControllerMode::jointImpedance);
            looping = true;
            rtCon->startLoop(false);
            std::cout << "Re-entered RT mode; holding for 8 seconds" << std::endl;
            waitUntil(std::chrono::steady_clock::now() + 8s);
        }
    } catch (const std::exception &e) {
        std::cerr << "Control error: " << e.what() << std::endl;
        result = 1;
    }

    // 包括异常路径：每项独立清理，停止失败也继续尝试下电。
    interrupted.store(true);
    const auto cleanup = [&](auto operation) {
        try { operation(); }
        catch (const std::exception &e) {
            std::cerr << "Cleanup error: " << e.what() << std::endl;
            result = 1;
        }
    };
    if (looping) cleanup([&] { rtCon->stopLoop(); });
    if (moving) cleanup([&] { rtCon->stopMove(); });
    if (receiving) robot.stopReceiveRobotState();
    if (powered) cleanup([&] { robot.setPowerState(false, ec); check(ec); });
    if (connected) {
        cleanup([&] { robot.setMotionControlMode(MotionControlMode::NrtCommand, ec); check(ec); });
        cleanup([&] { robot.disconnectFromRobot(ec); check(ec); });
    }
    return result;
}

// 手动真机诊断：单臂、当前位置、关节阻抗保持 5 秒。不会执行 home/MoveAbsJ/轮盘轨迹。
// 直接使用 xCore SDK，排除双臂对象、运动学及 Aviator 工作线程的影响。
#include <rokae/robot.h>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void requestStop(int) { interrupted = 1; }
void check(const std::error_code &ec, const char *action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message());
}
}

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: aviator_rokae_hold_test <robot_ip> <local_ip>\n"
                     "Powers on ONE real robot and holds its current joint position in joint impedance for 5 seconds.\n";
        return 0;
    }
    if (argc != 3) {
        std::cerr << "Usage: aviator_rokae_hold_test <robot_ip> <local_ip>\n";
        return 2;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    rokae::xMateErProRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt;
    bool connected = false, powered = false, receiving = false, moving = false, looping = false;
    const char *stage = "connectToRobot";
    std::array<double, 7> q{}; // 回调使用的数据必须存活到异常路径的 stopLoop 清理结束。
    int ticks = 0;
    int result = 0;
    try {
        std::cout << "Single-arm impedance hold: robot=" << argv[1] << " local=" << argv[2] << std::endl;
        robot.connectToRobot(argv[1], argv[2]);
        connected = true;
        std::error_code ec;
        stage = "setMotionControlMode(RtCommand)";
        robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        check(ec, stage);
        stage = "setOperateMode(automatic)";
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        check(ec, stage);
        stage = "setPowerState(true)";
        powered = true;
        robot.setPowerState(true, ec);
        check(ec, stage);
        stage = "getRtMotionController";
        rt = robot.getRtMotionController().lock();
        if (!rt) throw std::runtime_error("No realtime controller");
        stage = "startReceiveRobotState";
        receiving = true;
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m});

        // 与官方示例相同：固定起始角度、默认回调参数、阻塞运行；只去掉正弦运动。
        std::function<rokae::JointPosition()> callback = [&] {
            rokae::JointPosition command(std::vector<double>(q.begin(), q.end()));
            if (++ticks >= 5000 || interrupted) command.setFinished();
            return command;
        };
        stage = "setJointImpedance";
        rt->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
        check(ec, stage);
        rt->setControlLoop(callback);

        stage = "getCurrentJointPos";
        bool received = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (robot.updateRobotState(std::chrono::steady_clock::duration::zero())) {
            received = true;
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("Realtime state queue did not drain within 100 ms");
        }
        if (!received || robot.getStateData(rokae::RtSupportedFields::jointPos_m, q) != 0) {
            q = robot.jointPos(ec);
            check(ec, "jointPos fallback");
        }
        for (double angle : q)
            if (!std::isfinite(angle)) throw std::runtime_error("Nonfinite joint position");
        std::cout << "Holding current q(rad):";
        for (double angle : q) std::cout << ' ' << angle;
        std::cout << std::endl;
        if (interrupted) throw std::runtime_error("Interrupted before startMove");

        stage = "startMove(jointImpedance)";
        moving = true;
        rt->startMove(rokae::RtControllerMode::jointImpedance);
        std::cout << "Joint impedance started; holding for 5 seconds" << std::endl;
        stage = "startLoop";
        looping = true;
        rt->startLoop(true);
        std::cout << "Hold finished; callbacks=" << ticks << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Failed stage=" << stage << ": " << e.what() << std::endl;
        result = 1;
    }

    // 每项清理独立尝试；stopLoop 报错不能阻止 stopMove 和下电。
    auto cleanup = [&](const char *action, auto operation) {
        try { operation(); }
        catch (const std::exception &e) {
            std::cerr << action << ": " << e.what() << std::endl;
            result = 1;
        }
    };
    if (looping) cleanup("stopLoop", [&] { rt->stopLoop(); });
    if (moving) cleanup("stopMove", [&] { rt->stopMove(); });
    if (receiving) robot.stopReceiveRobotState();
    std::error_code ec;
    if (powered) cleanup("power off", [&] { robot.setPowerState(false, ec); check(ec, "power off"); });
    if (connected) {
        cleanup("Idle", [&] { robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec); check(ec, "Idle"); });
        cleanup("disconnect", [&] { robot.disconnectFromRobot(ec); check(ec, "disconnect"); });
    }
    return result;
}

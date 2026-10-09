// 单臂 xMateER7 Pro：锁定启动位置，运行中通过键盘修改关节刚度。
#include <rokae/robot.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace {
using Joints = std::array<double, 7>;
constexpr Joints initialStiffness{500, 500, 500, 500, 50, 50, 50};
constexpr Joints maxStiffness{3000, 3000, 3000, 3000, 300, 300, 300};
std::atomic<bool> stopping{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void requestStop(int) { stopping.store(true, std::memory_order_relaxed); }

void check(const std::error_code &ec, const char *action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message());
}

// 保留 ISIG，使 Ctrl+C 仍触发退出；异常路径也恢复终端。
class Terminal {
public:
    Terminal() {
        if (!isatty(STDIN_FILENO)) throw std::runtime_error("Please run in an interactive terminal");
        if (tcgetattr(STDIN_FILENO, &saved_) != 0)
            throw std::system_error(errno, std::generic_category(), "tcgetattr");
        auto raw = saved_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }
    ~Terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
    Terminal(const Terminal &) = delete;
    Terminal &operator=(const Terminal &) = delete;
private:
    termios saved_{};
};

void printKeys() {
    std::cout << "\n1..7: select joint | +/-: change stiffness | v: enter exact value\n"
                 "s: enter step | p: show stiffness | h: help | q / Ctrl+C: quit\n"
                 "For v/s: type a number then Enter; Esc cancels. Units: Nm/rad.\n";
}

void keyboardLoop(const std::function<void(const Joints &)> &apply) {
    Joints stiffness = initialStiffness;
    unsigned selected = 0;
    double step = 10;
    char editing = 0;
    std::string number;
    const auto show = [&] {
        std::cout << "K = [";
        for (unsigned i = 0; i < stiffness.size(); ++i)
            std::cout << (i ? ", " : "") << stiffness[i];
        std::cout << "] Nm/rad; selected J" << selected + 1 << "; step=" << step << std::endl;
    };
    const auto setValue = [&](double value) {
        if (!std::isfinite(value) || value < 0 || value > maxStiffness[selected]) {
            std::cout << "Rejected: J" << selected + 1 << " stiffness must be in [0, "
                      << maxStiffness[selected] << "] Nm/rad\n";
            return;
        }
        auto candidate = stiffness;
        candidate[selected] = value;
        if (stopping.load()) return;
        apply(candidate); // 普通线程发送参数 RPC；成功后才更新显示值。
        stiffness = candidate;
        show();
    };
    printKeys();
    show();
    while (!stopping.load()) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 100);
        if (ready < 0) {
            if (errno == EINTR) continue;
            throw std::system_error(errno, std::generic_category(), "poll");
        }
        if (!ready) continue;
        if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            stopping.store(true);
            break;
        }
        char key = 0;
        const auto count = read(STDIN_FILENO, &key, 1);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            throw std::system_error(errno, std::generic_category(), "read");
        }
        if (!count || key == 'q' || key == 4) { // Ctrl+D
            stopping.store(true);
            break;
        }
        if (editing) {
            if (key == 27) {
                editing = 0;
                number.clear();
                std::cout << "\nCancelled\n";
            } else if (key == '\n' || key == '\r') {
                std::cout << '\n';
                double value = 0;
                bool valid = false;
                try {
                    std::size_t end = 0;
                    value = std::stod(number, &end);
                    valid = end == number.size() && std::isfinite(value);
                } catch (const std::invalid_argument &) {
                } catch (const std::out_of_range &) {
                }
                if (!valid) {
                    std::cout << "Rejected: enter a finite number\n";
                } else if (editing == 'v') {
                    setValue(value);
                } else if (value <= 0 || value > 3000) {
                    std::cout << "Rejected: step must be in (0, 3000]\n";
                } else {
                    step = value;
                    show();
                }
                editing = 0;
                number.clear();
            } else if (key == 127 || key == '\b') {
                if (!number.empty()) {
                    number.pop_back();
                    std::cout << "\b \b";
                }
            } else if (key >= 32 && key <= 126 && number.size() < 64) {
                number += key;
                std::cout << key;
            }
        } else if (key >= '1' && key <= '7') {
            selected = static_cast<unsigned>(key - '1');
            show();
        } else if (key == '+' || key == '=') {
            setValue(stiffness[selected] + step);
        } else if (key == '-') {
            setValue(stiffness[selected] - step);
        } else if (key == 'v' || key == 's') {
            editing = key;
            std::cout << (key == 'v' ? "\nStiffness = " : "\nStep = ");
        } else if (key == 'p') {
            show();
        } else if (key == 'h') {
            printKeys();
        }
        std::cout << std::flush;
    }
}

int runRobot(const char *robotIp, const char *localIp) {
    rokae::xMateErProRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt;
    bool connected = false, powered = false, receiving = false, moving = false, looping = false;
    Joints target{}; // 生命周期覆盖 SDK 回调及全部清理操作。
    std::thread keyboard;
    std::exception_ptr keyboardError;
    int result = 0;
    try {
        std::error_code ec;
        std::cout << "Single-arm hold: robot=" << robotIp << " local=" << localIp << std::endl;
        robot.connectToRobot(robotIp, localIp);
        connected = true;
        robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        check(ec, "setMotionControlMode(RtCommand)");
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        check(ec, "setOperateMode(automatic)");
        if (stopping.load()) throw std::runtime_error("Interrupted before power on");
        powered = true;
        robot.setPowerState(true, ec);
        check(ec, "setPowerState(true)");
        rt = robot.getRtMotionController().lock();
        if (!rt) throw std::runtime_error("No realtime controller");
        receiving = true;
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m});
        rt->setJointImpedance(initialStiffness, ec);
        check(ec, "setJointImpedance(initial)");

        // 与官方 helper 一样先清空状态队列，但加超时，避免无限等待。
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (robot.updateRobotState(std::chrono::steady_clock::duration::zero())) {
            if (stopping.load()) throw std::runtime_error("Interrupted while reading position");
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("State queue did not drain within 100 ms");
        }
        if (robot.getStateData(rokae::RtSupportedFields::jointPos_m, target) != 0)
            throw std::runtime_error("Cannot read initial joint position");
        for (double q : target)
            if (!std::isfinite(q)) throw std::runtime_error("Nonfinite initial joint position");
        std::cout << "Fixed target (rad):";
        for (double q : target) std::cout << ' ' << q;
        std::cout << std::endl;

        std::function<rokae::JointPosition()> callback = [&target] {
            rokae::JointPosition command(7);
            for (unsigned i = 0; i < target.size(); ++i) command.joints[i] = target[i];
            if (stopping.load(std::memory_order_relaxed)) command.setFinished();
            return command;
        };
        rt->setControlLoop(callback);
        if (stopping.load()) throw std::runtime_error("Interrupted before startMove");
        moving = true;
        rt->startMove(rokae::RtControllerMode::jointImpedance);
        keyboard = std::thread([&] {
            try {
                keyboardLoop([&](const Joints &values) {
                    std::error_code updateError;
                    rt->setJointImpedance(values, updateError);
                    check(updateError, "setJointImpedance(update)");
                });
            } catch (...) {
                keyboardError = std::current_exception();
                stopping.store(true);
            }
        });
        looping = true;
        rt->startLoop(true); // 主线程捕获 SDK 实时故障；键盘输入不会阻塞实时回调。
    } catch (const std::exception &e) {
        std::cerr << "\nControl error: " << e.what() << std::endl;
        result = 1;
    }
    stopping.store(true);
    if (keyboard.joinable()) keyboard.join();
    if (keyboardError) {
        try { std::rethrow_exception(keyboardError); }
        catch (const std::exception &e) { std::cerr << "\nKeyboard/update error: " << e.what() << std::endl; }
        catch (...) { std::cerr << "\nUnknown keyboard/update error\n"; }
        result = 1;
    }
    // 分项清理：某项失败仍继续停止运动、下电、断开连接。
    const auto cleanup = [&](const char *name, auto operation) {
        try { operation(); }
        catch (const std::exception &e) {
            std::cerr << name << ": " << e.what() << std::endl;
            result = 1;
        }
    };
    if (looping) cleanup("stopLoop", [&] { rt->stopLoop(); });
    if (moving) cleanup("stopMove", [&] { rt->stopMove(); });
    if (receiving) robot.stopReceiveRobotState();
    std::error_code ec;
    if (powered) cleanup("power off", [&] { robot.setPowerState(false, ec); check(ec, "power off"); });
    if (connected) {
        cleanup("NrtCommand", [&] {
            robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
            check(ec, "NrtCommand");
        });
        cleanup("disconnect", [&] { robot.disconnectFromRobot(ec); check(ec, "disconnect"); });
    }
    std::cout << "Control finished" << std::endl;
    return result;
}
} // namespace

int main(int argc, char **argv) {
    const bool dryRun = argc == 2 && std::string(argv[1]) == "--dry-run";
    const bool help = argc == 2 && std::string(argv[1]) == "--help";
    if (help || (!dryRun && argc != 3)) {
        std::cout << "Usage: change_stiffness <robot_ip> <local_ip>\n"
                     "       change_stiffness --dry-run\n"
                     "Powers on ONE xMateER7 Pro and holds its startup joint position.\n"
                     "Initial K: [500, 500, 500, 500, 50, 50, 50] Nm/rad.\n"
                     "--dry-run checks keyboard controls without connecting to a robot.\n";
        printKeys();
        return help ? 0 : 2;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try {
        Terminal terminal;
        if (dryRun) {
            std::cout << "DRY RUN: no robot connection; values are local only.\n";
            keyboardLoop([](const Joints &) {});
            return 0;
        }
        return runRobot(argv[1], argv[2]);
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

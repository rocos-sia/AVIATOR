#include "aviator/Aviator.hpp"
#include "aviator/Kinematics.hpp"
#include "aviator/CollisionChecker.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <poll.h>
#include <sstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>
#include <future>
#include "RemoteLink.hpp"
#include "StateMachineRuntime.hpp"
#include <csignal>
namespace fs = std::filesystem;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt(int) { interrupted = 1; }

// 只有这个入口使用的普通函数，不再单独建立 application 接口。
static void status(aviator::Aviator &robot);
static void interactive(aviator::Aviator &robot, std::atomic<bool> &exit);

int main(int argc, char **argv) {
    try {
        bool demo = false, servo_demo = false;
        bool state_machine = false, fsm_simulation = false;
        std::string safety_file;
        fs::path config = aviator::defaultSystemConfig();
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--demo") demo = true;
            else if (arg == "--state-machine") state_machine = true;
            else if (arg == "--fsm-simulation") { state_machine = true; fsm_simulation = true; }
            else if (arg == "--safety-file" && i + 1 < argc) { state_machine = true; safety_file = argv[++i]; }
            else if (arg == "--servo-demo") servo_demo = true;
            else if (arg == "--headless") {} // Window belongs to manipulator.
            else if (arg == "--config" && i + 1 < argc) config = argv[++i];
            else if (arg == "--help") {
                std::cout << "aviator_core [--config <system.yaml>] [--demo | --servo-demo]\n"
                             "             [--state-machine [--safety-file PATH] | --fsm-simulation]\n"
                             "Start aviator_bus and manipulator first. Backend is configured in robot.yaml.\n";
                return 0;
            } else throw std::runtime_error("Unknown or incomplete option: " + arg);
        }
        if (demo && servo_demo) throw std::runtime_error("Choose --demo or --servo-demo");
        if (state_machine && (demo || servo_demo)) throw std::runtime_error("FSM and legacy demos are separate modes");
        const auto settings = aviator::loadMotionConfig(config);
        if (state_machine) {
            std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
            return aviator::runStateMachine(settings, safety_file, fsm_simulation, interrupted);
        }
        auto link = std::make_unique<aviator::RemoteLink>(settings);
        auto* connection = link.get();
        aviator::Aviator robot(std::move(link), nullptr, nullptr, settings.robot.string());
        robot.Init();
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, interrupt);
        std::cout << "Core connected through ZMQ | config: " << settings.system << std::endl;
        std::atomic<bool> exit{false}, task_done{false};
        int result = 0;
        // 阻塞动作在工作线程执行，主线程维护任务心跳并处理退出。
        std::thread task([&] {
            try {
                if (demo || servo_demo) {
                    robot.Enable();
                    std::cout << "DEMO approaching handles" << std::endl;
                    robot.ApproachHandles();
                    status(robot);
                    std::cout << "robot.ApproachHandles() done" << std::endl;
                    robot.LockHandles();
                    status(robot);
                    std::cout << "robot.LockHandles() done" << std::endl;
                    if (demo) {
                        // 每行依次为：转角 rad、推拉 m、速度倍率 v。
                        for (const auto target : {std::array<double, 3>{.87266, 0, .8},
                                                  {-.87266, 0, .8},
                                                  {0, 0, .8},
                                                  {0, -.170, .8},
                                                  {0, 0, .8}}) {
                            if (exit) break;
                            std::cout << "DEMO target angle=" << target[0]
                                      << " displacement=" << target[1] << std::endl;
                            robot.MoveWheel(target[0], target[1], target[2]);
                            status(robot);
                        }
                    } else {
                        // 在线示例：每 20 ms 发布一次最新目标，每个目标持续 2 s。
                        for (const auto target : {std::array<double, 3>{.02, -.002, .5},
                                                  {-.015, -.001, .5},
                                                  {0, 0, .5}}) {
                            auto next = std::chrono::steady_clock::now();
                            for (int tick = 0; tick < 100 && !exit; ++tick) {
                                robot.ServoWheel(target[0], target[1], target[2]);
                                next += std::chrono::milliseconds(20);
                                std::this_thread::sleep_until(next);
                            }
                            status(robot);
                        }
                        robot.Stop();
                        while (robot.GetState() == "SERVO")
                            std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        if (robot.GetState() != "LOCKED")
                            throw std::runtime_error(robot.GetStatus().motion_error);
                    }

                    robot.UnlockHandles();
                    robot.Disable();
                    std::cout << "Demo completed" << std::endl;
                } else {
                    interactive(robot, exit);
                }
            } catch (const std::exception &e) {
                std::cerr << "Execution failed: " << e.what() << std::endl;
                robot.Stop(); result = 1;
            }
            exit = true;
            task_done = true;
        });
        while (!exit) {
            connection->heartbeat();
            connection->report(robot.GetState());
            if (interrupted) { robot.Stop(); exit = true; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        // Keep the local-task lease alive while the motion worker finishes cancellation.
        while (!task_done) {
            connection->heartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        task.join();
        try {
            auto cleanup = std::async(std::launch::async, [&] { robot.Disable(); });
            while (cleanup.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready)
                connection->heartbeat();
            cleanup.get();
        } catch (const std::exception& e) {
            std::cerr << "Shutdown: " << e.what() << std::endl;
        }
        return result;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

static void status(aviator::Aviator &robot) {
    const auto s = robot.GetStatus();
    std::cout << "state=" << robot.GetState() << " angle=" << s.angle
              << " displacement=" << s.displacement << " locked=" << s.locked
              << " mode=" << (s.open_loop ? "open_loop" : "simulation")
              << " error=" << s.motion_error << " fault=" << s.fault << std::endl;
}

static void interactive(aviator::Aviator &robot, std::atomic<bool> &exit) {
    std::cout << "enable | disable | approach | lock | unlock | reset | status | stop | quit\n"
                 "wheel/servo <angle_rad> <displacement_m> [v=0.5]\n";
    std::atomic<bool> busy{false};
    std::thread action;
    std::string pending;
    bool eof = false;

    while (!exit) {
        // 按行读取，但不阻塞等待键盘：关闭窗口后也能及时退出。
        if (pending.find('\n') == std::string::npos && !eof) {
            pollfd fd{STDIN_FILENO, POLLIN, 0};
            if (poll(&fd, 1, 50) <= 0) continue;
            char buffer[1024];
            const auto size = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (size > 0) pending.append(buffer, static_cast<size_t>(size));
            else eof = true;
            continue;
        }
        if (pending.empty()) break;
        const auto newline = pending.find('\n');
        std::istringstream in(pending.substr(0, newline));
        pending.erase(0, newline == std::string::npos ? pending.size() : newline + 1);

        try {
            std::string command, extra;
            in >> command;
            if (command.empty()) continue;
            if (command == "quit" || command == "exit") { exit = true; break; }
            if (command == "stop") { robot.Stop(); continue; }
            if (command == "status") { status(robot); continue; }

            double angle = 0, displacement = 0, v = 0.5;
            if (command == "wheel" || command == "servo") {
                if (!(in >> angle >> displacement)) throw std::runtime_error("Expected angle and displacement");
                in >> std::ws;
                if (!in.eof() && !(in >> v)) throw std::runtime_error("Invalid speed ratio");
            } else if (command != "enable" && command != "disable" && command != "approach" &&
                       command != "lock" && command != "unlock" && command != "reset") {
                throw std::runtime_error("Unknown command: " + command);
            }
            if (in >> extra) throw std::runtime_error("Unexpected argument: " + extra);
            if (busy) throw std::runtime_error("Another action is running; use status or stop");

            // Servo 本身非阻塞，直接调用；不创建指令队列或每周期线程。
            if (command == "servo") {
                robot.ServoWheel(angle, displacement, v);
                continue;
            }
            if (action.joinable()) action.join();
            busy = true;
            action = std::thread([&, command, angle, displacement, v] {
                try {
                    if (command == "enable") robot.Enable();
                    else if (command == "disable") robot.Disable();
                    else if (command == "approach") robot.ApproachHandles();
                    else if (command == "lock") robot.LockHandles();
                    else if (command == "unlock") robot.UnlockHandles();
                    else if (command == "reset") robot.ResetFault();
                    else if (command == "wheel") robot.MoveWheel(angle, displacement, v);
                    std::cout << "[ok] " << robot.GetState() << std::endl;
                } catch (const std::exception &e) {
                    std::cerr << "[error] " << e.what() << std::endl;
                }
                busy = false;
            });
        } catch (const std::exception &e) {
            std::cerr << "[error] " << e.what() << std::endl;
        }
    }
    if (exit) robot.Stop();
    if (action.joinable()) action.join();
    robot.Stop();
}

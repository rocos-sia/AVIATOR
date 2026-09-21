#include "aviator/Aviator.hpp"
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
#ifdef AVIATOR_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif
#ifdef AVIATOR_HAVE_GLFW
#include "Viewer.hpp"
#endif
namespace fs = std::filesystem;

// 只有这个入口使用的普通函数，不再单独建立 application 接口。
static void status(aviator::Aviator &robot);
static void interactive(aviator::Aviator &robot, std::atomic<bool> &exit);

int main(int argc, char **argv) {
    try {
        bool demo = false, servo_demo = false, headless = false;
        fs::path config = fs::read_symlink("/proc/self/exe").parent_path() / "config/aviator.yaml";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--demo") demo = true;
            else if (arg == "--servo-demo") servo_demo = true;
            else if (arg == "--headless") headless = true;
            else if (arg == "--config" && i + 1 < argc) config = argv[++i];
            else if (arg == "--help") {
                std::cout << "aviator [--config <yaml-or-directory>] [--demo | --servo-demo] [--headless]\n"
                             "Backend is selected by YAML: mujoco or rokae. Default: mujoco.\n";
                return 0;
            } else throw std::runtime_error("Unknown or incomplete option: " + arg);
        }
        if (demo && servo_demo) throw std::runtime_error("Choose --demo or --servo-demo");
        if (fs::is_directory(config)) config /= "aviator.yaml";
        config = fs::absolute(config);
        const auto cfg = YAML::LoadFile(config.string());
        const std::string backend = cfg["backend"].as<std::string>("mujoco");
        mjModel *model = nullptr;
        mjData *data = nullptr;
#ifdef AVIATOR_HAVE_MUJOCO
        // Owners precede robot: its physics thread is destroyed before model/data.
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_owner(nullptr, mj_deleteModel);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data_owner(nullptr, mj_deleteData);
        if (backend == "mujoco") {
            fs::path file = cfg["model"].as<std::string>("../model/aviator.xml");
            if (file.is_relative()) file = config.parent_path() / file;
            char error[1024]{};
            model_owner.reset(mj_loadXML(file.c_str(), nullptr, error, sizeof(error)));
            if (!model_owner) throw std::runtime_error(error);
            model = model_owner.get();
            data_owner.reset(mj_makeData(model));
            data = data_owner.get();
            if (!data) throw std::runtime_error("Cannot allocate mjData");
            int key = mj_name2id(model, mjOBJ_KEY, "aviator_home");
            if (key < 0) throw std::runtime_error("Missing aviator_home keyframe");
            mj_resetDataKeyframe(model, data, key);
            mj_forward(model, data);
        }
#else
        if (backend == "mujoco") throw std::runtime_error("MuJoCo backend was disabled at build time");
#endif
        aviator::Aviator robot(model, data, config.string());
        robot.Init();
        robot.SetRealTime(!headless || !demo);
        if (robot.GetStatus().open_loop) std::cout << "Open-loop wheel control: wheel pose and lock state are software references.\n";
        std::cout << "Backend: " << backend << " | config: " << config << std::endl;
#ifdef AVIATOR_HAVE_GLFW
        std::unique_ptr<aviator::Viewer> viewer;
        if (model && !headless) {
            viewer = std::make_unique<aviator::Viewer>(model);
            viewer->draw(data, *robot.PhysicsMutex());
        }
#else
        if (model && !headless) std::cout << "Viewer disabled at build time; running headless\n";
#endif
        std::atomic<bool> exit{false};
        int result = 0;
        // GLFW 留在主线程；阻塞运动放在一个工作线程，窗口仍可旋转、缩放和退出。
        std::thread task([&] {
            try {
                if (demo || servo_demo) {
                    robot.Enable();
                    std::cout << "DEMO approaching handles" << std::endl;
                    robot.ApproachHandles();
                    status(robot);
                    robot.LockHandles();
                    status(robot);

                    if (demo) {
                        // 每行依次为：转角 rad、推拉 m、速度倍率 v。
                        for (const auto target : {std::array<double, 3>{.87266, 0, .5},
                                                  {-.87266, 0, .5},
                                                  {0, 0, .5},
                                                  {0, -.170, .5},
                                                  {0, 0, .5}}) {
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
        });
        while (!exit) {
#ifdef AVIATOR_HAVE_GLFW
            if (viewer && !viewer->draw(data, *robot.PhysicsMutex())) {
                robot.Stop(); exit = true;
            }
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        task.join();
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
              << " error=" << s.motion_error << " ready=" << s.ready << " fault=" << s.fault << std::endl;
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

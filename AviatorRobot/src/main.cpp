#include "aviator/Aviator.hpp"
#include <array>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
namespace {
volatile std::sig_atomic_t interrupted = 0;
}

int main(int argc, char **argv) {
    int bus = 0;
    bool demo = false;
    fs::path config = fs::path(AVIATOR_CONFIG_DIR) / "aviator.yaml";
    std::error_code error;
    const auto exe = fs::read_symlink("/proc/self/exe", error);
    if (!error && fs::exists(exe.parent_path() / "config/aviator.yaml"))
        config = exe.parent_path() / "config/aviator.yaml";
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc)
                config = argv[++i];
            else if (arg == "--ecat-id" && i + 1 < argc)
                bus = std::stoi(argv[++i]);
            else if (arg == "--demo")
                demo = true;
            else if (arg == "--help") {
                std::cout << "aviatorAppMain [--config path] [--ecat-id n] [--demo]\n"
                          << "Start rocos_mujoco_sim first on the same bus.\n"
                          << "Commands: enable, approach, lock, wheel <rad> <m> [seconds],\n"
                          << "          status, stop, unlock, disable, reset, quit\n";
                return 0;
            } else
                throw std::runtime_error("Unknown or incomplete option: " + arg);
        }
        aviator::Aviator robot(config.string(), bus);
        robot.Init();
        auto status = [&] {
            const auto f = robot.GetStatus();
            std::cout << std::fixed << std::setprecision(6) << "AVIATOR state=" << robot.GetState()
                      << " angle=" << f.angle << " displacement=" << f.displacement << " locked=" << f.locked
                      << " fault=" << f.fault << " position_error=" << f.position_error[0] << ","
                      << f.position_error[1] << " rotation_error=" << f.rotation_error[0] << ","
                      << f.rotation_error[1] << std::endl;
        };
        std::signal(SIGINT, [](int) { interrupted = 1; });
        std::signal(SIGTERM, [](int) { interrupted = 1; });
        std::atomic<bool> monitoring{true};
        std::thread monitor([&] {
            while (monitoring) {
                if (interrupted)
                    robot.Stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        int result = 0;
        try {
            if (demo) {
                robot.Enable();
                std::cout << "DEMO approaching handles" << std::endl;
                robot.ApproachHandles();
                status();
                robot.LockHandles();
                status();
                for (const auto target : {std::array<double, 3>{.87266, 0, 3},
                                          {-.87266, 0, 3},
                                          {0, 0, 3},
                                          {0, -.170, 3},
                                          {.87266, -.170, 8},
                                          {-.87266, -.170, 8},
                                          {0, 0, 3}}) {
                    std::cout << "DEMO target angle=" << target[0] << " displacement=" << target[1]
                              << std::endl;
                    robot.MoveWheel(target[0], target[1], target[2]);
                    status();
                }
                robot.UnlockHandles();
                robot.Disable();
                status();
                std::cout << "DEMO PASS" << std::endl;
            } else {
                std::cout << "Ready. Commands: enable, approach, lock, wheel <rad> <m> [seconds], status, "
                             "stop, unlock, disable, reset, quit\n";
                std::future<void> task;
                std::string line;
                while (!interrupted && std::getline(std::cin, line)) {
                    if (task.valid() && task.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                        try {
                            task.get();
                        } catch (const std::exception &e) {
                            std::cerr << "ERROR: " << e.what() << '\n';
                        }
                    }
                    std::istringstream input(line);
                    std::string command;
                    input >> command;
                    if (command.empty())
                        continue;
                    if (command == "quit")
                        break;
                    if (command == "stop") {
                        robot.Stop();
                        continue;
                    }
                    if (command == "status") {
                        status();
                        continue;
                    }
                    if (task.valid()) {
                        std::cerr << "Motion is busy; use stop or status\n";
                        continue;
                    }
                    if (command == "wheel") {
                        double angle, translation, seconds = 4;
                        if (!(input >> angle >> translation)) {
                            std::cerr << "Expected wheel <rad> <m> [seconds]\n";
                            continue;
                        }
                        input >> seconds;
                        task = std::async(std::launch::async, [&, angle, translation, seconds] {
                            robot.MoveWheel(angle, translation, seconds);
                            status();
                        });
                    } else {
                        task = std::async(std::launch::async, [&, command] {
                            if (command == "enable")
                                robot.Enable();
                            else if (command == "approach")
                                robot.ApproachHandles();
                            else if (command == "lock")
                                robot.LockHandles();
                            else if (command == "unlock")
                                robot.UnlockHandles();
                            else if (command == "disable")
                                robot.Disable();
                            else if (command == "reset")
                                robot.ResetFault();
                            else
                                throw std::runtime_error("Unknown command: " + command);
                            status();
                        });
                    }
                }
                robot.Stop();
                if (task.valid()) {
                    try {
                        task.get();
                    } catch (const std::exception &e) {
                        std::cerr << e.what() << '\n';
                    }
                }
                robot.UnlockHandles();
                robot.Disable();
            }
        } catch (const std::exception &e) {
            robot.Stop();
            std::cerr << "AVIATOR ERROR: " << e.what() << '\n';
            try {
                status();
                robot.UnlockHandles();
                robot.Disable();
            } catch (...) {
            }
            result = 1;
        }
        monitoring = false;
        monitor.join();
        return result;
    } catch (const std::exception &e) {
        std::cerr << "AVIATOR ERROR: " << e.what() << '\n';
        return 1;
    }
}

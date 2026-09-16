// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// main.cpp
// rocos-mujoco 仿真器入口点。
// 以独立进程运行 MuJoCo 仿真，通过 POSIX 共享内存模拟 EtherCAT 主站，
// 供 rocos-app 的 rocos::Hardware 连接 —— 与真实硬件完全一致的接口。

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "rocos_mujoco/mujoco_simulator.hpp"

namespace fs = std::filesystem;

static fs::path executableDirectory() {
    std::error_code error;
    const auto executable = fs::read_symlink("/proc/self/exe", error);
    return error ? fs::current_path() : executable.parent_path();
}

static std::string resolveFile(const std::string &name) {
    if (name.empty() || fs::path(name).is_absolute()) return name;
    for (const auto& root : {fs::current_path(), executableDirectory(),
                             fs::path(ROCOS_MUJOCO_SOURCE_DIR)}) {
        const auto candidate = root / name;
        if (fs::is_regular_file(candidate)) return candidate.string();
    }
    return name;
}

static std::string sparcModel(const std::string& filename) {
    const auto source = fs::path(SPARC_MODEL_DIR) / "urdf" / filename;
    if (fs::is_regular_file(source)) return source.string();
    return (executableDirectory() / "model/SPARC/urdf" / filename).string();
}

static void printUsage(const char *prog) {
    std::cout << "用法: " << prog << " [选项]\n"
              << "选项:\n"
              << "  --model <name>       预置模型选择: aviator | sparc | sparc_urdf | charge_arm | talon\n"
              << "                        (自动设置 --urdf 和 --config)\n"
              << "  --urdf <path>         MJCF/URDF 模型路径\n"
              << "                         (默认: model/aviator.xml)\n"
              << "  --config <path>       硬件描述 YAML 路径\n"
              << "                         (默认: config/hardware_aviator_config.yaml)\n"
              << "  --ecat-id <n>         EtherCAT 主站 ID (默认: 0)\n"
              << "  --cycle-time <us>     控制周期 [微秒] (默认: 1000)\n"
              << "  --duration <sec>      运行时长 [秒] (默认: -1 = 永久)\n"
              << "  --visualize, -v       启用 3D 可视化窗口\n"
              << "  --headless            显式禁用可视化（默认行为，无需指定）\n"
              << "  --help                显示此帮助信息\n"
              << "\n"
              << "共享内存命名规则 (ecat_id=0):\n"
              << "  EcatBus:  /ecm0\n"
              << "  PD 输入:  /pd_input0\n"
              << "  PD 输出:  /pd_output0\n"
              << "  信号量:   /sync0_0 ... /sync0_9\n"
              << "\n"
              << "典型用法:\n"
              << "  # 默认启动 AVIATOR 双臂与被动操纵盘\n"
              << "  " << prog << " -v\n"
              << "  " << prog << " --model aviator --duration 5\n"
              << "\n"
              << "  # 启动 SPARC 场景或单独加载 URDF\n"
              << "  " << prog << " --model sparc -v\n"
              << "  " << prog << " --model sparc_urdf --duration 5\n"
              << "\n"
              << "  # 启动 charge_arm 仿真（可视化）\n"
              << "  " << prog << " --model charge_arm -v\n"
              << "\n"
              << "  # 启动 talon 仿真（无头模式，运行 30 秒 — 用于测试）\n"
              << "  " << prog << " --model talon --duration 30\n"
              << "\n"
              << "  # 然后在另一个终端运行测试:\n"
              << "  ./build/bin/hardware_enable_test 0 charge_arm_config.yaml\n"
              << std::endl;
}

int main(int argc, char *argv[]) {
    // AVIATOR: 14 个双臂驱动轴，操纵盘的旋转/推拉两轴为被动关节。
    std::string urdf_path   = "model/aviator.xml";
    std::string config_path = "config/hardware_aviator_config.yaml";
    std::string explicit_urdf, explicit_config;
    int         ecat_id     = 0;
    double      cycle_time  = 1000.0;   // 微秒
    double      duration    = -1.0;     // 秒，-1 = 永久
    bool        visualize   = false;

    // ---- 解析命令行参数 --------------------------------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--model" && i + 1 < argc) {
            std::string model = argv[++i];
            if (model == "aviator" || model == "AVIATOR") {
                urdf_path   = "model/aviator.xml";
                config_path = "config/hardware_aviator_config.yaml";
            } else if (model == "sparc" || model == "SPARC" || model == "sparc_urdf") {
                urdf_path = sparcModel(model == "sparc_urdf" ? "SPARC.urdf" : "scene.xml");
                config_path = "config/hardware_SPARC_config.yaml";
            } else if (model == "charge_arm") {
                urdf_path   = "model/charge_arm.xml";
                config_path = "config/charge_arm_config.yaml";
            } else if (model == "talon") {
                urdf_path   = "model/scene.xml";
                config_path = "config/hardware_talon_config.yaml";
            } else {
                std::cerr << "未知模型: " << model
                          << " (可选: aviator, sparc, sparc_urdf, charge_arm, talon)" << std::endl;
                return 1;
            }
        } else if (arg == "--visualize" || arg == "-v") {
            visualize = true;
        } else if (arg == "--headless") {
            visualize = false;
        } else if (arg == "--urdf" && i + 1 < argc) {
            explicit_urdf = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            explicit_config = argv[++i];
        } else if (arg == "--ecat-id" && i + 1 < argc) {
            char *end = nullptr;
            long val = std::strtol(argv[++i], &end, 10);
            if (end == argv[i] || *end != '\0') {
                std::cerr << "无效的 --ecat-id: " << argv[i] << std::endl;
                return 1;
            }
            ecat_id = static_cast<int>(val);
        } else if (arg == "--cycle-time" && i + 1 < argc) {
            cycle_time = std::atof(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atof(argv[++i]);
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // 显式路径优先于预置，不受参数顺序影响。
    urdf_path = resolveFile(explicit_urdf.empty() ? urdf_path : explicit_urdf);
    config_path = resolveFile(explicit_config.empty() ? config_path : explicit_config);

    // ---- 打印启动信息 ----------------------------------------------------
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  ROCOS MuJoCo 仿真器 (EtherCAT 主站模拟)                    ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  模型:      " << std::setw(49) << std::left << urdf_path   << "║\n";
    std::cout << "║  配置:      " << std::setw(49) << std::left << config_path << "║\n";
    std::cout << "║  ecat_id:   " << std::setw(49) << std::left << ecat_id     << "║\n";
    std::cout << "║  周期:      " << std::setw(46) << std::left
              << (std::to_string(static_cast<int>(cycle_time)) + " us") << "║\n";
    std::cout << "║  可视化:    " << std::setw(49) << std::left
              << (visualize ? "开启" : "关闭 (无头模式)") << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    // ---- 创建并运行仿真器 ------------------------------------------------
    rocos_mujoco::MujocoSimulator sim(urdf_path, config_path,
                                       ecat_id, cycle_time);
    if (visualize) {
        sim.enableVisualization();
    }

    if (!sim.initialize()) {
        std::cerr << "仿真器初始化失败。" << std::endl;
        return 1;
    }

    sim.run(duration);

    std::cout << "仿真已结束。" << std::endl;
    return 0;
}

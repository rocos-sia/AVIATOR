#include "aviator/Aviator.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

using namespace aviator;

void printStatus(const Status &status) {
    std::cout << "  Status: angle=" << status.angle << " displacement=" << status.displacement
              << " locked=" << status.locked << " ready=" << status.ready
              << " fault=" << status.fault << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
        return 1;
    }

    try {
        std::string config_path = argv[1];
        std::cout << "Loading config: " << config_path << std::endl;

        // 创建 Aviator 实例
        Aviator aviator(config_path);
        std::cout << "=== Aviator created. State: " << aviator.GetState() << " ===" << std::endl;
        std::cout << std::endl;

        // 使能双臂
        aviator.Enable();
        std::cout << "=== Enabled both arms ===" << std::endl;
        std::cout << std::endl;

        // 初始化（移动到 home 位置）
        aviator.Initialize();
        auto status = aviator.GetStatus();
        std::cout << "=== Initialized (moved to home) ===" << std::endl;
        printStatus(status);
        std::cout << std::endl;

        // 接近把手
        std::cout << "=== DEMO: approaching handles ===" << std::endl;
        aviator.ApproachHandles();
        status = aviator.GetStatus();
        std::cout << "=== Approached handles ===" << std::endl;
        printStatus(status);
        std::cout << std::endl;

        // 锁定把手
        aviator.LockHandles();
        status = aviator.GetStatus();
        std::cout << "=== Locked handles ===" << std::endl;
        printStatus(status);
        std::cout << std::endl;

        // 转动轮盘
        std::cout << "=== DEMO: turning wheel ===" << std::endl;
        double angle = 15.0 * M_PI / 180.0;      // 15 度
        double displacement = 0.02;               // 2 cm
        aviator.MoveWheel(angle, displacement, 3.0);
        status = aviator.GetStatus();
        std::cout << "=== Moved wheel ===" << std::endl;
        printStatus(status);
        std::cout << std::endl;

        // 解锁把手
        aviator.UnlockHandles();
        status = aviator.GetStatus();
        std::cout << "=== Unlocked handles ===" << std::endl;
        printStatus(status);
        std::cout << std::endl;

        std::cout << "=== DEMO completed successfully ===" << std::endl;
        return 0;

    } catch (const std::exception &e) {
        std::cerr << std::endl;
        std::cerr << "*** ERROR: " << e.what() << " ***" << std::endl;
        return 1;
    }
}

#include "aviator/Aviator.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

using namespace aviator;

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
        std::cout << "=== Aviator created ===" << std::endl;

        // 使能
        aviator.Enable();
        std::cout << "=== Enabled ===" << std::endl;

        // 初始化
        aviator.Initialize();
        std::cout << "=== Initialized ===" << std::endl;

        auto status = aviator.GetStatus();
        std::cout << "Status: angle=" << status.angle
                  << " displacement=" << status.displacement << std::endl;

        std::cout << "=== Test completed ===" << std::endl;
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}

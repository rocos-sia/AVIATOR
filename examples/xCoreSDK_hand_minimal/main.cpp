// 最小顺序示例：全部逻辑在 main() 中，只读一次角度或初始化串口。
#include <rokae/robot.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    try {
        if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--help")) {
            std::cout << "用法：rh56_sdk_demo ROBOT_IP [ID] [--init-485] [--dry-run]\n"
                         "ID 默认 1，范围 1..254；默认只读取一次六通道角度，张开/闭合循环已屏蔽。\n"
                         "--init-485：设置 115200/8N1 和 500 ms 超时，需重启控制器后生效。\n"
                         "--dry-run：只打印计划和报文，不连接设备。\n";
            return 0;
        }

        // 1. 参数。保留原来的 IP / ID 用法。
        bool init485 = false, dryRun = false;
        std::vector<std::string> args;
        for (int i = 1; i < argc; ++i) {
            const std::string arg(argv[i]);
            if (arg == "--init-485") init485 = true;
            else if (arg == "--dry-run") dryRun = true;
            else if (arg.rfind("--", 0) == 0) throw std::invalid_argument("未知选项：" + arg);
            else args.push_back(arg);
        }
        if (args.empty() || args.size() > 2)
            throw std::invalid_argument("参数数量错误：只接受 ROBOT_IP [ID]，见 --help");
        int id = 1;
        if (args.size() == 2) {
            size_t end = 0;
            id = std::stoi(args[1], &end);
            if (end != args[1].size() || id < 1 || id > 254)
                throw std::invalid_argument("ID 必须为 1..254 的整数");
        }
        std::cout << "机器人=" << args[0] << "，手 ID=" << id
                  << (init485 ? "，仅初始化 485" : "，只读一次角度") << std::endl;

        // 2. RH56 读角度请求：地址 1546 (0x060A)，读取 12 字节。
        // ID=1 时完整报文为 EB 90 01 04 11 0A 06 0C 32。
        std::vector<uint8_t> tx{0xEB, 0x90, uint8_t(id), 0x04, 0x11, 0x0A, 0x06, 0x0C, 0};
        for (size_t i = 2; i < 8; ++i) tx[8] += tx[i];
        if (!init485) {
            std::cout << "TX（待发送） ";
            for (auto b : tx) std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(b) << ' ';
            std::cout << std::dec << std::endl;
        }
        if (dryRun) {
            std::cout << (init485
                ? "dry-run：连接 ArRobot → xPanelRS485Init(115200, none, stopbit1) → setxPanelRS485Overtime(500) → 断开；重启控制器后生效。\n"
                : "dry-run：连接 ArRobot → XPRS485SendData(9, 20) → 断开；沿用当前 485 参数和超时。\n");
            return 0;
        }

        // SDK 在工作目录写日志，构造前为本次运行创建独立目录。
        auto pattern = (std::filesystem::temp_directory_path() / "rh56-sdk-XXXXXX").string();
        const char* directory = mkdtemp(pattern.data());
        if (!directory) throw std::runtime_error("无法创建 SDK 日志目录");
        std::filesystem::create_directories(std::filesystem::path(directory) / "logs");
        std::filesystem::current_path(directory);
        std::cout << "SDK 日志目录：" << directory << std::endl;

        // 3. 连接机器人。
        rokae::ArRobot robot;
        std::error_code ec;
        robot.connectToRobot(args[0]);
        std::cout << "已连接机器人：" << args[0] << std::endl;
        int result = 0;
        try {
            if (init485) {
                // 4a. 仅初始化；本次不发送 RH56 报文。
                robot.xPanelRS485Init(rokae::xPanelCommOpt::Rs485Baud::baud115200,
                    rokae::xPanelCommOpt::CheckBit::none, rokae::xPanelCommOpt::StopBit::stopbit1, ec);
                if (ec) throw std::system_error(ec, "xPanelRS485Init");
                robot.setxPanelRS485Overtime(500, ec);
                if (ec) throw std::system_error(ec, "setxPanelRS485Overtime");
                std::cout << "485 初始化成功，请重启 AR 控制器后再读取。" << std::endl;
            } else {
                // 4b. 沿用当前参数和超时，直接调用一次裸数据收发接口。
                std::vector<uint8_t> rx;
                rx.assign(20, 0);
                robot.XPRS485SendData((int)tx.size(), (int)rx.size(), tx, rx, ec);
                std::cout << (ec ? "RX buffer（SDK 失败，无有效应答） " : "RX ");
                for (auto b : rx) std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(b) << ' ';
                std::cout << std::dec << " ec=" << ec.value() << std::endl;
                if (ec) throw std::system_error(ec, "XPRS485SendData");

                // 5. 成功后检查完整应答，再按小端序打印六通道角度。
                if (rx.size() != 20) throw std::runtime_error("RH56 应答长度不是 20 字节");
                uint8_t sum = 0;
                for (size_t i = 2; i < 19; ++i) sum += rx[i];
                if (rx[0] != 0x90 || rx[1] != 0xEB || rx[2] != id || rx[3] != 0x0F ||
                    rx[4] != 0x11 || rx[5] != 0x0A || rx[6] != 0x06 || rx[19] != sum)
                    throw std::runtime_error("RH56 应答帧头、ID、长度字段、指令、地址或校验和错误");
                std::cout << "实际角度（协议刻度，不是度数）：";
                for (size_t i = 7; i < 19; i += 2)
                    std::cout << (unsigned(rx[i]) | (unsigned(rx[i + 1]) << 8)) << ' ';
                std::cout << std::endl;

#if 0 // 暂时屏蔽张开/闭合控制，仅保留代码供后续排查。
                // 6. 循环写目标角度 1486 (0x05CE)，保留拇指旋转，速度和力阈值不另写。
                const unsigned thumbRotation = unsigned(rx[17]) | (unsigned(rx[18]) << 8);
                if (thumbRotation > 1000) throw std::runtime_error("拇指旋转反馈超出 0..1000");
                bool open = true;
                while (true) {
                    tx = {0xEB, 0x90, uint8_t(id), 0x0F, 0x12, 0xCE, 0x05};
                    for (int channel = 0; channel < 6; ++channel) {
                        const unsigned angle = channel == 5 ? thumbRotation : (open ? 1000 : 0);
                        tx.push_back(uint8_t(angle & 0xFF));
                        tx.push_back(uint8_t(angle >> 8));
                    }
                    sum = 0;
                    for (size_t i = 2; i < tx.size(); ++i) sum += tx[i];
                    tx.push_back(sum);
                    std::cout << (open ? "张开" : "闭合") << " TX（待发送） ";
                    for (auto b : tx) std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(b) << ' ';
                    std::cout << std::dec << std::endl;

                    rx.assign(9, 0);
                    robot.XPRS485SendData((int)tx.size(), (int)rx.size(), tx, rx, ec);
                    std::cout << (ec ? "RX buffer（SDK 失败，无有效应答） " : "RX ");
                    for (auto b : rx) std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(b) << ' ';
                    std::cout << std::dec << " ec=" << ec.value() << std::endl;
                    if (ec) throw std::system_error(ec, "XPRS485SendData 写角度");
                    if (rx.size() != 9) throw std::runtime_error("RH56 写角度应答长度不是 9 字节");
                    sum = 0;
                    for (size_t i = 2; i < 8; ++i) sum += rx[i];
                    if (rx[0] != 0x90 || rx[1] != 0xEB || rx[2] != id || rx[3] != 0x04 ||
                        rx[4] != 0x12 || rx[5] != 0xCE || rx[6] != 0x05 || rx[8] != sum)
                        throw std::runtime_error("RH56 写角度应答格式或校验和错误");
                    if (rx[7] != 0x01) throw std::runtime_error("RH56 写角度未返回成功状态");
                    std::cout << "写角度应答成功，等待 5 秒（Ctrl+C 退出）" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    open = !open;
                }
#endif
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            result = 1;
        }

        // 7. 读取/初始化完成或上述错误路径执行断开。
        robot.disconnectFromRobot(ec);
        if (ec) {
            std::cerr << "disconnectFromRobot: " << ec.message() << " [code=" << ec.value() << "]\n";
            result = 1;
        }
        return result;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}

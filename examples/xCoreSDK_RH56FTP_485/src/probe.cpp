#include "window.hpp"
#include "runtime.hpp"
#include "sdk_robot.hpp"
#include <rokae/robot.h>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
void checkInterrupted() {
    if (interrupted) throw std::runtime_error("诊断已中断，开始清理");
}
std::string describe(const std::error_code& ec) {
    return std::to_string(ec.value()) + " (" + ec.category().name() + ": " + ec.message() + ")";
}
void check(const std::error_code& ec, const char* action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + describe(ec));
}
void waitPower(rokae::ArRobot& robot, rokae::PowerState expected) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
        std::error_code ec;
        const auto power = robot.powerState(ec); check(ec, "powerState readback");
        if (power == expected) return;
        if (expected == rokae::PowerState::on) checkInterrupted();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("电机状态读回未达到预期值 " + std::to_string(int(expected)));
}
}
int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("xCoreSDK_RH56FTP_485");
    QCommandLineParser parser;
    parser.setApplicationDescription("AR5 SDK 485 read probe: start idle with motors OFF; optional --power-on tests powered motors without RT or motion commands. Close other robot clients first.");
    parser.addHelpOption();
    parser.addOption({"config", "Configuration INI", "path", QCoreApplication::applicationDirPath() + "/config.ini"});
    parser.addOption({"side", "Arm: left, right or both (sequential)", "side", "both"});
    parser.addOption({"dry-run", "Print requests only; no SDK objects or connections"});
    parser.addOption({"power-on", "Temporarily enable motors in the current mode, wait 2 s, read hand, then power OFF"});
    parser.addOption({"init-485", "Configure 115200/8N1 only, then disconnect; restart AR controller before reading"});
    parser.addOption({"full-angle", "Read all six angles in one transaction (9-byte request, 20-byte reply; default, AR SDK required)"});
    parser.addOption({"three-angle", "Legacy comparison: read only the first three angles (14-byte reply)"});
    parser.process(app);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    try {
        const auto config = demo::loadConfig(parser.value("config"));
        const auto side = parser.value("side");
        if (parser.isSet("full-angle") && parser.isSet("three-angle"))
            throw std::runtime_error("--full-angle cannot be combined with --three-angle");
        const bool fullAngle = !parser.isSet("three-angle");
        const size_t dataBytes = fullAngle ? 12 : 6;
        const size_t replyBytes = dataBytes + 8;
        const bool init485 = parser.isSet("init-485");
        if (init485 && (parser.isSet("power-on") || parser.isSet("full-angle") || parser.isSet("three-angle")))
            throw std::runtime_error("--init-485 cannot be combined with --power-on, --full-angle or --three-angle");
        const bool powerOn = parser.isSet("power-on");
        if (side != "left" && side != "right" && side != "both")
            throw std::runtime_error("--side must be left, right or both");
        if (parser.isSet("dry-run")) {
            for (int i = 0; i < 2; ++i) {
                if ((side == "left" && i != 0) || (side == "right" && i != 1)) continue;
                std::cout << (i == 0 ? "left" : "right") << " " << config.arms[i].robotIp;
                if (init485) {
                    std::cout << " xPanelRS485Init(115200,none,stopbit1) -> setxPanelRS485Overtime(500) -> disconnect; restart required (dry-run; no connection)\n";
                } else {
                    std::cout << " current RS485 settings/timeout; TX "
                        << rh56::hex(rh56::request(config.arms[i].handId, 0x11, rh56::angleAct, {uint8_t(dataBytes)}))
                        << " expected_rx=" << replyBytes << " mode=" << (powerOn ? "current-mode+motors-on -> read -> motors-off" : "motors-off")
                        << " (dry-run; no connection)\n";
                }
            }
            return 0;
        }
        const auto runtime = demo::prepareSdkRuntime();
        std::ofstream file((runtime + "/probe.log").toStdString());
        if (!file) throw std::runtime_error("Cannot create probe.log");
        const auto log = [&](const std::string& message) {
            const auto line = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz ").toStdString() + message;
            std::cout << line << std::endl;
            file << line << std::endl;
        };
        log("日志目录：" + runtime.toStdString());
        log(init485 ? "仅初始化 485 为 115200/8N1；断开后请重启控制器，不上电、不收发手部帧。" : powerOn ?
            "电机上电对照：从下电/静止开始，保持当前控制模式，上电并读回确认，等待 2 秒后只读手部，结束请求下电。不切换模式、不发送运动指令、不配置 xPanel。请先关闭其他机器人客户端。" :
            "仅在机械臂静止且电机已下电时读取手部；不设置模式/电源、不启动 RT、不写手部寄存器。请先关闭其他机器人客户端。");
        bool success = true;
        for (int i = 0; i < 2; ++i) {
            if (interrupted) { success = false; break; }
            if ((side == "left" && i != 0) || (side == "right" && i != 1)) continue;
            const std::string label = i == 0 ? "左臂/手：" : "右臂/手：";
            const auto& cfg = config.arms[i];
            auto robotOwner = demo::makeSdkRobot();
            auto& robot = *robotOwner;
            bool connected = false;
            bool powerAttempted = false;
            bool cleanupFailed = false;
            const auto cleanup = [&] {
                const auto attempt = [&](const char* name, auto action) {
                    try { action(); }
                    catch (const std::exception& e) {
                        cleanupFailed = true; success = false;
                        log(label + "清理失败 " + name + ": " + e.what());
                    }
                };
                if (powerAttempted) {
                    powerAttempted = false;
                    attempt("电机下电", [&] {
                        std::error_code ec;
                        robot.setPowerState(false, ec); check(ec, "setPowerState(false)");
                        waitPower(robot, rokae::PowerState::off);
                        log(label + "已读回确认电机下电 powerState=1");
                    });
                }
            };
            try {
                robot.connectToRobot(cfg.robotIp);
                connected = true;
                std::error_code ec;
                const auto info = robot.robotInfo(ec); check(ec, "robotInfo");
                log(label + "IP=" + cfg.robotIp + ", 型号=" + info.type + ", 控制器=" + info.version + ", SDK=" + robot.sdkVersion());
                const auto power = robot.powerState(ec); check(ec, "powerState");
                const auto state = robot.operationState(ec); check(ec, "operationState");
                log(label + "powerState=" + std::to_string(int(power)) + ", operationState=" + std::to_string(int(state)));
                if (power != rokae::PowerState::off || state != rokae::OperationState::idle)
                    throw std::runtime_error("拒绝开始：需电机初始已下电且机器人静止（off=1, idle=0），请停止其他控制程序");
                if (init485) {
                    robot.xPanelRS485Init(rokae::xPanelCommOpt::Rs485Baud::baud115200,
                        rokae::xPanelCommOpt::CheckBit::none, rokae::xPanelCommOpt::StopBit::stopbit1, ec);
                    check(ec, "xPanelRS485Init(115200, none, stopbit1)");
                    robot.setxPanelRS485Overtime(500, ec); check(ec, "setxPanelRS485Overtime(500)");
                    log(label + "初始化请求成功：115200/8N1，超时设置为 500 ms；请重启该侧控制器后再读取手部");
                } else {
                    checkInterrupted();
                    log(label + "沿用控制器当前 485 参数和超时");
                    if (powerOn) {
                        checkInterrupted();
                        powerAttempted = true; // Clean up even if the SDK reports a partially applied failure.
                        robot.setPowerState(true, ec); check(ec, "setPowerState(true)");
                        waitPower(robot, rokae::PowerState::on);
                        log(label + "当前模式下电机上电已读回确认 powerState=0；等待 2 秒供手部启动（不代表末端电压已验证）");
                        for (int tick = 0; tick < 40; ++tick) {
                            checkInterrupted();
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        }
                        const auto readPower = robot.powerState(ec); check(ec, "powerState before read");
                        const auto readState = robot.operationState(ec); check(ec, "operationState before read");
                        log(label + "读取前 powerState=" + std::to_string(int(readPower)) + ", operationState=" + std::to_string(int(readState)));
                        if (readPower != rokae::PowerState::on || readState != rokae::OperationState::idle)
                            throw std::runtime_error("拒绝读取：上电测试要求电机上电且机械臂静止");
                    }
                    checkInterrupted();
                    const auto tx = rh56::request(cfg.handId, 0x11, rh56::angleAct, {uint8_t(dataBytes)});
                    rh56::Bytes rx(replyBytes);
                    const auto began = std::chrono::steady_clock::now();
                    robot.XPRS485SendData(int(tx.size()), int(rx.size()), tx, rx, ec);
                    const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
                    log(label + "TX " + rh56::hex(tx) + " | RX buffer " + rh56::hex(rx) +
                        " | expected_rx=" + std::to_string(replyBytes) + ", buffer_size=" + std::to_string(rx.size()) + ", ec=" + describe(ec) + ", elapsed_ms=" + std::to_string(ms));
                    check(ec, "XPRS485SendData (失败时 RX 缓冲不代表有效应答)");
                    const auto values = rh56::response(tx, rx, dataBytes);
                    std::string angles;
                    for (size_t j = 0; j < dataBytes; j += 2) {
                        const int angle = int(values[j]) | (int(values[j + 1]) << 8);
                        if (angle > 1000) throw std::runtime_error("RH56 angle out of range");
                        if (j) angles += ",";
                        angles += std::to_string(angle);
                    }
                    log(label + "SDK 读取及 RH56 帧校验成功，" + (fullAngle ? "六" : "前三") + "通道角度=" + angles);
                }
            } catch (const std::exception& e) {
                success = false; log(label + e.what());
                cleanup();
                if (connected) {
                    std::error_code ec;
                    const auto entries = robot.queryControllerLog(10, {}, ec);
                    if (ec) log(label + "queryControllerLog: " + describe(ec));
                    for (const auto& entry : entries)
                        log(label + "控制器时间 " + entry.timestamp + " [" + std::to_string(entry.id) + "] " + entry.content + "；" + entry.repair);
                }
            }
            cleanup();
            if (connected) {
                std::error_code ec;
                robot.disconnectFromRobot(ec);
                log(label + "disconnect: " + describe(ec));
                if (ec) success = false;
            }
            if (cleanupFailed) { log("清理失败，停止后续机械臂测试，请核实电机状态。"); break; }
        }
        log(success ? (init485 ? "初始化完成，请重启控制器后再测试通信。" : "双侧/所选侧读取成功。") : "诊断未全部通过，详见本次 probe.log。");
        return success ? 0 : 2;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

#include "backend.hpp"
#include "sdk_robot.hpp"
#include <rokae/robot.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>

namespace demo {
using namespace std::chrono_literals;
long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
namespace {
class SessionStopped : public std::runtime_error {
public:
    explicit SessionStopped(const char* message = "会话停止") : std::runtime_error(message) {}
};
std::string errorText(const std::error_code& ec) {
    return ec.message() + " [code=" + std::to_string(ec.value()) + ", category=" + ec.category().name() + "]";
}
void check(const std::error_code& ec, const char* action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + errorText(ec));
}
void validate(const Config& c) {
    if (c.pollMs < 100 || c.pollMs > 10000)
        throw std::invalid_argument("poll_ms must be 100..10000");
    for (const auto& a : c.arms)
        if (a.robotIp.empty() || a.handId < 1 || a.handId > 254)
            throw std::invalid_argument("Invalid robot IP or hand ID (1..254)");
    if (c.arms[0].robotIp == c.arms[1].robotIp)
        throw std::invalid_argument("Left/right robot IP must differ");
}
}
struct Backend::Impl {
    struct Arm {
        ArmState state;
        std::deque<Command> queue;
        std::thread worker;
        std::atomic<bool> done{true};
        std::atomic<bool> stopping{false};
        std::atomic<long long> transaction{0};
    };
    mutable std::mutex mutex;
    std::array<Arm, 2> arms;
    std::vector<std::string> logs;
    std::thread monitor;
    std::atomic<bool> stopping{false}, active{false};
    Config config;

    void log(int i, const std::string& s) {
        std::lock_guard<std::mutex> lock(mutex);
        if (logs.size() >= 400) logs.erase(logs.begin(), logs.begin() + 100);
        logs.push_back(std::string(i == 0 ? "左臂/手：" : "右臂/手：") + s);
    }
    void fail(int i, const std::string& message) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            arms[i].stopping = true; // Only this arm stops; the other worker keeps running.
            arms[i].queue.clear();
            arms[i].state.phase = Phase::Failed;
            arms[i].state.handReady = false;
            if (arms[i].state.error.empty()) arms[i].state.error = message;
        }
        log(i, message);
    }
    void recover(int i, const std::string& message) {
        bool changed = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            auto& a = arms[i];
            if (stopping || a.stopping || a.done) return;
            changed = a.state.phase != Phase::Recovering || a.state.error != message;
            a.state.phase = Phase::Recovering;
            a.state.handReady = false;
            a.state.error = message;
            a.queue.clear();
        }
        if (changed) log(i, message + "；暂停本侧下发，保留连接和上电，等待自动读取恢复");
    }
    void cancelled(int i) const {
        if (stopping || arms[i].stopping) throw SessionStopped();
    }
    bool readyLocked(int mask = 3) const {
        if (!active || stopping) return false;
        for (int i = 0; i < 2; ++i) if (mask & (1 << i)) {
            const auto& a = arms[i];
            if (a.stopping || !a.state.handReady || a.state.phase != Phase::Connected) return false;
        }
        return true;
    }
    void run(int i) {
        auto& a = arms[i];
        const auto& cfg = config.arms[i];
        std::unique_ptr<rokae::ArRobot> robot;
        bool connected = false, powerAttempted = false;
        bool cleanupFailed = false;
        const auto cleanup = [&](const char* name, auto operation) {
            try { operation(); }
            catch (const std::exception& e) { cleanupFailed = true; fail(i, std::string(name) + ": " + e.what()); }
            catch (...) { cleanupFailed = true; fail(i, std::string(name) + ": unknown error"); }
        };
        try {
            cancelled(i);
            if (!config.offline) {
                robot = makeSdkRobot(cfg.robotIp);
                connected = true;
                std::error_code ec;
                const auto info = robot->robotInfo(ec);
                if (ec) log(i, "robotInfo: " + errorText(ec));
                else log(i, "机器人 IP=" + cfg.robotIp + ", 型号=" + info.type +
                    ", 控制器=" + info.version + ", 轴数=" + std::to_string(info.joint_num) +
                    ", SDK=" + robot->sdkVersion());
                cancelled(i);
                if (config.init485) {
                    robot->xPanelRS485Init(rokae::xPanelCommOpt::Rs485Baud::baud115200,
                        rokae::xPanelCommOpt::CheckBit::none, rokae::xPanelCommOpt::StopBit::stopbit1, ec);
                    check(ec, "xPanelRS485Init(115200, none, stopbit1)");
                    robot->setxPanelRS485Overtime(500, ec); check(ec, "setxPanelRS485Overtime(500)");
                    throw SessionStopped("485 初始化请求成功：115200/8N1，超时设置为 500 ms；请重启该侧控制器后再进行手部通信。本次未请求电机上电。");
                }
                log(i, "沿用控制器当前 485 参数和超时；串口参数应已初始化并在重启后生效");
                powerAttempted = true; // Also clean up a partially applied power-on failure.
                robot->setPowerState(true, ec); check(ec, "setPowerState(true)");
                log(i, "电机上电接口返回成功；沿用当前控制模式、末端供电和串口配置，开始 485 测试");
            } else {
                log(i, "离线演示：不连接机器人，不代表真机通信已验证");
            }
            cancelled(i);
            if (config.init485) throw SessionStopped("离线初始化演示结束：未连接设备或写入参数");
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (a.state.phase != Phase::Failed) a.state.phase = Phase::Connected;
            }
            log(i, "开始读取手部状态");
            rh56::Hand hand(cfg.handId, [&](const rh56::Bytes& tx, size_t receive) {
                cancelled(i);
                if (tx[4] == 0x12) {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (!readyLocked(1 << i)) throw std::runtime_error("恢复期间已取消参数下发");
                }
                if (tx.size() > 64 || receive > 64) throw std::runtime_error("AR xPanel frame exceeds 64 bytes");
                rh56::Bytes rx(receive);
                std::error_code ec;
                const auto began = std::chrono::steady_clock::now();
                a.transaction = nowMs();
                try { robot->XPRS485SendData(int(tx.size()), int(rx.size()), tx, rx, ec); }
                catch (...) { a.transaction = 0; throw; }
                a.transaction = 0;
                const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    a.state.transferMs = elapsed;
                }
                log(i, "TX " + rh56::hex(tx) + " | " + (ec ? "RX 缓冲（SDK 失败，不视为有效应答） " : "RX ") +
                    rh56::hex(rx) + " | send=" + std::to_string(tx.size()) +
                    ", expected_rx=" + std::to_string(receive) + ", buffer_size=" + std::to_string(rx.size()) +
                    ", ec=" + std::to_string(ec.value()) + " (" + std::to_string(elapsed) + " ms)");
                check(ec, "XPRS485SendData");
                if (elapsed > 3000) throw std::runtime_error("485 SDK transaction exceeded 3 s");
                cancelled(i);
                return rx;
            });
            ArmState handState;
            handState.angles.fill(1000); handState.speeds.fill(200); handState.limits.fill(300);
            auto read = [&](bool settings) {
                if (!config.offline) {
                    handState.angles = hand.read(rh56::angleAct);
                    handState.forces = hand.read(rh56::forceAct, true);
                    handState.errors = hand.errors();
                    if (settings) {
                        handState.speeds = hand.read(rh56::speedSet);
                        handState.limits = hand.read(rh56::forceSet);
                    }
                }
                rh56::validate(rh56::angleSet, handState.angles);
                rh56::validate(rh56::speedSet, handState.speeds);
                rh56::validate(rh56::forceSet, handState.limits);
                const bool fault = std::any_of(handState.errors.begin(), handState.errors.end(), [](int v) { return v != 0; });
                bool recovered = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    cancelled(i); // Do not restore readiness after an explicit stop.
                    a.state.angles = handState.angles; a.state.forces = handState.forces;
                    a.state.errors = handState.errors; a.state.speeds = handState.speeds; a.state.limits = handState.limits;
                    a.state.feedbackMs = nowMs();
                    a.state.handReady = !fault;
                    if (!fault) {
                        recovered = a.state.phase == Phase::Recovering;
                        a.state.phase = Phase::Connected;
                        a.state.error.clear();
                    }
                }
                if (fault)
                    recover(i, "机械手报告执行器故障，详见六通道错误码");
                else if (recovered)
                    log(i, "手部通信恢复且六通道错误码为零，可重新下发；未重放旧命令");
                return !fault;
            };
            bool settingsNeeded = true;
            auto nextRead = nowMs();
            while (!stopping && !a.stopping) {
                Command cmd; bool pending = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (readyLocked(1 << i) && !a.queue.empty()) {
                        cmd = a.queue.front(); a.queue.pop_front(); pending = true;
                    }
                }
                cancelled(i);
                if (pending || nowMs() >= nextRead) {
                    try {
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            settingsNeeded = settingsNeeded || a.state.phase == Phase::Recovering;
                        }
                        if (pending && cmd.action == Action::Apply) {
                            settingsNeeded = true;
                            if (!config.offline) {
                                hand.write(rh56::speedSet, cmd.speeds);
                                hand.write(rh56::forceSet, cmd.limits);
                                hand.write(rh56::angleSet, cmd.angles);
                            } else {
                                handState.angles = cmd.angles; handState.speeds = cmd.speeds; handState.limits = cmd.limits;
                            }
                            log(i, "速度、力阈值、角度已下发（每类参数各一帧六通道）");
                        }
                        settingsNeeded = !read(settingsNeeded);
                    } catch (const SessionStopped&) {
                        throw;
                    } catch (const std::exception& e) {
                        settingsNeeded = true;
                        recover(i, e.what());
                    }
                    nextRead = nowMs() + config.pollMs;
                }
                std::this_thread::sleep_for(5ms);
            }
        } catch (const SessionStopped& e) {
            log(i, e.what());
        } catch (const std::exception& e) {
            // Preserve each arm's original error even if the other arm failed first.
            fail(i, e.what());
        } catch (...) { fail(i, "Unknown worker error"); }
        {
            std::lock_guard<std::mutex> lock(mutex);
            a.state.handReady = false; a.queue.clear();
            if (a.state.phase != Phase::Failed) a.state.phase = Phase::Stopping;
        }
        if (connected) {
            std::error_code ec;
            if (powerAttempted) cleanup("power off", [&] {
                robot->setPowerState(false, ec); check(ec, "setPowerState(false)");
                log(i, "电机下电接口返回成功");
            });
            cleanup("disconnect", [&] { robot->disconnectFromRobot(ec); check(ec, "disconnectFromRobot"); });
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (a.state.phase != Phase::Failed) a.state.phase = Phase::Idle;
        }
        log(i, config.offline ? "离线会话结束" : cleanupFailed ?
            "会话已结束，但下电或断开失败，请查看上述错误并核实电机状态" : "会话清理完成");
        a.done = true;
    }
    void watch() {
        while (!arms[0].done || !arms[1].done) {
            for (int i = 0; i < 2; ++i) {
                auto& a = arms[i];
                const auto began = a.transaction.load();
                if (!stopping && !a.stopping && !a.done && began && nowMs() - began > 3000)
                    recover(i, "485 SDK 调用超过 3 s，等待当前调用返回");
                long long handFeedback = 0;
                bool handReady = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    handFeedback = a.state.feedbackMs;
                    handReady = a.state.handReady;
                }
                if (!stopping && !a.stopping && !a.done && handReady && handFeedback && nowMs() - handFeedback > std::max(3000, 2 * config.pollMs))
                    recover(i, "机械手反馈过期");
            }
            std::this_thread::sleep_for(10ms);
        }
        for (auto& a : arms) if (a.worker.joinable()) a.worker.join();
        active = false;
    }
};
Backend::Backend() : impl_(std::make_unique<Impl>()) {}
Backend::~Backend() {
    stop();
    if (impl_->monitor.joinable()) impl_->monitor.join();
}
void Backend::start(const Config& config) {
    validate(config);
    auto& s = *impl_;
    if (s.active) throw std::runtime_error("Session already active");
    if (s.monitor.joinable()) s.monitor.join();
    s.config = config; s.stopping = false;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        for (auto& a : s.arms) {
            a.state = {}; a.state.phase = Phase::Starting; a.queue.clear();
            a.done = false; a.stopping = false; a.transaction = 0;
        }
    }
    s.active = true;
    try {
        for (int i = 0; i < 2; ++i) s.arms[i].worker = std::thread([&s, i] { s.run(i); });
        s.monitor = std::thread([&s] { s.watch(); });
    } catch (...) {
        s.stopping = true;
        for (auto& a : s.arms) { if (a.worker.joinable()) a.worker.join(); a.done = true; }
        s.active = false;
        throw;
    }
}
void Backend::stop() { impl_->stopping = true; }
bool Backend::submit(int mask, const Command& command) {
    if (mask < 1 || mask > 3) throw std::invalid_argument("Invalid arm mask");
    if (command.action == Action::Apply) {
        rh56::validate(rh56::angleSet, command.angles);
        rh56::validate(rh56::speedSet, command.speeds);
        rh56::validate(rh56::forceSet, command.limits);
    }
    auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    if (!s.readyLocked(mask)) return false;
    for (int i = 0; i < 2; ++i) if ((mask & (1 << i)) && s.arms[i].queue.size() >= 8) return false;
    for (int i = 0; i < 2; ++i) if (mask & (1 << i)) s.arms[i].queue.push_back(command);
    return true;
}
bool Backend::submitBoth(const std::array<Command, 2>& commands) {
    for (const auto& cmd : commands) if (cmd.action == Action::Apply) {
        rh56::validate(rh56::angleSet, cmd.angles);
        rh56::validate(rh56::speedSet, cmd.speeds);
        rh56::validate(rh56::forceSet, cmd.limits);
    }
    auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    if (!s.readyLocked() || s.arms[0].queue.size() >= 8 || s.arms[1].queue.size() >= 8) return false;
    for (size_t i = 0; i < 2; ++i) s.arms[i].queue.push_back(commands[i]);
    return true;
}
Snapshot Backend::snapshot() const {
    auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    Snapshot out;
    out.active = s.active; out.stopping = s.stopping; out.ready = s.readyLocked();
    for (size_t i = 0; i < 2; ++i) out.arms[i] = s.arms[i].state;
    return out;
}
std::vector<std::string> Backend::takeLogs() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<std::string> out; out.swap(impl_->logs); return out;
}
} // namespace demo

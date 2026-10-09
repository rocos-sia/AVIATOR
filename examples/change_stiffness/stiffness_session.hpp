#pragma once

#include <rokae/robot.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace stiffness {
using Joints = std::array<double, 7>;
inline constexpr Joints defaults{500, 500, 500, 500, 50, 50, 50};
inline constexpr Joints limits{3000, 3000, 3000, 3000, 300, 300, 300};

inline void validate(const Joints &values) {
    for (unsigned i = 0; i < values.size(); ++i)
        if (!std::isfinite(values[i]) || values[i] < 0 || values[i] > limits[i])
            throw std::runtime_error("J" + std::to_string(i + 1) + " stiffness out of range");
}

// 全部 SDK 生命周期操作均由同一个普通工作线程调用。实时回调只读目标及原子变量。
class Session {
public:
    Session(std::atomic<bool> &stop, bool offline,
            std::function<void(const std::string &)> log)
        : stop_(stop), offline_(offline), log_(std::move(log)) {}
    ~Session() { shutdown(); }

    void start(const std::string &robotIp, const std::string &localIp, const Joints &values) {
        validate(values);
        cancelled();
        if (offline_) {
            pause();
            log_("离线演示：未连接机器人");
            return;
        }
        robot_ = std::make_unique<rokae::xMateErProRobot>();
        robot_->connectToRobot(robotIp, localIp);
        connected_ = true;
        std::error_code ec;
        robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        check(ec, "setOperateMode");
        enterRt(values);
        log_("已读取启动位置，正在原地保持");
    }

    void apply(const Joints &values) {
        validate(values);
        cancelled();
        if (offline_) {
            pause();
            log_("离线演示：保持 RT 模式，模拟暂停运动 → 设置刚度 → 恢复保持");
            return;
        }
        pauseHold();
        cancelled();
        startHold(values, false); // 复用 RT 控制器及启动时的固定关节角。
        log_("刚度已设置，已恢复原目标位置保持");
    }

    const Joints &target() const { return target_; }

    void checkLoop() {
        if (offline_ || !looping_ || stop_.load()) return;
        // 异步 startLoop 的故障通常在 stopLoop 才抛出；回调停滞时主动进入清理。
        if (nowMs() - heartbeat_.load() > 1000)
            throw std::runtime_error("实时回调超过 1 秒未更新，停止控制");
    }

    // 每项独立清理；返回错误文本，交给 UI 显示。重复调用无副作用。
    std::string shutdown() noexcept {
        std::string errors;
        const auto clean = [&](const char *name, auto fn) {
            try { fn(); }
            catch (const std::exception &e) { errors += std::string(name) + ": " + e.what() + "\n"; }
            catch (...) { errors += std::string(name) + ": unknown error\n"; }
        };
        if (looping_) { clean("stopLoop", [&] { rt_->stopLoop(); }); looping_ = false; }
        if (moving_) { clean("stopMove", [&] { rt_->stopMove(); }); moving_ = false; }
        if (receiving_) { robot_->stopReceiveRobotState(); receiving_ = false; }
        std::error_code ec;
        if (powered_) {
            clean("power off", [&] { robot_->setPowerState(false, ec); check(ec, "setPowerState(false)"); });
            powered_ = false;
        }
        if (connected_) {
            clean("NrtCommand", [&] {
                robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
                check(ec, "setMotionControlMode(NrtCommand)");
            });
            clean("disconnect", [&] { robot_->disconnectFromRobot(ec); check(ec, "disconnectFromRobot"); });
            connected_ = false;
        }
        rt_.reset();
        robot_.reset();
        return errors;
    }

private:
    static void check(const std::error_code &ec, const char *action) {
        if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message());
    }
    static long long nowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void cancelled() const {
        if (stop_.load()) throw std::runtime_error("操作已取消");
    }
    void pause() const {
        for (int i = 0; i < 15; ++i) {
            cancelled();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        cancelled();
    }
    void pauseHold() {
        rt_->stopLoop(); looping_ = false;
        rt_->stopMove(); moving_ = false;
        robot_->stopReceiveRobotState(); receiving_ = false;
        log_("已暂停运动和状态接收，保持 RT 模式并复用控制器");
    }
    void enterRt(const Joints &values) {
        cancelled();
        std::error_code ec;
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        check(ec, "setMotionControlMode(RtCommand)");
        cancelled();
        powered_ = true;
        robot_->setPowerState(true, ec);
        check(ec, "setPowerState(true)");
        cancelled();
        rt_ = robot_->getRtMotionController().lock();
        if (!rt_) throw std::runtime_error("No realtime controller");
        startHold(values, true);
    }
    void startHold(const Joints &values, bool readTarget) {
        cancelled();
        std::error_code ec;
        rt_->setJointImpedance(values, ec);
        check(ec, "setJointImpedance");
        receiving_ = true;
        robot_->startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m});
        if (readTarget) {
            target_ = robot_->jointPos(ec);
            check(ec, "jointPos");
            for (double q : target_)
                if (!std::isfinite(q)) throw std::runtime_error("Invalid joint position");
        }
        std::function<rokae::JointPosition()> callback = [this] {
            rokae::JointPosition command(7);
            for (unsigned i = 0; i < target_.size(); ++i) command.joints[i] = target_[i];
            heartbeat_.store(nowMs());
            if (stop_.load()) command.setFinished();
            return command;
        };
        rt_->setControlLoop(callback);
        cancelled();
        moving_ = true;
        rt_->startMove(rokae::RtControllerMode::jointImpedance);
        heartbeat_.store(nowMs());
        looping_ = true;
        rt_->startLoop(false);
    }

    std::atomic<bool> &stop_;
    bool offline_;
    std::function<void(const std::string &)> log_;
    std::unique_ptr<rokae::xMateErProRobot> robot_;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt_;
    Joints target_{};
    std::atomic<long long> heartbeat_{0};
    bool connected_ = false, powered_ = false, receiving_ = false, moving_ = false, looping_ = false;
};
} // namespace stiffness

#include "Logger.hpp"
#include "rokae_sdk.hpp"
#include <rokae/utility.h>
#include <rokae/robot.h>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <thread>
namespace aviator {
namespace {
void check(const std::error_code &ec, const char *action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message() +
        " [" + ec.category().name() + ":" + std::to_string(ec.value()) + "]");
}
}
class RokaeArm::Impl {
public:
    rokae::xMateErProRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt;
    std::array<double, 7> stiffness{};
    std::string endpoint;
    mutable std::mutex error_mutex;
    std::exception_ptr callback_error;
    bool powered = false, receiving = false, moving = false, looping = false, prepared = false;
    static constexpr int rt_priority = 80;
    std::atomic<unsigned long long> callbacks{0};
    std::atomic<double> max_gap_ms{0}, max_callback_us{0};
    std::atomic<int> thread_policy{-1}, thread_priority{-1};
    bool timing_reported = false;
    std::chrono::steady_clock::time_point previous_callback{};
    std::string timing() const {
        // return "RT callbacks=" + std::to_string(callbacks.load()) +
        //     " max_gap_ms=" + std::to_string(max_gap_ms.load()) +
        //     " max_callback_us=" + std::to_string(max_callback_us.load()) +
        //     " scheduling_policy=" + std::to_string(thread_policy.load()) +
        //     " priority=" + std::to_string(thread_priority.load()) +
        //     " requested_FIFO_priority=" + std::to_string(rt_priority);
        return "";
    }
    // 只在 startMove 被拒绝、实时回调尚未启动时查询，保留清理前的实际状态。
    void printStartFailure() {
        const std::string prefix = "[Rokae " + endpoint + "] ";
        auto printState = [&](const char *name, auto state, const std::error_code &ec,
                              std::initializer_list<const char *> labels) {
            if (ec) {
                aviator::Logger::error("{}{} query failed: {} [{}:{}]", prefix, name,
                                       ec.message(), ec.category().name(), ec.value());
            } else {
                const int value = static_cast<int>(state);
                const char* label = value >= 0 && static_cast<size_t>(value) < labels.size()
                    ? *(labels.begin() + value) : "unknown";
                aviator::Logger::error("{}{}={}({})", prefix, name, label, value);
            }
        };
        std::error_code ec;
        const auto power = robot.powerState(ec);
        printState("powerState", power, ec, {"on", "off", "estop", "gstop"});
        ec.clear();
        const auto mode = robot.operateMode(ec);
        printState("operateMode", mode, ec, {"manual", "automatic"});
        ec.clear();
        const auto state = robot.operationState(ec);
        printState("operationState", state, ec,
                   {"idle", "jog", "rtControlling", "drag", "rlProgram", "demo",
                    "dynamicIdentify", "frictionIdentify", "loadIdentify", "moving", "jogging"});
        ec.clear();
        const auto logs = robot.queryControllerLog(10, {rokae::LogInfo::warning, rokae::LogInfo::error}, ec);
        if (ec) {
            aviator::Logger::error("{}queryControllerLog failed: {} [{}:{}]",
                prefix, ec.message(), ec.category().name(), ec.value());
            return;
        }
        aviator::Logger::error("{}Recent controller warnings/errors (may include older events): {}",
            prefix, logs.size());
        for (const auto &log : logs)
            aviator::Logger::error("{}controller_log id={} time={} content={} repair={}",
                prefix, log.id, log.timestamp, log.content, log.repair);
    }
    void stop() {
        std::exception_ptr error;
        auto attempt = [&](const char *action, auto fn) {
            try { fn(); }
            catch (const std::exception &e) {
                aviator::Logger::error("[Rokae {}] {}: {}", endpoint, action, e.what());
                if (!error) error = std::current_exception();
            } catch (...) {
                aviator::Logger::error("[Rokae {}] {}: unknown exception", endpoint, action);
                if (!error) error = std::current_exception();
            }
        };
        if (looping) attempt("stopLoop (includes asynchronous SDK errors)", [&] { rt->stopLoop(); looping = false; });
        if (moving) attempt("stopMove", [&] { rt->stopMove(); moving = false; });
        if (receiving) { robot.stopReceiveRobotState(); receiving = false; }
        if (powered) attempt("power off", [&] {
            std::error_code ec;
            robot.setPowerState(false, ec);
            check(ec, "setPowerState(false)");
            powered = false;
        });
        prepared = false;
        if (!timing_reported && callbacks.load() > 0) {
            timing_reported = true;
            // 只在停止周期线程后输出；不在实时回调中打印。
            aviator::Logger::error("[Rokae {}] {}", endpoint, timing());
        }
        if (error) std::rethrow_exception(error);
    }
    ~Impl() {
        try { stop(); } catch (...) {}
        std::error_code ec;
        robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
        robot.disconnectFromRobot(ec);
    }
};
RokaeArm::RokaeArm(const std::string &ip, const std::string &local_ip,
                   const std::array<double, 16> &tool, const std::array<double, 7> &stiffness) : impl_(std::make_unique<Impl>()) {
    auto &s = *impl_;
    s.endpoint = "robot=" + ip + " local=" + local_ip;
    s.stiffness = stiffness;
    s.robot.connectToRobot(ip, local_ip);
    std::error_code ec;
    s.robot.setRtNetworkTolerance(20, ec);
    check(ec, "setRtNetworkTolerance");
    s.robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    check(ec, "setMotionControlMode");
    s.robot.setOperateMode(rokae::OperateMode::automatic, ec);
    check(ec, "setOperateMode");
    // s.robot.setRtNetworkTolerance(20, ec);
    // check(ec, "setRtNetworkTolerance");
    auto toolset = s.robot.toolset(ec);
    check(ec, "toolset");
    std::array<double, 6> tool_pose{};
    rokae::Utils::transArrayToPosture(tool, tool_pose);
    toolset.end = rokae::Frame(tool_pose);
    toolset.ref = rokae::Frame();
    // Retain the controller's calibrated load; grasp.json has no load calibration.
    s.robot.setToolset(toolset, ec);
    check(ec, "setToolset");
    s.rt = s.robot.getRtMotionController().lock();
    if (!s.rt) throw std::runtime_error("getRtMotionController returned null");
    s.rt->setEndEffectorFrame(tool, ec);
    check(ec, "setEndEffectorFrame");
}
RokaeArm::~RokaeArm() = default;
std::array<double, 7> RokaeArm::position() const {
    std::error_code ec;
    auto q = impl_->robot.jointPos(ec);
    check(ec, "jointPos");
    return q;
}
std::string RokaeArm::diagnostics() const {
    std::string result = "[Rokae " + impl_->endpoint + "] SDK hasMotionError=" +
        (impl_->rt && impl_->rt->hasMotionError() ? "true" : "false");
    std::exception_ptr error;
    { std::lock_guard<std::mutex> lock(impl_->error_mutex); error = impl_->callback_error; }
    if (error) {
        try { std::rethrow_exception(error); }
        catch (const std::exception &e) { result += std::string(" callback_error=") + e.what(); }
        catch (...) { result += " callback_error=unknown exception"; }
    } else result += " callback_error=none captured (SDK errors may be reported by stopLoop)";
    return result + " " + impl_->timing();
}
bool RokaeArm::motionFailed() const {
    std::lock_guard<std::mutex> lock(impl_->error_mutex);
    return bool(impl_->callback_error) || (impl_->rt && impl_->rt->hasMotionError());
}
void RokaeArm::prepare() {
    auto &s = *impl_;
    if (s.prepared) return;
    { std::lock_guard<std::mutex> lock(s.error_mutex); s.callback_error = nullptr; }
    s.callbacks = 0;
    s.max_gap_ms = 0;
    s.max_callback_us = 0;
    s.thread_policy = s.thread_priority = -1;
    s.previous_callback = {};
    s.timing_reported = false;
    const char *stage = "check realtime scheduling permission";
    try {
        // 在上电/startMove 前试验本进程的实时调度权限，避免启动运动后才发现权限不足。
        // 只改变这条短命探测线程；不修改主线程调度或系统配置。
        int scheduling_error = 0;
        std::thread probe([&] {
            sched_param parameters{};
            parameters.sched_priority = Impl::rt_priority;
            scheduling_error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
        });
        probe.join();
        if (scheduling_error)
            throw std::runtime_error("RT scheduling unavailable before power-on: " +
                std::error_code(scheduling_error, std::generic_category()).message() +
                "; SCHED_FIFO priority 80 requires CAP_SYS_NICE or an adequate rtprio limit");
        std::error_code ec;
        stage = "setPowerState(true)";
        s.powered = true; // 部分成功也必须尝试下电。
        s.robot.setPowerState(true, ec);
        check(ec, stage);
        stage = "startReceiveRobotState";
        s.receiving = true;
        s.robot.startReceiveRobotState(std::chrono::milliseconds(1),
            {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
             rokae::RtSupportedFields::tcpPose_m});
        stage = "setJointImpedance";
        s.rt->setJointImpedance(s.stiffness, ec);
        check(ec, stage);
        s.prepared = true;
    } catch (...) {
        aviator::Logger::info("[Rokae {}] prepare stage={}", s.endpoint, stage);
        auto error = std::current_exception();
        try { s.stop(); } catch (...) {}
        std::rethrow_exception(error);
    }
}

void RokaeArm::start(std::function<std::array<double, 7>(const RokaeSample &)> callback,
                     std::function<void(const std::array<double, 7> &)> initialize_target) {
    auto &s = *impl_;
    const char *stage = "prepare";
    try {
        prepare();
        stage = "setControlLoop";
        s.rt->setControlLoop(std::function<rokae::JointPosition()>([&s, callback, command = rokae::JointPosition(7)]() mutable {
          const auto began = std::chrono::steady_clock::now();
          try {
            if (s.callbacks.load(std::memory_order_relaxed) == 0) {
                int policy = -1;
                sched_param parameters{};
                const int rc = pthread_getschedparam(pthread_self(), &policy, &parameters);
                s.thread_policy = policy;
                s.thread_priority = parameters.sched_priority;
                if (rc || policy != SCHED_FIFO || parameters.sched_priority != Impl::rt_priority)
                    throw std::runtime_error("RT scheduling unavailable: expected SCHED_FIFO priority 80; "
                        "grant the process CAP_SYS_NICE or an adequate rtprio limit before running hardware");
            }
            if (s.previous_callback != std::chrono::steady_clock::time_point{}) {
                const double gap = std::chrono::duration<double, std::milli>(began - s.previous_callback).count();
                s.max_gap_ms.store(std::max(s.max_gap_ms.load(std::memory_order_relaxed), gap), std::memory_order_relaxed);
            }
            s.previous_callback = began;
            ++s.callbacks;
            RokaeSample sample;
            auto read = [&](const char *field, auto &value) {
                const int rc = s.robot.getStateData(field, value);
                if (rc != 0) throw std::runtime_error(std::string("getStateData(") + field + ") returned " + std::to_string(rc));
            };
            read(rokae::RtSupportedFields::jointPos_m, sample.position);
            read(rokae::RtSupportedFields::jointVel_m, sample.velocity);
            read(rokae::RtSupportedFields::tcpPose_m, sample.tcp);
            auto q = callback(sample);
            std::copy(q.begin(), q.end(), command.joints.begin());
            const double elapsed = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - began).count();
            s.max_callback_us.store(std::max(s.max_callback_us.load(std::memory_order_relaxed), elapsed), std::memory_order_relaxed);
            return command; // SDK 接口按值接收 JointPosition；复用填充缓存，返回仍由 SDK 类型管理。
          } catch (...) {
            std::lock_guard<std::mutex> lock(s.error_mutex);
            if (!s.callback_error) s.callback_error = std::current_exception();
            throw;
          }
        }), Impl::rt_priority, true);

        // 对齐官方 getCurrentJointPos：消费已排队的实时状态，再读取当前位置。
        // 此时只注册了回调，尚未 startLoop，不会与回调同时读取状态队列。
        stage = "getCurrentJointPos before startMove";
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        bool received = false;
        while (s.robot.updateRobotState(std::chrono::steady_clock::duration::zero())) {
            received = true;
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("Realtime state queue did not drain within 100 ms");
        }
        std::array<double, 7> initial_position{};
        const bool realtime_position = received &&
            s.robot.getStateData(rokae::RtSupportedFields::jointPos_m, initial_position) == 0;
        if (!realtime_position) {
            // 尚无实时位置时使用非实时查询；查询失败必须终止启动，不能返回默认零角度。
            std::error_code ec;
            initial_position = s.robot.jointPos(ec);
            check(ec, "jointPos startup fallback");
        }
        for (double q : initial_position)
            if (!std::isfinite(q)) throw std::runtime_error("Nonfinite startup joint position");
        initialize_target(initial_position);
        std::ostringstream position;
        for (double q : initial_position) position << ' ' << q;
        aviator::Logger::info("[Rokae {}] initial_position_rad source={}:{}", s.endpoint,
            realtime_position ? "jointPos_m" : "jointPos fallback", position.str());

        s.moving = true;
        stage = "startMove(jointImpedance)";
        s.rt->startMove(rokae::RtControllerMode::jointImpedance);
        s.looping = true;
        stage = "startLoop";
        s.rt->startLoop(false);
    } catch (...) {
        auto error = std::current_exception();
        try { std::rethrow_exception(error); }
        catch (const std::exception &e) { aviator::Logger::error("[Rokae {}] start stage={}: {}",
            s.endpoint, stage, e.what()); }
        catch (...) { aviator::Logger::error("[Rokae {}] start stage={}: unknown exception", s.endpoint, stage); }
        if (std::string(stage) == "startMove(jointImpedance)") {
            try { s.printStartFailure(); }
            catch (...) { aviator::Logger::error("[Rokae {}] Controller diagnostics unavailable", s.endpoint); }
        }
        try { s.stop(); } catch (...) {}
        std::rethrow_exception(error);
    }
}
void RokaeArm::stop() { impl_->stop(); }
}

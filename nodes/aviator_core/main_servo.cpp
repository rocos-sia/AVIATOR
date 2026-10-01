#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/Kinematics.hpp"
#include "RemoteLink.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <optional>
#include <thread>
#include <future>

static volatile std::sig_atomic_t interrupted = 0;
static void interrupt(int) { interrupted = 1; }

static void status(aviator::Aviator &robot) {
    const auto s = robot.GetStatus();
    std::cout << "state=" << robot.GetState() << " angle=" << s.angle
              << " displacement=" << s.displacement << " locked=" << s.locked
              << " error=" << s.motion_error << " fault=" << s.fault << std::endl;
}

int main(int argc, char **argv) {
    try {
        auto config = aviator::defaultSystemConfig();
        std::string gateway_session;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) config = argv[++i];
            else if (arg == "--gateway-session" && i + 1 < argc) gateway_session = argv[++i];
            else if (arg == "--help") {
                std::cout << "aviator_core_servo [--config <system.yaml>] [--gateway-session <ignored>]\n"
                             "Session pinning is disabled; publisher, freshness and sequence checks remain.\n"
                             "Enable, approach and lock, then follow flight_gateway JOYSTICK input.\n"
                             "roll -> +/-0.87266 rad; pitch [-1,0,1] -> [-0.170,-0.085,0] m; v=1.\n"
                             "Start bus, manipulator and flight_gateway first. Ctrl+C stops and disables.\n";
                return 0;
            } else throw std::runtime_error("Unknown or incomplete option: " + arg);
        }
        const auto settings = aviator::loadMotionConfig(config);
        aviator::InputPolicy policy;
        policy.publisher_id = "flight_gateway";
        policy.clock_id = aviator::local_clock_id();
        policy.source = "JOYSTICK";
        policy.allow_joystick_position_hold = true;
        policy.timeout_us = settings.origin_timeout_us;
        std::optional<aviator::InputGuard> input;
        auto link = std::make_unique<aviator::RemoteLink>(settings);
        auto *connection = link.get();
        aviator::Aviator robot(std::move(link), nullptr, nullptr, settings.robot.string());
        robot.init();
        const auto robot_config = YAML::LoadFile(settings.robot.string());
        const auto period = static_cast<uint64_t>(robot_config["servo_period"].as<double>(0.02) * 1e6);
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, interrupt);

        // 阻塞接近动作放在工作线程，主线程继续监督本地任务心跳。
        std::exception_ptr startup_error;
        std::atomic<bool> approached{false};
        std::thread startup([&] {
            try {
                if (interrupted) throw std::runtime_error("Startup interrupted");
                robot.enable();
                if (interrupted) throw std::runtime_error("Startup interrupted");
                std::cout << "DEMO approaching handles" << std::endl;
                robot.approachHandles();
                status(robot);
                std::cout << "robot.approachHandles() done" << std::endl;
                if (interrupted) throw std::runtime_error("Startup interrupted");
                robot.lockHandles();
                status(robot);
            } catch (...) { startup_error = std::current_exception(); }
            approached = true;
        });
        while (!approached) {
            connection->heartbeat();
            connection->report(robot.GetState());
            if (interrupted) robot.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        startup.join();

        int result = 0;
        try {
            if (startup_error) std::rethrow_exception(startup_error);
            // 接近完成后才订阅，避免排队执行接近期间的摇杆目标。
            zmq::context_t context(1);
            zmq::socket_t sub(context, zmq::socket_type::sub);
            aviator::configure(sub);
            aviator::subscribe(sub, "flight.command");
            sub.connect(settings.subscribe);
            aviator::ReceiveState receiver;
            double roll = 0, pitch = 0;
            uint64_t next_servo = 0, next_print = 0;
            bool was_fresh = false;
            std::string last_state, input_error;
            std::cout << "Waiting for flight.command from flight_gateway (no session authorization)" << std::endl;
            while (!interrupted) {
                connection->heartbeat();
                // 原始事件时间保持不变；POSITION_HOLD 显式使用设备检查时间判定时效。
                for (int i = 0; i < 64; ++i) {
                    aviator::WireMessage wire;
                    std::string error;
                    const auto received = aviator::receive(sub, receiver, wire, error);
                    if (received == aviator::ReceiveResult::empty) break;
                    aviator::Message message;
                    if (received != aviator::ReceiveResult::received ||
                        !aviator::decode(wire.topic, wire.payload, message, error)) continue;
                    if (!input) {
                        // 首条消息仍需通过来源、时钟、有效性和时效检查。
                        policy.session_id = message.header.session_id;
                        aviator::InputGuard candidate(policy);
                        if (!candidate.accept(message, aviator::monotonic_us(), error)) continue;
                        input = std::move(candidate);
                        std::cout << "Received valid flight_gateway input" << std::endl;
                    } else if (!input->accept(message, aviator::monotonic_us(), error)) {
                        if (message.header.publisher_id == policy.publisher_id)
                            input_error = error;
                        continue;
                    }
                    input_error.clear();
                    // decode 已拒绝非有限值/越界值；这里显式限定映射范围。
                    roll = std::clamp(message.body.at("control").at("roll").get<double>(), -1.0, 1.0);
                    pitch = std::clamp(message.body.at("control").at("pitch").get<double>(), -1.0, 1.0);
                }
                const auto now = aviator::monotonic_us();
                const bool fresh = input && !input->expired(now);
                if (fresh != was_fresh) {
                    if (fresh) std::cout << "Flight input: valid" << std::endl;
                    else std::cout << "Flight input: stale/invalid; stop updating ServoWheel ("
                                   << (input_error.empty() ? "gateway/device check timeout" : input_error)
                                   << ")" << std::endl;
                    was_fresh = fresh;
                }
                const auto state = robot.GetState();
                const auto current = robot.GetStatus();
                if (state != last_state) { status(robot); last_state = state; }
                if (state == "FAULT" || current.fault)
                    throw std::runtime_error("Servo fault: " + current.motion_error);
                connection->report(state, fresh ? "JOYSTICK" : "NONE");
                if (fresh && now >= next_servo &&
                    (state == "LOCKED" || (state == "SERVO" && current.motion_error.empty()))) {
                    const double angle = roll * 0.87266;
                    const double displacement = aviator::joystickWheelDisplacement(pitch);
                    robot.servoWheel(angle, displacement, 1.0);
                    next_servo = now + period; // 不补发错过的周期。
                    if (now >= next_print) {
                        std::cout << "Servo target angle=" << angle << " displacement=" << displacement
                                  << " v=1" << std::endl;
                        next_print = now + 100000;
                    }
                }
                // 无有效输入时不刷新目标，让 Servo 自身超时减速，保持软件锁定。
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        } catch (const std::exception &e) {
            std::cerr << "Execution failed: " << e.what() << std::endl;
            result = 1;
        }
        robot.stop();
        while (robot.GetState() == "SERVO") {
            connection->heartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        auto cleanup = std::async(std::launch::async, [&] {
            if (robot.GetState() == "LOCKED") robot.unlockHandles();
            robot.disable();
        });
        while (cleanup.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready)
            connection->heartbeat();
        cleanup.get();
        return result;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

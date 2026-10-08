#include "Logger.hpp"
#include "aviator/Aviator.hpp"
#include "aviator/CollisionChecker.hpp"
#include "aviator/Kinematics.hpp"
#include "RemoteLink.hpp"
#include "CameraWheelInput.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <thread>
#include <future>

static volatile std::sig_atomic_t interrupted = 0;
static void interrupt(int) { interrupted = 1; }

static void status(aviator::Aviator &robot) {
    const auto s = robot.GetStatus();
    aviator::Logger::info("state={} angle={} displacement={} locked={} error={} fault={}",
        robot.GetState(), s.angle, s.displacement, s.locked, s.motion_error, s.fault);
}

int main(int argc, char **argv) {
    try {
        auto config = aviator::defaultSystemConfig();
        std::string camera_id = "cockpit";
        uint64_t camera_timeout_us = 200000;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) config = argv[++i];
            else if (arg == "--camera-id" && i + 1 < argc) camera_id = argv[++i];
            else if (arg == "--camera-timeout-ms" && i + 1 < argc) {
                const std::string value = argv[++i];
                size_t used = 0;
                const auto ms = std::stoul(value, &used);
                if (used != value.size() || ms < 20 || ms > 1000)
                    throw std::runtime_error("camera timeout must be 20..1000 ms");
                camera_timeout_us = ms * 1000;
            } else if (arg == "--help") {
                aviator::Logger::info("aviator_core_camera_servo [--config system.yaml] [--camera-id cockpit]\n"
                    "  [--camera-timeout-ms 200]\n"
                    "Automatically enable, approach and lock, then follow camera.detection.steering_wheel.\n"
                    "Joint impedance uses robot.yaml stiffness; servoWheel speed ratio is 1.\n"
                    "angle=-theta_rad, displacement=-translation_along_axis_m-0.085; targets are clamped.\n"
                    "Start bus, manipulator, hand and camera first; run only one Core.\n"
                    "Stale/invalid input stops target refresh; Ctrl+C stops and disables.");
                return 0;
            } else throw std::runtime_error("Unknown or incomplete option: " + arg);
        }
        const auto settings = aviator::loadMotionConfig(config);
        if (camera_id.empty() || camera_id.size() > 128)
            throw std::runtime_error("invalid camera id");
        auto link = std::make_unique<aviator::RemoteLink>(settings);
        auto *connection = link.get();
        aviator::Aviator robot(std::move(link), nullptr, nullptr, settings.robot.string());
        robot.init();
        const auto robot_config = YAML::LoadFile(settings.robot.string());
        const double period_s = robot_config["servo_period"].as<double>(0.02);
        if (!std::isfinite(period_s) || period_s < .001 || period_s > 1)
            throw std::runtime_error("servo_period must be 0.001..1 seconds");
        const auto period = static_cast<uint64_t>(period_s * 1e6);
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
                aviator::Logger::info("DEMO approaching handles");
                robot.approachHandles();
                status(robot);
                aviator::Logger::info("robot.approachHandles() done");
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
            // 接近完成后才订阅，避免排队执行接近期间的相机目标。
            zmq::context_t context(1);
            zmq::socket_t sub(context, zmq::socket_type::sub);
            aviator::configure(sub);
            aviator::subscribe(sub, "camera.detection");
            sub.connect(settings.subscribe);
            aviator::ReceiveState receiver;
            aviator::CameraWheelInput input(camera_id, aviator::local_clock_id(),
                                           camera_timeout_us, aviator::monotonic_us());
            uint64_t next_servo = 0, next_print = 0;
            bool was_fresh = false;
            std::string last_state, input_error;
            aviator::Logger::info("Waiting for camera.detection: publisher=camera camera_id={} timeout={} ms",
                                  camera_id, camera_timeout_us / 1000);
            while (!interrupted) {
                connection->heartbeat();
                // Drain bounded input batches; preserve the camera acquisition timestamp.
                for (int i = 0; i < 64; ++i) {
                    aviator::WireMessage wire;
                    std::string error;
                    const auto received = aviator::receive(sub, receiver, wire, error);
                    if (received == aviator::ReceiveResult::empty) break;
                    aviator::Message message;
                    if (received != aviator::ReceiveResult::received ||
                        !aviator::decode(wire.topic, wire.payload, message, error)) continue;
                    input.accept(message, aviator::monotonic_us(), input_error);
                }
                const auto now = aviator::monotonic_us();
                const bool fresh = input.fresh(now);
                if (fresh != was_fresh) {
                    if (fresh) aviator::Logger::info("Camera input: valid");
                    else aviator::Logger::warn("Camera input: stale/invalid; stop updating ServoWheel ({})",
                        (input_error.empty() ? "camera sample timeout" : input_error));
                    was_fresh = fresh;
                }
                const auto state = robot.GetState();
                const auto current = robot.GetStatus();
                if (state != last_state) { status(robot); last_state = state; }
                if (state == "FAULT" || current.fault)
                    throw std::runtime_error("Servo fault: " + current.motion_error);
                connection->report(state, fresh ? "CAMERA" : "NONE");
                if (fresh && now >= next_servo &&
                    (state == "LOCKED" || (state == "SERVO" && current.motion_error.empty()))) {
                    const auto& target = input.target();
                    robot.servoWheel(target.angle, target.displacement, 1.0);
                    next_servo = now + period; // 不补发错过的周期。
                    if (now >= next_print) {
                        aviator::Logger::info("Camera servo target angle={} displacement={} v=1 limited={}",
                                              target.angle, target.displacement, target.limited);
                        if (target.limited)
                            aviator::Logger::warn("Camera target clamped to angle +/-0.87266 rad, displacement [-0.170,0] m");
                        next_print = now + 100000;
                    }
                }
                // 无有效输入时不刷新目标，让 Servo 自身超时减速，保持软件锁定。
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        } catch (const std::exception &e) {
            aviator::Logger::error("Execution failed: {}", e.what());
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
        aviator::Logger::error("Error: {}", e.what());
        return 1;
    }
}

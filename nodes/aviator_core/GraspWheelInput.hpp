#pragma once
#include "CameraWheelInput.hpp"
#include "transport.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <chrono>
#include <thread>

namespace aviator {
struct GraspWheelConfig {
    bool camera = false;
    std::string camera_id = "cockpit";
    uint64_t freshness_us = 200000, wait_us = 2000000;

    static GraspWheelConfig load(const YAML::Node& robot) {
        GraspWheelConfig result;
        const auto node = robot["grasp_wheel"];
        if (!node) return result;
        if (!node.IsMap()) throw std::runtime_error("grasp_wheel must be a map");
        const auto source = node["source"].as<std::string>("wheel_initial");
        if (source != "wheel_initial" && source != "camera")
            throw std::runtime_error("grasp_wheel.source must be wheel_initial or camera");
        result.camera = source == "camera";
        result.camera_id = node["camera_id"].as<std::string>("cockpit");
        const auto freshness = node["max_age_ms"].as<int>(200);
        const auto wait = node["wait_timeout_ms"].as<int>(2000);
        if (result.camera_id.empty() || result.camera_id.size() > 128 ||
            freshness < 20 || freshness > 1000 || wait < freshness || wait > 10000)
            throw std::runtime_error("Invalid grasp_wheel camera settings: max_age_ms=20..1000, wait_timeout_ms=max_age_ms..10000");
        result.freshness_us = uint64_t(freshness) * 1000;
        result.wait_us = uint64_t(wait) * 1000;
        return result;
    }
};

// One subscription per grasp. No camera socket or observation is retained at startup
// or between grasps. Require an acquisition timestamp after this request began.
inline std::array<double, 2> acquireGraspWheel(zmq::context_t& context, const std::string& endpoint,
        const std::string& camera_id, uint64_t freshness_us, uint64_t wait_us,
        const std::function<void()>& check) {
    check();
    const auto began = monotonic_us();
    zmq::socket_t sub(context, zmq::socket_type::sub);
    configure(sub);
    subscribe(sub, "camera.detection");
    sub.connect(endpoint);
    ReceiveState receiver;
    CameraWheelInput input(camera_id, local_clock_id(), freshness_us, began,
                           CameraWheelInput::Mode::Grasp);
    std::string reason = "no new camera observation";
    while (monotonic_us() - began < wait_us) {
        check();
        for (int i = 0; i < 64; ++i) {
            WireMessage wire;
            std::string error;
            const auto received = receive(sub, receiver, wire, error);
            if (received == ReceiveResult::empty) break;
            Message message;
            if (received != ReceiveResult::received || !decode(wire.topic, wire.payload, message, error)) continue;
            input.accept(message, monotonic_us(), reason);
        }
        if (input.fresh(monotonic_us())) {
            check();
            return {input.target().angle, input.target().displacement};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("Camera grasp acquisition timed out: " +
        (reason.empty() ? "camera observation expired" : reason));
}
} // namespace aviator

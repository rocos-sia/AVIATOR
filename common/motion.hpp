#pragma once
#include "aviator/DataLink.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <array>
#include <filesystem>

namespace aviator {
using Json = nlohmann::json;
using Joints = std::array<double, 14>;
struct MotionConfig {
    std::filesystem::path system, robot;
    std::string publish, subscribe, service, config_id;
    unsigned period_us = 10000, timeout_us = 50000, origin_timeout_us = 100000;
};
MotionConfig loadMotionConfig(const std::filesystem::path &file);
std::filesystem::path defaultSystemConfig();
Message motionMessage(Topic, const std::string &publisher, const std::string &session, uint64_t sequence,
                      bool valid = true);
void publishMessage(zmq::socket_t &, const Message &);
Json parseService(std::string_view); // Same size, depth, duplicate/numeric rules as bus JSON.
Json serviceRequest(const std::string &session, const std::string &operation, const Json &parameters);
Json callService(zmq::context_t &, const MotionConfig &, const Json &request);

// Fixed capacity transport window. Only typed data crosses into the executor.
// Tick cursor, not wall-clock catch-up, is authoritative for both arms.
struct TrajectoryWindow {
    uint64_t id = 0, first = 0, total = 0, count = 0, sequence = 0;
    uint64_t sample = 0, origin_sample = 0, start = 0;
    std::array<JointFrame, 63> frames{};
};
Json encodeWindow(const TrajectoryWindow &, const std::string &session, const std::string &epoch);
TrajectoryWindow decodeWindow(const Message &, const Joints &lower, const Joints &upper,
                              const Joints &max_velocity);
struct DeviceState {
    Joints q{}, dq{}, target{};
    std::array<double, 14> tcp{}; // each side: x,y,z,qx,qy,qz,qw
    std::array<uint64_t, 2> sampled{};
    std::array<bool, 2> enabled{};
    uint64_t id = 0, cursor = 0, sequence = 0;
    double angle = 0, displacement = 0;
    double measured_angle = 0, measured_displacement = 0;
    bool locked = false, fault = false, stopping = false;
    std::string error;
};
} // namespace aviator

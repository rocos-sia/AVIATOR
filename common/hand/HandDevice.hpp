#pragma once
// SDK-free RH56 device boundary. All arrays use AVIATOR order (thumb rotation first).
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
namespace aviator::hand {
using Values = std::array<int, 6>;
using Bytes = std::vector<uint8_t>;
using Transfer = std::function<Bytes(const Bytes&, size_t)>;
inline constexpr Values open_pose{500, 1000, 1000, 1000, 1000, 1000};
inline constexpr int unchanged = -2;
struct Sample {
    Values position{}, force{}, current{}, errors{}, status{}, temperature{};
};
class Device {
  public:
    virtual ~Device() = default;
    virtual void setLabel(const std::string&) {} // Set by the owner before starting workers.
    virtual Values readPosition() = 0;
    virtual Sample readTelemetry() = 0;
    Sample read() {
        auto position = readPosition();
        auto sample = readTelemetry();
        sample.position = position;
        return sample;
    }
    virtual void write(uint16_t address, const Values&,
                       const std::function<void()>& check = {}) = 0;
};
inline constexpr uint16_t angle_set = 1486, force_set = 1498, speed_set = 1522, mode_set = 1625;
Bytes request(int id, uint8_t operation, uint16_t address, const Bytes& payload);
Bytes response(const Bytes& request, const Bytes& reply, size_t payload_size);
std::unique_ptr<Device> rs485Device(int id, Transfer transfer);
std::unique_ptr<Device> fakeDevice(double motion_rate = 2.0);
} // namespace aviator::hand

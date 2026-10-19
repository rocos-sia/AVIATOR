#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flight_simulator::rs422 {
inline constexpr uint64_t period_us = 12500;
inline constexpr size_t downlink_size = 32;
inline constexpr size_t uplink_size = 64;
inline constexpr size_t downlink_checksum_offset = downlink_size - 1;
inline constexpr size_t uplink_checksum_offset = uplink_size - 1;
inline constexpr int16_t invalid_position = -32768;
inline constexpr uint8_t unknown = 0xff;
inline constexpr uint16_t unknown_fault = 0xffff;
struct Downlink {
    uint8_t counter = 0, operation = 0;
    int16_t roll = 0, pitch = 0;
};
struct Uplink {
    uint8_t counter = 0, selected = unknown, communication = 3, operation = 0;
    int16_t roll_echo = invalid_position, pitch_echo = invalid_position;
    uint8_t state = unknown, rear = unknown, brake = unknown;
    int16_t roll = invalid_position, pitch = invalid_position;
    uint16_t fault = unknown_fault, last_fault = unknown_fault;
    uint8_t severity = unknown;
    uint16_t version = 0, checksum = 0;
};
using Bytes = std::vector<uint8_t>;
// Low eight bits of the supplied bytes; callers exclude header and checksum.
uint8_t checksum(const uint8_t*, size_t);
Bytes encode(const Downlink&);
Bytes encode(const Uplink&);
bool decode(const Bytes&, Downlink&);
bool decode(const Bytes&, Uplink&);
uint8_t operationCode(const std::string&);
const char* operationName(uint8_t);
uint8_t stateCode(const std::string&);
const char* stateName(uint8_t);
bool knownFault(uint16_t);
int16_t quantize(double normalized, double limit = 1.0);
bool samePayload(const Downlink&, const Downlink&);
class Parser {
  public:
    explicit Parser(bool uplink) : size_(uplink ? uplink_size : downlink_size) {}
    std::vector<Bytes> feed(const uint8_t*, size_t);
    void clear() { buffer_.clear(); }
    uint64_t rejected = 0;

  private:
    size_t size_;
    Bytes buffer_;
};
class CommandSender {
  public:
    bool request(uint8_t);
    uint8_t next();
    void reset() {
        operation_ = 0;
        remaining_ = 0;
        neutral_ = 10;
    }

  private:
    uint8_t operation_ = 0;
    unsigned remaining_ = 0, neutral_ = 10;
};
} // namespace flight_simulator::rs422

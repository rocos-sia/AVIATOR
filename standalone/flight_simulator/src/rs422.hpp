#pragma once
#include "wheel_units.hpp"
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
inline constexpr uint8_t not_started = 0xfe; // ICD 1127: normal startup waiting; 0xff remains emergency stop.
struct Downlink {
    uint8_t counter = 0, operation = 0;
    int16_t roll = 0, pitch = aviator::wheel::encodePitch(0);
};
struct Uplink {
    uint8_t counter = 0, selected = unknown, communication = 3, operation = 0;
    int16_t roll_echo = invalid_position, pitch_echo = invalid_position;
    uint8_t state = not_started, rear = unknown, brake = unknown;
    int16_t roll = invalid_position, pitch = invalid_position; // Camera feedback, bytes 13..16.
    uint16_t fault = 0, last_fault = 0;
    uint8_t severity = 0;
    uint16_t version = 0, checksum = 0;
    int16_t joint_roll = invalid_position, joint_pitch = invalid_position; // Joint FK, bytes 26..29.
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
uint8_t stateCode(const std::string&); // Throws for an unsupported name; never fabricates NOT_STARTED.
const char* stateName(uint8_t);
bool knownFault(uint16_t);
const char* faultName(uint16_t); // Description of the ICD 1127 additions; empty for existing codes.
using aviator::wheel::encodeRoll;
using aviator::wheel::encodePitch;
using aviator::wheel::decodeRoll;
using aviator::wheel::decodePitch;
bool samePayload(const Downlink&, const Downlink&);
class Parser {
  public:
    explicit Parser(bool uplink) : size_(uplink ? uplink_size : downlink_size) {}
    std::vector<Bytes> feed(const uint8_t*, size_t);
    void clear() { head_ = count_ = 0; }
    uint64_t rejected = 0;

  private:
    size_t size_;
    // One maximum-size frame is sufficient: feed parses after each byte is appended.
    // Consuming bytes only advances head_; unread bytes never move or get overwritten.
    std::array<uint8_t, uplink_size> buffer_{};
    size_t head_ = 0, count_ = 0;
    uint8_t peek(size_t offset) const { return buffer_[(head_ + offset) % buffer_.size()]; }
    void consume(size_t n) {
        head_ = (head_ + n) % buffer_.size();
        count_ -= n;
    }
};
class CommandSender {
  public:
    bool request(uint8_t);
    uint8_t next();
    void reset() {
        operation_ = 0;
        remaining_ = 0;
        last_emitted_ = 0;
        separator_ = false;
    }

  private:
    uint8_t operation_ = 0, last_emitted_ = 0;
    unsigned remaining_ = 0;
    bool separator_ = false;
};
} // namespace flight_simulator::rs422

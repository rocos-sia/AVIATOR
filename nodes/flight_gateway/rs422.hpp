#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace flight_gateway::rs422 {
inline constexpr uint64_t period_us = 12500;
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
    explicit Parser(bool uplink) : size_(uplink ? 29 : 9) {}
    std::vector<Bytes> feed(const uint8_t*, size_t);
    void clear() { buffer_.clear(); }
    uint64_t rejected = 0;

  private:
    size_t size_;
    Bytes buffer_;
};
// Startup, discontinuity and failover require ten neutral frames before any operation.
class CommandGate {
  public:
    uint8_t update(const Downlink&, bool consecutive);
    void reset();
    bool armed() const { return armed_; }

  private:
    unsigned neutral_ = 0, count_ = 0;
    uint8_t candidate_ = 0;
    bool armed_ = false;
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
struct Channel {
    Downlink frame;
    uint64_t received = 0, generation = 0;
    unsigned consecutive = 0;
    bool have = false;
};
// Latest samples only: queued bursts cannot satisfy the five-cycle operation gate.
class DualReceiver {
  public:
    DualReceiver(uint64_t timeout, unsigned recovery, int jump)
        : timeout_(timeout), recovery_(recovery), jump_(jump) {}
    bool receive(unsigned, const Downlink&, uint64_t now);
    void disconnect(unsigned);
    uint8_t update(uint64_t now);
    bool healthy(unsigned, uint64_t now) const;
    bool valid(uint64_t now) const;
    int selected() const { return selected_; }
    const Downlink& target() const { return target_; }
    uint64_t sampleTime() const { return sample_; }
    bool blocked() const { return blocked_; }
    std::array<Channel, 2> channels;
    uint64_t duplicates = 0, conflicts = 0, switches = 0;

  private:
    uint64_t timeout_, sample_ = 0, processed_ = 0, blocked_at_ = 0;
    unsigned recovery_, neutral_ = 0;
    int jump_, selected_ = -1;
    bool blocked_ = false, had_target_ = false, continuous_ = false;
    Downlink target_;
    CommandGate gate_;
};
} // namespace flight_gateway::rs422

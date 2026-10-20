#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flight_gateway::rs422 {
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
// Startup, discontinuity and failover require ten neutral frames before any operation.
class CommandGate {
  public:
    uint8_t update(const Downlink&, bool consecutive, uint64_t now);
    void reset();
    bool armed() const { return armed_; }

  private:
    unsigned neutral_ = 0, count_ = 0;
    uint8_t candidate_ = 0;
    bool armed_ = false;
    uint64_t neutral_since_ = 0, candidate_since_ = 0;
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
// Targets use the latest sample. Small USB batches retain their individual operation
// frames; oversized backlogs cannot satisfy the operation gate.
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
    uint64_t coalesced_batches = 0, rejected_batches = 0;

  private:
    uint64_t timeout_, sample_ = 0, processed_ = 0, blocked_at_ = 0;
    unsigned recovery_, neutral_ = 0;
    int jump_, selected_ = -1;
    bool blocked_ = false, had_target_ = false, continuous_ = false;
    Downlink target_;
    CommandGate gate_;
    struct Pending {
        Downlink frame;
        bool consecutive;
    };
    // Five entries are sufficient to detect and reject an oversized batch.
    std::array<std::vector<Pending>, 2> pending_;
};
} // namespace flight_gateway::rs422

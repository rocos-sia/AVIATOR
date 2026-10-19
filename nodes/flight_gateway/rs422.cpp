#include "rs422.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_gateway::rs422 {
namespace {
uint16_t u16(const Bytes& b, size_t p) { return uint16_t(b[p]) | uint16_t(b[p + 1]) << 8; }
int16_t i16(const Bytes& b, size_t p) {
    const auto v = u16(b, p);
    return static_cast<int16_t>(v <= 32767 ? int(v) : int(v) - 65536);
}
void put(Bytes& b, size_t p, uint16_t v) {
    b[p] = v & 255;
    b[p + 1] = v >> 8;
}
const std::array<const char*, 7> operations{{"", "enter_standby", "grasp_wheel", "start_control",
                                             "exit_control", "leave_wheel", "reset_error"}};
const std::array<std::pair<uint8_t, const char*>, 11> states{{{0, "EMERGENCY_STOP"},
                                                              {1, "INIT"},
                                                              {3, "READY"},
                                                              {5, "STANDBY"},
                                                              {7, "FOLLOWING"},
                                                              {8, "CONTROL"},
                                                              {0x10, "ERROR"},
                                                              {0x11, "INITIALIZING"},
                                                              {0x12, "HOMING"},
                                                              {0x13, "GRASPING"},
                                                              {0x14, "RELEASING"}}};
bool envelope(const Bytes& b, size_t n) {
    return b.size() == n && b[0] == 0xeb && b[1] == 0x90 && b[n - 1] == checksum(b.data() + 2, n - 3);
}
} // namespace
uint8_t checksum(const uint8_t* p, size_t n) {
    unsigned sum = 0;
    for (size_t i = 0; i < n; ++i)
        sum += p[i];
    return uint8_t(sum);
}
const char* operationName(uint8_t c) {
    return c && c % 16 == 0 && c <= 0x60 ? operations[c / 16] : "";
}
uint8_t operationCode(const std::string& s) {
    for (unsigned i = 1; i < operations.size(); ++i)
        if (s == operations[i])
            return i * 16;
    return 0;
}
uint8_t stateCode(const std::string& s) {
    for (auto [v, n] : states)
        if (s == n)
            return v;
    return unknown;
}
const char* stateName(uint8_t c) {
    for (auto [v, n] : states)
        if (c == v)
            return n;
    return "UNKNOWN";
}
bool knownFault(uint16_t c) {
    if (!c)
        return true;
    const int cat = c / 1000, tail = c % 1000;
    return ((cat == 1 || cat == 2) && ((tail >= 201 && tail <= 206) || tail == 301)) ||
           ((cat == 3 || cat == 4) &&
            (tail == 101 || (tail >= 201 && tail <= 203) || tail == 301)) ||
           (cat == 5 && (tail == 101 || tail == 102 || tail == 301)) ||
           (cat == 6 && tail >= 301 && tail <= 303) || (cat == 7 && tail >= 101 && tail <= 108) ||
           (cat == 8 &&
            (tail == 1 || tail == 2 || (tail >= 201 && tail <= 208) || tail == 210 || tail == 211));
}
int16_t quantize(double n, double limit) {
    if (!std::isfinite(n) || std::abs(n) > limit)
        throw std::invalid_argument("RS422 position out of range");
    return static_cast<int16_t>(std::lround(n * 1000));
}
bool samePayload(const Downlink& a, const Downlink& b) {
    return a.operation == b.operation && a.roll == b.roll && a.pitch == b.pitch;
}
Bytes encode(const Downlink& f) {
    if ((f.operation && !*operationName(f.operation)) || std::abs(int(f.roll)) > 1000 ||
        std::abs(int(f.pitch)) > 1000)
        throw std::invalid_argument("invalid RS422 downlink");
    Bytes b(downlink_size, 0); // Reserved bytes 8..30 are transmitted as zero.
    b[0] = 0xeb;
    b[1] = 0x90;
    b[2] = f.counter;
    b[3] = f.operation;
    put(b, 4, f.roll);
    put(b, 6, f.pitch);
    b[downlink_checksum_offset] = checksum(b.data() + 2, downlink_checksum_offset - 2);
    return b;
}
Bytes encode(const Uplink& f) {
    Bytes b(uplink_size, 0); // Reserved bytes 26..62 are transmitted as zero.
    b[0] = 0xeb;
    b[1] = 0x90;
    b[2] = f.counter;
    b[3] = f.selected;
    b[4] = f.communication;
    b[5] = f.operation;
    put(b, 6, f.roll_echo);
    put(b, 8, f.pitch_echo);
    b[10] = f.state;
    b[11] = f.rear;
    b[12] = f.brake;
    put(b, 13, f.roll);
    put(b, 15, f.pitch);
    put(b, 17, f.fault);
    put(b, 19, f.last_fault);
    b[21] = f.severity;
    put(b, 22, f.version);
    put(b, 24, f.checksum);
    b[uplink_checksum_offset] = checksum(b.data() + 2, uplink_checksum_offset - 2);
    return b;
}
bool decode(const Bytes& b, Downlink& f) {
    if (!envelope(b, downlink_size))
        return false;
    Downlink v{b[2], b[3], i16(b, 4), i16(b, 6)};
    if ((v.operation && !*operationName(v.operation)) || std::abs(int(v.roll)) > 1000 ||
        std::abs(int(v.pitch)) > 1000)
        return false;
    f = v;
    return true;
}
bool decode(const Bytes& b, Uplink& f) {
    if (!envelope(b, uplink_size))
        return false;
    if ((b[3] > 1 && b[3] != unknown) || b[4] > 3 || (b[5] && !*operationName(b[5])) ||
        (b[10] != unknown && stateCode(stateName(b[10])) != b[10]) ||
        (b[11] > 1 && b[11] != unknown) || (b[12] > 1 && b[12] != unknown) ||
        (b[21] != unknown && (b[21] > 0x40 || b[21] % 16)))
        return false;
    for (size_t p : {size_t(6), size_t(8), size_t(13), size_t(15)}) {
        const int v = i16(b, p), limit = p == 13 ? 1040 : p == 15 ? 1059 : 1000;
        if (v != invalid_position && std::abs(v) > limit)
            return false;
    }
    for (size_t p : {size_t(17), size_t(19)})
        if (!knownFault(u16(b, p)) && u16(b, p) != unknown_fault)
            return false;
    f = {b[2],  b[3],       b[4],       b[5],       i16(b, 6),  i16(b, 8), b[10],      b[11],
         b[12], i16(b, 13), i16(b, 15), u16(b, 17), u16(b, 19), b[21],     u16(b, 22), u16(b, 24)};
    return true;
}
std::vector<Bytes> Parser::feed(const uint8_t* data, size_t n) {
    std::vector<Bytes> out;
    for (size_t i = 0; i < n; ++i) {
        buffer_.push_back(data[i]);
        while (!buffer_.empty()) {
            if (buffer_[0] != 0xeb || (buffer_.size() > 1 && buffer_[1] != 0x90)) {
                buffer_.erase(buffer_.begin());
                ++rejected;
                continue;
            }
            if (buffer_.size() < size_)
                break;
            Downlink down;
            Uplink up;
            if (size_ == downlink_size ? decode(buffer_, down) : decode(buffer_, up)) {
                out.push_back(buffer_);
                buffer_.clear();
            } else {
                buffer_.erase(buffer_.begin());
                ++rejected;
            }
        }
    }
    return out;
}
void CommandGate::reset() {
    neutral_ = count_ = 0;
    candidate_ = 0;
    armed_ = false;
}
uint8_t CommandGate::update(const Downlink& f, bool consecutive) {
    if (!consecutive)
        reset();
    if (!f.operation) {
        count_ = 0;
        candidate_ = 0;
        if (++neutral_ >= 10) {
            armed_ = true;
            neutral_ = 10;
        }
        return 0;
    }
    neutral_ = 0;
    if (!armed_)
        return 0;
    if (candidate_ != f.operation) {
        candidate_ = f.operation;
        count_ = 0;
    }
    if (++count_ == 5) {
        armed_ = false;
        count_ = 0;
        return f.operation;
    }
    return 0;
}
bool CommandSender::request(uint8_t op) {
    if (!*operationName(op) || remaining_ || neutral_)
        return false;
    operation_ = op;
    remaining_ = 10;
    return true;
}
uint8_t CommandSender::next() {
    if (remaining_) {
        if (!--remaining_)
            neutral_ = 10;
        return operation_;
    }
    if (neutral_)
        --neutral_;
    return 0;
}
bool DualReceiver::healthy(unsigned i, uint64_t now) const {
    const auto& c = channels.at(i);
    return c.have && now >= c.received && now - c.received < timeout_;
}
void DualReceiver::disconnect(unsigned i) {
    channels.at(i).have = false;
    channels.at(i).consecutive = 0;
}
bool DualReceiver::receive(unsigned i, const Downlink& f, uint64_t now) {
    auto& c = channels.at(i);
    bool consecutive = false;
    if (healthy(i, now)) {
        const uint8_t delta = f.counter - c.frame.counter;
        if (delta == 0 || delta >= 128) {
            ++duplicates;
            return false;
        }
        consecutive = delta == 1;
    }
    c.consecutive = consecutive ? std::min(c.consecutive + 1, 1000u) : 1;
    c.frame = f;
    c.received = now;
    c.have = true;
    ++c.generation;
    return true;
}
bool DualReceiver::valid(uint64_t now) const {
    return selected_ >= 0 && healthy(selected_, now) && !blocked_ && sample_ && now >= sample_ &&
           now - sample_ < timeout_;
}
uint8_t DualReceiver::update(uint64_t now) {
    const bool both = healthy(0, now) && healthy(1, now);
    const bool conflict = both && channels[0].frame.counter == channels[1].frame.counter &&
                          !samePayload(channels[0].frame, channels[1].frame);
    if (conflict && !blocked_) {
        blocked_ = true;
        blocked_at_ = now;
        neutral_ = 0;
        ++conflicts;
        gate_.reset();
    }
    if (selected_ < 0 || !healthy(selected_, now)) {
        int next = -1;
        for (int i = 0; i < 2; ++i)
            if (healthy(i, now) && channels[i].consecutive >= recovery_) {
                next = i;
                break;
            }
        if (next != selected_) {
            if (next >= 0 && had_target_ &&
                (std::abs(int(channels[next].frame.roll) - target_.roll) > jump_ ||
                 std::abs(int(channels[next].frame.pitch) - target_.pitch) > jump_)) {
                blocked_ = true;
                blocked_at_ = now;
                neutral_ = 0;
            }
            if (selected_ >= 0 || had_target_)
                ++switches;
            selected_ = next;
            processed_ = 0;
            gate_.reset();
            continuous_ = false;
        }
    }
    if (selected_ < 0)
        return 0;
    const auto& c = channels[selected_];
    if (c.generation == processed_)
        return 0;
    const bool consecutive = continuous_ && c.generation == processed_ + 1 &&
                             uint8_t(c.frame.counter - target_.counter) == 1 && c.consecutive > 1;
    processed_ = c.generation;
    target_ = c.frame;
    sample_ = c.received;
    had_target_ = true;
    continuous_ = true;
    if (blocked_) {
        // Require a visible invalid interval, then stable neutral input; a live disagreement cannot
        // clear.
        if (!conflict && (!both || samePayload(channels[0].frame, channels[1].frame)) &&
            !target_.operation && consecutive)
            ++neutral_;
        else
            neutral_ = 0;
        if (neutral_ >= 10 && now - blocked_at_ >= 100000) {
            blocked_ = false;
            gate_.reset();
        }
        return 0;
    }
    return gate_.update(target_, consecutive);
}
} // namespace flight_gateway::rs422

#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rh56 {
using Bytes = std::vector<uint8_t>;
using Values = std::array<int, 6>;
constexpr uint16_t angleSet = 1486, forceSet = 1498, speedSet = 1522;
constexpr uint16_t angleAct = 1546, forceAct = 1582, errorAct = 1606;
inline uint8_t checksum(const Bytes& b) {
    unsigned sum = 0;
    for (size_t i = 2; i < b.size(); ++i) sum += b[i];
    return static_cast<uint8_t>(sum);
}
inline std::string hex(const Bytes& b) {
    std::ostringstream out;
    for (auto v : b) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(v) << ' ';
    return out.str();
}
inline Bytes request(int id, uint8_t op, uint16_t address, const Bytes& data) {
    if (id < 1 || id > 254 || data.empty() || data.size() > 12 ||
        (op != 0x11 && op != 0x12) || (op == 0x11 && (data.size() != 1 || data[0] < 1 || data[0] > 12)))
        throw std::invalid_argument("Invalid RH56 request");
    Bytes b{0xEB, 0x90, uint8_t(id), uint8_t(data.size() + 3), op,
            uint8_t(address & 255), uint8_t(address >> 8)};
    for (auto value : data) b.push_back(value);
    b.push_back(checksum(b));
    return b;
}
inline Bytes response(const Bytes& req, const Bytes& reply, size_t count) {
    if (req.size() < 9 || count < 1 || count > 12 || reply.size() != count + 8)
        throw std::runtime_error("RH56 response length mismatch");
    Bytes body(reply.begin(), reply.end() - 1);
    if (reply[0] != 0x90 || reply[1] != 0xEB || reply[2] != req[2] ||
        reply[3] != count + 3 || reply[4] != req[4] || reply[5] != req[5] ||
        reply[6] != req[6] || reply.back() != checksum(body))
        throw std::runtime_error("RH56 response header/ID/address/op/checksum mismatch");
    if (req[4] == 0x12 && (count != 1 || reply[7] != 1))
        throw std::runtime_error("RH56 write rejected");
    return Bytes(reply.begin() + 7, reply.end() - 1);
}
inline void validate(uint16_t address, const Values& values) {
    const int limit = address == forceSet ? 3000 : 1000;
    if (address != angleSet && address != speedSet && address != forceSet)
        throw std::invalid_argument("Register is not an allowed control register");
    for (int value : values)
        if (value < 0 || value > limit) throw std::invalid_argument("Hand value out of range");
}
class Hand {
public:
    using Transfer = std::function<Bytes(const Bytes&, size_t)>;
    Hand(int id, Transfer transfer) : id_(id), transfer_(std::move(transfer)) {
        if (id < 1 || id > 254) throw std::invalid_argument("Invalid RH56 ID");
    }
    // AR SDK: one complete transaction per six-channel register block.
    void write(uint16_t address, const Values& values) {
        validate(address, values);
        Bytes payload;
        payload.reserve(12);
        for (int value : values) {
            payload.push_back(uint8_t(value & 255));
            payload.push_back(uint8_t(value >> 8));
        }
        const auto req = request(id_, 0x12, address, payload);
        response(req, transfer_(req, 9), 1);
    }
    Values read(uint16_t address, bool signedValues = false) {
        const auto req = request(id_, 0x11, address, {12});
        const auto data = response(req, transfer_(req, 20), 12);
        Values result{};
        for (size_t j = 0; j < result.size(); ++j) {
            const int value = int(data[2*j]) | (int(data[2*j+1]) << 8);
            result[j] = signedValues && value >= 32768 ? value - 65536 : value;
        }
        return result;
    }
    Values errors() {
        auto req = request(id_, 0x11, errorAct, {6});
        auto data = response(req, transfer_(req, 14), 6);
        Values result{};
        std::copy(data.begin(), data.end(), result.begin());
        return result;
    }
private:
    int id_;
    Transfer transfer_;
};
} // namespace rh56

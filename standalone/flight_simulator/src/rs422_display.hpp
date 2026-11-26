#pragma once
#include "rs422.hpp"
#include <array>
#include <string>
#include <vector>

namespace flight_simulator {
// Keep the received bytes, including reserved fields; never reconstruct a capture
// with encode(), which would erase reserved data supplied by the peer.
struct ReceptionSnapshot {
    rs422::Bytes frame;
    uint64_t received_us = 0, accepted_frames = 0, ignored_frames = 0;
    uint64_t rejected_bytes = 0, rx_bytes = 0, tx_frames = 0;
    bool connected = false, current = false;
};
struct UplinkField {
    std::string offset, name, hex, value;
};
std::string hexBytes(const rs422::Bytes& bytes, size_t start = 0, size_t count = rs422::uplink_size);
std::vector<UplinkField> describeUplink(const rs422::Bytes& frame);
const char* receptionStatus(const ReceptionSnapshot&, uint64_t now, uint64_t timeout);
} // namespace flight_simulator

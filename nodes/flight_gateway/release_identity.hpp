#pragma once
#include "config.hpp"
#include <cstdint>
namespace flight_gateway {
struct ReleaseIdentity {
    uint16_t version, checksum;
    bool bench;
};
ReleaseIdentity releaseIdentity(const Rs422Config&);
} // namespace flight_gateway

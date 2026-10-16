#pragma once
#include "config.hpp"
#include "protocol.hpp"
#include "rs422.hpp"
namespace flight_gateway {
void runRs422(const Config&, int signals_fd);
// Adds only fresh, validated business feedback; default fields remain explicitly unknown.
void populateTelemetry(rs422::Uplink&, const aviator::Message&, uint64_t now);
} // namespace flight_gateway

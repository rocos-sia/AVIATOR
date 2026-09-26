#pragma once

#include "protocol.hpp"
#include "runtime.hpp"
#include <linux/input.h>

namespace flight_gateway {
struct Axis {
    unsigned code;
    int minimum, maximum, value;
    bool inverted = false;
};

// Decoder has no device IO: EV_ABS changes become one snapshot at SYN_REPORT.
struct JoystickSample {
    Axis roll, pitch;
    double roll_value = 0, pitch_value = 0;
    std::uint64_t sample_us = 0;
    bool valid = false;
    bool failed = false; // Dropped/malformed input requires restart, not auto-enable.
    void update(const input_event& event, std::uint64_t now_us);
    bool fresh(std::uint64_t now_us, std::uint64_t timeout_us) const;
};

aviator::Message command(const JoystickSample& sample, const std::string& session,
                         const std::string& clock, std::uint64_t sequence,
                         std::uint64_t now_us, std::uint64_t utc_us,
                         std::uint64_t timeout_us);

// Only validates the system summary consumed by this joystick gateway.
// Complete actuator/vision schemas remain the producer/Core's responsibility.
bool valid_state_summary(const aviator::Message& message);
} // namespace flight_gateway

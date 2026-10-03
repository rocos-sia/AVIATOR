#pragma once

#include "protocol.hpp"
#include "runtime.hpp"
#include "config.hpp"
#include <array>
#include <linux/input.h>
#include <vector>

namespace flight_gateway {
// Button numbers are the first 11 advertised evdev key codes, in ascending order.
struct JoystickButtons {
    std::array<unsigned, 11> codes{};
    std::array<bool, 11> held{};
    std::array<std::uint64_t, 11> pending{};
    std::array<std::string, 11> operations{{"enter_standby", "grasp_wheel", "start_control",
                                            "exit_control", "leave_wheel", "reset_error", "", "",
                                            "", "", ""}};
    std::vector<unsigned> update(const input_event&, std::uint64_t now_us, bool input_valid);
};

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
    std::uint64_t checked_us = 0;
    std::uint64_t initial_snapshot_us = 0;
    bool device_connected = false;
    bool report_pending = false;
    bool valid = false;
    bool failed = false; // Dropped/malformed input requires restart, not auto-enable.
    void update(const input_event& event, std::uint64_t now_us);
    void initializePosition(int roll_raw, int pitch_raw, std::uint64_t now_us);
    void deviceChecked(std::uint64_t now_us);
    void invalidate();
    bool fresh(std::uint64_t now_us, std::uint64_t timeout_us) const;
};

// Keyboard emulates the existing normalized JOYSTICK protocol for Core compatibility.
inline constexpr std::array<unsigned, 4> keyboard_arrows{KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN};
inline constexpr std::array<unsigned, 11> keyboard_buttons{
    KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_MINUS};
class KeyboardInput {
public:
    KeyboardInput(JoystickSample& sample, const KeyboardConfig& config)
        : sample_(sample), config_(config) {}
    void suppressHeld(unsigned code);
    void update(const input_event& event, std::uint64_t now_us);
    // Call only after a successful EVIOCGKEY and a fully drained event queue.
    void deviceChecked(std::uint64_t now_us);
private:
    void advance(std::uint64_t now_us);
    JoystickSample& sample_;
    KeyboardConfig config_;
    std::array<bool, 4> held_{}, pending_{}, suppressed_{};
    double roll_ = 0, pitch_ = 0;
    std::uint64_t integrated_us_ = 0, event_us_ = 0;
};

aviator::Message command(const JoystickSample& sample, const std::string& session,
                         const std::string& clock, std::uint64_t sequence,
                         std::uint64_t now_us, std::uint64_t utc_us,
                         std::uint64_t timeout_us);

// Only validates the system summary consumed by this joystick gateway.
// Complete actuator/vision schemas remain the producer/Core's responsibility.
bool valid_state_summary(const aviator::Message& message);

// Core feedback follows valid same-host publisher messages; legacy session pins are ignored.
class CoreFeedback {
public:
    CoreFeedback(const std::string& session, const std::string& clock);
    bool accept(const aviator::Message&, std::uint64_t now, std::string& error);
    bool expired(std::uint64_t now) const { return !guard_ || guard_->expired(now); }
    bool requestsReady(std::uint64_t now) const { return source_authorized_ && !expired(now); }
    const std::string& session() const { return policy_.session_id; }
private:
    aviator::InputPolicy policy_;
    std::optional<aviator::InputGuard> guard_;
    bool source_authorized_ = false;
};
} // namespace flight_gateway

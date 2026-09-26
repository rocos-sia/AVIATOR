#pragma once

#include "protocol.hpp"
#include <mutex>
#include <optional>
#include <utility>

namespace aviator {
std::uint64_t utc_us();
std::uint64_t monotonic_us();
std::string local_clock_id(); // Linux hostname + boot_id.
std::string new_session_id(); // UUID from Linux random/uuid.

// Non-real-time mailbox: one instance per Topic/authorized producer. Lock and
// copy are deliberately simple; this is NOT the RT servo handoff primitive.
template<class T> class LatestMailbox {
public:
    void store(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = std::move(value);
    }
    std::optional<T> load() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return value_;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        value_.reset();
    }
private:
    mutable std::mutex mutex_;
    std::optional<T> value_;
};

struct InputPolicy {
    Topic topic = Topic::flight_command;
    std::string publisher_id;
    std::string session_id; // Explicitly authorize a session; never auto-switch.
    std::string clock_id;
    std::uint64_t timeout_us = 100000;
    std::uint64_t future_tolerance_us = 0;
    std::string source; // Required for flight.command.
    std::string control_epoch; // Required for arm/hand.command.
    std::string origin_publisher_id;
    std::string origin_session_id;
    std::uint64_t origin_timeout_us = 100000;
};

// Single-threaded, after decode AND node business validation. One guard per
// Topic/producer. Reconstruct explicitly when authorizing a new session/epoch.
class InputGuard {
public:
    explicit InputGuard(InputPolicy policy);
    bool accept(const Message& message, std::uint64_t now_mono_us, std::string& error);
    bool expired(std::uint64_t now_mono_us) const;
private:
    InputPolicy policy_;
    std::uint64_t sequence_ = 0;
    std::uint64_t sample_ = 0;
    std::uint64_t origin_sample_ = 0;
    std::uint64_t received_ = 0;
    bool valid_ = false;
    bool has_origin_ = false;
};
} // namespace aviator

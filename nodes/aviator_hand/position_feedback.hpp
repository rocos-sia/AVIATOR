#pragma once
#include "inspire_hand.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

namespace inspire_hand {
// One outstanding read per hand. A complete six-register cycle replaces the
// snapshot atomically; partial cycles never mix old and new channel values.
// CAN has no transaction ID: times are host observations, not sensor timestamps.
class PositionFeedback {
public:
    PositionFeedback(int hand_id, int poll_rate_hz, std::uint64_t response_timeout_us)
        : hand_id_(hand_id), period_us_(1000000 / poll_rate_hz),
          response_timeout_us_(response_timeout_us) {}

    void step(InspireHand& hand, std::uint64_t now) {
        if (pending_) {
            if (now >= requested_at_ && now - requested_at_ < response_timeout_us_) return;
            ++timeouts_;
            pending_ = false;
            channel_ = 0;
        }
        if (channel_ == 0) {
            if (now < next_cycle_) return;
            cycle_start_ = now;
            next_cycle_ = now + period_us_;
        }
        requested_at_ = now;
        pending_ = true;
        try {
            hand.read_register(kActualPositionRegisters[channel_], hand_id_);
        } catch (const std::exception& e) {
            transport_error(e.what(), now);
        }
    }

    bool consume(const can_frame& frame, std::uint64_t now) {
        // Reject write acknowledgments, local read-request echoes, standard/RTR/
        // error frames, other devices, unexpected registers and invalid values.
        if (!pending_ || now < requested_at_ || now - requested_at_ >= response_timeout_us_ ||
            !(frame.can_id & CAN_EFF_FLAG) || (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) ||
            frame.can_id != (CAN_EFF_FLAG | InspireHand::can_id(
                kActualPositionRegisters[channel_], hand_id_, false)) || frame.len != 2)
            return false;
        const int value = frame.data[0] | (frame.data[1] << 8);
        if (value > kMaxValue) return false; // Includes negative short/sentinel 0xffff.
        staged_[channel_] = value;
        pending_ = false;
        if (++channel_ == staged_.size()) {
            values_ = staged_;
            sample_mono_us_ = cycle_start_; // Conservative oldest request in the snapshot.
            received_ = true;
            io_failed_ = false;
            last_error_.clear();
            ++samples_;
            channel_ = 0;
        }
        return true;
    }

    void transport_error(const std::string& error, std::uint64_t now) {
        ++io_errors_;
        last_error_ = error;
        io_failed_ = true;
        pending_ = false;
        channel_ = 0;
        next_cycle_ = now + period_us_;
    }
    bool fresh(std::uint64_t now, std::uint64_t timeout) const {
        return received_ && !io_failed_ && now >= sample_mono_us_ && now - sample_mono_us_ < timeout;
    }
    bool received() const { return received_; }
    const std::array<int, 6>& values() const { return values_; }
    std::uint64_t sample_mono_us() const { return sample_mono_us_; }
    std::uint64_t samples() const { return samples_; }
    std::uint64_t timeouts() const { return timeouts_; }
    std::uint64_t io_errors() const { return io_errors_; }
    const std::string& last_error() const { return last_error_; }

private:
    int hand_id_;
    std::uint64_t period_us_, response_timeout_us_;
    std::array<int, 6> values_{}, staged_{};
    std::size_t channel_ = 0;
    bool pending_ = false, received_ = false, io_failed_ = false;
    std::uint64_t next_cycle_ = 0, requested_at_ = 0, cycle_start_ = 0, sample_mono_us_ = 0;
    std::uint64_t samples_ = 0, timeouts_ = 0, io_errors_ = 0;
    std::string last_error_;
};
} // namespace inspire_hand

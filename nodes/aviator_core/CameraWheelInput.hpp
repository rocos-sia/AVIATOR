#pragma once
#include "runtime.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aviator {
struct CameraWheelTarget {
    double angle = 0;
    double displacement = -.085;
    bool limited = false;
};

// Same coordinate mapping as Monitor's camera model joints, with motion limits.
// Only decoded camera.detection messages enter this non-real-time adapter.
class CameraWheelInput {
public:
    CameraWheelInput(std::string camera_id, std::string clock, uint64_t timeout_us,
                     uint64_t start_us = 0)
        : camera_id_(std::move(camera_id)), guard_(policy(std::move(clock), timeout_us)),
          start_us_(start_us) {}

    bool accept(const Message& message, uint64_t now, std::string& error) {
        if (message.topic != Topic::camera_detection || message.header.publisher_id != "camera" ||
            !message.body.is_object() || !message.body.contains("camera_id") ||
            message.body.at("camera_id") != camera_id_) return false;
        // A bad observation from our camera immediately revokes the cached target.
        valid_ = false;
        try {
            if (message.header.sample_mono_us < start_us_ ||
                (last_sample_ && message.header.sample_mono_us <= last_sample_))
                throw std::runtime_error("old or regressing camera sample");
            // Track the envelope even when the observation is invalid, so a
            // delayed older valid frame cannot reactivate motion. This guard
            // checks only time/order; valid_ below controls target usability.
            Message envelope;
            envelope.topic = message.topic;
            envelope.header = message.header;
            envelope.header.valid = true;
            if (!guard_.accept(envelope, now, error)) return false;
            last_sample_ = message.header.sample_mono_us;
            const auto& wheel = message.body.at("steering_wheel");
            if (!message.header.valid || wheel.at("valid") != true)
                throw std::runtime_error("invalid camera/steering_wheel observation");
            const auto& theta = wheel.at("theta_rad");
            const auto& travel = wheel.at("translation_along_axis_m");
            if (!theta.is_number() || !travel.is_number())
                throw std::runtime_error("camera wheel coordinates must be numeric");
            const double angle = -theta.get<double>();
            const double displacement = -travel.get<double>() - .085;
            if (!std::isfinite(angle) || !std::isfinite(displacement))
                throw std::runtime_error("non-finite camera wheel coordinates");
            target_.angle = std::clamp(angle, -.87266, .87266);
            target_.displacement = std::clamp(displacement, -.170, 0.0);
            target_.limited = target_.angle != angle || target_.displacement != displacement;
            valid_ = true;
            error.clear();
            return true;
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
    }
    bool fresh(uint64_t now) const { return valid_ && !guard_.expired(now); }
    const CameraWheelTarget& target() const { return target_; }

private:
    static InputPolicy policy(std::string clock, uint64_t timeout_us) {
        InputPolicy p;
        p.topic = Topic::camera_detection;
        p.publisher_id = "camera";
        p.clock_id = std::move(clock);
        p.timeout_us = timeout_us;
        return p;
    }
    std::string camera_id_;
    InputGuard guard_;
    uint64_t start_us_, last_sample_ = 0;
    bool valid_ = false;
    CameraWheelTarget target_;
};
} // namespace aviator

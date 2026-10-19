#pragma once
#include "motion.hpp"
#include <atomic>
#include <mutex>
#include <thread>

namespace aviator {
// Sticky input evidence only. The FSM remains on its owner thread.
class PhotoelectricEmergency {
public:
    struct State {
        bool healthy = false, latched = false;
        std::string reason;
        uint64_t sample = 0;
    };
    explicit PhotoelectricEmergency(const MotionConfig&);
    ~PhotoelectricEmergency();
    State snapshot();
private:
    void run();
    void latch(const std::string&);
    PhotoelectricEmergencyConfig config_;
    std::string endpoint_, clock_;
    std::atomic<bool> quit_{false};
    std::mutex mutex_;
    State state_;
    bool armed_ = false;
    uint64_t received_ = 0;
    std::thread thread_;
};
}

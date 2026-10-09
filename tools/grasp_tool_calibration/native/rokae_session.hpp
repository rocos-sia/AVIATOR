#pragma once

// The SDK and its Eigen/KDL symbols remain inside the existing shared-library
// isolation boundary. Only standard-library values cross this interface.
#include <array>
#include <csignal>
#include <memory>
#include <string>

namespace aviator::calibration {

struct SessionArmStatus {
    bool connected = false;
    bool dragging = false;
    bool drag_owned = false;
    int operation_state_code = -1;
    std::string operation_state = "disconnected";
    int power_state_code = -1;
    std::string power_state = "unknown";
};

class RokaeSessionArm {
 public:
    explicit RokaeSessionArm(std::string ip, const volatile std::sig_atomic_t* cancellation = nullptr);
    ~RokaeSessionArm();
    RokaeSessionArm(const RokaeSessionArm&) = delete;
    RokaeSessionArm& operator=(const RokaeSessionArm&) = delete;
    void connect();
    SessionArmStatus status();
    void startDrag();
    void stopDrag();
    std::array<double, 7> position();
    // Attempts both drag cleanup and disconnection and reports every failure.
    // The SDK documents that disconnect itself stops robot motion.
    void disconnect();

 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace aviator::calibration

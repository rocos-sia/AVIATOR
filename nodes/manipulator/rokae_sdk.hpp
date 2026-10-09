#pragma once
// SDK isolation boundary: no KDL/Eigen/SDK types cross this interface.
#include <array>
#include <functional>
#include <memory>
#include <string>
namespace aviator {
struct RokaeSample {
    double received_time = 0; // Monotonic receipt time; repeated reads retain this timestamp.
    std::array<double, 7> position{}, velocity{};
    std::array<double, 16> tcp{};
};
struct RokaeTiming {
    unsigned long long callbacks = 0;
    double max_gap_ms = 0, max_callback_us = 0;
    int policy = -1, priority = -1;
};
class RokaeArm {
public:
    RokaeArm(const std::string &ip, const std::string &local_ip,
             const std::array<double, 16> &tool, const std::array<double, 7> &stiffness);
    ~RokaeArm();
    std::array<double, 7> position() const;
    std::string diagnostics() const;
    RokaeTiming timing() const;
    void beginTargetTrace(const std::array<double, 7>& expected);
    void pollTargetTrace();
    bool motionFailed() const;
    void prepare(); // 上电、订阅、设置参数；不启动周期运动。
    void start(std::function<std::array<double, 7>(const RokaeSample &)> callback,
               std::function<void(const std::array<double, 7> &)> initialize_target);
    void pauseImpedance(); // Stop motion/state reception; keep power, RtCommand and the callback.
    void setPausedStiffness(const std::array<double, 7>&);
    void resumeImpedance(const std::function<void()>& check);
    void stop();
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}

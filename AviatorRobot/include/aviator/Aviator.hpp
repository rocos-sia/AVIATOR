#pragma once
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <rocos_mujoco/aviator_protocol.hpp>
#include <string>

namespace aviator {
struct JointTarget {
    std::array<double, 14> q{};
    std::array<double, 2> next_task{}; // theta [rad], displacement [m]
};
// 公共接口为阻塞操作，失败抛出带原因的 std::runtime_error。
// Stop 可由另一线程调用；GetStatus 可在运动期间读取。
class Aviator {
  public:
    explicit Aviator(const std::string &config, int bus = 0);
    ~Aviator();
    Aviator(const Aviator &) = delete;
    Aviator &operator=(const Aviator &) = delete;
    void Init();
    void Enable();
    void Disable();
    void ApproachHandles();
    void LockHandles();
    void MoveWheel(double angle_rad, double displacement_m, double duration_s = 4.0);
    // A single controller owns all 14 axes. The callback receives an atomic
    // MuJoCo feedback snapshot every 10 ms; nullopt holds the measured pose.
    void RunJointFeedback(double duration_s,
                          const std::function<std::optional<JointTarget>(
                              const rocos_mujoco::aviator::Feedback &)> &controller);
    void UnlockHandles();
    void ResetFault();
    void Stop() noexcept;
    rocos_mujoco::aviator::Feedback GetStatus();
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator

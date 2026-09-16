#pragma once
#include <memory>
#include <rocos_mujoco/aviator_protocol.hpp>
#include <string>

namespace aviator {
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

#pragma once

#include <array>
#include <memory>
#include <string>

namespace aviator {

constexpr int kArmJoints = 7;
constexpr int kTotalJoints = 14;

using Joints = std::array<double, kTotalJoints>;

struct Status {
    double angle;        // 轮盘角度 (rad)
    double displacement; // 轮盘位移 (m)
    bool locked;         // 把手是否锁定
    bool ready;          // 系统是否就绪
    bool fault;          // 是否有故障
};

class Aviator {
  public:
    explicit Aviator(const std::string &config_path);
    ~Aviator();

    // 禁止拷贝和移动
    Aviator(const Aviator &) = delete;
    Aviator &operator=(const Aviator &) = delete;

    // 系统控制
    void Enable();
    void Disable();
    void Initialize();
    void ApproachHandles();
    void LockHandles();
    void MoveWheel(double angle_rad, double displacement_m, double duration_s);
    void UnlockHandles();
    void ResetFault();
    void Stop() noexcept;

    // 状态查询
    Status GetStatus();
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace aviator

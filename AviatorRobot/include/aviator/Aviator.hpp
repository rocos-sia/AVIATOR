#pragma once
#include "aviator/DataLink.hpp"
#include <cstdint>
#include <memory>
#include <string>

namespace aviator {

// 公共状态:抓取状态 + 内部跟踪的轮盘角度/位移。
// 轮盘角度/位移是算法的规划变量(MoveWheel 输入 + 内部跟踪),不来自编码器。
struct Status {
    double angle = 0, displacement = 0;
    double position_error[2]{}, rotation_error[2]{};
    uint32_t locked = 0, ready = 0, fault = 0;
    uint64_t ack = 0;
    GraspResult result = GraspResult::Ok;
};

// 公共接口为阻塞操作,失败抛出带原因的 std::runtime_error。
// Stop 可由另一线程调用;GetStatus 可在运动期间读取。
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
    Status GetStatus();
    std::string GetState() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator

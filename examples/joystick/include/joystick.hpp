#pragma once
#include <chrono>
#include <string>
#include <vector>

namespace joystick {
struct State {
    bool connected = false;
    std::string name, error;
    std::vector<int> axes;     // 原始值：-32767..32767
    std::vector<int> buttons;  // 0=松开，1=按下
    bool rumble_available = false;
    std::string rumble_error;
};
// 单线程使用；poll() 非阻塞，断开后再次调用会尝试重连。
class Device {
public:
    explicit Device(std::string path = "/dev/input/js0");
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    const State& poll();
    // 启动一次 600ms 震动；持续震动需定期续期，false 立即停止。
    bool rumble(bool enabled);
private:
    void openRumble();
    void closeRumble();
    int rumble_fd_ = -1, effect_id_ = -1;
    std::chrono::steady_clock::time_point retry_rumble_{};
    void disconnect(const std::string& error);
    std::string path_;
    int fd_ = -1;
    State state_;
};
} // namespace joystick

#include "joystick.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <linux/input.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace joystick {
Device::Device(std::string path) : path_(std::move(path)) {}
Device::~Device() { closeRumble(); if (fd_ >= 0) ::close(fd_); }
void Device::disconnect(const std::string& error) {
    closeRumble();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    state_ = {};
    state_.error = error;
}
void Device::closeRumble() {
    if (rumble_fd_ >= 0) {
        if (effect_id_ >= 0) {
            rumble(false);
            ::ioctl(rumble_fd_, EVIOCRMFF, effect_id_);
        }
        ::close(rumble_fd_);
    }
    rumble_fd_ = effect_id_ = -1;
    state_.rumble_available = false;
}
void Device::openRumble() {
    retry_rumble_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    state_.rumble_error = "未找到手柄对应的 event 节点";
    struct stat info{};
    if (::fstat(fd_, &info) < 0) { state_.rumble_error = std::strerror(errno); return; }
    // 按设备号查找同一个 input 设备的 event 节点，兼容 js 路径的符号链接。
    const auto directory = "/sys/dev/char/" + std::to_string(major(info.st_rdev)) +
        ":" + std::to_string(minor(info.st_rdev)) + "/device";
    std::error_code error;
    std::filesystem::directory_iterator it(directory, error), end;
    for (; !error && it != end; it.increment(error)) {
        const auto name = it->path().filename().string();
        if (name.rfind("event", 0) != 0) continue;
        const auto path = "/dev/input/" + name;
        const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            state_.rumble_error = path + ": " + std::strerror(errno);
            if (errno == EACCES) state_.rumble_error += "（震动需要此节点的读写权限）";
            continue;
        }
        unsigned char bits[(FF_CNT + 7) / 8]{};
        const bool queried = ::ioctl(fd, EVIOCGBIT(EV_FF, sizeof(bits)), bits) >= 0;
        const auto supports = [&](int code) { return (bits[code / 8] & (1 << (code % 8))) != 0; };
        if (!queried) {
            state_.rumble_error = path + ": 查询力反馈能力失败：" + std::strerror(errno);
        } else {
            state_.rumble_error = path + ": 驱动未提供 RUMBLE 或周期震动力反馈";
            // 部分飞行摇杆只暴露周期力反馈；RUMBLE 上传失败也尝试周期效果。
            for (const int type : {FF_RUMBLE, FF_PERIODIC}) {
                const int wave = supports(FF_SINE) ? FF_SINE : FF_TRIANGLE;
                if (!supports(type) || (type == FF_PERIODIC && !supports(wave))) continue;
                ff_effect effect{};
                effect.type = type;
                effect.id = -1;
                if (type == FF_RUMBLE) {
                    effect.u.rumble.strong_magnitude = 0x8000;
                    effect.u.rumble.weak_magnitude = 0x8000;
                } else {
                    effect.direction = 0x4000;
                    effect.u.periodic.waveform = wave;
                    effect.u.periodic.period = 25; // 40 Hz
                    effect.u.periodic.magnitude = 0x4000;
                }
                effect.replay.length = 600;
                if (::ioctl(fd, EVIOCSFF, &effect) >= 0) {
                    rumble_fd_ = fd;
                    effect_id_ = effect.id;
                    state_.rumble_available = true;
                    state_.rumble_error.clear();
                    return;
                }
                state_.rumble_error = path + ": 上传震动效果失败：" + std::strerror(errno);
            }
        }
        ::close(fd);
    }
    if (error) state_.rumble_error = "查找力反馈设备失败：" + error.message();
}
bool Device::rumble(bool enabled) {
    if (rumble_fd_ < 0 || effect_id_ < 0) {
        if (enabled && state_.rumble_error.empty()) state_.rumble_error = "手柄未连接或震动不可用";
        return !enabled;
    }
    input_event event{};
    event.type = EV_FF;
    event.code = effect_id_;
    event.value = enabled ? 1 : 0;
    ssize_t n;
    do { n = ::write(rumble_fd_, &event, sizeof(event)); } while (n < 0 && errno == EINTR);
    if (n != sizeof(event)) {
        state_.rumble_error = n < 0 ? std::strerror(errno) : "震动指令写入失败";
        return false;
    }
    state_.rumble_error.clear();
    return true;
}
const State& Device::poll() {
    if (fd_ < 0) {
        fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            state_.error = path_ + ": " + std::strerror(errno);
            return state_;
        }
        unsigned char axes = 0, buttons = 0;
        char name[256] = {};
        if (::ioctl(fd_, JSIOCGAXES, &axes) < 0 ||
            ::ioctl(fd_, JSIOCGBUTTONS, &buttons) < 0) {
            disconnect(std::strerror(errno));
            return state_;
        }
        ::ioctl(fd_, JSIOCGNAME(sizeof(name)), name);
        name[sizeof(name) - 1] = '\0';
        state_ = {true, name[0] ? name : "USB Joystick", "",
                  std::vector<int>(axes), std::vector<int>(buttons), false, ""};
        openRumble();
    }
    if (rumble_fd_ < 0 && std::chrono::steady_clock::now() >= retry_rumble_) openRumble();
    js_event event{};
    // 有界批量读取，避免持续输入独占调用线程。
    for (int i = 0; i < 1024; ++i) {
        const auto n = ::read(fd_, &event, sizeof(event));
        if (n == sizeof(event)) {
            const int type = event.type & ~JS_EVENT_INIT;
            if (type == JS_EVENT_AXIS && event.number < state_.axes.size())
                state_.axes[event.number] = event.value;
            if (type == JS_EVENT_BUTTON && event.number < state_.buttons.size())
                state_.buttons[event.number] = event.value != 0;
        } else if (n < 0 && errno == EINTR) {
            continue;
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        } else {
            disconnect(n < 0 ? std::strerror(errno) : "Device disconnected");
            break;
        }
    }
    return state_;
}
} // namespace joystick

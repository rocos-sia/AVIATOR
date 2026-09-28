#pragma once
// Minimal raw SocketCAN writer for the Inspire robotic hand (因时手). Mirrors the
// can_init()/write() logic in examples/sub_hand_position.cpp, but wrapped as a
// small RAII class so the hand driver doesn't own raw sockets directly.
//
// The interface must already be brought up before open(), e.g.:
//   sudo ip link set can0 up type can bitrate 500000

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

namespace inspire_hand {

class SocketCan {
public:
    SocketCan() = default;
    ~SocketCan() { close(); }

    SocketCan(const SocketCan&) = delete;
    SocketCan& operator=(const SocketCan&) = delete;

    // Opens the named CAN interface (e.g. "can0"). Throws std::runtime_error on
    // failure so the caller can report a helpful bring-up hint.
    void open(const std::string& interface) {
        close();
        const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0)
            throw std::runtime_error(std::string("socket: ") + std::strerror(errno));

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            const std::string msg = std::string("ioctl SIOCGIFINDEX ") + interface +
                                    ": " + std::strerror(errno);
            ::close(fd);
            throw std::runtime_error(msg);
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            const std::string msg =
                std::string("bind ") + interface + ": " + std::strerror(errno);
            ::close(fd);
            throw std::runtime_error(msg);
        }

        fd_ = fd;
        interface_ = interface;
    }

    // Sends one extended CAN frame. `can_id` is the 29-bit identifier without the
    // EFF flag; this helper ORs CAN_EFF_FLAG in. len <= 8.
    void write(std::uint32_t can_id, const std::uint8_t* data, std::size_t len) {
        if (fd_ < 0) throw std::runtime_error("SocketCan::write before open");
        if (len > 8) throw std::runtime_error("SocketCan::write len > 8");
        struct can_frame frame{};
        frame.can_id = can_id | CAN_EFF_FLAG;
        frame.len = static_cast<std::uint8_t>(len);
        std::memcpy(frame.data, data, len);
        if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame)))
            throw std::runtime_error(std::string("SocketCan::write: ") + std::strerror(errno));
    }

    // Non-blocking receive of one frame. Returns nullopt if nothing is pending.
    // Provided for future feedback (angleAct/forceAct) support.
    std::optional<can_frame> try_recv() {
        if (fd_ < 0) return std::nullopt;
        struct can_frame frame{};
        const ssize_t n = ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return std::nullopt;
            throw std::runtime_error(std::string("SocketCan::try_recv: ") + std::strerror(errno));
        }
        if (n != static_cast<ssize_t>(sizeof(can_frame))) return std::nullopt;
        return frame;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const { return fd_ >= 0; }
    const std::string& interface() const { return interface_; }

private:
    int fd_ = -1;
    std::string interface_;
};

}  // namespace inspire_hand

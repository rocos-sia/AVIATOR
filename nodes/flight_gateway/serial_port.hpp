#pragma once
#include "rs422.hpp"
#include <string>
namespace flight_gateway {
// Owns one nonblocking full-duplex 115200 8E1 tty. At most one frame is queued.
class SerialPort {
  public:
    explicit SerialPort(std::string path) : path_(std::move(path)) {}
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    bool connect(uint64_t now);
    void close();
    void cancelOutput();
    bool read(rs422::Bytes&);
    bool send(const rs422::Bytes&, uint64_t now);
    void flush(uint64_t now);
    int fd() const { return fd_; }
    uint64_t connections() const { return connections_; }
    uint64_t tx_frames = 0, rx_bytes = 0, dropped = 0;

  private:
    std::string path_;
    int fd_ = -1;
    bool owned_ = false;
    uint64_t retry_ = 0, connections_ = 0, queued_at_ = 0;
    size_t offset_ = 0;
    rs422::Bytes pending_;
};
} // namespace flight_gateway

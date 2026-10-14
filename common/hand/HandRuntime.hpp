#pragma once
#include "HandDevice.hpp"
#include "protocol.hpp"
#include <filesystem>
#include <memory>
#include <string>
namespace aviator::hand {
struct Config {
    bool enabled = true, fake = false;
    std::array<bool, 2> sides{true, true};
    std::array<int, 2> ids{1, 1};
    int speed = 500, force = 1000, hold_speed = 100, hold_force = 100, threshold = 10;
    unsigned command_us = 100000, feedback_us = 500000, hold_us = 10000000, period_us = 60000;
    unsigned write_period_us = 50000, position_period_us = 100000, telemetry_period_us = 200000;
    int rs485_timeout_ms = 50;
    double motion_rate = 2;
    std::string publisher = "rh56ftp_hand";
};
Config loadConfig(const std::filesystem::path& system, bool fake_override = false,
                  const std::filesystem::path& hand_file = {});
// Worker owns control policy; independent readers and bus snapshots never wait
// for a control write. Owner must destroy Runtime before its SDK connections.
class Runtime {
  public:
    Runtime(Config, std::array<std::unique_ptr<Device>, 2>);
    ~Runtime();
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;
    void receive(const Message&); // bounded latest-command mailbox, no device I/O
    unsigned periodUs() const;
    Message state(); // current envelope + original per-side timestamps
  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
// Used by the calibration SDK owner and offline process tests. ZMQ sockets are
// created and used in this bus thread; no additional controller connection.
class Bus {
  public:
    Bus(Runtime&, const std::string& publish, const std::string& subscribe);
    ~Bus();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator::hand

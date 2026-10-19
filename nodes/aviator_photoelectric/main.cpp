#include "runtime.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t stopping = 0;
void on_signal(int) { stopping = 1; }
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void keys(const YAML::Node& node, std::initializer_list<const char*> allowed) {
    require(node.IsMap(), "configuration section must be a map");
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        require(std::any_of(allowed.begin(), allowed.end(), [&](const char* s) { return key == s; }),
                "unknown configuration key: " + key);
    }
}
int bounded(const YAML::Node& node, const char* key, int low, int high) {
    const auto value = node[key].as<int>();
    require(value >= low && value <= high, std::string(key) + " out of range");
    return value;
}
speed_t baud_speed(int baud) {
    switch (baud) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: throw std::runtime_error("unsupported baud_rate (4800/9600/19200/38400/57600/115200)");
    }
}
struct Config {
    std::string device, parity, publisher, endpoint;
    int baud, stop_bits, slave, address;
    std::uint64_t poll_us, publish_us, timeout_us, retry_us, stale_us, gap_us;
    bool invert;
};
Config load_config(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    keys(root, {"serial", "modbus", "poll_hz", "publish_hz", "stale_timeout_ms",
                "invert", "publisher_id", "publish_endpoint"});
    const auto serial = root["serial"], modbus = root["modbus"];
    keys(serial, {"device", "baud_rate", "data_bits", "parity", "stop_bits"});
    keys(modbus, {"slave_address", "input_address", "response_timeout_ms", "reconnect_interval_ms"});
    Config c;
    c.device = serial["device"].as<std::string>();
    require(!c.device.empty() && c.device[0] == '/' && c.device.find('\0') == std::string::npos,
            "serial.device must be an absolute path");
    c.baud = serial["baud_rate"].as<int>();
    (void)baud_speed(c.baud);
    bounded(serial, "data_bits", 8, 8);
    c.stop_bits = bounded(serial, "stop_bits", 1, 2);
    c.parity = serial["parity"].as<std::string>();
    require(c.parity == "none" || c.parity == "even" || c.parity == "odd", "invalid parity");
    c.slave = bounded(modbus, "slave_address", 1, 255);
    c.address = bounded(modbus, "input_address", 0, 63);
    c.timeout_us = bounded(modbus, "response_timeout_ms", 10, 5000) * 1000ULL;
    c.retry_us = bounded(modbus, "reconnect_interval_ms", 100, 60000) * 1000ULL;
    require(c.retry_us >= c.timeout_us, "reconnect_interval_ms must cover response_timeout_ms");
    c.poll_us = 1000000 / bounded(root, "poll_hz", 1, 100);
    c.publish_us = 1000000 / bounded(root, "publish_hz", 1, 100);
    c.stale_us = bounded(root, "stale_timeout_ms", 10, 60000) * 1000ULL;
    require(c.stale_us >= c.poll_us + c.timeout_us,
            "stale_timeout_ms must cover polling interval plus response timeout");
    const int bits = 1 + 8 + c.stop_bits + (c.parity != "none");
    c.gap_us = c.baud > 19200 ? 1750 : (3500000ULL * bits + c.baud - 1) / c.baud;
    require(c.poll_us >= 14000000ULL * bits / c.baud + 2 * c.gap_us,
            "poll_hz too high for baud_rate");
    c.invert = root["invert"].as<bool>();
    c.publisher = root["publisher_id"].as<std::string>();
    require(!c.publisher.empty() && c.publisher.size() <= 128 &&
            c.publisher.find('\0') == std::string::npos, "invalid publisher_id");
    c.endpoint = root["publish_endpoint"].as<std::string>();
    require(c.endpoint.rfind("tcp://", 0) == 0 && c.endpoint.find('\0') == std::string::npos,
            "publish_endpoint must be a TCP endpoint");
    return c;
}

std::uint16_t crc16(const std::uint8_t* bytes, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
    return crc;
}

class TerminalStatus {
public:
    ~TerminalStatus() {
        if (visible_) std::cout << '\n' << std::flush;
    }
    void show(const std::string& channel, int value, bool valid, const std::string& status) {
        if (!enabled_) return;
        std::cout << "\r\033[2K" << channel << '=' << value
                  << "  valid=" << (valid ? "true" : "false")
                  << "  status=" << status << std::flush;
        visible_ = true;
    }
    void clear() {
        if (visible_) std::cout << "\r\033[2K" << std::flush;
        visible_ = false;
    }
private:
    bool enabled_ = ::isatty(STDOUT_FILENO);
    bool visible_ = false;
};

// One outstanding request. Serial work never blocks the publication clock.
class Input {
public:
    Input(const Config& config, TerminalStatus& terminal) : c_(config), terminal_(terminal) {
        request_ = {static_cast<std::uint8_t>(c_.slave), 2,
            static_cast<std::uint8_t>(c_.address >> 8), static_cast<std::uint8_t>(c_.address), 0, 1, 0, 0};
        const auto crc = crc16(request_.data(), 6);
        request_[6] = crc & 0xff;
        request_[7] = crc >> 8;
    }
    ~Input() { close_port(); }
    Input(const Input&) = delete;
    Input& operator=(const Input&) = delete;
    int value = 0;
    std::uint64_t sample = 0;
    std::string status = "waiting";
    bool valid(std::uint64_t now) const {
        return status == "ok" && sample != 0 && now - sample < c_.stale_us;
    }
    void tick(std::uint64_t now) {
        if (fd_ < 0) {
            if (now < retry_at_) return;
            fd_ = ::open(c_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
            if (fd_ < 0) { fail(now, "open: " + std::string(std::strerror(errno))); return; }
            if (flock(fd_, LOCK_EX | LOCK_NB) < 0) { fail(now, "serial device is locked"); return; }
            termios t{};
            if (tcgetattr(fd_, &t) < 0) { fail(now, "tcgetattr failed"); return; }
            cfmakeraw(&t);
            t.c_cflag = (t.c_cflag & ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
            if (c_.parity != "none") t.c_cflag |= PARENB;
            if (c_.parity == "odd") t.c_cflag |= PARODD;
            if (c_.stop_bits == 2) t.c_cflag |= CSTOPB;
            t.c_cc[VMIN] = 0;
            t.c_cc[VTIME] = 0;
            cfsetispeed(&t, baud_speed(c_.baud));
            cfsetospeed(&t, baud_speed(c_.baud));
            if (tcsetattr(fd_, TCSANOW, &t) < 0 || tcflush(fd_, TCIOFLUSH) < 0) {
                fail(now, "serial configuration failed"); return;
            }
            next_request_ = now + c_.gap_us;
        }
        pollfd event{fd_, POLLIN, 0};
        const int ready = ::poll(&event, 1, 0);
        if ((ready < 0 && errno != EINTR) || (event.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fail(now, "serial disconnected"); return;
        }
        if (!pending_) {
            if (now < next_request_) return;
            if (tcflush(fd_, TCIFLUSH) < 0) { fail(now, "input flush failed"); return; }
            pending_ = true;
            sent_ = 0;
            response_.clear();
            deadline_ = now + c_.timeout_us;
            next_request_ = now + c_.poll_us;
        }
        if (now >= deadline_) { fail(now, "response timeout"); return; }
        if (sent_ < request_.size()) {
            const auto count = ::write(fd_, request_.data() + sent_, request_.size() - sent_);
            if (count > 0) sent_ += static_cast<std::size_t>(count);
            else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                fail(now, "serial write failed"); return;
            }
            if (sent_ < request_.size()) return;
        }
        std::uint8_t bytes[32];
        const auto count = ::read(fd_, bytes, sizeof(bytes));
        if (count < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) fail(now, "serial read failed");
            return;
        }
        if (count == 0) return;
        response_.insert(response_.end(), bytes, bytes + count);
        if (response_[0] != c_.slave ||
            (response_.size() >= 2 && response_[1] != 2 && response_[1] != 0x82)) {
            fail(now, "unexpected Modbus address/function"); return;
        }
        if (response_.size() < 2) return;
        const std::size_t expected = response_[1] == 0x82 ? 5 : 6;
        if (response_.size() < expected) return;
        if (response_.size() != expected || crc16(response_.data(), response_.size()) != 0) {
            fail(now, "invalid Modbus length/CRC"); return;
        }
        if (response_[1] == 0x82) {
            fail(now, "Modbus exception " + std::to_string(response_[2])); return;
        }
        if (response_[2] != 1 || response_[3] > 1) {
            fail(now, "invalid discrete input response"); return;
        }
        value = static_cast<int>(response_[3]) ^ static_cast<int>(c_.invert);
        sample = aviator::monotonic_us();
        if (status != "ok") {
            terminal_.clear();
            aviator::Logger::info("Photoelectric input connected: {}", c_.device);
        }
        status = "ok";
        pending_ = false;
        next_request_ = std::max(next_request_, sample + c_.gap_us);
    }
private:
    void close_port() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
        pending_ = false;
    }
    void fail(std::uint64_t now, const std::string& why) {
        if (status != why) {
            terminal_.clear();
            aviator::Logger::warn("Photoelectric input: {}", why);
        }
        status = why;
        close_port();
        retry_at_ = now + c_.retry_us;
    }
    const Config& c_;
    TerminalStatus& terminal_;
    int fd_ = -1;
    bool pending_ = false;
    std::size_t sent_ = 0;
    std::uint64_t retry_at_ = 0, next_request_ = 0, deadline_ = 0;
    std::array<std::uint8_t, 8> request_{};
    std::vector<std::uint8_t> response_;
};

std::string default_config() {
    const auto executable = std::filesystem::read_symlink("/proc/self/exe");
    const auto installed = executable.parent_path() / PHOTOELECTRIC_INSTALLED_CONFIG;
    return std::filesystem::exists(installed) ? installed.string() : PHOTOELECTRIC_SOURCE_CONFIG;
}
} // namespace

int main(int argc, char** argv) {
    try {
        std::string path;
        bool check = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                aviator::Logger::info("Usage: aviator_photoelectric [--config path] [--check-config]\n"
                    "Read one Modbus RTU discrete input and publish photoelectric.state continuously.");
                return 0;
            }
            if (arg == "--config" && i + 1 < argc && path.empty()) path = argv[++i];
            else if (arg == "--check-config") check = true;
            else throw std::runtime_error("unknown/incomplete argument: " + arg);
        }
        if (path.empty()) path = default_config();
        const auto c = load_config(path);
        aviator::print_startup("aviator_photoelectric", {{"Config", path}, {"Serial", c.device},
            {"Baud rate", std::to_string(c.baud)}, {"Input", "X" + std::to_string(c.address + 1)},
            {"PUB connect", c.endpoint}, {"PUB topic", "photoelectric.state"},
            {"Poll / PUB Hz", std::to_string(1000000 / c.poll_us) + " / " + std::to_string(1000000 / c.publish_us)}});
        if (check) return 0;
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        TerminalStatus terminal;
        Input input(c, terminal);
        zmq::context_t context(1);
        zmq::socket_t pub(context, zmq::socket_type::pub);
        aviator::configure(pub);
        pub.connect(c.endpoint);
        aviator::Message message;
        message.topic = aviator::Topic::photoelectric_state;
        message.header.publisher_id = c.publisher;
        message.header.session_id = aviator::new_instance_id();
        message.header.clock_id = aviator::local_clock_id();
        auto next_publish = aviator::monotonic_us();
        const auto publish = [&](std::uint64_t now, bool shutdown) {
            auto& h = message.header;
            ++h.sequence;
            h.timestamp = aviator::utc_us();
            h.sample_mono_us = input.sample;
            h.valid = !shutdown && input.valid(now);
            message.body = {{"channel", "X" + std::to_string(c.address + 1)}, {"value", input.value},
                {"status", shutdown ? "stopped" : (input.status == "ok" && !h.valid ? "stale" : input.status)}};
            std::string payload, error;
            require(aviator::encode(message, payload, error), "encode: " + error);
            aviator::send(pub, aviator::topic_name(message.topic), payload);
            terminal.show(message.body.at("channel").get<std::string>(), input.value,
                          h.valid, message.body.at("status").get<std::string>());
        };
        while (!stopping) {
            input.tick(aviator::monotonic_us());
            const auto now = aviator::monotonic_us();
            if (now >= next_publish) {
                publish(now, false);
                next_publish += ((now - next_publish) / c.publish_us + 1) * c.publish_us;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        publish(aviator::monotonic_us(), true); // Best effort: subscribers also check freshness.
        return 0;
    } catch (const std::exception& e) {
        aviator::Logger::error("aviator_photoelectric: {}", e.what());
        return 1;
    }
}

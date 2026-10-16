#include "simulator.hpp"
#include "clock.hpp"
#include "sdl_input.hpp"
#include "serial_port.hpp"
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
namespace flight_simulator {
namespace {
std::string position(int16_t value) {
    return value == rs422::invalid_position ? "unknown" : std::to_string(value / 10.0) + "%";
}
} // namespace
void run(const Config& config, int stop_fd) {
    using namespace rs422;
    SdlInput input(config);
    std::array<SerialPort, 2> ports{
        {SerialPort(config.rs422.channel_a), SerialPort(config.rs422.channel_b)}};
    std::array<Parser, 2> parsers{{Parser(true), Parser(true)}};
    std::array<uint64_t, 2> connections{}, received{};
    std::array<Uplink, 2> replies{};
    JoystickSample sample{{config.roll_axis, -32768, 32767, 0, config.invert_roll},
                          {config.pitch_axis, -32768, 32767, 0, config.invert_pitch}};
    CommandSender sender;
    uint64_t next = monotonic_us(), next_log = next, ticks = 0;
    uint8_t counter = 0;
    bool was_valid = false;
    const uint64_t timeout = config.rs422.receive_timeout_ms * 1000ULL;
    std::clog << "flight_simulator: aviator-bench-v1, 115200 8N1, 80 Hz\n";
    if (!config.simulator.send_a || !config.simulator.send_b || config.simulator.corrupt_every)
        std::clog << "Fault injection enabled: send A=" << config.simulator.send_a
                  << " B=" << config.simulator.send_b
                  << " corrupt A every=" << config.simulator.corrupt_every << " cycles\n";
    while (true) {
        auto now = monotonic_us();
        for (unsigned i = 0; i < 2; ++i)
            if (ports[i].connect(now) && connections[i] != ports[i].connections()) {
                connections[i] = ports[i].connections();
                received[i] = 0;
                parsers[i].clear();
                sender.reset();
            }
        pollfd fds[3]{{stop_fd, POLLIN, 0}, {ports[0].fd(), POLLIN, 0}, {ports[1].fd(), POLLIN, 0}};
        ::poll(fds, 3, now >= next ? 0 : 1);
        if (fds[0].revents)
            break;
        now = monotonic_us();
        for (unsigned i = 0; i < 2; ++i) {
            if (fds[i + 1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
                ports[i].close();
                received[i] = 0;
                continue;
            }
            if (fds[i + 1].revents & POLLIN) {
                Bytes bytes;
                if (!ports[i].read(bytes)) {
                    received[i] = 0;
                    continue;
                }
                for (const auto& frame : parsers[i].feed(bytes.data(), bytes.size())) {
                    Uplink up;
                    if (!decode(frame, up))
                        continue;
                    const uint8_t delta = up.counter - replies[i].counter;
                    if (received[i] && now - received[i] < timeout && (delta == 0 || delta >= 128))
                        continue;
                    replies[i] = up;
                    received[i] = now;
                }
            }
            ports[i].flush(now);
        }
        std::vector<unsigned> pressed;
        if (!input.poll(sample, pressed))
            break;
        now = monotonic_us();
        const bool valid = sample.fresh(now, config.input_timeout_ms * 1000ULL);
        if (!valid) {
            sender.reset();
            for (auto& port : ports)
                port.cancelOutput();
        }
        if (valid != was_valid) {
            std::clog << "Input=" << (valid ? "valid; transmitting" : "invalid; TX stopped")
                      << "\n";
            was_valid = valid;
        }
        for (const auto index : pressed)
            if (valid) {
                const auto op = operationCode(config.buttons.at(index));
                if (op)
                    std::clog << "Operation " << operationName(op) << ": "
                              << (sender.request(op) ? "10_FRAMES" : "BUSY_OR_NEUTRAL; press again")
                              << "\n";
            }
        now = monotonic_us();
        if (now >= next) {
            const auto missed = (now - next) / period_us;
            next += (missed + 1) * period_us;
            if (missed) {
                sender.reset();
                counter = uint8_t(counter + missed);
            }
            ++counter;
            ++ticks;
            if (sample.fresh(now, config.input_timeout_ms * 1000ULL)) {
                auto bytes = encode(Downlink{counter, sender.next(), quantize(sample.roll_value),
                                             quantize(sample.pitch_value)});
                if (config.simulator.send_b)
                    ports[1].send(bytes, now);
                if (config.simulator.corrupt_every && ticks % config.simulator.corrupt_every == 0)
                    bytes[8] ^= 1;
                if (config.simulator.send_a)
                    ports[0].send(bytes, now);
            }
        }
        if (now >= next_log) {
            next_log = now + 1000000;
            std::string title = "Flight simulator";
            for (unsigned i = 0; i < 2; ++i) {
                const bool fresh = received[i] && now - received[i] < timeout;
                const auto& f = replies[i];
                title +=
                    std::string(i ? " | B:" : " | A:") + (fresh ? stateName(f.state) : "STALE");
                std::ostringstream line;
                line << "SIM " << (i ? "B" : "A") << " " << (fresh ? "LIVE" : "STALE")
                     << " seq=" << unsigned(f.counter) << " selected=" << unsigned(f.selected)
                     << " comm=" << unsigned(f.communication) << " state=" << stateName(f.state)
                     << " echo_op=" << unsigned(f.operation) << " echo=(" << position(f.roll_echo)
                     << ", " << position(f.pitch_echo) << ") measured=(" << position(f.roll) << ", "
                     << position(f.pitch) << ") rear=" << unsigned(f.rear)
                     << " brake=" << unsigned(f.brake) << " fault=" << f.fault
                     << " previous=" << f.last_fault << " severity=" << unsigned(f.severity)
                     << " version=" << std::hex << std::uppercase << std::setfill('0')
                     << std::setw(4) << f.version << " checksum=" << std::setw(4) << f.checksum
                     << std::dec << " bad_bytes=" << parsers[i].rejected;
                std::clog << line.str() << "\n";
            }
            input.title(title);
        }
    }
}
} // namespace flight_simulator

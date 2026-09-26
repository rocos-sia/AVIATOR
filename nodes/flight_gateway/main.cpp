#include "gateway.hpp"
#include "transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
unsigned number(const std::string& text, unsigned maximum) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("expected nonnegative integer: " + text);
    const auto value = std::stoul(text);
    if (value > maximum) throw std::runtime_error("numeric option out of range");
    return static_cast<unsigned>(value);
}
void require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
}
int main(int argc, char** argv) {
    int device = -1, signals_fd = -1, lock_fd = -1, result = 0;
    try {
        std::string source = "joystick", path = "/dev/input/event0", core_session;
        std::string pub_endpoint = aviator::publish_endpoint, sub_endpoint = aviator::subscribe_endpoint;
        std::string lock = "/tmp/flight_gateway-" + std::to_string(getuid()) + ".lock";
        unsigned roll_axis = ABS_X, pitch_axis = ABS_Y, timeout_ms = 100;
        bool invert_roll = false, invert_pitch = false;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                std::cout << "Usage: flight_gateway [--device /dev/input/event0] [options]\n"
                    "Invalid/unavailable device: enter another evdev path at the prompt.\n"
                    "  --source joystick|rs422 (rs422 requires an external ICD; unavailable)\n"
                    "  --publish tcp://127.0.0.1:5555 --subscribe tcp://127.0.0.1:5556\n"
                    "  --roll-axis 0 --pitch-axis 1 --invert-roll --invert-pitch\n"
                    "  --input-timeout-ms 100 --core-session UUID --lock-file PATH\n"
                    "Publishes at 50 Hz. Core feedback is unconfigured without --core-session.\n";
                return 0;
            }
            if (key == "--invert-roll") { invert_roll = true; continue; }
            if (key == "--invert-pitch") { invert_pitch = true; continue; }
            require(i + 1 < argc, "missing option value: " + key);
            const std::string value = argv[++i];
            if (key == "--source") source = value;
            else if (key == "--device") path = value;
            else if (key == "--publish") pub_endpoint = value;
            else if (key == "--subscribe") sub_endpoint = value;
            else if (key == "--core-session") core_session = value;
            else if (key == "--lock-file") lock = value;
            else if (key == "--roll-axis") roll_axis = number(value, ABS_MAX);
            else if (key == "--pitch-axis") pitch_axis = number(value, ABS_MAX);
            else if (key == "--input-timeout-ms") timeout_ms = number(value, 100);
            else throw std::runtime_error("unknown option: " + key);
        }
        require(source == "joystick", source == "rs422" ?
                "RS422 wire format/baud/checksum/state encoding are not frozen; mode unavailable" : "unknown source");
        require(roll_axis != pitch_axis && timeout_ms > 0, "distinct axes and positive timeout required");
        require(pub_endpoint.rfind("tcp://", 0) == 0 && sub_endpoint.rfind("tcp://", 0) == 0 &&
                pub_endpoint != sub_endpoint, "distinct TCP endpoints required");
        // Resolve the device before blocking signals so Ctrl+C also works at
        // the terminal prompt. No ZMQ threads or instance lock exist yet.
        input_absinfo roll{}, pitch{};
        for (;;) {
            device = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            int clock_type = CLOCK_MONOTONIC;
            std::string reason;
            if (device < 0) reason = std::strerror(errno);
            else if (ioctl(device, EVIOCSCLOCKID, &clock_type) != 0)
                reason = "device must support monotonic evdev timestamps";
            else if (ioctl(device, EVIOCGABS(roll_axis), &roll) != 0 ||
                     ioctl(device, EVIOCGABS(pitch_axis), &pitch) != 0)
                reason = "selected absolute axes unavailable";
            else if (roll.minimum >= roll.maximum || pitch.minimum >= pitch.maximum)
                reason = "invalid device axis ranges";
            else break;
            if (device >= 0) { close(device); device = -1; }
            std::cerr << "flight_gateway: " << path << ": " << reason << '\n'
                      << "请输入 USB 摇杆设备路径（/dev/input/eventN，空行退出）：" << std::flush;
            if (!std::getline(std::cin, path)) throw std::runtime_error("no device path provided (EOF)");
            const auto first = path.find_first_not_of(" \t\r");
            if (first == std::string::npos) throw std::runtime_error("device selection cancelled");
            path = path.substr(first, path.find_last_not_of(" \t\r") - first + 1);
        }
        sigset_t signals;
        sigemptyset(&signals); sigaddset(&signals, SIGINT); sigaddset(&signals, SIGTERM);
        const auto mask_error = pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        require(mask_error == 0, "cannot block termination signals");
        signals_fd = signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC);
        require(signals_fd >= 0, "cannot create signal fd");
        lock_fd = open(lock.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        struct stat info{};
        require(lock_fd >= 0 && fstat(lock_fd, &info) == 0 && S_ISREG(info.st_mode) && info.st_uid == getuid(),
                "cannot open owned regular lock file");
        require(flock(lock_fd, LOCK_EX | LOCK_NB) == 0, "another flight_gateway holds the lock");
        flight_gateway::JoystickSample sample{
            {roll_axis, roll.minimum, roll.maximum, roll.value, invert_roll},
            {pitch_axis, pitch.minimum, pitch.maximum, pitch.value, invert_pitch}};
        const auto session = aviator::new_session_id(), clock = aviator::local_clock_id();
        std::unique_ptr<aviator::InputGuard> feedback;
        if (!core_session.empty()) {
            aviator::InputPolicy policy;
            policy.topic = aviator::Topic::flight_state;
            policy.publisher_id = "aviator_core";
            policy.session_id = core_session;
            policy.clock_id = clock;
            feedback = std::make_unique<aviator::InputGuard>(policy);
        }
        zmq::context_t context{1};
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        aviator::configure(pub); aviator::configure(sub);
        aviator::subscribe(sub, "flight.state");
        pub.connect(pub_endpoint); sub.connect(sub_endpoint);
        std::cout << "STARTED source=JOYSTICK session=" << session << " clock=" << clock
                  << " feedback=" << (feedback ? "STALE" : "UNCONFIGURED") << std::endl;
        aviator::ReceiveState receive_state;
        std::uint64_t sequence = 0, next_publish = aviator::monotonic_us();
        std::string error, payload, last_status = "STALE", system_state;
        bool running = true, last_valid = false;
        const auto publish = [&](std::uint64_t now) {
            require(sequence < aviator::max_json_integer, "sequence exhausted; restart required");
            auto message = flight_gateway::command(sample, session, clock, ++sequence, now,
                aviator::utc_us(), timeout_ms * 1000ULL);
            if (!aviator::encode(message, payload, error))
                throw std::runtime_error("encode: " + error);
            aviator::send(pub, "flight.command", payload);
            if (message.header.valid != last_valid) {
                last_valid = message.header.valid;
                std::cout << "input=" << (last_valid ? "VALID" : "INVALID") << std::endl;
            }
        };
        while (running) {
            const auto now = aviator::monotonic_us();
            const auto wait_ms = now >= next_publish ? 0 : (next_publish - now + 999) / 1000;
            zmq::pollitem_t items[]{{nullptr, signals_fd, ZMQ_POLLIN, 0},
                {nullptr, device, ZMQ_POLLIN, 0}, {sub.handle(), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 3, std::chrono::milliseconds(wait_ms));
            if (items[0].revents) { running = false; break; }
            if (device >= 0 && items[1].revents) {
                for (int i = 0; i < 128; ++i) {
                    input_event event{};
                    const auto n = read(device, &event, sizeof(event));
                    if (n == sizeof(event)) sample.update(event, aviator::monotonic_us());
                    else if (n < 0 && errno == EINTR) continue;
                    else if (n < 0 && errno == EAGAIN) break;
                    else { sample.failed = true; sample.valid = false; }
                    if (sample.failed) {
                        std::cerr << "flight_gateway: input lost/corrupt; restart required\n";
                        close(device); device = -1;
                        break;
                    }
                }
            }
            for (int i = 0; i < 64; ++i) {
                aviator::WireMessage wire;
                const auto received = aviator::receive(sub, receive_state, wire, error);
                if (received == aviator::ReceiveResult::empty) break;
                aviator::Message state;
                if (received == aviator::ReceiveResult::received && feedback &&
                    aviator::decode(wire.topic, wire.payload, state, error) &&
                    flight_gateway::valid_state_summary(state) &&
                    feedback->accept(state, aviator::monotonic_us(), error))
                    system_state = state.body.at("system").at("state").get<std::string>();
            }
            const auto current = aviator::monotonic_us();
            if (feedback) {
                const auto status = feedback->expired(current) ? "STALE" : system_state;
                if (status != last_status) {
                    std::cout << "feedback=" << status << std::endl;
                    last_status = status;
                }
            }
            if (current >= next_publish) {
                publish(current);
                next_publish += ((current - next_publish) / 20000 + 1) * 20000;
            }
        }
        sample.valid = false;
        publish(aviator::monotonic_us()); // Best effort; PUB/SUB has no delivery ACK.
    } catch (const std::exception& error) {
        std::cerr << "flight_gateway: " << error.what() << '\n'; result = 1;
    }
    if (device >= 0) close(device);
    if (signals_fd >= 0) close(signals_fd);
    if (lock_fd >= 0) close(lock_fd);
    return result;
}

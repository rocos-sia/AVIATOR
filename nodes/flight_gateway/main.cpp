#include "gateway.hpp"
#include "config.hpp"
#include "service.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include <map>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
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
void require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
}
int main(int argc, char** argv) {
    int device = -1, signals_fd = -1, lock_fd = -1, result = 0;
    const bool inline_output = isatty(STDOUT_FILENO);
    bool value_line = false;
    const auto finish_value_line = [&] {
        if (value_line) { std::cout << std::endl; value_line = false; }
    };
    try {
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
            std::cout << "Usage: flight_gateway\n"
                         "Configuration: config/flight.yaml (installed: share/aviator/config/flight.yaml).\n"
                         "Edit the YAML file for device, endpoints, axes and 11 button mappings.\n"
                         "Publishes at 50 Hz; restart after configuration changes.\n";
            return 0;
        }
        require(argc == 1, "startup options are not supported; edit config/flight.yaml");
        const auto config_path = flight_gateway::default_config_path();
        const auto config = flight_gateway::load_config(config_path);
        const auto& path = config.device;
        const auto& core_session = config.core_session;
        const auto& pub_endpoint = config.publish;
        const auto& sub_endpoint = config.subscribe;
        const auto& service_endpoint = config.service;
        const auto& lock = config.lock_file;
        const auto roll_axis = config.roll_axis, pitch_axis = config.pitch_axis;
        const auto timeout_ms = config.input_timeout_ms, service_timeout_ms = config.service_timeout_ms;
        const auto invert_roll = config.invert_roll, invert_pitch = config.invert_pitch;
        flight_gateway::JoystickButtons buttons;
        buttons.operations = config.buttons;
        std::cout << "Configuration: " << config_path << std::endl;
        input_absinfo roll{}, pitch{};
        device = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        require(device >= 0, "cannot open " + path + ": " + std::strerror(errno));
        int clock_type = CLOCK_MONOTONIC;
        require(ioctl(device, EVIOCSCLOCKID, &clock_type) == 0,
                "device must support monotonic evdev timestamps: " + path);
        require(ioctl(device, EVIOCGABS(roll_axis), &roll) == 0 &&
                    ioctl(device, EVIOCGABS(pitch_axis), &pitch) == 0,
                "selected absolute axes unavailable: " + path);
        require(roll.minimum < roll.maximum && pitch.minimum < pitch.maximum,
                "invalid device axis ranges: " + path);
        std::array<unsigned char, (KEY_MAX + 8) / 8> keys{}, held_keys{};
        require(ioctl(device, EVIOCGBIT(EV_KEY, keys.size()), keys.data()) >= 0 &&
                    ioctl(device, EVIOCGKEY(held_keys.size()), held_keys.data()) >= 0,
                "cannot query joystick buttons");
        unsigned button_count = 0;
        for (unsigned code = BTN_MISC; code <= KEY_MAX && button_count < 11; ++code) {
            if (!(keys[code / 8] & (1u << (code % 8))))
                continue;
            buttons.codes[button_count] = code;
            buttons.held[button_count] = held_keys[code / 8] & (1u << (code % 8));
            std::cout << "button=" << button_count + 1 << " evdev_code=" << code << " event="
                      << (buttons.operations[button_count].empty()
                              ? "none"
                              : buttons.operations[button_count])
                      << '\n';
            ++button_count;
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
        const auto session = aviator::new_instance_id(), clock = aviator::local_clock_id();
        flight_gateway::CoreFeedback feedback(core_session, clock);
        zmq::context_t context{1};
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        aviator::configure(pub); aviator::configure(sub);
        aviator::subscribe(sub, "flight.state");
        pub.connect(pub_endpoint); sub.connect(sub_endpoint);
        std::cout << "STARTED source=JOYSTICK session=" << session << " clock=" << clock
                  << " feedback=WAITING" << std::endl;
        aviator::print_startup(
            "flight_gateway",
            {{"Config", config_path},
            {"Device", path},
             {"PUB connect", pub_endpoint},
             {"PUB topics",
              "flight.command (50 Hz); record.service.request / record.service.reply"},
             {"SUB connect", sub_endpoint},
             {"SUB topics", "flight.state"},
             {"Feedback", "Waiting for valid Core publisher status; no session pinning"},
             {"Session", session},
             {"Transport", "Async connect; bus connectivity is not yet confirmed."},
             {"Exit", "Ctrl+C"}});
        zmq::socket_t service(context, zmq::socket_type::dealer);
        aviator::configure(service, {16, 16, 0});
        service.set(zmq::sockopt::immediate, 1); // Never queue commands for an absent server.
        service.set(zmq::sockopt::maxmsgsize,
                    static_cast<std::int64_t>(aviator::max_payload_bytes));
        service.connect(service_endpoint);
        std::cout << "Service DEALER connect=" << service_endpoint
                  << " timeout_ms=" << service_timeout_ms << '\n';
        std::map<std::string, nlohmann::json> pending_requests;
        aviator::ReceiveState service_receiving;
        const auto request_button = [&](unsigned index) {
            finish_value_line();
            if (buttons.operations[index].empty()) {
                std::cout << "button=" << index + 1 << " unassigned\n";
                return;
            }
            if (pending_requests.size() >= 11) {
                std::cerr << "service pending limit; button request not sent\n";
                return;
            }
            nlohmann::json parameters = {{"source", "JOYSTICK"}, {"button", index + 1}};
            auto request = aviator::make_service_request("flight_gateway", session, "aviator_core",
                                                         buttons.operations[index], parameters,
                                                         service_timeout_ms);
            // Record the attempted request even when no Core is connected. Optional observation
            // metadata is not part of the service envelope sent to Core.
            const bool sent = feedback.requestsReady(aviator::monotonic_us()) && aviator::send_service(service, request);
            if (feedback.session().empty())
                std::cerr << "No valid Core feedback yet; button not sent, press again after service_ready=1\n";
            else if (!feedback.requestsReady(aviator::monotonic_us()))
                std::cerr << "Core feedback stale or Gateway input not confirmed; button not sent, "
                             "press again after service_ready=1\n";
            auto record = request;
            record["gateway_observation"] = sent ? "QUEUED" : "NOT_SENT";
            if (!aviator::send(pub, aviator::service_request_topic, record.dump()))
                std::cerr << "service request record not queued\n";
            std::cout << "button=" << index + 1 << " operation=" << buttons.operations[index]
                      << " request_id=" << request.at("request_id")
                      << (sent ? " QUEUED" : " NOT_SENT") << '\n';
            if (sent)
                pending_requests.emplace(request.at("request_id").get<std::string>(),
                                         std::move(request));
        };
        aviator::ReceiveState receive_state;
        std::uint64_t sequence = 0, next_publish = aviator::monotonic_us();
        std::string error, payload, last_status = "STALE", system_state;
        std::string feedback_error;
        bool last_service_ready = false;
        bool running = true;
        std::uint64_t next_print = 0;
        const auto publish = [&](std::uint64_t now) {
            require(sequence < aviator::max_json_integer, "sequence exhausted; restart required");
            auto message = flight_gateway::command(sample, session, clock, ++sequence, now,
                aviator::utc_us(), timeout_ms * 1000ULL);
            if (!aviator::encode(message, payload, error))
                throw std::runtime_error("encode: " + error);
            aviator::send(pub, "flight.command", payload);
            if (running && now >= next_print) {
                std::cout << (inline_output ? "\r\033[2K" : "") << std::fixed << std::setprecision(4)
                          << "roll=" << sample.roll_value << " pitch=" << sample.pitch_value;
                if (inline_output) { std::cout << std::flush; value_line = true; }
                else std::cout << std::endl;
                next_print = now + 100000; // Limit console output to 10 Hz.
            }
        };
        while (running) {
            const auto now = aviator::monotonic_us();
            const auto wait_ms = now >= next_publish ? 0 : (next_publish - now + 999) / 1000;
            zmq::pollitem_t items[]{{nullptr, signals_fd, ZMQ_POLLIN, 0},
                                    {nullptr, device, ZMQ_POLLIN, 0},
                                    {sub.handle(), 0, ZMQ_POLLIN, 0},
                                    {service.handle(), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 4, std::chrono::milliseconds(wait_ms));
            if (items[0].revents) { running = false; break; }
            bool input_drained = true;
            if (device >= 0 && items[1].revents) {
                input_drained = false;
                for (int i = 0; i < 128; ++i) {
                    input_event event{};
                    const auto n = read(device, &event, sizeof(event));
                    if (n == sizeof(event)) {
                        const auto event_now = aviator::monotonic_us();
                        sample.update(event, event_now);
                        for (auto index : buttons.update(event, event_now, !sample.failed))
                            request_button(index);
                    } else if (n < 0 && errno == EINTR)
                        continue;
                    else if (n < 0 && errno == EAGAIN) { input_drained = true; break; }
                    else { sample.invalidate(); }
                    if (sample.failed) {
                        finish_value_line();
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
                const bool discovering = feedback.session().empty();
                if (received == aviator::ReceiveResult::received &&
                    aviator::decode(wire.topic, wire.payload, state, error) &&
                    feedback.accept(state, aviator::monotonic_us(), error)) {
                    if (discovering) {
                        finish_value_line();
                        std::cout << "Received aviator_core startup marker=" << feedback.session() << std::endl;
                    }
                    system_state = state.body.at("system").at("state").get<std::string>();
                    feedback_error.clear();
                } else if (received != aviator::ReceiveResult::empty) {
                    feedback_error = error;
                }
            }
            for (unsigned i = 0; i < 32; ++i) {
                std::string raw;
                nlohmann::json reply;
                const auto received =
                    aviator::receive_service(service, service_receiving, raw, reply, error);
                if (received == aviator::ReceiveResult::empty)
                    break;
                if (received == aviator::ReceiveResult::rejected) {
                    finish_value_line();
                    std::cerr << "invalid service reply: " << error << '\n';
                    continue;
                }
                if (reply.at("msg_type") != "ServiceReply")
                    continue;
                // Preserve even late/unmatched replies for diagnosis; never change state locally.
                aviator::send(pub, aviator::service_reply_topic, raw);
                const auto it = pending_requests.find(reply.at("request_id").get<std::string>());
                finish_value_line();
                if (it == pending_requests.end() ||
                    !aviator::matches_service_reply(it->second, reply) ||
                    aviator::monotonic_us() -
                            it->second.at("issued_mono_us").get<std::uint64_t>() >=
                        service_timeout_ms * 1000ULL) {
                    std::cerr << "late/unmatched service reply: " << reply.at("request_id") << '\n';
                    continue;
                }
                std::cout << "service reply=" << reply.dump() << '\n';
                pending_requests.erase(it);
            }
            const auto current = aviator::monotonic_us();
            for (auto it = pending_requests.begin(); it != pending_requests.end();) {
                const auto& request = it->second;
                if (current - request.at("issued_mono_us").get<std::uint64_t>() >=
                    service_timeout_ms * 1000ULL) {
                    finish_value_line();
                    std::cerr << "service timeout UNKNOWN request_id=" << it->first
                              << "; no automatic retry\n";
                    auto record = request;
                    record["gateway_observation"] = "TIMEOUT_UNKNOWN";
                    aviator::send(pub, aviator::service_request_topic, record.dump());
                    it = pending_requests.erase(it);
                } else
                    ++it;
            }
            if (!feedback.session().empty()) {
                const auto status = feedback.expired(current) ? "STALE" : system_state;
                if (status != last_status) {
                    finish_value_line();
                    std::cout << "feedback=" << status << std::endl;
                    if (status == "STALE")
                        std::cerr << "No accepted Core feedback within 100 ms; last rejection="
                                  << (feedback_error.empty() ? "none (check Core/bus publication)" : feedback_error)
                                  << '\n';
                    last_status = status;
                }
            }
            const bool service_ready = feedback.requestsReady(current);
            if (service_ready != last_service_ready) {
                finish_value_line();
                std::cout << "service_ready=" << service_ready << std::endl;
                last_service_ready = service_ready;
            }
            if (current >= next_publish) {
                // Query kernel device availability every 20 ms. This is not a new hardware sample.
                // Only renew the lease after draining events and completing their SYN_REPORT.
                if (device >= 0 && !sample.failed) {
                    input_absinfo current_roll{}, current_pitch{};
                    if (ioctl(device, EVIOCGABS(roll_axis), &current_roll) != 0 ||
                        ioctl(device, EVIOCGABS(pitch_axis), &current_pitch) != 0) {
                        sample.invalidate();
                        finish_value_line();
                        std::cerr << "flight_gateway: device check failed: " << std::strerror(errno)
                                  << "; restart required\n";
                        close(device); device = -1;
                    } else if (current_roll.minimum != sample.roll.minimum ||
                               current_roll.maximum != sample.roll.maximum ||
                               current_pitch.minimum != sample.pitch.minimum ||
                               current_pitch.maximum != sample.pitch.maximum ||
                               current_roll.value < current_roll.minimum || current_roll.value > current_roll.maximum ||
                               current_pitch.value < current_pitch.minimum || current_pitch.value > current_pitch.maximum) {
                        sample.invalidate();
                        finish_value_line();
                        std::cerr << "flight_gateway: invalid/changed axis range; restart required\n";
                        close(device); device = -1;
                    } else if (input_drained) {
                        const auto checked = aviator::monotonic_us();
                        sample.initializePosition(current_roll.value, current_pitch.value, checked);
                        sample.deviceChecked(checked);
                    }
                }
                publish(aviator::monotonic_us());
                next_publish += ((current - next_publish) / 20000 + 1) * 20000;
            }
        }
        sample.valid = false;
        publish(aviator::monotonic_us()); // Best effort; PUB/SUB has no delivery ACK.
    } catch (const std::exception& error) {
        finish_value_line();
        std::cerr << "flight_gateway: " << error.what() << '\n'; result = 1;
    }
    finish_value_line();
    if (device >= 0) close(device);
    if (signals_fd >= 0) close(signals_fd);
    if (lock_fd >= 0) close(lock_fd);
    return result;
}

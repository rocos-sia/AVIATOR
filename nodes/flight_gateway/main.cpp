#include "Logger.hpp"
#include "gateway.hpp"
#include "config.hpp"
#include "sdl_input.hpp"
#include "service.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include <map>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
void require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
}
int main(int argc, char** argv) {
    int signals_fd = -1, lock_fd = -1, result = 0;
    try {
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
            aviator::Logger::info("Usage: flight_gateway\n"
                "Configuration: config/flight.yaml (installed: share/aviator/config/flight.yaml).\n"
                "Edit the YAML file for joystick/rs422 source, SDL device, speeds and button mappings.\n"
                "Publishes at 50 Hz; restart after configuration changes.");
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
        aviator::Logger::info("Configuration: {}", config_path);
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
        // RS422 is reserved; reject before initializing any SDL device or window.
        require(config.source == "joystick",
                "RS422 mode not implemented; joystick and keyboard disabled");
        flight_gateway::SdlInput input(config);
        flight_gateway::JoystickSample sample{
            {roll_axis, -32768, 32767, 0, invert_roll},
            {pitch_axis, -32768, 32767, 0, invert_pitch}};
        const auto session = aviator::new_instance_id(), clock = aviator::local_clock_id();
        flight_gateway::CoreFeedback feedback(core_session, clock);
        zmq::context_t context{1};
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        aviator::configure(pub); aviator::configure(sub);
        aviator::subscribe(sub, "flight.state");
        pub.connect(pub_endpoint); sub.connect(sub_endpoint);
        aviator::Logger::info("STARTED input={} source=JOYSTICK session={} clock={} feedback=WAITING",
            config.source, session, clock);
        aviator::print_startup(
            "flight_gateway",
            {{"Config", config_path},
             {"Input", config.source},
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
        aviator::Logger::info("Service DEALER connect={} timeout_ms={}", service_endpoint, service_timeout_ms);
        std::map<std::string, nlohmann::json> pending_requests;
        aviator::ReceiveState service_receiving;
        const auto request_button = [&](unsigned index) {
            if (buttons.operations[index].empty()) {
                aviator::Logger::info("button={} unassigned", index + 1);
                return;
            }
            if (pending_requests.size() >= 11) {
                aviator::Logger::warn("service pending limit; button request not sent");
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
                aviator::Logger::warn("No valid Core feedback yet; button not sent, press again after service_ready=1");
            else if (!feedback.requestsReady(aviator::monotonic_us()))
                aviator::Logger::warn(
                    "Core feedback stale or Gateway input not confirmed; button not sent, press again after "
                    "service_ready=1");
            auto record = request;
            record["gateway_observation"] = sent ? "QUEUED" : "NOT_SENT";
            if (!aviator::send(pub, aviator::service_request_topic, record.dump()))
                aviator::Logger::warn("service request record not queued");
            aviator::Logger::info("button={} operation={} request_id={}{}",
                index + 1, buttons.operations[index], request.at("request_id").dump(),
                (sent ? " QUEUED" : " NOT_SENT"));
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
                aviator::Logger::info("roll={:.4f} pitch={:.4f}", sample.roll_value, sample.pitch_value);
                next_print = now + 100000; // Limit console output to 10 Hz.
            }
        };
        while (running) {
            const auto now = aviator::monotonic_us();
            const auto wait_ms = now >= next_publish ? 0 : (next_publish - now + 999) / 1000;
            zmq::pollitem_t items[]{{nullptr, signals_fd, ZMQ_POLLIN, 0},
                                    {sub.handle(), 0, ZMQ_POLLIN, 0},
                                    {service.handle(), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 3, std::chrono::milliseconds(wait_ms));
            if (items[0].revents) { running = false; break; }
            std::vector<unsigned> pressed;
            running = input.poll(sample, pressed);
            if (!running) break;
            for (const auto index : pressed)
                if (sample.fresh(aviator::monotonic_us(), timeout_ms * 1000ULL)) request_button(index);
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
                        aviator::Logger::info("Received aviator_core startup marker={}", feedback.session());
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
                    aviator::Logger::error("invalid service reply: {}", error);
                    continue;
                }
                if (reply.at("msg_type") != "ServiceReply")
                    continue;
                // Preserve even late/unmatched replies for diagnosis; never change state locally.
                aviator::send(pub, aviator::service_reply_topic, raw);
                const auto it = pending_requests.find(reply.at("request_id").get<std::string>());
                if (it == pending_requests.end() ||
                    !aviator::matches_service_reply(it->second, reply) ||
                    aviator::monotonic_us() -
                            it->second.at("issued_mono_us").get<std::uint64_t>() >=
                        service_timeout_ms * 1000ULL) {
                    aviator::Logger::warn("late/unmatched service reply: {}", reply.at("request_id").dump());
                    continue;
                }
                aviator::Logger::info("service reply={}", reply.dump());
                pending_requests.erase(it);
            }
            const auto current = aviator::monotonic_us();
            for (auto it = pending_requests.begin(); it != pending_requests.end();) {
                const auto& request = it->second;
                if (current - request.at("issued_mono_us").get<std::uint64_t>() >=
                    service_timeout_ms * 1000ULL) {
                    aviator::Logger::warn("service timeout UNKNOWN request_id={}; no automatic retry", it->first);
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
                    aviator::Logger::info("feedback={}", status);
                    if (status == "STALE")
                        aviator::Logger::warn("No accepted Core feedback within 100 ms; last rejection={}",
                            (feedback_error.empty() ? "none (check Core/bus publication)" : feedback_error));
                    last_status = status;
                }
            }
            const bool service_ready = feedback.requestsReady(current);
            if (service_ready != last_service_ready) {
                aviator::Logger::info("service_ready={}", static_cast<int>(service_ready));
                last_service_ready = service_ready;
            }
            if (current >= next_publish) {
                publish(aviator::monotonic_us());
                next_publish += ((current - next_publish) / 20000 + 1) * 20000;
            }
        }
        sample.valid = false;
        publish(aviator::monotonic_us()); // Best effort; PUB/SUB has no delivery ACK.
    } catch (const std::exception& error) {
        aviator::Logger::error("flight_gateway: {}", error.what()); result = 1;
    }
    if (signals_fd >= 0) close(signals_fd);
    if (lock_fd >= 0) close(lock_fd);
    return result;
}

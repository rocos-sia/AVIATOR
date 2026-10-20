#include "rs422_runtime.hpp"
#include "Logger.hpp"
#include "gateway.hpp"
#include "release_identity.hpp"
#include "sdl_input.hpp"
#include "serial_port.hpp"
#include "service.hpp"
#include <algorithm>
#include <map>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>

namespace flight_gateway {
namespace {
using namespace rs422;
// Local Core services remain asynchronous and bounded, identical to joystick service semantics.
class CoreClient {
  public:
    explicit CoreClient(const Config& c)
        : config_(c), session_(aviator::new_instance_id()), clock_(aviator::local_clock_id()),
          feedback_("", clock_) {
        aviator::configure(pub_);
        aviator::configure(sub_);
        aviator::configure(service_, {16, 16, 0});
        aviator::subscribe(sub_, "flight.state");
        pub_.connect(c.publish);
        sub_.connect(c.subscribe);
        service_.set(zmq::sockopt::immediate, 1);
        service_.set(zmq::sockopt::maxmsgsize, int64_t(aviator::max_payload_bytes));
        service_.connect(c.service);
    }
    void poll(uint64_t now) {
        std::string error;
        for (unsigned i = 0; i < 64; ++i) {
            aviator::WireMessage wire;
            aviator::Message candidate;
            const auto r = aviator::receive(sub_, state_rx_, wire, error);
            if (r == aviator::ReceiveResult::empty)
                break;
            if (r == aviator::ReceiveResult::received &&
                aviator::decode(wire.topic, wire.payload, candidate, error) &&
                feedback_.accept(candidate, now, error))
                state_ = std::move(candidate);
        }
        for (unsigned i = 0; i < 32; ++i) {
            std::string raw;
            nlohmann::json reply;
            const auto r = aviator::receive_service(service_, service_rx_, raw, reply, error);
            if (r == aviator::ReceiveResult::empty)
                break;
            if (r != aviator::ReceiveResult::received || reply.at("msg_type") != "ServiceReply")
                continue;
            aviator::send(pub_, aviator::service_reply_topic, raw);
            const auto it = pending_.find(reply.at("request_id").get<std::string>());
            if (it != pending_.end() && aviator::matches_service_reply(it->second, reply) &&
                now - it->second.at("issued_mono_us").get<uint64_t>() <
                    config_.service_timeout_ms * 1000ULL) {
                aviator::Logger::info("RS422 Core reply={}", raw);
                pending_.erase(it);
            }
        }
        for (auto it = pending_.begin(); it != pending_.end();) {
            if (now - it->second.at("issued_mono_us").get<uint64_t>() >=
                config_.service_timeout_ms * 1000ULL) {
                auto record = it->second;
                record["gateway_observation"] = "TIMEOUT_UNKNOWN";
                aviator::send(pub_, aviator::service_request_topic, record.dump());
                aviator::Logger::warn("RS422 operation outcome UNKNOWN request_id={}; no retry",
                                      it->first);
                it = pending_.erase(it);
            } else
                ++it;
        }
    }
    void publish(const DualReceiver& input, const std::array<Bytes, 2>& raw_frames,
                 const std::array<Parser, 2>& parsers,
                 const std::array<nlohmann::json, 2>& last_operations,
                 uint64_t now, bool shutdown = false) {
        aviator::Message m;
        m.topic = aviator::Topic::flight_command;
        // Invalid snapshots also carry a fresh revocation time so InputGuard can revoke
        // immediately.
        const bool valid = !shutdown && input.valid(now);
        m.header = {"1.0",  ++sequence_,      aviator::utc_us(), valid ? input.sampleTime() : now,
                    clock_, "flight_gateway", session_,          valid};
        m.body = {
            {"source", "FLIGHT"},
            {"control",
             {{"roll", input.target().roll / 1000.0}, {"pitch", input.target().pitch / 1000.0}}}};
        // Optional observation only. Preserve received bytes (including reserved bytes),
        // never reconstruct a frame from the normalized control values.
        auto& observation = m.body["rs422"];
        observation = {{"format", "aviator-downlink-32-v1"},
                       {"selected_channel", input.selected() < 0 ? nlohmann::json(nullptr)
                                                                 : nlohmann::json(input.selected() ? "B" : "A")},
                       {"blocked", input.blocked()},
                       {"receive_timeout_ms", config_.rs422.receive_timeout_ms},
                       {"channels", nlohmann::json::array()}};
        for (unsigned i = 0; i < 2; ++i)
            observation["channels"].push_back(
                {{"channel", i ? "B" : "A"},
                 {"raw_bytes", raw_frames[i]},
                 {"received_mono_us", raw_frames[i].empty() ? 0 : input.channels[i].received},
                 {"healthy", !shutdown && input.healthy(i, now)},
                 {"accepted_frames", input.channels[i].generation},
                 {"rejected_bytes", parsers[i].rejected},
                 {"last_operation", last_operations[i]}});
        std::string raw, error;
        if (!aviator::encode(m, raw, error))
            throw std::runtime_error(error);
        aviator::send(pub_, "flight.command", raw);
    }
    std::string operation(uint8_t code) {
        const auto req = aviator::make_service_request(
            "flight_gateway", session_, "aviator_core", operationName(code),
            {{"source", "FLIGHT"}, {"operation_code", code}}, config_.service_timeout_ms);
        const bool sent =
            pending_.size() < 11 && feedback_.requestsReady(aviator::monotonic_us()) &&
            state_.body.at("system").value("input_source", std::string()) == "FLIGHT" &&
            aviator::send_service(service_, req);
        auto record = req;
        record["gateway_observation"] = sent ? "QUEUED" : "NOT_SENT";
        aviator::send(pub_, aviator::service_request_topic, record.dump());
        aviator::Logger::info("RS422 operation={} {} request_id={}", operationName(code),
                              sent ? "QUEUED" : "NOT_SENT", req.at("request_id").dump());
        if (sent)
            pending_.emplace(req.at("request_id").get<std::string>(), req);
        return req.at("request_id").get<std::string>();
    }
    void telemetry(Uplink& up, uint64_t now) const {
        if (!feedback_.expired(now))
            populateTelemetry(up, state_, now);
    }

  private:
    Config config_;
    std::string session_, clock_;
    CoreFeedback feedback_;
    aviator::Message state_;
    zmq::context_t context_{1};
    zmq::socket_t pub_{context_, zmq::socket_type::pub}, sub_{context_, zmq::socket_type::sub},
        service_{context_, zmq::socket_type::dealer};
    aviator::ReceiveState state_rx_, service_rx_;
    uint64_t sequence_ = 0;
    std::map<std::string, nlohmann::json> pending_;
};
std::string position(int16_t v) {
    return v == invalid_position ? "unknown" : std::to_string(v / 10.0) + "%";
}
} // namespace
void runRs422(const Config& config, int signals_fd) {
    if (config.rs422.profile != "aviator-bench-v1")
        throw std::runtime_error(
            "RS422 requires explicit rs422.profile: aviator-bench-v1 and two serial paths");
    const bool simulator = config.role == "flight_simulator";
    std::unique_ptr<SdlInput> input;
    std::unique_ptr<CoreClient> core;
    ReleaseIdentity identity{};
    if (simulator)
        input = std::make_unique<SdlInput>(config);
    else {
        identity = releaseIdentity(config.rs422);
        core = std::make_unique<CoreClient>(config);
    }
    std::array<SerialPort, 2> ports{
        {SerialPort(config.rs422.channel_a), SerialPort(config.rs422.channel_b)}};
    std::array<Parser, 2> parsers{{Parser(simulator), Parser(simulator)}};
    std::array<Bytes, 2> raw_frames;
    std::array<nlohmann::json, 2> last_operations;
    std::array<uint64_t, 2> connections{}, received{};
    std::array<uint8_t, 2> observed_operations{};
    std::array<Uplink, 2> replies{};
    const uint64_t timeout = config.rs422.receive_timeout_ms * 1000ULL;
    DualReceiver receiver(timeout, config.rs422.recovery_frames, config.rs422.switch_max_step);
    CommandSender sender;
    JoystickSample sample{{config.roll_axis, -32768, 32767, 0, config.invert_roll},
                          {config.pitch_axis, -32768, 32767, 0, config.invert_pitch}};
    uint64_t next = aviator::monotonic_us(), next_log = next, next_pub = next, ticks = 0;
    uint8_t counter = 0, echo_operation = 0;
    bool running = true, was_valid = false;
    int last_selected = -2;
    aviator::Logger::info(
        "STARTED role={} profile={} serial=80Hz Core=50Hz; unknown fields use bench sentinels",
        config.role, config.rs422.profile);
    if (simulator &&
        (!config.simulator.send_a || !config.simulator.send_b || config.simulator.corrupt_every))
        aviator::Logger::warn("SIMULATOR FAULT INJECTION enabled: A={} B={} corrupt_A_every={}",
                              config.simulator.send_a, config.simulator.send_b,
                              config.simulator.corrupt_every);
    while (running) {
        auto now = aviator::monotonic_us();
        for (unsigned i = 0; i < 2; ++i) {
            if (ports[i].connect(now) && connections[i] != ports[i].connections()) {
                connections[i] = ports[i].connections();
                parsers[i].clear();
                received[i] = 0;
                receiver.disconnect(i);
                raw_frames[i].clear();
                observed_operations[i] = 0;
                sender.reset();
            }
        }
        pollfd fds[3]{
            {signals_fd, POLLIN, 0}, {ports[0].fd(), POLLIN, 0}, {ports[1].fd(), POLLIN, 0}};
        // 1 ms maximum sleep keeps SDL and Core services responsive; serial cadence uses absolute
        // deadlines.
        ::poll(fds, 3, now >= next ? 0 : 1);
        if (fds[0].revents)
            break;
        now = aviator::monotonic_us();
        for (unsigned i = 0; i < 2; ++i) {
            if (fds[i + 1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
                ports[i].close();
                receiver.disconnect(i);
                received[i] = 0;
                continue;
            }
            if (fds[i + 1].revents & POLLIN) {
                Bytes bytes;
                if (!ports[i].read(bytes)) {
                    receiver.disconnect(i);
                    received[i] = 0;
                    continue;
                }
                const auto frames = parsers[i].feed(bytes.data(), bytes.size());
                for (const auto& frame : frames) {
                    if (simulator) {
                        Uplink up;
                        if (!decode(frame, up))
                            continue;
                        const uint8_t delta = up.counter - replies[i].counter;
                        if (received[i] && now - received[i] < timeout &&
                            (delta == 0 || delta >= 128))
                            continue;
                        replies[i] = up;
                        received[i] = now;
                    } else {
                        Downlink down;
                        if (decode(frame, down) && receiver.receive(i, down, now)) {
                            raw_frames[i] = frame;
                            if (down.operation) {
                                auto& saved = last_operations[i];
                                // Retain every accepted nonzero frame before 50 Hz sampling.
                                // A neutral interval starts a new operation even for the same code.
                                if (saved.is_null() || down.operation != observed_operations[i] ||
                                    now - saved.at("received_mono_us").get<uint64_t>() >= timeout)
                                    saved = {{"request_id", nullptr}};
                                saved["raw_bytes"] = frame;
                                saved["received_mono_us"] = now;
                                saved["received_utc_us"] = aviator::utc_us();
                            }
                            if (down.operation != observed_operations[i]) {
                                if (down.operation)
                                    aviator::Logger::info(
                                        "RS422 received channel={} operation={} counter={} (awaiting confirmation)",
                                        i ? "B" : "A", operationName(down.operation), down.counter);
                                observed_operations[i] = down.operation;
                            }
                        }
                    }
                }
            }
            ports[i].flush(now);
        }
        if (core) {
            core->poll(now);
            const auto operation = receiver.update(now);
            const bool valid = receiver.valid(now);
            if (!valid)
                echo_operation = 0;
            if (now >= next_pub || valid != was_valid) {
                core->publish(receiver, raw_frames, parsers, last_operations, now);
                next_pub = now + 20000;
            }
            was_valid = valid;
            if (operation && valid) {
                echo_operation = operation;
                const auto request_id = core->operation(operation);
                auto& saved = last_operations[receiver.selected()];
                if (!saved.is_null() && saved.at("raw_bytes").at(3) == operation)
                    saved["request_id"] = request_id;
            }
            if (receiver.selected() != last_selected) {
                last_selected = receiver.selected();
                aviator::Logger::info("RS422 selected={} valid={}", last_selected, valid);
            }
        }
        std::vector<unsigned> pressed;
        if (input) {
            running = input->poll(sample, pressed);
            if (!running)
                break;
            const bool valid =
                sample.fresh(aviator::monotonic_us(), config.input_timeout_ms * 1000ULL);
            if (!valid) {
                sender.reset();
                for (auto& port : ports)
                    port.cancelOutput();
            }
            if (valid != was_valid) {
                aviator::Logger::info("Simulator input={} downlink={}", valid ? "valid" : "invalid",
                                      valid ? "enabled" : "stopped");
                was_valid = valid;
            }
            for (auto index : pressed)
                if (valid) {
                    const auto code = operationCode(config.buttons.at(index));
                    if (code)
                        aviator::Logger::info(
                            "Simulator operation={} {}", operationName(code),
                            sender.request(code) ? "10_FRAMES" : "BUSY_OR_NEUTRAL; press again");
                }
        }
        now = aviator::monotonic_us();
        if (now >= next) {
            const auto missed = (now - next) / period_us;
            next += (missed + 1) * period_us;
            // No catch-up bursts. A stalled command train must be started again by the operator.
            if (missed) {
                sender.reset();
                counter = uint8_t(counter + missed);
            }
            ++counter;
            ++ticks;
            if (simulator) {
                if (sample.fresh(now, config.input_timeout_ms * 1000ULL)) {
                    const Downlink down{counter, sender.next(), quantize(sample.roll_value),
                                        quantize(sample.pitch_value)};
                    auto bytes = encode(down);
                    if (config.simulator.send_b)
                        ports[1].send(bytes, now);
                    if (config.simulator.corrupt_every &&
                        ticks % config.simulator.corrupt_every == 0)
                        bytes[8] ^= 1;
                    if (config.simulator.send_a)
                        ports[0].send(bytes, now);
                }
            } else {
                Uplink up;
                up.counter = counter;
                up.version = identity.version;
                up.checksum = identity.checksum;
                up.selected = receiver.selected() < 0 ? unknown : uint8_t(receiver.selected());
                up.communication =
                    uint8_t(!receiver.healthy(0, now)) | (uint8_t(!receiver.healthy(1, now)) << 1);
                core->telemetry(up, now);
                if (receiver.valid(now)) {
                    up.operation = echo_operation;
                    up.roll_echo = receiver.target().roll;
                    up.pitch_echo = receiver.target().pitch;
                }
                // Report link faults without disguising a missing/unmapped Core diagnostic.
                if (up.fault == 0 && up.communication) {
                    up.fault = (up.communication & 1) ? 7101 : 7102;
                    up.severity = 0x20;
                }
                const auto bytes = encode(up);
                for (auto& p : ports)
                    p.send(bytes, now);
            }
        }
        if (now >= next_log) {
            next_log = now + 1000000;
            if (simulator) {
                for (unsigned i = 0; i < 2; ++i) {
                    const bool fresh = received[i] && now - received[i] < timeout;
                    const auto& f = replies[i];
                    aviator::Logger::info(
                        "SIM {} {} seq={} selected={} comm={} state={} echo_op={:02X} echo=({}, "
                        "{}) measured=({}, {}) rear={} brake={} fault={} previous={} "
                        "severity={:02X} version={:04X} checksum={:04X} bad_bytes={} "
                        "port={} rx_bytes={} tx_frames={} dropped={}",
                        i ? "B" : "A", fresh ? "LIVE" : "STALE", f.counter, f.selected,
                        f.communication, stateName(f.state), f.operation, position(f.roll_echo),
                        position(f.pitch_echo), position(f.roll), position(f.pitch), f.rear,
                        f.brake, f.fault, f.last_fault, f.severity, f.version, f.checksum,
                        parsers[i].rejected, ports[i].fd() >= 0 ? "open" : "closed",
                        ports[i].rx_bytes, ports[i].tx_frames, ports[i].dropped);
                }
                const bool a = received[0] && now - received[0] < timeout,
                           b = received[1] && now - received[1] < timeout;
                input->title(std::string("Flight simulator | A:") +
                             (a ? stateName(replies[0].state) : "STALE") +
                             " B:" + (b ? stateName(replies[1].state) : "STALE"));
            } else
                aviator::Logger::info("RS422 A={} B={} selected={} valid={} blocked={} switches={} "
                                      "duplicates={} conflicts={} bad_bytes={}/{} "
                                      "port={}/{} rx_bytes={}/{} tx_frames={}/{} dropped={}/{} "
                                      "coalesced_batches={} rejected_batches={}",
                                      receiver.healthy(0, now), receiver.healthy(1, now),
                                      receiver.selected(), receiver.valid(now), receiver.blocked(),
                                      receiver.switches, receiver.duplicates, receiver.conflicts,
                                      parsers[0].rejected, parsers[1].rejected,
                                      ports[0].fd() >= 0 ? "open" : "closed",
                                      ports[1].fd() >= 0 ? "open" : "closed",
                                      ports[0].rx_bytes, ports[1].rx_bytes,
                                      ports[0].tx_frames, ports[1].tx_frames,
                                      ports[0].dropped, ports[1].dropped,
                                      receiver.coalesced_batches, receiver.rejected_batches);
        }
    }
    if (core)
        core->publish(receiver, raw_frames, parsers, last_operations, aviator::monotonic_us(), true);
}
} // namespace flight_gateway

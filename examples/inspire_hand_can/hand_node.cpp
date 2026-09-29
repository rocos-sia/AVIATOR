// hand_node.cpp — SocketCAN control node for the Inspire robotic hand (因时手).
//
// Subscribes to `hand.command` on the AVIATOR ZMQ bus and drives both hands over
// SocketCAN. Self-contained (does not link common/), but follows the
// hand.command / hand.state wire format from docs/AVIATOR_ZMQ协议格式说明.md.
//
// Supported modes (see config/inspire_hand.yaml `node.supported_modes`):
//   NORMALIZED_POSITION  hands.{side}.drive_position_normalized[6]  [0,1] -> 0-1000
//   GRASP_SETPOINT       hands.{side}.grasp.closure                 [0,1] -> 0-1000
// Values are passed through directly: raw = round(value * 1000), no interpolation.

#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <unistd.h>

#include "can_writer.hpp"
#include "inspire_hand.hpp"

namespace {

using namespace inspire_hand;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
struct Config {
    std::string right_interface = "can0";
    std::string left_interface = "can1";
    int right_id = 1;
    int left_id = 2;
    std::array<int, 6> speed{1000, 1000, 1000, 1000, 1000, 1000};
    std::array<int, 6> force{500, 500, 500, 500, 500, 500};
    std::array<int, 6> safe_pose{1000, 1000, 1000, 1000, 1000, 1000};
    std::string subscribe_endpoint = "tcp://127.0.0.1:5556";
    std::string publish_endpoint = "tcp://127.0.0.1:5555";
    std::string publisher_id = "inspire_hand";
    std::vector<std::string> supported_modes{"NORMALIZED_POSITION", "GRASP_SETPOINT"};
    std::uint64_t timeout_us = 100000;
    int state_rate_hz = 10;
};

std::array<int, 6> load_int6(const YAML::Node& node, const std::array<int, 6>& dflt) {
    if (!node || !node.IsSequence() || node.size() != 6) return dflt;
    std::array<int, 6> out{};
    for (int i = 0; i < 6; ++i) out[i] = node[i].as<int>();
    return out;
}

Config load_config(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    Config cfg;
    if (const auto& n = root["can"]) {
        if (n["right_interface"])
            cfg.right_interface = n["right_interface"].as<std::string>();
        if (n["left_interface"])
            cfg.left_interface = n["left_interface"].as<std::string>();
    }
    if (const auto& n = root["hand"]) {
        if (n["right_id"]) cfg.right_id = n["right_id"].as<int>();
        if (n["left_id"]) cfg.left_id = n["left_id"].as<int>();
        cfg.speed = load_int6(n["speed"], cfg.speed);
        cfg.force = load_int6(n["force"], cfg.force);
        cfg.safe_pose = load_int6(n["safe_pose"], cfg.safe_pose);
    }
    if (const auto& n = root["zmq"]) {
        if (n["subscribe_endpoint"])
            cfg.subscribe_endpoint = n["subscribe_endpoint"].as<std::string>();
        if (n["publish_endpoint"])
            cfg.publish_endpoint = n["publish_endpoint"].as<std::string>();
    }
    if (const auto& n = root["node"]) {
        if (n["publisher_id"]) cfg.publisher_id = n["publisher_id"].as<std::string>();
        if (n["timeout_ms"])
            cfg.timeout_us = static_cast<std::uint64_t>(n["timeout_ms"].as<int>()) * 1000;
        if (n["state_rate_hz"]) cfg.state_rate_hz = n["state_rate_hz"].as<int>();
        if (n["supported_modes"]) {
            cfg.supported_modes.clear();
            for (const auto& m : n["supported_modes"])
                cfg.supported_modes.push_back(m.as<std::string>());
        }
    }
    return cfg;
}

// ---------------------------------------------------------------------------
// Small runtime helpers (mirror common/runtime.hpp semantics)
// ---------------------------------------------------------------------------
std::string read_proc(const char* path) {
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);
    if (!line.empty() && line.back() == '\n') line.pop_back();
    return line;
}

std::string new_session_id() { return read_proc("/proc/sys/kernel/random/uuid"); }

std::string local_clock_id() {
    char host[256]{};
    gethostname(host, sizeof(host));
    return std::string(host) + "-" + read_proc("/proc/sys/kernel/random/boot_id");
}

std::uint64_t utc_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::uint64_t monotonic_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               Clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// hand.command model + decoder
// ---------------------------------------------------------------------------
struct Origin {
    std::string publisher_id, session_id, clock_id;
    std::uint64_t sequence = 0, sample_mono_us = 0;
};

struct HandTarget {
    bool present = false;
    std::vector<double> normalized;  // NORMALIZED_POSITION
    bool has_closure = false;        // GRASP_SETPOINT
    double closure = 0.0;
};

struct HandCommand {
    std::uint64_t sequence = 0, timestamp = 0, sample_mono_us = 0;
    std::string clock_id, publisher_id, session_id;
    bool valid = false;
    std::string control_epoch, mode;
    Origin origin;
    HandTarget left, right;
};

// Minimal HandCommand decoder matching common/protocol.hpp semantics for the
// hand.command topic. Returns false and fills `error` on any violation.
bool decode_hand_command(std::string_view topic, std::string_view payload,
                         HandCommand& out, std::string& error) {
    const auto fail = [&](const std::string& why) {
        error = why;
        return false;
    };
    if (topic != "hand.command") return fail("topic != hand.command");
    if (payload.empty() || payload.size() > 65536) return fail("payload size");

    Json j;
    try {
        j = Json::parse(payload.begin(), payload.end());
    } catch (const std::exception& e) {
        return fail(std::string("JSON parse: ") + e.what());
    }
    if (!j.is_object()) return fail("payload not an object");
    if (j.value("msg_type", "") != "HandCommand") return fail("msg_type != HandCommand");

    try {
        out.sequence = j.at("sequence").get<std::uint64_t>();
        out.timestamp = j.at("timestamp").get<std::uint64_t>();
        out.sample_mono_us = j.at("sample_mono_us").get<std::uint64_t>();
        out.clock_id = j.at("clock_id").get<std::string>();
        out.publisher_id = j.at("publisher_id").get<std::string>();
        out.session_id = j.at("session_id").get<std::string>();
        out.valid = j.at("valid").get<bool>();

        out.control_epoch = j.at("control_epoch").get<std::string>();
        out.mode = j.at("mode").get<std::string>();
        const auto& origin = j.at("origin");
        out.origin.publisher_id = origin.at("publisher_id").get<std::string>();
        out.origin.session_id = origin.at("session_id").get<std::string>();
        out.origin.sequence = origin.at("sequence").get<std::uint64_t>();
        out.origin.sample_mono_us = origin.at("sample_mono_us").get<std::uint64_t>();
        out.origin.clock_id = origin.at("clock_id").get<std::string>();

        const auto parse_hand = [&](const char* side, HandTarget& t) {
            if (!j.contains("hands") || !j["hands"].contains(side)) return;
            const auto& h = j["hands"][side];
            if (!h.is_object()) throw std::runtime_error(std::string(side) + " not object");
            t.present = true;
            if (h.contains("drive_position_normalized")) {
                for (const auto& v : h["drive_position_normalized"])
                    t.normalized.push_back(v.get<double>());
            }
            if (h.contains("grasp") && h["grasp"].contains("closure")) {
                t.has_closure = true;
                t.closure = h["grasp"]["closure"].get<double>();
            }
        };
        parse_hand("left", out.left);
        parse_hand("right", out.right);
    } catch (const std::exception& e) {
        return fail(e.what());
    }

    if (!out.left.present && !out.right.present) return fail("no hands present");
    return true;
}

// ---------------------------------------------------------------------------
// Session / epoch / sequence guard (lightweight InputGuard)
// ---------------------------------------------------------------------------
struct Guard {
    enum class Verdict { accept, reject_old, reject_unauthorized };

    bool authorized = false;
    std::string session, epoch;
    std::uint64_t last_sequence = 0;
    std::uint64_t last_recv_mono = 0;

    Verdict accept(const HandCommand& cmd, std::uint64_t now_mono, std::string& why) {
        if (!authorized) {
            session = cmd.session_id;
            epoch = cmd.control_epoch;
            authorized = true;
            std::cerr << "[guard] authorizing session=" << session
                      << " epoch=" << epoch << "\n";
        }
        if (cmd.session_id != session) {
            why = "session mismatch";
            return Verdict::reject_unauthorized;
        }
        if (cmd.control_epoch != epoch) {
            why = "control_epoch mismatch";
            return Verdict::reject_unauthorized;
        }
        if (cmd.sequence <= last_sequence) {
            why = "stale sequence";
            return Verdict::reject_old;
        }
        last_sequence = cmd.sequence;
        last_recv_mono = now_mono;
        return Verdict::accept;
    }

    bool expired(std::uint64_t now_mono, std::uint64_t timeout_us) const {
        if (!authorized) return false;
        return now_mono - last_recv_mono > timeout_us;
    }
};

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------
class HandNode {
public:
    explicit HandNode(Config cfg) : cfg_(std::move(cfg)) {}

    int run() {
        session_id_ = new_session_id();
        clock_id_ = local_clock_id();

        // --- CAN (one interface per hand) ---
        try {
            can_right_.open(cfg_.right_interface);
            can_left_.open(cfg_.left_interface);
        } catch (const std::exception& e) {
            std::cerr << "CAN open failed: " << e.what() << "\n";
            std::cerr << "  bring the interfaces up first, e.g.:\n"
                      << "  sudo ip link set " << cfg_.right_interface
                      << " up type can bitrate 500000\n"
                      << "  sudo ip link set " << cfg_.left_interface
                      << " up type can bitrate 500000\n";
            return 1;
        }
        InspireHand hand_right(can_right_);
        InspireHand hand_left(can_left_);
        InspireAction right(hand_right, cfg_.right_id);
        InspireAction left(hand_left, cfg_.left_id);

        // Apply speed/force defaults on startup.
        const std::vector<int> speed(cfg_.speed.begin(), cfg_.speed.end());
        const std::vector<int> force(cfg_.force.begin(), cfg_.force.end());
        right.set_speed(speed);
        left.set_speed(speed);
        right.set_force(force);
        left.set_force(force);

        // --- ZMQ ---
        zmq::context_t ctx(1);

        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::rcvhwm, 8);
        sub.set(zmq::sockopt::linger, 0);
        sub.set(zmq::sockopt::rcvtimeo, 0);
        sub.set(zmq::sockopt::subscribe, std::string_view("hand.command"));
        sub.connect(cfg_.subscribe_endpoint);

        zmq::socket_t pub(ctx, zmq::socket_type::pub);
        pub.set(zmq::sockopt::sndhwm, 8);
        pub.set(zmq::sockopt::linger, 0);
        pub.set(zmq::sockopt::sndtimeo, 0);
        pub.connect(cfg_.publish_endpoint);

        std::cout << "inspire_hand: right=" << cfg_.right_interface
                  << "(id " << cfg_.right_id << ")"
                  << " left=" << cfg_.left_interface
                  << "(id " << cfg_.left_id << ")\n";
        std::cout << "inspire_hand: sub=" << cfg_.subscribe_endpoint
                  << " pub=" << cfg_.publish_endpoint
                  << " publisher_id=" << cfg_.publisher_id << "\n";
        std::cout << "inspire_hand: session=" << session_id_
                  << " clock=" << clock_id_ << "\n";

        std::uint64_t state_seq = 0;
        auto last_state = Clock::now();

        while (g_running) {
            // Wait up to 10 ms for one message, then handle a single command.
            zmq_pollitem_t items[] = {{sub.handle(), 0, ZMQ_POLLIN, 0}};
            const int rc = zmq_poll(items, 1, 10);
            if (rc > 0 && (items[0].revents & ZMQ_POLLIN)) {
                std::string topic, payload;
                if (try_recv(sub, topic, payload))
                    handle_message(topic, payload, right, left);
            }

            // Command watchdog -> safe pose.
            if (guard_.expired(monotonic_us(), cfg_.timeout_us)) {
                if (last_valid_) {
                    std::cerr << "inspire_hand: command timeout -> safe pose\n";
                    apply_safe(right, left);
                    last_valid_ = false;
                }
            }

            // Heartbeat state publish.
            const auto now = Clock::now();
            const auto period =
                std::chrono::milliseconds(1000 / std::max(1, cfg_.state_rate_hz));
            if (now - last_state >= period) {
                publish_state(pub, state_seq++);
                last_state = now;
            }
        }

        // Graceful shutdown: move to the safe pose before releasing the buses.
        apply_safe(right, left);
        can_right_.close();
        can_left_.close();
        std::cout << "inspire_hand: stopped\n";
        return 0;
    }

private:
    bool try_recv(zmq::socket_t& sub, std::string& topic, std::string& payload) {
        zmq::message_t t;
        if (!sub.recv(t, zmq::recv_flags::dontwait)) return false;
        if (!t.more()) return false;  // malformed: topic without payload
        topic = t.to_string();
        zmq::message_t p;
        if (!sub.recv(p, zmq::recv_flags::none)) return false;  // payload must follow
        payload = p.to_string();
        while (sub.get(zmq::sockopt::rcvmore)) {  // drain any extra frames
            zmq::message_t extra;
            if (!sub.recv(extra, zmq::recv_flags::none)) break;  // shouldn't happen
        }
        return true;
    }

    void handle_message(const std::string& topic, const std::string& payload,
                        InspireAction& right, InspireAction& left) {
        HandCommand cmd;
        std::string error;
        if (!decode_hand_command(topic, payload, cmd, error)) {
            std::cerr << "inspire_hand: reject decode: " << error << "\n";
            return;
        }
        if (!mode_supported(cmd.mode)) {
            std::cerr << "inspire_hand: reject unsupported mode=" << cmd.mode << "\n";
            return;
        }

        std::string why;
        const auto verdict = guard_.accept(cmd, monotonic_us(), why);
        if (verdict == Guard::Verdict::reject_unauthorized) {
            std::cerr << "inspire_hand: reject " << why << "\n";
            return;
        }
        if (verdict == Guard::Verdict::reject_old) return;  // stale/duplicate: ignore

        if (!cmd.valid) {
            apply_safe(right, left);
            last_valid_ = false;
            return;
        }

        if (!apply_command(cmd, right, left, error)) {
            std::cerr << "inspire_hand: reject apply: " << error << "\n";
            return;
        }
        last_valid_ = true;
        last_command_ = cmd;
        echo_normalized(cmd, last_norm_left_, last_norm_right_);
    }

    bool mode_supported(const std::string& mode) const {
        for (const auto& m : cfg_.supported_modes)
            if (m == mode) return true;
        return false;
    }

    bool apply_command(const HandCommand& cmd, InspireAction& right,
                       InspireAction& left, std::string& error) {
        const auto apply_one = [&](InspireAction& action, const HandTarget& t) {
            std::vector<int> raw(6, 0);
            if (cmd.mode == "NORMALIZED_POSITION") {
                if (t.normalized.size() != 6) {
                    error = "NORMALIZED_POSITION needs 6 values";
                    return false;
                }
                for (int i = 0; i < 6; ++i) {
                    const double n = t.normalized[i];
                    if (n < 0.0 || n > 1.0) {
                        error = "drive_position_normalized out of [0,1]";
                        return false;
                    }
                    raw[i] = static_cast<int>(std::lround(n * 1000.0));
                }
            } else if (cmd.mode == "GRASP_SETPOINT") {
                if (!t.has_closure) {
                    error = "GRASP_SETPOINT needs grasp.closure";
                    return false;
                }
                const double c = t.closure;
                if (c < 0.0 || c > 1.0) {
                    error = "closure out of [0,1]";
                    return false;
                }
                raw.assign(6, static_cast<int>(std::lround(c * 1000.0)));
            }
            return action.set_positions(raw);
        };
        if (cmd.right.present && !apply_one(right, cmd.right)) return false;
        if (cmd.left.present && !apply_one(left, cmd.left)) return false;
        return true;
    }

    void apply_safe(InspireAction& right, InspireAction& left) {
        const std::vector<int> safe(cfg_.safe_pose.begin(), cfg_.safe_pose.end());
        right.set_positions(safe);
        left.set_positions(safe);
    }

    static void echo_normalized(const HandCommand& cmd, std::vector<double>& left,
                                std::vector<double>& right) {
        const auto echo = [&](const HandTarget& t, std::vector<double>& out) {
            out.clear();
            if (cmd.mode == "NORMALIZED_POSITION") {
                out = t.normalized;
            } else if (cmd.mode == "GRASP_SETPOINT" && t.has_closure) {
                out.assign(6, t.closure);
            }
        };
        echo(cmd.left, left);
        echo(cmd.right, right);
    }

    void publish_state(zmq::socket_t& pub, std::uint64_t seq) {
        Json j;
        j["msg_type"] = "HandState";
        j["version"] = "1.0";
        j["sequence"] = seq;
        j["timestamp"] = utc_us();
        j["sample_mono_us"] = monotonic_us();
        j["clock_id"] = clock_id_;
        j["publisher_id"] = cfg_.publisher_id;
        j["session_id"] = session_id_;
        j["valid"] = last_valid_;

        const auto make_hand = [&](bool present, const std::vector<double>& norm) {
            Json h;
            h["valid"] = present && last_valid_;
            h["status"] = last_valid_ ? "ACTIVE" : "SAFE";
            h["enabled"] = present;
            h["error_code"] = 0;
            h["drive_position_normalized"] = norm;
            return h;
        };
        j["hands"]["left"] = make_hand(last_command_.left.present, last_norm_left_);
        j["hands"]["right"] = make_hand(last_command_.right.present, last_norm_right_);
        j["accepted_command"]["publisher_id"] = last_command_.publisher_id;
        j["accepted_command"]["session_id"] = last_command_.session_id;
        j["accepted_command"]["sequence"] = last_command_.sequence;
        j["accepted_command"]["sample_mono_us"] = last_command_.sample_mono_us;

        const std::string payload = j.dump();
        try {
            pub.send(zmq::buffer(std::string_view("hand.state")),
                     zmq::send_flags::sndmore | zmq::send_flags::dontwait);
            pub.send(zmq::buffer(payload), zmq::send_flags::dontwait);
        } catch (const zmq::error_t&) {
            // PUB down: drop the state report.
        }
    }

    Config cfg_;
    SocketCan can_right_, can_left_;
    Guard guard_;
    std::string session_id_, clock_id_;
    bool last_valid_ = false;
    HandCommand last_command_;
    std::vector<double> last_norm_left_, last_norm_right_;
};

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string config_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: inspire_hand_node [--config <yaml>]\n"
                      << "  default config: config/inspire_hand.yaml next to the "
                         "executable\n";
            return 0;
        } else {
            std::cerr << "unknown arg: " << arg << "\n";
            return 2;
        }
    }

    if (config_path.empty()) {
        char buf[4096]{};
        const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        const std::filesystem::path exe = (n > 0) ? std::string(buf, n) : argv[0];
        config_path = (exe.parent_path() / "config" / "inspire_hand.yaml").string();
    }

    Config cfg;
    try {
        cfg = load_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "config load failed (" << config_path << "): " << e.what() << "\n";
        return 1;
    }

    HandNode node(std::move(cfg));
    return node.run();
}

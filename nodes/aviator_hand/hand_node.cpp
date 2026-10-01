// hand_node.cpp — SocketCAN control node for the Inspire robotic hand (因时手).
//
// Subscribes to `hand.command` on the AVIATOR ZMQ bus and drives both hands over
// SocketCAN. Independent of the robotics/SDK stack; follows the
// hand.command / hand.state wire format from docs/AVIATOR_ZMQ协议格式说明.md.
//
// Supported modes (see config/inspire_hand.yaml `node.supported_modes`):
//   NORMALIZED_POSITION  hands.{side}.drive_position_normalized[6]  [0,1] -> 0-1000
//   GRASP_SETPOINT       hands.{side}.grasp.closure                 [0,1] -> 1000-0
// Normalized drive positions pass through; closure=0 opens and closure=1 closes.

#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <functional>
#include <set>
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
#include "position_feedback.hpp"

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
    int feedback_rate_hz = 10;
    std::uint64_t feedback_timeout_us = 300000;
    std::uint64_t response_timeout_us = 30000;
};

std::array<int, 6> load_int6(const YAML::Node& node, const std::array<int, 6>& dflt) {
    if (!node) return dflt;
    if (!node.IsSequence() || node.size() != 6)
        throw std::invalid_argument("hand vectors must contain exactly 6 values");
    std::array<int, 6> out{};
    for (int i = 0; i < 6; ++i) {
        out[i] = node[i].as<int>();
        if (out[i] < 0 || out[i] > 1000)
            throw std::invalid_argument("hand config value outside [0,1000]");
    }
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
        if (n["timeout_ms"]) {
            const int timeout_ms = n["timeout_ms"].as<int>();
            if (timeout_ms <= 0) throw std::invalid_argument("timeout_ms must be positive");
            cfg.timeout_us = static_cast<std::uint64_t>(timeout_ms) * 1000;
        }
        if (n["state_rate_hz"]) cfg.state_rate_hz = n["state_rate_hz"].as<int>();
        if (n["supported_modes"]) {
            cfg.supported_modes.clear();
            for (const auto& m : n["supported_modes"])
                cfg.supported_modes.push_back(m.as<std::string>());
        }
    }
    if (const auto n = root["feedback"]) {
        if (n["poll_rate_hz"]) cfg.feedback_rate_hz = n["poll_rate_hz"].as<int>();
        const auto timeout = [&](const char* key, std::uint64_t& output) {
            if (!n[key]) return;
            const auto ms = n[key].as<int>();
            if (ms <= 0) throw std::invalid_argument(std::string("feedback.") + key + " must be positive");
            output = static_cast<std::uint64_t>(ms) * 1000;
        };
        timeout("timeout_ms", cfg.feedback_timeout_us);
        timeout("response_timeout_ms", cfg.response_timeout_us);
    }
    if (cfg.feedback_rate_hz <= 0 || cfg.feedback_rate_hz > 50)
        throw std::invalid_argument("feedback.poll_rate_hz must be in [1,50]");
    if (cfg.feedback_timeout_us <= 1000000ULL / cfg.feedback_rate_hz ||
        cfg.feedback_timeout_us < cfg.response_timeout_us)
        throw std::invalid_argument("feedback timeout must exceed poll period and cover response timeout");
    if (cfg.right_id <= 0 || cfg.right_id > 0x3fff || cfg.left_id <= 0 || cfg.left_id > 0x3fff)
        throw std::invalid_argument("invalid hand CAN id");
    if (cfg.state_rate_hz <= 0 || cfg.state_rate_hz > 1000)
        throw std::invalid_argument("state_rate_hz must be in [1,1000]");
    if (cfg.supported_modes.empty()) throw std::invalid_argument("no supported modes");
    for (const auto& mode : cfg.supported_modes)
        if (mode != "NORMALIZED_POSITION" && mode != "GRASP_SETPOINT")
            throw std::invalid_argument("unsupported configured hand mode: " + mode);
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

std::string new_instance_id() {
    return "run-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
           "-" + std::to_string(getpid());
}

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

constexpr std::uint64_t kMaxJsonInteger = (1ULL << 53) - 1;

std::uint64_t positive_integer(const Json& j, const char* key) {
    const auto& value = j.at(key);
    if (!value.is_number_integer() ||
        (!value.is_number_unsigned() && value.get<std::int64_t>() <= 0))
        throw std::invalid_argument(std::string(key) + " must be a positive integer");
    const auto number = value.get<std::uint64_t>();
    if (number == 0 || number > kMaxJsonInteger)
        throw std::invalid_argument(std::string(key) + " outside uint53 range");
    return number;
}
std::string identifier(const Json& j, const char* key, bool uuid = false) {
    const auto value = j.at(key).get<std::string>();
    if (value.empty() || value.size() > 128 || value.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string("invalid ") + key);
    if (uuid) {
        if (value.size() != 36) throw std::invalid_argument("invalid UUID length");
        for (std::size_t i = 0; i < value.size(); ++i) {
            const char c = value[i];
            const bool dash = i == 8 || i == 13 || i == 18 || i == 23;
            if (!(dash ? c == '-' : ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))))
                throw std::invalid_argument("invalid canonical UUID");
        }
    }
    return value;
}

// Decode into a temporary: failed parsing never leaves a partially usable command.
bool decode_hand_command(std::string_view topic, std::string_view payload,
                         HandCommand& out, std::string& error) {
    try {
        if (topic != "hand.command" || payload.empty() || payload.size() > 65536)
            throw std::invalid_argument("invalid topic or payload size");
        std::vector<std::set<std::string>> keys;
        const auto j = Json::parse(payload.begin(), payload.end(),
            [&](int depth, Json::parse_event_t event, Json& value) {
                if (depth > 16) throw std::invalid_argument("JSON nesting limit");
                if (event == Json::parse_event_t::object_start) keys.emplace_back();
                if (event == Json::parse_event_t::key &&
                    !keys.back().insert(value.get<std::string>()).second)
                    throw std::invalid_argument("duplicate JSON key");
                if (event == Json::parse_event_t::object_end) keys.pop_back();
                return true;
            });
        if (!j.is_object() || j.at("msg_type").get<std::string>() != "HandCommand")
            throw std::invalid_argument("msg_type != HandCommand");
        const auto version = identifier(j, "version");
        if (version.size() < 3 || version.substr(0, 2) != "1." ||
            version.find_first_not_of("0123456789", 2) != std::string::npos)
            throw std::invalid_argument("unsupported version");
        HandCommand cmd;
        cmd.sequence = positive_integer(j, "sequence");
        cmd.timestamp = positive_integer(j, "timestamp");
        cmd.sample_mono_us = positive_integer(j, "sample_mono_us");
        cmd.clock_id = identifier(j, "clock_id");
        cmd.publisher_id = identifier(j, "publisher_id");
        cmd.session_id = identifier(j, "session_id");
        cmd.valid = j.at("valid").get<bool>();
        cmd.control_epoch = identifier(j, "control_epoch", true);
        cmd.mode = identifier(j, "mode");
        const auto& origin = j.at("origin");
        cmd.origin.publisher_id = identifier(origin, "publisher_id");
        cmd.origin.session_id = identifier(origin, "session_id");
        cmd.origin.sequence = positive_integer(origin, "sequence");
        cmd.origin.sample_mono_us = positive_integer(origin, "sample_mono_us");
        cmd.origin.clock_id = identifier(origin, "clock_id");

        // An authenticated invalidation does not require usable numeric targets.
        if (cmd.valid) {
            const auto& hands = j.at("hands");
            const auto parse_hand = [&](const char* side, HandTarget& target) {
                const auto& h = hands.at(side);
                if (!h.is_object()) throw std::invalid_argument("hand target must be object");
                target.present = true;
                if (cmd.mode == "NORMALIZED_POSITION") {
                    const auto& values = h.at("drive_position_normalized");
                    if (!values.is_array() || values.size() != 6 || h.contains("grasp") ||
                        h.contains("joint_position"))
                        throw std::invalid_argument("expected exactly 6 normalized positions");
                    for (const auto& value : values) {
                        if (!value.is_number()) throw std::invalid_argument("position must be numeric");
                        const double n = value.get<double>();
                        if (!std::isfinite(n) || n < 0 || n > 1)
                            throw std::invalid_argument("position outside [0,1]");
                        target.normalized.push_back(n);
                    }
                } else if (cmd.mode == "GRASP_SETPOINT") {
                    if (h.contains("drive_position_normalized") || h.contains("joint_position"))
                        throw std::invalid_argument("mixed hand target modes");
                    const auto& value = h.at("grasp").at("closure");
                    if (!value.is_number()) throw std::invalid_argument("closure must be numeric");
                    target.closure = value.get<double>();
                    if (!std::isfinite(target.closure) || target.closure < 0 || target.closure > 1)
                        throw std::invalid_argument("closure outside [0,1]");
                    target.has_closure = true;
                } else {
                    throw std::invalid_argument("unsupported hand mode");
                }
            };
            parse_hand("left", cmd.left);
            parse_hand("right", cmd.right);
        }
        out = std::move(cmd);
        error.clear();
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

struct Guard {
    enum class Verdict { accept, reject_old, reject_unauthorized, reject_stale };
    bool authorized = false;
    std::string publisher, session, epoch, origin_publisher, origin_session;
    std::uint64_t last_sequence = 0, last_recv_mono = 0, sample_mono = 0, origin_mono = 0;

    static bool fresh(std::uint64_t sample, std::uint64_t now, std::uint64_t timeout) {
        return sample > 0 && sample <= now && now - sample < timeout;
    }
    // Called on a candidate copy after complete target validation; commit only after CAN success.
    Verdict accept(const HandCommand& cmd, std::uint64_t now, std::uint64_t timeout,
                   const std::string& clock, std::string& why) {
        if (cmd.clock_id != clock || cmd.origin.clock_id != clock ||
            !fresh(cmd.sample_mono_us, now, timeout) ||
            !fresh(cmd.origin.sample_mono_us, now, timeout)) {
            why = "stale/future source or origin, or clock domain mismatch";
            return Verdict::reject_stale;
        }
        if (authorized && (cmd.publisher_id != publisher ||
            cmd.control_epoch != epoch || cmd.origin.publisher_id != origin_publisher)) {
            why = "publisher/epoch/origin mismatch";
            return Verdict::reject_unauthorized;
        }
        if (cmd.sequence == 0 || cmd.sequence > kMaxJsonInteger ||
            (cmd.session_id == session && cmd.sequence <= last_sequence) ||
            (authorized && cmd.session_id != session && cmd.sample_mono_us <= sample_mono)) {
            why = "stale or invalid sequence";
            return Verdict::reject_old;
        }
        authorized = true;
        publisher = cmd.publisher_id;
        session = cmd.session_id;
        epoch = cmd.control_epoch;
        origin_publisher = cmd.origin.publisher_id;
        origin_session = cmd.origin.session_id;
        last_sequence = cmd.sequence;
        last_recv_mono = now;
        sample_mono = cmd.sample_mono_us;
        origin_mono = cmd.origin.sample_mono_us;
        return Verdict::accept;
    }
    bool expired(std::uint64_t now, std::uint64_t timeout) const {
        return authorized && (!fresh(last_recv_mono, now, timeout) ||
                              !fresh(sample_mono, now, timeout) ||
                              !fresh(origin_mono, now, timeout));
    }
};

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------
class HandNode {
public:
    explicit HandNode(Config cfg, bool feedback_only = false)
        : cfg_(std::move(cfg)), feedback_only_(feedback_only),
          feedback_right_(cfg_.right_id, cfg_.feedback_rate_hz, cfg_.response_timeout_us),
          feedback_left_(cfg_.left_id, cfg_.feedback_rate_hz, cfg_.response_timeout_us) {}

    int run() {
        session_id_ = new_instance_id();
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

        const int result = run_safely(right, left, [&] { control_loop(right, left); });
        can_right_.close();
        can_left_.close();
        std::cout << "inspire_hand: stopped\n";
        return result;
    }

private:
    friend struct HandNodeTestAccess;

    int run_safely(InspireAction& right, InspireAction& left, const std::function<void()>& loop) {
        int result = 0;
        try {
            loop();
        } catch (const std::exception& e) {
            std::cerr << "inspire_hand: runtime failure: " << e.what() << '\n';
            result = 1;
        } catch (...) {
            std::cerr << "inspire_hand: unknown runtime failure\n";
            result = 1;
        }
        last_valid_ = false;
        if (!feedback_only_ && !try_safe_pose(right, left)) result = 1;
        return result;
    }

    void control_loop(InspireAction& right, InspireAction& left) {
        // Apply speed/force defaults on startup.
        const std::vector<int> speed(cfg_.speed.begin(), cfg_.speed.end());
        const std::vector<int> force(cfg_.force.begin(), cfg_.force.end());
        if (!feedback_only_) {
            right.set_speed(speed);
            left.set_speed(speed);
            right.set_force(force);
            left.set_force(force);
        }

        // --- ZMQ ---
        zmq::context_t ctx(1);

        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::rcvhwm, 8);
        sub.set(zmq::sockopt::linger, 0);
        sub.set(zmq::sockopt::rcvtimeo, 0);
        if (!feedback_only_)
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

        std::cout << "inspire_hand: feedback=" << cfg_.feedback_rate_hz
                  << " Hz timeout=" << cfg_.feedback_timeout_us / 1000
                  << " ms feedback_only=" << (feedback_only_ ? "true" : "false") << std::endl;
        InspireHand feedback_right(can_right_), feedback_left(can_left_);
        std::uint64_t state_seq = 0;
        auto last_state = Clock::now();

        while (g_running) {
            // Wait up to 10 ms for one message, then handle a single command.
            zmq_pollitem_t items[] = {{sub.handle(), 0, ZMQ_POLLIN, 0},
                                     {nullptr, can_right_.native_handle(), ZMQ_POLLIN, 0},
                                     {nullptr, can_left_.native_handle(), ZMQ_POLLIN, 0}};
            const int rc = zmq_poll(items, 3, 10);
            if (rc > 0 && (items[0].revents & ZMQ_POLLIN)) {
                std::string topic, payload;
                if (try_recv(sub, topic, payload))
                    handle_message(topic, payload, right, left);
            }

            // Command watchdog -> safe pose.
            if (!feedback_only_) check_watchdog(right, left, monotonic_us());
            service_feedback(can_right_, feedback_right, feedback_right_);
            service_feedback(can_left_, feedback_left, feedback_left_);

            // Heartbeat state publish.
            const auto now = Clock::now();
            const auto period =
                std::chrono::milliseconds(1000 / std::max(1, cfg_.state_rate_hz));
            if (now - last_state >= period) {
                publish_state(pub, ++state_seq);
                last_state = now;
            }
        }
    }

    template <class Receiver>
    void service_feedback(Receiver& can, InspireHand& hand, PositionFeedback& feedback) {
        try {
            // Bounded drain prevents unrelated traffic from starving the command watchdog.
            for (unsigned i = 0; i < 64; ++i) {
                const auto frame = can.try_recv();
                if (!frame) break;
                feedback.consume(*frame, monotonic_us());
            }
            feedback.step(hand, monotonic_us());
        } catch (const std::exception& e) {
            feedback.transport_error(e.what(), monotonic_us());
        }
    }

    void check_watchdog(InspireAction& right, InspireAction& left, std::uint64_t now) {
        if (last_valid_ && guard_.expired(now, cfg_.timeout_us)) {
            std::cerr << "inspire_hand: command timeout -> safe pose\n";
            apply_safe(right, left);
        }
    }

    bool try_recv(zmq::socket_t& sub, std::string& topic, std::string& payload) {
        zmq::message_t t;
        if (!sub.recv(t, zmq::recv_flags::dontwait)) return false;
        if (!t.more()) return false;  // malformed: topic without payload
        topic = t.to_string();
        zmq::message_t p;
        if (!sub.recv(p, zmq::recv_flags::none)) return false;  // payload must follow
        payload = p.to_string();
        const bool extra_frames = p.more();
        while (sub.get(zmq::sockopt::rcvmore)) {  // drain and reject extra frames
            zmq::message_t extra;
            if (!sub.recv(extra, zmq::recv_flags::none)) break;  // shouldn't happen
        }
        return !extra_frames;
    }

    void handle_message(const std::string& topic, const std::string& payload,
                        InspireAction& right, InspireAction& left,
                        std::uint64_t now = monotonic_us()) {
        if (feedback_only_) return;
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

        std::vector<int> raw_right, raw_left;
        if (cmd.valid && !prepare_command(cmd, raw_right, raw_left, error)) {
            std::cerr << "inspire_hand: reject targets: " << error << '\n';
            return;
        }
        auto candidate = guard_;
        std::string why;
        const auto verdict = candidate.accept(cmd, now, cfg_.timeout_us, clock_id_, why);
        if (verdict != Guard::Verdict::accept) {
            if (verdict != Guard::Verdict::reject_old)
                std::cerr << "inspire_hand: reject " << why << '\n';
            return;
        }
        if (!cmd.valid) {
            apply_safe(right, left);
            guard_ = std::move(candidate);
            last_command_ = cmd;
            return;
        }
        // All targets and authorization are checked before the first CAN write.
        // CAN itself is not transactional; any I/O error invokes run_safely cleanup.
        if (!right.set_positions(raw_right) || !left.set_positions(raw_left))
            throw std::runtime_error("validated hand target rejected by driver");
        guard_ = std::move(candidate);
        last_valid_ = true;
        last_command_ = cmd;
        echo_normalized(cmd, last_norm_left_, last_norm_right_);
    }

    bool mode_supported(const std::string& mode) const {
        for (const auto& m : cfg_.supported_modes)
            if (m == mode) return true;
        return false;
    }

    static bool prepare_command(const HandCommand& cmd, std::vector<int>& right,
                                std::vector<int>& left, std::string& error) {
        const auto prepare = [&](const HandTarget& target, std::vector<int>& raw) {
            if (!target.present) { error = "both hand targets required"; return false; }
            raw.clear();
            if (cmd.mode == "NORMALIZED_POSITION" && target.normalized.size() == 6) {
                for (double n : target.normalized) {
                    if (!std::isfinite(n) || n < 0 || n > 1) {
                        error = "position outside [0,1]";
                        return false;
                    }
                    raw.push_back(static_cast<int>(std::lround(n * 1000)));
                }
            } else if (cmd.mode == "GRASP_SETPOINT" && target.has_closure &&
                       std::isfinite(target.closure) && target.closure >= 0 && target.closure <= 1) {
                raw.assign(6, static_cast<int>(std::lround((1.0 - target.closure) * 1000)));
            } else {
                error = "invalid hand target or mode";
                return false;
            }
            return true;
        };
        return prepare(cmd.right, right) && prepare(cmd.left, left);
    }

    // Independent best-effort writes: one failed channel must not skip the others.
    bool try_safe_pose(InspireAction& right, InspireAction& left) {
        using Set = void (InspireAction::*)(int);
        static constexpr std::array<Set, 6> setters = {
            &InspireAction::thumb_rot, &InspireAction::thumb, &InspireAction::index,
            &InspireAction::middle, &InspireAction::ring, &InspireAction::pinky};
        bool ok = true;
        for (auto* action : {&right, &left})
            for (std::size_t i = 0; i < setters.size(); ++i) {
                try {
                    (action->*setters[i])(cfg_.safe_pose[i]);
                } catch (const std::exception& e) {
                    ok = false;
                    std::cerr << "inspire_hand: safe pose failed on "
                              << (action == &right ? "right" : "left") << " channel " << i
                              << ": " << e.what() << '\n';
                }
            }
        return ok;
    }

    void apply_safe(InspireAction& right, InspireAction& left) {
        last_valid_ = false;
        if (!try_safe_pose(right, left))
            throw std::runtime_error("failed to send complete safe pose");
    }

    static void echo_normalized(const HandCommand& cmd, std::vector<double>& left,
                                std::vector<double>& right) {
        const auto echo = [&](const HandTarget& t, std::vector<double>& out) {
            out.clear();
            if (cmd.mode == "NORMALIZED_POSITION") {
                out = t.normalized;
            } else if (cmd.mode == "GRASP_SETPOINT" && t.has_closure) {
                out.assign(6, 1.0 - t.closure);
            }
        };
        echo(cmd.left, left);
        echo(cmd.right, right);
    }

    Json make_state(std::uint64_t seq, std::uint64_t now) const {
        Json j;
        j["msg_type"] = "HandState";
        j["version"] = "1.0";
        j["sequence"] = seq;
        j["timestamp"] = utc_us();
        j["sample_mono_us"] = now;
        j["clock_id"] = clock_id_;
        j["publisher_id"] = cfg_.publisher_id;
        j["session_id"] = session_id_;
        const bool right_fresh = feedback_right_.fresh(now, cfg_.feedback_timeout_us);
        const bool left_fresh = feedback_left_.fresh(now, cfg_.feedback_timeout_us);
        j["valid"] = right_fresh && left_fresh;
        j["command_valid"] = last_valid_;
        j["feedback_only"] = feedback_only_;

        const auto make_hand = [&](const PositionFeedback& feedback, bool fresh,
                                   const std::vector<double>& command) {
            Json h;
            h["valid"] = fresh;
            h["status"] = fresh ? (last_valid_ ? "ACTIVE" : "READY")
                                  : (feedback.received() ? "STALE" : "OFFLINE");
            h["enabled"] = last_valid_; // Command acceptance state, not measured device enable.
            h["error_code"] = 0; // Hardware error registers are not sampled here.
            h["feedback_available"] = fresh;
            h["position_source"] = "angle_act_register";
            h["sample_mono_us"] = feedback.received() ? Json(feedback.sample_mono_us()) : Json(nullptr);
            h["sample_time_basis"] = "host_read_request";
            h["feedback_age_ms"] = feedback.received() && now >= feedback.sample_mono_us()
                ? Json((now - feedback.sample_mono_us()) / 1000.0) : Json(nullptr);
            h["drive_position_raw"] = feedback.received() ? Json(feedback.values()) : Json(nullptr);
            h["drive_position_normalized"] = Json::array();
            if (feedback.received())
                for (int value : feedback.values()) h["drive_position_normalized"].push_back(value / 1000.0);
            h["commanded_drive_position_normalized"] = command;
            h["joint_position"] = nullptr; // Drive counts are not calibrated joint radians.
            h["joint_velocity"] = nullptr;
            h["grasp_verified"] = false;
            h["feedback_samples"] = feedback.samples();
            h["feedback_timeouts"] = feedback.timeouts();
            h["feedback_io_errors"] = feedback.io_errors();
            h["feedback_last_error"] = feedback.last_error();
            return h;
        };
        j["hands"]["left"] = make_hand(feedback_left_, left_fresh, last_norm_left_);
        j["hands"]["right"] = make_hand(feedback_right_, right_fresh, last_norm_right_);
        j["accepted_command"]["publisher_id"] = last_command_.publisher_id;
        j["accepted_command"]["session_id"] = last_command_.session_id;
        j["accepted_command"]["sequence"] = last_command_.sequence;
        j["accepted_command"]["sample_mono_us"] = last_command_.sample_mono_us;

        return j;
    }

    void publish_state(zmq::socket_t& pub, std::uint64_t seq) {
        const auto j = make_state(seq, monotonic_us());
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
    bool feedback_only_ = false;
    PositionFeedback feedback_right_, feedback_left_;
    SocketCan can_right_, can_left_;
    Guard guard_;
    std::string session_id_, clock_id_;
    bool last_valid_ = false;
    HandCommand last_command_;
    std::vector<double> last_norm_left_, last_norm_right_;
};

}  // namespace

#ifndef INSPIRE_HAND_TESTING
std::filesystem::path default_hand_config(const std::filesystem::path& executable) {
#ifdef AVIATOR_HAND_SOURCE_CONFIG
    const auto installed = executable.parent_path() / AVIATOR_HAND_INSTALLED_CONFIG;
    if (std::filesystem::exists(installed)) return installed;
    return AVIATOR_HAND_SOURCE_CONFIG;
#else
    // Preserve the original standalone example's executable-relative configuration.
    return executable.parent_path() / "config" / "inspire_hand.yaml";
#endif
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string config_path;
    bool feedback_only = false, check_config = false;
    char executable_path[4096]{};
    const auto length = readlink("/proc/self/exe", executable_path, sizeof(executable_path) - 1);
    const std::filesystem::path executable = length > 0 ? std::string(executable_path, length) : argv[0];
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--feedback-only") {
            feedback_only = true;
        } else if (arg == "--check-config") {
            check_config = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: " << executable.filename().string()
                      << " [--config <yaml>] [--feedback-only] [--check-config]\n"
                      << "  --feedback-only: read positions and publish state; no control or safe-pose writes\n"
                      << "  --check-config: validate and print configuration without opening CAN or ZMQ\n"
                      << "  default config: " << default_hand_config(executable).string() << '\n';
            return 0;
        } else {
            std::cerr << "unknown arg: " << arg << "\n";
            return 2;
        }
    }

    if (config_path.empty()) {
        config_path = default_hand_config(executable).string();
    }

    Config cfg;
    try {
        cfg = load_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "config load failed (" << config_path << "): " << e.what() << "\n";
        return 1;
    }

    if (check_config) {
        std::cout << Json{{"config", std::filesystem::absolute(config_path).lexically_normal().string()},
                          {"right_interface", cfg.right_interface}, {"right_id", cfg.right_id},
                          {"left_interface", cfg.left_interface}, {"left_id", cfg.left_id},
                          {"publisher_id", cfg.publisher_id},
                          {"subscribe_endpoint", cfg.subscribe_endpoint},
                          {"publish_endpoint", cfg.publish_endpoint},
                          {"feedback_only", feedback_only}}.dump(2) << '\n';
        return 0;
    }

    try {
        HandNode node(std::move(cfg), feedback_only);
        return node.run();
    } catch (const std::exception& e) {
        std::cerr << "inspire_hand: startup failed: " << e.what() << '\n';
        return 1;
    }
}
#endif

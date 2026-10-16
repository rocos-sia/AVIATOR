#include "HandRuntime.hpp"
#include "Logger.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <optional>
#include <thread>
#include <yaml-cpp/yaml.h>
namespace aviator::hand {
namespace {
using Json = nlohmann::json;
constexpr const char* sides[]{"left", "right"};
void require(bool b, const std::string& why) {
    if (!b)
        throw std::runtime_error(why);
}
bool fresh(uint64_t stamp, uint64_t now, unsigned timeout) {
    return stamp && stamp <= now && now - stamp < timeout;
}
int setting(const YAML::Node& n, const char* key, int fallback, int lo, int hi) {
    int v = n && n[key] ? n[key].as<int>() : fallback;
    require(v >= lo && v <= hi, std::string("Invalid hand setting: ") + key);
    return v;
}
struct Snapshot {
    Sample data;
    uint64_t stamp = 0, reads = 0, errors = 0;
    uint64_t telemetry_stamp = 0, telemetry_reads = 0, telemetry_errors = 0;
    std::string telemetry_error;
    std::string error;
    bool recent(uint64_t now, unsigned timeout) const {
        // A failed poll never renews these stamps. One failed transaction does
        // not invalidate the previous successful samples within their deadline.
        return fresh(stamp, now, timeout) && fresh(telemetry_stamp, now, timeout);
    }
    bool good(uint64_t now, unsigned timeout) const {
        return recent(now, timeout) && error.empty() && telemetry_error.empty();
    }
};
struct Hold {
    int requested = -2, held = -1, direction = 0;
    bool at_target = false;
    uint64_t target_since = 0, command = 0, sample = 0, errors = 0, residual_since = 0;
    std::deque<std::pair<uint64_t, int>> samples;
};
struct Command {
    Message message;
    Origin origin;
    uint64_t received = 0;
    std::array<Values, 2> targets{};
};
class CommandExpired : public std::runtime_error {
public:
    CommandExpired(const Command& c, uint64_t now, unsigned limit)
        : std::runtime_error("Hand command expired during write: sequence=" +
            std::to_string(c.message.header.sequence) + " received_age_us=" +
            std::to_string(int64_t(now) - int64_t(c.received)) + " command_age_us=" +
            std::to_string(int64_t(now) - int64_t(c.message.header.sample_mono_us)) +
            " origin_age_us=" + std::to_string(int64_t(now) - int64_t(c.origin.sample_mono_us)) +
            " limit_us=" + std::to_string(limit)) {}
};
bool live(const Command& c, uint64_t now, unsigned timeout) {
    return fresh(c.received, now, timeout) &&
           fresh(c.message.header.sample_mono_us, now, timeout) &&
           fresh(c.origin.sample_mono_us, now, timeout);
}
bool successor(const Command& c, const Command& next, unsigned timeout) {
    const auto& a = c.message.header;
    const auto& b = next.message.header;
    return a.valid && b.valid && a.publisher_id == b.publisher_id &&
           a.session_id == b.session_id && b.sequence > a.sequence &&
           b.sample_mono_us >= a.sample_mono_us &&
           c.message.body.at("control_epoch") == next.message.body.at("control_epoch") &&
           c.message.body.at("mode") == next.message.body.at("mode") &&
           c.origin.publisher_id == next.origin.publisher_id &&
           c.origin.session_id == next.origin.session_id &&
           next.origin.sequence >= c.origin.sequence &&
           next.origin.sample_mono_us >= c.origin.sample_mono_us &&
           live(next, monotonic_us(), timeout);
}
Command parse(const Message& m) {
    require(m.topic == Topic::hand_command, "Unexpected hand topic");
    Command c;
    c.message = m;
    c.received = monotonic_us();
    std::string error;
    require(read_origin(m.body, c.origin, error), error);
    // encode/decode validates the common envelope/UUID even for direct callers.
    std::string wire;
    require(encode(m, wire, error), error);
    require(m.header.clock_id == local_clock_id() && c.origin.clock_id == local_clock_id(),
            "Hand clock mismatch");
    const auto mode = m.body.at("mode").get<std::string>();
    if (m.header.valid) {
        require(mode == "NORMALIZED_POSITION" || mode == "GRASP_SETPOINT", "Unsupported hand mode");
        for (int side = 0; side < 2; ++side) {
            const auto& h = m.body.at("hands").at(sides[side]);
            require(!h.contains("joint_position"), "Mixed hand target modes");
            if (mode == "NORMALIZED_POSITION") {
                require(!h.contains("grasp"), "Mixed hand target modes");
                const auto& p = h.at("drive_position_normalized");
                require(p.is_array() && p.size() == 6, "Hand requires six positions");
                for (int i = 0; i < 6; ++i) {
                    require(p[i].is_number(), "Non-numeric hand target");
                    double x = p[i].get<double>();
                    require(std::isfinite(x) && x >= 0 && x <= 1, "Hand target outside [0,1]");
                    c.targets[side][i] = int(std::floor(x * 1000 + .5));
                }
            } else {
                require(!h.contains("drive_position_normalized"), "Mixed hand target modes");
                const auto& value = h.at("grasp").at("closure");
                require(value.is_number(), "Non-numeric closure");
                double x = value.get<double>();
                require(std::isfinite(x) && x >= 0 && x <= 1, "Closure outside [0,1]");
                c.targets[side].fill(int(std::floor((1 - x) * 1000 + .5)));
            }
        }
    }
    return c;
}
Json normalized(const Values& values) {
    Json j = Json::array();
    for (int v : values)
        j.push_back(v / 1000.);
    return j;
}
} // namespace
Config loadConfig(const std::filesystem::path& system, bool fake_override,
                  const std::filesystem::path& hand_file) {
    Config c;
    auto root = YAML::LoadFile(system.string());
    auto n = root["xcore_sdk_hand"];
    if (n) {
        c.enabled = n["enabled"].as<bool>(true);
        c.fake = n["fake_hand"].as<bool>(false);
    }
    c.fake = c.fake || fake_override;
    auto path = hand_file;
    if (path.empty()) {
        path = n && n["config"] ? n["config"].as<std::string>() : "rh56ftp_hand.yaml";
        if (path.is_relative())
            path = system.parent_path() / path;
    }
    YAML::Node h(YAML::NodeType::Map);
    if (std::filesystem::exists(path))
        h = YAML::LoadFile(path.string());
    else
        require(hand_file.empty() && !(n && n["config"]), "Hand config missing: " + path.string());
    for (int s = 0; s < 2; ++s) {
        c.ids[s] = setting(h, (std::string(sides[s]) + "_id").c_str(), 1, 1, 254);
        auto enabled = h[std::string(sides[s]) + "_enabled"];
        if (enabled)
            c.sides[s] = enabled.as<bool>();
    }
    for (auto name : {"speed", "force"}) {
        auto v = h[name];
        int limit = std::string(name) == "speed" ? 1000 : 3000;
        int grasp = v && v.IsScalar()
                        ? v.as<int>()
                        : setting(v, "grasp", std::string(name) == "speed" ? 500 : 1000, 0, limit);
        require(grasp >= 0 && grasp <= limit, "Invalid hand grasp setting");
        int hold = v && v.IsMap() ? setting(v, "hold", 100, 0, limit) : 100;
        if (std::string(name) == "speed") {
            c.speed = grasp;
            c.hold_speed = hold;
        } else {
            c.force = grasp;
            c.hold_force = hold;
        }
    }
    c.threshold = setting(h, "threshold", 10, 0, 999);
    c.command_us = setting(h, "command_timeout_ms", 100, 1, 10000) * 1000u;
    c.feedback_us = setting(h, "feedback_timeout_ms", 500, 1, 10000) * 1000u;
    c.hold_us = setting(h, "closing_hold_ms", 10000, 1, 3600000) * 1000u;
    c.period_us = setting(h, "state_period_ms", 60, 1, 1000) * 1000u;
    c.write_period_us = 1000000u / setting(h, "position_write_hz", 20, 1, 100);
    c.position_period_us = 1000000u / setting(h, "position_read_hz", 10, 1, 100);
    c.telemetry_period_us = 1000000u / setting(h, "telemetry_read_hz", 5, 1, 100);
    require(c.position_period_us < c.feedback_us && c.telemetry_period_us < c.feedback_us,
            "Hand read periods must be shorter than feedback_timeout_ms");
    require(c.write_period_us < c.command_us,
            "Hand write period must be shorter than command_timeout_ms");
    c.rs485_timeout_ms = setting(h, "rs485_timeout_ms", 50, 50, 10000);
    c.motion_rate = h["motion_rate"].as<double>(2);
    require(std::isfinite(c.motion_rate) && c.motion_rate > 0, "Invalid hand motion_rate");
    if (root["core_hand"] && root["core_hand"]["publisher_id"])
        c.publisher = root["core_hand"]["publisher_id"].as<std::string>();
    require(!c.publisher.empty() && c.publisher.size() <= 128, "Invalid hand publisher");
    require(!c.enabled || c.sides[0] || c.sides[1], "No enabled hands");
    return c;
}
class Runtime::Impl {
  public:
    Config cfg;
    std::array<std::unique_ptr<Device>, 2> devices;
    std::mutex mutex;
    std::condition_variable wake;
    std::atomic<bool> stop{false};
    std::array<Snapshot, 2> samples;
    std::optional<Command> pending;
    Json output;
    uint64_t output_deadline = 0;
    std::thread readers[2], control;
    uint64_t sequence = 0;
    std::string session = new_instance_id();
    // The following state belongs exclusively to the control worker.
    std::optional<Command> accepted;
    std::array<std::array<Hold, 6>, 2> holds;
    struct Settings {
        bool mode = false;
        std::optional<Values> speed, force;
    };
    std::array<Settings, 2> settings;
    std::array<std::optional<Values>, 2> angles;
    std::array<bool, 2> open_fault{};
    std::array<Json, 2> faults{Json::object(), Json::object()};
    std::array<Json, 2> requested{Json::array(), Json::array()},
        commanded{Json::array(), Json::array()};
    bool command_valid = false, safe_applied = false, failed = false;
    std::string fault, write_error, last_log;
    uint64_t logged = 0, retry_logged = 0;
    Impl(Config c, std::array<std::unique_ptr<Device>, 2> d)
        : cfg(std::move(c)), devices(std::move(d)) {
        for (int s = 0; s < 2; ++s)
            if (cfg.sides[s]) {
                require(bool(devices[s]), "Missing hand device");
                devices[s]->setLabel(sides[s]);
            }
        Logger::info("Hand scheduling: position_write_hz={} position_read_hz={} telemetry_read_hz={} state_hz={}",
                     1000000. / cfg.write_period_us, 1000000. / cfg.position_period_us,
                     1000000. / cfg.telemetry_period_us, 1000000. / cfg.period_us);
        publishControl();
        try {
            for (int s = 0; s < 2; ++s)
                if (cfg.sides[s])
                    readers[s] = std::thread([this, s] { readLoop(s); });
            control = std::thread([this] { controlLoop(); });
        } catch (...) {
            stop = true;
            wake.notify_all();
            for (auto& t : readers)
                if (t.joinable())
                    t.join();
            throw;
        }
    }
    ~Impl() {
        stop = true;
        wake.notify_all();
        if (control.joinable())
            control.join();
        for (auto& t : readers)
            if (t.joinable())
                t.join();
    }
    void log(const std::string& text) {
        auto now = monotonic_us();
        if (text != last_log || now - logged >= 10000000) {
            Logger::error("Hand: {}", text);
            logged = now;
            last_log = text;
        }
    }
    std::array<Snapshot, 2> snapshots() {
        std::lock_guard<std::mutex> lock(mutex);
        return samples;
    }
    void readLoop(int s) {
        uint64_t due[2]{}, last_report[2]{};
        std::string previous_error[2];
        while (!stop) {
            // One reader per hand schedules independent position/telemetry jobs.
            // Select the oldest due job so neither can be starved by overruns.
            const int kind = due[0] <= due[1] ? 0 : 1;
            {
                std::unique_lock<std::mutex> lock(mutex);
                const auto now = monotonic_us();
                if (now < due[kind]) {
                    wake.wait_for(lock, std::chrono::microseconds(due[kind] - now),
                                  [&] { return stop.load(); });
                    continue;
                }
            }
            const auto start = monotonic_us();
            auto next = snapshots()[s];
            auto& error = kind ? next.telemetry_error : next.error;
            auto& errors = kind ? next.telemetry_errors : next.errors;
            try {
                if (kind == 0) {
                    next.data.position = devices[s]->readPosition();
                    next.stamp = start;
                    ++next.reads;
                } else {
                    auto telemetry = devices[s]->readTelemetry();
                    telemetry.position = next.data.position;
                    next.data = telemetry;
                    next.telemetry_stamp = start;
                    ++next.telemetry_reads;
                }
                error.clear();
            } catch (const std::exception& e) {
                error = e.what();
                ++errors;
            }
            const char* name = kind ? "telemetry" : "position";
            if (!error.empty() && (error != previous_error[kind] || start - last_report[kind] >= 10000000)) {
                Logger::error("Hand {} {} read: {} (errors={})", sides[s], name, error, errors);
                last_report[kind] = start;
            } else if (error.empty() && !previous_error[kind].empty()) {
                Logger::info("Hand {} {} read recovered", sides[s], name);
            }
            previous_error[kind] = error;
            {
                std::lock_guard<std::mutex> lock(mutex);
                samples[s] = std::move(next);
            }
            const auto period = kind ? cfg.telemetry_period_us : cfg.position_period_us;
            due[kind] = std::max(start + period, monotonic_us() + 2000);
        }
    }
    void validateGuard(const Command& c) {
        require(live(c, monotonic_us(), cfg.command_us), "Stale/future hand command");
        if (!accepted)
            return;
        const auto& old = *accepted;
        const auto& a = old.message.header;
        const auto& b = c.message.header;
        require(a.publisher_id == b.publisher_id &&
                    old.message.body.at("control_epoch") == c.message.body.at("control_epoch") &&
                    old.origin.publisher_id == c.origin.publisher_id,
                "Hand publisher/epoch/origin mismatch");
        require(a.session_id == b.session_id ? b.sequence > a.sequence
                                             : b.sample_mono_us > a.sample_mono_us,
                "Regressing hand command");
    }
    bool observe(const std::array<Snapshot, 2>& data, uint64_t now) {
        bool clear = true;
        for (int s = 0; s < 2; ++s) {
            faults[s] = Json::object();
            if (!cfg.sides[s])
                continue;
            if (data[s].recent(now, cfg.feedback_us)) {
                for (int i = 0; i < 6; ++i) {
                    auto& v = data[s].data;
                    std::string why = v.errors[i] ? "device error " + std::to_string(v.errors[i])
                                      : v.position[i] < 0 || v.position[i] > 1000
                                          ? "invalid position feedback"
                                          : "";
                    if (!why.empty())
                        faults[s][std::to_string(i)] = why;
                }
            } else
                clear = false;
            // Keep diagnostics strict, and never clear a device fault on a
            // failed poll. A transport error must not hide known device errors.
            if (!data[s].good(now, cfg.feedback_us))
                clear = false;
            if (!faults[s].empty()) {
                clear = false;
                if (fault.empty()) {
                    fault = std::string(sides[s]) + ": " + faults[s].dump();
                    safe_applied = false;
                    log(fault);
                }
            }
            if (faults[s].size() == 6 && !open_fault[s]) {
                open_fault[s] = true;
                safe_applied = false;
            }
        }
        if (!fault.empty() && clear) {
            Logger::info("Hand feedback recovered");
            fault.clear();
            open_fault = {};
            holds = {};
            settings = {};
            angles = {};
            safe_applied = false;
            return true;
        }
        return false;
    }
    void refreshEquivalent(Command& c) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!pending) return;
        const auto& next = *pending;
        // Consume an actual newer command, never alter timestamps to renew an
        // old one. Only identical authorized targets can share in-flight work.
        if (next.targets == c.targets && successor(c, next, cfg.command_us)) {
            c = std::move(*pending);
            pending.reset();
        }
    }
    void writeTargets(const std::array<Values, 2>& targets, Command* c = nullptr,
                      bool best_effort = false) {
        std::mutex command_mutex;
        auto check = [&] {
            if (c) {
                std::lock_guard<std::mutex> lock(command_mutex);
                refreshEquivalent(*c);
                const auto now = monotonic_us();
                if (stop || !live(*c, now, cfg.command_us)) throw CommandExpired(*c, now, cfg.command_us);
            }
        };
        const auto both = [&](const auto& operation) {
            std::future<void> right;
            if (cfg.sides[1]) right = std::async(std::launch::async, [&] { operation(1); });
            std::exception_ptr error;
            try { if (cfg.sides[0]) operation(0); }
            catch (...) { error = std::current_exception(); }
            try { if (right.valid()) right.get(); }
            catch (const CommandExpired&) { if (!error) error = std::current_exception(); }
            catch (...) { error = std::current_exception(); } // Device errors outrank expiry.
            if (error) std::rethrow_exception(error);
        };
        auto configureSide = [&](int s) {
            Values speed, force;
            for (int i = 0; i < 6; ++i) {
                const bool held = targets[s][i] == -1;
                speed[i] = held ? cfg.hold_speed : cfg.speed;
                force[i] = held ? cfg.hold_force : cfg.force;
            }
            auto& applied = settings[s];
            if (!applied.mode) {
                check();
                devices[s]->write(mode_set, Values{}, check);
                applied.mode = true;
            }
            if (applied.speed != speed) {
                check();
                devices[s]->write(speed_set, speed, check);
                applied.speed = speed;
            }
            if (applied.force != force) {
                check();
                devices[s]->write(force_set, force, check);
                applied.force = force;
            }
        };
        auto angleSide = [&](int s) {
            Values changed;
            bool any = false;
            for (int i = 0; i < 6; ++i) {
                changed[i] =
                    angles[s] && (*angles[s])[i] == targets[s][i] ? unchanged : targets[s][i];
                any |= changed[i] != unchanged;
            }
            if (any) {
                check();
                // A segmented write may stop after only some channels. Until
                // all segments succeed, the previous angle cache is unknown.
                angles[s].reset();
                devices[s]->write(angle_set, changed, check);
                angles[s] = targets[s];
            }
        };
        if (best_effort) {
            std::array<std::string, 2> side_errors;
            both([&](int s) {
                try {
                    configureSide(s);
                    angleSide(s);
                } catch (const std::exception& e) {
                    settings[s] = {};
                    angles[s].reset();
                    side_errors[s] = std::string(sides[s]) + ": " + e.what() + "; ";
                }
            });
            const auto errors = side_errors[0] + side_errors[1];
            write_error = errors;
            require(errors.empty(), errors);
            return;
        }
        try {
            // Prepare every hand before moving either; ACK follows all writes.
            both(configureSide);
            both(angleSide);
            check();
            write_error.clear();
        } catch (const CommandExpired& e) {
            // Preserve confirmed mode/speed/force writes. Expiry says nothing
            // about the device losing those settings; ACK still does not move.
            write_error = e.what();
            command_valid = false;
            safe_applied = false;
            throw;
        } catch (const std::exception& e) {
            settings = {};
            angles = {};
            write_error = e.what();
            safe_applied = false;
            throw;
        }
    }
    std::array<Values, 2> protection() const {
        std::array<Values, 2> targets;
        for (int s = 0; s < 2; ++s) {
            if (open_fault[s])
                targets[s] = open_pose;
            else
                targets[s].fill(-1);
        }
        return targets;
    }
    void safe(const std::array<Snapshot, 2>& data, uint64_t now) {
        bool unavailable = false;
        for (int s = 0; s < 2; ++s)
            if (cfg.sides[s] && accepted && !data[s].recent(now, cfg.feedback_us))
                unavailable = true;
        failed = !fault.empty() || unavailable || !write_error.empty();
        auto targets = failed ? protection() : std::array<Values, 2>{open_pose, open_pose};
        command_valid = false;
        writeTargets(targets, nullptr, true);
        if (!failed)
            holds = {};
        for (int s = 0; s < 2; ++s) {
            commanded[s] = targets[s][0] == -1 ? Json::array() : normalized(targets[s]);
            if (!failed)
                requested[s] = Json::array();
        }
        command_valid = false;
        safe_applied = true;
    }
    void apply(Command& c, const std::array<Snapshot, 2>& data, uint64_t now) {
        validateGuard(c);
        if (!c.message.header.valid) {
            safe(data, now);
            requested = {Json::array(), Json::array()};
            commanded = requested;
            accepted = c;
            return;
        }
        auto next = holds;
        auto targets = c.targets;
        for (int s = 0; s < 2; ++s)
            for (int i = 0; i < 6; ++i) {
                auto& h = next[s][i];
                const auto& d = data[s];
                int target = c.targets[s][i];
                bool changed = target != h.requested;
                if (!fresh(h.command, now, cfg.command_us) ||
                    (h.held >= 0 &&
                     ((h.at_target && changed) || h.direction * (target - h.held) > 0))) {
                    h = Hold{};
                    h.target_since = now;
                    h.errors = d.errors;
                }
                if (changed)
                    h.samples.clear();
                h.requested = target;
                h.command = now;
                if (h.held >= 0) {
                    targets[s][i] = -1;
                    continue;
                }
                if (!d.good(now, cfg.feedback_us) || d.data.position[i] < 0 ||
                    d.data.position[i] > 1000) {
                    h.residual_since = h.sample = 0;
                    h.samples.clear();
                    continue;
                }
                if (h.errors != d.errors || d.stamp < h.sample ||
                    (h.sample && d.stamp - h.sample >= cfg.feedback_us)) {
                    h.residual_since = 0;
                    h.samples.clear();
                }
                h.errors = d.errors;
                int actual = d.data.position[i], residual = actual - target,
                    dir = residual >= 0 ? 1 : -1;
                if (dir != h.direction || std::abs(residual) <= cfg.threshold)
                    h.residual_since = 0;
                h.direction = dir;
                if (d.stamp >= h.target_since && d.stamp != h.sample) {
                    if (std::abs(residual) > cfg.threshold && !h.residual_since)
                        h.residual_since = d.stamp;
                    h.samples.emplace_back(d.stamp, actual);
                    while (h.samples.size() > 1 && h.samples[1].first + cfg.hold_us <= d.stamp)
                        h.samples.pop_front();
                    int lo = actual, hi = actual;
                    for (const auto& p : h.samples) {
                        lo = std::min(lo, p.second);
                        hi = std::max(hi, p.second);
                    }
                    bool stable =
                        d.stamp - h.samples.front().first >= cfg.hold_us && hi - lo < cfg.threshold;
                    if (stable || (h.residual_since && d.stamp - h.residual_since >= cfg.hold_us)) {
                        h.held = actual;
                        h.at_target = std::abs(residual) <= cfg.threshold;
                        targets[s][i] = -1;
                    }
                }
                h.sample = d.stamp;
            }
        failed = !fault.empty() || !write_error.empty();
        for (int s = 0; s < 2; ++s)
            if (cfg.sides[s] && !data[s].recent(now, cfg.feedback_us))
                failed = true;
        if (failed)
            targets = protection();
        writeTargets(targets, &c);
        holds = std::move(next);
        for (int s = 0; s < 2; ++s) {
            requested[s] = normalized(c.targets[s]);
            Values shown = targets[s];
            for (int i = 0; i < 6; ++i)
                if (shown[i] == -1)
                    shown[i] = holds[s][i].held >= 0 ? holds[s][i].held : data[s].data.position[i];
            commanded[s] = normalized(shown);
        }
        accepted = c;
        command_valid = true;
        safe_applied = false;
    }
    void publishControl() {
        Json j = {{"hold_control_error", fault},
                  {"command_valid", command_valid},
                  {"feedback_only", false},
                  {"accepted_command", nullptr}};
        if (accepted) {
            const auto& h = accepted->message.header;
            j["accepted_command"] = {{"publisher_id", h.publisher_id},
                                     {"session_id", h.session_id},
                                     {"sequence", h.sequence},
                                     {"sample_mono_us", h.sample_mono_us},
                                     {"control_epoch", accepted->message.body.at("control_epoch")}};
        }
        for (int s = 0; s < 2; ++s) {
            Json active = Json::array(), positions = Json::array(), phases = Json::array();
            for (const auto& h : holds[s]) {
                active.push_back(h.held >= 0);
                positions.push_back(h.held >= 0 ? Json(h.held / 1000.) : Json(nullptr));
                phases.push_back({{"phase", failed        ? "failed"
                                            : h.held >= 0 ? "holding"
                                                          : "tracking"}});
            }
            j["hands"][sides[s]] = {{"requested_drive_position_normalized", requested[s]},
                                    {"commanded_drive_position_normalized", commanded[s]},
                                    {"closing_hold_active", active},
                                    {"closing_hold_position_normalized", positions},
                                    {"hold_control", phases},
                                    {"fault_protection",
                                     {{"fault_channels", faults[s]},
                                      {"fault_count", faults[s].size()},
                                      {"open_requested", open_fault[s]}}}};
        }
        std::lock_guard<std::mutex> lock(mutex);
        output = std::move(j);
        output_deadline =
            accepted ? std::min({accepted->received, accepted->message.header.sample_mono_us,
                                 accepted->origin.sample_mono_us}) +
                           cfg.command_us
                     : 0;
    }
    void controlLoop() {
        uint64_t next_command = 0;
        while (!stop) {
            std::optional<Command> c;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (pending && (!pending->message.header.valid || monotonic_us() >= next_command)) {
                    c = std::move(pending);
                    pending.reset();
                    next_command = monotonic_us() + cfg.write_period_us;
                }
            }
            auto data = snapshots();
            auto now = monotonic_us();
            observe(data, now);
            try {
                if (c) {
                    try {
                        apply(*c, data, now);
                    } catch (const CommandExpired& e) {
                        // A trajectory sample can expire while a newer target
                        // is already queued. Do not insert slow stop/configure
                        // writes between those samples. The next iteration
                        // revalidates the NEW command and recomputes its holds;
                        // neither this old target nor its ACK is renewed.
                        bool retry = !stop && !failed && fault.empty();
                        const auto latest_data = snapshots();
                        for (int s = 0; s < 2; ++s)
                            if (cfg.sides[s] && !latest_data[s].recent(monotonic_us(), cfg.feedback_us))
                                retry = false;
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            retry = retry && pending && successor(*c, *pending, cfg.command_us);
                        }
                        if (retry) {
                            write_error.clear();
                            const auto retry_time = monotonic_us();
                            if (!retry_logged || retry_time - retry_logged >= 10000000) {
                                Logger::warn("Hand: {}; retrying fresh queued target", e.what());
                                retry_logged = retry_time;
                            }
                            publishControl();
                            continue;
                        }
                        log(e.what());
                    } catch (const std::exception& e) {
                        log(e.what());
                    }
                }
                // I/O above may outlast a poll or a feedback deadline. Decide
                // protection from current samples, not the pre-write snapshot.
                data = snapshots();
                now = monotonic_us();
                observe(data, now);
                if (command_valid && accepted && !live(*accepted, now, cfg.command_us))
                    safe_applied = false;
                bool unavailable = false;
                for (int s = 0; s < 2; ++s)
                    if (cfg.sides[s] && accepted && !data[s].recent(now, cfg.feedback_us))
                        unavailable = true;
                bool pending_live = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    pending_live = accepted && pending && successor(*accepted, *pending, cfg.command_us);
                }
                if (((!command_valid && !pending_live) || !fault.empty() || !write_error.empty() || unavailable ||
                     (accepted && !live(*accepted, monotonic_us(), cfg.command_us) && !pending_live)) &&
                    !safe_applied)
                    safe(data, monotonic_us());
            } catch (const std::exception& e) {
                log(e.what());
            }
            publishControl();
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait_for(lock, std::chrono::milliseconds(2),
                          [&] { return stop.load() || (pending && (!pending->message.header.valid || monotonic_us() >= next_command)); });
        }
        try {
            auto data = snapshots();
            observe(data, monotonic_us());
            safe(data, monotonic_us());
        } catch (const std::exception& e) {
            log(std::string("shutdown: ") + e.what());
        }
    }
};
Runtime::Runtime(Config c, std::array<std::unique_ptr<Device>, 2> d)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(d))) {}
Runtime::~Runtime() = default;
unsigned Runtime::periodUs() const { return impl_->cfg.period_us; }
void Runtime::receive(const Message& message) {
    try {
        auto c = parse(message);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->pending = std::move(c);
        impl_->wake.notify_all();
    } catch (const std::exception& e) {
        Logger::warn("Reject hand command: {}", e.what());
    }
}
Message Runtime::state() {
    auto& p = *impl_;
    std::array<Snapshot, 2> samples;
    Json body;
    uint64_t deadline;
    {
        std::lock_guard<std::mutex> lock(p.mutex);
        samples = p.samples;
        body = p.output;
        deadline = p.output_deadline;
    }
    uint64_t now = monotonic_us();
    bool valid = true;
    bool command_valid = body["command_valid"].get<bool>();
    command_valid &= deadline && now < deadline;
    body["command_valid"] = command_valid;
    for (int s = 0; s < 2; ++s) {
        auto& h = body["hands"][sides[s]];
        const auto& d = samples[s];
        bool available = p.cfg.sides[s] && fresh(d.stamp, now, p.cfg.feedback_us);
        bool positions = std::all_of(d.data.position.begin(), d.data.position.end(),
                                     [](int x) { return x >= 0 && x <= 1000; });
        const bool telemetry_available = p.cfg.sides[s] && fresh(d.telemetry_stamp, now, p.cfg.feedback_us);
        bool good = available && d.good(now, p.cfg.feedback_us) && positions;
        if (p.cfg.sides[s])
            valid &= good;
        int code = *std::max_element(d.data.errors.begin(), d.data.errors.end());
        h["valid"] = good;
        h["status"] = !p.cfg.sides[s] || !d.stamp ? "OFFLINE"
                      : !available || !telemetry_available ? "STALE"
                      : !d.error.empty() || !d.telemetry_error.empty() ? "ERROR"
                      : code                      ? "ERROR"
                      : command_valid             ? "ACTIVE"
                                                  : "READY";
        h["feedback_available"] = available;
        h["position_source"] = p.cfg.fake ? "simulated" : "angle_act_register";
        h["sample_time_basis"] = p.cfg.fake ? "host_simulation" : "host_rs485_read";
        h["sample_mono_us"] = d.stamp ? Json(d.stamp) : Json(nullptr);
        h["feedback_age_ms"] =
            d.stamp && now >= d.stamp ? Json((now - d.stamp) / 1000.) : Json(nullptr);
        h["feedback_samples"] = d.reads;
        h["feedback_io_errors"] = d.errors + d.telemetry_errors;
        h["feedback_last_error"] = d.error.empty() ? d.telemetry_error : d.error;
        h["telemetry_sample_mono_us"] = d.telemetry_stamp ? Json(d.telemetry_stamp) : Json(nullptr);
        h["telemetry_age_ms"] = d.telemetry_stamp && now >= d.telemetry_stamp
                                     ? Json((now - d.telemetry_stamp) / 1000.) : Json(nullptr);
        h["telemetry_samples"] = d.telemetry_reads;
        h["telemetry_available"] = telemetry_available && d.telemetry_error.empty();
        h["grasp_verified"] = false;
        h["error_code"] = telemetry_available ? code : 0;
        h["drive_position_raw"] = available ? Json(d.data.position) : Json::array();
        h["drive_position_normalized"] =
            available && positions ? normalized(d.data.position) : Json::array();
        h["force"] = telemetry_available ? Json(d.data.force) : Json::array();
        h["current_register_raw"] = telemetry_available ? Json(d.data.current) : Json::array();
        Values current = d.data.current;
        for (int& x : current)
            if (x >= 32768)
                x -= 65536;
        h["current"] = telemetry_available ? Json(current) : Json::array();
        h["error_codes"] = telemetry_available ? Json(d.data.errors) : Json::array();
        h["status_codes"] = telemetry_available ? Json(d.data.status) : Json::array();
        h["temperature"] = telemetry_available ? Json(d.data.temperature) : Json::array();
    }
    if (p.cfg.fake)
        body["simulated"] = true;
    Message m;
    m.topic = Topic::hand_state;
    m.header = {"1.0",           ++p.sequence, utc_us(), now, local_clock_id(),
                p.cfg.publisher, p.session,    valid};
    m.body = std::move(body);
    return m;
}
class Bus::Impl {
  public:
    std::atomic<bool> stop{false};
    std::thread thread;
    Impl(Endpoint& runtime, const std::string& publish, const std::string& endpoint) {
        thread = std::thread([this, &runtime, publish, endpoint] {
            try {
                zmq::context_t context(1);
                zmq::socket_t pub(context, zmq::socket_type::pub),
                    sub(context, zmq::socket_type::sub);
                configure(pub);
                configure(sub);
                subscribe(sub, "hand.command");
                pub.connect(publish);
                sub.connect(endpoint);
                ReceiveState receive_state;
                uint64_t next = 0;
                while (!stop) {
                    WireMessage wire;
                    std::string error;
                    for (int n = 0; n < 32; ++n) {
                        auto r = receive(sub, receive_state, wire, error);
                        if (r == ReceiveResult::empty)
                            break;
                        Message m;
                        if (r == ReceiveResult::received &&
                            decode(wire.topic, wire.payload, m, error))
                            runtime.receive(m);
                    }
                    auto now = monotonic_us();
                    if (now >= next) {
                        std::string wire;
                        auto m = runtime.state();
                        if (encode(m, wire, error))
                            send(pub, "hand.state", wire);
                        next = now + runtime.periodUs();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } catch (const std::exception& e) {
                Logger::error("Hand bus stopped: {}", e.what());
            }
        });
    }
    ~Impl() {
        stop = true;
        if (thread.joinable())
            thread.join();
    }
};
Bus::Bus(Endpoint& r, const std::string& p, const std::string& s)
    : impl_(std::make_unique<Impl>(r, p, s)) {}
Bus::~Bus() = default;
} // namespace aviator::hand

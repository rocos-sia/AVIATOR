#include "calibration_hand.hpp"
#include "Logger.hpp"
#include "runtime.hpp"
#include "../../../examples/xCoreSDK_RH56FTP_485/src/protocol.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

namespace aviator::calibration {
namespace {
using Json = nlohmann::json;
constexpr const char* sides[]{"left", "right"};
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
hand::Values reversed(hand::Values v) {
    std::reverse(v.begin(), v.end());
    return v;
}
Json normalized(const hand::Values& values) {
    Json result = Json::array();
    for (int v : values) result.push_back(v / 1000.);
    return result;
}
bool fresh(uint64_t stamp, uint64_t now, uint64_t age) {
    return stamp && now >= stamp && now - stamp < age;
}
}
class CalibrationHands::Impl {
 public:
    struct Command {
        Message message;
        uint64_t received = 0;
        std::array<uint64_t, 2> revision{};
        std::array<hand::Values, 2> targets{};
    };
    struct Side {
        hand::Values position{}, force{}, errors{};
        uint64_t stamp = 0, reads = 0, failures = 0, applied = 0, blocked = 0;
        bool healthy = false, recovering = false;
        std::string error;
    };
    hand::Config cfg;
    std::array<hand::Transfer, 2> transfers;
    std::array<hand::Values, 2> open;
    std::array<Side, 2> sides_state;
    std::array<std::thread, 2> workers;
    std::mutex mutex;
    std::condition_variable wake;
    std::atomic<bool> stop{false};
    std::optional<Command> latest;
    std::string session = new_instance_id();
    uint64_t sequence = 0;
    Impl(hand::Config config, std::array<hand::Transfer, 2> io,
         std::array<hand::Values, 2> opened)
        : cfg(std::move(config)), transfers(std::move(io)), open(opened) {
        cfg.feedback_us = std::max(cfg.feedback_us, 3000000u);
        for (int s = 0; s < 2; ++s) {
            require(cfg.fake || bool(transfers[s]), "Missing hand transport");
            rh56::validate(rh56::angleSet, open[s]);
        }
        try {
            for (int s = 0; s < 2; ++s) workers[s] = std::thread([this, s] { run(s); });
        } catch (...) {
            stop = true;
            wake.notify_all();
            for (auto& worker : workers) if (worker.joinable()) worker.join();
            throw;
        }
    }
    ~Impl() {
        stop = true;
        wake.notify_all();
        for (auto& worker : workers) if (worker.joinable()) worker.join();
    }
    bool live(const Command& c, uint64_t now) const {
        return fresh(c.received, now, cfg.command_us) &&
               fresh(c.message.header.sample_mono_us, now, cfg.command_us) &&
               fresh(c.message.body.at("origin").at("sample_mono_us").get<uint64_t>(), now, cfg.command_us);
    }
    void receive(const Message& message) {
        try {
            Command c;
            c.message = message;
            c.received = monotonic_us();
            std::string wire, error;
            Origin origin;
            require(message.topic == Topic::hand_command && encode(message, wire, error), "Invalid hand command");
            require(read_origin(message.body, origin, error), "Invalid hand command origin");
            require(message.header.clock_id == local_clock_id() && origin.clock_id == local_clock_id(), "Hand clock mismatch");
            require(live(c, c.received), "Stale hand command");
            if (message.header.valid) {
                require(message.body.at("mode") == "NORMALIZED_POSITION", "Expected normalized hand positions");
                for (int s = 0; s < 2; ++s) {
                    const auto& h = message.body.at("hands").at(sides[s]);
                    const auto& revision = h.at("calibration_revision");
                    require(revision.is_number_unsigned() && revision.get<uint64_t>() > 0, "Invalid calibration revision");
                    c.revision[s] = revision.get<uint64_t>();
                    const auto& values = h.at("drive_position_normalized");
                    require(values.is_array() && values.size() == 6, "Expected six hand channels");
                    for (int i = 0; i < 6; ++i) {
                        require(values[i].is_number(), "Invalid hand target");
                        double v = values[i].get<double>();
                        require(std::isfinite(v) && v >= 0 && v <= 1, "Hand target outside [0,1]");
                        c.targets[s][i] = int(std::floor(v * 1000 + .5));
                    }
                }
            }
            std::lock_guard<std::mutex> lock(mutex);
            if (latest) {
                const auto& old = latest->message;
                require(message.header.publisher_id == old.header.publisher_id &&
                        message.header.session_id == old.header.session_id &&
                        message.body.at("control_epoch") == old.body.at("control_epoch") &&
                        origin.publisher_id == old.body.at("origin").at("publisher_id") &&
                        origin.session_id == old.body.at("origin").at("session_id"), "Foreign hand publisher/session");
                require(message.header.sequence > old.header.sequence &&
                        message.header.sample_mono_us >= old.header.sample_mono_us, "Regressing hand command");
                if (!message.header.valid) c.revision = latest->revision;
                if (message.header.valid && old.header.valid)
                    for (int s = 0; s < 2; ++s)
                        require(c.revision[s] >= latest->revision[s] &&
                                (c.revision[s] != latest->revision[s] || c.targets[s] == latest->targets[s]),
                                "Changed target without new calibration revision");
            }
            if (!message.header.valid) {
                for (int s = 0; s < 2; ++s) {
                    sides_state[s].blocked = std::max(sides_state[s].blocked, c.revision[s]);
                    sides_state[s].applied = 0;
                }
            }
            latest = std::move(c);
            wake.notify_all();
        } catch (const std::exception& e) {
            Logger::warn("Calibration hand command rejected: {}", e.what());
        }
    }
    void run(int s) {
        auto fake = cfg.fake ? hand::fakeDevice(cfg.motion_rate) : nullptr;
        rh56::Hand device(cfg.ids[s], transfers[s]);
        auto write = [&](uint16_t address, const hand::Values& values) {
            if (fake) fake->write(address, values);
            else device.write(address, reversed(values));
        };
        auto read = [&] {
            Side result;
            if (fake) {
                auto sample = fake->read();
                result.position = sample.position;
                result.force = sample.force;
                result.errors = sample.errors;
            } else {
                result.position = reversed(device.read(rh56::angleAct));
                result.force = reversed(device.read(rh56::forceAct, true));
                result.errors = reversed(device.errors());
            }
            rh56::validate(rh56::angleSet, result.position);
            return result;
        };
        auto apply = [&](const hand::Values& target, const std::function<void()>& guard) {
            hand::Values speed, force;
            speed.fill(cfg.speed); force.fill(cfg.force);
            guard(); write(rh56::speedSet, speed);
            guard(); write(rh56::forceSet, force);
            guard(); write(rh56::angleSet, target);
        };
        uint64_t next_read = 0;
        while (!stop) {
            try {
                if (monotonic_us() >= next_read) {
                    auto sample = read();
                    std::lock_guard<std::mutex> lock(mutex);
                    auto& state = sides_state[s];
                    const bool recovered = state.recovering;
                    if (recovered && latest) state.blocked = std::max(state.blocked, latest->revision[s]);
                    state.position = sample.position; state.force = sample.force; state.errors = sample.errors;
                    state.stamp = monotonic_us(); ++state.reads;
                    require(std::all_of(state.errors.begin(), state.errors.end(), [](int x) { return x == 0; }),
                            "RH56 actuator error; waiting for recovery");
                    state.healthy = true; state.recovering = false; state.error.clear();
                    if (recovered) Logger::info("Calibration hand {} recovered; click open/close to send a new command", sides[s]);
                    next_read = monotonic_us() + 500000;
                }
                std::optional<Command> command;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    auto& state = sides_state[s];
                    if (latest && (!latest->message.header.valid || !live(*latest, monotonic_us()))) {
                        state.blocked = std::max(state.blocked, latest->revision[s]);
                        state.applied = 0;
                    }
                    if (latest && latest->message.header.valid && live(*latest, monotonic_us()) && state.healthy &&
                        fresh(state.stamp, monotonic_us(), cfg.feedback_us) &&
                        latest->revision[s] > state.blocked && latest->revision[s] != state.applied)
                        command = latest;
                }
                if (command) {
                    auto guard = [&] {
                        std::lock_guard<std::mutex> lock(mutex);
                        require(!stop && latest && latest->message.header.valid && live(*latest, monotonic_us()) &&
                                latest->revision[s] == command->revision[s] &&
                                command->revision[s] > sides_state[s].blocked, "Hand command cancelled or expired");
                    };
                    apply(command->targets[s], guard);
                    std::lock_guard<std::mutex> lock(mutex);
                    sides_state[s].applied = command->revision[s];
                    next_read = 0;
                }
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(mutex);
                auto& state = sides_state[s];
                if (!state.recovering || state.error != e.what())
                    Logger::warn("Calibration hand {}: {}", sides[s], e.what());
                state.healthy = false; state.recovering = true; state.applied = 0;
                state.error = e.what(); ++state.failures;
                if (latest) state.blocked = std::max(state.blocked, latest->revision[s]);
                next_read = monotonic_us() + 500000;
            }
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait_for(lock, std::chrono::milliseconds(10));
        }
        // The session ends owned dragging before joining these workers. A
        // failed hand gets no blind writes; the other hand still releases.
        try {
            const auto sample = read();
            require(std::all_of(sample.errors.begin(), sample.errors.end(), [](int x) { return x == 0; }),
                    "RH56 actuator error during release");
            apply(open[s], [] {});
        } catch (const std::exception& e) {
            Logger::error("Calibration hand {} release failed: {}", sides[s], e.what());
        }
    }
    Message state() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = monotonic_us();
        bool valid = true, accepted = true;
        Json body = {{"calibration_control", true}, {"feedback_only", false},
                     {"hold_control_error", ""}, {"accepted_command", nullptr}};
        if (latest) {
            const auto& h = latest->message.header;
            body["accepted_command"] = {{"publisher_id", h.publisher_id}, {"session_id", h.session_id},
                {"sequence", h.sequence}, {"sample_mono_us", h.sample_mono_us},
                {"control_epoch", latest->message.body.at("control_epoch")}};
        }
        for (int s = 0; s < 2; ++s) {
            const auto& d = sides_state[s];
            bool available = fresh(d.stamp, now, cfg.feedback_us);
            bool healthy = d.healthy && available;
            bool applied = healthy && latest && latest->message.header.valid && live(*latest, now) &&
                           d.applied == latest->revision[s] && d.applied > d.blocked;
            valid &= healthy; accepted &= applied;
            body["hands"][sides[s]] = {{"valid", healthy}, {"feedback_available", available},
                {"status", !healthy ? "RECOVERING" : applied ? "ACTIVE" : "READY"},
                {"sample_mono_us", d.stamp}, {"sample_time_basis", cfg.fake ? "host_simulation" : "host_rs485_read"},
                {"drive_position_normalized", available ? normalized(d.position) : Json::array()},
                {"force", available ? Json(d.force) : Json::array()}, {"error_codes", d.errors},
                {"feedback_last_error", d.error}, {"feedback_io_errors", d.failures},
                {"feedback_samples", d.reads}, {"grasp_verified", false},
                {"command_valid", applied}, {"applied_revision", d.applied},
                {"retry_required", healthy && latest && latest->revision[s] <= d.blocked}};
        }
        body["command_valid"] = accepted;
        if (cfg.fake) body["simulated"] = true;
        Message message;
        message.topic = Topic::hand_state;
        message.header = {"1.0", ++sequence, utc_us(), now, local_clock_id(), cfg.publisher, session, valid};
        message.body = std::move(body);
        return message;
    }
};
CalibrationHands::CalibrationHands(hand::Config c, std::array<hand::Transfer, 2> transfers,
                                   std::array<hand::Values, 2> open)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(transfers), open)) {}
CalibrationHands::~CalibrationHands() = default;
void CalibrationHands::receive(const Message& message) { impl_->receive(message); }
unsigned CalibrationHands::periodUs() const { return impl_->cfg.period_us; }
Message CalibrationHands::state() { return impl_->state(); }
} // namespace aviator::calibration

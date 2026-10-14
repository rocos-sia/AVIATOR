#pragma once
#include "protocol.hpp"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace demo {
struct ArmConfig {
    std::string robotIp;
    int handId = 1;
};
struct Config {
    std::array<ArmConfig, 2> arms{{{"192.168.1.160", 1}, {"192.168.2.160", 1}}};
    int pollMs = 500;
    bool offline = false;
    bool init485 = false; // Configuration-only session; AR requires a subsequent controller restart.
};
enum class Phase { Idle, Starting, Connected, Recovering, Stopping, Failed };
struct ArmState {
    Phase phase = Phase::Idle;
    bool handReady = false;
    rh56::Values angles{}, forces{}, errors{}, speeds{}, limits{};
    long long feedbackMs = 0;
    double transferMs = 0;
    std::string error;
};
struct Snapshot {
    std::array<ArmState, 2> arms;
    // ready means BOTH arms are ready; individual controls use their own arm state.
    bool active = false, ready = false, stopping = false;
};
enum class Action { Apply, Read };
struct Command {
    Action action = Action::Apply;
    rh56::Values angles{}, speeds{200,200,200,200,200,200}, limits{300,300,300,300,300,300};
};
long long nowMs();
class Backend {
public:
    Backend();
    ~Backend();
    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;
    void start(const Config& config);
    void stop();
    bool submit(int armMask, const Command& command); // bit 0 = left, bit 1 = right
    bool submitBoth(const std::array<Command, 2>& commands);
    Snapshot snapshot() const;
    std::vector<std::string> takeLogs();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace demo

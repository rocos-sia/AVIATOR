#pragma once
#include <array>
#include <string>

namespace flight_gateway {
struct KeyboardConfig {
    // Normalized travel per second and normalized absolute travel limits.
    double roll_speed = 1.0, pitch_speed = 1.0;
    double roll_limit = 1.0, pitch_limit = 1.0;
};
struct Rs422Config {
    std::string profile, channel_a, channel_b;
    unsigned receive_timeout_ms = 50, recovery_frames = 10, switch_max_step = 100;
    std::string release_root;
    bool allow_test_identity = false;
};
struct SimulatorConfig {
    bool require_joystick = true;
    // Test injection is opt-in, per channel; zero disables corruption.
    bool send_a = true, send_b = true;
    unsigned corrupt_every = 0;
};
struct Config {
    std::string role = "robot";
    Rs422Config rs422;
    SimulatorConfig simulator;
    std::string source, device, publish, subscribe, service, core_session, lock_file;
    unsigned roll_axis, pitch_axis, input_timeout_ms, service_timeout_ms;
    bool invert_roll, invert_pitch;
    std::array<std::string, 11> buttons;
    KeyboardConfig keyboard;
};
// Installed share/aviator/config/flight.yaml first; source-tree config otherwise.
std::string default_config_path();
Config load_config(const std::string& path);
} // namespace flight_gateway

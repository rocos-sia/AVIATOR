#pragma once
#include <array>
#include <string>
namespace flight_simulator {
struct KeyboardConfig {
    double roll_speed_deg_s = 50.0, pitch_speed_mm_s = 75.0;
    double roll_limit_deg = 50.0, pitch_min_mm = -80.0, pitch_max_mm = 70.0;
};
struct Rs422Config {
    std::string channel_a, channel_b;
    unsigned receive_timeout_ms = 50;
};
struct SimulatorConfig {
    bool require_joystick = true, send_a = true, send_b = true;
    unsigned corrupt_every = 0;
};
struct Config {
    std::string device = "auto";
    unsigned roll_axis = 0, pitch_axis = 1, input_timeout_ms = 100;
    bool invert_roll = false, invert_pitch = false;
    std::array<std::string, 11> buttons{{"enter_standby", "grasp_wheel", "start_control",
                                         "exit_control", "leave_wheel", "reset_error", "", "", "",
                                         "", ""}};
    KeyboardConfig keyboard;
    Rs422Config rs422;
    SimulatorConfig simulator;
};
Config load_config(const std::string& path);
std::string default_config_path();
} // namespace flight_simulator

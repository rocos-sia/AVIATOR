#pragma once
#include <array>
#include <string>
namespace flight_simulator {
struct KeyboardConfig {
    double roll_speed = 1, pitch_speed = 1, roll_limit = 1, pitch_limit = 1;
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

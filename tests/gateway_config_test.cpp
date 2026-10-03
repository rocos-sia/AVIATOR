#include "config.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace {
void check(bool ok, const char* reason) {
    if (!ok)
        throw std::runtime_error(reason);
}
} // namespace
int main(int argc, char** argv) {
    const auto file = std::filesystem::temp_directory_path() /
                      ("aviator-flight-config-" + std::to_string(getpid()) + ".yaml");
    try {
        check(argc == 2, "expected source config path");
        const auto config = flight_gateway::load_config(argv[1]);
        check(config.roll_axis == 0 && config.pitch_axis == 1 && config.input_timeout_ms == 100 &&
                  config.service_timeout_ms == 100 && config.buttons[0] == "enter_standby" &&
                  config.buttons[5] == "reset_error" && config.buttons[10].empty(),
              "default configuration");
        check(config.core_session.empty() &&
                  config.lock_file == "/tmp/flight_gateway-" + std::to_string(getuid()) + ".lock",
              "empty session/lock defaults");
        check(std::filesystem::is_regular_file(flight_gateway::default_config_path()),
              "default path resolution");
        const auto write = [&](const YAML::Node& root) {
            std::ofstream out(file);
            out << root;
        };
        const auto rejected = [&](const YAML::Node& root) {
            write(root);
            bool failed = false;
            try {
                flight_gateway::load_config(file.string());
            } catch (const std::invalid_argument&) {
                failed = true;
            }
            check(failed, "invalid configuration accepted");
        };
        auto root = YAML::LoadFile(argv[1]);
        {
            auto keyboard = YAML::Clone(root);
            keyboard["source"] = "keyboard";
            keyboard["device"] = "/dev/input/by-id/test-event-kbd";
            keyboard["keyboard"]["roll_speed"] = 0.5;
            keyboard["keyboard"]["pitch_speed"] = 2.0;
            keyboard["keyboard"]["roll_limit"] = 0.75;
            write(keyboard);
            const auto parsed = flight_gateway::load_config(file.string());
            check(parsed.source == "keyboard" && parsed.keyboard.roll_speed == 0.5 &&
                      parsed.keyboard.pitch_speed == 2 && parsed.keyboard.roll_limit == 0.75,
                  "keyboard configuration ignored");
            keyboard.remove("keyboard");
            write(keyboard);
            check(flight_gateway::load_config(file.string()).keyboard.roll_speed == 1,
                  "optional keyboard defaults");
            for (const auto* key : {"roll_speed", "pitch_speed", "roll_limit", "pitch_limit"}) {
                for (const auto* invalid : {"0", "-1", ".nan", ".inf", "abc"}) {
                    auto bad = YAML::Clone(root);
                    bad["keyboard"][key] = invalid;
                    rejected(bad);
                }
            }
            for (const auto* key : {"roll_limit", "pitch_limit"}) {
                auto bad = YAML::Clone(root);
                bad["keyboard"][key] = 1.01;
                rejected(bad);
            }
            auto bad = YAML::Clone(root);
            bad["keyboard"]["roll_spped"] = 1;
            rejected(bad);
            bad = YAML::Clone(root);
            bad["keyboard"] = "invalid";
            rejected(bad);
            auto legacy = YAML::Clone(root);
            legacy["source"] = "joystick";
            legacy.remove("keyboard");
            write(legacy);
            check(flight_gateway::load_config(file.string()).source == "joystick",
                  "legacy joystick config must remain valid");
        }
        root["buttons"][10] = "exit_control";
        root["invert_roll"] = true;
        root["service_timeout_ms"] = 500;
        root["core_session"] = "legacy-text-marker";
        write(root);
        const auto changed = flight_gateway::load_config(file.string());
        check(changed.buttons[10] == "exit_control" && changed.invert_roll &&
                  changed.service_timeout_ms == 500 && !changed.core_session.empty(),
              "custom configuration ignored");
        for (const auto* key : {"device", "buttons", "publish"}) {
            auto bad = YAML::Clone(root);
            bad.remove(key);
            rejected(bad);
        }
        for (const auto* key : {"input_timeout_ms", "service_timeout_ms"}) {
            auto bad = YAML::Clone(root);
            bad[key] = 0;
            rejected(bad);
            bad[key] = -1;
            rejected(bad);
            bad[key] = 10001;
            rejected(bad);
        }
        auto bad = YAML::Clone(root);
        bad["source"] = "rs422";
        rejected(bad);
        bad = YAML::Clone(root);
        bad["pitch_axis"] = 0;
        rejected(bad);
        bad = YAML::Clone(root);
        bad["input_timeout_ms"] = 101;
        rejected(bad);
        bad = YAML::Clone(root);
        bad["subscribe"] = bad["publish"];
        rejected(bad);
        bad = YAML::Clone(root);
        bad["buttons"].push_back("none");
        rejected(bad);
        bad = YAML::Clone(root);
        bad["buttons"][0] = "Boot";
        rejected(bad);
        bad = YAML::Clone(root);
        bad["servcie"] = "typo";
        rejected(bad);
        write(root);
        {
            std::ofstream out(file, std::ios::app);
            out << "\nsource: joystick\n";
        }
        bool duplicate = false;
        try {
            flight_gateway::load_config(file.string());
        } catch (const std::invalid_argument&) {
            duplicate = true;
        }
        check(duplicate, "duplicate key accepted");
        std::filesystem::remove(file);
        std::cout << "flight config tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::filesystem::remove(file);
        std::cerr << e.what() << '\n';
        return 1;
    }
}

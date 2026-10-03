#include "config.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
using monitor::Json;
void check(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}
int main() {
    char directory[] = "/tmp/aviator-monitor-config-XXXXXX";
    if (!mkdtemp(directory))
        return 1;
    const auto path = std::filesystem::path(directory) / "monitor.yaml";
    try {
        auto config = monitor::parse_config(
            "version: 1\nsources:\n  arm.state: simulation\npreview:\n  endpoint: ''\n");
        check(config["sources"]["arm.state"] == "simulation", "YAML source override");
        check(config["preview"]["endpoint"] == "", "empty endpoint");
        check(config["hand_calibration"].is_null(), "default calibration");
        config["sources"]["arm.state"] = "123";
        config["sources"]["hand.state"] = "true";
        config["arm_joints"]["left"][0]["offset_rad"] = .123456789123456;
        monitor::save_config(path, config);
        check(monitor::load_config(path) == config, "YAML type/precision roundtrip");
        check(monitor::parse_config(config.dump()) == config, "legacy JSON compatibility");
        for (const auto* text :
             {"sources: {arm.state: null}", "sources: {typo: camera}",
              "timeouts_ms: {arm.state: 0}", "preview: {endpoint: ipc://invalid}",
              "preview: {typo: 1}", "version: 1\nversion: 1", "version: 1\n---\nversion: 1",
              "preview: {timeout_ms: .nan}", "&loop [*loop]", "not-a-map", "["}) {
            bool rejected = false;
            try {
                monitor::parse_config(text);
            } catch (const std::exception&) {
                rejected = true;
            }
            check(rejected, "invalid YAML accepted");
        }
        auto invalid = config;
        invalid["timeouts_ms"]["arm.state"] = -1;
        bool rejected = false;
        try {
            monitor::save_config(path, invalid);
        } catch (const std::exception&) {
            rejected = true;
        }
        check(rejected && monitor::load_config(path) == config, "invalid save altered disk");
        rejected = false;
        try {
            monitor::save_config((path / "missing.yaml").string(), config);
        } catch (const std::exception&) {
            rejected = true;
        }
        check(rejected && monitor::load_config(path) == config, "failed save altered disk");
        std::filesystem::remove_all(directory);
        std::cout << "monitor YAML configuration tests passed\n";
    } catch (const std::exception& e) {
        std::filesystem::remove_all(directory);
        std::cerr << e.what() << '\n';
        return 1;
    }
}

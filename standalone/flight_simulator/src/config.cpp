#include "config.hpp"
#include "rs422.hpp"
#include <cmath>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
namespace flight_simulator {
namespace {
void require(bool value, const std::string& reason) {
    if (!value)
        throw std::invalid_argument(reason);
}
void keys(const YAML::Node& map, std::initializer_list<const char*> allowed) {
    require(map.IsMap(), "expected mapping");
    std::set<std::string> seen;
    for (const auto& entry : map) {
        const auto key = entry.first.as<std::string>();
        bool found = false;
        for (const auto* candidate : allowed)
            found = found || key == candidate;
        require(found && seen.insert(key).second, "unknown or duplicate key: " + key);
    }
}
unsigned integer(const YAML::Node& value, unsigned maximum, bool zero = false) {
    const auto text = value.as<std::string>();
    require(!text.empty() && text.find_first_not_of("0123456789") == std::string::npos,
            "expected unsigned integer");
    const auto n = std::stoull(text);
    require(n <= maximum && (zero || n > 0), "integer out of range");
    return n;
}
bool boolean(const YAML::Node& value) {
    const auto s = value.as<std::string>();
    require(s == "true" || s == "false", "expected true or false");
    return s == "true";
}
} // namespace
std::string default_config_path() {
    const auto exe = std::filesystem::read_symlink("/proc/self/exe");
    const auto installed = exe.parent_path() / "../share/flight_simulator/config.yaml";
    if (std::filesystem::exists(installed))
        return installed.lexically_normal().string();
    const auto local = exe.parent_path() / "config.yaml";
    if (std::filesystem::exists(local))
        return local.string();
    throw std::runtime_error("Configuration missing; use --config <path>");
}
Config load_config(const std::string& path) {
    try {
        const auto root = YAML::LoadFile(path);
        keys(root, {"profile", "device", "roll_axis", "pitch_axis", "invert_roll", "invert_pitch",
                    "input_timeout_ms", "buttons", "keyboard", "rs422", "simulator"});
        require(root["profile"].as<std::string>() == "aviator-bench-v1",
                "profile must be aviator-bench-v1");
        Config c;
        if (root["device"])
            c.device = root["device"].as<std::string>();
        if (c.device != "auto")
            integer(root["device"], std::numeric_limits<int>::max(), true);
        if (root["roll_axis"])
            c.roll_axis = integer(root["roll_axis"], 255, true);
        if (root["pitch_axis"])
            c.pitch_axis = integer(root["pitch_axis"], 255, true);
        require(c.roll_axis != c.pitch_axis, "axes must differ");
        if (root["invert_roll"])
            c.invert_roll = boolean(root["invert_roll"]);
        if (root["invert_pitch"])
            c.invert_pitch = boolean(root["invert_pitch"]);
        if (root["input_timeout_ms"])
            c.input_timeout_ms = integer(root["input_timeout_ms"], 100);
        if (auto buttons = root["buttons"]) {
            require(buttons.IsSequence() && buttons.size() == 11, "buttons must have 11 entries");
            for (unsigned i = 0; i < 11; ++i) {
                const auto op = buttons[i].as<std::string>();
                require(op == "none" || rs422::operationCode(op), "unknown operation: " + op);
                c.buttons[i] = op == "none" ? "" : op;
            }
        }
        if (auto k = root["keyboard"]) {
            keys(k, {"roll_speed", "pitch_speed", "roll_limit", "pitch_limit"});
            for (auto pair : {std::pair<const char*, double*>{"roll_speed", &c.keyboard.roll_speed},
                              {"pitch_speed", &c.keyboard.pitch_speed},
                              {"roll_limit", &c.keyboard.roll_limit},
                              {"pitch_limit", &c.keyboard.pitch_limit}}) {
                if (k[pair.first])
                    *pair.second = k[pair.first].as<double>();
                require(std::isfinite(*pair.second) && *pair.second > 0,
                        "keyboard values must be finite and positive");
            }
            require(c.keyboard.roll_limit <= 1 && c.keyboard.pitch_limit <= 1,
                    "keyboard limit must be <=1");
        }
        const auto rs = root["rs422"];
        keys(rs, {"channel_a", "channel_b", "receive_timeout_ms"});
        c.rs422.channel_a = rs["channel_a"].as<std::string>();
        c.rs422.channel_b = rs["channel_b"].as<std::string>();
        require(std::filesystem::path(c.rs422.channel_a).is_absolute() &&
                    std::filesystem::path(c.rs422.channel_b).is_absolute() &&
                    c.rs422.channel_a != c.rs422.channel_b,
                "two distinct absolute serial paths required");
        struct stat a {
        }, b{};
        if (::stat(c.rs422.channel_a.c_str(), &a) == 0 &&
            ::stat(c.rs422.channel_b.c_str(), &b) == 0)
            require(!(a.st_dev == b.st_dev && a.st_ino == b.st_ino) &&
                        !(S_ISCHR(a.st_mode) && S_ISCHR(b.st_mode) && a.st_rdev == b.st_rdev),
                    "serial paths alias one device");
        if (rs["receive_timeout_ms"])
            c.rs422.receive_timeout_ms = integer(rs["receive_timeout_ms"], 100);
        require(c.rs422.receive_timeout_ms >= 25, "receive timeout must be 25..100 ms");
        if (auto sim = root["simulator"]) {
            keys(sim, {"require_joystick", "send_a", "send_b", "corrupt_every"});
            if (sim["require_joystick"])
                c.simulator.require_joystick = boolean(sim["require_joystick"]);
            if (sim["send_a"])
                c.simulator.send_a = boolean(sim["send_a"]);
            if (sim["send_b"])
                c.simulator.send_b = boolean(sim["send_b"]);
            if (sim["corrupt_every"])
                c.simulator.corrupt_every = integer(sim["corrupt_every"], 100000, true);
        }
        return c;
    } catch (const std::exception& e) {
        throw std::invalid_argument("Config " + path + ": " + e.what());
    }
}
} // namespace flight_simulator

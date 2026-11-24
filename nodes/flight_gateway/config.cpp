#include "config.hpp"
#include "service.hpp"
#include <filesystem>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unistd.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

namespace flight_gateway {
namespace {
void require(bool ok, const std::string& reason) {
    if (!ok)
        throw std::invalid_argument(reason);
}
unsigned number(const YAML::Node& node, unsigned maximum, bool zero = false) {
    require(node.IsScalar() && (node.Tag() == "?" || node.Tag() == "tag:yaml.org,2002:int"),
            "expected integer");
    const auto text = node.Scalar();
    require(!text.empty() && text.find_first_not_of("0123456789") == std::string::npos,
            "expected unsigned integer");
    const auto value = std::stoull(text);
    require(value <= maximum && (zero || value > 0), "integer out of range");
    return static_cast<unsigned>(value);
}
bool boolean(const YAML::Node& node) {
    require(node.IsScalar() && (node.Tag() == "?" || node.Tag() == "tag:yaml.org,2002:bool") &&
                (node.Scalar() == "true" || node.Scalar() == "false"),
            "expected true or false");
    return node.Scalar() == "true";
}
std::string string(const YAML::Node& node) {
    require(node.IsScalar(), "expected string");
    return node.as<std::string>();
}
} // namespace
std::string default_config_path() {
    const auto installed = std::filesystem::read_symlink("/proc/self/exe").parent_path() /
                           "../share/aviator/config/flight.yaml";
    if (std::filesystem::exists(installed))
        return installed.lexically_normal().string();
    return AVIATOR_FLIGHT_CONFIG;
}
Config load_config(const std::string& path) {
    try {
        const auto root = YAML::LoadFile(path);
        const std::set<std::string> allowed{"source",
                                            "device",
                                            "publish",
                                            "subscribe",
                                            "service",
                                            "core_session",
                                            "lock_file",
                                            "roll_axis",
                                            "pitch_axis",
                                            "invert_roll",
                                            "invert_pitch",
                                            "input_timeout_ms",
                                            "service_timeout_ms",
                                            "buttons", "keyboard", "role", "rs422", "simulator"};
        require(root.IsMap(), "expected mapping");
        std::set<std::string> seen;
        for (const auto& item : root) {
            const auto key = string(item.first);
            require(allowed.count(key) && seen.insert(key).second,
                    "unknown or duplicate key: " + key);
        }
        for (const auto& key : allowed)
            require(key == "keyboard" || key == "role" || key == "rs422" || key == "simulator" || seen.count(key), "missing key: " + key);
        Config c;
        if(root["role"]) c.role = string(root["role"]);
        require(c.role == "robot" || c.role == "flight_simulator", "invalid role");
        c.source = string(root["source"]);
        require(c.source == "joystick" || c.source == "rs422", "source must be joystick or rs422");
        c.device = string(root["device"]);
        if (c.source == "joystick" && c.device != "auto") {
            require(!c.device.empty() && c.device.find_first_not_of("0123456789") == std::string::npos,
                    "device must be auto or a non-negative SDL joystick index");
            require(std::stoull(c.device) <= std::numeric_limits<int>::max(),
                    "SDL joystick index out of range");
        }
        c.publish = string(root["publish"]);
        c.subscribe = string(root["subscribe"]);
        c.service = string(root["service"]);
        require(c.publish.rfind("tcp://", 0) == 0 && c.subscribe.rfind("tcp://", 0) == 0 &&
                    c.service.rfind("tcp://127.0.0.1:", 0) == 0,
                "TCP bus and local TCP service endpoints required");
        require(c.publish != c.subscribe && c.publish != c.service && c.subscribe != c.service,
                "endpoints must differ");
        c.core_session = string(root["core_session"]);
        c.lock_file = string(root["lock_file"]);
        if (c.lock_file.empty())
            c.lock_file = "/tmp/flight_gateway-" + std::to_string(getuid()) + ".lock";
        require(std::filesystem::path(c.lock_file).is_absolute(),
                "lock_file must be absolute or empty");
        c.roll_axis = number(root["roll_axis"], 255, true);
        c.pitch_axis = number(root["pitch_axis"], 255, true);
        require(c.roll_axis != c.pitch_axis, "axes must differ");
        c.invert_roll = boolean(root["invert_roll"]);
        c.invert_pitch = boolean(root["invert_pitch"]);
        c.input_timeout_ms = number(root["input_timeout_ms"], 100);
        c.service_timeout_ms = number(root["service_timeout_ms"], 10000);
        if (const auto keyboard = root["keyboard"]) {
            require(keyboard.IsMap(), "keyboard must be a mapping");
            std::set<std::string> seen;
            for (const auto& item : keyboard) {
                const auto key = item.first.as<std::string>();
                require(seen.insert(key).second, "duplicate keyboard key");
                const double v = item.second.as<double>();
                require(std::isfinite(v), "keyboard value must be finite");
                if (key == "roll_speed_deg_s") c.keyboard.roll_speed_deg_s = v;
                else if (key == "pitch_speed_mm_s") c.keyboard.pitch_speed_mm_s = v;
                else if (key == "roll_limit_deg") c.keyboard.roll_limit_deg = v;
                else if (key == "pitch_min_mm") c.keyboard.pitch_min_mm = v;
                else if (key == "pitch_max_mm") c.keyboard.pitch_max_mm = v;
                else require(false, "unknown keyboard key (ICD 1120 physical units required): " + key);
            }
            require(c.keyboard.roll_speed_deg_s > 0 && c.keyboard.pitch_speed_mm_s > 0,
                    "keyboard speeds must be finite and positive");
            require(c.keyboard.roll_limit_deg > 0 && c.keyboard.roll_limit_deg <= 50 &&
                    c.keyboard.pitch_min_mm >= -80 && c.keyboard.pitch_min_mm < 0 &&
                    c.keyboard.pitch_max_mm > 0 && c.keyboard.pitch_max_mm <= 70,
                    "keyboard limits must be within roll +/-50 deg and pitch [-80,70] mm");
        }
        const auto buttons = root["buttons"];
        require(buttons.IsSequence() && buttons.size() == c.buttons.size(),
                "buttons must contain exactly 11 entries");
        for (unsigned i = 0; i < c.buttons.size(); ++i) {
            auto operation = string(buttons[i]);
            require(operation == "none" || aviator::state_operation(operation),
                    "invalid button operation: " + operation);
            c.buttons[i] = operation == "none" ? "" : operation;
        }
        if(const auto rs = root["rs422"]) {
            require(rs.IsMap(), "rs422 must be a map");
            std::set<std::string> keys;
            for(const auto& item : rs) {
                const auto key = string(item.first);
                require(keys.insert(key).second, "duplicate rs422 key: " + key);
                if(key == "profile") c.rs422.profile = string(item.second);
                else if(key == "channel_a") c.rs422.channel_a = string(item.second);
                else if(key == "channel_b") c.rs422.channel_b = string(item.second);
                else if(key == "receive_timeout_ms") c.rs422.receive_timeout_ms = number(item.second,100);
                else if(key == "recovery_frames") c.rs422.recovery_frames = number(item.second,100);
                else if(key == "switch_max_roll_deg" || key == "switch_max_pitch_mm") {
                    const double v = item.second.as<double>();
                    require(std::isfinite(v) && v >= 0 && v <= (key == "switch_max_roll_deg" ? 100 : 150),
                            "invalid physical switch threshold");
                    if (key == "switch_max_roll_deg") c.rs422.switch_max_roll_deg = v;
                    else c.rs422.switch_max_pitch_mm = v;
                }
                else if(key == "release_root") c.rs422.release_root = string(item.second);
                else if(key == "allow_test_identity") c.rs422.allow_test_identity = boolean(item.second);
                else if(key == "camera_id") c.rs422.camera_id = string(item.second);
                else if(key == "camera_publisher") c.rs422.camera_publisher = string(item.second);
                else require(false, "unknown rs422 key: " + key);
            }
            require(c.rs422.profile == "aviator-icd-1120", "rs422.profile must be aviator-icd-1120");
            require(!c.rs422.camera_id.empty() && !c.rs422.camera_publisher.empty(), "RS422 camera identity must not be empty");
            require(c.rs422.receive_timeout_ms >= 25, "RS422 timeout must be 25..100 ms");
            require(std::filesystem::path(c.rs422.channel_a).is_absolute() &&
                    std::filesystem::path(c.rs422.channel_b).is_absolute() &&
                    c.rs422.channel_a != c.rs422.channel_b, "two distinct absolute serial paths required");
            struct stat a{}, b{};
            if(::stat(c.rs422.channel_a.c_str(), &a)==0 && ::stat(c.rs422.channel_b.c_str(), &b)==0)
                require(!(a.st_dev==b.st_dev && a.st_ino==b.st_ino) &&
                        !(S_ISCHR(a.st_mode) && S_ISCHR(b.st_mode) && a.st_rdev==b.st_rdev),
                        "serial paths alias one device");
            if(!c.rs422.release_root.empty() && std::filesystem::path(c.rs422.release_root).is_relative())
                c.rs422.release_root = (std::filesystem::absolute(path).parent_path()/c.rs422.release_root).lexically_normal().string();
        }
        if(const auto sim = root["simulator"]) {
            require(sim.IsMap(), "simulator must be a map");
            std::set<std::string> keys;
            for(const auto& item : sim) {
                const auto key = string(item.first);
                require(keys.insert(key).second, "duplicate simulator key: " + key);
                if(key == "require_joystick") c.simulator.require_joystick = boolean(item.second);
                else if(key == "send_a") c.simulator.send_a = boolean(item.second);
                else if(key == "send_b") c.simulator.send_b = boolean(item.second);
                else if(key == "corrupt_every") c.simulator.corrupt_every = number(item.second,100000,true);
                else require(false, "unknown simulator key: " + key);
            }
        }
        require(c.role != "flight_simulator" || (c.source == "joystick" && root["rs422"]),
                "flight_simulator requires joystick source and rs422 configuration");
        return c;
    } catch (const std::exception& e) {
        throw std::invalid_argument("flight config " + path + ": " + e.what());
    }
}
} // namespace flight_gateway

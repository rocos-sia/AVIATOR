#include "config.hpp"
#include "service.hpp"
#include <filesystem>
#include <linux/input.h>
#include <regex>
#include <set>
#include <stdexcept>
#include <unistd.h>
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
                                            "buttons"};
        require(root.IsMap(), "expected mapping");
        std::set<std::string> seen;
        for (const auto& item : root) {
            const auto key = string(item.first);
            require(allowed.count(key) && seen.insert(key).second,
                    "unknown or duplicate key: " + key);
        }
        for (const auto& key : allowed)
            require(seen.count(key), "missing key: " + key);
        Config c;
        c.source = string(root["source"]);
        require(c.source == "joystick",
                c.source == "rs422" ? "RS422 mode not implemented" : "unknown source");
        c.device = string(root["device"]);
        require(std::filesystem::path(c.device).is_absolute(),
                "device must be an absolute evdev path");
        c.publish = string(root["publish"]);
        c.subscribe = string(root["subscribe"]);
        c.service = string(root["service"]);
        require(c.publish.rfind("tcp://", 0) == 0 && c.subscribe.rfind("tcp://", 0) == 0 &&
                    c.service.rfind("tcp://127.0.0.1:", 0) == 0,
                "TCP bus and local TCP service endpoints required");
        require(c.publish != c.subscribe && c.publish != c.service && c.subscribe != c.service,
                "endpoints must differ");
        c.core_session = string(root["core_session"]);
        require(c.core_session.empty() ||
                    std::regex_match(c.core_session,
                                     std::regex("[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-"
                                                "9a-fA-F]{4}-[0-9a-fA-F]{12}")),
                "invalid core_session UUID");
        c.lock_file = string(root["lock_file"]);
        if (c.lock_file.empty())
            c.lock_file = "/tmp/flight_gateway-" + std::to_string(getuid()) + ".lock";
        require(std::filesystem::path(c.lock_file).is_absolute(),
                "lock_file must be absolute or empty");
        c.roll_axis = number(root["roll_axis"], ABS_MAX, true);
        c.pitch_axis = number(root["pitch_axis"], ABS_MAX, true);
        require(c.roll_axis != c.pitch_axis, "axes must differ");
        c.invert_roll = boolean(root["invert_roll"]);
        c.invert_pitch = boolean(root["invert_pitch"]);
        c.input_timeout_ms = number(root["input_timeout_ms"], 100);
        c.service_timeout_ms = number(root["service_timeout_ms"], 10000);
        const auto buttons = root["buttons"];
        require(buttons.IsSequence() && buttons.size() == c.buttons.size(),
                "buttons must contain exactly 11 entries");
        for (unsigned i = 0; i < c.buttons.size(); ++i) {
            auto operation = string(buttons[i]);
            require(operation == "none" || aviator::state_operation(operation),
                    "invalid button operation: " + operation);
            c.buttons[i] = operation == "none" ? "" : operation;
        }
        return c;
    } catch (const std::exception& e) {
        throw std::invalid_argument("flight config " + path + ": " + e.what());
    }
}
} // namespace flight_gateway

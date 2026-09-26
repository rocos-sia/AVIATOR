#include "protocol.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace aviator {
namespace {
using Json = nlohmann::json;
constexpr std::array<std::string_view, 11> names{
    "flight.command", "flight.state", "arm.command", "arm.state",
    "hand.command", "hand.state", "camera.command", "camera.detection",
    "system.state", "system.diagnostic", "system.event"};
constexpr std::array<std::string_view, 11> types{
    "FlightCommand", "FlightState", "ArmCommand", "ArmState",
    "HandCommand", "HandState", "CameraCommand", "CameraDetection",
    "SystemState", "SystemDiagnostic", "SystemEvent"};
constexpr std::array<const char*, 9> header_keys{
    "msg_type", "version", "sequence", "timestamp", "sample_mono_us",
    "clock_id", "publisher_id", "session_id", "valid"};

void require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

std::string identifier(const Json& value, const char* key) {
    const auto& field = value.at(key);
    require(field.is_string(), "identifier must be a string");
    auto result = field.get<std::string>();
    require(!result.empty() && result.size() <= 128 &&
            result.find('\0') == std::string::npos, "invalid identifier");
    return result;
}

std::string uuid(const Json& value, const char* key) {
    auto result = identifier(value, key);
    require(result.size() == 36, "UUID must have 36 characters");
    for (std::size_t i = 0; i < result.size(); ++i) {
        const char c = result[i];
        const bool separator = i == 8 || i == 13 || i == 18 || i == 23;
        require(separator ? c == '-' : ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')),
                "UUID must use lowercase canonical form");
    }
    return result;
}

std::uint64_t integer(const Json& value, const char* key, bool nonzero = false) {
    const auto& field = value.at(key);
    require(field.is_number_integer(), "expected integer");
    if (!field.is_number_unsigned())
        require(field.get<std::int64_t>() >= 0, "negative integer");
    const auto result = field.get<std::uint64_t>();
    require(result <= max_json_integer && (!nonzero || result > 0),
            "integer outside uint53 range");
    return result;
}

void version(const std::string& value) {
    require(value.size() >= 3 && value.size() <= 16 && value.substr(0, 2) == "1.",
            "unsupported version");
    require(value.size() == 3 || value[2] != '0', "noncanonical minor version");
    for (std::size_t i = 2; i < value.size(); ++i)
        require(value[i] >= '0' && value[i] <= '9', "invalid version");
}

// Enforce resource/numeric limits before recursive business decoding.
void check_value(const Json& value, unsigned depth = 1) {
    require(depth <= 16, "JSON nesting exceeds 16");
    if (value.is_number_unsigned())
        require(value.get<std::uint64_t>() <= max_json_integer, "integer exceeds uint53");
    else if (value.is_number_integer()) {
        const auto number = value.get<std::int64_t>();
        require(number >= -static_cast<std::int64_t>(max_json_integer) &&
                number <= static_cast<std::int64_t>(max_json_integer), "integer exceeds exact range");
    } else if (value.is_number_float())
        require(std::isfinite(value.get<double>()), "non-finite number");
    if (value.is_structured())
        for (const auto& child : value) check_value(child, depth + 1);
}

void check_commands(const Message& message) {
    const auto& body = message.body;
    if (message.topic == Topic::flight_command) {
        const auto source = identifier(body, "source");
        require(source == "FLIGHT" || source == "JOYSTICK", "unknown control source");
        const auto& control = body.at("control");
        for (const char* key : {"roll", "pitch"}) {
            const auto& axis = control.at(key);
            require(axis.is_number(), "control axis must be numeric");
            const double number = axis.get<double>();
            require(number >= -1 && number <= 1, "control axis out of range");
        }
    }
    if (message.topic == Topic::arm_command || message.topic == Topic::hand_command) {
        uuid(body, "control_epoch");
        identifier(body, "mode");
        Origin origin;
        std::string error;
        require(read_origin(body, origin, error), "invalid command origin");
    }
}
} // namespace

std::string_view topic_name(Topic topic) { return names.at(static_cast<std::size_t>(topic)); }
std::string_view message_type(Topic topic) { return types.at(static_cast<std::size_t>(topic)); }
std::optional<Topic> find_topic(std::string_view name) {
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name) return static_cast<Topic>(i);
    return std::nullopt;
}

bool read_origin(const Json& body, Origin& output, std::string& error) {
    try {
        const auto& value = body.at("origin");
        Origin origin;
        origin.publisher_id = identifier(value, "publisher_id");
        origin.session_id = uuid(value, "session_id");
        origin.sequence = integer(value, "sequence", true);
        origin.sample_mono_us = integer(value, "sample_mono_us");
        origin.clock_id = identifier(value, "clock_id");
        if (value.contains("topic"))
            require(value.at("topic") == "flight.command", "unsupported origin topic");
        output = std::move(origin);
        error.clear();
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}

bool decode(std::string_view topic, std::string_view payload,
            Message& output, std::string& error) {
    try {
        const auto type = find_topic(topic);
        require(type.has_value(), "unknown topic");
        require(!payload.empty() && payload.size() <= max_payload_bytes, "invalid payload size");
        require(payload.substr(0, 3) != "\xef\xbb\xbf", "UTF-8 BOM is not allowed");
        require(payload.find('\0') == std::string_view::npos, "NUL is not allowed");
        std::vector<std::unordered_set<std::string>> keys;
        auto callback = [&](int depth, Json::parse_event_t event, Json& parsed) {
            require(depth < 16, "JSON nesting exceeds 16");
            if (event == Json::parse_event_t::object_start) keys.emplace_back();
            if (event == Json::parse_event_t::key)
                require(keys.back().insert(parsed.get<std::string>()).second, "duplicate JSON key");
            if (event == Json::parse_event_t::object_end) keys.pop_back();
            return true;
        };
        auto json = Json::parse(payload.begin(), payload.end(), callback);
        require(json.is_object(), "payload must be an object");
        check_value(json);
        Message message;
        message.topic = *type;
        require(json.at("msg_type") == message_type(*type), "topic/msg_type mismatch");
        auto& h = message.header;
        h.version = identifier(json, "version");
        version(h.version);
        h.sequence = integer(json, "sequence", true);
        h.timestamp = integer(json, "timestamp");
        h.sample_mono_us = integer(json, "sample_mono_us");
        h.clock_id = identifier(json, "clock_id");
        h.publisher_id = identifier(json, "publisher_id");
        h.session_id = uuid(json, "session_id");
        require(json.at("valid").is_boolean(), "valid must be boolean");
        h.valid = json.at("valid").get<bool>();
        for (const auto* key : header_keys) json.erase(key);
        message.body = std::move(json);
        check_commands(message);
        output = std::move(message);
        error.clear();
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}

bool encode(const Message& message, std::string& payload, std::string& error) {
    try {
        auto json = message.body;
        require(json.is_object(), "body must be an object");
        for (const auto* key : header_keys)
            require(!json.contains(key), "body shadows common header");
        const auto& h = message.header;
        json["msg_type"] = message_type(message.topic);
        json["version"] = h.version;
        json["sequence"] = h.sequence;
        json["timestamp"] = h.timestamp;
        json["sample_mono_us"] = h.sample_mono_us;
        json["clock_id"] = h.clock_id;
        json["publisher_id"] = h.publisher_id;
        json["session_id"] = h.session_id;
        json["valid"] = h.valid;
        check_value(json); // dump() otherwise silently converts NaN/Inf to null.
        auto encoded = json.dump();
        Message checked;
        if (!decode(topic_name(message.topic), encoded, checked, error)) return false;
        payload = std::move(encoded);
        error.clear();
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}
} // namespace aviator

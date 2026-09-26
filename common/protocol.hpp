#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <nlohmann/json.hpp>

namespace aviator {

constexpr std::uint64_t max_json_integer = 9007199254740991ULL;
constexpr std::size_t max_payload_bytes = 65536;
constexpr std::size_t max_topic_bytes = 128;

enum class Topic {
    flight_command, flight_state, arm_command, arm_state,
    hand_command, hand_state, camera_command, camera_detection,
    system_state, system_diagnostic, system_event
};

std::string_view topic_name(Topic topic);
std::string_view message_type(Topic topic);
std::optional<Topic> find_topic(std::string_view name);

struct Header {
    std::string version = "1.0";
    std::uint64_t sequence = 0;
    std::uint64_t timestamp = 0;
    std::uint64_t sample_mono_us = 0;
    std::string clock_id;
    std::string publisher_id;
    std::string session_id;
    bool valid = false;
};

struct Origin {
    std::string publisher_id;
    std::string session_id;
    std::uint64_t sequence = 0;
    std::uint64_t sample_mono_us = 0;
    std::string clock_id;
};

// Non-real-time wire model. Nodes convert body to their own typed control data.
struct Message {
    Topic topic = Topic::flight_command;
    Header header;
    nlohmann::json body = nlohmann::json::object();
};

// Validate the common envelope and command source/origin fields, not hardware
// limits or complete business schemas. Failure leaves output unchanged.
bool decode(std::string_view topic, std::string_view payload,
            Message& output, std::string& error);
bool encode(const Message& message, std::string& payload, std::string& error);
bool read_origin(const nlohmann::json& body, Origin& output, std::string& error);

} // namespace aviator

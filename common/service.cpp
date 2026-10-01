#include "service.hpp"
#include "runtime.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace aviator {
namespace {
void require(bool ok, const char* reason) {
    if (!ok)
        throw std::invalid_argument(reason);
}
bool uuid(const std::string& value) {
    if (value.size() != 36)
        return false;
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) {
            if (value[i] != '-')
                return false;
        } else if (!std::isxdigit(static_cast<unsigned char>(value[i])))
            return false;
    }
    return true;
}
} // namespace
bool state_operation(std::string_view op) {
    return op == "enter_standby" || op == "grasp_wheel" || op == "start_control" ||
           op == "exit_control" || op == "leave_wheel" || op == "reset_error";
}
nlohmann::json service_schema(bool reply) {
    using J = nlohmann::json;
    J p = {{"msg_type", {{"type", "string"}, {"const", reply ? "ServiceReply" : "ServiceRequest"}}},
           {"version", {{"type", "string"}, {"const", "1.0"}}}};
    for (const char* key : {"request_id", "client_id", "client_session_id"})
        p[key] = {{"type", "string"}, {"minLength", 1}};
    p["timestamp"] = {{"type", "integer"}, {"minimum", 0}, {"maximum", max_json_integer}};
    if (reply) {
        p["server_id"] = p["server_session_id"] = {{"type", "string"}, {"minLength", 1}};
        p["status"] = {{"type", "string"},
                       {"enum",
                        {"ACCEPTED", "RUNNING", "COMPLETED", "REJECTED", "FAILED", "CANCELLED",
                         "EXPIRED", "UNKNOWN"}}};
        p["error_code"] = p["timestamp"];
        p["result"] = {{"type", "object"}};
    } else {
        for (const char* key : {"clock_id", "operation", "target"})
            p[key] = {{"type", "string"}, {"minLength", 1}};
        p["issued_mono_us"] = p["timestamp"];
        p["deadline_ms"] = p["timestamp"];
        p["deadline_ms"]["minimum"] = 1;
        p["parameters"] = {{"type", "object"}};
    }
    J required = J::array();
    for (auto it = p.begin(); it != p.end(); ++it)
        required.push_back(it.key());
    p["request_id"]["format"] = "uuid";
    if (!reply)
        p["gateway_observation"] = {{"type", "string"},
                                    {"enum", {"QUEUED", "NOT_SENT", "TIMEOUT_UNKNOWN"}}};
    if (reply)
        p["message"] = {{"type", "string"}};
    return {{"$schema", "http://json-schema.org/draft-07/schema#"},
            {"type", "object"},
            {"required", required},
            {"properties", p}};
}
void validate_service(const nlohmann::json& m) {
    require(m.is_object(), "Service must be object");
    const bool reply = m.value("msg_type", "") == "ServiceReply";
    const auto schema = service_schema(reply);
    for (const auto& key : schema.at("required"))
        require(m.contains(key.get<std::string>()), "Missing service field");
    for (auto it = schema.at("properties").begin(); it != schema.at("properties").end(); ++it) {
        if (!m.contains(it.key()))
            continue;
        const auto& v = m.at(it.key());
        const auto& rule = it.value();
        const auto type = rule.at("type").get<std::string>();
        if (type == "string")
            require(v.is_string() &&
                        (!rule.contains("minLength") || !v.get_ref<const std::string&>().empty()),
                    "Invalid service string");
        if (type == "object")
            require(v.is_object(), "Invalid service object");
        if (type == "integer")
            require(v.is_number_integer() && v.get<double>() >= rule.at("minimum").get<double>() &&
                        v.get<double>() <= max_json_integer,
                    "Invalid service uint53");
        if (rule.contains("const"))
            require(v == rule.at("const"), "Unsupported service type/version");
        if (rule.contains("enum"))
            require(std::find(rule.at("enum").begin(), rule.at("enum").end(), v) !=
                        rule.at("enum").end(),
                    "Invalid service status");
    }
    require(uuid(m.at("request_id").get<std::string>()), "Invalid request UUID");
}
nlohmann::json decode_service(std::string_view bytes) {
    try {
        require(!bytes.empty() && bytes.size() <= max_payload_bytes &&
                    bytes.find('\0') == bytes.npos && bytes.substr(0, 3) != "\xef\xbb\xbf",
                "Invalid service payload size/encoding");
        std::vector<std::unordered_set<std::string>> keys;
        auto m = nlohmann::json::parse(
            bytes.begin(), bytes.end(),
            [&](int depth, nlohmann::json::parse_event_t e, nlohmann::json& value) {
                require(depth < 16, "Service nesting limit");
                if (e == nlohmann::json::parse_event_t::object_start)
                    keys.emplace_back();
                if (e == nlohmann::json::parse_event_t::key)
                    require(keys.back().insert(value.get<std::string>()).second,
                            "Duplicate service key");
                if (e == nlohmann::json::parse_event_t::object_end)
                    keys.pop_back();
                if (value.is_number())
                    require(std::isfinite(value.get<double>()) &&
                                std::abs(value.get<double>()) <= max_json_integer,
                            "Invalid service number");
                return true;
            });
        validate_service(m);
        return m;
    } catch (const std::exception& e) {
        throw std::invalid_argument(e.what());
    }
}
nlohmann::json make_service_request(const std::string& client, const std::string& session,
                                    const std::string& target, const std::string& operation,
                                    const nlohmann::json& parameters, unsigned deadline_ms) {
    nlohmann::json m = {{"msg_type", "ServiceRequest"},     {"version", "1.0"},
                        {"request_id", new_session_id()},   {"client_id", client},
                        {"client_session_id", session},     {"timestamp", utc_us()},
                        {"issued_mono_us", monotonic_us()}, {"clock_id", local_clock_id()},
                        {"deadline_ms", deadline_ms},       {"target", target},
                        {"operation", operation},           {"parameters", parameters}};
    validate_service(m);
    return m;
}
nlohmann::json make_service_reply(const nlohmann::json& request, const std::string& server,
                                  const std::string& session, const std::string& status,
                                  std::uint64_t error_code, const nlohmann::json& result) {
    validate_service(request);
    require(request.at("msg_type") == "ServiceRequest", "Expected request");
    nlohmann::json m = {{"msg_type", "ServiceReply"},
                        {"version", "1.0"},
                        {"request_id", request.at("request_id")},
                        {"client_id", request.at("client_id")},
                        {"client_session_id", request.at("client_session_id")},
                        {"server_id", server},
                        {"server_session_id", session},
                        {"timestamp", utc_us()},
                        {"status", status},
                        {"error_code", error_code},
                        {"result", result}};
    validate_service(m);
    return m;
}
bool matches_service_reply(const nlohmann::json& request, const nlohmann::json& reply) {
    try {
        validate_service(request);
        validate_service(reply);
        return request.at("msg_type") == "ServiceRequest" &&
               reply.at("msg_type") == "ServiceReply" &&
               request.at("request_id") == reply.at("request_id") &&
               request.at("client_id") == reply.at("client_id") &&
               request.at("target") == reply.at("server_id");
    } catch (const std::exception&) {
        return false;
    }
}
bool send_service(zmq::socket_t& socket, const nlohmann::json& m) {
    const auto bytes = m.dump();
    decode_service(bytes);
    return bool(socket.send(zmq::buffer(bytes), zmq::send_flags::dontwait));
}
ReceiveResult receive_service(zmq::socket_t& socket, ReceiveState& state, std::string& bytes,
                              nlohmann::json& m, std::string& error) {
    for (unsigned i = 0; i < 32; ++i) {
        zmq::message_t frame;
        if (!socket.recv(frame, zmq::recv_flags::dontwait))
            return ReceiveResult::empty;
        if (state.discarding || frame.more()) {
            state.discarding = frame.more();
            if (state.discarding)
                continue;
            error = "Multipart service message rejected";
            return ReceiveResult::rejected;
        }
        try {
            auto raw = frame.to_string();
            auto decoded = decode_service(raw);
            bytes = std::move(raw);
            m = std::move(decoded);
            return ReceiveResult::received;
        } catch (const std::exception& e) {
            error = e.what();
            return ReceiveResult::rejected;
        }
    }
    return ReceiveResult::empty;
}
} // namespace aviator

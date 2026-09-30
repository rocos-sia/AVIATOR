#pragma once
#include "protocol.hpp"
#include "transport.hpp"

namespace aviator {
inline constexpr const char* core_operation_endpoint = "tcp://127.0.0.1:5559";
inline constexpr const char* service_request_topic = "record.service.request";
inline constexpr const char* service_reply_topic = "record.service.reply";
// Service envelopes are independent of the continuous-message header (§14).
// These functions throw invalid_argument on invalid input. No business execution/deduplication.
nlohmann::json decode_service(std::string_view bytes);
void validate_service(const nlohmann::json& message);
nlohmann::json service_schema(bool reply);
nlohmann::json make_service_request(const std::string& client, const std::string& session,
                                    const std::string& target, const std::string& operation,
                                    const nlohmann::json& parameters = nlohmann::json::object(),
                                    unsigned deadline_ms = 100);
nlohmann::json make_service_reply(const nlohmann::json& request, const std::string& server,
                                  const std::string& session, const std::string& status,
                                  std::uint64_t error_code = 0,
                                  const nlohmann::json& result = nlohmann::json::object());
bool matches_service_reply(const nlohmann::json& request, const nlohmann::json& reply);
bool state_operation(std::string_view operation);
// REQ/REP or DEALER single application frame; ROUTER adds routing_id itself.
bool send_service(zmq::socket_t& socket, const nlohmann::json& message);
ReceiveResult receive_service(zmq::socket_t& socket, ReceiveState& state, std::string& bytes,
                              nlohmann::json& message, std::string& error);
} // namespace aviator

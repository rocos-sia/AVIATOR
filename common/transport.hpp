#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <zmq.hpp>

namespace aviator {

inline constexpr const char* publish_endpoint = "tcp://127.0.0.1:5555";
inline constexpr const char* subscribe_endpoint = "tcp://127.0.0.1:5556";
inline constexpr const char* replay_publish_endpoint = "tcp://127.0.0.1:6555";
inline constexpr const char* replay_subscribe_endpoint = "tcp://127.0.0.1:6556";

struct SocketOptions {
    int send_hwm = 8;
    int receive_hwm = 8;
    int linger_ms = 0;
};

// Use before bind/connect. Context can be shared, sockets must not be shared
// across threads. Errors are zmq::error_t (including ETERM on context shutdown).
void configure(zmq::socket_t& socket, const SocketOptions& options = {});
void subscribe(zmq::socket_t& socket, std::string_view prefix);

struct WireMessage { std::string topic; std::string payload; };
struct ReceiveState { bool discarding = false; }; // One per socket.
enum class ReceiveResult { empty, received, rejected };

// Nonblocking, at most 32 frames per call. Invalid multipart tails are drained
// across calls without ever interpreting a tail as a new message. Only received
// changes output; exact Topic and JSON validation is performed by decode().
ReceiveResult receive(zmq::socket_t& socket, ReceiveState& state,
                      WireMessage& output, std::string& error);
// true means queued, not delivered/executed. A partial send exception requires
// closing/recreating the socket; do not continue on an uncertain multipart state.
bool send(zmq::socket_t& socket, std::string_view topic, std::string_view payload);

// Caller creates context (normally 1 IO thread), owns lifecycle, and calls
// context.shutdown() from its control thread to stop this blocking function.
// Sockets are created/used/closed entirely in the calling thread.
// on_ready runs in that thread only after both binds succeed.
void run_bus(zmq::context_t& context,
             const std::string& input = publish_endpoint,
             const std::string& output = subscribe_endpoint,
             const std::function<void()>& on_ready = {});

} // namespace aviator

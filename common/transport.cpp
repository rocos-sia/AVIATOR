#include "transport.hpp"
#include "protocol.hpp"

#include <stdexcept>

namespace aviator {
void configure(zmq::socket_t& socket, const SocketOptions& options) {
    if (options.send_hwm <= 0 || options.receive_hwm <= 0 || options.linger_ms < 0)
        throw std::invalid_argument("socket queues and linger must be bounded");
    socket.set(zmq::sockopt::sndhwm, options.send_hwm);
    socket.set(zmq::sockopt::rcvhwm, options.receive_hwm);
    socket.set(zmq::sockopt::linger, options.linger_ms);
    socket.set(zmq::sockopt::sndtimeo, 0);
    socket.set(zmq::sockopt::rcvtimeo, 0);
    socket.set(zmq::sockopt::maxmsgsize, static_cast<std::int64_t>(max_payload_bytes));
    socket.set(zmq::sockopt::conflate, 0);
}

void subscribe(zmq::socket_t& socket, std::string_view prefix) {
    socket.set(zmq::sockopt::subscribe, prefix);
}

bool send(zmq::socket_t& socket, std::string_view topic, std::string_view payload) {
    if (topic.empty() || topic.size() > max_topic_bytes || payload.empty() ||
        payload.size() > max_payload_bytes)
        throw std::invalid_argument("invalid wire message size");
    if (!socket.send(zmq::buffer(topic), zmq::send_flags::sndmore | zmq::send_flags::dontwait))
        return false;
    if (!socket.send(zmq::buffer(payload), zmq::send_flags::dontwait))
        throw std::runtime_error("partial multipart send; recreate socket");
    return true;
}

ReceiveResult receive(zmq::socket_t& socket, ReceiveState& state,
                      WireMessage& output, std::string& error) {
    error.clear();
    WireMessage candidate;
    unsigned count = 0;
    bool invalid = state.discarding;
    for (unsigned budget = 0; budget < 32; ++budget) {
        zmq::message_t frame;
        if (!socket.recv(frame, zmq::recv_flags::dontwait)) {
            if (count != 0) throw std::runtime_error("incomplete multipart; recreate socket");
            return ReceiveResult::empty;
        }
        ++count;
        if (!invalid) {
            if (count == 1 && frame.size() > 0 && frame.size() <= max_topic_bytes)
                candidate.topic = frame.to_string();
            else if (count == 2 && frame.size() > 0 && frame.size() <= max_payload_bytes)
                candidate.payload = frame.to_string();
            else invalid = true;
        }
        if (!frame.more()) {
            state.discarding = false;
            if (invalid || count != 2) {
                error = "invalid multipart frame count or size";
                return ReceiveResult::rejected;
            }
            output = std::move(candidate);
            return ReceiveResult::received;
        }
    }
    state.discarding = true;
    error = "multipart drain budget reached";
    return ReceiveResult::rejected;
}

void run_bus(zmq::context_t& context, const std::string& input, const std::string& output,
             const std::function<void()>& on_ready) {
    try {
        zmq::socket_t frontend(context, zmq::socket_type::xsub);
        zmq::socket_t backend(context, zmq::socket_type::xpub);
        configure(frontend, {64, 64, 0});
        configure(backend, {64, 64, 0});
        frontend.bind(input);
        backend.bind(output);
        if (on_ready) on_ready();
        zmq::proxy(frontend, backend);
    } catch (const zmq::error_t& error) {
        if (error.num() != ETERM) throw;
    }
}
} // namespace aviator

#pragma once
#include "protocol.hpp"
#include <deque>
#include <optional>
#include <utility>

namespace aviator {
using Json = nlohmann::json;
// Writer-thread-only, bounded correlation. Never publishes or changes control traffic.
class ArmTargetRecorder {
public:
    explicit ArmTargetRecorder(std::string session) : session_(std::move(session)) {}
    void command(std::string_view payload);
    std::optional<Json> state(std::string_view payload);
    Json summary() const;
private:
    struct Window { Header header; std::string epoch, payload; bool conflict = false; };
    std::string session_;
    uint64_t sequence_ = 0, commands_ = 0, malformed_ = 0, evicted_ = 0, matched_ = 0, missing_ = 0;
    std::deque<Window> windows_; // <=64 original messages (<=4 MiB payload)
    std::deque<std::pair<std::string, uint64_t>> states_; // <=64 source session sequence watermarks
};
} // namespace aviator

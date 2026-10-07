#pragma once
#include "motion.hpp"
#include <optional>

namespace aviator {
// Pure command/feedback state, protected by HandLink's dedicated hand mutex.
class HandControl {
public:
    void configure(const std::filesystem::path&);
    bool enabled() const { return enabled_; }
    void request(bool close, uint64_t now);
    void beginApproach(uint64_t now);
    void approachProgress(const std::array<double, 2>&, uint64_t now);
    void revoke();
    void fail(const std::string& reason);
    void receive(const Message&, uint64_t now, const std::string& core_session);
    std::optional<Message> command(uint64_t now, uint64_t heartbeat, const std::string& session);
    bool complete(uint64_t now) const;
    bool fresh(uint64_t now) const;
    bool waitExpired(uint64_t now) const;
    std::string fault(uint64_t now) const;
    void startMonitoring(uint64_t now) { monitor_started_ = now; }
    std::string messageFault(uint64_t now) const;
    const Json& body() const { return body_; }
    uint64_t sample() const { return sample_; }
    uint64_t sequence() const { return state_sequence_; }
private:
    bool feedbackFresh(uint64_t now) const;
    using Pose = std::array<std::array<double, 6>, 2>; // left, right
    bool enabled_ = false, has_close_ = false, active_ = false, closing_ = false;
    bool valid_ = false, acknowledged_ = false, synchronized_ = false, endpoint_ = false;
    bool feedback_timely_ = false;
    bool endpoint_acknowledged_ = false;
    // Normalized targets: thumb rotation 0.5 (register 500), five bends fully open.
    Pose open_{{{.5,1,1,1,1,1}, {.5,1,1,1,1,1}}}, close_{}, target_{};
    std::string publisher_ = "inspire_hand", node_session_, epoch_ = new_session_id(), error_;
    uint64_t timeout_ = 5000000, feedback_timeout_ = 500000;
    uint64_t requested_ = 0, first_sequence_ = 0, command_sequence_ = 0, next_ = 0;
    uint64_t completion_started_ = 0;
    uint64_t received_ = 0, sample_ = 0, state_sequence_ = 0, accepted_at_ = 0;
    uint64_t monitor_started_ = 0;
    double tolerance_ = .03;
    Json body_ = Json::object();
};
} // namespace aviator

#pragma once
#include "StateMachineRuntime.hpp"
#include "aviator/Aviator.hpp"
#include "service.hpp"
#include <map>

namespace aviator {
// Owner-thread adapter only: all motion and state decisions remain in Aviator.
class ManagedGateway {
public:
    ManagedGateway(const MotionConfig&, const ManagedGatewayOptions&);
    void receiveInput();
    void receiveRequests(Aviator&, const std::string& core_session);
    void drive(Aviator&);
    bool bound() const { return input_.has_value(); }
    bool fresh() const { return input_ && !input_->expired(monotonic_us()); }
private:
    Json request(Aviator&, const std::string& core_session, const Json&);
    struct Cached { std::string body; Json reply; };
    zmq::context_t context_{1};
    zmq::socket_t sub_{context_, zmq::socket_type::sub};
    zmq::socket_t router_{context_, zmq::socket_type::router};
    InputPolicy policy_;
    std::optional<InputGuard> input_;
    ReceiveState receiving_;
    uint64_t effective_sample_ = 0, forwarded_ = 0, next_servo_ = 0;
    double roll_ = 0, pitch_ = 0;
    bool was_fresh_ = false;
    std::map<std::string, Cached> cache_;
    std::string route_, payload_;
    unsigned frames_ = 0;
};
} // namespace aviator

#include "PhotoelectricEmergency.hpp"
#include <chrono>

namespace aviator {
PhotoelectricEmergency::PhotoelectricEmergency(const MotionConfig& c)
    : config_(c.photoelectric), endpoint_(c.subscribe), clock_(local_clock_id()) {
    if (config_.enabled) thread_ = std::thread([this] { run(); });
    else state_.healthy = true;
}
PhotoelectricEmergency::~PhotoelectricEmergency() {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
}
void PhotoelectricEmergency::latch(const std::string& reason) {
    if (!state_.latched) state_.reason = reason;
    state_.latched = true;
    state_.healthy = false;
}
PhotoelectricEmergency::State PhotoelectricEmergency::snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = monotonic_us();
    if (armed_ && (now < state_.sample || now - state_.sample >= config_.timeout_us ||
                   now < received_ || now - received_ >= config_.timeout_us))
        latch("Photoelectric X1 sample expired");
    return state_;
}
void PhotoelectricEmergency::run() {
    try {
        zmq::context_t context(1);
        zmq::socket_t sub(context, zmq::socket_type::sub);
        configure(sub, {8, 256, 0});
        subscribe(sub, "photoelectric.state");
        sub.connect(endpoint_);
        ReceiveState receive_state;
        std::string session;
        uint64_t sequence = 0, sample = 0;
        while (!quit_) {
            // Check expiry before processing new evidence; a late 1 cannot erase a timeout.
            snapshot();
            for (unsigned i = 0; i < 128 && !quit_; ++i) {
                WireMessage wire;
                Message message;
                std::string error;
                const auto result = receive(sub, receive_state, wire, error);
                if (result == ReceiveResult::empty) break;
                if (result != ReceiveResult::received || wire.topic != "photoelectric.state" ||
                    !decode(wire.topic, wire.payload, message, error)) continue;
                const auto& h = message.header;
                if (h.publisher_id != config_.publisher || h.clock_id != clock_ ||
                    !message.body.contains("channel") || message.body["channel"] != config_.channel) continue;
                const auto now = monotonic_us();
                if ((session == h.session_id && h.sequence <= sequence) ||
                    h.sample_mono_us > now || (h.valid && h.sample_mono_us < sample)) continue;
                // Invalid feedback retains the old sample timestamp, including startup zero.
                if (session != h.session_id && sequence && h.valid && h.sample_mono_us <= sample) continue;
                session = h.session_id;
                sequence = h.sequence;
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_.latched) continue;
                const auto value = message.body.find("value");
                const bool shaped = value != message.body.end() && value->is_number_integer() &&
                                    (*value == 0 || *value == 1);
                const bool fresh = h.sample_mono_us && now - h.sample_mono_us < config_.timeout_us;
                if (!h.valid || !shaped || !fresh) {
                    if (armed_) latch("Photoelectric X1 invalid or stale");
                    continue;
                }
                sample = h.sample_mono_us;
                state_.sample = sample;
                received_ = now;
                if (*value == 0) latch("Photoelectric X1=0");
                else { armed_ = true; state_.healthy = true; }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        latch(std::string("Photoelectric receiver failed: ") + e.what());
    }
}
}

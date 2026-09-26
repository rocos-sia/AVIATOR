#include "runtime.hpp"

#include <ctime>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace aviator {
namespace {
std::uint64_t clock_us(clockid_t id) {
    timespec time{};
    if (clock_gettime(id, &time) != 0) throw std::runtime_error("clock_gettime failed");
    return static_cast<std::uint64_t>(time.tv_sec) * 1000000 + time.tv_nsec / 1000;
}
std::string read_id(const char* path) {
    std::ifstream file(path);
    std::string value;
    if (!(file >> value) || value.empty()) throw std::runtime_error("cannot read Linux identifier");
    return value;
}
bool fresh(std::uint64_t sample, std::uint64_t now,
           std::uint64_t timeout, std::uint64_t future) {
    return sample > now ? sample - now <= future : now - sample < timeout;
}
bool motion(Topic topic) { return topic == Topic::arm_command || topic == Topic::hand_command; }
} // namespace

std::uint64_t utc_us() { return clock_us(CLOCK_REALTIME); }
std::uint64_t monotonic_us() { return clock_us(CLOCK_MONOTONIC); }
std::string new_session_id() { return read_id("/proc/sys/kernel/random/uuid"); }
std::string local_clock_id() {
    char hostname[256]{};
    if (gethostname(hostname, sizeof(hostname) - 1) != 0)
        throw std::runtime_error("gethostname failed");
    return std::string(hostname).substr(0, 80) + "-" + read_id("/proc/sys/kernel/random/boot_id");
}

InputGuard::InputGuard(InputPolicy policy) : policy_(std::move(policy)) {
    if (policy_.publisher_id.empty() || policy_.session_id.empty() ||
        policy_.clock_id.empty() || policy_.timeout_us == 0 ||
        (policy_.topic == Topic::flight_command &&
         policy_.source != "FLIGHT" && policy_.source != "JOYSTICK") ||
        (motion(policy_.topic) && (policy_.control_epoch.empty() ||
         policy_.origin_publisher_id.empty() || policy_.origin_session_id.empty() ||
         policy_.origin_timeout_us == 0)))
        throw std::invalid_argument("incomplete input authorization policy");
}

bool InputGuard::accept(const Message& message, std::uint64_t now, std::string& error) {
    const auto reject = [&](const char* reason) { error = reason; return false; };
    const auto& h = message.header;
    if (message.topic != policy_.topic || h.publisher_id != policy_.publisher_id ||
        h.session_id != policy_.session_id) return reject("unauthorized topic/publisher/session");
    if (h.clock_id != policy_.clock_id) return reject("clock domain mismatch");
    if (h.sequence == 0 || h.sequence > max_json_integer || h.sequence <= sequence_)
        return reject("duplicate or out-of-order sequence");
    if (!fresh(h.sample_mono_us, now, policy_.timeout_us, policy_.future_tolerance_us))
        return reject("stale or future sample");
    if (message.topic == Topic::flight_command &&
        (!message.body.contains("source") || message.body.at("source") != policy_.source))
        return reject("unauthorized source");
    Origin origin;
    const bool has_origin = motion(message.topic);
    if (has_origin) {
        if (!message.body.contains("control_epoch") ||
            message.body.at("control_epoch") != policy_.control_epoch)
            return reject("unauthorized control epoch");
        if (!read_origin(message.body, origin, error)) return false;
        if (origin.publisher_id != policy_.origin_publisher_id ||
            origin.session_id != policy_.origin_session_id || origin.clock_id != policy_.clock_id)
            return reject("unauthorized origin");
        if (!fresh(origin.sample_mono_us, now, policy_.origin_timeout_us, policy_.future_tolerance_us))
            return reject("stale or future origin");
    }
    sequence_ = h.sequence;
    if (!h.valid) {
        valid_ = false; // Keep last numeric snapshot but immediately revoke usability.
        return reject("invalid business data");
    }
    sample_ = h.sample_mono_us;
    origin_sample_ = origin.sample_mono_us;
    received_ = now;
    has_origin_ = has_origin;
    valid_ = true;
    error.clear();
    return true;
}

bool InputGuard::expired(std::uint64_t now) const {
    return !valid_ || now < received_ || now - received_ >= policy_.timeout_us ||
        !fresh(sample_, now, policy_.timeout_us, policy_.future_tolerance_us) ||
        (has_origin_ && !fresh(origin_sample_, now, policy_.origin_timeout_us, policy_.future_tolerance_us));
}
} // namespace aviator

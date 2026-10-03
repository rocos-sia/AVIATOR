#include "HandLink.hpp"

namespace aviator {
HandLink::~HandLink() { stop(); }
void HandLink::configure(const std::filesystem::path& path) {
    control_.configure(path);
}
void HandLink::start(zmq::context_t& context, const MotionConfig& config, const std::string& session,
                     const std::atomic<uint64_t>& heartbeat, bool allowed) {
    if (!enabled()) return;
    // Configuration and ownership are established before exposing the worker.
    allowed_ = allowed;
    stopping_ = false;
    thread_ = std::thread([this, &context, config, session, &heartbeat] {
        io(context, config, session, heartbeat);
    });
}
void HandLink::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        control_.revoke();
        changed_.notify_all();
    }
    if (thread_.joinable()) thread_.join();
}
void HandLink::allow(bool allowed) {
    std::lock_guard<std::mutex> lock(mutex_);
    allowed_ = allowed;
    if (!allowed) { control_.revoke(); ++version_; }
    changed_.notify_all();
}
uint64_t HandLink::request(bool close) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || failed_) throw std::runtime_error("Hand IO worker unavailable: " + control_.fault(monotonic_us()));
    if (!allowed_) throw std::runtime_error("Core hand authorization revoked");
    control_.request(close, monotonic_us());
    changed_.notify_all();
    return ++version_;
}
uint64_t HandLink::beginApproach() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || failed_) throw std::runtime_error("Hand IO worker unavailable");
    if (!allowed_) throw std::runtime_error("Core hand authorization revoked");
    const auto now = monotonic_us();
    if (const auto reason = control_.fault(now); !reason.empty()) throw std::runtime_error(reason);
    control_.beginApproach(now);
    changed_.notify_all();
    return ++version_;
}
void HandLink::approachProgress(uint64_t version, const std::array<double, 2>& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || failed_) throw std::runtime_error("Hand IO worker unavailable");
    if (!allowed_) throw std::runtime_error("Core hand authorization revoked");
    if (version != version_) throw std::runtime_error("Hand target superseded");
    const auto now = monotonic_us();
    if (const auto reason = control_.fault(now); !reason.empty()) throw std::runtime_error(reason);
    control_.approachProgress(progress, now);
    changed_.notify_all();
}
void HandLink::wait(uint64_t version, const std::atomic<bool>* cancel) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (cancel && cancel->load()) throw std::runtime_error("Motion stopped during hand completion");
        if (stopping_) throw std::runtime_error("Hand IO worker stopped");
        if (!allowed_) throw std::runtime_error("Core hand authorization revoked");
        if (version != version_) throw std::runtime_error("Hand target superseded");
        const auto now = monotonic_us(); // Sample only AFTER acquiring the target/feedback lock.
        const auto reason = control_.fault(now);
        if (!reason.empty()) { control_.fail(reason); throw std::runtime_error(reason); }
        if (control_.complete(now)) return;
        changed_.wait_for(lock, std::chrono::milliseconds(5));
    }
}
void HandLink::fail(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    control_.fail(reason);
    changed_.notify_all();
}
std::string HandLink::fault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return control_.fault(monotonic_us());
}
HandLink::Status HandLink::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = monotonic_us();
    return {control_.body(), control_.fresh(now),
            control_.sample() && now >= control_.sample() ? Json(double(now - control_.sample()) / 1000) : Json(nullptr),
            control_.sequence(), version_, control_.fault(now)};
}
void HandLink::io(zmq::context_t& context, const MotionConfig& config, const std::string& session,
                  const std::atomic<uint64_t>& heartbeat) {
    try {
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        aviator::configure(pub);
        aviator::configure(sub);
        subscribe(sub, "hand.state");
        pub.connect(config.publish);
        sub.connect(config.subscribe);
        ReceiveState receiver;
        while (true) {
            // Bound receive work so a feedback backlog cannot starve command publication.
            for (int n = 0; n < 16; ++n) {
                WireMessage wire;
                std::string error;
                const auto received = receive(sub, receiver, wire, error);
                if (received == ReceiveResult::empty) break;
                Message m;
                if (received != ReceiveResult::received || !decode(wire.topic, wire.payload, m, error)) continue;
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_) return;
                control_.receive(m, monotonic_us(), session);
                changed_.notify_all();
            }
            std::unique_lock<std::mutex> lock(mutex_);
            if (stopping_) return;
            // Load the heartbeat FIRST, then time. A concurrent newer heartbeat must not
            // be compared against an older timestamp captured before waiting on a lock.
            const auto origin = heartbeat.load();
            const auto now = monotonic_us();
            if (allowed_ && !failed_) {
                if (auto command = control_.command(now, origin, session))
                    publishMessage(pub, *command);
            }
            // Publication is serialized with target replacement/revocation. On return
            // from allow(false), no old snapshot can be published by this worker.
            changed_.notify_all();
            changed_.wait_for(lock, std::chrono::milliseconds(1));
        }
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true; // Unlike a command failure, the worker cannot accept another target.
        control_.fail(std::string("Hand IO failed: ") + e.what());
        changed_.notify_all();
    }
}
} // namespace aviator

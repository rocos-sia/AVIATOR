#include "HandLink.hpp"
#include "Logger.hpp"

namespace aviator {
HandLink::~HandLink() { stop(); }
void HandLink::warnLocked(const std::string& reason) {
    if (reason.empty()) return;
    const auto now = monotonic_us();
    if (reason != last_warning_ || now - warning_at_ >= 10000000) {
        Logger::warn("Core hand warning: {}; hand action unavailable, Core continues without hand confirmation", reason);
        last_warning_ = reason;
        warning_at_ = now;
    }
}
void HandLink::configure(const std::filesystem::path& path) {
    try { control_.configure(path); }
    catch (const std::exception& e) {
        control_ = HandControl{}; // Disable only the unusable hand connection.
        control_.fail(std::string("Invalid hand configuration: ") + e.what());
        warnLocked(control_.fault(monotonic_us()));
    }
}
void HandLink::start(zmq::context_t& context, const MotionConfig& config, const std::string& session,
                     const std::atomic<uint64_t>& heartbeat, bool allowed) {
    if (!enabled()) return;
    // Configuration and ownership are established before exposing the worker.
    allowed_ = allowed;
    stopping_ = false;
    control_.startMonitoring(monotonic_us());
    try {
        thread_ = std::thread([this, &context, config, session, &heartbeat] {
            io(context, config, session, heartbeat);
        });
    } catch (const std::exception& e) {
        failed_ = true;
        control_.fail(std::string("Hand IO startup failed: ") + e.what());
        warnLocked(control_.fault(monotonic_us()));
    }
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
    if (!allowed_) throw MotionCancelled();
    if (stopping_ || failed_) { warnLocked("Hand IO worker unavailable: " + control_.fault(monotonic_us())); return 0; }
    try { control_.request(close, monotonic_us()); }
    catch (const std::exception& e) { control_.fail(e.what()); warnLocked(e.what()); return 0; }
    changed_.notify_all();
    return ++version_;
}
uint64_t HandLink::beginApproach() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allowed_) throw MotionCancelled();
    if (stopping_ || failed_) { warnLocked("Hand IO worker unavailable"); return 0; }
    const auto now = monotonic_us();
    if (const auto reason = control_.fault(now); !reason.empty()) { warnLocked(reason); return 0; }
    try { control_.beginApproach(now); }
    catch (const std::exception& e) { control_.fail(e.what()); warnLocked(e.what()); return 0; }
    changed_.notify_all();
    return ++version_;
}
void HandLink::approachProgress(uint64_t version, const std::array<double, 2>& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allowed_) throw MotionCancelled();
    if (!version) return;
    if (stopping_ || failed_) { warnLocked("Hand IO worker unavailable"); return; }
    if (version != version_) { warnLocked("Hand target superseded"); return; }
    const auto now = monotonic_us();
    if (const auto reason = control_.fault(now); !reason.empty()) { warnLocked(reason); return; }
    try { control_.approachProgress(progress, now); }
    catch (const std::exception& e) { control_.fail(e.what()); warnLocked(e.what()); return; }
    changed_.notify_all();
}
bool HandLink::wait(uint64_t version, const std::atomic<bool>* cancel) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if ((cancel && cancel->load()) || !allowed_) throw MotionCancelled();
        if (!version) return false;
        if (stopping_) { warnLocked("Hand IO worker stopped"); return false; }
        if (version != version_) { warnLocked("Hand target superseded"); return false; }
        const auto now = monotonic_us(); // Sample only AFTER acquiring the target/feedback lock.
        const auto reason = control_.fault(now);
        if (!reason.empty()) { control_.fail(reason); warnLocked(reason); return false; }
        if (control_.complete(now)) return true;
        changed_.wait_for(lock, std::chrono::milliseconds(5));
    }
}
void HandLink::fail(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    control_.fail(reason);
    warnLocked(reason);
    changed_.notify_all();
}
std::string HandLink::fault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return control_.fault(monotonic_us());
}
std::string HandLink::messageFault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return control_.messageFault(monotonic_us());
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
                warnLocked(control_.fault(now));
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
        warnLocked(control_.fault(monotonic_us()));
        changed_.notify_all();
    }
}
} // namespace aviator

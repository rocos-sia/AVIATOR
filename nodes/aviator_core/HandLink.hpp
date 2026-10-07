#pragma once
#include "HandControl.hpp"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace aviator {
// Independent Core hand connection. No arm mutex, planning work or arm sockets here.
// Callers update a whole dual-hand target; this worker owns both ZMQ sockets.
class HandLink {
public:
    struct Status {
        Json body;
        bool fresh;
        Json age_ms;
        uint64_t sequence, target_version;
        std::string error;
    };
    ~HandLink();
    void configure(const std::filesystem::path& path);
    bool enabled() const { return control_.enabled(); } // Immutable after configure/before start.
    void start(zmq::context_t&, const MotionConfig&, const std::string& session,
               const std::atomic<uint64_t>& heartbeat, bool allowed);
    void stop();
    void allow(bool);
    uint64_t request(bool close);
    uint64_t beginApproach();
    void approachProgress(uint64_t version, const std::array<double, 2>&);
    bool wait(uint64_t version, const std::atomic<bool>* cancel = nullptr);
    void fail(const std::string&);
    std::string fault() const;
    Status status() const;
private:
    void warnLocked(const std::string& reason); // Hand failures are advisory to Core; mutex_ held.
    void io(zmq::context_t&, const MotionConfig&, const std::string& session,
            const std::atomic<uint64_t>& heartbeat);
    HandControl control_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    bool allowed_ = false, stopping_ = true, failed_ = false;
    uint64_t version_ = 0;
    uint64_t warning_at_ = 0;
    std::string last_warning_;
    std::thread thread_;
};
} // namespace aviator

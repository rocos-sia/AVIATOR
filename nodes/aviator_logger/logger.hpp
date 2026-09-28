#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <string>

namespace aviator {

struct TopicStats {
    std::string type;          // message type name (e.g. "ArmState")
    std::uint64_t messages = 0;
};

struct RecorderSummary {
    std::uint64_t messages = 0;  // valid bus messages written
    std::uint64_t invalid = 0;   // decode failures, counted and skipped
    std::uint64_t rejected = 0;  // transport-level rejections (bad multipart)
    std::uint64_t channels = 0;  // distinct topics recorded
    std::string path;            // final (renamed) MCAP path
    std::map<std::string, TopicStats> topics;  // topic -> {type, count}
    std::uint64_t start_log_ns = 0;      // first receive (UTC ns)
    std::uint64_t end_log_ns = 0;        // last receive (UTC ns)
    std::uint64_t start_publish_ns = 0;  // first source timestamp (UTC ns)
    std::uint64_t end_publish_ns = 0;    // last source timestamp (UTC ns)
};

// Records bus traffic to `output_path` (written as `.partial`, atomically
// renamed on success) until `stop` is set. Runs on the calling thread and owns
// the SUB socket and MCAP Writer; the caller joins the thread afterwards.
// Throws on open/write failure so the node can exit non-zero rather than
// silently lose data.
RecorderSummary record_bus(const std::string& subscribe_endpoint,
                           const std::string& output_path,
                           const std::string& session_id,
                           const std::atomic<bool>& stop);

} // namespace aviator

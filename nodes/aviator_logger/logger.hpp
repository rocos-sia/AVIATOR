#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace aviator {

struct RecorderSummary {
    std::uint64_t messages = 0;  // valid bus messages written
    std::uint64_t invalid = 0;   // decode failures, counted and skipped
    std::uint64_t rejected = 0;  // transport-level rejections (bad multipart)
    std::uint64_t channels = 0;  // distinct topics recorded
    std::string path;            // final (renamed) MCAP path
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

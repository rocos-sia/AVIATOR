#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>

namespace aviator {
struct TopicStats {
    std::string type;
    std::uint64_t messages = 0;
};

struct RecorderSummary {
    std::uint64_t messages = 0;
    std::uint64_t invalid = 0;
    std::uint64_t rejected = 0;
    std::uint64_t dropped = 0;
    std::uint64_t channels = 0;
    std::uint64_t sequence_gaps = 0;
    std::uint64_t duplicate_or_reordered = 0;
    std::string path;
    std::map<std::string, TopicStats> topics;
    std::uint64_t start_log_ns = 0;
    std::uint64_t end_log_ns = 0;
};

// Non-real-time, single-thread-owned storage. Original JSON bytes are retained.
// Abandoning a writer preserves .partial; only finish() publishes a final file.
class RecordingWriter {
  public:
    RecordingWriter(const std::string& path, const std::string& session);
    ~RecordingWriter();
    RecordingWriter(const RecordingWriter&) = delete;
    RecordingWriter& operator=(const RecordingWriter&) = delete;
    void append(std::string_view topic, std::string_view payload, std::uint64_t receive_utc_ns);
    RecorderSummary finish(std::uint64_t rejected = 0, std::uint64_t dropped = 0);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator

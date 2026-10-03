#pragma once
#include "config.hpp"
#include <deque>
#include <map>
#include <mutex>
#include <string>
namespace monitor {
// Separate from the control-bus cache and the Logger's PUSH/PULL path.
class Preview {
  public:
    explicit Preview(Json settings);
    void configure(Json settings);
    void ingest(const std::string& topic, const std::string& metadata, std::string jpeg,
                std::uint64_t now);
    Json latest(std::uint64_t now, const std::string& clock);
    std::string frame(const std::string& token);
    void reject(const std::string& reason);

  private:
    struct Frame {
        std::string token, jpeg;
        Json meta;
        std::uint64_t received;
    };
    struct Origin {
        std::uint64_t sequence = 0, received = 0;
    };
    Json settings_;
    std::mutex mutex_;
    std::deque<Frame> frames_;
    std::map<std::string, Origin> origins_;
    std::size_t bytes_ = 0;
    std::uint64_t counter_ = 0, rejected_ = 0;
    std::string session_, error_;
};
} // namespace monitor

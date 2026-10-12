#pragma once
#include "protocol.hpp"
#include <cstdint>
#include <deque>
#include <filesystem>
#include <map>
#include <optional>
#include <string>

namespace aviator::tts {
struct Cue {
    std::string text;
    int priority = 0;
};
struct Config {
    std::string subscribe = "tcp://127.0.0.1:5556", publisher = "aviator_core", device = "default";
    std::filesystem::path audio_dir;
    uint64_t freshness_us = 200000, lost_after_us = 2000000, ttl_us = 5000000;
    uint64_t startup_grace_us = 15000000;
    bool announce_initial = false;
    std::map<std::string, Cue> cues;
};
std::string default_config_path();
Config load_config(const std::string& path);
// Throws on missing, stale or malformed audio. No synthesis/download at runtime.
std::map<std::string, std::filesystem::path> load_audio(const Config& config);
struct Reminder {
    std::string id;
    int priority;
    uint64_t revision, expires;
};
// Single owner: the same non-real-time loop receives, evaluates and schedules.
class Rules {
  public:
    Rules(Config config, std::string clock, uint64_t now);
    bool ingest(const Message& message, uint64_t now, std::string& error);
    void tick(uint64_t now);
    std::optional<Reminder> next(uint64_t now);
    bool relevant(const Reminder& reminder, uint64_t now) const;
    bool current(const Reminder& reminder) const { return reminder.revision == revision_; }
    int pending_priority(uint64_t now) const;

  private:
    struct Stream {
        uint64_t sequence = 0, sample = 0, received = 0;
    };
    void change();
    void enqueue(const std::string& id, uint64_t now);
    Config config_;
    std::string clock_, instance_, state_, error_;
    std::map<std::string, Stream> streams_;
    std::deque<Reminder> queue_;
    uint64_t last_received_ = 0, last_sample_ = 0, unavailable_since_, revision_ = 0;
    bool available_ = false, lost_ = false, conflict_ = false, ever_valid_ = false;
};
} // namespace aviator::tts

#include "tts.hpp"
#include <algorithm>
#include <fstream>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace aviator::tts {
namespace {
const std::map<std::string, int> states{
    {"INIT", 0},      {"INITIALIZING", 10},  {"STANDBY", 1},   {"GRASPING", 2},
    {"FOLLOWING", 3}, {"CONTROL", 4},        {"RELEASING", 8},
    {"ERROR", 6},     {"EMERGENCY_STOP", 7}, {"READY", 11},    {"HOMING", 12}};
void require(bool ok, const std::string& reason) {
    if (!ok)
        throw std::runtime_error(reason);
}
void keys(const YAML::Node& node, const std::set<std::string>& expected) {
    require(node.IsMap(), "expected YAML mapping");
    std::set<std::string> seen;
    for (const auto& field : node) {
        const auto name = field.first.as<std::string>();
        require(expected.count(name) && seen.insert(name).second, "unknown/duplicate key: " + name);
    }
    require(seen == expected, "missing YAML key");
}
uint64_t milliseconds(const YAML::Node& node) {
    const auto s = node.as<std::string>();
    require(!s.empty() && s.find_first_not_of("0123456789") == std::string::npos,
            "expected positive milliseconds");
    const auto n = std::stoull(s);
    require(n > 0 && n <= 60000, "milliseconds outside 1..60000");
    return n * 1000;
}
// Accept the generator's PCM16 WAV and other recordings with the same format.
void check_wave(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    require(bool(f), "cannot read WAV: " + path.string());
    auto u32 = [&]() {
        unsigned char b[4]{};
        f.read(reinterpret_cast<char*>(b), 4);
        require(bool(f), "truncated WAV");
        return uint32_t(b[0]) | uint32_t(b[1]) << 8 | uint32_t(b[2]) << 16 | uint32_t(b[3]) << 24;
    };
    auto tag = [&]() {
        char b[4]{};
        f.read(b, 4);
        require(bool(f), "truncated WAV");
        return std::string(b, 4);
    };
    require(tag() == "RIFF", "expected RIFF WAV");
    const auto size = u32();
    require(size >= 36 && size <= 6000000 && std::filesystem::file_size(path) == uint64_t(size) + 8,
            "invalid WAV size");
    require(tag() == "WAVE", "expected WAVE");
    bool fmt = false, data = false;
    uint32_t byte_rate = 0, data_size = 0, alignment = 0;
    while (uint64_t(f.tellg()) + 8 <= uint64_t(size) + 8) {
        const auto id = tag();
        const auto n = u32();
        const auto pos = uint64_t(f.tellg());
        require(n <= size && pos + n + (n & 1) <= uint64_t(size) + 8, "WAV chunk outside file");
        if (id == "fmt ") {
            require(!fmt && n >= 16, "invalid WAV fmt");
            const auto format_channels = u32();
            const auto rate = u32();
            byte_rate = u32();
            const auto align_bits = u32();
            const auto channels = format_channels >> 16;
            alignment = align_bits & 65535;
            require((format_channels & 65535) == 1 && (channels == 1 || channels == 2) &&
                        rate >= 8000 && rate <= 48000 && (align_bits >> 16) == 16 &&
                        alignment == channels * 2 && byte_rate == rate * alignment,
                    "expected PCM16 mono/stereo WAV");
            fmt = true;
        } else if (id == "data") {
            require(!data && n > 0, "empty/duplicate WAV data");
            data = true;
            data_size = n;
        }
        f.seekg(static_cast<std::streamoff>(pos + n + (n & 1)));
    }
    require(fmt && data && data_size % alignment == 0 && data_size <= uint64_t(byte_rate) * 30,
            "missing/oversized WAV audio (maximum 30 seconds)");
}
} // namespace
std::string default_config_path() {
    const auto installed = std::filesystem::read_symlink("/proc/self/exe").parent_path() /
                           "../share/aviator/config/tts.yaml";
    return std::filesystem::exists(installed) ? installed.lexically_normal().string()
                                              : AVIATOR_TTS_CONFIG;
}
Config load_config(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    keys(root, {"subscribe", "publisher", "device", "audio_dir", "freshness_ms", "lost_after_ms",
                "startup_grace_ms", "queue_ttl_ms", "announce_initial", "cues"});
    Config c;
    c.subscribe = root["subscribe"].as<std::string>();
    c.publisher = root["publisher"].as<std::string>();
    c.device = root["device"].as<std::string>();
    require(c.subscribe.rfind("tcp://", 0) == 0 && !c.publisher.empty() && !c.device.empty(),
            "invalid endpoint/publisher/device");
    for (const auto* value : {&c.subscribe, &c.publisher, &c.device})
        require(value->size() <= 256 && value->find('\0') == std::string::npos, "invalid string");
    const auto audio = root["audio_dir"].as<std::string>();
    require(!audio.empty(), "empty audio_dir");
    c.audio_dir = (std::filesystem::absolute(path).parent_path() / audio).lexically_normal();
    c.freshness_us = milliseconds(root["freshness_ms"]);
    c.lost_after_us = milliseconds(root["lost_after_ms"]);
    c.startup_grace_us = milliseconds(root["startup_grace_ms"]);
    c.ttl_us = milliseconds(root["queue_ttl_ms"]);
    require(c.lost_after_us >= c.freshness_us, "lost_after_ms must be >= freshness_ms");
    const auto initial = root["announce_initial"].as<std::string>();
    require(initial == "true" || initial == "false", "announce_initial must be true/false");
    c.announce_initial = initial == "true";
    std::set<std::string> names{"FOLLOWING_FROM_CONTROL", "COMM_LOST", "COMM_RESTORED",
                                "SOURCE_CONFLICT"};
    for (const auto& item : states)
        names.insert(item.first);
    keys(root["cues"], names);
    for (const auto& name : names) {
        const auto n = root["cues"][name];
        keys(n, {"text", "priority"});
        Cue cue{n["text"].as<std::string>(), n["priority"].as<int>()};
        require(!cue.text.empty() && cue.text.size() <= 256 &&
                    cue.text.find('\0') == std::string::npos && cue.priority >= 0 &&
                    cue.priority <= 3,
                "invalid cue: " + name);
        c.cues.emplace(name, cue);
    }
    return c;
}
std::map<std::string, std::filesystem::path> load_audio(const Config& c) {
    const auto manifest = c.audio_dir / "manifest.json";
    require(std::filesystem::exists(manifest) && std::filesystem::file_size(manifest) < 131072,
            "missing/oversized audio manifest; run tools/tts/generate.py first: " +
                manifest.string());
    std::ifstream f(manifest);
    nlohmann::json m;
    f >> m;
    require(m.at("schema_version") == 1 && m.at("cues").is_object(), "invalid audio manifest");
    std::map<std::string, std::filesystem::path> result;
    for (const auto& [id, cue] : c.cues) {
        const auto& entry = m.at("cues").at(id);
        require(entry.at("text") == cue.text, "stale audio text for " + id + "; regenerate audio");
        const auto name = entry.at("file").get<std::string>();
        require(name.size() > 4 && name.size() <= 128 &&
                    name.find_first_not_of(
                        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-. ") ==
                        std::string::npos &&
                    std::filesystem::path(name).extension() == ".wav" && name != ".wav",
                "invalid audio filename");
        const auto path = c.audio_dir / name;
        check_wave(path);
        result.emplace(id, path);
    }
    return result;
}
Rules::Rules(Config c, std::string clock, uint64_t now)
    : config_(std::move(c)), clock_(std::move(clock)), unavailable_since_(now) {}
void Rules::change() {
    ++revision_;
    queue_.clear();
}
void Rules::enqueue(const std::string& id, uint64_t now) {
    if (queue_.size() == 4)
        queue_.pop_front();
    queue_.push_back({id, config_.cues.at(id).priority, revision_, now + config_.ttl_us});
}
bool Rules::ingest(const Message& m, uint64_t now, std::string& reason) {
    const auto reject = [&](const char* s) {
        reason = s;
        return false;
    };
    const auto& h = m.header;
    if (m.topic != Topic::flight_state || h.publisher_id != config_.publisher)
        return reject("wrong source/topic");
    if (h.clock_id != clock_ || !h.sample_mono_us || h.sample_mono_us > now ||
        now - h.sample_mono_us >= config_.freshness_us)
        return reject("stale/future/cross-clock sample");
    std::string state, error;
    try {
        const auto& s = m.body.at("system");
        state = s.at("state").get<std::string>();
        // Managed Core includes state_code + generation; Direct Core does not.
        if (!states.count(state) || !s.at("state_code").is_number_integer() ||
            s.at("state_code") != states.at(state) || !s.at("generation").is_number_unsigned())
            return reject("not a supported Managed state");
        error = s.at("current_error").get<std::string>();
        if (error.size() > 4096)
            return reject("oversized current_error");
    } catch (const nlohmann::json::exception&) {
        return reject("missing Managed fields");
    }
    for (auto it = streams_.begin(); it != streams_.end();) {
        if (now - it->second.received >= config_.freshness_us && it->first != instance_)
            it = streams_.erase(it);
        else
            ++it;
    }
    auto it = streams_.find(h.session_id);
    if (!h.sequence || (it != streams_.end() && (h.sequence <= it->second.sequence ||
                                                 h.sample_mono_us < it->second.sample)))
        return reject("duplicate/regressing message");
    if (it == streams_.end() && last_sample_ && h.sample_mono_us <= last_sample_)
        return reject("old instance");
    if (it == streams_.end() && streams_.size() >= 8)
        return reject("too many instances");
    streams_[h.session_id] = {h.sequence, h.sample_mono_us, now};
    size_t live = 0;
    for (const auto& [id, stream] : streams_)
        if (now - stream.received < config_.freshness_us)
            ++live;
    if (live > 1) {
        if (!conflict_) {
            change();
            enqueue("SOURCE_CONFLICT", now);
        }
        conflict_ = true;
        if (available_)
            unavailable_since_ = now;
        available_ = false;
        return reject("multiple active Core instances");
    }
    if (!h.valid) {
        if (available_) {
            change();
            unavailable_since_ = now;
        }
        available_ = false;
        return reject("invalid status");
    }
    const bool baseline = !available_ || instance_ != h.session_id;
    const bool recovery = lost_ || conflict_;
    const bool changed = state != state_ || error != error_;
    if (baseline || changed) {
        change();
        if (recovery)
            enqueue("COMM_RESTORED", now);
        const auto id = !baseline && state == "FOLLOWING" && state_ == "CONTROL"
                            ? "FOLLOWING_FROM_CONTROL"
                            : state;
        if (!baseline || config_.announce_initial || config_.cues.at(id).priority >= 2 ||
            (recovery && ever_valid_))
            enqueue(id, now);
    }
    state_ = state;
    error_ = error;
    instance_ = h.session_id;
    last_received_ = now;
    last_sample_ = h.sample_mono_us;
    available_ = ever_valid_ = true;
    conflict_ = lost_ = false;
    reason.clear();
    return true;
}
void Rules::tick(uint64_t now) {
    if (available_ && (now - last_received_ >= config_.freshness_us ||
                       now - last_sample_ >= config_.freshness_us)) {
        change();
        available_ = false;
        unavailable_since_ = std::min(last_received_, last_sample_) + config_.freshness_us;
    }
    const auto timeout = ever_valid_ ? config_.lost_after_us : config_.startup_grace_us;
    if (!available_ && !lost_ && !conflict_ && now - unavailable_since_ >= timeout) {
        change();
        lost_ = true;
        enqueue("COMM_LOST", now);
    }
}
bool Rules::relevant(const Reminder& r, uint64_t now) const {
    return r.revision == revision_ && now < r.expires;
}
int Rules::pending_priority(uint64_t now) const {
    int result = -1;
    for (const auto& r : queue_)
        if (relevant(r, now))
            result = std::max(result, r.priority);
    return result;
}
std::optional<Reminder> Rules::next(uint64_t now) {
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(),
                                [&](const auto& r) { return !relevant(r, now); }),
                 queue_.end());
    if (queue_.empty())
        return {};
    auto it = std::max_element(queue_.begin(), queue_.end(), [](const auto& a, const auto& b) {
        return a.priority < b.priority;
    });
    const auto result = *it;
    queue_.erase(it);
    return result;
}
} // namespace aviator::tts

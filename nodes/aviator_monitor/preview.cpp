#include "preview.hpp"
#include "runtime.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>
namespace monitor {
namespace {
void require(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
bool unsigned_number(const Json& v) {
    return v.is_number_integer() && v.get<double>() >= 0 &&
           v.get<double>() <= aviator::max_json_integer;
}
// Read SOF dimensions without invoking a decoder in the SUB thread.
bool jpeg_size(const std::string& bytes, unsigned& width, unsigned& height) {
    const auto* p = reinterpret_cast<const unsigned char*>(bytes.data());
    if (bytes.size() < 4 || p[0] != 255 || p[1] != 216 || p[bytes.size() - 2] != 255 ||
        p[bytes.size() - 1] != 217)
        return false;
    std::size_t i = 2;
    while (i + 4 <= bytes.size()) {
        if (p[i++] != 255)
            return false;
        while (i < bytes.size() && p[i] == 255)
            ++i;
        if (i + 3 > bytes.size())
            return false;
        const unsigned marker = p[i++];
        if (marker == 218 || marker == 217)
            return false;
        if (marker == 1 || (marker >= 208 && marker <= 215))
            continue;
        const auto length = static_cast<unsigned>(p[i]) * 256 + p[i + 1];
        if (length < 2 || i + length > bytes.size())
            return false;
        if (marker == 192 || marker == 193 || marker == 194) {
            if (length < 8)
                return false;
            height = p[i + 3] * 256 + p[i + 4];
            width = p[i + 5] * 256 + p[i + 6];
            return width > 0 && height > 0;
        }
        i += length;
    }
    return false;
}
std::string identity(const Json& meta) {
    return meta.at("publisher_id").get<std::string>() + "/" +
           meta.at("session_id").get<std::string>() + "/" + meta.at("camera_id").get<std::string>();
}
} // namespace
Preview::Preview(Json settings)
    : settings_(std::move(settings)), session_(aviator::new_instance_id()) {}
void Preview::configure(Json settings) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.swap(settings);
    frames_.clear();
    origins_.clear();
    bytes_ = 0;
    rejected_ = 0;
    error_.clear();
}
void Preview::reject(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++rejected_;
    error_ = reason.substr(0, 200);
}
void Preview::ingest(const std::string& topic, const std::string& metadata, std::string jpeg,
                     std::uint64_t now) {
    Json meta;
    try {
        require(metadata.size() <= 8192 && jpeg.size() <= 2 * 1024 * 1024, "preview size limit");
        std::vector<std::set<std::string>> keys;
        meta = Json::parse(metadata, [&](int depth, Json::parse_event_t event, Json& value) {
            require(depth < 16, "preview nesting limit");
            if (event == Json::parse_event_t::object_start)
                keys.emplace_back();
            if (event == Json::parse_event_t::key)
                require(keys.back().insert(value.get<std::string>()).second,
                        "duplicate preview key");
            if (event == Json::parse_event_t::object_end)
                keys.pop_back();
            return true;
        });
        require(meta.is_object() && meta.at("version") == 1 && meta.at("encoding") == "jpeg",
                "invalid preview envelope");
        for (auto* key : {"publisher_id", "session_id", "camera_id", "clock_id"})
            require(meta.at(key).is_string() && !meta.at(key).get<std::string>().empty() &&
                        meta.at(key).get<std::string>().size() <= 128,
                    "invalid preview identity");
        require(topic == "camera.rgb." + meta.at("camera_id").get<std::string>(),
                "preview topic mismatch");
        for (auto* key : {"sequence", "frame_id", "sample_mono_us", "width", "height"})
            require(unsigned_number(meta.at(key)), "invalid preview integer");
        require(meta.at("sequence").get<std::uint64_t>() > 0, "preview sequence must advance");
        unsigned width = 0, height = 0;
        require(jpeg_size(jpeg, width, height) && width <= 1920 && height <= 1080 &&
                    meta.at("width") == width && meta.at("height") == height,
                "invalid JPEG dimensions");
    } catch (const std::exception& e) {
        reject(e.what());
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (meta.at("camera_id") != settings_.at("camera_id") ||
        meta.at("publisher_id") != settings_.at("publisher_id")) {
        ++rejected_;
        error_ = "unexpected preview source";
        return;
    }
    const auto key = identity(meta);
    auto previous = origins_.find(key);
    if (previous != origins_.end() &&
        meta.at("sequence").get<std::uint64_t>() <= previous->second.sequence)
        return;
    if (previous == origins_.end() && origins_.size() == 8) {
        auto oldest = std::min_element(origins_.begin(), origins_.end(), [](auto& a, auto& b) {
            return a.second.received < b.second.received;
        });
        origins_.erase(oldest);
    }
    origins_[key] = {meta.at("sequence").get<std::uint64_t>(), now};
    bytes_ += jpeg.size();
    frames_.push_back(
        {session_ + "-" + std::to_string(++counter_), std::move(jpeg), std::move(meta), now});
    while (frames_.size() > 4 || bytes_ > 8 * 1024 * 1024) {
        bytes_ -= frames_.front().jpeg.size();
        frames_.pop_front();
    }
}
Json Preview::latest(std::uint64_t now, const std::string& clock) {
    std::lock_guard<std::mutex> lock(mutex_);
    Json result{{"state", settings_.at("endpoint") == "" ? "UNCONFIGURED" : "WAITING"},
                {"camera_id", settings_.at("camera_id")},
                {"current", nullptr},
                {"rejected", rejected_},
                {"error", error_},
                {"cached_frames", frames_.size()},
                {"cached_bytes", bytes_},
                {"fresh_for_ms", nullptr},
                {"receive_age_ms", nullptr},
                {"sample_age_ms", nullptr}};
    if (frames_.empty())
        return result;
    const auto limit = static_cast<std::uint64_t>(settings_.at("timeout_ms").get<double>() * 1000);
    auto count = std::count_if(origins_.begin(), origins_.end(), [&](auto& o) {
        return now >= o.second.received && now - o.second.received < limit;
    });
    if (count > 1) {
        result["state"] = "SOURCE_CONFLICT";
        return result;
    }
    const auto& f = frames_.back();
    result["receive_age_ms"] = (now - f.received) / 1000.0;
    result["state"] = "FRESH";
    result["current"] = f.meta;
    result["current"]["token"] = f.token;
    result["current"]["url"] = "/api/camera/frame/" + f.token;
    if (f.meta.at("clock_id") != clock)
        result["state"] = "CLOCK_UNKNOWN";
    else if (f.meta.at("sample_mono_us").get<std::uint64_t>() > now)
        result["state"] = "FUTURE";
    else {
        auto age = now - f.meta.at("sample_mono_us").get<std::uint64_t>();
        result["sample_age_ms"] = age / 1000.0;
        if (age >= limit || now - f.received >= limit)
            result["state"] = "STALE";
        else
            result["fresh_for_ms"] = (limit - std::max(age, now - f.received)) / 1000.0;
    }
    return result;
}
std::string Preview::frame(const std::string& token) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& f : frames_)
        if (f.token == token)
            return f.jpeg;
    return {};
}
} // namespace monitor

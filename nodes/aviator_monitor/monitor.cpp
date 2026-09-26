#include "monitor.hpp"
#include <algorithm>

namespace monitor {
using Json = nlohmann::json;
namespace {
std::uint64_t timeout(aviator::Topic topic) {
    using aviator::Topic;
    if (topic == Topic::arm_command || topic == Topic::arm_state ||
        topic == Topic::hand_command || topic == Topic::hand_state) return 50000;
    if (topic == Topic::flight_command || topic == Topic::flight_state) return 100000;
    if (topic == Topic::camera_command || topic == Topic::camera_detection) return 200000;
    return 2000000;
}
Json field(const Json& body, const char* path) {
    try {
        const auto& value = body.at(Json::json_pointer(path));
        if (value.is_string()) return value.get<std::string>().substr(0, 128);
        if (value.is_primitive()) return value;
    } catch (const Json::exception&) {}
    return nullptr;
}
}
void State::reject(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex);
    ++rejected; error = reason.substr(0, 200);
}
void State::ingest(const std::string& topic, const std::string& payload, std::uint64_t now) {
    aviator::Message message;
    std::string reason;
    if (!aviator::decode(topic, payload, message, reason)) { reject(reason); return; }
    std::lock_guard<std::mutex> lock(mutex);
    const auto& h = message.header;
    auto it = std::find_if(streams.begin(), streams.end(), [&](const Stream& s) {
        return s.message.topic == message.topic && s.message.header.publisher_id == h.publisher_id &&
               s.message.header.session_id == h.session_id;
    });
    if (it == streams.end()) {
        if (streams.size() == 64) {
            streams.erase(std::min_element(streams.begin(), streams.end(),
                [](const Stream& a, const Stream& b) { return a.received_us < b.received_us; }));
            ++evicted;
        }
        streams.push_back({next_id++, message, payload, now, 0, 0, {now}});
        return;
    }
    if (h.sequence <= it->message.header.sequence) { ++it->duplicates; return; }
    it->gaps += h.sequence - it->message.header.sequence - 1;
    it->message = std::move(message); it->payload = payload; it->received_us = now;
    it->arrivals.push_back(now);
    while (it->arrivals.size() > 512 || now - it->arrivals.front() > 2000000) it->arrivals.pop_front();
}
Json State::snapshot(std::uint64_t now, const std::string& clock) {
    std::lock_guard<std::mutex> lock(mutex);
    Json rows = Json::array();
    for (const auto& s : streams) {
        const auto& h = s.message.header;
        const auto& body = s.message.body;
        const auto limit = timeout(s.message.topic);
        const bool same_clock = h.clock_id == clock;
        std::string status = "FRESH";
        if (!same_clock) status = "CLOCK_UNKNOWN";
        else if (h.sample_mono_us > now) status = "FUTURE";
        else if (now - s.received_us >= limit || now - h.sample_mono_us >= limit) status = "STALE";
        else if (!h.valid) status = "INVALID";
        if (s.message.topic == aviator::Topic::system_event) status = "EVENT";
        if (status == "FRESH" && (s.message.topic == aviator::Topic::arm_command ||
            s.message.topic == aviator::Topic::hand_command)) {
            aviator::Origin origin; std::string reason;
            if (!aviator::read_origin(body, origin, reason) || origin.clock_id != clock) status = "CLOCK_UNKNOWN";
            else if (origin.sample_mono_us > now) status = "FUTURE";
            else if (now - origin.sample_mono_us >= 100000) status = "STALE_ORIGIN";
        }
        const auto count = std::count_if(s.arrivals.begin(), s.arrivals.end(),
            [&](std::uint64_t time) { return now >= time && now - time < 2000000; });
        // Fixed 2-second window, including startup; receiving rate, not source rate.
        Json row{{"id", s.id}, {"topic", aviator::topic_name(s.message.topic)},
            {"publisher", h.publisher_id}, {"session", h.session_id}, {"sequence", h.sequence},
            {"valid", h.valid}, {"status", status}, {"hz", count / 2.0},
            {"age_ms", same_clock && now >= h.sample_mono_us ? Json((now - h.sample_mono_us) / 1000.0) : Json(nullptr)},
            {"receive_age_ms", (now - s.received_us) / 1000.0}, {"gaps", s.gaps}, {"duplicates", s.duplicates},
            {"summary", {{"state", field(body, "/system/state")}, {"source", field(body, "/system/control_source")},
                {"error", field(body, "/system/current_error_code")}, {"last_error", field(body, "/system/last_error_code")},
                {"roll", field(body, "/control/roll")}, {"pitch", field(body, "/control/pitch")},
                {"vision", field(body, "/vision/status")}, {"confidence", field(body, "/confidence")}}}};
        if (s.message.topic == aviator::Topic::flight_state) {
            for (const auto* group : {"arm", "hand", "camera"}) {
                const auto base = std::string("/freshness/") + group;
                const auto valid = field(body, (base + "/valid").c_str());
                const auto age = field(body, (base + "/age_ms").c_str());
                row["summary"][group] = {{"valid", valid}, {"age_ms", nullptr}};
                if (same_clock && now >= h.sample_mono_us && age.is_number() && age.get<double>() >= 0)
                    row["summary"][group]["age_ms"] = age.get<double>() + (now - h.sample_mono_us) / 1000.0;
            }
        }
        rows.push_back(std::move(row));
    }
    return {{"streams", rows}, {"rejected", rejected}, {"evicted", evicted}, {"error", error}};
}
std::string State::detail(unsigned id) {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& stream : streams) if (stream.id == id) return stream.payload;
    return {};
}
} // namespace monitor

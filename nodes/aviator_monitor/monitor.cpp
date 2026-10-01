#include "monitor.hpp"
#include "service.hpp"
#include <algorithm>

namespace monitor {
using Json = nlohmann::json;
namespace {
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
    if (topic == aviator::service_request_topic || topic == aviator::service_reply_topic) {
        ingest_service(topic, payload, now);
        return;
    }
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
void State::ingest_service(const std::string& topic, const std::string& payload,
                           std::uint64_t now) {
    Json message;
    const bool reply = topic == aviator::service_reply_topic;
    try {
        message = aviator::decode_service(payload);
        if (message.at("msg_type") != (reply ? "ServiceReply" : "ServiceRequest"))
            throw std::invalid_argument("service topic/type mismatch");
    } catch (const std::exception& e) {
        reject(e.what());
        return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    auto it = std::find_if(services.begin(), services.end(), [&](const Service& s) {
        return s.client == message.at("client_id") &&
               s.request_id == message.at("request_id");
    });
    if (it == services.end()) {
        if (services.size() == 64) {
            services.erase(std::min_element(
                services.begin(), services.end(),
                [](const Service& a, const Service& b) { return a.received_us < b.received_us; }));
            ++service_evicted;
        }
        Service item;
        item.id = next_id++;
        item.client = message.at("client_id");
        item.session = message.at("client_session_id");
        item.request_id = message.at("request_id");
        services.push_back(std::move(item));
        it = std::prev(services.end());
    }
    auto& s = *it;
    if (reply) {
        if (!s.request.is_null() && !aviator::matches_service_reply(s.request, message)) {
            ++rejected;
            error = "service reply identity mismatch";
            return;
        }
        ++s.reply_count;
        // Repeated/older copies must not replace a later decision or refresh its age.
        if (!s.reply.is_null() && message.at("timestamp") <= s.reply.at("timestamp"))
            return;
        s.reply = message;
        s.reply_raw = payload;
        s.reply_received_us = now;
    } else {
        auto original = message;
        original.erase("gateway_observation");
        original.erase("client_session_id");
        original["parameters"].erase("server_session_id");
        if (!s.request.is_null()) {
            auto previous = s.request;
            previous.erase("gateway_observation");
            previous.erase("client_session_id");
            previous["parameters"].erase("server_session_id");
            if (original != previous) {
                ++rejected;
                error = "service request identity/content conflict";
                return;
            }
        }
        if (!s.reply.is_null() && !aviator::matches_service_reply(message, s.reply)) {
            ++rejected;
            error = "orphan service reply does not match request";
            s.reply = nullptr;
            s.reply_raw.clear();
            s.reply_received_us = 0;
        }
        if (s.request.is_null())
            s.request_received_us = now;
        ++s.request_count;
        const auto observation = message.value("gateway_observation", "OBSERVED");
        if (s.observation != "TIMEOUT_UNKNOWN" || observation == "TIMEOUT_UNKNOWN") {
            s.request = message;
            s.request_raw = payload;
            s.observation = observation;
        }
    }
    s.received_us = now;
}
std::uint64_t State::timeout_us(aviator::Topic topic) const {
    const auto name = std::string(aviator::topic_name(topic));
    const auto& values = config.at("timeouts_ms");
    return values.contains(name) ? static_cast<std::uint64_t>(values.at(name).get<double>() * 1000) : 2000000;
}
Json State::snapshot(std::uint64_t now, const std::string& clock) {
    std::lock_guard<std::mutex> lock(mutex);
    Json rows = Json::array();
    for (const auto& s : streams) {
        const auto& h = s.message.header;
        const auto& body = s.message.body;
        const auto limit = timeout_us(s.message.topic);
        const bool same_clock = h.clock_id == clock;
        std::uint64_t effective_sample = h.sample_mono_us;
        if (s.message.topic == aviator::Topic::flight_command && body.contains("input_state")) {
            std::uint64_t checked = 0; bool connected = false; std::string reason;
            if (aviator::read_position_hold(s.message, checked, connected, reason) && connected) effective_sample = checked;
        }
        std::string status = "FRESH";
        if (!same_clock) status = "CLOCK_UNKNOWN";
        else if (effective_sample > now) status = "FUTURE";
        else if (now - s.received_us >= limit || now - effective_sample >= limit) status = "STALE";
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
    Json transactions = Json::array();
    for (const auto& s : services) {
        std::string status = "WAITING";
        if (!s.reply.is_null())
            status = s.reply.at("status");
        else if (s.observation == "NOT_SENT" || s.observation == "TIMEOUT_UNKNOWN")
            status = s.observation;
        else if (s.request.at("clock_id") != clock)
            status = "CLOCK_UNKNOWN";
        else {
            const auto issued = s.request.at("issued_mono_us").get<std::uint64_t>();
            if (issued > now)
                status = "FUTURE";
            else if ((now - issued) / 1000 >= s.request.at("deadline_ms").get<std::uint64_t>())
                status = "TIMEOUT_UNKNOWN";
        }
        transactions.push_back(
            {{"id", s.id},
             {"request_id", s.request_id},
             {"client", s.client},
             {"session", s.session},
             {"operation", s.request.is_null() ? Json(nullptr) : s.request.at("operation")},
             {"target", s.request.is_null() ? s.reply.at("server_id") : s.request.at("target")},
             {"status", status},
             {"observation", s.request.is_null() ? "REPLY_ONLY" : s.observation},
             {"error_code", s.reply.is_null() ? Json(nullptr) : s.reply.at("error_code")},
             {"reply_status", s.reply.is_null() ? Json(nullptr) : s.reply.at("status")},
             {"request_count", s.request_count},
             {"reply_count", s.reply_count},
             {"receive_age_ms", now >= s.received_us ? (now - s.received_us) / 1000.0 : 0},
             {"observed_reply_ms",
              !s.request.is_null() && !s.reply.is_null() &&
                      s.reply_received_us >= s.request_received_us
                  ? Json((s.reply_received_us - s.request_received_us) / 1000.0)
                  : Json(nullptr)}});
    }
    std::reverse(transactions.begin(), transactions.end());
    return {
        {"streams", rows},      {"services", transactions}, {"service_evicted", service_evicted},
        {"rejected", rejected}, {"evicted", evicted},       {"error", error}};
}
std::string State::detail(unsigned id) {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& stream : streams) if (stream.id == id) return stream.payload;
    for (const auto& s : services)
        if (s.id == id)
            return Json{{"request", s.request},
                        {"reply", s.reply},
                        {"request_raw", s.request_raw},
                        {"reply_raw", s.reply_raw},
                        {"gateway_observation", s.observation},
                        {"request_count", s.request_count},
                        {"reply_count", s.reply_count}}
                .dump();
    return {};
}
} // namespace monitor

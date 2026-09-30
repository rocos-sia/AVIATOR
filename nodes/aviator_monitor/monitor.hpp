#pragma once
#include "protocol.hpp"
#include <deque>
#include <mutex>
#include <vector>

namespace monitor {
struct Stream {
    unsigned id;
    aviator::Message message;
    std::string payload;
    std::uint64_t received_us;
    std::uint64_t gaps = 0, duplicates = 0;
    std::deque<std::uint64_t> arrivals;
};
struct Service {
    unsigned id;
    std::string client, session, request_id;
    nlohmann::json request, reply;
    std::string request_raw, reply_raw;
    std::uint64_t received_us = 0, request_received_us = 0, reply_received_us = 0;
    std::uint64_t request_count = 0, reply_count = 0;
    std::string observation;
};
// One bounded cache, shared by the SUB thread and HTTP thread. No control output.
struct State {
    std::mutex mutex;
    std::vector<Stream> streams;
    std::vector<Service> services;
    std::uint64_t service_evicted = 0;
    unsigned next_id = 1;
    std::uint64_t rejected = 0, evicted = 0;
    std::string error;
    void ingest(const std::string& topic, const std::string& payload, std::uint64_t now);
    nlohmann::json snapshot(std::uint64_t now, const std::string& clock);
    std::string detail(unsigned id);
    void ingest_service(const std::string& topic, const std::string& payload, std::uint64_t now);
    void reject(const std::string& reason);
};
} // namespace monitor

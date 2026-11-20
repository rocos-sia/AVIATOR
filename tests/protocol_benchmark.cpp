// Offline benchmark: JSONL rows contain {topic, payload, received_s?}.
// Counts C++ allocations in the codec only, excluding corpus loading/caller setup.
#include "protocol.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <new>
#include <vector>
namespace {
thread_local bool counting = false;
thread_local std::size_t allocations = 0;
}
void* operator new(std::size_t n) {
    if (counting) ++allocations;
    if (auto* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete[](void* p) noexcept { ::operator delete(p); }
void operator delete[](void* p, std::size_t) noexcept { ::operator delete(p); }
using Json = nlohmann::json;
struct Packet { std::string payload; aviator::Message message; };
int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: protocol_benchmark --bench|--verify corpus.jsonl");
        std::ifstream file(argv[2]);
        if (!file) throw std::runtime_error("cannot open corpus");
        const bool verify = std::string(argv[1]) == "--verify";
        std::map<std::string, std::vector<Packet>> topics;
        std::string line;
        while (std::getline(file, line)) {
            const auto row = Json::parse(line);
            const auto topic = row.at("topic").get<std::string>();
            auto payload = row.at("payload").get<std::string>();
            aviator::Message message;
            std::string error, encoded;
            const bool ok = aviator::decode(topic, payload, message, error);
            if (verify) {
                if (ok && !aviator::encode(message, encoded, error))
                    throw std::runtime_error("accepted packet could not be encoded");
                std::cout << Json{{"accepted", ok}, {"error", error}, {"encoded", encoded}}.dump() << '\n';
            } else if (ok) topics[topic].push_back({std::move(payload), std::move(message)});
        }
        if (verify) return 0;
        for (const auto& [topic, packets] : topics) {
            std::size_t bytes = 0;
            for (const auto& packet : packets) bytes += packet.payload.size();
            for (const auto* mode : {"decode", "encode_const", "encode_move"}) {
#ifdef AVIATOR_BENCHMARK_BASELINE
                if (std::string(mode) == "encode_move") continue;
#endif
                for (int round = 0; round < 3; ++round) {
                    std::size_t count = 0, total_allocations = 0;
                    double ns = 0;
                    const auto samples = std::min<std::size_t>(packets.size(), 256);
                    for (int repeat = 0; repeat < 4; ++repeat) {
                        for (std::size_t i = 0; i < samples; ++i) {
                            const auto& packet = packets[i * packets.size() / samples];
                            auto message = packet.message;
                            std::string payload, error;
                            allocations = 0; counting = true;
                            const auto begin = std::chrono::steady_clock::now();
                            bool ok;
                            if (std::string_view(mode) == "decode") {
                                aviator::Message decoded;
                                ok = aviator::decode(topic, packet.payload, decoded, error);
                            } else if (std::string_view(mode) == "encode_const") {
                                ok = aviator::encode(std::as_const(message), payload, error);
                            } else {
#ifndef AVIATOR_BENCHMARK_BASELINE
                                ok = aviator::encode(std::move(message), payload, error);
#else
                                ok = false;
#endif
                            }
                            const auto end = std::chrono::steady_clock::now();
                            counting = false;
                            if (!ok) throw std::runtime_error(error);
                            ns += std::chrono::duration<double, std::nano>(end - begin).count();
                            total_allocations += allocations; ++count;
                        }
                    }
                    std::cout << Json{{"topic", topic}, {"mode", mode}, {"round", round},
                        {"corpus_packets", packets.size()}, {"mean_bytes", double(bytes) / packets.size()},
                        {"samples", count}, {"us_per_packet", ns / count / 1000},
                        {"allocations_per_packet", double(total_allocations) / count}}.dump() << '\n';
                }
            }
        }
    } catch (const std::exception& e) { counting = false; std::cerr << e.what() << '\n'; return 1; }
}

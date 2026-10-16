#include "Logger.hpp"
#include <spdlog/sinks/ostream_sink.h>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <condition_variable>
#include <future>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void check(bool async) {
    std::ostringstream output;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    aviator::Logger::configure(async, spdlog::level::info, sink);
    aviator::Logger::debug("filtered message");
    aviator::Logger::info("formatted value={:.2f}", 1.25);
    std::vector<std::thread> workers;
    for (int worker = 0; worker < 4; ++worker) {
        workers.emplace_back([worker] {
            for (int index = 0; index < 2500; ++index)
                aviator::Logger::info("worker={} item={}", worker, index);
        });
    }
    for (auto& worker : workers) worker.join();
    aviator::Logger::warn("warning marker");
    aviator::Logger::error("error marker");
    // Reconfiguration must drain the old queue, including more than 8192 records.
    aviator::Logger::configure();
    const auto text = output.str();
    require(text.find("filtered message") == std::string::npos, "level filter failed");
    require(text.find("formatted value=1.25") != std::string::npos, "format failed");
    require(text.find("[warning]") != std::string::npos, "warning level missing");
    require(text.find("[error]") != std::string::npos, "error level missing");
    std::istringstream lines(text);
    std::string line;
    int count = 0;
    while (std::getline(lines, line)) ++count;
    require(count == 10003, "logging lost or interleaved records");
    for (int worker = 0; worker < 4; ++worker)
        require(text.find(fmt::format("worker={} item=2499", worker)) != std::string::npos,
                "async queue not drained");
}
// Hold the output sink indefinitely, including warning/error auto-flushes.
// Producers must progress even after exceeding the entire bounded log queue.
class BlockedSink final : public spdlog::sinks::base_sink<std::mutex> {
public:
    std::mutex gate;
    std::condition_variable changed;
    bool entered = false, released = false, recovered = false;
    void release() {
        std::lock_guard<std::mutex> lock(gate);
        released = true;
        changed.notify_all();
    }
private:
    void sink_it_(const spdlog::details::log_msg& message) override {
        std::unique_lock<std::mutex> lock(gate);
        entered = true;
        changed.notify_all();
        changed.wait(lock, [&] { return released; });
        if (std::string(message.payload.data(), message.payload.size()) == "recovery marker")
            recovered = true;
    }
    void flush_() override {}
};
void nonblocking_check() {
    auto sink = std::make_shared<BlockedSink>();
    aviator::Logger::configureNonBlocking(sink);
    aviator::Logger::info("block output");
    bool entered;
    {
        std::unique_lock<std::mutex> lock(sink->gate);
        entered = sink->changed.wait_for(lock, std::chrono::seconds(2), [&] { return sink->entered; });
    }
    auto producer = std::async(std::launch::async, [] {
        for (int i = 0; i < 10000; ++i) {
            aviator::Logger::warn("hand warning {}", i);
            aviator::Logger::error("motion fault {}", i);
        }
        aviator::Logger::flush();
    });
    const bool finished = producer.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    const auto overruns = aviator::Logger::overrunCount();
    sink->release(); // Always unblock cleanup, including the negative test case.
    producer.get();
    aviator::Logger::info("recovery marker");
    aviator::Logger::configure(); // Drain and join before checking output recovery.
    require(entered, "test sink never blocked");
    require(finished, "control producer waited for blocked output/full log queue");
    require(overruns > 0, "test did not saturate the diagnostic queue");
    require(sink->recovered, "logging did not resume after output recovered");
    require(aviator::Logger::overrunCount() == 0, "old overrun count leaked into new logger");
}
}
int main() {
    try {
        check(false);
        check(true);
        nonblocking_check();
        return 0;
    } catch (const std::exception& error) {
        aviator::Logger::error("Logger test: {}", error.what());
        return 1;
    }
}

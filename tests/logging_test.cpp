#include "Logger.hpp"
#include <spdlog/sinks/ostream_sink.h>
#include <chrono>
#include <future>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

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
// Model a blocked stdout pipe or disk write. The logging worker stays blocked
// until the producer has attempted more records than fit in the bounded queue.
class GatedSink final : public spdlog::sinks::base_sink<std::mutex> {
    bool first_ = true;
    std::shared_future<void> release_;
    void sink_it_(const spdlog::details::log_msg& message) override {
        if (first_) {
            first_ = false;
            entered.set_value();
            release_.wait();
        }
        ++count;
        last.assign(message.payload.data(), message.payload.size());
    }
    void flush_() override {}
public:
    explicit GatedSink(std::shared_future<void> release) : release_(std::move(release)) {}
    std::promise<void> entered;
    size_t count = 0;
    std::string last;
};
void checkControlBackpressure() {
    using namespace std::chrono_literals;
    std::promise<void> release;
    auto sink = std::make_shared<GatedSink>(release.get_future().share());
    auto entered = sink->entered.get_future();
    aviator::Logger::configureControl(sink);
    aviator::Logger::info("block output");
    const bool started = entered.wait_for(2s) == std::future_status::ready;
    if (!started) {
        release.set_value();
        aviator::Logger::configure();
        require(false, "logging worker did not start");
    }
    auto producer = std::async(std::launch::async, [] {
        for (int i = 0; i < 20000; ++i) aviator::Logger::info("control tick={}", i);
        aviator::Logger::error("latest fault retained");
    });
    const bool completed = producer.wait_for(2s) == std::future_status::ready;
    // Always unblock before asserting, so a regression fails without deadlock.
    release.set_value();
    producer.get();
    aviator::Logger::configure(); // Join/drain before examining the sink.
    require(completed, "control logging waited for blocked output/full queue");
    require(sink->count <= 8193, "control logging did not bound its backlog");
    require(sink->last == "latest fault retained", "queue did not retain newest fault");
}
}
int main() {
    try {
        check(false);
        check(true);
        checkControlBackpressure();
        return 0;
    } catch (const std::exception& error) {
        aviator::Logger::error("Logger test: {}", error.what());
        return 1;
    }
}

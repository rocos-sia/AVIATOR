#include "Logger.hpp"
#include <spdlog/sinks/ostream_sink.h>
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
}
int main() {
    try {
        check(false);
        check(true);
        return 0;
    } catch (const std::exception& error) {
        aviator::Logger::error("Logger test: {}", error.what());
        return 1;
    }
}

#include "Logger.hpp"
#include "logger.hpp"
#include "runtime.hpp"
#include "startup.hpp"
#include "transport.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <limits>
#include <pthread.h>
#include <signal.h>
#include <string>
#include <thread>

namespace {
// Use local start time as the readable batch name, with microsecond precision
// for rapid restarts. RecordingWriter still refuses to overwrite existing files.
std::string default_output_path() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const std::time_t now = static_cast<std::time_t>(elapsed / 1'000'000);
    std::tm local{};
    localtime_r(&now, &local);
    char buffer[40];
    std::strftime(buffer, sizeof(buffer), "aviator_%Y-%m-%d_%H-%M-%S", &local);
    char suffix[24];
    std::snprintf(suffix, sizeof(suffix), "_%06lld.mcap",
                  static_cast<long long>(elapsed % 1'000'000));
    return std::string(buffer) + suffix;
}

void usage() {
    aviator::Logger::info("Usage: aviator_logger [--config config/recording.yaml] [--output FILE] [--image-output "
        "FILE]\n"
        "                       [--subscribe tcp://127.0.0.1:5556] [--session LABEL]\n"
        "                       [--queue-bytes 16777216] [--receive-hwm 4096]\n"
        "Writes data and images to separate MCAP files; SIGINT/SIGTERM finalizes.");
}

// Log times are stored as UTC ns; render an ISO-8601 timestamp (ms) for stdout.
std::string format_utc_ns(std::uint64_t ns) {
    const std::time_t secs = static_cast<std::time_t>(ns / 1'000'000'000ULL);
    const int ms = static_cast<int>((ns % 1'000'000'000ULL) / 1'000'000ULL);
    std::tm utc{};
    gmtime_r(&secs, &utc);
    char buffer[40];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &utc);
    const std::size_t length = std::strlen(buffer);
    std::snprintf(buffer + length, sizeof(buffer) - length, ".%03dZ", ms);
    return buffer;
}
} // namespace

int main(int argc, char** argv) {
    int result = 0;
    try {
        aviator::RecordingConfig config;
        // Load YAML before applying explicit CLI overrides, independent of argument order.
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                usage();
                return 0;
            }
            if (key == "--config") {
                if (++i == argc)
                    throw std::runtime_error("missing --config path");
                config = aviator::load_recording_config(argv[i]);
            } else if (key == "--output" || key == "--image-output" || key == "--subscribe" || key == "--session" ||
                       key == "--queue-bytes" || key == "--receive-hwm")
                ++i;
        }
        std::string endpoint = config.subscribe_endpoint;
        std::string output = config.output.empty() ? default_output_path() : config.output;
        std::string session = aviator::new_instance_id();
        aviator::RecorderOptions options = config.options;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                usage();
                return 0;
            }
            if (key == "--config") {
                ++i;
                continue;
            }
            if (key != "--output" && key != "--image-output" && key != "--subscribe" && key != "--session" &&
                key != "--queue-bytes" && key != "--receive-hwm")
                throw std::runtime_error("unknown argument: " + key);
            if (++i == argc || std::string(argv[i]).empty())
                throw std::runtime_error("missing value for " + key);
            if (key == "--output")
                output = argv[i];
            else if (key == "--image-output")
                options.image_output = argv[i];
            else if (key == "--subscribe")
                endpoint = argv[i];
            else if (key == "--session")
                session = argv[i];
            else {
                const std::string value = argv[i];
                if (value.find_first_not_of("0123456789") != std::string::npos)
                    throw std::runtime_error("invalid positive integer for " + key);
                const auto number = std::stoull(value);
                if (number == 0 ||
                    number > (key == "--receive-hwm"
                                  ? static_cast<unsigned long long>(std::numeric_limits<int>::max())
                                  : static_cast<unsigned long long>(
                                        std::numeric_limits<std::size_t>::max())))
                    throw std::runtime_error("out-of-range value for " + key);
                if (key == "--receive-hwm")
                    options.receive_hwm = static_cast<int>(number);
                else
                    options.queue_bytes = static_cast<std::size_t>(number);
            }
        }
        if (endpoint.rfind("tcp://", 0) != 0)
            throw std::runtime_error("subscription must use TCP");

        aviator::validate_recording_config({endpoint, output, options});

        // Block signals before spawning threads; no handler touches ZMQ or C++ objects.
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0)
            throw std::runtime_error("signal mask failed");

        std::atomic<bool> stop{false};
        std::atomic<bool> failed{false};
        aviator::RecorderSummary summary;
        std::thread worker([&] {
            try {
                summary = aviator::record_bus(endpoint, output, session, stop, options, [&] {
                    aviator::print_startup(
                        "aviator_logger",
                        {{"SUB connect", endpoint},
                         {"SUB topics", "* (all topics; PUB/SUB delivery is unverified)"},
                         {"Output", output + "  (.partial -> atomic rename)"},
                         {"Image output", options.camera.mode == "disabled" ? "disabled"
                             : aviator::image_output_path(output, options)},
                         {"Session", session},
                         {"Queue bytes", std::to_string(options.queue_bytes)},
                         {"Camera mode", options.camera.mode},
                         {"Camera ingress", options.camera.mode == "disabled"
                                                ? "disabled"
                                                : options.camera.record_endpoint},
                         {"Camera state", options.camera.mode == "disabled"
                                              ? "DISABLED"
                                              : "LISTENING (waiting for RGB/depth sources)"},
                         {"Exit", "Ctrl+C (drain + finalize)"}});
                });
            } catch (const std::exception& error) {
                aviator::Logger::error("aviator_logger: {}", error.what());
                failed.store(true);
            }
            stop.store(true);
        });

        const timespec timeout{0, 100000000};
        while (!stop.load()) {
            const int signal = sigtimedwait(&signals, nullptr, &timeout);
            if (signal == SIGINT || signal == SIGTERM)
                break;
            if (signal < 0 && errno != EAGAIN && errno != EINTR)
                break;
        }
        stop.store(true);
        worker.join();

        if (failed.load())
            return 1;
        aviator::Logger::info("recorded {} messages across {} topic(s)", summary.messages, summary.topics.size());
        aviator::Logger::info("  data: {} messages -> {}", summary.messages - summary.camera_messages, summary.path);
        if (!summary.image_path.empty())
            aviator::Logger::info("  images: {} messages -> {}", summary.camera_messages, summary.image_path);
        for (const auto& [topic, stats] : summary.topics)
            aviator::Logger::info("  {}  ({})  {} msgs", topic, stats.type, stats.messages);
        if (summary.messages != 0)
            aviator::Logger::info("  window (UTC): {} -> {}",
                format_utc_ns(summary.start_log_ns), format_utc_ns(summary.end_log_ns));
        if (summary.invalid || summary.rejected || summary.dropped)
            aviator::Logger::warn("  skipped: {} invalid, {} rejected, {} queue overflow",
                summary.invalid, summary.rejected, summary.dropped);
        aviator::Logger::info("  sequence gaps: {}, duplicate/reordered: {}; completeness unverified (PUB/SUB)",
            summary.sequence_gaps, summary.duplicate_or_reordered);
    } catch (const std::exception& error) {
        aviator::Logger::error("aviator_logger: {}", error.what());
        result = 1;
    }
    return result;
}

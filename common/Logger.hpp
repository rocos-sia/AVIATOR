#pragma once

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <atomic>
#include <memory>
#include <spdlog/sinks/dist_sink.h>
#include <cstdlib>
#include <mutex>
#include <utility>

namespace aviator {
// Process-wide console logger. Configure at startup, before starting workers.
// A private pool keeps async logging independent of spdlog's global registry.
class Logger {
    class ConsoleSink final : public spdlog::sinks::dist_sink_mt {
        std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> out_ =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>(
                std::getenv("NO_COLOR") ? spdlog::color_mode::never : spdlog::color_mode::automatic);
        std::shared_ptr<spdlog::sinks::stderr_color_sink_mt> err_ =
            std::make_shared<spdlog::sinks::stderr_color_sink_mt>(
                std::getenv("NO_COLOR") ? spdlog::color_mode::never : spdlog::color_mode::automatic);
        void sink_it_(const spdlog::details::log_msg& message) override {
            (message.level >= spdlog::level::warn ? static_cast<spdlog::sinks::sink&>(*err_)
                                                 : static_cast<spdlog::sinks::sink&>(*out_)).log(message);
        }
    public:
        ConsoleSink() { add_sink(out_); add_sink(err_); }
    };
    struct Backend {
        std::shared_ptr<spdlog::details::thread_pool> pool;
        std::shared_ptr<spdlog::logger> logger;
        explicit Backend(bool async, spdlog::sink_ptr sink) {
            if (async) {
                pool = std::make_shared<spdlog::details::thread_pool>(8192, 1);
                logger = std::make_shared<spdlog::async_logger>("aviator", sink, pool,
                    spdlog::async_overflow_policy::block);
            } else {
                logger = std::make_shared<spdlog::logger>("aviator", sink);
            }
            logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] [thread %t] %v");
            logger->flush_on(spdlog::level::warn);
        }
        ~Backend() noexcept { try { logger->flush(); } catch (...) {} }
    };
    static std::shared_ptr<Backend>& backend() {
        static auto instance = std::make_shared<Backend>(false, std::make_shared<ConsoleSink>());
        return instance;
    }
public:
    static void configure(bool async = false, spdlog::level::level_enum level = spdlog::level::info,
                          spdlog::sink_ptr sink = nullptr) {
        auto next = std::make_shared<Backend>(async, sink ? std::move(sink) : std::make_shared<ConsoleSink>());
        next->logger->set_level(level);
        std::atomic_store(&backend(), std::move(next));
    }
    static void set_level(spdlog::level::level_enum level) {
        std::atomic_load(&backend())->logger->set_level(level);
    }
    static void flush() { std::atomic_load(&backend())->logger->flush(); }
    template <typename... Args>
    static void log(spdlog::level::level_enum level, fmt::format_string<Args...> format, Args&&... args) noexcept {
        try {
            auto current = std::atomic_load(&backend());
            current->logger->log(level, format, std::forward<Args>(args)...);
        } catch (...) {
            // Logging failures must not interrupt device control or exception unwinding.
        }
    }
#define AVIATOR_LOG_METHOD(name) \
    template <typename... Args> \
    static void name(fmt::format_string<Args...> format, Args&&... args) { \
        log(spdlog::level::name, format, std::forward<Args>(args)...); \
    }
    AVIATOR_LOG_METHOD(trace)
    AVIATOR_LOG_METHOD(debug)
    AVIATOR_LOG_METHOD(info)
    AVIATOR_LOG_METHOD(warn)
    AVIATOR_LOG_METHOD(critical)
#undef AVIATOR_LOG_METHOD
    template <typename... Args>
    static void error(fmt::format_string<Args...> format, Args&&... args) {
        log(spdlog::level::err, format, std::forward<Args>(args)...);
    }

    // Machine-readable CLI output must keep its original format (e.g. --check-config).
    template <typename... Args>
    static void output(fmt::format_string<Args...> format, Args&&... args) {
        static spdlog::logger raw("output", [] {
            auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::never);
            sink->set_pattern("%v");
            return sink;
        }());
        raw.info(format, std::forward<Args>(args)...);
        raw.flush();
    }
};
} // namespace aviator

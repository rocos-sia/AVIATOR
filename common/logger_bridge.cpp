#include "Logger.hpp"
#include <spdlog/sinks/base_sink.h>

namespace {
using Callback = void (*)(int, const char*);
class PythonSink final : public spdlog::sinks::base_sink<std::mutex> {
    Callback callback_;
    void sink_it_(const spdlog::details::log_msg& message) override {
        spdlog::memory_buf_t buffer;
        formatter_->format(message, buffer);
        const std::string text(buffer.data(), buffer.size());
        callback_(static_cast<int>(message.level), text.c_str());
    }
    void flush_() override {}
public:
    explicit PythonSink(Callback callback) : callback_(callback) {}
};
}
// Never propagate C++ exceptions across the Python FFI boundary.
extern "C" int aviator_logger_configure(int async, Callback callback) noexcept {
    try {
        aviator::Logger::configure(async != 0, spdlog::level::info,
                                  std::make_shared<PythonSink>(callback));
        return 0;
    } catch (...) { return -1; }
}
extern "C" void aviator_logger_write(int level, const char* text) noexcept {
    try { aviator::Logger::log(static_cast<spdlog::level::level_enum>(level), "{}", text); }
    catch (...) {}
}
extern "C" void aviator_logger_shutdown() noexcept {
    try { aviator::Logger::configure(); } catch (...) {}
}

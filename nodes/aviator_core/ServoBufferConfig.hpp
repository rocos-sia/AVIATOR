#pragma once
#include "motion.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>

namespace aviator {
struct ServoBufferConfig {
    size_t prefill_ms = 80;
    size_t lookahead_ms = 80;
    size_t effective_prefill_ms = 80;

    static ServoBufferConfig load(const YAML::Node& config, double period_s) {
        if (!std::isfinite(period_s) || period_s < .01 || period_s > .05 ||
            std::abs(period_s * 1000 - std::round(period_s * 1000)) >= 1e-8)
            throw std::runtime_error("servo_period must be 10..50 integer milliseconds (in seconds)");
        const size_t block_ms = std::llround(period_s * 1000);
        const auto read = [&](const char* key) -> size_t {
            long long value = 80;
            if (config[key]) {
                try { value = config[key].as<long long>(); }
                catch (const YAML::Exception&) {
                    throw std::runtime_error(std::string(key) + " must be an integer number of milliseconds");
                }
            }
            if (value < 1 || value >= static_cast<long long>(servo_queue_points))
                throw std::runtime_error(std::string(key) + " must be in [1, 250] ms");
            return static_cast<size_t>(value);
        };
        ServoBufferConfig result;
        result.prefill_ms = read("servo_prefill_ms");
        result.lookahead_ms = read("servo_lookahead_ms");
        result.effective_prefill_ms = ((result.prefill_ms + block_ms - 1) / block_ms) * block_ms;
        if (result.effective_prefill_ms >= servo_queue_points)
            throw std::runtime_error("servo_prefill_ms rounded up to servo_period exceeds 250 ms");
        // Reserve one whole planning block, retained history and the shared
        // endpoint counted by RemoteLink's conservative append capacity check.
        const size_t max_lookahead_ms = servo_queue_points - 1 - block_ms - servo_history_ticks - 1;
        if (result.lookahead_ms > max_lookahead_ms)
            throw std::runtime_error("servo_lookahead_ms must be <= " + std::to_string(max_lookahead_ms) +
                                     " ms for this servo_period (queue capacity including history)");
        return result;
    }
};
} // namespace aviator

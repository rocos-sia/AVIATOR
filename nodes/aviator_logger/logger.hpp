#pragma once

#include "recording.hpp"
#include <atomic>
#include <cstddef>
#include <functional>
#include <string>

namespace aviator {
// Caller owns stop and joins this function. SUB lives here; a separate worker
// exclusively owns MCAP. Shutdown drains all accepted queue entries.
RecorderSummary record_bus(const std::string& subscribe_endpoint, const std::string& output_path,
                           const std::string& session_id, const std::atomic<bool>& stop,
                           const RecorderOptions& options = {},
                           const std::function<void()>& on_ready = {});
} // namespace aviator

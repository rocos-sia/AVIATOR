#pragma once
#include "config.hpp"
#include "runtime.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <map>

namespace monitor {
// Single-owner collector; the monitor samples it in a worker and serves cached
// results to HTTP clients. snapshot() itself is not thread-safe.
class SystemStats {
  public:
    explicit SystemStats(std::filesystem::path proc = "/proc", std::filesystem::path sys = "/sys",
                         std::uint64_t started_us = aviator::monotonic_us())
        : proc_(std::move(proc)), sys_(std::move(sys)), started_us_(started_us) {}
    Json snapshot(std::uint64_t now);

  private:
    using Counters = std::map<std::string, std::array<std::uint64_t, 2>>;
    std::filesystem::path proc_, sys_;
    Counters cpu_, disks_, network_, intel_gpu_;
    const std::uint64_t started_us_;
    std::uint64_t sampled_at_ = 0;
    Json cached_;
};
} // namespace monitor

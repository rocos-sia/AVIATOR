#pragma once
#include "config.hpp"
#include "runtime.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>

namespace monitor {
// Single-owner collector; the monitor samples it in a worker and serves cached
// results to HTTP clients. snapshot() itself is not thread-safe.
class SystemStats {
  public:
    explicit SystemStats(std::filesystem::path proc = "/proc", std::filesystem::path sys = "/sys",
                         std::uint64_t started_us = aviator::monotonic_us(),
                         unsigned gpu_discovery_ms = 3000)
        : proc_(std::move(proc)), sys_(std::move(sys)), started_us_(started_us),
          gpu_discovery_us_(std::uint64_t(gpu_discovery_ms) * 1000) {
        if (gpu_discovery_ms < 2000 || gpu_discovery_ms > 5000)
            throw std::invalid_argument("GPU discovery interval must be 2000–5000 ms");
    }
    Json snapshot(std::uint64_t now);

  private:
    using Counters = std::map<std::string, std::array<std::uint64_t, 2>>;
    std::filesystem::path proc_, sys_;
    Counters cpu_, disks_, network_, intel_gpu_;
    const std::uint64_t started_us_;
    std::uint64_t sampled_at_ = 0;
    Json cached_;
    struct GpuClient {
        std::uint64_t starttime;
        std::string device, client;
    };
    std::map<std::filesystem::path, GpuClient> gpu_clients_;
    std::set<std::string> gpu_devices_;
    const std::uint64_t gpu_discovery_us_;
    std::uint64_t gpu_discovered_at_ = 0;
    bool gpu_discovered_ = false;
    Json gpuPercent(std::uint64_t now, double seconds);
    Json intelGpuSample(const std::set<std::string>& devices, std::uint64_t now,
                        double seconds, double& total, unsigned& samples);
};
} // namespace monitor

#pragma once
#include <cstdint>
#include <ctime>
#include <stdexcept>
namespace flight_simulator {
inline std::uint64_t monotonic_us() {
    timespec now{};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        throw std::runtime_error("clock_gettime failed");
    return std::uint64_t(now.tv_sec) * 1000000 + now.tv_nsec / 1000;
}
} // namespace flight_simulator

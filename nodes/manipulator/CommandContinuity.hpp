#pragma once
#include <array>
#include <cmath>
#include <cstddef>

namespace aviator {
// 同时用于执行层和 SDK 回调。固定按一个 1 ms 指令周期检查，延迟不能放宽限值。
template <std::size_t N>
int excessiveJointStep(const std::array<double, N> &previous,
                       const std::array<double, N> &next,
                       const std::array<double, N> &speed) {
    for (std::size_t i = 0; i < N; ++i)
        if (!std::isfinite(previous[i]) || !std::isfinite(next[i]) ||
            !std::isfinite(speed[i]) || speed[i] <= 0 ||
            std::abs(next[i] - previous[i]) > speed[i] * 0.001 + 1e-10)
            return static_cast<int>(i);
    return -1;
}
}

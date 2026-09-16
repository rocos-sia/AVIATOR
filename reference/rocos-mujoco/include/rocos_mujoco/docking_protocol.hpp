// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace rocos_mujoco::docking {
// Existing YAML-mapped digital I/O of one drive (no extra joint).
constexpr int32_t RequestLockB = 1 << 0;
constexpr int32_t RequestReleaseA = 1 << 1; // Rising edge; ignored unless B is confirmed stable.
constexpr int32_t LockedA = 1 << 0;
constexpr int32_t LockedB = 1 << 1;
constexpr int32_t ReadyB = 1 << 2;
constexpr int32_t Fault = 1 << 3;
constexpr int32_t Active = 1 << 4;
constexpr int32_t ClosingB = 1 << 5; // Weld active, post-lock stability not yet confirmed.
constexpr int32_t ReleasedA = 1 << 6; // Latched acknowledgement; B is now the support.
}  // namespace rocos_mujoco::docking

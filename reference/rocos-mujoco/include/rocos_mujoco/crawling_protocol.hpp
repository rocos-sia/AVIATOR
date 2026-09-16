// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace rocos_mujoco::crawling {
// Drive io_slave digital_outputs: opcode[0:3], end[4], anchor[5:7], sequence[8:15].
enum Operation { Lock = 1, Release = 2, Finish = 3 };
constexpr int32_t LockedA = 1, LockedB = 2, Busy = 4, Fault = 8;
constexpr int32_t Active = 16, Done = 32, Rejected = 128;
// digital_inputs: flags[0:7], A anchor+1[8:10], B anchor+1[11:13], ack[16:23].
inline int command(Operation op, int end, int anchor, int sequence) {
    return op | (end << 4) | (anchor << 5) | (sequence << 8);
}
inline int anchor(int status, int end) { return ((status >> (8 + 3 * end)) & 7) - 1; }
inline int acknowledgement(int status) { return (status >> 16) & 255; }
} // namespace rocos_mujoco::crawling


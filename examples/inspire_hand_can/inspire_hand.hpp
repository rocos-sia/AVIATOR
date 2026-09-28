#pragma once
// Inspire robotic hand driver (因时手), ported from Inspire_Robot/src/Inspire.hpp.
//
// The original talks to a GCAN USBCAN-II adapter through the ECanVci library;
// this version talks to the same hand over Linux SocketCAN (see can_writer.hpp).
// The wire protocol is unchanged from the original and matches the hardcoded IDs
// in examples/sub_hand_position.cpp:
//
//   extended 29-bit CAN id = (flag << 26) | (register << 14) | hand_id
//       flag = 1 for a write, 0 for a read
//   2 bytes little-endian payload; value in [0, 1000], or -1 -> 0xFFFF "keep"
//   right hand = hand_id 1, left hand = hand_id 2
//
// All six-channel vectors use the canonical order:
//   [thumb_rot, thumb, index, middle, ring, pinky]

#include <array>
#include <cstdint>
#include <vector>

#include "can_writer.hpp"

namespace inspire_hand {

constexpr int kMaxValue = 1000;
constexpr int kKeepValue = -1;  // 0xFFFF on the wire = "leave this channel alone"

// Register addresses (same numeric values as Inspire_Robot).
struct Registers {
    // Finger target positions.
    static constexpr int THUMB_ROT = 1496;  // 拇指旋转
    static constexpr int THUMB = 1494;      // 拇指
    static constexpr int INDEX = 1492;      // 食指
    static constexpr int MIDDLE = 1490;     // 中指
    static constexpr int RING = 1488;       // 无名指
    static constexpr int PINKY = 1486;      // 小指

    // Finger speed limits.
    static constexpr int SPEED_THUMB_ROT = 1532;
    static constexpr int SPEED_THUMB = 1530;
    static constexpr int SPEED_INDEX = 1528;
    static constexpr int SPEED_MIDDLE = 1526;
    static constexpr int SPEED_RING = 1524;
    static constexpr int SPEED_PINKY = 1522;

    // Finger force limits.
    static constexpr int FORCE_THUMB_ROT = 1508;
    static constexpr int FORCE_THUMB = 1506;
    static constexpr int FORCE_INDEX = 1504;
    static constexpr int FORCE_MIDDLE = 1502;
    static constexpr int FORCE_RING = 1500;
    static constexpr int FORCE_PINKY = 1498;

    // Action-sequence block.
    static constexpr int ACTION_SEQ_CHECKDATA1 = 2000;
    static constexpr int ACTION_SEQ_CHECKDATA2 = 2001;
    static constexpr int ACTION_SEQ_STEPNUM = 2002;
    static constexpr int ACTION_SEQ_STEP0 = 2016;  // steps are packed with stride 2
    static constexpr int ACTION_SEQ_INDEX = 2320;
    static constexpr int SAVE_ACTION_SEQ = 2321;
    static constexpr int ACTION_SEQ_RUN = 2322;
};

inline constexpr std::array<int, 6> kPositionRegisters = {
    Registers::THUMB_ROT, Registers::THUMB, Registers::INDEX,
    Registers::MIDDLE, Registers::RING, Registers::PINKY};
inline constexpr std::array<int, 6> kSpeedRegisters = {
    Registers::SPEED_THUMB_ROT, Registers::SPEED_THUMB, Registers::SPEED_INDEX,
    Registers::SPEED_MIDDLE, Registers::SPEED_RING, Registers::SPEED_PINKY};
inline constexpr std::array<int, 6> kForceRegisters = {
    Registers::FORCE_THUMB_ROT, Registers::FORCE_THUMB, Registers::FORCE_INDEX,
    Registers::FORCE_MIDDLE, Registers::FORCE_RING, Registers::FORCE_PINKY};

// Low-level writer: computes CAN ids and encodes the 2-byte payload.
class InspireHand {
public:
    explicit InspireHand(SocketCan& can) : can_(can) {}

    // Computes the 29-bit extended CAN id for (register, hand_id, write-flag).
    static std::uint32_t can_id(int reg, int hand_id, bool write) {
        return ((write ? 1u : 0u) << 26) |
               ((static_cast<std::uint32_t>(reg) & 0xFFFu) << 14) |
               (static_cast<std::uint32_t>(hand_id) & 0x3FFFu);
    }

    // Encodes a value into the 2-byte little-endian payload. Returns false if the
    // value is outside the supported [-1, 1000] range.
    static bool encode(int value, std::uint8_t out[2]) {
        std::uint16_t v;
        if (value == kKeepValue) {
            v = 0xFFFF;
        } else if (value < 0 || value > kMaxValue) {
            return false;
        } else {
            v = static_cast<std::uint16_t>(value);
        }
        out[0] = static_cast<std::uint8_t>(v & 0xFF);
        out[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
        return true;
    }

    // Writes one register for one hand. Returns false on a range error (no frame
    // sent); throws std::runtime_error on a socket error.
    bool write_register(int reg, int value, int hand_id) {
        std::uint8_t data[2];
        if (!encode(value, data)) return false;
        can_.write(can_id(reg, hand_id, /*write=*/true), data, 2);
        return true;
    }

private:
    SocketCan& can_;
};

// High-level actions, mirroring Inspire_Robot's InspireAction.
class InspireAction {
public:
    InspireAction(InspireHand& hand, int hand_id) : hand_(hand), hand_id_(hand_id) {}

    void thumb_rot(int v) { write(Registers::THUMB_ROT, v); }
    void thumb(int v) { write(Registers::THUMB, v); }
    void index(int v) { write(Registers::INDEX, v); }
    void middle(int v) { write(Registers::MIDDLE, v); }
    void ring(int v) { write(Registers::RING, v); }
    void pinky(int v) { write(Registers::PINKY, v); }

    // Sets all six finger positions, canonical order. Returns false if the vector
    // is not length 6 (no frames sent).
    bool set_positions(const std::vector<int>& v) {
        if (v.size() != 6) return false;
        for (int i = 0; i < 6; ++i) write(kPositionRegisters[i], v[i]);
        return true;
    }

    // Sets all six finger speeds. Range [0, 1000]; returns false on bad input.
    bool set_speed(const std::vector<int>& v) {
        if (v.size() != 6) return false;
        for (int i = 0; i < 6; ++i) {
            if (v[i] < 0 || v[i] > kMaxValue) return false;
            write(kSpeedRegisters[i], v[i]);
        }
        return true;
    }

    // Sets all six finger forces. Range [0, 1000]; returns false on bad input.
    bool set_force(const std::vector<int>& v) {
        if (v.size() != 6) return false;
        for (int i = 0; i < 6; ++i) {
            if (v[i] < 0 || v[i] > kMaxValue) return false;
            write(kForceRegisters[i], v[i]);
        }
        return true;
    }

    // Stores an action sequence (gesture) under `index` and arms it. Kept as a
    // verbatim port of Inspire_Robot::setActionSequenceData: a 19-entry placeholder
    // (18 x "keep" + 400) is written first, then the two step vectors over the same
    // block, then the 0x90/0xEB checksum pair and the save flag.
    void save_action_sequence(int index, int step_num, const std::vector<int>& steps,
                              const std::vector<int>& steps1) {
        write(Registers::ACTION_SEQ_INDEX, index);
        write(Registers::ACTION_SEQ_STEPNUM, step_num);

        std::vector<int> placeholder(18, kKeepValue);
        placeholder.push_back(400);
        for (std::size_t j = 0; j < placeholder.size(); ++j)
            write(Registers::ACTION_SEQ_STEP0 + static_cast<int>(j) * 2, placeholder[j]);
        for (std::size_t i = 0; i < steps.size(); ++i)
            write(Registers::ACTION_SEQ_STEP0 + static_cast<int>(i) * 2, steps[i]);
        for (std::size_t i = 0; i < steps1.size(); ++i)
            write(Registers::ACTION_SEQ_STEP0 + static_cast<int>(i) * 2, steps1[i]);

        write(Registers::ACTION_SEQ_CHECKDATA1, 144);  // 0x90
        write(Registers::ACTION_SEQ_CHECKDATA2, 235);  // 0xEB
        write(Registers::SAVE_ACTION_SEQ, 1);
    }

    void run_action_sequence(int index) {
        write(Registers::ACTION_SEQ_INDEX, index);
        write(Registers::ACTION_SEQ_RUN, 1);
    }

private:
    void write(int reg, int value) { hand_.write_register(reg, value, hand_id_); }

    InspireHand& hand_;
    int hand_id_;
};

}  // namespace inspire_hand

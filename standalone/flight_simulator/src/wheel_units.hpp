#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

// ICD 1120: physical positions, left/push positive. The model origin is the rear stop.
namespace aviator::wheel {
inline constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
inline constexpr double roll_limit_deg = 50.0, pitch_min_mm = -80.0, pitch_max_mm = 70.0;
inline constexpr double neutral_m = -0.080, minimum_m = -0.150;
inline constexpr int16_t invalid_position = -32768;
inline bool validRoll(double value) { return std::isfinite(value) && std::abs(value) <= roll_limit_deg; }
inline bool validPitch(double value) { return std::isfinite(value) && value >= pitch_min_mm && value <= pitch_max_mm; }
inline double angle(double deg) { return deg * radians_per_degree; }
inline double roll(double rad) { return rad / radians_per_degree; }
inline double displacement(double mm) { return -(mm + 80.0) / 1000.0; }
inline double pitch(double m) { return (neutral_m - m) * 1000.0; }
// SDL positive axes point right/back; invert axes before converting to physical coordinates.
inline double stickRoll(double normalized) { return -normalized * roll_limit_deg; }
inline double stickPitch(double normalized) {
    return normalized >= 0 ? -normalized * 80.0 : -normalized * 70.0;
}
inline int16_t encodeRoll(double deg) {
    if (!validRoll(deg)) throw std::invalid_argument("Roll outside [-50,50] deg");
    return static_cast<int16_t>(std::lround(deg * 32767.0 / 50.0));
}
inline int16_t encodePitch(double mm) {
    if (!validPitch(mm)) throw std::invalid_argument("Pitch outside [-80,70] mm");
    return static_cast<int16_t>(std::lround((mm + 5.0) * 32767.0 / 75.0));
}
inline double decodeRoll(int16_t raw) {
    if (raw == invalid_position) throw std::invalid_argument("Invalid roll position");
    return raw * 50.0 / 32767.0;
}
inline double decodePitch(int16_t raw) {
    if (raw == invalid_position) throw std::invalid_argument("Invalid pitch position");
    return raw * 75.0 / 32767.0 - 5.0;
}
} // namespace aviator::wheel

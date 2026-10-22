#pragma once
#include "backend.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <locale>
#include <optional>

namespace demo {
// One writer per hand worker. Reconnects append to the same CSV files.
class TelemetryCsv {
public:
    TelemetryCsv(const std::string& path, unsigned connection, bool offline)
        : file_(path, std::ios::app), connection_(connection), offline_(offline), began_(nowMs()) {
        file_.imbue(std::locale::classic());
        if (!file_) throw std::runtime_error("无法打开 CSV：" + path);
        if (file_.tellp() == std::streampos(0)) {
            file_ << "timestamp_utc,connection_id,elapsed_ms,offline";
            for (const auto* channel : {"little", "ring", "middle", "index", "thumb_flex", "thumb_rotation"})
                for (const auto* field : {"target_position_raw", "configured_force_g", "configured_speed_raw",
                                         "actual_position_raw", "actual_force_g", "actual_current_mA", "error_code"})
                    file_ << ',' << channel << '_' << field;
            file_ << '\n';
            flush();
        }
    }

    void append(const ArmState& state, const std::optional<rh56::Values>& target) {
        const auto wall = std::chrono::system_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall.time_since_epoch()).count();
        const auto seconds = std::chrono::system_clock::to_time_t(wall);
        std::tm utc{};
        gmtime_r(&seconds, &utc);
        file_ << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
              << std::setfill('0') << std::setw(3) << ms % 1000 << "Z,"
              << connection_ << ',' << nowMs() - began_ << ',' << (offline_ ? 1 : 0);
        for (size_t j = 0; j < state.angles.size(); ++j) {
            file_ << ',';
            if (target) file_ << (*target)[j];
            file_ << ',' << state.limits[j] << ',' << state.speeds[j]
                  << ',' << state.angles[j] << ',' << state.forces[j]
                  << ',' << state.currents[j] << ',' << state.errors[j];
        }
        file_ << '\n';
        flush();
    }
private:
    void flush() {
        file_.flush();
        if (!file_) throw std::runtime_error("CSV 写入或刷新失败（请检查磁盘空间和目录权限）");
    }
    std::ofstream file_;
    unsigned connection_;
    bool offline_;
    long long began_;
};
} // namespace demo

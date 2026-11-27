#include "rs422_display.hpp"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace flight_simulator {
std::string hexBytes(const rs422::Bytes& bytes, size_t start, size_t count) {
    if (start >= bytes.size()) return {};
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    const auto end = start + std::min(count, bytes.size() - start);
    for (size_t i = start; i < end; ++i) {
        if (i != start) out << ' ';
        out << std::setw(2) << unsigned(bytes[i]);
    }
    return out.str();
}
const char* receptionStatus(const ReceptionSnapshot& s, uint64_t now, uint64_t timeout) {
    if (s.frame.empty()) return "WAITING";
    return s.connected && s.current && now >= s.received_us && now - s.received_us < timeout
               ? "LIVE" : "STALE";
}
std::vector<UplinkField> describeUplink(const rs422::Bytes& frame) {
    rs422::Uplink f;
    if (!rs422::decode(frame, f))
        throw std::invalid_argument("display requires a valid 64-byte RS422 uplink");
    std::vector<UplinkField> fields;
    const auto add = [&](size_t offset, size_t length, const char* name, std::string value) {
        fields.push_back({std::to_string(offset) + (length > 1 ? "–" + std::to_string(offset + length - 1) : ""),
                          name, hexBytes(frame, offset, std::min(length, size_t(4))) + (length > 4 ? " …" : ""),
                          std::move(value)});
    };
    const auto position = [](int16_t raw, bool pitch) {
        if (raw == rs422::invalid_position) return std::string("-32768 → 无效 / unknown");
        std::ostringstream value;
        value << raw << " → " << std::fixed << std::setprecision(6)
              << (pitch ? rs422::decodePitch(raw) : rs422::decodeRoll(raw)) << (pitch ? " mm" : " deg");
        return value.str();
    };
    const auto fault = [](uint16_t code, bool history = false) {
        if (!code) return std::string(history ? "0 → 本次网关运行尚无上一条记录" : "0 → 无故障");
        const auto* name = rs422::faultName(code);
        return std::to_string(code) + (*name ? std::string(" → ") + name : "");
    };
    add(0, 2, "帧头", "EB 90 · 匹配");
    add(2, 1, "帧计数器", std::to_string(f.counter) + " · 0–255 循环");
    add(3, 1, "选中下行通道", f.selected == 0 ? "A / 主通道" : f.selected == 1 ? "B / 备通道" : "无可选通道");
    add(4, 1, "接收飞控通信状态", std::string("A: ") + (f.communication & 1 ? "故障" : "正常") +
        " / B: " + (f.communication & 2 ? "故障" : "正常"));
    add(5, 1, "操作回绕", f.operation ? rs422::operationName(f.operation) : "空闲 / 无新操作");
    add(6, 2, "滚转目标回绕", position(f.roll_echo, false));
    add(8, 2, "俯仰目标回绕", position(f.pitch_echo, true));
    add(10, 1, "机器人运行状态", f.state == rs422::not_started ? "未启动 / NOT_STARTED" : rs422::stateName(f.state));
    add(11, 1, "后退到位", f.rear == 0 ? "未到位" : f.rear == 1 ? "到位" : "未知 / unknown");
    add(12, 1, "应急手刹（X1）", f.brake == 0 ? "未触发（X1=1）" : f.brake == 1 ? "已触发（X1=0）"
                                                                                 : "X1 信号无效 / 未接收 / 过期");
    add(13, 2, "相机滚转反馈", position(f.roll, false));
    add(15, 2, "相机俯仰反馈", position(f.pitch, true));
    add(17, 2, "当前故障码", fault(f.fault));
    add(19, 2, "历史故障码", fault(f.last_fault, true));
    const char* severity = f.severity == 0 ? "正常" : f.severity == 0x10 ? "注意" : f.severity == 0x20 ? "提示"
                           : f.severity == 0x30 ? "告警" : "危险";
    add(21, 1, "故障等级", severity);
    add(22, 2, "软件版本", std::to_string(f.version >> 8) + "." + std::to_string(f.version & 255) + " · PATCH 不上串口");
    add(24, 2, "软件短校验码", "0x" + hexBytes({uint8_t(f.checksum >> 8)}, 0, 1) + hexBytes({uint8_t(f.checksum)}, 0, 1));
    add(26, 2, "关节 FK 滚转反馈", position(f.joint_roll, false));
    add(28, 2, "关节 FK 俯仰反馈", position(f.joint_pitch, true));
    add(30, 33, "保留位（33 字节）", std::any_of(frame.begin() + 30, frame.begin() + 63, [](uint8_t v) { return v != 0; })
        ? "含非零字节 · 原样保留，见上方 HEX" : "全零");
    add(63, 1, "帧累加校验", "通过 · sum(bytes[2..62]) & FF = " + hexBytes(frame, 63, 1));
    return fields;
}
} // namespace flight_simulator

#include "input_help.hpp"
#include "clock.hpp"
#include <iostream>
#include <stdexcept>

using namespace flight_simulator;
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
rs422::Bytes fixture() {
    // Independent 64-byte uplink: signed little-endian physical positions,
    // emergency stop, a nonzero reserved byte, and distinct software/frame checksums.
    rs422::Bytes bytes(64, 0);
    bytes[0] = 0xeb; bytes[1] = 0x90; bytes[2] = 42;
    bytes[3] = 1; bytes[4] = 2; bytes[5] = 0x60;
    bytes[6] = 0x01; bytes[7] = 0x80; // -32767 = -50 deg
    bytes[8] = 0x88; bytes[9] = 0x08; // 2184 ~= 0 mm
    bytes[10] = 0xff; bytes[11] = 1; bytes[12] = 0xff;
    bytes[13] = 0xff; bytes[14] = 0x7f;
    bytes[15] = 0x01; bytes[16] = 0x80;
    bytes[17] = 0x09; bytes[18] = 0x20; // 8201
    bytes[19] = 0x15; bytes[20] = 0x05; // 1301
    bytes[21] = 0x30;
    bytes[22] = 1; bytes[23] = 2; bytes[24] = 0xcd; bytes[25] = 0xab;
    bytes[27] = 0x80; // invalid joint roll
    bytes[28] = 0xff; bytes[29] = 0x7f;
    bytes[30] = 0xa5;
    unsigned sum = 0;
    for (unsigned i = 2; i < 63; ++i) sum += bytes[i];
    bytes[63] = uint8_t(sum);
    return bytes;
}
int main(int argc, char** argv) try {
    auto raw = fixture();
    const auto fields = describeUplink(raw);
    check(fields.size() == 21, "missing uplink fields");
    check(fields[2].value == "B / 备通道" && fields[3].value == "A: 正常 / B: 故障", "channel fields");
    check(fields[4].value == "reset_error", "operation echo");
    check(fields[5].value == "-32767 → -50.000000 deg", "roll echo units");
    check(fields[6].hex == "88 08" && fields[6].value == "2184 → -0.001068 mm", "pitch zero offset");
    check(fields[7].value == "EMERGENCY_STOP", "FF emergency stop");
    check(fields[9].value == "X1 信号无效 / 未接收 / 过期", "invalid X1 is not an asserted trigger");
    check(fields[10].value == "32767 → 50.000000 deg", "camera roll");
    check(fields[11].value == "-32767 → -80.000000 mm", "camera pitch");
    check(fields[12].value == "8201" && fields[13].value == "1301", "fault endianness");
    check(fields[15].value.find("2.1") == 0 && fields[16].value == "0xABCD", "software identity");
    check(fields[17].value.find("无效") != std::string::npos, "unknown axis must not be zero");
    check(fields[18].value == "32767 → 70.000000 mm", "FK pitch");
    check(fields[19].offset == "30–62" && fields[19].hex == "A5 00 00 00 …", "preserve reserved bytes");
    check(fields[20].offset == "63" && fields[20].value.find("通过") == 0, "frame checksum");
    rs422::Uplink startup;
    const auto startup_fields = describeUplink(rs422::encode(startup));
    check(startup_fields[7].value == "未启动 / NOT_STARTED", "FE is normal startup waiting");
    check(startup_fields[12].value == "0 → 无故障" && startup_fields[14].value == "正常", "startup must be healthy");
    check(startup_fields[13].value == "0 → 本次网关运行尚无上一条记录", "history zero is no previous record");
    for (const uint8_t trigger : {0, 1}) {
        auto feedback = startup;
        feedback.brake = trigger;
        feedback.state = 0xff; // Current switch position is independent of sticky emergency state.
        const auto bytes = rs422::encode(feedback);
        check(bytes[12] == trigger && bytes[11] == 0xff, "handbrake byte offset");
        const auto described = describeUplink(bytes);
        check(described[9].offset == "12" && described[9].value ==
                  (trigger ? "已触发（X1=0）" : "未触发（X1=1）"), "X1 polarity must not be inverted twice");
    }
    for (const uint16_t code : {7202, 7203, 7204, 7205, 7206, 8212}) {
        auto alarm = startup;
        alarm.state = 0x10; alarm.fault = alarm.last_fault = code; alarm.severity = 0x30;
        const auto described = describeUplink(rs422::encode(alarm));
        check(described[12].value.find(std::to_string(code) + " → Core ") == 0, "alarm description missing");
        check(described[13].value == described[12].value && described[14].value == "告警", "alarm history/severity");
    }
    for (auto invalid : {rs422::Bytes(32, 0), raw}) {
        invalid.back() ^= 1;
        bool rejected = false;
        try { describeUplink(invalid); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "display must reject invalid or downlink data");
    }
    ReceptionSnapshot s;
    check(std::string(receptionStatus(s, 100000, 50000)) == "WAITING", "initial status");
    s.frame = raw; s.current = s.connected = true; s.received_us = 100000;
    check(std::string(receptionStatus(s, 149999, 50000)) == "LIVE", "live status");
    check(std::string(receptionStatus(s, 150000, 50000)) == "STALE", "timeout boundary");
    s.current = false;
    check(std::string(receptionStatus(s, 100001, 50000)) == "STALE", "reconnect must not renew data");

    check(SDL_Init(SDL_INIT_VIDEO) == 0, "SDL init");
    auto* window = SDL_CreateWindow("RS422 display test", 0, 0, InputHelp::width, InputHelp::height, 0);
    check(window != nullptr, "SDL window");
    {
        Config config;
        InputHelp ui(config);
        std::array<ReceptionSnapshot, 2> snapshots;
        s.current = true; s.received_us = monotonic_us();
        s.accepted_frames = 512; s.rx_bytes = 32768;
        snapshots[0] = s;
        ui.feedback(snapshots);
        check(ui.viewKey(SDL_SCANCODE_F2), "select channel A");
        ui.show(window, true, true, true);
        if (argc == 2) check(SDL_SaveBMP(SDL_GetWindowSurface(window), argv[1]) == 0, "save preview");
        const auto copied = [&] {
            check(ui.viewKey(SDL_SCANCODE_F5), "copy HEX key");
            char* text = SDL_GetClipboardText();
            std::string result = text ? text : "";
            SDL_free(text);
            return result;
        };
        check(copied() == hexBytes(raw), "copy exact displayed frame");
        check(ui.viewKey(SDL_SCANCODE_F4), "freeze");
        snapshots[0].frame[2]++;
        snapshots[0].frame.back()++;
        ui.feedback(snapshots);
        ui.show(window, true, true, true);
        check(copied() == hexBytes(raw), "frozen snapshot changed");
        check(ui.viewKey(SDL_SCANCODE_F4), "resume");
        ui.show(window, true, true, true);
        check(copied() == hexBytes(snapshots[0].frame), "resume did not show latest frame");
        for (bool alarm : {false, true}) {
            auto feedback = startup;
            feedback.brake = alarm ? 1 : 0;
            if (alarm) {
                feedback.state = 0xff; feedback.fault = 8212; feedback.last_fault = 7202; feedback.severity = 0x30;
            }
            snapshots[0].frame = rs422::encode(feedback);
            snapshots[0].received_us = monotonic_us();
            ui.feedback(snapshots);
            ui.show(window, true, true, true);
            check(copied() == hexBytes(snapshots[0].frame), "1127 monitor frame differs");
            if (argc == 2) {
                const auto path = std::string(argv[1]) + (alarm ? ".alarm.bmp" : ".startup.bmp");
                check(SDL_SaveBMP(SDL_GetWindowSurface(window), path.c_str()) == 0, "save 1127 preview");
            }
        }
        check(ui.viewKey(SDL_SCANCODE_F3), "select waiting channel B");
        ui.show(window, true, false, true);
        check(ui.viewKey(SDL_SCANCODE_F1), "return to input guide");
        ui.show(window, true, false, true);
        check(!ui.viewKey(SDL_SCANCODE_1), "monitor consumed operation key");
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "PASS: 64B uplink fields, raw capture, stale state, channel switching, freeze and clipboard\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}

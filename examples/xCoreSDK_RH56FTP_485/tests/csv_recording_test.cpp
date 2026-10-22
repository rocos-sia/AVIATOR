#include "backend.hpp"
#include "telemetry_csv.hpp"
#include <rokae/robot.h>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class F> void until(F condition) {
    const auto began = demo::nowMs();
    while (!condition()) {
        if (demo::nowMs() - began > 4000) throw std::runtime_error("CSV test timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
struct Table {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    std::string value(const std::vector<std::string>& row, const std::string& column) const {
        const auto it = std::find(header.begin(), header.end(), column);
        require(it != header.end(), "CSV column missing");
        return row.at(size_t(it - header.begin()));
    }
};
Table read(const char* path) {
    std::ifstream file(path);
    require(bool(file), "CSV missing");
    Table result;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream fields(line);
        std::vector<std::string> row;
        std::string field;
        while (std::getline(fields, field, ',')) row.push_back(field);
        if (result.header.empty()) result.header = row;
        else {
            require(row.size() == result.header.size(), "CSV column count mismatch");
            require(row != result.header, "Duplicate CSV header on reconnect");
            result.rows.push_back(row);
        }
    }
    require(result.header.size() == 46, "Expected metadata and seven fields per channel");
    return result;
}
void stop(demo::Backend& backend) {
    backend.stop(); until([&] { return !backend.snapshot().active; });
}
void recording() {
    fake_sdk::reset();
    demo::Backend backend;
    demo::Config config; config.pollMs = 100;
    backend.start(config);
    until([&] { return backend.snapshot().ready; });
    std::array<demo::Command, 2> commands;
    commands[0].angles = {10,20,30,40,50,60}; commands[0].speeds.fill(123); commands[0].limits.fill(456);
    commands[1].angles = {60,50,40,30,20,10}; commands[1].speeds.fill(321); commands[1].limits.fill(654);
    require(backend.submitBoth(commands), "Dual submit rejected");
    until([&] { const auto s = backend.snapshot(); return s.arms[0].angles == commands[0].angles &&
        s.arms[1].angles == commands[1].angles; });
    // Freeze the failing side after a complete record; healthy side keeps sampling.
    fake_sdk::arms[1].failReadAddress = rh56::currentAct;
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Recovering; });
    const auto failedCount = read("right_hand.csv").rows.size();
    const int attempts = fake_sdk::arms[1].attempts;
    until([&] { return fake_sdk::arms[1].attempts > attempts + 8; });
    require(read("right_hand.csv").rows.size() == failedCount, "Failed current read recorded stale feedback");
    stop(backend);
    const auto left = read("left_hand.csv"), right = read("right_hand.csv");
    require(left.rows.size() > right.rows.size(), "Healthy side stopped recording");
    for (int i = 0; i < 2; ++i) {
        const auto& table = i == 0 ? left : right;
        require(table.rows.size() >= 2, "Missing initial/command rows");
        const auto& first = table.rows.front(); const auto& last = table.rows.back();
        require(table.value(first, "little_target_position_raw").empty(), "Invented startup target");
        require(table.value(first, "little_configured_speed_raw") == "200", "Missing initial speed readback");
        require(table.value(first, "little_configured_force_g") == "300", "Missing initial force readback");
        const auto timestamp = table.value(last, "timestamp_utc");
        require(timestamp.size() == 24 && timestamp.back() == 'Z', "Missing UTC millisecond timestamp");
        require(table.value(last, "offline") == "0", "Incorrect real-mode marker");
        require(table.value(last, "little_actual_current_mA") == "-128", "Lost signed current");
        require(table.value(last, "little_actual_force_g") == "-20", "Lost signed force");
        require(table.value(last, "thumb_rotation_actual_current_mA") == std::to_string(2000+i), "Mixed hand feedback");
        const char* channels[] = {"little", "ring", "middle", "index", "thumb_flex", "thumb_rotation"};
        for (int j = 0; j < 6; ++j) {
            const std::string channel = channels[j];
            require(table.value(last, channel + "_target_position_raw") == std::to_string(commands[i].angles[j]), "Wrong target channel");
            require(table.value(last, channel + "_actual_position_raw") == std::to_string(commands[i].angles[j]), "Wrong feedback channel");
            require(table.value(last, channel + "_configured_speed_raw") == std::to_string(commands[i].speeds[j]), "Wrong configured speed");
            require(table.value(last, channel + "_configured_force_g") == std::to_string(commands[i].limits[j]), "Wrong configured force");
        }
    }
    fake_sdk::reset();
    config.offline = true;
    backend.start(config); until([&] { return backend.snapshot().ready; }); stop(backend);
    const auto appended = read("left_hand.csv");
    require(appended.rows.size() > left.rows.size(), "Reconnect overwrote previous data");
    require(appended.rows.front() == left.rows.front(), "Reconnect changed existing row");
    require(appended.value(appended.rows.back(), "connection_id") == "2", "Missing reconnect marker");
    require(appended.value(appended.rows.back(), "offline") == "1", "Missing simulation marker");
    require(appended.value(appended.rows.back(), "little_target_position_raw").empty(), "Reconnect kept previous target");
}
void partialWrites() {
    fake_sdk::reset();
    demo::Backend backend;
    demo::Config config; config.pollMs = 100;
    backend.start(config); until([&] { return backend.snapshot().ready; });
    demo::Command command; command.angles.fill(111);
    require(backend.submit(1, command), "Initial command rejected");
    until([&] { return backend.snapshot().arms[0].angles[0] == 111; });
    fake_sdk::arms[0].loseWriteAck = true;
    command.speeds.fill(789); command.angles.fill(222);
    require(backend.submit(1, command), "Partial command rejected");
    until([&] { return backend.snapshot().arms[0].phase == demo::Phase::Recovering; });
    until([&] { return backend.snapshot().ready; });
    stop(backend);
    auto table = read("left_hand.csv");
    require(table.value(table.rows.back(), "little_target_position_raw") == "111", "Partial speed write changed target");
    require(table.value(table.rows.back(), "little_configured_speed_raw") == "789", "Partial write not reconciled from readback");

    backend.start(config); until([&] { return backend.snapshot().ready; });
    fake_sdk::arms[0].loseAckAddress = rh56::angleSet;
    require(backend.submit(1, command), "Uncertain angle command rejected");
    until([&] { return backend.snapshot().arms[0].phase == demo::Phase::Recovering; });
    until([&] { return backend.snapshot().ready; });
    fake_sdk::arms[0].errorCode = 1;
    until([&] { return backend.snapshot().arms[0].errors[0] == 1; });
    stop(backend);
    table = read("left_hand.csv");
    require(table.value(table.rows.back(), "little_target_position_raw").empty(), "Lost ACK falsely marked target confirmed");
    require(table.value(table.rows.back(), "little_actual_position_raw") == "222", "Missing actual movement after lost ACK");
    require(table.value(table.rows.back(), "little_error_code") == "1", "Valid actuator fault sample not recorded");
}
void fileErrorsAndInit() {
    require(QDir().mkdir("left_hand.csv"), "Cannot create unwritable CSV fixture");
    demo::Backend backend;
    demo::Config config; config.offline = true; config.pollMs = 100;
    backend.start(config); until([&] { return backend.snapshot().ready; }); stop(backend);
    const auto logs = backend.takeLogs();
    require(std::any_of(logs.begin(), logs.end(), [](const std::string& s) {
        return s.find("CSV 记录已禁用") != std::string::npos;
    }), "CSV open error hidden");
    require(!read("right_hand.csv").rows.empty(), "CSV open error affected healthy side");
    bool rejected = false;
    try { demo::TelemetryCsv full("/dev/full", 1, false); full.append({}, std::nullopt); }
    catch (const std::exception&) { rejected = true; }
    require(rejected, "CSV disk-full error hidden");
    require(QDir().mkdir("init") && QDir::setCurrent("init"), "Cannot create init fixture");
    config.init485 = true;
    backend.start(config); until([&] { return !backend.snapshot().active; });
    require(!QFile::exists("left_hand.csv") && !QFile::exists("right_hand.csv"), "Init-only session created telemetry");
}
}
int main() {
    QTemporaryDir logs;
    const auto cwd = QDir::currentPath();
    try {
        require(logs.isValid() && QDir::setCurrent(logs.path()), "Cannot create CSV test directory");
        recording();
        require(QDir().mkdir("partial") && QDir::setCurrent("partial"), "Cannot create partial fixture");
        partialWrites();
        require(QDir().mkdir("errors") && QDir::setCurrent("errors"), "Cannot create error fixture");
        fileErrorsAndInit();
        QDir::setCurrent(cwd);
        std::cout << "CSV channel mapping, signed current, failures, reconnect, partial writes and file errors passed\n";
        return 0;
    } catch (const std::exception& e) {
        QDir::setCurrent(cwd); std::cerr << e.what() << '\n'; return 1;
    }
}

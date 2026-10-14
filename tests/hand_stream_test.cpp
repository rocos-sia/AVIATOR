#include "HandControl.hpp"
#include "HandRuntime.hpp"
#include "runtime.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

using namespace aviator;
using namespace aviator::hand;
namespace {
void check(bool ok, const std::string& why) {
    if (!ok) throw std::runtime_error(why);
}
Bytes reply(const Bytes& tx, const Bytes& payload) {
    Bytes rx{0x90, 0xeb, tx[2], uint8_t(payload.size() + 3), tx[4], tx[5], tx[6]};
    for (auto value : payload) rx.push_back(value);
    unsigned sum = 0;
    for (size_t i = 2; i < rx.size(); ++i) sum += rx[i];
    rx.push_back(uint8_t(sum));
    return rx;
}
}
enum class Failure { stalled_write, position_read, telemetry_read };
void stream(Failure failure) {
    const auto path = std::filesystem::temp_directory_path() / ("hand-stream-" + new_session_id());
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::filesystem::remove(path); }
    } cleanup{path};
    std::ofstream(path) << "core_hand:\n  enabled: true\n  publisher_id: rh56ftp_hand\n"
        "  completion_timeout_ms: 1000\n  feedback_timeout_ms: 500\n"
        "  close: {left: [0.5, 0, 0, 0, 0, 0], right: [0.5, 0, 0, 0, 0, 0]}\n";
    HandControl core;
    core.configure(path);
    std::atomic<bool> inject{false}, injected{false}, protective_stop{false};
    std::atomic<unsigned> settings_writes{0};
    std::atomic<unsigned> intermediate[2]{};
    auto device = [&](int side) {
        auto position = std::make_shared<Values>(open_pose);
        return rs485Device(1, [&, side, position](const Bytes& tx, size_t count) {
            const int address = tx[5] | (tx[6] << 8);
            const bool angle = tx[4] == 0x12 && address >= angle_set && address < angle_set + 12;
            // One stalled write while Core keeps publishing changing targets.
            // The real per-hand transaction queue and feedback polls are active.
            const bool read_failure = tx[4] == 0x11 &&
                ((failure == Failure::position_read && side == 0 && address == 1546) ||
                 (failure == Failure::telemetry_read && side == 1 && address == 1582));
            const bool trigger = ((failure == Failure::stalled_write && side == 0 && angle) ||
                                  read_failure) && inject.exchange(false);
            const bool stall = trigger && failure == Failure::stalled_write;
            if (trigger) injected = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(stall ? 140 : 35));
            if (trigger && read_failure) throw std::runtime_error("injected SDK read failure");
            if (tx[4] == 0x12) {
                if (!angle) ++settings_writes;
                if (angle) {
                    for (size_t i = 7; i + 1 < tx.size() - 1; i += 2) {
                        const int channel = 5 - ((address - angle_set) / 2 + (i - 7) / 2);
                        const int value = tx[i] | (tx[i + 1] << 8);
                        if (value == 65535) { protective_stop = true; continue; }
                        (*position)[channel] = value;
                        if (channel == 5 && value > 0 && value < 1000) ++intermediate[side];
                    }
                }
                return reply(tx, {1});
            }
            Bytes data(count - 8);
            if (address == 1546)
                for (int i = 0; i < 6; ++i) {
                    data[2*i] = uint8_t((*position)[5-i]);
                    data[2*i+1] = uint8_t((*position)[5-i] >> 8);
                }
            return reply(tx, data);
        });
    };
    Runtime runtime(Config{}, {device(0), device(1)});
    auto deadline = monotonic_us() + 3000000;
    while (runtime.state().body["hands"]["right"]["commanded_drive_position_normalized"] !=
           Json({.5, 1, 1, 1, 1, 1})) {
        check(monotonic_us() < deadline, "startup did not finish");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto session = new_session_id();
    const unsigned configured = settings_writes;
    const auto began = monotonic_us();
    core.beginApproach(began);
    bool armed = false, complete = false;
    uint64_t next_feedback = 0;
    while (monotonic_us() - began < 4000000) {
        auto now = monotonic_us();
        const double progress = std::min(1., (now - began) / 2000000.);
        if (now >= next_feedback) {
            auto state = runtime.state();
            now = monotonic_us();
            core.receive(state, now, session);
            next_feedback = now + 60000;
        }
        check(core.fault(now).empty(), "Core stopped the grasp: " + core.fault(now));
        core.approachProgress({progress, progress}, now);
        if (!armed && now - began > 400000) { inject = true; armed = true; }
        if (auto message = core.command(now, now, session)) runtime.receive(*message);
        check(!protective_stop, "expired trajectory sample triggered blocking protective writes despite fresh targets");
        check(settings_writes == configured, "transient read/write failure rewrote speed/force/mode");
        const auto state = runtime.state().body;
        if (progress == 1 && core.complete(now) &&
            state["hands"]["left"]["drive_position_normalized"] == Json({.5, 0, 0, 0, 0, 0}) &&
            state["hands"]["right"]["drive_position_normalized"] == Json({.5, 0, 0, 0, 0, 0})) {
            complete = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(injected && complete && intermediate[0] > 2 && intermediate[1] > 2,
          "synchronized stream did not recover and reach both endpoints");
    // Removing the fresh stream must still trigger the normal 100 ms watchdog.
    deadline = monotonic_us() + 1500000;
    while (true) {
        const auto state = runtime.state().body;
        if (state["command_valid"] == false &&
            state["hands"]["left"]["drive_position_normalized"] == Json({.5, 1, 1, 1, 1, 1}) &&
            state["hands"]["right"]["drive_position_normalized"] == Json({.5, 1, 1, 1, 1, 1})) break;
        check(monotonic_us() < deadline, "stopping the stream failed to apply the safe pose");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
int main() try {
    stream(Failure::stalled_write);
    stream(Failure::position_read);
    stream(Failure::telemetry_read);
    std::cout << "PASS Core + RS485 stream survives a stalled write and transient position/telemetry read failures\n";
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}

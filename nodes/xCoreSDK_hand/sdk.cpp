// Transport only. No robot motion, power, RT mode, or disconnect operations.
#include <rokae/robot.h>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
thread_local std::string error;
void check(const std::error_code& ec, const char* operation) {
    if (ec) throw std::runtime_error(std::string(operation) + ": " + ec.message() +
        " [" + ec.category().name() + ":" + std::to_string(ec.value()) + "]");
}
}
extern "C" const char* hand_sdk_error() { return error.c_str(); }
extern "C" void* hand_sdk_connect(const char* host, int timeout_ms) noexcept {
    try {
        if (!host || !*host || timeout_ms < 50 || timeout_ms > 10000)
            throw std::runtime_error("Invalid SDK host/RS485 timeout");
        // Intentionally process-owned, including partial connection failures.
        // The vendor destructor/disconnect may stop arm motion. Never delete
        // this object: the OS reclaims its memory/sockets on process termination.
        auto* robot = new rokae::ArRobot;
        robot->connectToRobot(host); // No local RT endpoint or RT subscription.
        std::error_code ec;
        robot->setxPanelRS485Overtime(timeout_ms, ec);
        check(ec, "setxPanelRS485Overtime");
        return robot;
    } catch (const std::exception& e) { error = e.what(); return nullptr; }
    catch (...) { error = "Unknown SDK connection failure"; return nullptr; }
}
extern "C" int hand_sdk_transfer(void* handle, const uint8_t* tx, int size,
                                  uint8_t* output, int count) noexcept {
    try {
        if (!handle || !tx || !output || size < 1 || size > 64 || count < 1 || count > 64)
            throw std::runtime_error("Invalid SDK transfer (1..64 bytes required)");
        std::vector<uint8_t> input(tx, tx + size), result(count);
        std::error_code ec;
        static_cast<rokae::ArRobot*>(handle)->XPRS485SendData(size, count, input, result, ec);
        check(ec, "XPRS485SendData");
        if (result.size() != size_t(count)) throw std::runtime_error("Short RS485 response");
        std::copy(result.begin(), result.end(), output);
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
    catch (...) { error = "Unknown SDK transfer failure"; return -1; }
}

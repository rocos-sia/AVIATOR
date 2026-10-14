#pragma once
// Test-only SDK substitute. The fault-isolation test does not link the real SDK.
#include "protocol.hpp"
#include <atomic>
#include <chrono>
#include <string>
#include <system_error>
#include <thread>

namespace fake_sdk {
struct Arm {
    std::atomic<bool> failRead{false}, stall{false}, enteredStall{false}, failCleanup{false};
    std::atomic<int> reads{0}, writes{0}, powerOff{0}, disconnects{0};
};
inline Arm arms[2];
inline void reset() {
    for (auto& a : arms) {
        a.failRead = false; a.stall = false; a.enteredStall = false; a.failCleanup = false;
        a.reads = 0; a.writes = 0; a.powerOff = 0; a.disconnects = 0;
    }
}
// Declared after the backend/window in each test, so failures also release blocked calls.
struct ReleaseStalls {
    ~ReleaseStalls() { for (auto& a : arms) a.stall = false; }
};
}
namespace rokae {
struct xPanelCommOpt {
    enum class Rs485Baud { baud115200 };
    enum class CheckBit { none };
    enum class StopBit { stopbit1 };
};
struct Info { std::string type = "Fake AR", version = "test"; int joint_num = 7; };
class ArRobot {
    int side_ = 0;
    rh56::Values angles_{1000,1000,1000,1000,1000,1000};
    rh56::Values speeds_{200,200,200,200,200,200};
    rh56::Values limits_{300,300,300,300,300,300};
public:
    void connectToRobot(const std::string& ip) {
        if (ip == "192.168.1.160") side_ = 0;
        else if (ip == "192.168.2.160") side_ = 1;
        else throw std::runtime_error("Unexpected test endpoint");
    }
    Info robotInfo(std::error_code& ec) { ec.clear(); return {}; }
    std::string sdkVersion() { return "fake"; }
    void xPanelRS485Init(xPanelCommOpt::Rs485Baud, xPanelCommOpt::CheckBit,
                        xPanelCommOpt::StopBit, std::error_code& ec) { ec.clear(); }
    void setxPanelRS485Overtime(int, std::error_code& ec) { ec.clear(); }
    void setPowerState(bool on, std::error_code& ec) {
        ec.clear();
        if (!on) {
            ++fake_sdk::arms[side_].powerOff;
            if (fake_sdk::arms[side_].failCleanup) ec = std::make_error_code(std::errc::io_error);
        }
    }
    void disconnectFromRobot(std::error_code& ec) {
        ++fake_sdk::arms[side_].disconnects;
        ec.clear();
    }
    void XPRS485SendData(int send, int receive, const rh56::Bytes& tx, rh56::Bytes& rx, std::error_code& ec) {
        if (send != int(tx.size()) || receive != int(rx.size()) || tx.size() < 9)
            throw std::runtime_error("Wrong SDK buffer lengths");
        auto& a = fake_sdk::arms[side_];
        ec.clear();
        if (a.stall) {
            a.enteredStall = true;
            while (a.stall) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (a.failRead) {
            rx.clear(); ec = std::make_error_code(std::errc::timed_out); return;
        }
        const int address = tx[5] | (int(tx[6]) << 8);
        rh56::Bytes data;
        if (tx[4] == 0x12) {
            if (tx.size() != 20 || receive != 9) throw std::runtime_error("Wrong write lengths");
            rh56::Values* values = address == rh56::angleSet ? &angles_ :
                address == rh56::speedSet ? &speeds_ : address == rh56::forceSet ? &limits_ : nullptr;
            if (!values) throw std::runtime_error("Unexpected write address");
            for (int i = 0; i < 6; ++i) (*values)[i] = tx[7 + 2*i] | (int(tx[8 + 2*i]) << 8);
            ++a.writes;
            data = {1};
        } else {
            ++a.reads;
            if (address == rh56::errorAct) data.assign(6, 0);
            else {
                const rh56::Values values = address == rh56::angleAct ? angles_ :
                    address == rh56::speedSet ? speeds_ : address == rh56::forceSet ? limits_ : rh56::Values{};
                for (int value : values) { data.push_back(uint8_t(value)); data.push_back(uint8_t(value >> 8)); }
            }
        }
        rx = {0x90, 0xEB, tx[2], uint8_t(data.size() + 3), tx[4], tx[5], tx[6]};
        rx.insert(rx.end(), data.begin(), data.end());
        rx.push_back(rh56::checksum(rx));
        if (int(rx.size()) != receive) throw std::runtime_error("Wrong test reply length");
    }
};
}

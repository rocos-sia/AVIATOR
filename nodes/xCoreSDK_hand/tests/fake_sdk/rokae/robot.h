#pragma once
#include <cstdint>
#include <cstdlib>
#include <string>
#include <system_error>
#include <vector>
namespace rokae {
class ArRobot {
public:
    // A destructor call is an error: the real SDK may stop the arm here.
    ~ArRobot() { std::abort(); }
    void connectToRobot(const std::string&) {}
    void setxPanelRS485Overtime(int, std::error_code& ec) { ec.clear(); }
    void XPRS485SendData(int, int count, const std::vector<uint8_t>& tx,
                        std::vector<uint8_t>& rx, std::error_code& ec) {
        rx = {0x90, 0xeb, tx[2], uint8_t(count - 5), tx[4], tx[5], tx[6]};
        rx.resize(count, 0);
        if (tx[4] == 0x12) rx[7] = 1;
        for (int i = 2; i < count - 1; ++i) rx.back() += rx[i];
        ec.clear();
    }
};
}

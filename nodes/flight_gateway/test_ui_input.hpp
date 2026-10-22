#pragma once
#include "gateway.hpp"

namespace flight_gateway {
// Explicit local test input. SOCK_SEQPACKET preserves command boundaries and
// keeps all socket IO nonblocking on the normal gateway owner thread.
class TestUiInput {
public:
    explicit TestUiInput(const std::string& path);
    ~TestUiInput();
    TestUiInput(const TestUiInput&) = delete;
    TestUiInput& operator=(const TestUiInput&) = delete;
    void poll(JoystickSample&, std::vector<unsigned>& pressed, const nlohmann::json& state,
              bool ready, bool busy);
    void result(const std::string& status, const std::string& detail = "");
    void report(const nlohmann::json& state, bool fresh, bool ready);
private:
    void disconnect();
    int server_ = -1, client_ = -1;
    std::string path_, command_id_;
    std::uint64_t checked_ = 0, last_command_ = 0;
    double angle_ = 0, displacement_ = -.085;
    nlohmann::json result_ = nullptr;
};
} // namespace flight_gateway

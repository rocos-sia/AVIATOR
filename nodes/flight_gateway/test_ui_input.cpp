#include "test_ui_input.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace flight_gateway {
namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
constexpr std::uint64_t lease_us = 100000;
}
TestUiInput::TestUiInput(const std::string& path) : path_(path) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    require(!path.empty() && path.front() == '/' && path.size() < sizeof(address.sun_path),
            "Test UI socket must be a short absolute path in a private directory");
    std::strcpy(address.sun_path, path.c_str());
    server_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    require(server_ >= 0, "Cannot create test UI socket");
    // Never remove an existing endpoint or take over another running UI.
    if (bind(server_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        close(server_); server_ = -1;
        throw std::runtime_error("Cannot bind test UI socket: " + path);
    }
    if (chmod(path.c_str(), 0600) != 0 || listen(server_, 1) != 0) {
        close(server_); server_ = -1; unlink(path.c_str());
        throw std::runtime_error("Cannot prepare test UI socket");
    }
}
TestUiInput::~TestUiInput() {
    disconnect();
    if (server_ >= 0) { close(server_); unlink(path_.c_str()); }
}
void TestUiInput::disconnect() {
    if (client_ >= 0) close(client_);
    client_ = -1; checked_ = 0;
}
void TestUiInput::result(const std::string& status, const std::string& detail) {
    result_ = {{"id", command_id_}, {"status", status}, {"detail", detail}};
}
void TestUiInput::poll(JoystickSample& sample, std::vector<unsigned>& pressed,
                       const nlohmann::json& state, bool ready, bool busy) {
    auto now = aviator::monotonic_us();
    const auto system = state.value("system", nlohmann::json::object());
    const auto phase = system.value("state", "");
    // Before START_CONTROL, hold the current reference, including after regrasp
    // or a camera-selected grasp pose. A point is accepted only in CONTROL.
    if (phase != "CONTROL" && ready && state.contains("wheel_reference")) {
        angle_ = state.at("wheel_reference").at("angle").get<double>();
        displacement_ = state.at("wheel_reference").at("displacement").get<double>();
    }
    if (client_ < 0) {
        client_ = accept4(server_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (client_ >= 0) {
            ucred peer{}; socklen_t size = sizeof(peer);
            if (getsockopt(client_, SOL_SOCKET, SO_PEERCRED, &peer, &size) || peer.uid != getuid())
                disconnect();
        }
    }
    for (unsigned i = 0; client_ >= 0 && i < 16; ++i) {
        char buffer[4096];
        const auto size = recv(client_, buffer, sizeof(buffer), MSG_DONTWAIT | MSG_TRUNC);
        if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (size <= 0 || size > static_cast<ssize_t>(sizeof(buffer))) { disconnect(); break; }
        try {
            const auto message = nlohmann::json::parse(buffer, buffer + size);
            now = aviator::monotonic_us();
            const auto kind = message.at("kind").get<std::string>();
            if (kind != "heartbeat") {
                if (busy || !pressed.empty()) continue; // Preserve the outstanding reply identity.
                const auto id = message.at("id").get<std::string>();
                require(!id.empty() && id.size() <= 64, "Invalid command id");
                command_id_ = id;
            }
            require(message.at("sample_us").is_number_unsigned(), "Invalid UI timestamp");
            const auto stamp = message.at("sample_us").get<std::uint64_t>();
            require(stamp > 0 && stamp <= now && now - stamp < lease_us, "Expired UI input");
            if (kind == "heartbeat") {
                require(stamp > checked_, "Replayed UI heartbeat");
                checked_ = stamp;
                continue;
            }
            // Strict monotonic command identity prevents an old command from
            // being executed again after its reply was lost. Never auto-retry.
            require(stamp > last_command_, "Replayed UI command");
            last_command_ = stamp;
            require(checked_ && now >= checked_ && now - checked_ < lease_us, "UI heartbeat expired");
            require(ready, "Core feedback/input not ready");
            if (kind == "target") {
                require(phase == "CONTROL", "Target requires CONTROL");
                const double angle = message.at("angle").get<double>();
                const double displacement = message.at("displacement").get<double>();
                require(std::isfinite(angle) && angle >= -.87266 && angle <= .87266 &&
                        std::isfinite(displacement) && displacement >= -.170 && displacement <= 0,
                        "Wheel target outside range");
                angle_ = angle; displacement_ = displacement;
                result("ACCEPTED", "Target selected; reference tracking is not a measurement");
                busy = true;
            } else if (kind == "operation") {
                const auto operation = message.at("operation").get<std::string>();
                const std::array<std::string, 6> names{{"enter_standby", "grasp_wheel", "start_control",
                    "exit_control", "leave_wheel", "reset_error"}};
                unsigned index = 0;
                while (index < names.size() && names[index] != operation) ++index;
                require(index < names.size(), "Unknown test operation");
                pressed.push_back(index);
                result("PENDING", operation);
            } else throw std::runtime_error("Unknown UI command");
        } catch (const std::exception& e) { result("REJECTED", e.what()); }
    }
    sample.roll_value = -angle_ / .87266;
    sample.pitch_value = displacement_ / .085 + 1;
    sample.sample_us = sample.checked_us = checked_;
    sample.device_connected = client_ >= 0;
    sample.failed = false;
    now = aviator::monotonic_us();
    sample.valid = client_ >= 0 && checked_ && now >= checked_ && now - checked_ < lease_us;
}
void TestUiInput::report(const nlohmann::json& state, bool fresh, bool ready) {
    if (client_ < 0) return;
    auto message = nlohmann::json{{"kind", "status"}, {"sample_us", aviator::monotonic_us()},
        {"fresh", fresh}, {"ready", ready}, {"state", state}, {"result", result_},
        {"target", {{"angle", angle_}, {"displacement", displacement_}}}};
    const auto payload = message.dump();
    const auto sent = send(client_, payload.data(), payload.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) disconnect();
}
} // namespace flight_gateway

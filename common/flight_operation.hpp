#pragma once
#include <array>
#include <nlohmann/json.hpp>
#include <string>
namespace aviator {
// Service source and operation must agree with the explicitly selected Core input policy.
inline bool validFlightOperation(const nlohmann::json& p, const std::string& operation,
                                 const std::string& source) {
    static constexpr std::array<const char*, 6> names{{"enter_standby", "grasp_wheel",
                                                       "start_control", "exit_control",
                                                       "leave_wheel", "reset_error"}};
    if (!p.is_object() || !p.contains("source") || p["source"] != source)
        return false;
    unsigned index = 0;
    for (; index < names.size(); ++index)
        if (operation == names[index])
            break;
    if (index == names.size())
        return false;
    if (source == "FLIGHT")
        return p.size() == 2 && p.contains("operation_code") &&
               p["operation_code"].is_number_unsigned() &&
               p["operation_code"].get<uint64_t>() == (index + 1) * 16;
    if (source != "JOYSTICK")
        return false;
    return p.size() == (p.contains("server_session_id") ? 3u : 2u) && p.contains("button") &&
           p["button"].is_number_unsigned() && p["button"].get<uint64_t>() >= 1 &&
           p["button"].get<uint64_t>() <= 11;
}
} // namespace aviator

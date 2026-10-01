#pragma once
#include <nlohmann/json.hpp>
#include <string>
namespace monitor {
using Json = nlohmann::json;
Json default_config();
Json load_config(const std::string& path);
void validate_config(const Json& config);
} // namespace monitor

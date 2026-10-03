#pragma once
#include <nlohmann/json.hpp>
#include <string>
namespace monitor {
using Json = nlohmann::json;
Json default_config();
Json load_config(const std::string& path);
Json parse_config(const std::string& yaml);
std::string config_yaml(const Json& config);
void save_config(const std::string& path, const Json& config);
void validate_config(const Json& config);
} // namespace monitor

#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace monitor {
// Only regular .log files directly inside the configured run directory are exposed.
class Logs {
public:
    explicit Logs(std::filesystem::path directory) : directory_(std::move(directory)) {}
    nlohmann::json list() const;
    nlohmann::json read(const std::string& encoded_name) const;
private:
    std::filesystem::path directory_;
};
} // namespace monitor

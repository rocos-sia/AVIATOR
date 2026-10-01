#pragma once
#include "config.hpp"
#include <filesystem>
#include <map>
namespace monitor {
struct Asset {
    std::string body, type;
};
class Assets {
  public:
    Assets(const std::filesystem::path& web, const std::filesystem::path& models,
           const Json& config);
    bool read(const std::string& url, Asset& result) const;
    Json manifest;

  private:
    std::map<std::string, std::pair<std::filesystem::path, std::string>> files_;
};
} // namespace monitor

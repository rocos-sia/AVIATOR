#pragma once
#include "recording_config.hpp"
#include <memory>
#include <string>
#include <string_view>
namespace aviator {
struct CameraFrame {
    nlohmann::json metadata;
    std::string data;
};
inline constexpr const char* camera_schema_name = "aviator.record.v1.CameraPacket";
std::string camera_schema_descriptor();
std::string serialize_camera_frame(const CameraFrame& frame);
CameraFrame parse_camera_frame(std::string_view bytes);
// Validates raw ingress; returns configured topic. Throws invalid_argument on bad input.
std::string validate_camera_frame(const CameraFrame& frame, const CameraRecordingOptions& options);
class CameraCompressor {
  public:
    explicit CameraCompressor(const CameraRecordingOptions& options);
    ~CameraCompressor();
    CameraFrame encode(CameraFrame frame);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator

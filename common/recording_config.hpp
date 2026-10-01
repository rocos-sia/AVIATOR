#pragma once
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
namespace aviator {
struct CameraSource {
    std::string camera_id = "cockpit";
    std::string rgb_topic = "record.camera.cockpit.rgb";
    std::string depth_topic = "record.camera.cockpit.depth";
    bool record_depth = true;
};
struct CameraRecordingOptions {
    std::string mode = "disabled";
    std::string record_endpoint = "tcp://127.0.0.1:5557";
    int receive_hwm = 16;
    std::size_t max_record_bytes = 16 * 1024 * 1024;
    std::size_t queue_bytes = 256 * 1024 * 1024;
    std::vector<CameraSource> sources{CameraSource{}};
    std::string codec = "h265", encoder = "auto";
    int bitrate = 8000000, keyframe_interval = 30, zstd_level = 3;
};
struct RecorderOptions {
    std::string arm_command_mode = "full"; // full wire history or compact execution targets
    std::size_t queue_bytes = 16 * 1024 * 1024;
    int receive_hwm = 4096;
    std::size_t chunk_size_bytes = 4 * 1024 * 1024;
    CameraRecordingOptions camera;
    std::string image_output; // Empty: derive <data stem>.images.mcap.
};
struct RecordingConfig {
    std::string subscribe_endpoint = "tcp://127.0.0.1:5556";
    std::string output;
    RecorderOptions options;
};
std::string image_output_path(const std::string& output, const RecorderOptions& options);
RecordingConfig load_recording_config(const std::string& path);
void validate_recording_config(const RecordingConfig& config);
nlohmann::json recording_config_json(const RecordingConfig& config);
} // namespace aviator

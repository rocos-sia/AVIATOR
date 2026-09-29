#include "recording_config.hpp"
#include <climits>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace aviator {
namespace {
void require(bool ok, const std::string& reason) {
    if (!ok)
        throw std::invalid_argument("recording config: " + reason);
}
void keys(const YAML::Node& n, std::initializer_list<const char*> allowed) {
    require(n.IsMap(), "expected mapping");
    std::set<std::string> names, valid;
    for (auto key : allowed)
        valid.insert(key);
    for (const auto& entry : n) {
        require(entry.first.IsScalar(), "non-scalar key");
        const auto key = entry.first.Scalar();
        require(valid.count(key) && names.insert(key).second, "unknown or duplicate key: " + key);
    }
}
std::string string(const YAML::Node& n) {
    require(n.IsScalar(), "expected string");
    require(n.Tag() == "?" || n.Tag() == "!" || n.Tag() == "tag:yaml.org,2002:str",
            "expected string tag");
    const auto value = n.Scalar();
    if (n.Tag() != "!" && n.Tag() != "tag:yaml.org,2002:str") {
        require(!std::regex_match(value, std::regex("(true|false|True|False|TRUE|FALSE|yes|no|on|"
                                                    "off|null|~|[-+]?[0-9]+([.][0-9]+)?)")),
                "expected string, not boolean/number");
    }
    return value;
}
std::size_t number(const YAML::Node& n, std::size_t limit = INT_MAX, bool zero = false) {
    require(n.IsScalar(), "expected integer");
    require(n.Tag() == "?" || n.Tag() == "tag:yaml.org,2002:int", "expected integer tag");
    const auto text = n.Scalar();
    require(!text.empty() && text.find_first_not_of("0123456789") == std::string::npos,
            "expected unsigned integer");
    require(n.Tag() != "!" && n.Tag() != "tag:yaml.org,2002:str", "quoted integer");
    const auto value = std::stoull(text);
    require((zero || value > 0) && value <= limit, "integer out of range");
    return value;
}
bool boolean(const YAML::Node& n) {
    require(n.IsScalar() && (n.Tag() == "?" || n.Tag() == "tag:yaml.org,2002:bool"),
            "expected boolean");
    require(n.Scalar() == "true" || n.Scalar() == "false", "expected true or false");
    return n.Scalar() == "true";
}
void fixed(const YAML::Node& n, const char* value) {
    require(string(n) == value, std::string("expected ") + value);
}
} // namespace
std::string image_output_path(const std::string& output, const RecorderOptions& options) {
    if (!options.image_output.empty())
        return options.image_output;
    auto path = std::filesystem::path(output);
    path.replace_extension(".images.mcap");
    return path.string();
}
RecordingConfig load_recording_config(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    keys(root, {"config_version", "bus", "output", "camera"});
    require(number(root["config_version"]) == 1, "unsupported config_version");
    RecordingConfig c;
    auto& o = c.options;
    if (const auto n = root["bus"]) {
        keys(n, {"subscribe_endpoint", "receive_hwm", "queue_bytes"});
        if (n["subscribe_endpoint"])
            c.subscribe_endpoint = string(n["subscribe_endpoint"]);
        if (n["receive_hwm"])
            o.receive_hwm = number(n["receive_hwm"]);
        if (n["queue_bytes"])
            o.queue_bytes = number(n["queue_bytes"]);
    }
    if (const auto n = root["output"]) {
        keys(n, {"path", "image_path", "chunk_compression", "chunk_size_bytes"});
        if (n["path"])
            c.output = string(n["path"]);
        if (n["image_path"])
            o.image_output = string(n["image_path"]);
        if (n["chunk_compression"])
            fixed(n["chunk_compression"], "none");
        if (n["chunk_size_bytes"])
            o.chunk_size_bytes = number(n["chunk_size_bytes"]);
    }
    if (const auto n = root["camera"]) {
        keys(n, {"mode", "record_endpoint", "receive_hwm", "max_record_bytes", "queue_bytes",
                 "overflow_policy", "sources", "compressed"});
        auto& camera = o.camera;
        if (n["mode"])
            camera.mode = string(n["mode"]);
        if (n["record_endpoint"])
            camera.record_endpoint = string(n["record_endpoint"]);
        if (n["receive_hwm"])
            camera.receive_hwm = number(n["receive_hwm"]);
        if (n["max_record_bytes"])
            camera.max_record_bytes = number(n["max_record_bytes"]);
        if (n["queue_bytes"])
            camera.queue_bytes = number(n["queue_bytes"]);
        if (n["overflow_policy"])
            fixed(n["overflow_policy"], "drop_newest");
        if (const auto sources = n["sources"]) {
            require(sources.IsSequence(), "sources must be a sequence");
            camera.sources.clear();
            for (const auto& source : sources) {
                keys(source, {"camera_id", "rgb_topic", "depth_topic", "rgb_pixel_format",
                              "depth_pixel_format", "record_depth"});
                fixed(source["rgb_pixel_format"], "RGB8");
                fixed(source["depth_pixel_format"], "Z16");
                camera.sources.push_back({string(source["camera_id"]), string(source["rgb_topic"]),
                                          string(source["depth_topic"]),
                                          source["record_depth"] ? boolean(source["record_depth"])
                                                                 : true});
            }
        }
        if (const auto enc = n["compressed"]) {
            keys(enc, {"rgb", "depth"});
            if (const auto rgb = enc["rgb"]) {
                keys(rgb, {"codec", "encoder", "pixel_format", "target_bitrate_bps",
                           "keyframe_interval_frames", "b_frames"});
                if (rgb["codec"])
                    camera.codec = string(rgb["codec"]);
                if (rgb["encoder"])
                    camera.encoder = string(rgb["encoder"]);
                if (rgb["pixel_format"])
                    fixed(rgb["pixel_format"], "yuv420p");
                if (rgb["target_bitrate_bps"])
                    camera.bitrate = number(rgb["target_bitrate_bps"]);
                if (rgb["keyframe_interval_frames"])
                    camera.keyframe_interval = number(rgb["keyframe_interval_frames"]);
                if (rgb["b_frames"])
                    require(number(rgb["b_frames"], INT_MAX, true) == 0, "b_frames must be zero");
            }
            if (const auto depth = enc["depth"]) {
                keys(depth, {"codec", "level"});
                if (depth["codec"])
                    fixed(depth["codec"], "zstd");
                if (depth["level"])
                    camera.zstd_level = number(depth["level"], 19);
            }
        }
    }
    validate_recording_config(c);
    return c;
}
void validate_recording_config(const RecordingConfig& c) {
    const auto& o = c.options;
    const auto& k = o.camera;
    if (k.mode != "disabled" && !c.output.empty()) {
        const auto data = std::filesystem::weakly_canonical(std::filesystem::absolute(c.output));
        const auto image = std::filesystem::weakly_canonical(
            std::filesystem::absolute(image_output_path(c.output, o)));
        require(data != image && data.string() + ".partial" != image.string() &&
                    image.string() + ".partial" != data.string(), "data/image output paths conflict");
    }
    require(c.subscribe_endpoint.rfind("tcp://", 0) == 0, "bus requires TCP");
    require(o.queue_bytes > 0 && o.queue_bytes <= INT_MAX && o.receive_hwm > 0 &&
                o.chunk_size_bytes > 0 && o.chunk_size_bytes <= INT_MAX,
            "invalid bus/chunk limits");
    require(k.mode == "disabled" || k.mode == "raw" || k.mode == "compressed",
            "invalid camera.mode");
    require(k.record_endpoint.rfind("tcp://", 0) == 0, "camera requires TCP");
    require(k.mode == "disabled" || k.record_endpoint != c.subscribe_endpoint,
            "camera/bus endpoint conflict");
    require(k.receive_hwm > 0 && k.max_record_bytes > 0 && k.max_record_bytes <= INT_MAX &&
                k.queue_bytes >= k.max_record_bytes && k.queue_bytes <= INT_MAX,
            "invalid camera limits");
    require(k.codec == "h264" || k.codec == "h265", "invalid RGB codec");
    require(k.encoder == "auto" || k.encoder == "hardware" || k.encoder == "software",
            "invalid encoder");
    require(k.bitrate > 0 && k.keyframe_interval > 0 && k.zstd_level >= 1 && k.zstd_level <= 19,
            "invalid compression settings");
    require(!k.sources.empty(), "empty sources");
    std::set<std::string> ids;
    for (const auto& s : k.sources) {
        require(std::regex_match(s.camera_id, std::regex("[A-Za-z0-9_-]{1,80}")) &&
                    ids.insert(s.camera_id).second,
                "invalid/duplicate camera_id");
        require(s.rgb_topic == "record.camera." + s.camera_id + ".rgb" &&
                    s.depth_topic == "record.camera." + s.camera_id + ".depth",
                "topic does not match camera_id");
    }
}
nlohmann::json recording_config_json(const RecordingConfig& c) {
    const auto& o = c.options;
    const auto& k = o.camera;
    nlohmann::json sources = nlohmann::json::array();
    for (const auto& s : k.sources)
        sources.push_back({{"camera_id", s.camera_id},
                           {"rgb_topic", s.rgb_topic},
                           {"depth_topic", s.depth_topic},
                           {"record_depth", s.record_depth},
                           {"rgb_pixel_format", "RGB8"},
                           {"depth_pixel_format", "Z16"}});
    return {{"config_version", 1},
            {"bus",
             {{"subscribe_endpoint", c.subscribe_endpoint},
              {"receive_hwm", o.receive_hwm},
              {"queue_bytes", o.queue_bytes}}},
            {"output",
             {{"path", c.output},
              {"image_path", k.mode == "disabled" ? "" : image_output_path(c.output, o)},
              {"chunk_compression", "none"},
              {"chunk_size_bytes", o.chunk_size_bytes}}},
            {"camera",
             {{"mode", k.mode},
              {"record_endpoint", k.record_endpoint},
              {"receive_hwm", k.receive_hwm},
              {"max_record_bytes", k.max_record_bytes},
              {"queue_bytes", k.queue_bytes},
              {"overflow_policy", "drop_newest"},
              {"sources", sources},
              {"compressed",
               {{"rgb",
                 {{"codec", k.codec},
                  {"encoder", k.encoder},
                  {"pixel_format", "yuv420p"},
                  {"target_bitrate_bps", k.bitrate},
                  {"keyframe_interval_frames", k.keyframe_interval},
                  {"b_frames", 0}}},
                {"depth", {{"codec", "zstd"}, {"level", k.zstd_level}}}}}}}};
}
} // namespace aviator

#include "config.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
namespace monitor {
Json default_config() {
    Json arms = Json::object();
    for (const auto* side : {"left", "right"}) {
        Json joints = Json::array();
        for (unsigned i = 1; i <= 7; ++i)
            joints.push_back({{"name", std::string("AR5-5_07") + (side[0] == 'l' ? "L" : "R") +
                                           "-W4C4A2_joint_" + std::to_string(i)},
                              {"sign", 1},
                              {"offset_rad", 0}});
        arms[side] = joints;
    }
    return {{"version", 1},
            {"sources",
             {{"flight.command", "flight_gateway"},
              {"flight.state", "aviator_core"},
              {"arm.state", "manipulator"},
              {"hand.state", "inspire_hand"},
              {"camera.detection", "camera"}}},
            {"timeouts_ms",
             {{"flight.command", 100},
              {"flight.state", 100},
              {"arm.command", 50},
              {"arm.state", 50},
              {"hand.command", 100},
              {"hand.state", 300},
              {"camera.command", 200},
              {"camera.detection", 200}}},
            {"preview",
             {{"endpoint", "tcp://127.0.0.1:5561"},
              {"camera_id", "cockpit"},
              {"publisher_id", "camera"},
              {"timeout_ms", 500}}},
            {"arm_joints", arms},
            {"hand_calibration", nullptr},
            {"yoke_calibration", nullptr}};
}
namespace {
void require(bool ok, const std::string& message) {
    if (!ok)
        throw std::runtime_error("monitor config: " + message);
}
bool finite(const Json& j) { return j.is_number() && std::isfinite(j.get<double>()); }
void vector(const Json& j, unsigned size) {
    require(j.is_array() && j.size() == size, "invalid vector length");
    for (const auto& v : j)
        require(finite(v), "non-finite vector");
}
void pose(const Json& j) {
    vector(j.at("position_m"), 3);
    vector(j.at("quaternion_xyzw"), 4);
    double norm = 0;
    for (const auto& q : j.at("quaternion_xyzw"))
        norm += q.get<double>() * q.get<double>();
    require(std::abs(norm - 1) < 1e-5, "quaternion must be unit xyzw");
}
} // namespace
void validate_config(const Json& c) {
    require(c.is_object() && c.at("version") == 1, "version must be 1");
    const auto defaults = default_config();
    for (const auto& [key, value] : c.items())
        require(defaults.contains(key), "unknown key " + key);
    require(c.at("timeouts_ms").is_object(), "timeouts_ms must be object");
    require(c.at("preview").is_object(), "preview must be object");
    for (const auto& [key, value] : c.at("preview").items())
        require(defaults.at("preview").contains(key), "unknown preview key " + key);
    require(c.at("sources").is_object(), "sources must be object");
    for (const auto& [key, value] : defaults.at("sources").items())
        require(c.at("sources").contains(key), "source cannot be null/deleted: " + key);
    for (const auto& [key, value] : defaults.at("timeouts_ms").items())
        require(c.at("timeouts_ms").contains(key), "timeout cannot be null/deleted: " + key);
    for (const auto& [topic, value] : c.at("sources").items()) {
        require(defaults.at("sources").contains(topic), "unknown overview topic " + topic);
        if (value.is_string()) {
            require(value.get<std::string>().size() <= 128, "invalid publisher");
        } else {
            require(value.is_object() && value.size() <= 2 &&
                        value.at("publisher_id").is_string() &&
                        !value.at("publisher_id").get<std::string>().empty() &&
                        value.at("publisher_id").get<std::string>().size() <= 128,
                    "source must be publisher string or publisher/session selector");
            // Legacy session selectors are accepted but no longer filter sources.
        }
    }
    for (const auto& [topic, value] : c.at("timeouts_ms").items()) {
        require(defaults.at("timeouts_ms").contains(topic), "unknown timeout topic " + topic);
        require(finite(value) && value.get<double>() >= 1 && value.get<double>() <= 10000,
                "invalid timeout");
    }
    const auto& p = c.at("preview");
    require(p.at("endpoint").is_string(), "preview endpoint must be string");
    const auto endpoint = p.at("endpoint").get<std::string>();
    require(endpoint.empty() || endpoint.rfind("tcp://", 0) == 0, "preview endpoint must use TCP");
    require(p.at("camera_id").is_string() && !p.at("camera_id").get<std::string>().empty() &&
                p.at("camera_id").get<std::string>().size() <= 64,
            "invalid camera id");
    require(p.at("publisher_id").is_string() && !p.at("publisher_id").get<std::string>().empty(),
            "invalid preview publisher");
    require(finite(p.at("timeout_ms")) && p.at("timeout_ms").get<double>() >= 1 &&
                p.at("timeout_ms").get<double>() <= 10000,
            "invalid preview timeout");
    std::set<std::string> names;
    for (const auto* side : {"left", "right"}) {
        const auto& joints = c.at("arm_joints").at(side);
        require(joints.is_array() && joints.size() == 7, "arm mapping must contain seven joints");
        for (const auto& joint : joints) {
            require(joint.at("name").is_string() &&
                        names.insert(joint.at("name").get<std::string>()).second,
                    "duplicate arm joint");
            require(finite(joint.at("sign")) && std::abs(joint.at("sign").get<double>()) == 1 &&
                        finite(joint.at("offset_rad")),
                    "invalid arm sign/offset");
        }
    }
    const auto& hand = c.at("hand_calibration");
    if (!hand.is_null()) {
        require(hand.at("id").is_string() && !hand.at("id").get<std::string>().empty(),
                "hand calibration id required");
        for (const auto* side : {"left", "right"}) {
            const auto& curves = hand.at(side);
            require(curves.is_array() && curves.size() == 6, "six hand curves required per side");
            for (const auto& curve : curves) {
                require(curve.at("joint").is_string() &&
                            names.insert(curve.at("joint").get<std::string>()).second,
                        "duplicate calibrated joint");
                const auto& knots = curve.at("knots");
                require(knots.is_array() && knots.size() >= 2 && knots.size() <= 32,
                        "invalid hand knots");
                double previous = -1;
                for (const auto& knot : knots) {
                    vector(knot, 2);
                    double x = knot[0].get<double>();
                    require(x >= 0 && x <= 1 && x > previous, "knots must increase within [0,1]");
                    previous = x;
                }
                require(knots.front()[0] == 0 && knots.back()[0] == 1,
                        "hand curves must cover [0,1]");
            }
        }
    }
    const auto& y = c.at("yoke_calibration");
    if (!y.is_null()) {
        require(y.at("id").is_string() && !y.at("id").get<std::string>().empty(),
                "yoke calibration id required");
        for (const auto* key : {"aircraft_camera", "tag_yoke", "aircraft_yoke_zero"})
            pose(y.at(key));
        for (const auto* key : {"roll_axis", "pitch_axis"}) {
            vector(y.at(key), 3);
            double norm = 0;
            for (const auto& v : y.at(key))
                norm += v.get<double>() * v.get<double>();
            require(std::abs(norm - 1) < 1e-5, "motion axes must be unit vectors");
        }
        for (const auto* key : {"pitch_zero_mm", "min_confidence", "max_rotation_residual_deg",
                                "max_translation_residual_mm"})
            require(finite(y.at(key)), "invalid yoke threshold");
        require(y.at("pitch_zero_mm") >= -5 && y.at("pitch_zero_mm") <= 175 &&
                    y.at("min_confidence") >= 0 && y.at("min_confidence") <= 1 &&
                    y.at("max_rotation_residual_deg") > 0 &&
                    y.at("max_translation_residual_mm") > 0,
                "yoke thresholds out of range");
        const auto& display = y.at("model");
        for (const auto* key : {"roll_sign", "roll_offset_rad", "pitch_sign", "pitch_offset_m"})
            require(finite(display.at(key)), "invalid display mapping");
        require(std::abs(display.at("roll_sign").get<double>()) == 1 &&
                    std::abs(display.at("pitch_sign").get<double>()) == 1,
                "display sign must be +/-1");
    }
}
namespace {
Json from_yaml(const YAML::Node& node, unsigned depth, unsigned& budget) {
    require(depth < 32 && budget-- > 0, "YAML nesting/size limit exceeded");
    if (node.IsNull())
        return nullptr;
    if (node.IsMap()) {
        Json result = Json::object();
        for (const auto& item : node) {
            require(item.first.IsScalar(), "YAML keys must be strings");
            const auto key = item.first.Scalar();
            require(!result.contains(key), "duplicate YAML key " + key);
            result[key] = from_yaml(item.second, depth + 1, budget);
        }
        return result;
    }
    if (node.IsSequence()) {
        Json result = Json::array();
        for (const auto& item : node)
            result.push_back(from_yaml(item, depth + 1, budget));
        return result;
    }
    require(node.IsScalar(), "invalid YAML value");
    const auto value = node.Scalar();
    if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str")
        return value;
    if (value == "true")
        return true;
    if (value == "false")
        return false;
    long long integer;
    if (YAML::convert<long long>::decode(node, integer))
        return integer;
    double number;
    if (YAML::convert<double>::decode(node, number)) {
        require(std::isfinite(number), "non-finite YAML number");
        return number;
    }
    return value;
}
void emit_yaml(YAML::Emitter& out, const Json& value) {
    if (value.is_object()) {
        out << YAML::BeginMap;
        for (const auto& [key, item] : value.items()) {
            out << YAML::Key << key << YAML::Value;
            emit_yaml(out, item);
        }
        out << YAML::EndMap;
    } else if (value.is_array()) {
        out << YAML::BeginSeq;
        for (const auto& item : value)
            emit_yaml(out, item);
        out << YAML::EndSeq;
    } else if (value.is_string())
        out << YAML::DoubleQuoted << value.get<std::string>();
    else if (value.is_null())
        out << YAML::Null;
    else if (value.is_boolean())
        out << value.get<bool>();
    else if (value.is_number_unsigned())
        out << value.get<std::uint64_t>();
    else if (value.is_number_integer())
        out << value.get<std::int64_t>();
    else
        out << value.get<double>();
}
} // namespace
Json parse_config(const std::string& yaml) {
    require(yaml.size() <= 65536, "configuration exceeds 64 KiB");
    unsigned budget = 8192;
    const auto documents = YAML::LoadAll(yaml);
    require(documents.size() == 1, "exactly one YAML document required");
    const auto overrides = from_yaml(documents.front(), 0, budget);
    Json c = default_config();
    require(overrides.is_object(), "root must be object");
    for (const auto& [key, value] : overrides.items())
        require(c.contains(key), "unknown key " + key);
    c.merge_patch(overrides);
    for (const auto* key : {"hand_calibration", "yoke_calibration"})
        if (!c.contains(key))
            c[key] = nullptr;
    validate_config(c);
    return c;
}
std::string config_yaml(const Json& config) {
    validate_config(config);
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    emit_yaml(out, config);
    require(out.good(), "cannot serialize YAML");
    return std::string(out.c_str()) + "\n";
}
Json load_config(const std::string& path) {
    if (path.empty())
        return default_config();
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    require(file && file.tellg() >= 0 && file.tellg() <= 65536,
            "cannot read configuration (64 KiB maximum): " + path);
    file.seekg(0);
    return parse_config(std::string(std::istreambuf_iterator<char>(file), {}));
}
void save_config(const std::string& path, const Json& config) {
    require(!path.empty(), "no configuration file configured");
    const auto yaml = config_yaml(config);
    require(yaml.size() <= 65536, "configuration exceeds 64 KiB");
    // Resolve a configured symlink, and atomically replace its target in the same directory.
    const auto target = std::filesystem::weakly_canonical(path).string();
    std::string temporary = target + ".tmp.XXXXXX";
    int fd = mkstemp(temporary.data());
    require(fd >= 0, "cannot create temporary configuration: " + std::string(std::strerror(errno)));
    try {
        struct stat existing {};
        if (stat(target.c_str(), &existing) == 0)
            require(fchmod(fd, existing.st_mode & 0777) == 0, "cannot preserve file permissions");
        std::size_t written = 0;
        while (written < yaml.size()) {
            const auto n = write(fd, yaml.data() + written, yaml.size() - written);
            if (n < 0 && errno == EINTR)
                continue;
            require(n > 0, "cannot write configuration");
            written += n;
        }
        require(fsync(fd) == 0, "cannot flush configuration");
        const int closed = close(fd);
        fd = -1;
        require(closed == 0, "cannot close configuration");
        require(rename(temporary.c_str(), target.c_str()) == 0,
                "cannot replace configuration: " + std::string(std::strerror(errno)));
    } catch (...) {
        if (fd >= 0)
            close(fd);
        unlink(temporary.c_str());
        throw;
    }
}
} // namespace monitor

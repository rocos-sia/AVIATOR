#include "aviator/backend.hpp"
#ifdef AVIATOR_GRASP_HAVE_ROKAE
#include "rokae_joint_reader.hpp"
#endif

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <time.h>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
using Joints = std::array<double, 7>;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Options {
    fs::path robot_config = "config/robot.yaml";
    fs::path joints_file;
    bool read_robot = false;
    int samples = 20;
    int interval_ms = 50;
};

void usage() {
    std::cerr << "Usage: aviator_grasp_tool_state [--robot-config PATH] "
                 "(--read-robot | --joints-file PATH) [--samples N] [--interval-ms N]\n"
                 "Outputs aircraft-frame flange poses as JSONL; joints are in radians.\n"
                 "Defaults: config/robot.yaml, 20 samples, 50 ms between sample starts.\n"
                 "Offline JSON: {\"left\":[7 joint radians],\"right\":[7 joint radians]}.\n"
                 "Robot mode reads jointPos only; do not run alongside control nodes.\n";
}

int integerOption(const std::string& text, const std::string& name) {
    int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    require(result.ec == std::errc{} && result.ptr == text.data() + text.size(),
            name + " must be an integer");
    return value;
}

Options parseOptions(int argc, char** argv) {
    Options options;
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        require(seen.insert(key).second, "Repeated argument " + key);
        if (key == "--read-robot") {
            options.read_robot = true;
            continue;
        }
        require(key == "--robot-config" || key == "--joints-file" ||
                key == "--samples" || key == "--interval-ms", "Unknown argument " + key);
        require(i + 1 < argc, "Missing value for " + key);
        const std::string value = argv[++i];
        require(!value.empty(), "Empty value for " + key);
        if (key == "--robot-config") options.robot_config = value;
        else if (key == "--joints-file") options.joints_file = value;
        else if (key == "--samples") options.samples = integerOption(value, key);
        else options.interval_ms = integerOption(value, key);
    }
    require(options.read_robot != !options.joints_file.empty(),
            "Choose exactly one of --read-robot and --joints-file");
    require(options.samples >= 2 && options.samples <= 10000, "--samples must be in [2, 10000]");
    require(options.interval_ms >= 1 && options.interval_ms <= 10000,
            "--interval-ms must be in [1, 10000]");
    require(static_cast<long long>(options.samples - 1) * options.interval_ms <= 300000,
            "Sampling duration must not exceed 300 seconds");
#ifndef AVIATOR_GRASP_HAVE_ROKAE
    require(!options.read_robot,
            "Robot acquisition requires a build with AVIATOR_BUILD_XCORE_SDK=ON; "
            "--joints-file remains available offline");
#endif
    return options;
}

// CLOCK_MONOTONIC uses the same host clock as Python time.monotonic_ns(). These
// are acquisition times, not controller timestamps or command-reference times.
std::int64_t monoUs() {
    timespec value{};
    require(clock_gettime(CLOCK_MONOTONIC, &value) == 0, "clock_gettime failed");
    return static_cast<std::int64_t>(value.tv_sec) * 1000000 + value.tv_nsec / 1000;
}

// Preserve a dedicated output descriptor and route SDK/library stdout to
// stderr, so even vendor diagnostics cannot corrupt the machine-readable pipe.
class JsonLines {
 public:
    JsonLines() {
        std::cout.flush();
        std::fflush(stdout);
        const int output_fd = dup(STDOUT_FILENO);
        require(output_fd >= 0, "Cannot duplicate stdout");
        output_ = fdopen(output_fd, "w");
        if (!output_) {
            close(output_fd);
            throw std::runtime_error("Cannot open JSON output stream");
        }
        if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            std::fclose(output_);
            output_ = nullptr;
            throw std::runtime_error("Cannot redirect SDK diagnostics to stderr");
        }
    }
    ~JsonLines() { if (output_) std::fclose(output_); }
    void write(const Json& value) {
        const std::string line = value.dump() + '\n';
        require(std::fwrite(line.data(), 1, line.size(), output_) == line.size() &&
                std::fflush(output_) == 0, "Cannot write JSON sample");
    }
 private:
    FILE* output_ = nullptr;
};

fs::path configPath(const YAML::Node& config, const char* key, const fs::path& directory) {
    require(config[key] && config[key].IsScalar(), std::string("Missing path: ") + key);
    fs::path path = config[key].as<std::string>();
    require(!path.empty(), std::string("Empty path: ") + key);
    if (path.is_relative()) path = directory / path;
    return path.lexically_normal();
}

Joints parseJoints(const Json& json, const char* side) {
    require(json.contains(side) && json.at(side).is_array() && json.at(side).size() == 7,
            std::string(side) + " must contain exactly seven joint positions in radians");
    Joints values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto& item = json.at(side).at(i);
        require(item.is_number(), std::string(side) + " joint values must be JSON numbers");
        values[i] = item.get<double>();
        require(std::isfinite(values[i]), std::string(side) + " joint values must be finite");
    }
    return values;
}

Json armPose(aviator::Kinematics& kinematics, aviator::Side side, const Joints& joints) {
    const std::string name = side == aviator::Side::Left ? "left" : "right";
    for (int axis = 0; axis < 7; ++axis)
        require(std::isfinite(joints[axis]) && joints[axis] >= kinematics.jointLower(side, axis) &&
                joints[axis] <= kinematics.jointUpper(side, axis),
                name + " J" + std::to_string(axis + 1) +
                " is non-finite or outside the physical URDF position limits (radians)");
    pinocchio::SE3 flange;
    require(kinematics.solveFk(side, joints, flange) && flange.translation().allFinite() &&
            flange.rotation().allFinite(), name + " flange FK failed");
    auto quaternion = Eigen::Quaterniond(flange.rotation()).normalized();
    if (quaternion.w() < 0) quaternion.coeffs() *= -1;
    const auto& p = flange.translation();
    return {{"joint_position", joints},
            {"flange", {{"position", {p.x(), p.y(), p.z()}},
                        {"quaternion", {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()}}}}};
}

void run(const Options& options) {
    JsonLines output;
    const fs::path robot_path = fs::absolute(options.robot_config).lexically_normal();
    const auto config = YAML::LoadFile(robot_path.string());
    require(config.IsMap(), "Robot config must be a mapping");
    const auto posture = YAML::LoadFile(configPath(config, "posture", robot_path.parent_path()).string());
    const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
    const double margin = posture["joint2_planning_margin_deg"].as<double>();
    require(limits.size() == 2 && std::isfinite(limits[0]) && std::isfinite(limits[1]) &&
            std::isfinite(margin) && margin >= 0 && limits[0] + margin < limits[1] - margin,
            "Invalid posture J2 limits/planning margin");
    constexpr double radians = std::numbers::pi / 180.0;
    auto kinematics = aviator::makePinIkKinematics(
        configPath(config, "urdf", robot_path.parent_path()).string(),
        (limits[0] + margin) * radians, (limits[1] - margin) * radians);

    std::array<Joints, 2> joints{};
#ifdef AVIATOR_GRASP_HAVE_ROKAE
    std::array<std::unique_ptr<aviator::calibration::RokaeJointReader>, 2> readers;
#endif
    if (!options.read_robot) {
        std::ifstream file(options.joints_file);
        require(bool(file), "Cannot read joints file " + options.joints_file.string());
        const auto json = Json::parse(file);
        require(json.is_object(), "Joints file must be a JSON object");
        joints = {parseJoints(json, "left"), parseJoints(json, "right")};
        // Validate the entire input before emitting any samples.
        armPose(*kinematics, aviator::Side::Left, joints[0]);
        armPose(*kinematics, aviator::Side::Right, joints[1]);
    } else {
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        const auto rokae = config["rokae"];
        require(rokae && rokae.IsMap(), "Missing rokae configuration");
        const std::array<std::string, 2> ips{
            rokae["left_ip"].as<std::string>(), rokae["right_ip"].as<std::string>()};
        for (const auto& ip : ips)
            require(!ip.empty() && ip.size() <= 255 && ip.find_first_of(" \t\r\n") == std::string::npos,
                    "Invalid robot IP address");
        require(ips[0] != ips[1], "Left and right robot addresses must be different");
        std::cerr << "Reading static joint positions; no motion/mode/power/tool commands. "
                     "The SDK disconnect operation itself stops robot motion.\n";
        for (std::size_t i = 0; i < readers.size(); ++i)
            readers[i] = std::make_unique<aviator::calibration::RokaeJointReader>(ips[i]);
#endif
    }

    auto next_sample = Clock::now();
    for (int index = 0; index < options.samples; ++index) {
        std::this_thread::sleep_until(next_sample);
        const auto start = monoUs();
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        if (options.read_robot) {
            joints[0] = readers[0]->position();
            joints[1] = readers[1]->position();
        }
#endif
        const auto end = monoUs();
        output.write({{"sample_index", index}, {"sample_start_mono_us", start},
                      {"sample_end_mono_us", end}, {"sample_mono_us", start + (end - start) / 2},
                      {"source", options.read_robot ? "robot" : "offline"},
                      {"frame", "aircraft"},
                      {"left", armPose(*kinematics, aviator::Side::Left, joints[0])},
                      {"right", armPose(*kinematics, aviator::Side::Right, joints[1])}});
        // Never catch up with a burst if an SDK read took longer than a period.
        next_sample = std::max(next_sample + std::chrono::milliseconds(options.interval_ms), Clock::now());
    }
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        usage();
        return 0;
    }
    try {
        run(parseOptions(argc, argv));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "grasp tool state: " << error.what() << '\n';
        return 1;
    }
}

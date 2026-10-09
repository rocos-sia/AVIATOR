#include "aviator/backend.hpp"
#ifdef AVIATOR_GRASP_HAVE_ROKAE
#include "rokae_session.hpp"
#endif

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <poll.h>
#include <time.h>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
using Joints = std::array<double, 7>;
namespace fs = std::filesystem;
volatile sig_atomic_t interrupted = 0;
void onSignal(int signal) { interrupted = signal; }
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Options {
    fs::path robot_config = "config/robot.yaml";
    bool dry_run = false;
};

void usage() {
    std::cerr << "Usage: aviator_grasp_tool_session [--robot-config PATH] [--dry-run]\n"
                 "Persistent SDK/FK service. JSONL stdin/stdout: {id: integer, op: string, side?: left/right/both}.\n"
                 "Operations: connect, status, drag_start, drag_stop, sample, disconnect.\n"
                 "Drag uses joint space, free type, with the physical drag button required.\n"
                 "The SDK disconnect operation stops motion. Exit other robot controllers first.\n";
}

Options parseOptions(int argc, char** argv) {
    Options options;
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        require(seen.insert(key).second, "Repeated argument " + key);
        if (key == "--dry-run") options.dry_run = true;
        else {
            require(key == "--robot-config", "Unknown argument " + key);
            require(i + 1 < argc && *argv[i + 1], "Missing value for " + key);
            options.robot_config = argv[++i];
        }
    }
#ifndef AVIATOR_GRASP_HAVE_ROKAE
    require(options.dry_run, "Robot session requires AVIATOR_BUILD_XCORE_SDK=ON; --dry-run is available offline");
#endif
    return options;
}

std::int64_t monoUs() {
    timespec value{};
    require(clock_gettime(CLOCK_MONOTONIC, &value) == 0, "clock_gettime failed");
    return static_cast<std::int64_t>(value.tv_sec) * 1000000 + value.tv_nsec / 1000;
}

// Save the protocol pipe before redirecting every SDK/Core stdout diagnostic.
class JsonLines {
 public:
    JsonLines() {
        std::cout.flush();
        std::fflush(stdout);
        const int descriptor = dup(STDOUT_FILENO);
        require(descriptor >= 0, "Cannot duplicate stdout");
        output_ = fdopen(descriptor, "w");
        if (!output_) { close(descriptor); throw std::runtime_error("Cannot open JSON stream"); }
        if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            std::fclose(output_);
            output_ = nullptr;
            throw std::runtime_error("Cannot redirect SDK diagnostics");
        }
    }
    ~JsonLines() { if (output_) std::fclose(output_); }
    void write(const Json& json) {
        const auto line = json.dump() + '\n';
        require(std::fwrite(line.data(), 1, line.size(), output_) == line.size() &&
                std::fflush(output_) == 0, "Cannot write JSON response");
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

Json armPose(aviator::Kinematics& kinematics, aviator::Side side, const Joints& joints) {
    const std::string name = side == aviator::Side::Left ? "left" : "right";
    for (int axis = 0; axis < 7; ++axis)
        require(std::isfinite(joints[axis]) && joints[axis] >= kinematics.jointLower(side, axis) &&
                joints[axis] <= kinematics.jointUpper(side, axis),
                name + " J" + std::to_string(axis + 1) + " is outside physical URDF joint limits");
    pinocchio::SE3 flange;
    require(kinematics.solveFk(side, joints, flange) && flange.translation().allFinite() &&
            flange.rotation().allFinite(), name + " flange FK failed");
    auto quaternion = Eigen::Quaterniond(flange.rotation()).normalized();
    if (quaternion.w() < 0) quaternion.coeffs() *= -1;
    const auto& p = flange.translation();
    return {{"joint_position", joints}, {"flange", {{"position", {p.x(), p.y(), p.z()}},
        {"quaternion", {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()}}}}};
}

class Session {
 public:
    explicit Session(const Options& options) : dry_(options.dry_run) {
        const auto robot_path = fs::absolute(options.robot_config).lexically_normal();
        const auto config = YAML::LoadFile(robot_path.string());
        require(config.IsMap(), "Robot config must be a mapping");
        const auto posture = YAML::LoadFile(configPath(config, "posture", robot_path.parent_path()).string());
        const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        const double margin = posture["joint2_planning_margin_deg"].as<double>();
        require(limits.size() == 2 && std::isfinite(limits[0]) && std::isfinite(limits[1]) &&
                std::isfinite(margin) && margin >= 0 && limits[0] + margin < limits[1] - margin,
                "Invalid posture J2 limits/planning margin");
        constexpr double rad = std::numbers::pi / 180.0;
        kinematics_ = aviator::makePinIkKinematics(
            configPath(config, "urdf", robot_path.parent_path()).string(),
            (limits[0] + margin) * rad, (limits[1] - margin) * rad);
        if (dry_) return; // Never construct or connect an SDK robot in dry mode.
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        const auto rokae = config["rokae"];
        require(rokae && rokae.IsMap(), "Missing rokae configuration");
        const std::array<std::string, 2> ips{
            rokae["left_ip"].as<std::string>(), rokae["right_ip"].as<std::string>()};
        for (const auto& ip : ips)
            require(!ip.empty() && ip.size() <= 255 && ip.find_first_of(" \t\r\n") == std::string::npos,
                    "Invalid robot IP address");
        require(ips[0] != ips[1], "Left and right robot addresses must differ");
        for (std::size_t i = 0; i < arms_.size(); ++i)
            arms_[i] = std::make_unique<aviator::calibration::RokaeSessionArm>(ips[i], &interrupted);
#endif
    }

    Json status() {
        Json result = {{"dry_run", dry_}};
        for (int i = 0; i < 2; ++i) result[sideName(i)] = armStatus(i);
        return result;
    }

    Json execute(const Json& request) {
        require(request.contains("op") && request["op"].is_string(), "op must be a string");
        const auto op = request["op"].get<std::string>();
        if (op == "status") return status();
        if (op == "connect") {
            for (int i = 0; i < 2; ++i) {
                if (dry_) connected_[i] = true;
#ifdef AVIATOR_GRASP_HAVE_ROKAE
                else arms_[i]->connect();
#endif
            }
            return status();
        }
        if (op == "disconnect") {
            disconnect();
            return status();
        }
        if (op == "sample") return sample();
        require(op == "drag_start" || op == "drag_stop", "Unknown operation " + op);
        require(request.contains("side") && request["side"].is_string(), "side must be left, right or both");
        const auto side = request["side"].get<std::string>();
        require(side == "left" || side == "right" || side == "both", "side must be left, right or both");
        std::vector<int> selected = side == "both" ? std::vector<int>{0, 1} :
            std::vector<int>{side == "left" ? 0 : 1};
        if (op == "drag_stop") {
            std::string errors;
            for (const int i : selected) {
                try { stopDrag(i); }
                catch (const std::exception& error) { addError(errors, error.what()); }
            }
            require(errors.empty(), errors);
            return status();
        }
        // Check every selected arm before changing either of them.
        for (const int i : selected) {
            const auto arm = armStatus(i);
            require(arm["connected"].get<bool>(), sideName(i) + " is not connected");
            const auto state = arm["operation_state"].get<std::string>();
            require((state == "drag" && arm["drag_owned"].get<bool>()) || state == "idle" || state == "jog",
                    sideName(i) + " is controlled elsewhere: " + state);
        }
        std::vector<int> attempted;
        try {
            for (const int i : selected) {
                const auto arm = armStatus(i);
                if (arm["drag_owned"].get<bool>() && arm["operation_state"] == "drag") continue;
                attempted.push_back(i);
                startDrag(i);
            }
        } catch (const std::exception& error) {
            std::string errors = error.what();
            for (const int i : attempted) {
                try { stopDrag(i); }
                catch (const std::exception& cleanup) { addError(errors, "Rollback: " + std::string(cleanup.what())); }
            }
            throw std::runtime_error(errors);
        }
        return status();
    }

    void disconnect() {
        std::string errors;
        for (int i = 0; i < 2; ++i) {
            if (dry_) { dragging_[i] = false; connected_[i] = false; }
#ifdef AVIATOR_GRASP_HAVE_ROKAE
            else {
                try { arms_[i]->disconnect(); }
                catch (const std::exception& error) { addError(errors, error.what()); }
            }
#endif
        }
        require(errors.empty(), errors);
    }

 private:
    static std::string sideName(int i) { return i == 0 ? "left" : "right"; }
    static void addError(std::string& errors, const std::string& value) {
        if (!errors.empty()) errors += "; ";
        errors += value;
    }
    Json armStatus(int i) {
        if (dry_) return {{"connected", connected_[i]}, {"dragging", dragging_[i]},
            {"drag_owned", dragging_[i]},
            {"operation_state", !connected_[i] ? "disconnected" : dragging_[i] ? "drag" : "idle"},
            {"operation_state_code", !connected_[i] ? -1 : dragging_[i] ? 3 : 0},
            {"powered", connected_[i] ? Json(false) : Json(nullptr)},
            {"power_state", connected_[i] ? "off" : "unknown"},
            {"power_state_code", connected_[i] ? 1 : -1}};
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        const auto value = arms_[i]->status();
        return {{"connected", value.connected}, {"dragging", value.dragging},
            {"drag_owned", value.drag_owned}, {"operation_state", value.operation_state},
            {"operation_state_code", value.operation_state_code},
            {"powered", value.power_state_code < 0 ? Json(nullptr) : Json(value.power_state_code == 0)},
            {"power_state", value.power_state}, {"power_state_code", value.power_state_code}};
#else
        throw std::runtime_error("SDK unavailable");
#endif
    }
    void startDrag(int i) {
        if (dry_) { require(connected_[i], sideName(i) + " is not connected"); dragging_[i] = true; }
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        else arms_[i]->startDrag();
#endif
    }
    void stopDrag(int i) {
        if (dry_) { require(connected_[i], sideName(i) + " is not connected"); dragging_[i] = false; }
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        else arms_[i]->stopDrag();
#endif
    }
    Json sample() {
        for (int i = 0; i < 2; ++i) {
            const auto arm = armStatus(i);
            require(arm["connected"].get<bool>(), sideName(i) + " is not connected");
            require(!arm["drag_owned"].get<bool>() &&
                    (arm["operation_state"] == "idle" || arm["operation_state"] == "jog"),
                    sideName(i) + " must finish dragging/motion before sampling");
        }
        std::array<Joints, 2> joints{{{{0, 1.5, 0, 0, 0, 0, 0}}, {{0, 1.5, 0, 0, 0, 0, 0}}}};
        const auto start = monoUs();
#ifdef AVIATOR_GRASP_HAVE_ROKAE
        if (!dry_) for (int i = 0; i < 2; ++i) joints[i] = arms_[i]->position();
#endif
        const auto end = monoUs();
        return {{"sample_index", sample_index_++}, {"sample_start_mono_us", start},
            {"sample_end_mono_us", end}, {"sample_mono_us", start + (end - start) / 2},
            {"source", dry_ ? "offline" : "robot"}, {"dry_run", dry_}, {"frame", "aircraft"},
            {"left", armPose(*kinematics_, aviator::Side::Left, joints[0])},
            {"right", armPose(*kinematics_, aviator::Side::Right, joints[1])}};
    }
    bool dry_;
    std::array<bool, 2> connected_{};
    std::array<bool, 2> dragging_{};
    std::uint64_t sample_index_ = 0;
    std::unique_ptr<aviator::Kinematics> kinematics_;
#ifdef AVIATOR_GRASP_HAVE_ROKAE
    std::array<std::unique_ptr<aviator::calibration::RokaeSessionArm>, 2> arms_;
#endif
};

// poll/read instead of blocking getline lets SIGINT/SIGTERM trigger cleanup
// while the service is waiting for its next request (no SDK in a handler).
bool readLine(std::string& line, std::string& pending) {
    while (!interrupted) {
        if (const auto end = pending.find('\n'); end != std::string::npos) {
            line = pending.substr(0, end);
            pending.erase(0, end + 1);
            return true;
        }
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&descriptor, 1, 100);
        if (ready < 0 && errno == EINTR) continue;
        require(ready >= 0, "Cannot poll stdin");
        if (ready == 0) continue;
        require(!(descriptor.revents & (POLLERR | POLLNVAL)), "stdin pipe error");
        char buffer[4096];
        const auto count = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count < 0 && errno == EINTR) continue;
        require(count >= 0, "Cannot read stdin");
        if (count == 0) {
            if (pending.empty()) return false;
            line.swap(pending);
            return true;
        }
        pending.append(buffer, static_cast<std::size_t>(count));
        require(pending.size() <= 1024 * 1024, "JSON request exceeds 1 MiB");
    }
    return false;
}

int run(const Options& options) {
    JsonLines output;
    Session session(options);
    int exit_code = 0;
    try {
        std::string line, pending;
        while (readLine(line, pending)) {
            Json id = nullptr;
            Json response;
            try {
                const auto request = Json::parse(line);
                require(request.is_object(), "Request must be an object");
                require(request.contains("id") && request["id"].is_number_integer(), "id must be an integer");
                id = request["id"];
                response = {{"id", id}, {"ok", true}, {"result", session.execute(request)}};
            } catch (const std::exception& error) {
                response = {{"id", id}, {"ok", false}, {"error", error.what()}};
            }
            output.write(response);
        }
    } catch (const std::exception& error) {
        std::cerr << "Session protocol failure: " << error.what() << '\n';
        exit_code = 1;
    }
    try {
        session.disconnect();
        std::cerr << "Session cleanup complete" << (options.dry_run ? " (dry run)" : "") << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Session cleanup FAILED: " << error.what() << '\n';
        return 2;
    }
    return interrupted ? 128 + interrupted : exit_code;
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        usage();
        return 0;
    }
    struct sigaction action{};
    action.sa_handler = onSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    std::signal(SIGPIPE, SIG_IGN);
    try { return run(parseOptions(argc, argv)); }
    catch (const std::exception& error) {
        std::cerr << "grasp tool session: " << error.what() << '\n';
        return 1;
    }
}

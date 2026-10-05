#include "Logger.hpp"
#include "DataLink_direct.hpp"
#include "Viewer.hpp"
#include "aviator/GraspTools.hpp"
#include "camera.hpp"
#include "camera_output.hpp"
#include "camera_window.hpp"
#include "device_server.hpp"
#include "simulation.hpp"
#include <atomic>
#include <cmath>
#include <future>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
void usage() {
    aviator::Logger::info("simulation [--config system.yaml] [--model path] [--headless] [--no-camera]\n"
        "  [--camera-config camera.yaml] [--camera-id cockpit] [--no-camera-window]\n"
        "  [--preview-endpoint endpoint|off] [--recording-config recording.yaml]\n"
        "  [--duration seconds] [--pub-endpoint endpoint] [--sub-endpoint endpoint]\n"
        "  [--control-epoch UUID] [--core-publisher id] [--origin-publisher id]\n"
        "Arm services: system.yaml manipulator_service; same protocol as manipulator.\n"
        "Arm authorization is installed by Core's authorize service.\n"
        "Hands bind publisher/epoch on first valid command, like rh56ftp_hand.\n"
        "--control-epoch applies to legacy camera command authorization only.");
}
} // namespace
int main(int argc, char** argv) try {
    auto file = aviator::defaultSystemConfig();
    std::string model, pub_endpoint, sub_endpoint, camera_config, preview_endpoint, recording_config;
    std::string camera_id = "cockpit";
    simulation::Authorization auth;
    bool headless = false, no_camera = false, camera_window = true;
    double duration = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        }
        if (arg == "--headless") {
            headless = true;
            continue;
        }
        if (arg == "--no-camera") {
            no_camera = true;
            continue;
        }
        if (arg == "--no-camera-window") { camera_window = false; continue; }
        if (++i == argc)
            throw std::runtime_error("Missing value for " + arg);
        const std::string value = argv[i];
        if (arg == "--config")
            file = value;
        else if (arg == "--camera-config")
            camera_config = value;
        else if (arg == "--camera-id")
            camera_id = value;
        else if (arg == "--preview-endpoint")
            preview_endpoint = value;
        else if (arg == "--recording-config")
            recording_config = value;
        else if (arg == "--model")
            model = value;
        else if (arg == "--pub-endpoint")
            pub_endpoint = value;
        else if (arg == "--sub-endpoint")
            sub_endpoint = value;
        else if (arg == "--control-epoch")
            auth.epoch = value;
        else if (arg == "--core-publisher")
            auth.publisher = value;
        else if (arg == "--origin-publisher")
            auth.origin_publisher = value;
        else if (arg == "--core-session")
            auth.session = value;
        else if (arg == "--origin-session")
            auth.origin_session = value;
        else if (arg == "--duration" || arg == "--camera-timeout-ms") {
            std::size_t used = 0;
            const double v = std::stod(value, &used);
            if (used != value.size() || !std::isfinite(v) || v <= 0 || v > 86400)
                throw std::runtime_error("Invalid " + arg);
            if (arg == "--duration")
                duration = v;
            else
                auth.camera_timeout_us = static_cast<uint64_t>(v * 1000);
        } else
            throw std::runtime_error("Unknown argument: " + arg);
    }
    auto config = aviator::loadMotionConfig(file);
    if (!pub_endpoint.empty())
        config.publish = pub_endpoint;
    if (!sub_endpoint.empty())
        config.subscribe = sub_endpoint;
    for (const auto* endpoint : {&config.publish, &config.subscribe})
        if (endpoint->rfind("tcp://127.0.0.1:", 0) != 0)
            throw std::runtime_error("Simulation requires same-host loopback TCP");
    if (config.publish == config.subscribe || config.publish == config.service ||
        config.subscribe == config.service)
        throw std::runtime_error("Device endpoints must differ");
    const aviator::DeviceSettings settings(config);
    if (model.empty())
        model = settings.path("model").string();
    simulation::Simulation sim(model, auth, true);
    sim.setCameraId(camera_id);
    sim.setInitialWheel(settings.initial_wheel.angle, settings.initial_wheel.displacement);
    auto grasp = YAML::LoadFile(settings.path("grasp").string());
    aviator::SimulationTools tools;
    for (int side = 0; side < 2; ++side) {
        const auto frame = aviator::toolFrameConfig(grasp, side);
        tools[side].position = frame["position"].as<std::array<double, 3>>();
        tools[side].quaternion = frame["quaternion"].as<std::array<double, 4>>();
        double norm = 0;
        for (double v : tools[side].position)
            if (!std::isfinite(v))
                throw std::runtime_error("Invalid tool position");
        for (double v : tools[side].quaternion)
            norm += v * v;
        if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-6)
            throw std::runtime_error("Invalid tool quaternion");
    }
    const auto hand = YAML::LoadFile(config.system.string())["core_hand"];
    const auto hand_publisher =
        hand && hand["publisher_id"] ? hand["publisher_id"].as<std::string>() : "rh56ftp_hand";
    // Create GL resources before physics starts mutating mjData. All subsequent reads are locked.
    std::unique_ptr<simulation::Camera> camera;
    if (!no_camera)
        camera = std::make_unique<simulation::Camera>(sim);
    std::unique_ptr<aviator::Viewer> viewer;
    if (!headless && settings.robot["viewer"].as<bool>(true)) {
        viewer = std::make_unique<aviator::Viewer>(sim.model());
        glfwSwapInterval(0);
    }
    std::unique_ptr<simulation::CameraWindow> camera_view;
    if (viewer && camera && camera_window)
        camera_view = std::make_unique<simulation::CameraWindow>(sim.cameraWidth(), sim.cameraHeight(), viewer->window());
    std::unique_ptr<simulation::CameraOutput> camera_output;
    if (camera) {
        if (camera_config.empty()) {
            const auto adjacent = config.system.parent_path() / "camera.yaml";
            if (std::filesystem::exists(adjacent)) camera_config = adjacent.string();
        }
        camera_output = std::make_unique<simulation::CameraOutput>(
            simulation::previewSettings(camera_config, preview_endpoint), recording_config, camera_id);
    }
    const auto joint_stiffness = settings.robot["rokae"]["joint_stiffness"].as<std::array<double, 7>>();
    aviator::MuJoCoDirectDataLink device(sim.model(), sim.data(), settings.path("urdf").string(),
                                         tools, joint_stiffness, [&] { sim.applyHands(aviator::monotonic_us()); });
    aviator::DeviceServerOptions options;
    options.name = "simulation";
    options.tcp_frames = {"aircraft", "aircraft"};
    options.wheel_measurement = true;
    options.topics = {"hand.command", "camera.command"};
    options.message = [&](const aviator::Message& message, uint64_t now) {
        std::string error;
        std::lock_guard<std::mutex> lock(*device.physicsMutex());
        const bool ok = message.topic == aviator::Topic::hand_command
                            ? sim.handCommand(message, now, error)
                            : sim.command(message, now, error);
        if (!ok) {
            static uint64_t last_report = 0;
            if (now - last_report >= 1000000) {
                aviator::Logger::warn("Rejected {}: {}", aviator::topic_name(message.topic), error);
                last_report = now;
            }
        }
    };
    const auto start = aviator::monotonic_us();
    std::atomic<bool> quit{false};
    std::mutex detection_mutex;
    aviator::Message latest_detection = sim.detection(start, false, false);
    uint64_t next_hand = 0, detection_sequence = 0;
    options.tick = [&](zmq::socket_t& pub, uint64_t now) {
        if (quit || (duration > 0 && (now - start) / 1e6 >= duration))
            return false;
        if (now >= next_hand) {
            aviator::Message state;
            {
                std::lock_guard<std::mutex> lock(*device.physicsMutex());
                state = sim.handState(aviator::monotonic_us(), hand_publisher);
            }
            aviator::publishMessage(pub, state);
            next_hand = now + 10000;
        }
        {
            aviator::Message detection;
            {
                std::lock_guard<std::mutex> lock(detection_mutex);
                detection = latest_detection;
            }
            if (detection.header.sequence != detection_sequence) {
                aviator::publishMessage(pub, detection);
                detection_sequence = detection.header.sequence;
            }
        }
        return true;
    };
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    aviator::Logger::info("Simulation model={} hands={}", model, hand_publisher);
    // Networking/lifecycle and physics continue while EGL/GLFW renders on the main thread.
    auto server = std::async(std::launch::async, [&] {
        return aviator::runDeviceServer(device, config, settings, options, stopping);
    });
    try {
        uint64_t next_camera = 0, next_view = 0;
        while (!stopping &&
               server.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            const auto now = aviator::monotonic_us();
            if (now >= next_camera) {
                if (viewer)
                    glfwMakeContextCurrent(nullptr);
                const bool in_roi = camera && camera->capture(sim, device.physicsMutex());
                aviator::Message detection;
                if (camera) detection = camera->detection(in_roi);
                else {
                    std::lock_guard<std::mutex> lock(*device.physicsMutex());
                    detection = sim.detection(aviator::monotonic_us(), false, false);
                }
                if (camera_output) camera_output->submit(*camera, detection);
                if (camera_view) camera_view->update(camera->rgb());
                {
                    std::lock_guard<std::mutex> lock(detection_mutex);
                    latest_detection = std::move(detection);
                }
                next_camera = aviator::monotonic_us() + 33333;
            }
            if (viewer && now >= next_view) {
                glfwMakeContextCurrent(viewer->window());
                if (!viewer->draw(sim.data(), *device.physicsMutex()))
                    break;
                if (camera_view) camera_view->draw();
                next_view = aviator::monotonic_us() + 16667;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (...) {
        quit = true;
        server.wait();
        throw;
    }
    quit = true;
    return server.get();
} catch (const zmq::error_t& e) {
    if (stopping && e.num() == EINTR)
        return 0;
    aviator::Logger::error("simulation: {}", e.what());
    return 1;
} catch (const std::exception& e) {
    aviator::Logger::error("simulation: {}", e.what());
    return 1;
}

#include "GraspWheelInput.hpp"
#include <atomic>
#include <future>
#include <iostream>

using namespace aviator;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
template<class F> void rejects(F action, const std::string& reason) {
    try { action(); }
    catch (const std::exception& e) {
        check(std::string(e.what()).find(reason) != std::string::npos, e.what());
        return;
    }
    throw std::runtime_error("Expected rejection: " + reason);
}

int main() try {
    const auto defaults = GraspWheelConfig::load(YAML::Load("{}"));
    check(!defaults.camera, "Omitted config changed legacy behavior");
    const auto configured = GraspWheelConfig::load(YAML::Load(
        "grasp_wheel: {source: camera, camera_id: other, max_age_ms: 100, wait_timeout_ms: 500}"));
    check(configured.camera && configured.camera_id == "other" && configured.freshness_us == 100000 &&
          configured.wait_us == 500000, "Camera config ignored");
    for (const auto* invalid : {"[]", "{source: typo}", "{camera_id: ''}", "{max_age_ms: -1}",
                               "{max_age_ms: 1001}", "{wait_timeout_ms: 10001}",
                               "{max_age_ms: 500, wait_timeout_ms: 100}"})
        rejects([&] { GraspWheelConfig::load(YAML::Load(std::string("grasp_wheel: ") + invalid)); }, "grasp_wheel");

    zmq::context_t context(1);
    zmq::socket_t pub(context, zmq::socket_type::pub);
    configure(pub);
    const std::string endpoint = "inproc://camera-grasp-test";
    pub.bind(endpoint);
    uint64_t sequence = 0;
    const auto publish = [&](uint64_t sample, double theta, double travel, const char* camera = "cockpit") {
        Message m;
        m.topic = Topic::camera_detection;
        m.header.publisher_id = "camera";
        m.header.session_id = "camera-test";
        m.header.clock_id = local_clock_id();
        m.header.sequence = ++sequence;
        m.header.sample_mono_us = sample;
        m.header.valid = true;
        m.body = {{"camera_id", camera}, {"steering_wheel", {{"valid", true},
            {"theta_rad", theta}, {"translation_along_axis_m", travel}, {"axis_match", false}}}};
        std::string payload, error;
        check(encode(m, payload, error), error.c_str());
        send(pub, "camera.detection", payload);
    };
    // A previous grasp's measurement is still recent, but predates the new request.
    const auto old_sample = monotonic_us();
    publish(old_sample, -.1, .01);
    for (int attempt = 0; attempt < 2; ++attempt) {
        std::atomic<int> supervised{0};
        auto result = std::async(std::launch::async, [&] {
            return acquireGraspWheel(context, endpoint, "cockpit", 200000, 1000000,
                                     [&] { ++supervised; });
        });
        for (int i = 0; i < 30; ++i) {
            publish(old_sample, -.1, .01); // Republished with a new sequence, still an old acquisition.
            publish(monotonic_us(), -.7, .04, "other-camera");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        check(result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready,
              "Pre-request or foreign observation satisfied grasp");
        const double theta = attempt ? .2 : -.15, travel = attempt ? -.02 : .04;
        for (int i = 0; i < 100 && result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) {
            publish(monotonic_us(), theta, travel);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        const auto pose = result.get();
        check(std::abs(pose[0] + theta) < 1e-12 && std::abs(pose[1] + travel + .085) < 1e-12,
              "Wrong camera grasp conversion or reused previous grasp pose");
        check(supervised > 2, "No supervision during acquisition");
    }
    rejects([&] { acquireGraspWheel(context, endpoint, "cockpit", 20000, 40000, [] {}); }, "timed out");
    int checks = 0;
    rejects([&] { acquireGraspWheel(context, endpoint, "cockpit", 20000, 1000000, [&] {
        if (++checks == 3) throw std::runtime_error("cancelled by supervisor");
    }); }, "cancelled by supervisor");
    std::cout << "Camera grasp config, on-demand subscription, fresh frames, retry, timeout and cancellation passed\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}

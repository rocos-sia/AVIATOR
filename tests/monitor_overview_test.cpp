#include "assets.hpp"
#include "monitor.hpp"
#include "preview.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using monitor::Json;
namespace {
void check(bool ok, const char* reason) {
    if (!ok)
        throw std::runtime_error(reason);
}
std::string wire(aviator::Topic topic, Json body, std::uint64_t time = 1000000, bool valid = true,
                 std::string publisher = "",
                 std::string session = "11111111-1111-4111-8111-111111111111",
                 unsigned sequence = 1) {
    aviator::Message m;
    m.topic = topic;
    if (publisher.empty()) {
        if (topic == aviator::Topic::arm_state)
            publisher = "manipulator";
        else if (topic == aviator::Topic::hand_state)
            publisher = "inspire_hand";
        else if (topic == aviator::Topic::camera_detection)
            publisher = "camera";
        else
            publisher = "flight_gateway";
    }
    m.header = {"1.0", sequence, 1, time, "clock", publisher, session, valid};
    m.body = std::move(body);
    std::string result, error;
    check(aviator::encode(m, result, error), "fixture encode");
    return result;
}
Json pose() { return {{"position_m", {0, 0, 0}}, {"quaternion_xyzw", {0, 0, 0, 1}}}; }
Json calibration() {
    return {
        {"id", "test-only"},
        {"aircraft_camera", pose()},
        {"tag_yoke", pose()},
        {"aircraft_yoke_zero", pose()},
        {"roll_axis", {0, 0, 1}},
        {"pitch_axis", {0, 0, 1}},
        {"pitch_zero_mm", 85},
        {"min_confidence", .5},
        {"max_rotation_residual_deg", 2},
        {"max_translation_residual_mm", 2},
        {"model",
         {{"roll_sign", 1}, {"roll_offset_rad", 0}, {"pitch_sign", -1}, {"pitch_offset_m", 0}}}};
}
Json detection(double roll, double pitch) {
    double rad = roll * 3.14159265358979323846 / 180;
    return {{"camera_id", "cockpit"},
            {"frame_id", 5},
            {"status", "TRACKING"},
            {"confidence", .9},
            {"pose",
             {{"position", {{"x", 0}, {"y", 0}, {"z", (pitch - 85) / 1000}}},
              {"orientation",
               {{"qx", 0}, {"qy", 0}, {"qz", std::sin(rad / 2)}, {"qw", std::cos(rad / 2)}}}}}};
}
void ingest(monitor::State& s, aviator::Topic t, Json b, std::uint64_t sample = 1000000,
            bool valid = true) {
    s.ingest(std::string(aviator::topic_name(t)), wire(t, b, sample, valid), 1000000);
}
} // namespace
int main(int argc, char** argv) {
    try {
        monitor::State state;
        check(state.overview(1000000, "clock")["system"]["current"].is_null(), "empty overview");
        ingest(state, aviator::Topic::flight_command,
               {{"source", "JOYSTICK"}, {"control", {{"roll", .35}, {"pitch", -.12}}}});
        auto command = state.overview(1010000, "clock")["flight_command"];
        check(command["current"]["roll_percent"] == 35 &&
                  std::abs(command["current"]["pitch_mm"].get<double>() - 74.8) < 1e-9,
              "command mapping");
        check(state.overview(1100000, "clock")["flight_command"]["current"].is_null(),
              "stale command remains current");
        check(state.overview(1010000, "other")["flight_command"]["measurement_state"] ==
                  "CLOCK_UNKNOWN",
              "cross clock");
        Json side = {{"valid", true},
                     {"sample_mono_us", 1000000},
                     {"joint_position", {.1, .2, .3, .4, .5, .6, .7}},
                     {"enabled", true}};
        Json right = side;
        right["valid"] = false;
        right["sample_mono_us"] = 800000;
        ingest(state, aviator::Topic::arm_state, {{"arms", {{"left", side}, {"right", right}}}},
               800000, false);
        auto o = state.overview(1010000, "clock");
        check(o["arms"]["left"]["current"]["joint_position_rad"].size() == 7,
              "old aggregate masks fresh side");
        check(o["arms"]["right"]["current"].is_null(), "invalid side drives model");
        Json hand = {{"valid", true},
                     {"feedback_available", true},
                     {"sample_mono_us", 900000},
                     {"feedback_age_ms", 100},
                     {"joint_position", nullptr},
                     {"joint_velocity", nullptr},
                     {"commanded_drive_position_normalized", {.9, .9, .9, .9, .9, .9}},
                     {"drive_position_raw", {100, 200, 300, 400, 500, 600}},
                     {"drive_position_normalized", {.1, .2, .3, .4, .5, .6}}};
        Json invalid = hand;
        invalid["valid"] = false;
        ingest(state, aviator::Topic::hand_state, {{"hands", {{"left", hand}, {"right", invalid}}}},
               1000000, false);
        o = state.overview(1090000, "clock");
        check(o["hands"]["left"]["measurement_state"] == "VALID", "10Hz hand marked stale");
        const auto& hand_pose = o["hands"]["left"]["current"];
        check(hand_pose["pose_state"] == "ESTIMATED" &&
                  hand_pose["pose_mapping"] == "URDF_LIMITS" &&
                  hand_pose["drive_position_normalized"] == hand["drive_position_normalized"] &&
                  hand_pose["model_joints"].empty(),
              "default hand pose must map actual feedback using loaded URDF limits");
        check(o["hands"]["right"]["current"].is_null(), "invalid hand current");
        check(state.overview(1200000, "clock")["hands"]["left"]["current"].is_null(),
              "outer new stamp hides old hand sample");
        for (const auto& bad : {Json::array(), Json{0, 0, 0, 0, 0, 1.1},
                               Json{0, 0, 0, 0, 0, nullptr}, Json{.9, .9, .9, .9, .9, .9}}) {
            monitor::State bad_feedback;
            auto broken = hand;
            broken["drive_position_normalized"] = bad;
            ingest(bad_feedback, aviator::Topic::hand_state,
                   {{"hands", {{"left", broken}, {"right", hand}}}});
            const auto hands = bad_feedback.overview(1090000, "clock")["hands"];
            check(hands["left"]["current"].is_null() &&
                      hands["right"]["current"]["pose_mapping"] == "URDF_LIMITS",
                  "invalid feedback must not animate hand or mask healthy side");
        }
        monitor::State held;
        Json hold = {{"source", "JOYSTICK"},
                     {"control", {{"roll", .35}, {"pitch", -.12}}},
                     {"input_state",
                      {{"mode", "POSITION_HOLD"},
                       {"device_connected", true},
                       {"checked_mono_us", 1000000}}}};
        ingest(held, aviator::Topic::flight_command, hold, 500000);
        auto hold_view = held.overview(1010000, "clock")["flight_command"];
        check(hold_view["measurement_state"] == "VALID" && hold_view["sample_age_ms"] == 510 &&
                  held.snapshot(1010000, "clock")["streams"][0]["status"] == "FRESH",
              "position-hold check time or original age lost");
        check(held.overview(1100000, "clock")["flight_command"]["current"].is_null(),
              "position hold did not expire");
        Json curves = Json::object();
        curves["id"] = "test-hand";
        for (const auto* side_name : {"left", "right"}) {
            curves[side_name] = Json::array();
            for (const auto* finger :
                 {"thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "little_1"})
                curves[side_name].push_back(
                    {{"joint", std::string(side_name) + "_" + finger + "_joint"},
                     {"knots", {{0, .5}, {1, 0}}}});
        }
        state.config["hand_calibration"] = curves;
        monitor::validate_config(state.config);
        auto mapped_hand = state.overview(1090000, "clock")["hands"]["left"];
        check(mapped_hand["current"]["pose_state"] == "ESTIMATED" &&
                  mapped_hand["current"]["pose_mapping"] == "CALIBRATED" &&
                  std::abs(
                      mapped_hand["current"]["model_joints"]["left_thumb_1_joint"].get<double>() -
                      .45) < 1e-9,
              "drive-to-joint calibration not applied");
        auto invalid_config = state.config;
        invalid_config["hand_calibration"]["left"][0]["knots"] = {{1, 0}, {0, .5}};
        bool rejected_config = false;
        try {
            monitor::validate_config(invalid_config);
        } catch (const std::exception&) {
            rejected_config = true;
        }
        check(rejected_config, "nonmonotonic calibration accepted");
        monitor::State vision;
        ingest(vision, aviator::Topic::camera_detection, detection(17, 73.1));
        check(vision.overview(1010000, "clock")["yoke_observation"]["measurement_state"] ==
                  "UNCALIBRATED",
              "unconfigured pose converted");
        vision.config["yoke_calibration"] = calibration();
        monitor::validate_config(vision.config);
        auto y = vision.overview(1010000, "clock")["yoke_observation"]["current"];
        check(std::abs(y["roll_percent"].get<double>() - 34) < 1e-8 &&
                  std::abs(y["pitch_percent"].get<double>() + 14) < 1e-8,
              "visual mapping");
        monitor::State edges;
        edges.config["yoke_calibration"] = calibration();
        ingest(edges, aviator::Topic::camera_detection, detection(51.9, 174.9));
        check(edges.overview(1010000, "clock")["yoke_observation"]["current"]["roll_percent"] > 100,
              "feedback overshoot clipped");
        monitor::State outside;
        outside.config["yoke_calibration"] = calibration();
        ingest(outside, aviator::Topic::camera_detection, detection(53, 176));
        check(outside.overview(1010000, "clock")["yoke_observation"]["current"].is_null(),
              "out of range feedback accepted");
        monitor::State other_camera;
        other_camera.config["yoke_calibration"] = calibration();
        auto different_camera = detection(17, 73.1);
        different_camera["camera_id"] = "other";
        ingest(other_camera, aviator::Topic::camera_detection, different_camera);
        check(other_camera.overview(1010000, "clock")["yoke_observation"]["current"].is_null(),
              "wrong camera calibration applied");
        auto searching = detection(0, 85);
        searching["status"] = "SEARCHING";
        searching["pose"] = nullptr;
        monitor::State search;
        ingest(search, aviator::Topic::camera_detection, searching, 1000000, false);
        o = search.overview(1010000, "clock");
        check(o["publishers"][4]["state"] == "FRESH" && o["yoke_observation"]["current"].is_null(),
              "publication confused with detection");
        auto second = wire(aviator::Topic::camera_detection, detection(10, 80), 1000000, true,
                           "camera", "22222222-2222-4222-8222-222222222222");
        vision.ingest("camera.detection", second, 1000000);
        check(vision.overview(1010000, "clock")["yoke_observation"]["measurement_state"] ==
                  "SOURCE_CONFLICT",
              "multiple live sessions silently selected");
        vision.config["sources"]["camera.detection"] = {
            {"publisher_id", "camera"}, {"session_id", "11111111-1111-4111-8111-111111111111"}};
        monitor::validate_config(vision.config);
        check(vision.overview(1010000, "clock")["yoke_observation"]["measurement_state"] == "SOURCE_CONFLICT",
              "legacy session pin hid a source conflict");
        // Camera-derived motion works without monitor geometry calibration and
        // takes precedence over legacy pose/model transforms when configured.
        auto wheel_detection = detection(-20, 10);
        wheel_detection["steering_wheel"] = {{"valid", true}, {"theta_rad", .25},
                                             {"translation_along_axis_m", 0},
                                             {"axis_match", nullptr},
                                             {"calibration_id", "camera-test"}};
        for (bool legacy : {false, true}) {
            for (const auto& [travel, expected_pitch] :
                 std::array<std::array<double, 2>, 5>{{{-.1, .015}, {-.085, 0.},
                                                       {0., -.085}, {.085, -.170}, {.1, -.185}}}) {
                monitor::State direct;
                if (legacy)
                    direct.config["yoke_calibration"] = calibration();
                wheel_detection["steering_wheel"]["translation_along_axis_m"] = travel;
                ingest(direct, aviator::Topic::camera_detection, wheel_detection);
                auto observed = direct.overview(1010000, "clock")["yoke_observation"];
                const auto& current = observed["current"];
                check(observed["measurement_state"] == "VALID" &&
                          observed["calibration_id"] == "camera-test" &&
                          current["model_joints"]["roll_input_joint"] == -.25 &&
                          std::abs(current["model_joints"]["pitch_input_joint"].get<double>() -
                                   expected_pitch) < 1e-9 &&
                          std::abs(current["pitch_mm"].get<double>() - (travel + .085) * 1000) < 1e-9 &&
                          std::abs(current["pitch_percent"].get<double>() - travel / .085 * 100) < 1e-9 &&
                          current["pose_mapping"] == "CAMERA_STEERING_WHEEL",
                      "camera wheel mapping or calibration precedence");
                check(direct.overview(1200000, "clock")["yoke_observation"]["current"].is_null(),
                      "stale camera wheel drives model");
            }
        }
        // Trust valid camera motion despite diagnostic axis/range warnings,
        // preview identity, capture latency, or a different producer clock.
        for (const auto sample : {500000u, 1000000u, 2000000u}) {
            for (const auto* clock : {"clock", "other-clock"}) {
                monitor::State live_camera;
                auto incoming = wheel_detection;
                incoming["camera_id"] = "different-from-preview";
                incoming["status"] = "SEARCHING";
                incoming["steering_wheel"].update(
                    {{"theta_rad", 1.2}, {"translation_along_axis_m", .12}, {"axis_match", false}});
                ingest(live_camera, aviator::Topic::camera_detection, incoming, sample);
                auto observed = live_camera.overview(1010000, clock)["yoke_observation"];
                check(observed["measurement_state"] == "VALID" &&
                          observed["fresh_for_ms"] == 190 &&
                          observed["current"]["model_joints"]["roll_input_joint"] == -1.2 &&
                          std::abs(observed["current"]["model_joints"]["pitch_input_joint"].get<double>() + .205) < 1e-9,
                      "valid received camera motion blocked by display diagnostics");
                incoming["steering_wheel"]["theta_rad"] = -1.3;
                live_camera.ingest("camera.detection",
                    wire(aviator::Topic::camera_detection, incoming, sample, true, "camera",
                         "11111111-1111-4111-8111-111111111111", 2), 1190000);
                observed = live_camera.overview(1200000, clock)["yoke_observation"];
                check(observed["measurement_state"] == "VALID" &&
                          observed["current"]["model_joints"]["roll_input_joint"] == 1.3,
                      "subsequent valid detection did not refresh camera pose");
                check(live_camera.overview(1390000, clock)["yoke_observation"]["current"].is_null(),
                      "camera display did not expire after reception stopped");
            }
        }
        monitor::State invalid_detection;
        ingest(invalid_detection, aviator::Topic::camera_detection, wheel_detection, 1000000, false);
        check(invalid_detection.overview(1010000, "clock")["yoke_observation"]["current"].is_null(),
              "invalid detection drove camera pose");
        for (const Json& patch : {
                 Json{{"valid", false}}, Json{{"theta_rad", nullptr}},
                 Json{{"translation_along_axis_m", "0"}}}) {
            monitor::State bad_wheel;
            bad_wheel.config["yoke_calibration"] = calibration();
            auto bad_detection = wheel_detection;
            bad_detection["steering_wheel"].update(patch);
            ingest(bad_wheel, aviator::Topic::camera_detection, bad_detection);
            auto rejected = bad_wheel.overview(1010000, "clock")["yoke_observation"];
            check(rejected["measurement_state"] == "INVALID" && rejected["current"].is_null(),
                  "invalid camera wheel fell back to raw pose");
        }
        monitor::Preview preview(state.config["preview"]);
        std::ifstream image(argv[1], std::ios::binary);
        std::string jpeg((std::istreambuf_iterator<char>(image)), {});
        Json meta = {{"version", 1},
                     {"encoding", "jpeg"},
                     {"publisher_id", "camera"},
                     {"camera_id", "cockpit"},
                     {"session_id", "11111111-1111-4111-8111-111111111111"},
                     {"clock_id", "clock"},
                     {"sequence", 1},
                     {"frame_id", 1},
                     {"sample_mono_us", 1000000},
                     {"width", 16},
                     {"height", 16}};
        preview.ingest("camera.rgb.cockpit", meta.dump(), jpeg, 1000000);
        auto latest = preview.latest(1010000, "clock");
        check(latest["state"] == "FRESH", "JPEG rejected");
        const auto token = latest["current"]["token"].get<std::string>();
        check(preview.frame(token) == jpeg, "frame bytes changed");
        preview.ingest("camera.rgb.cockpit", meta.dump(), jpeg, 1400000);
        check(preview.latest(1500000, "clock")["state"] == "STALE",
              "duplicate image refreshed age");
        for (unsigned i = 2; i < 8; ++i) {
            meta["sequence"] = i;
            meta["frame_id"] = i;
            preview.ingest("camera.rgb.cockpit", meta.dump(), jpeg, 1000000 + i);
        }
        check(preview.latest(1010000, "clock")["cached_frames"] == 4 &&
                  preview.frame(token).empty(),
              "image cache eviction");
        meta["width"] = 999;
        preview.ingest("camera.rgb.cockpit", meta.dump(), jpeg, 1000000);
        check(preview.latest(1010000, "clock")["rejected"] == 1,
              "JPEG dimension mismatch accepted");
        monitor::Assets assets(argv[2], argv[3], state.config);
        monitor::Asset asset;
        check(assets.manifest["resource_errors"].empty(), "model resource missing");
        check(assets.read("/models/urdf/aviator.urdf", asset), "URDF route missing");
        check(!assets.read("/models/../config/flight.yaml", asset) &&
                  !assets.read("/assets/../main.cpp", asset),
              "path traversal");
        check(assets.manifest["aliases"]["/models/meshes/aircraft.STL"] ==
                  "/models/meshes/Cessna/aircraft.STL",
              "resource alias");
        std::cout << "monitor overview, calibration, preview and asset tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

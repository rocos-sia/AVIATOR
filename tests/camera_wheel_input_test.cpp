#include "CameraWheelInput.hpp"
#include <iostream>
#include <limits>

using namespace aviator;
static void check(bool value, const char* reason) {
    if (!value) throw std::runtime_error(reason);
}
static void near(double actual, double expected) {
    check(std::abs(actual - expected) < 1e-12, "coordinate mapping mismatch");
}
static Message observation(uint64_t seq, uint64_t sample, double theta = .3, double travel = 0) {
    Message m;
    m.topic = Topic::camera_detection;
    m.header.publisher_id = "camera";
    m.header.session_id = "camera-run";
    m.header.clock_id = "local";
    m.header.sequence = seq;
    m.header.sample_mono_us = sample;
    m.header.valid = true;
    m.body = {{"camera_id", "cockpit"}, {"steering_wheel", {
        {"valid", true}, {"theta_rad", theta}, {"translation_along_axis_m", travel}}}};
    return m;
}
int main() try {
    std::string error;
    for (const double travel : {-.085, 0., .085}) {
        CameraWheelInput input("cockpit", "local", 200000);
        auto m = observation(1, 1000000, .3, travel);
        // Exercise the actual common wire codec too.
        std::string payload;
        Message decoded;
        check(encode(m, payload, error), error.c_str());
        check(decode("camera.detection", payload, decoded, error), error.c_str());
        check(input.accept(decoded, 1000000, error), error.c_str());
        near(input.target().angle, -.3);
        near(input.target().displacement, -travel - .085);
        check(!input.target().limited, "valid endpoints should not clamp");
        check(input.fresh(1199999), "premature timeout");
        check(!input.fresh(1200000), "stale cached target remained usable");
    }
    for (const double sign : {-1., 1.}) {
        CameraWheelInput input("cockpit", "local", 200000);
        check(input.accept(observation(1, 1000000, sign * 2, sign), 1000000, error), error.c_str());
        near(input.target().angle, -sign * .87266);
        near(input.target().displacement, sign > 0 ? -.170 : 0);
        check(input.target().limited, "out-of-range target not marked limited");
    }
    // Each bad observation must revoke a previously live target immediately.
    for (int kind = 0; kind < 12; ++kind) {
        CameraWheelInput input("cockpit", "local", 200000, 900000);
        check(input.accept(observation(1, 1000000), 1000000, error), error.c_str());
        auto m = observation(2, 1010000);
        switch (kind) {
        case 0: m.header.valid = false; break;
        case 1: m.body["steering_wheel"]["valid"] = false; break;
        case 2: m.body.erase("steering_wheel"); break;
        case 3: m.body["steering_wheel"]["theta_rad"] = "bad"; break;
        case 4: m.body["steering_wheel"]["translation_along_axis_m"] = nullptr; break;
        case 5: m.body["steering_wheel"]["theta_rad"] = std::numeric_limits<double>::infinity(); break;
        case 6: m.header.sample_mono_us = 500000; break;
        case 7: m.header.sample_mono_us = 1010001; break;
        case 8: m.header.sequence = 1; break;
        case 9: m.header.sample_mono_us = 999999; break;
        case 10: m.header.clock_id = "foreign"; break;
        case 11: m.body["steering_wheel"] = nullptr; break;
        }
        check(!input.accept(m, 1010000, error), "bad observation accepted");
        check(!input.fresh(1010000), "invalid observation left old target active");
        check(input.accept(observation(3, 1020000), 1020000, error), "fresh input did not recover");
    }
    // Invalid observations still advance the receive watermark. A delayed
    // valid frame or replay of that invalid frame's sequence must not resume.
    for (int kind = 0; kind < 4; ++kind) {
        CameraWheelInput ordered("cockpit", "local", 200000);
        check(ordered.accept(observation(1, 1000000), 1000000, error), error.c_str());
        auto bad = observation(3, 1030000);
        if (kind == 0) bad.header.valid = false;
        if (kind == 1) bad.body["steering_wheel"]["valid"] = false;
        if (kind == 2) bad.body.erase("steering_wheel");
        if (kind == 3) bad.body["steering_wheel"]["theta_rad"] = "bad";
        check(!ordered.accept(bad, 1030000, error), "invalid frame accepted");
        check(!ordered.accept(observation(2, 1020000), 1040000, error), "older valid frame resumed motion");
        check(!ordered.accept(observation(3, 1040000), 1040000, error), "invalid sequence replay resumed motion");
        check(!ordered.fresh(1040000), "invalid watermark did not revoke target");
        check(ordered.accept(observation(4, 1050000), 1050000, error), "new valid frame did not recover");
    }
    CameraWheelInput input("cockpit", "local", 200000, 1000000);
    check(!input.accept(observation(1, 999999), 1000000, error), "pre-start frame accepted");
    check(input.accept(observation(2, 1000000), 1000000, error), error.c_str());
    for (int kind = 0; kind < 3; ++kind) {
        auto other = observation(3, 1010000);
        if (kind == 0) other.header.publisher_id = "simulation";
        if (kind == 1) other.body["camera_id"] = "other-camera";
        if (kind == 2) other.topic = Topic::hand_state;
        check(!input.accept(other, 1010000, error), "foreign source accepted");
        check(input.fresh(1010000), "foreign source revoked selected camera");
    }
    auto restarted = observation(1, 1020000);
    restarted.header.session_id = "new-camera-run";
    check(input.accept(restarted, 1020000, error), "camera restart rejected");
    check(!input.accept(observation(99, 1010000), 1020000, error), "old session replay accepted");
    // Grasp uses the same units/sign/offset, but must never clamp a measured pose.
    for (int kind = 0; kind < 3; ++kind) {
        CameraWheelInput grasp("cockpit", "local", 200000, 1000000, CameraWheelInput::Mode::Grasp);
        auto m = observation(1, 1000000, -.15, .04);
        m.body["steering_wheel"]["axis_match"] = true;
        check(grasp.accept(m, 1000000, error), error.c_str());
        near(grasp.target().angle, .15);
        near(grasp.target().displacement, -.125);
        m.header.sequence = 2;
        m.header.sample_mono_us = 1010000;
        switch (kind) {
        case 0: m.body["steering_wheel"]["theta_rad"] = 1.; break;
        case 1: m.body["steering_wheel"]["translation_along_axis_m"] = .086; break;
        case 2: m.body["steering_wheel"]["translation_along_axis_m"] = -.086; break;
        }
        check(!grasp.accept(m, 1010000, error), "invalid grasp observation accepted");
        check(!grasp.fresh(1010000), "invalid grasp observation left cached target usable");
        m = observation(3, 1020000, 0, 0);
        m.body["steering_wheel"]["axis_match"] = nullptr;
        check(grasp.accept(m, 1020000, error), "zero pose with unknown axis rejected");
        near(grasp.target().angle, 0);
        near(grasp.target().displacement, -.085);
    }
    // Axis diagnostics must not block an otherwise valid grasp observation.
    for (int kind = 0; kind < 3; ++kind) {
        CameraWheelInput grasp("cockpit", "local", 200000, 1000000, CameraWheelInput::Mode::Grasp);
        auto m = observation(1, 1000000, .082321, .002622);
        if (kind == 0) m.body["steering_wheel"]["axis_match"] = false;
        if (kind == 1) m.body["steering_wheel"]["axis_match"] = nullptr;
        // kind 2 deliberately omits the diagnostic field.
        check(grasp.accept(m, 1000000, error), "Axis diagnostic blocked valid grasp input");
        near(grasp.target().angle, -.082321);
        near(grasp.target().displacement, -.087622);
        m.header.sequence = 2;
        m.header.sample_mono_us = 1010000;
        m.body["steering_wheel"]["valid"] = false;
        check(!grasp.accept(m, 1010000, error), "Invalid camera observation accepted");
        check(!grasp.fresh(1010000), "Invalid observation left grasp target active");
    }
    std::cout << "Camera mapping, clamping, wire decoding, validity, freshness and replay tests passed\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}

#include "../nodes/manipulator/OpenLoopGrasp.hpp"
#include "motion.hpp"
#include <iostream>
#include <fstream>
#include <limits>
using namespace aviator;
void check(bool ok, const char *why) {
    if (!ok)
        throw std::runtime_error(why);
}
template <class F> void rejected(F f) {
    try {
        f();
    } catch (const std::exception &) {
        return;
    }
    throw std::runtime_error("Invalid input accepted");
}
int main() {
    try {
        const auto session = new_session_id(), epoch = new_session_id();
        for (const auto [pitch, expected] : {std::pair<double,double>{-1, -.170}, {0, -.085}, {1, 0}, {-.5, -.1275}, {.5, -.0425}})
            check(std::abs(joystickWheelDisplacement(pitch) - expected) < 1e-12, "pitch mapping");
        rejected([] { joystickWheelDisplacement(1.01); });
        rejected([] { joystickWheelDisplacement(std::numeric_limits<double>::quiet_NaN()); });
        const auto robot_file = std::filesystem::temp_directory_path() / ("wheel-initial-" + session + ".yaml");
        struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{robot_file};
        std::ofstream(robot_file) << "{}\n";
        check(loadInitialWheel(robot_file).displacement == 0, "legacy initial wheel default changed");
        std::ofstream(robot_file) << "wheel_initial: {angle: 0, displacement: -0.085}\n";
        const auto initial = loadInitialWheel(robot_file);
        check(initial.angle == 0 && initial.displacement == -.085, "wheel initial config");
        for (const char* invalid : {"{angle: 0}", "{angle: 0, displacement: 0.01}",
                                  "{angle: 0, displacement: -0.171}", "{angle: 0.9, displacement: 0}",
                                  "{angle: .nan, displacement: 0}"}) {
            std::ofstream(robot_file) << "wheel_initial: " << invalid << '\n';
            rejected([&] { loadInitialWheel(robot_file); });
        }
        Joints lo, hi, speed;
        lo.fill(-3);
        hi.fill(3);
        speed.fill(1);
        TrajectoryWindow w;
        w.id = 1;
        w.count = 63;
        w.total = 100;
        w.sample = w.origin_sample = w.start = monotonic_us();
        w.sequence = 1;
        for (size_t k = 0; k < w.count; ++k)
            for (size_t j = 0; j < 14; ++j)
                w.frames[k].q[j] = k * .0005 * (j < 7 ? 1 : -1);
        auto m = motionMessage(Topic::arm_command, "aviator_core", session, 1);
        m.body = encodeWindow(w, session, epoch);
        std::string payload, error;
        check(encode(m, payload, error), error.c_str());
        Message decoded;
        check(decode("arm.command", payload, decoded, error), error.c_str());
        const auto actual = decodeWindow(decoded, lo, hi, speed);
        check(actual.count == 63, "Expanded count");
        for (size_t k = 0; k < w.count; ++k)
            for (size_t j = 0; j < 14; ++j)
                check(std::abs(actual.frames[k].q[j] - w.frames[k].q[j]) < 1e-12,
                      "2 ms interpolation changed linear trajectory");
        auto stream = w;
        stream.streaming = true;
        stream.count = servo_window_points;
        stream.first = 1;
        stream.total = 101;
        for (size_t k = 0; k < stream.count; ++k)
            for (size_t j = 0; j < 14; ++j) {
                stream.frames[k].q[j] = .1234567891234567 + k * .000123456789123456;
                stream.frames[k].dq[j] = .1234567891234567;
                stream.frames[k].ddq[j] = -.01234567891234567;
            }
        auto sm = m;
        sm.body = encodeWindow(stream, session, epoch);
        sm.body["config_id"] = std::string(64, 'a');
        check(encode(sm, payload, error), error.c_str());
        check(payload.size() > 65536 && payload.size() <= max_payload_bytes,
              "Full precision Servo window must exercise the enlarged wire budget");
        std::cout << "81-point dual-arm payload: " << payload.size() << " bytes\n";
        Message wire;
        check(decode("arm.command", payload, wire, error), error.c_str());
        const auto restored = decodeWindow(wire, lo, hi, speed);
        check(restored.streaming && restored.count == servo_window_points && restored.first == 1, "Servo tick grid");
        for (size_t k = 0; k < stream.count; ++k)
            check(restored.frames[k].q == stream.frames[k].q && restored.frames[k].dq == stream.frames[k].dq &&
                  restored.frames[k].ddq == stream.frames[k].ddq, "Servo samples changed in transport");
        // Exercise rolling windows on both sides of the old one-hour limit and
        // the 24/48-hour marks without a wall-clock soak or millions of frames.
        for (uint64_t boundary : {3600000ULL, 86400000ULL, 172800000ULL}) {
            for (uint64_t total = boundary - 1; total <= boundary + 1; ++total) {
                auto long_stream = stream;
                long_stream.total = total;
                long_stream.first = total - (long_stream.count - 1);
                auto message = sm;
                message.body = encodeWindow(long_stream, session, epoch);
                message.body["config_id"] = std::string(64, 'a');
                check(encode(message, payload, error), error.c_str());
                Message roundtrip;
                check(decode("arm.command", payload, roundtrip, error), error.c_str());
                const auto window = decodeWindow(roundtrip, lo, hi, speed);
                check(window.streaming && window.first == long_stream.first && window.total == total &&
                          window.count == servo_window_points, "Long-running Servo tick range changed");
                check(window.frames.back().q == long_stream.frames.back().q &&
                          window.frames.back().dq == long_stream.frames.back().dq &&
                          window.frames.back().ddq == long_stream.frames.back().ddq,
                      "Long-running Servo samples changed");
                auto invalid = roundtrip;
                invalid.body["first_tick"] = total - servo_window_points + 2;
                rejected([&] { decodeWindow(invalid, lo, hi, speed); });
                invalid = roundtrip;
                invalid.body["finished"] = true;
                rejected([&] { decodeWindow(invalid, lo, hi, speed); }); // Must finish at rest.
                for (const char* side : {"left", "right"}) {
                    auto& last = invalid.body["arms"][side]["points"].back();
                    last["joint_velocity"] = std::array<double, 7>{};
                    last["joint_acceleration"] = std::array<double, 7>{};
                }
                check(decodeWindow(invalid, lo, hi, speed).finished, "Long-running Servo cannot finish");
            }
        }
        // The bounded, preplanned trajectory budget and uint53 wire range remain enforced.
        auto finite = decoded;
        finite.body["total_ticks"] = 3600000;
        check(!decodeWindow(finite, lo, hi, speed).streaming, "Finite trajectory boundary rejected");
        finite.body["total_ticks"] = 3600002;
        rejected([&] { decodeWindow(finite, lo, hi, speed); });
        auto oversized = wire;
        oversized.body["total_ticks"] = max_json_integer + 1;
        rejected([&] { decodeWindow(oversized, lo, hi, speed); });
        for (auto message : {decoded, wire}) {
            auto& points = message.body["arms"]["left"]["points"];
            points.push_back(points.back());
            rejected([&] { decodeWindow(message, lo, hi, speed); });
        }
        // Servo is allowed past the application dynamic caps; positions and finite values remain checked.
        auto fast = stream;
        for (size_t k = 0; k < fast.count; ++k) {
            fast.frames[k].q.fill(.2 + k * .0025);
            fast.frames[k].dq.fill(2.5);
            fast.frames[k].ddq.fill(25);
        }
        auto fast_message = m;
        fast_message.body = encodeWindow(fast, session, epoch);
        check(decodeWindow(fast_message, lo, hi, speed).frames[1].dq[0] == 2.5,
              "Servo dynamic cap still applied in protocol");
        auto invalid_position = fast_message;
        invalid_position.body["arms"]["left"]["points"][1]["joint_position"][0] = 4;
        rejected([&] { decodeWindow(invalid_position, lo, hi, speed); });
        wire.body["arms"]["left"]["points"][1].erase("joint_acceleration");
        rejected([&] { decodeWindow(wire, lo, hi, speed); });
        auto bad = decoded;
        bad.body["arms"].erase("right");
        rejected([&] { decodeWindow(bad, lo, hi, speed); });
        bad = decoded;
        bad.body["arms"]["right"]["points"][1]["joint_position"][0] = 2;
        rejected([&] { decodeWindow(bad, lo, hi, speed); });
        bad = decoded;
        bad.body["arms"]["left"]["points"][1]["time_from_start_us"] = 1000;
        rejected([&] { decodeWindow(bad, lo, hi, speed); });
        bad = decoded;
        bad.body["mode"] = "JOINT_POSITION";
        rejected([&] { decodeWindow(bad, lo, hi, speed); });
        rejected([] { parseService("{\"a\":1,\"a\":2}"); });
        rejected([] { parseService("[]"); });
        InputPolicy p;
        p.topic = Topic::arm_command;
        p.publisher_id = "aviator_core";
        p.session_id = session;
        p.clock_id = local_clock_id();
        p.control_epoch = epoch;
        p.origin_publisher_id = "aviator_core";
        p.origin_session_id = session;
        InputGuard flight_only(p);
        check(!flight_only.accept(decoded, monotonic_us(), error),
              "Local task bypassed default flight origin policy");
        p.origin_topic = "local.task";
        InputGuard local(p);
        check(local.accept(decoded, monotonic_us(), error), error.c_str());
        check(!local.accept(decoded, monotonic_us(), error), "Repeated sequence accepted");
        decoded.header.sequence++;
        decoded.header.valid = false;
        check(!local.accept(decoded, monotonic_us(), error) && local.expired(monotonic_us()),
              "Invalid did not revoke input");
        check(local.expired(monotonic_us() + 200000), "Watchdog failed");
        OpenLoopGrasp grasp;
        const double position[2] = {1.0, 2.0}, rotation[2] = {2.0, 3.0};
        grasp.update(1.0, 1.0, true, position, rotation, 0);
        grasp.command(GraspCommand::Lock);
        auto gs = grasp.update(2.0, 2.0, true, position, rotation, 0);
        check(gs.locked == 3 && !gs.fault, "Removed TCP deviation gate was restored");
        gs = grasp.update(2.2, 2.0, true, position, rotation, 0);
        check(gs.fault, "Removing grasp gates also removed feedback watchdog");
        std::cout
            << "Motion protocol: dual-arm windows, interpolation, rejection, origin and watchdog passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

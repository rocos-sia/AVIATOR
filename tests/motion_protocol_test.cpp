#include "../nodes/manipulator/OpenLoopGrasp.hpp"
#include "motion.hpp"
#include <iostream>
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

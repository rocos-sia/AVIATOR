#include "tts.hpp"
#include <iostream>
#include <stdexcept>

using namespace aviator;
using namespace aviator::tts;
namespace {
void check(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
Message status(const char* state, int code, uint64_t seq, uint64_t time,
               const char* instance = "core-1") {
    Message m;
    m.topic = Topic::flight_state;
    m.header = {"1.0", seq, 0, time, "clock", "aviator_core", instance, true};
    m.body = {{"system",
               {{"state", state},
                {"state_code", code},
                {"generation", uint64_t(1)},
                {"current_error", ""}}}};
    return m;
}
void expect(Rules& r, uint64_t now, const std::string& id) {
    const auto next = r.next(now);
    check(next && next->id == id, ("expected " + id).c_str());
}
} // namespace
int main(int argc, char** argv) {
    try {
        check(argc == 2, "config path required");
        const auto config = load_config(argv[1]);
        std::string error;
        const uint64_t t = 1000000;
        Rules r(config, "clock", t);
        check(r.ingest(status("READY", 11, 1, t), t, error), "accept managed READY");
        check(!r.next(t), "initial ordinary snapshot silent");
        check(r.ingest(status("CONTROL", 4, 2, t + 20000), t + 20000, error), "CONTROL");
        auto playing = r.next(t + 20000);
        check(playing && playing->id == "CONTROL", "control cue");
        check(r.ingest(status("CONTROL", 4, 3, t + 40000), t + 40000, error), "heartbeat");
        check(r.current(*playing) && !r.next(t + 40000), "heartbeat does not interrupt/repeat");
        check(!r.ingest(status("CONTROL", 4, 3, t + 40000), t + 40000, error),
              "duplicate rejected");
        check(r.ingest(status("FOLLOWING", 3, 4, t + 60000), t + 60000, error), "exit control");
        check(!r.current(*playing), "stale active speech cancelled");
        expect(r, t + 60000, "FOLLOWING_FROM_CONTROL");
        check(r.ingest(status("GRASPING", 2, 5, t + 80000), t + 80000, error), "grasping");
        check(r.ingest(status("EMERGENCY_STOP", 7, 6, t + 100000), t + 100000, error), "emergency");
        check(r.pending_priority(t + 100000) == 3, "emergency priority");
        expect(r, t + 100000, "EMERGENCY_STOP");
        check(!r.next(t + 100000), "obsolete queue removed");
        r.tick(t + 300000);
        check(!r.current(*playing), "freshness timeout");
        r.tick(t + 2300000);
        expect(r, t + 2300000, "COMM_LOST");
        r.tick(t + 2400000);
        check(!r.next(t + 2400000), "loss only once");
        check(r.ingest(status("STANDBY", 1, 1, t + 2500000, "core-2"), t + 2500000, error),
              "restart after loss");
        expect(r, t + 2500000, "COMM_RESTORED");
        expect(r, t + 2500000, "STANDBY");
        check(!r.ingest(status("ERROR", 6, 100, t + 100000, "core-1"), t + 2500000, error),
              "old instance cannot return");

        Rules faults(config, "clock", t);
        auto fault = status("ERROR", 6, 1, t);
        fault.body["system"]["current_error"] = "first";
        check(faults.ingest(fault, t, error), "initial fault");
        expect(faults, t, "ERROR");
        fault.header.sequence++;
        fault.header.sample_mono_us += 20000;
        fault.body["system"]["current_error"] = "second";
        check(faults.ingest(fault, t + 20000, error), "same state new error");
        expect(faults, t + 20000, "ERROR");
        fault.header.sequence++;
        fault.header.sample_mono_us += 20000;
        fault.header.valid = false;
        check(!faults.ingest(fault, t + 40000, error), "invalid clears availability");
        faults.tick(t + 2040000);
        expect(faults, t + 2040000, "COMM_LOST");

        Rules conflict(config, "clock", t);
        check(conflict.ingest(status("READY", 11, 1, t), t, error), "first core");
        check(!conflict.ingest(status("ERROR", 6, 1, t + 20000, "core-2"), t + 20000, error),
              "second active core conflicts");
        expect(conflict, t + 20000, "SOURCE_CONFLICT");
        check(conflict.ingest(status("ERROR", 6, 2, t + 220000, "core-2"), t + 220000, error),
              "recover after old source expires");
        expect(conflict, t + 220000, "ERROR");
        expect(conflict, t + 220000, "COMM_RESTORED");

        Rules guards(config, "clock", t);
        auto bad = status("CONTROL", 4, 1, t);
        bad.body["system"].erase("state_code");
        check(!guards.ingest(bad, t, error), "Direct Core rejected");
        bad = status("CONTROL", 6, 1, t);
        check(!guards.ingest(bad, t, error), "wrong state code rejected");
        bad = status("CONTROL", 4, 1, t);
        bad.header.clock_id = "other";
        check(!guards.ingest(bad, t, error), "cross host rejected");
        bad = status("CONTROL", 4, 1, t + 1);
        check(!guards.ingest(bad, t, error), "future rejected");
        bad = status("CONTROL", 4, 1, t);
        bad.header.publisher_id = "other";
        check(!guards.ingest(bad, t, error), "wrong publisher rejected");
        guards.tick(t + config.lost_after_us);
        check(!guards.next(t + config.lost_after_us), "startup grace avoids launch alarm");
        guards.tick(t + config.startup_grace_us);
        expect(guards, t + config.startup_grace_us, "COMM_LOST");
        Rules ttl(config, "clock", t);
        ttl.ingest(status("ERROR", 6, 1, t), t, error);
        check(!ttl.next(t + config.ttl_us), "expired queued speech dropped");
        std::cout << "TTS rules passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

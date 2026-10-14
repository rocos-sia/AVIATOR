#include "HandControl.hpp"
#include "HandLink.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
using namespace aviator;
void check(bool v, const char* reason) { if (!v) throw std::runtime_error(reason); }
int main(int argc, char** argv) {
    try {
        const auto path = std::filesystem::temp_directory_path() / ("core-hand-" + new_session_id() + ".yaml");
        struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{path};
        std::ofstream(path) << "core_hand:\n  enabled: true\n  completion_timeout_ms: 500\n  close:\n    left: [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]\n    right: [0.3, 0.4, 0.5, 0.6, 0.7, 0.8]\n";
        const auto session = new_session_id(), node = new_session_id();
        auto create = [&] { HandControl h; h.configure(path); return h; };
        auto now = monotonic_us();
        auto h = create(); h.request(true, now);
        const auto command = *h.command(now, now, session);
        check(command.body["hands"]["left"]["drive_position_normalized"][0] == .9, "left mapping");
        check(command.body["hands"]["right"]["drive_position_normalized"][0] == .3, "right mapping");
        std::string payload, error;
        check(encode(command, payload, error), error.c_str());
        if (argc > 1) {
            auto stop = command;
            ++stop.header.sequence;
            ++stop.header.sample_mono_us;
            stop.header.valid = false;
            stop.body["origin"]["sequence"] = stop.header.sequence;
            stop.body["origin"]["sample_mono_us"] = stop.header.sample_mono_us;
            std::string stopped;
            check(encode(stop, stopped, error), error.c_str());
            std::ofstream(argv[1]) << '[' << payload << ',' << stopped << ']';
        }
        auto feedback = [&](const Message& cmd, uint64_t t, uint64_t seq) {
            auto m = motionMessage(Topic::hand_state, "inspire_hand", node, seq);
            m.header.sample_mono_us = t;
            m.body = {{"feedback_only", false}, {"command_valid", true},
                      {"accepted_command", {{"publisher_id", "aviator_core"}, {"session_id", session},
                          {"sequence", cmd.header.sequence}, {"sample_mono_us", cmd.header.sample_mono_us}}}};
            for (const char* side : {"left", "right"})
                m.body["hands"][side] = {{"valid", true}, {"sample_mono_us", t},
                    {"drive_position_normalized", cmd.body["hands"][side]["drive_position_normalized"]}};
            return m;
        };
        auto state = feedback(command, now + 1, 1);
        {
            auto stream = create();
            stream.beginApproach(now);
            auto cmd = *stream.command(now, now, session);
            uint64_t state_seq = 0;
            // Every ACK arrives after its 100 ms validity window, but applied
            // command sequences keep advancing during the synchronized path.
            for (uint64_t elapsed = 120000; elapsed <= 1200000; elapsed += 120000) {
                const auto t = now + elapsed;
                auto m = feedback(cmd, t, ++state_seq);
                m.body["command_valid"] = false;
                stream.receive(m, t, session);
                check(stream.fault(t).empty() && !stream.complete(t),
                      "delayed advancing ACK stopped publication or completed grasp");
                stream.approachProgress({.5, .5}, t);
                auto next = stream.command(t, t, session);
                check(next.has_value(), "advancing delayed ACK stream was revoked");
                cmd = *next;
            }
            // Feedback continues but the device repeats its last applied ACK.
            const auto last_progress = now + 1200000;
            auto repeated = feedback(cmd, last_progress + 1000, ++state_seq);
            repeated.body["command_valid"] = false;
            stream.receive(repeated, last_progress + 1000, session);
            for (uint64_t elapsed = 100000; elapsed <= 600000; elapsed += 100000) {
                auto m = feedback(cmd, last_progress + elapsed, ++state_seq);
                m.body["command_valid"] = false;
                stream.receive(m, last_progress + elapsed, session);
            }
            check(!stream.fault(last_progress + 600000).empty() &&
                  stream.command(last_progress + 600000, last_progress + 600000, session).has_value(),
                  "stalled ACK must remain visible without stopping latest targets");
        }
        // Invalid device measurements must not terminate an otherwise live
        // command/ACK stream, including after the normal completion deadline.
        for (int invalid_field = 0; invalid_field < 3; ++invalid_field) {
            auto continued = create();
            continued.startMonitoring(now);
            continued.request(true, now);
            auto cmd = *continued.command(now, now, session);
            continued.receive(feedback(cmd, now + 1, 1), now + 1, session);
            const auto invalidate = [&](Message& m) {
                if (invalid_field == 0) m.header.valid = false;
                if (invalid_field == 1 || invalid_field == 2) {
                    auto& hand = m.body["hands"][invalid_field == 1 ? "left" : "right"];
                    hand["valid"] = false;
                    hand.erase("drive_position_normalized");
                }
            };
            uint64_t seq = 1;
            for (uint64_t elapsed = 20000; elapsed <= 1200000; elapsed += 20000) {
                const auto t = now + elapsed;
                auto next = continued.command(t, t, session);
                check(next.has_value() && next->header.valid, "invalid feedback stopped valid hand commands");
                cmd = *next;
                auto m = feedback(cmd, t + 1, ++seq);
                invalidate(m);
                continued.receive(m, t + 1, session);
                check(!continued.fresh(t + 1) && !continued.complete(t + 1),
                      "invalid feedback claimed successful completion");
                check(continued.fault(t + 1).empty() && continued.messageFault(t + 1).empty(),
                      "invalid online feedback raised a publication/communication fault");
            }
            check(continued.waitExpired(now + 1200001), "invalid completion wait is not bounded");
            const auto open_at = now + 1220000;
            continued.request(false, open_at);
            for (uint64_t elapsed = 0; elapsed <= 600000; elapsed += 20000) {
                const auto t = open_at + elapsed;
                auto next = continued.command(t, t, session);
                check(next.has_value(), "opening stopped during invalid feedback");
                cmd = *next;
                for (const char* side : {"left", "right"})
                    check(cmd.body["hands"][side]["drive_position_normalized"] == Json({.5,1,1,1,1,1}),
                          "invalid feedback prevented changing to the open target");
                auto m = feedback(cmd, t + 1, ++seq);
                invalidate(m);
                continued.receive(m, t + 1, session);
            }
            const auto recovered_at = open_at + 620000;
            auto recovering = feedback(cmd, recovered_at, ++seq);
            recovering.body["hands"]["left"]["drive_position_normalized"][0] = .9;
            continued.receive(recovering, recovered_at, session);
            check(continued.fresh(recovered_at) && !continued.complete(recovered_at) &&
                  continued.fault(recovered_at).empty(), "recovery inherited the invalid completion deadline");
            auto reopened = continued.command(recovered_at, recovered_at, session);
            check(reopened.has_value(), "recovery stopped command publication");
            continued.receive(feedback(*reopened, recovered_at + 1, ++seq), recovered_at + 1, session);
            check(continued.complete(recovered_at + 1), "recovered opening did not complete");
        }
        {
            auto stale_side = create();
            stale_side.request(true, now);
            auto cmd = *stale_side.command(now, now, session);
            stale_side.receive(feedback(cmd, now + 1, 1), now + 1, session);
            auto m = feedback(cmd, now + 20000, 2);
            m.header.valid = false;
            m.body["hands"]["left"]["sample_mono_us"] = now - 500000;
            stale_side.receive(m, now + 20000, session);
            check(!stale_side.fault(now + 20000).empty() &&
                  stale_side.command(now + 20000, now + 20000, session).has_value(),
                  "stale feedback must remain visible without stopping publication");
        }
        {
            auto startup = create();
            startup.startMonitoring(now);
            startup.request(true, now);
            auto cmd = *startup.command(now, now, session);
            auto m = feedback(cmd, now + 1, 1);
            m.body["command_valid"] = false;
            m.body["accepted_command"] = nullptr;
            startup.receive(m, now + 1, session);
            check(startup.fresh(now + 1) && !startup.complete(now + 1),
                  "startup feedback requires a command ACK or completes motion without one");
            m.header.sequence = 2;
            m.header.sample_mono_us = now + 1000002;
            for (auto& hand : m.body["hands"]) hand["sample_mono_us"] = now + 1000002;
            m.body["command_valid"] = true;
            startup.receive(m, now + 1000002, session);
            check(!startup.messageFault(now + 1000002).empty(), "missing command ACK accepted");
            m.body["command_valid"] = false;
            m.header.valid = false;
            for (auto& hand : m.body["hands"]) hand["valid"] = false;
            startup.receive(m, now + 1000002, session);
            check(startup.messageFault(now + 1000002).empty() && !startup.fresh(now + 1000002),
                  "invalid startup feedback was treated as missing messages");
        }
        {
            auto protected_hand = create();
            protected_hand.request(false, now);
            auto cmd = *protected_hand.command(now, now, session);
            auto m = feedback(cmd, now + 1, 1);
            m.body["hold_control_error"] = "left[0]: device error 4";
            m.body["hands"]["left"]["drive_position_normalized"][1] = 0;
            protected_hand.receive(m, now + 1, session);
            check(protected_hand.fresh(now + 1) && !protected_hand.complete(now + 1),
                  "historical hand error invalidated measurements or bypassed actual opening");
            m = feedback(cmd, now + 2, 2);
            m.body["hold_control_error"] = "left[0]: device error 4";
            protected_hand.receive(m, now + 2, session);
            check(protected_hand.complete(now + 2), "latched historical error blocked recovered opening");
            check(protected_hand.body()["hold_control_error"] == "left[0]: device error 4",
                  "historical device diagnostics were discarded");
        }
        {
            auto monitored = create();
            monitored.startMonitoring(now);
            check(monitored.messageFault(now + 1000000).empty(), "message timeout before >1s");
            check(!monitored.messageFault(now + 1000001).empty(), "missing startup messages not detected");
            auto m = feedback(command, now + 1000002, 1);
            m.header.valid = false;
            for (const char* side : {"left", "right"}) m.body["hands"][side]["valid"] = false;
            monitored.receive(m, now + 1000002, session);
            check(!monitored.fresh(now + 1000002), "invalid feedback unexpectedly valid");
            check(monitored.messageFault(now + 2000002).empty(), "invalid messages treated as missing");
            monitored.revoke();
            check(!monitored.messageFault(now + 2000003).empty(), "revocation hid message timeout");
            auto duplicate = m;
            duplicate.header.sample_mono_us = now + 2000003;
            monitored.receive(duplicate, now + 2000003, session);
            check(!monitored.messageFault(now + 2000003).empty(), "replayed sequence refreshed watchdog");
            auto foreign = feedback(command, now + 2000004, 2);
            foreign.header.publisher_id = "other_hand";
            monitored.receive(foreign, now + 2000004, session);
            check(!monitored.messageFault(now + 2000004).empty(), "foreign publisher refreshed watchdog");
            monitored.receive(feedback(command, now + 2000005, 2), now + 2000005, session);
            check(monitored.messageFault(now + 2000005).empty(), "resumed messages did not clear timeout");
            HandControl disabled;
            disabled.startMonitoring(now);
            check(disabled.messageFault(now + 2000000).empty(), "disabled hand raised message timeout");
        }
        auto wrong = state; wrong.header.publisher_id = "manipulator";
        h.receive(wrong, now + 1, session); check(!h.complete(now + 1), "synthetic feedback accepted");
        h.receive(state, now + 1, session); check(h.complete(now + 1), "close ack rejected");
        auto restarted_hand = state;
        restarted_hand.header.session_id = "hand-restarted";
        restarted_hand.header.sample_mono_us = now + 2;
        restarted_hand.body["accepted_command"]["session_id"] = "ignored-core-label";
        h.receive(restarted_hand, now + 2, session);
        check(h.complete(now + 2) && h.fault(now + 2).empty(), "hand restart or ACK label rejected");
        check(h.command(now + 20000, now + 20000, session).has_value(), "hold publication missing");
        h.request(false, now + 30000);
        auto opening = *h.command(now + 30000, now + 30000, session);
        for (const char* side : {"left", "right"})
            check(opening.body["hands"][side]["drive_position_normalized"] == Json({.5,1,1,1,1,1}),
                  "default opening must keep thumb rotation at register 500");
        auto old_ack = feedback(command, now + 30000, 2);
        h.receive(old_ack, now + 30000, session);
        check(!h.complete(now + 30000), "old target acknowledgement completed new target");
        auto pending = feedback(opening, now + 30001, 3);
        pending.body["hands"]["left"]["drive_position_normalized"][0] = 0;
        h.receive(pending, now + 30001, session); check(!h.complete(now + 30001), "open completed before actual position");
        auto opened = feedback(opening, now + 40000, 4);
        h.receive(opened, now + 40000, session); check(h.complete(now + 40000), "open did not complete");
        // Continuous command/ACK traffic does not imply physical opening completed.
        // Reproduce a release with one finger stuck: publish until the completion
        // deadline, report it, and keep sending the latest target without
        // claiming that the physical opening completed.
        for (bool stuck : {false, true}) {
            auto release = create();
            release.request(true, now);
            auto close = *release.command(now, now, session);
            release.receive(feedback(close, now + 1, 1), now + 1, session);
            check(release.complete(now + 1), "release setup close acknowledgement failed");
            const auto start = now + 20000;
            release.request(false, start);
            uint64_t feedback_sequence = 1;
            unsigned sent = 0;
            for (uint64_t elapsed = 0; elapsed < 500000; elapsed += 20000) {
                const auto t = start + elapsed;
                auto cmd = release.command(t, t, session);
                check(cmd.has_value(), "release command stopped before completion deadline");
                ++sent;
                auto observed = feedback(*cmd, t + 1, ++feedback_sequence);
                if (stuck) observed.body["hands"]["left"]["drive_position_normalized"][0] = .9;
                release.receive(observed, t + 1, session);
                check(release.fresh(t + 1), "release test feedback unexpectedly stale");
                check(release.complete(t + 1) == !stuck, "release actual-position gate incorrect");
            }
            check(sent == 25, "release did not publish at every 20 ms step");
            const auto deadline = start + 500000; // Test configuration uses 500 ms, production 5 s.
            if (stuck) {
                check(release.fault(deadline).find("Hand target timeout") != std::string::npos,
                      "fresh transport with a stuck finger did not hit target timeout");
                check(release.command(deadline, deadline, session).has_value() && !release.complete(deadline),
                      "unreached target stopped publication or claimed completion");
            } else {
                check(release.fault(deadline).empty() && release.command(deadline, deadline, session).has_value(),
                      "completed open target should continue publishing after deadline");
            }
        }
        // An approach may last longer than the ordinary completion timeout;
        // intermediate ACKs keep it healthy but cannot acknowledge its endpoint.
        {
            auto moving = create(); moving.beginApproach(now);
            uint64_t sequence = 0;
            Message partial;
            for (uint64_t dt = 0; dt <= 800000; dt += 20000) {
                const auto t = now + dt;
                moving.approachProgress({double(dt)/1000000, double(dt)/2000000}, t);
                partial = *moving.command(t, t, session);
                const double left = partial.body["hands"]["left"]["drive_position_normalized"][0];
                const double right = partial.body["hands"]["right"]["drive_position_normalized"][0];
                check(std::abs(left - (.5 + .4*double(dt)/1000000)) < 1e-12, "left interpolation");
                check(std::abs(right - (.5 - .2*double(dt)/2000000)) < 1e-12, "right interpolation");
                moving.receive(feedback(partial, t + 1, ++sequence), t + 1, session);
                check(!moving.complete(t + 1) && moving.fault(t + 1).empty(), "partial trajectory completed/timed out");
            }
            const auto end = now + 820000;
            moving.approachProgress({1,1}, end);
            check(!moving.complete(end), "partial ACK completed endpoint");
            auto final = *moving.command(end, end, session);
            moving.receive(feedback(partial, end + 1, ++sequence), end + 1, session);
            check(!moving.complete(end + 1), "old ACK completed endpoint");
            auto invalid = feedback(final, end + 2, ++sequence);
            invalid.body["accepted_command"]["sequence"] = 999999;
            moving.receive(invalid, end + 2, session);
            check(!moving.complete(end + 2), "unrecognized ACK completed endpoint");
            moving.receive(feedback(final, end + 3, ++sequence), end + 3, session);
            check(moving.complete(end + 3), "endpoint ACK rejected");
            moving.revoke();
            check(!moving.command(end + 20000, end + 20000, session), "revoked approach kept closing");
        }
        h.revoke(); check(!h.command(now + 50000, now + 50000, session), "revoked hand still publishing");
        auto expired = create(); expired.request(true, now);
        check(!expired.command(now + 100000, now, session), "stale heartbeat published");
        check(expired.fault(now + 100000).find("age_us=100000") != std::string::npos, "actual heartbeat age missing");
        auto future = create(); future.request(true, now);
        check(!future.command(now, now + 1, session), "invalid clock order accepted");
        check(future.fault(now).find("clock order invalid: ahead_us=1") != std::string::npos,
              "future heartbeat mislabeled as expired");
        for (int failure = 0; failure < 5; ++failure) {
            auto test = create(); test.request(true, now);
            auto cmd = *test.command(now, now, session);
            auto m = feedback(cmd, now + 1, 1);
            if (failure == 0) m.body["feedback_only"] = true;
            if (failure == 1) m.body["accepted_command"]["publisher_id"] = "another-controller";
            if (failure == 2) m.body["accepted_command"]["sequence"] = 999;
            test.receive(m, now + 1, session);
            if (failure == 3) { m.header.sequence = 2; m.header.clock_id = "other-clock"; test.receive(m, now + 2, session); }
            const auto t = failure >= 2 ? now + 600000 : now + 2;
            check(!test.fault(t).empty(), "missing failure detection");
            check(test.command(t, t, session).has_value() == (failure >= 2),
                  "timeouts must stream; readonly/foreign ownership must still block");
        }
        // Reproduce the field failure: a completed synchronized grasp keeps
        // receiving fresh measurements and advancing ACKs, but hold-setting
        // writes make those ACKs older than 100 ms. Completion can become
        // unconfirmed again; this must never revoke the target stream.
        {
            auto holding = create();
            holding.beginApproach(now);
            holding.approachProgress({1,1}, now);
            std::vector<Message> commands;
            for (unsigned i = 0; i < 150; ++i) {
                const auto t = now + uint64_t(i) * 20000;
                auto cmd = holding.command(t, t, session);
                check(cmd.has_value(), "delayed hold ACK stopped a completed grasp's stream");
                commands.push_back(*cmd);
                const auto& ack = commands[i >= 100 ? i - 10 : i];
                holding.receive(feedback(ack, t + 1, i + 1), t + 1, session);
                if (i == 90) check(holding.complete(t + 1), "initial grasp did not complete");
                if (i >= 125) {
                    check(!holding.complete(t + 1) && !holding.failed(),
                          "delayed ACK was falsely completed or latched as failure");
                    check(holding.fault(t + 1).find("Hand target timeout") != std::string::npos,
                          "completion timeout diagnostic missing");
                }
            }
            const auto recovered_at = now + 3000000;
            auto cmd = *holding.command(recovered_at, recovered_at, session);
            holding.receive(feedback(cmd, recovered_at + 1, 151), recovered_at + 1, session);
            check(holding.complete(recovered_at + 1) && holding.fault(recovered_at + 1).empty(),
                  "fresh ACK did not recover without a new request");
        }
        // Missing feedback/ACKs cannot freeze ongoing synchronized interpolation.
        {
            auto silent = create();
            silent.beginApproach(now);
            for (unsigned i = 0; i <= 100; ++i) {
                const auto t = now + uint64_t(i) * 20000;
                const double u = i / 100.;
                silent.approachProgress({u,u}, t);
                auto cmd = silent.command(t,t,session);
                check(cmd.has_value(), "missing feedback stopped synchronized targets");
                const double left = cmd->body["hands"]["left"]["drive_position_normalized"][0];
                check(std::abs(left-(.5+.4*u)) < 1e-12, "missing ACK froze latest target");
                check(!silent.complete(t), "missing ACK claimed completion");
            }
            silent.revoke();
            check(!silent.command(now + 2020000, now + 2020000, session),
                  "revocation must still stop unconfirmed targets");
        }
        // Whole-target versions must cancel obsolete waits, including revoke/shutdown.
        {
            zmq::context_t context(1);
            std::atomic<uint64_t> heartbeat{monotonic_us()};
            HandLink link;
            link.configure(path);
            MotionConfig config;
            config.publish = "inproc://hand-test-pub";
            config.subscribe = "inproc://hand-test-sub";
            link.start(context, config, session, heartbeat, true);
            auto first = link.request(false);
            auto second = link.request(true);
            auto expect = [&](uint64_t version, const std::string& why) {
                bool rejected = false;
                try { link.wait(version); }
                catch (const std::exception& e) { rejected = std::string(e.what()).find(why) != std::string::npos; }
                check(rejected, "obsolete/revoked/stopped wait not cancelled");
            };
            check(!link.wait(first), "superseded hand wait claimed completion");
            link.allow(false);
            expect(second, "revoked");
            link.allow(true);
            link.fail("Hand target timeout from previous request");
            auto recovered = link.beginApproach();
            check(recovered != 0 && link.fault().empty(), "old hand timeout skipped a new synchronized grasp");
            link.approachProgress(recovered, {.3, .6});
            auto third = link.request(false);
            link.stop();
            check(!link.wait(third), "stopped hand worker claimed completion");
            check(link.request(false) == 0, "stopped hand worker accepted target");
        }
        auto sim = HandControl(); sim.configure(path); check(sim.enabled(), "hand control must not depend on device implementation");
        {
            std::ofstream(path) << R"(core_hand:
  enabled: true
  closing_start:
    left: [0.5, 1, 0.6, 0.6, 0.6, 0.6]
    right: [0.5, 1, 0.7, 0.7, 0.7, 0.7]
  close:
    left: [0.5, 0, 0, 0, 0, 0]
    right: [0.5, 0.2, 0.2, 0.2, 0.2, 0.2]
)";
            auto moving = create();
            moving.beginApproach(now);
            const auto poseAt = [&](double left, double right, uint64_t t) {
                moving.approachProgress({left, right}, t);
                return moving.command(t, t, session).value();
            };
            const auto position = [](const Message& m, const char* side, int joint) {
                return m.body["hands"][side]["drive_position_normalized"][joint].get<double>();
            };
            auto cmd = poseAt(0, 0, now);
            check(position(cmd, "left", 2) == 1 && position(cmd, "right", 2) == 1,
                  "closing start moved hands before entering closing range");
            cmd = poseAt(.0001, 0, now + 20000);
            check(std::abs(position(cmd, "left", 2) - .59994) < 1e-12 && position(cmd, "right", 2) == 1,
                  "independent hand did not begin at configured closing start");
            check(std::abs(position(cmd, "left", 1) - .9999) < 1e-12,
                  "thumb flexion did not retain original open starting position");
            cmd = poseAt(.5, .25, now + 40000);
            for (int j = 2; j < 6; ++j)
                check(std::abs(position(cmd, "left", j) - .3) < 1e-12 &&
                      std::abs(position(cmd, "right", j) - .575) < 1e-12,
                      "closing start interpolation incorrect");
            check(position(cmd, "left", 0) == .5 && position(cmd, "right", 0) == .5,
                  "closing start changed thumb rotation");
            check(std::abs(position(cmd, "left", 1) - .5) < 1e-12 &&
                  std::abs(position(cmd, "right", 1) - .8) < 1e-12,
                  "closing start changed thumb flexion interpolation");
            cmd = poseAt(1, 1, now + 60000);
            check(position(cmd, "left", 1) == 0 && std::abs(position(cmd, "right", 1) - .2) < 1e-12,
                  "closing start changed endpoint");
            moving.request(false, now + 80000);
            cmd = moving.command(now + 80000, now + 80000, session).value();
            check(position(cmd, "left", 1) == 1 && position(cmd, "right", 1) == 1,
                  "closing start changed release pose");
            moving.request(true, now + 100000);
            cmd = moving.command(now + 100000, now + 100000, session).value();
            check(position(cmd, "left", 1) == 0, "direct close used approach starting pose");
        }
        for (const char* invalid : {"[0.5, 0.6]", "[0.5, 1.1, 0.6, 0.6, 0.6, 0.6]",
                                    "[0.5, .nan, 0.6, 0.6, 0.6, 0.6]"}) {
            std::ofstream(path) << "core_hand:\n  enabled: true\n  closing_start:\n    left: " << invalid
                                << "\n    right: [0.5, 0.6, 0.6, 0.6, 0.6, 0.6]\n";
            bool invalid_rejected = false;
            try { create(); } catch (...) { invalid_rejected = true; }
            check(invalid_rejected, "invalid closing start accepted");
        }
        std::ofstream(path) << "core_hand: {enabled: true, close: null}\n";
        auto missing = create(); bool rejected = false;
        try { missing.request(true, now); } catch (...) { rejected = true; }
        check(rejected, "missing calibrated close accepted");
        missing.request(false, now); check(missing.command(now, now, session).has_value(), "missing close prevents open");
        std::ofstream(path) << "core_hand: {enabled: true, close: {left: [1000,1,1,1,1,1], right: [1,1,1,1,1,1]}}\n";
        rejected = false; try { create(); } catch (...) { rejected = true; }
        check(rejected, "raw values accepted as normalized");
        std::cout << "Core hand control passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

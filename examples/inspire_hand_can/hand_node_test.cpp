// Exercises the actual decoder/controller/driver with an in-memory CAN writer.
// No SocketCAN interface is opened and no hardware command can be transmitted.
#define INSPIRE_HAND_TESTING
#include "hand_node.cpp"
#include "protocol.hpp"

namespace {
struct HandNodeTestAccess {
    static void clock(HandNode& n, const std::string& clock = "test-boot") { n.clock_id_ = clock; }
    static void command(HandNode& n, const Json& j, InspireAction& r, InspireAction& l,
                        std::uint64_t now) {
        n.handle_message("hand.command", j.dump(), r, l, now);
    }
    static void watchdog(HandNode& n, InspireAction& r, InspireAction& l, std::uint64_t now) {
        n.check_watchdog(r, l, now);
    }
    static const Guard& guard(const HandNode& n) { return n.guard_; }
    static bool valid(const HandNode& n) { return n.last_valid_; }
    static const std::vector<double>& echo(const HandNode& n) { return n.last_norm_right_; }
    static int run(HandNode& n, InspireAction& r, InspireAction& l, const std::function<void()>& f) {
        return n.run_safely(r, l, f);
    }
    static PositionFeedback& feedback(HandNode& n, bool right) {
        return right ? n.feedback_right_ : n.feedback_left_;
    }
    static Json state(const HandNode& n, std::uint64_t now) { return n.make_state(1, now); }
    template <class Receiver>
    static void service(HandNode& n, Receiver& can, InspireHand& hand, bool right) {
        n.service_feedback(can, hand, feedback(n, right));
    }
    static bool receive(HandNode& n, zmq::socket_t& socket) {
        std::string topic, payload;
        return n.try_recv(socket, topic, payload);
    }
};
void check(bool ok, const char* why) {
    if (!ok) throw std::runtime_error(why);
}
struct Write { std::uint32_t id; int value; bool failed; };
struct MemoryCan : CanWriter {
    std::vector<Write> writes;
    int fail_call = 0;
    bool fail_all = false;
    void write(std::uint32_t id, const std::uint8_t* data, std::size_t len) override {
        check(len == 2, "expected 2-byte CAN payload");
        const bool fail = fail_all || static_cast<int>(writes.size() + 1) == fail_call;
        writes.push_back({id, data[0] | (data[1] << 8), fail});
        if (fail) throw std::runtime_error("injected CAN write failure");
    }
};
struct Harness {
    HandNode node{Config{}};
    MemoryCan r, l;
    InspireHand rh{r}, lh{l};
    InspireAction right{rh, 1}, left{lh, 2};
    Harness() { HandNodeTestAccess::clock(node); }
    void send(const Json& j, std::uint64_t now = 1000000) {
        HandNodeTestAccess::command(node, j, right, left, now);
    }
    void clear() { r.writes.clear(); l.writes.clear(); }
    void no_writes() { check(r.writes.empty() && l.writes.empty(), "rejected message wrote CAN"); }
};
Json command(std::uint64_t sequence = 1, std::uint64_t sample = 1000000) {
    return {{"msg_type", "HandCommand"}, {"version", "1.0"}, {"sequence", sequence},
        {"timestamp", 1790121600000000ULL}, {"sample_mono_us", sample},
        {"clock_id", "test-boot"}, {"publisher_id", "core"},
        {"session_id", "11111111-1111-4111-8111-111111111111"}, {"valid", true},
        {"control_epoch", "22222222-2222-4222-8222-222222222222"},
        {"mode", "NORMALIZED_POSITION"},
        {"origin", {{"publisher_id", "gateway"},
                    {"session_id", "33333333-3333-4333-8333-333333333333"},
                    {"sequence", sequence}, {"sample_mono_us", sample}, {"clock_id", "test-boot"}}},
        {"hands", {{"left", {{"drive_position_normalized", {0., .1, .2, .3, .4, .5}}}},
                   {"right", {{"drive_position_normalized", {.5, .6, .7, .8, .9, 1.}}}}}}};
}
void parsing() {
    for (const auto& value : std::vector<Json>{123, nullptr, true, Json::array(), Json::object()}) {
        auto j = command(); j["msg_type"] = value;
        HandCommand out; std::string error;
        check(!decode_hand_command("hand.command", j.dump(), out, error), "bad msg_type accepted");
    }
    for (const auto& value : std::vector<Json>{-1, 0, 1.5, "1", true, kMaxJsonInteger + 1}) {
        for (const auto* key : {"sequence", "sample_mono_us", "timestamp"}) {
            auto j = command(); j[key] = value;
            Harness h; h.send(j); h.no_writes();
            check(!HandNodeTestAccess::guard(h.node).authorized, "invalid integer authorized session");
            h.send(command());
            check(h.r.writes.size() == 6, "invalid integer poisoned next valid command");
        }
        auto j = command(); j["origin"]["sequence"] = value;
        Harness h; h.send(j); h.no_writes();
    }
    for (const auto* key : {"msg_type", "version", "sequence", "session_id", "origin", "valid"}) {
        auto j = command(); j.erase(key);
        Harness h; h.send(j); h.no_writes();
    }
    HandCommand out; std::string error;
    auto text = command().dump(); text.insert(1, "\"sequence\":2,");
    check(!decode_hand_command("hand.command", text, out, error), "duplicate keys accepted");
    check(!decode_hand_command("hand.command", std::string(65537, 'x'), out, error), "oversize accepted");
    text = "0";
    for (int i = 0; i < 32; ++i) text = "[" + text + "]";
    check(!decode_hand_command("hand.command", text, out, error), "deep JSON accepted");
}
void targets_and_mapping() {
    for (const auto* side : {"left", "right"}) {
        for (const auto& bad : std::vector<Json>{Json::array({0, 0, 0, 0, 0, 2}),
                 Json::array({0, 0, 0, 0, 0, -1}), Json::array({0, 0}), nullptr,
                 Json::array({0, 0, 0, 0, 0, true}), Json::array({0, 0, 0, 0, 0, "nan"})}) {
            auto j = command(); j["hands"][side]["drive_position_normalized"] = bad;
            Harness h; h.send(j); h.no_writes();
            check(!HandNodeTestAccess::guard(h.node).authorized, "invalid target locked session");
        }
        auto j = command(); j["hands"].erase(side);
        Harness h; h.send(j); h.no_writes();
    }
    Harness h; h.send(command());
    check(h.r.writes.size() == 6 && h.l.writes.size() == 6, "valid dual-hand command missing writes");
    for (std::size_t i = 0; i < 6; ++i) {
        check(h.r.writes[i].value == 500 + static_cast<int>(i) * 100, "normalized right mapping");
        check(h.l.writes[i].value == static_cast<int>(i) * 100, "normalized left mapping");
        check(h.r.writes[i].id == InspireHand::can_id(kPositionRegisters[i], 1, true), "right register/id");
        check(h.l.writes[i].id == InspireHand::can_id(kPositionRegisters[i], 2, true), "left register/id");
    }
    for (double closure : {0., .5, 1.}) {
        Harness c; auto j = command(); j["mode"] = "GRASP_SETPOINT";
        for (const auto* side : {"left", "right"})
            j["hands"][side] = {{"grasp", {{"closure", closure}}}};
        c.send(j);
        check(c.r.writes.size() == 6 && c.l.writes.size() == 6, "closure missing writes");
        for (const auto* can : {&c.r, &c.l})
            for (const auto& w : can->writes)
                check(w.value == std::lround((1. - closure) * 1000), "closure direction");
        check(HandNodeTestAccess::echo(c.node) == std::vector<double>(6, 1. - closure), "closure echo");
    }
    h.clear();
    check(!h.right.set_positions({0, 0, 0, 0, 0, 1001}), "driver accepted invalid position");
    check(!h.right.set_speed({0, 0, 0, 0, 0, 1001}), "driver accepted invalid speed");
    check(!h.right.set_force({0, 0, 0, 0, 0, -1}), "driver accepted invalid force");
    h.no_writes();
}
void watchdog_and_guard() {
    Harness h; h.send(command()); h.clear();
    auto bad = command(2, 1090000);
    bad["hands"]["left"]["drive_position_normalized"][5] = 2;
    h.send(bad, 1090000); h.no_writes();
    check(HandNodeTestAccess::guard(h.node).last_recv_mono == 1000000, "bad command refreshed watchdog");
    check(HandNodeTestAccess::guard(h.node).last_sequence == 1, "bad command advanced sequence");
    HandNodeTestAccess::watchdog(h.node, h.right, h.left, 1099999); h.no_writes();
    HandNodeTestAccess::watchdog(h.node, h.right, h.left, 1100000);
    check(!HandNodeTestAccess::valid(h.node), "watchdog kept old command valid");
    check(h.r.writes.size() == 6 && h.l.writes.size() == 6, "watchdog did not set both hands safe");
    for (const auto* can : {&h.r, &h.l})
        for (const auto& w : can->writes) check(w.value == 1000, "wrong safe position");
    for (const auto* clock_key : {"clock_id", "origin"}) {
        Harness c; auto j = command();
        if (std::string(clock_key) == "origin") j["origin"]["clock_id"] = "foreign";
        else j["clock_id"] = "foreign";
        c.send(j); c.no_writes();
        check(!HandNodeTestAccess::guard(c.node).authorized, "bad clock locked session");
    }
    for (auto sample : {1ULL, 900000ULL, 1000001ULL}) {
        Harness c; c.send(command(1, sample)); c.no_writes();
        auto j = command(); j["origin"]["sample_mono_us"] = sample;
        c.send(j); c.no_writes();
    }
    Harness origin; auto j = command(); j["origin"]["sample_mono_us"] = 900001;
    origin.send(j); origin.clear();
    HandNodeTestAccess::watchdog(origin.node, origin.right, origin.left, 1000001);
    check(origin.r.writes.size() == 6, "watchdog ignored origin age");
    for (const auto* key : {"publisher_id", "session_id", "control_epoch", "origin"}) {
        Harness c; c.send(command()); c.clear(); auto changed = command(2);
        if (std::string(key) == "origin") changed["origin"]["publisher_id"] = "other";
        else if (std::string(key) == "publisher_id") changed[key] = "other";
        else changed[key] = "44444444-4444-4444-8444-444444444444";
        c.send(changed); c.no_writes();
        check(HandNodeTestAccess::guard(c.node).last_sequence == 1, "unauthorized command committed");
    }
    Harness c; c.send(command()); c.clear();
    c.send(command()); c.no_writes();
    auto invalid = command(2); invalid["valid"] = false; invalid["hands"] = nullptr;
    c.send(invalid);
    check(!HandNodeTestAccess::valid(c.node) && c.r.writes.size() == 6, "invalidation did not stop");
    c.clear(); c.send(command()); c.no_writes(); // Cannot revive with an old command.
}
void faults() {
    // Partial target write failure must trigger safe attempts for all channels on both buses.
    Harness h; h.r.fail_call = 3;
    const int result = HandNodeTestAccess::run(h.node, h.right, h.left, [&] { h.send(command()); });
    check(result == 1 && !HandNodeTestAccess::valid(h.node), "CAN failure returned success");
    check(!HandNodeTestAccess::guard(h.node).authorized, "failed write committed authorization");
    check(h.r.writes.size() == 9 && h.l.writes.size() == 6, "incomplete fault cleanup");
    for (std::size_t i = 3; i < h.r.writes.size(); ++i)
        check(h.r.writes[i].value == 1000, "right safe cleanup target");
    for (const auto& w : h.l.writes) check(w.value == 1000, "left safe cleanup target");
    // A dead right CAN bus must not prevent six attempts on the left bus.
    Harness dead; dead.r.fail_all = true;
    check(HandNodeTestAccess::run(dead.node, dead.right, dead.left, [&] { dead.send(command()); }) == 1,
          "dead bus returned success");
    check(dead.r.writes.size() == 7 && dead.l.writes.size() == 6, "dead bus skipped other hand");
    // Startup exceptions and clean exits use the same cleanup boundary as the receive loop.
    Harness startup;
    check(HandNodeTestAccess::run(startup.node, startup.right, startup.left,
          [] { throw std::runtime_error("startup failure"); }) == 1, "startup failure escaped");
    check(startup.r.writes.size() == 6 && startup.l.writes.size() == 6, "startup cleanup missing");
    Harness clean;
    check(HandNodeTestAccess::run(clean.node, clean.right, clean.left, [] {}) == 0, "clean shutdown failed");
    check(clean.r.writes.size() == 6 && clean.l.writes.size() == 6, "clean shutdown missing safe writes");
    Harness cleanup; cleanup.l.fail_all = true;
    check(HandNodeTestAccess::run(cleanup.node, cleanup.right, cleanup.left, [] {}) == 1,
          "cleanup failure returned success");
    check(cleanup.r.writes.size() == 6 && cleanup.l.writes.size() == 6, "cleanup aborted early");
}
struct ReadCan : CanWriter {
    std::vector<can_frame> requests;
    std::vector<can_frame> replies;
    bool fail_write = false, fail_read = false;
    void write(std::uint32_t id, const std::uint8_t* data, std::size_t len) override {
        check(len == 1 && data[0] == 2 && (id >> 26) == 0, "position request must only read 2 bytes");
        if (fail_write) throw std::runtime_error("injected read-request failure");
        can_frame frame{}; frame.can_id = id | CAN_EFF_FLAG; frame.len = 1; frame.data[0] = data[0];
        requests.push_back(frame);
    }
    std::optional<can_frame> try_recv() {
        if (fail_read) throw std::runtime_error("injected receive failure");
        if (replies.empty()) return std::nullopt;
        const auto frame = replies.front(); replies.erase(replies.begin()); return frame;
    }
};
can_frame reply(int hand_id, std::size_t channel, int value) {
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | InspireHand::can_id(kActualPositionRegisters[channel], hand_id, false);
    frame.len = 2; frame.data[0] = value & 255; frame.data[1] = value >> 8;
    return frame;
}
void complete_snapshot(PositionFeedback& feedback, ReadCan& can, int hand_id, std::uint64_t start) {
    InspireHand hand(can);
    for (std::size_t i = 0; i < 6; ++i) {
        feedback.step(hand, start + i * 1000);
        check(can.requests.back().can_id == (CAN_EFF_FLAG |
              InspireHand::can_id(kActualPositionRegisters[i], hand_id, false)), "read register order");
        check(feedback.consume(reply(hand_id, i, static_cast<int>(i) * 200), start + i * 1000 + 100),
              "valid position reply rejected");
    }
}
void feedback_tests() {
    ReadCan can; InspireHand hand(can); PositionFeedback feedback(1, 10, 30000);
    check(!feedback.fresh(1000000, 300000), "no feedback reported fresh");
    feedback.step(hand, 1000000);
    check(can.requests.size() == 1, "first request missing");
    feedback.step(hand, 1000001);
    check(can.requests.size() == 1, "multiple outstanding requests");
    std::vector<can_frame> invalid;
    auto frame = reply(1, 0, 1000); frame.can_id &= ~CAN_EFF_FLAG; invalid.push_back(frame);
    frame = reply(1, 0, 1000); frame.can_id |= CAN_RTR_FLAG; invalid.push_back(frame);
    frame = reply(1, 0, 1000); frame.can_id |= CAN_ERR_FLAG; invalid.push_back(frame);
    frame = reply(1, 0, 1000); frame.can_id |= 1u << 26; invalid.push_back(frame);
    frame = reply(1, 0, 1000); frame.len = 1; invalid.push_back(frame);
    invalid.push_back(reply(2, 0, 1000)); invalid.push_back(reply(1, 1, 1000));
    invalid.push_back(reply(1, 0, 1001)); invalid.push_back(reply(1, 0, 65535));
    for (const auto& bad : invalid)
        check(!feedback.consume(bad, 1000100), "invalid CAN response accepted");
    check(!feedback.received(), "partial/invalid response published");
    check(feedback.consume(reply(1, 0, 0), 1000200), "first valid reply rejected");
    check(!feedback.consume(reply(1, 0, 1000), 1000300), "duplicate reply accepted");
    for (std::size_t i = 1; i < 6; ++i) {
        feedback.step(hand, 1000000 + i * 1000);
        check(feedback.consume(reply(1, i, i * 200), 1000000 + i * 1000 + 100), "reply rejected");
        check(feedback.received() == (i == 5), "partial cycle became a snapshot");
    }
    check(feedback.values() == std::array<int,6>{0,200,400,600,800,1000}, "feedback channel order");
    check(feedback.sample_mono_us() == 1000000 && feedback.samples() == 1, "snapshot timestamps/count");
    check(feedback.fresh(1299999,300000) && !feedback.fresh(1300000,300000), "feedback timeout boundary");
    feedback.step(hand, 1100000);
    check(feedback.consume(reply(1,0,999), 1100100), "next partial cycle rejected");
    feedback.step(hand, 1101000);
    feedback.step(hand, 1131000);
    check(feedback.timeouts() == 1 && feedback.values()[0] == 0 && feedback.samples() == 1,
          "timeout changed complete snapshot");
    check(!feedback.consume(reply(1,1,999),1131100), "late response accepted");
    complete_snapshot(feedback,can,1,1200000);
    check(feedback.samples() == 2, "did not recover after timeout");
    can.fail_write = true; feedback.step(hand, 1300000);
    check(feedback.io_errors() == 1 && !feedback.fresh(1300100,300000), "read failure not invalidated");
    can.fail_write = false; complete_snapshot(feedback,can,1,1400000);
    check(feedback.fresh(1406000,300000) && feedback.last_error().empty(), "read recovery failed");

    Harness h;
    auto state = HandNodeTestAccess::state(h.node, 1000000);
    check(!state.at("valid").get<bool>() && state["hands"]["right"]["sample_mono_us"].is_null(),
          "empty state pretends to have feedback");
    ReadCan r,l;
    complete_snapshot(HandNodeTestAccess::feedback(h.node,true),r,1,1000000);
    state = HandNodeTestAccess::state(h.node,1006000);
    check(!state.at("valid").get<bool>() && state["hands"]["right"]["valid"] == true &&
          state["hands"]["left"]["valid"] == false, "one hand missing should invalidate aggregate");
    complete_snapshot(HandNodeTestAccess::feedback(h.node,false),l,2,1000000);
    h.send(command(),1006000);
    state = HandNodeTestAccess::state(h.node,1006000);
    check(state.at("valid").get<bool>() && state["command_valid"] == true, "complete state not valid");
    // Production node obtains these header identities during run(); fill them for this unit harness.
    auto wire_state = state;
    wire_state["session_id"] = "11111111-1111-4111-8111-111111111111";
    wire_state["timestamp"] = 1790121600000000ULL;
    aviator::Message decoded; std::string decode_error;
    check(aviator::decode("hand.state",wire_state.dump(),decoded,decode_error),
          "feedback state incompatible with common protocol decoder");
    for (const char* side : {"left", "right"}) {
        const auto& actual = state["hands"][side];
        check(actual["drive_position_normalized"] == Json::array({0.,.2,.4,.6,.8,1.}), "normalized feedback");
        check(actual["joint_position"].is_null() && actual["joint_velocity"].is_null(), "counts mislabeled radians");
        check(actual["position_source"] == "angle_act_register" && actual["feedback_available"] == true,
              "feedback source missing");
    }
    check(state["hands"]["right"]["commanded_drive_position_normalized"] == command()["hands"]["right"]["drive_position_normalized"],
          "command echo no longer separate");
    state = HandNodeTestAccess::state(h.node,1300000);
    check(state["valid"] == false && state["hands"]["right"]["valid"] == false &&
          state["hands"]["right"]["drive_position_raw"][5] == 1000, "stale values/validity wrong");

    // Exercise the node's try_recv drain path with actual request/response framing.
    Harness service; ReadCan receiver; InspireHand transport(receiver);
    for (std::size_t i = 0; i < 6; ++i) {
        HandNodeTestAccess::service(service.node,receiver,transport,true);
        receiver.replies.push_back(reply(1,i,500));
        HandNodeTestAccess::service(service.node,receiver,transport,true);
    }
    check(HandNodeTestAccess::feedback(service.node,true).samples() == 1, "node did not consume replies");
    receiver.fail_read=true; HandNodeTestAccess::service(service.node,receiver,transport,true);
    check(HandNodeTestAccess::feedback(service.node,true).io_errors() == 1, "receive failure escaped node");

    // Read-only observation must never send position/speed/force/safe-pose writes.
    HandNode readonly(Config{},true); MemoryCan a,b; InspireHand ah(a),bh(b);
    InspireAction ar(ah,1),bl(bh,2); HandNodeTestAccess::clock(readonly);
    check(HandNodeTestAccess::run(readonly,ar,bl,[&] {
        HandNodeTestAccess::command(readonly,command(),ar,bl,1000000);
    }) == 0 && a.writes.empty() && b.writes.empty(), "feedback-only mode wrote motion registers");
}

void config_and_framing() {
    const auto path = std::filesystem::temp_directory_path() /
        ("aviator-hand-config-test-" + std::to_string(getpid()) + ".yaml");
    for (const auto* yaml : {"node: {timeout_ms: -1}", "node: {timeout_ms: 0}",
            "node: {state_rate_hz: 0}", "node: {supported_modes: [UNSUPPORTED]}",
            "hand: {safe_pose: [1,2]}", "hand: {force: [0,0,0,0,0,1001]}",
            "hand: {right_id: -1}", "feedback: {poll_rate_hz: 0}",
            "feedback: {response_timeout_ms: -1}", "feedback: {timeout_ms: 1}"}) {
        { std::ofstream out(path); out << yaml; }
        bool rejected = false;
        try { load_config(path.string()); } catch (const std::exception&) { rejected = true; }
        check(rejected, "invalid safety config accepted");
    }
    std::filesystem::remove(path);
    zmq::context_t context(1);
    zmq::socket_t send(context, zmq::socket_type::pair), receive(context, zmq::socket_type::pair);
    send.bind("inproc://framing"); receive.connect("inproc://framing");
    send.send(zmq::buffer(std::string("hand.command")), zmq::send_flags::sndmore);
    send.send(zmq::buffer(command().dump()), zmq::send_flags::sndmore);
    send.send(zmq::buffer(std::string("extra")));
    Harness h;
    check(!HandNodeTestAccess::receive(h.node, receive), "multipart accepted");
    send.send(zmq::buffer(std::string("hand.command")), zmq::send_flags::sndmore);
    send.send(zmq::buffer(command().dump()));
    check(HandNodeTestAccess::receive(h.node, receive), "multipart drain broke next command");
}
} // namespace
int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--publisher-fixture") {
            std::ifstream input(argv[2]);
            const Json messages = Json::parse(input);
            check(messages.is_array() && messages.size() >= 2, "missing publisher fixtures");
            Harness h;
            HandNodeTestAccess::clock(h.node, local_clock_id());
            bool saw_valid = false, saw_stop = false;
            for (const auto& msg : messages) {
                h.clear();
                // Replay at the original host sample time, independently of test execution delay.
                h.send(msg, msg.at("sample_mono_us").get<std::uint64_t>());
                check(HandNodeTestAccess::guard(h.node).last_sequence == msg.at("sequence"),
                      "Python publisher command rejected by actual controller");
                check(h.r.writes.size() == 6 && h.l.writes.size() == 6, "missing dual-hand writes");
                const bool valid = msg.at("valid").get<bool>();
                saw_valid |= valid; saw_stop |= !valid;
                check(HandNodeTestAccess::valid(h.node) == valid, "wrong command validity");
                for (std::size_t i = 0; i < 6; ++i) {
                    for (const auto& pair : {std::make_pair("left", &h.l), std::make_pair("right", &h.r)}) {
                        const int expected = valid ? std::lround(msg.at("hands").at(pair.first)
                            .at("drive_position_normalized").at(i).get<double>() * 1000) : 1000;
                        check(pair.second->writes[i].value == expected, "Python target mapping mismatch");
                    }
                }
            }
            check(saw_valid && saw_stop, "missing motion/stop coverage");
            std::cout << "Python publisher wire accepted; motion and stop verified with mock CAN\n";
            return 0;
        }
        parsing(); targets_and_mapping(); watchdog_and_guard(); faults(); feedback_tests(); config_and_framing();
        std::cout << "hand controller regression tests passed (mock CAN only)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "hand controller test: " << e.what() << '\n';
        return 1;
    }
}

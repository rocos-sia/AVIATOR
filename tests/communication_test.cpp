#include "protocol.hpp"
#include "runtime.hpp"
#include "transport.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace aviator;
using namespace std::chrono_literals;
namespace {
void check(bool condition, const char* text) {
    if (!condition) throw std::runtime_error(text);
}
Message flight() {
    Message message;
    message.header = {"1.0", 1, 1790121600000000, 1000000,
        "host-boot", "flight_gateway", "11111111-1111-4111-8111-111111111111", true};
    message.body = {{"source", "JOYSTICK"}, {"control", {{"roll", 0.35}, {"pitch", -0.12}}}};
    return message;
}
InputPolicy flight_policy() {
    InputPolicy policy;
    policy.publisher_id = "flight_gateway";
    policy.session_id = flight().header.session_id;
    policy.clock_id = "host-boot";
    policy.source = "JOYSTICK";
    return policy;
}
void codec_tests() {
    std::string payload, error;
    auto message = flight();
    check(encode(message, payload, error), "encode flight");
    Message parsed;
    check(decode("flight.command", payload, parsed, error), "decode flight");
    check(parsed.body == message.body && parsed.header.sequence == 1, "roundtrip fields");
    const auto good = payload;
    check(!decode("flight.command.extra", payload, parsed, error), "exact topic matching");
    check(!decode("arm.command", payload, parsed, error), "type mismatch");
    check(!decode("flight.command", "[]", parsed, error), "object root");
    check(!decode("flight.command", good + "{}", parsed, error), "trailing JSON");
    check(!decode("flight.command", std::string("\xef\xbb\xbf") + good, parsed, error), "BOM");
    check(!decode("flight.command", good + std::string(1, '\0'), parsed, error), "NUL");
    check(!decode("flight.command", std::string(65537, ' '), parsed, error), "size limit");
    check(!decode("flight.command", "{\"x\":1,\"x\":2}", parsed, error), "duplicate root key");
    check(!decode("flight.command", "{\"x\":{\"y\":1,\"y\":2}}", parsed, error), "nested duplicate key");
    check(!decode("flight.command", "{\"x\":\"\xff\"}", parsed, error), "UTF-8");
    auto json = nlohmann::json::parse(good);
    for (const auto& bad : {nlohmann::json(-1), nlohmann::json(0), nlohmann::json(1.5),
                           nlohmann::json(max_json_integer + 1), nlohmann::json("1")}) {
        json["sequence"] = bad;
        check(!decode("flight.command", json.dump(), parsed, error), "sequence constraint");
    }
    json = nlohmann::json::parse(good);
    json["session_id"] = "not-a-uuid";
    check(!decode("flight.command", json.dump(), parsed, error), "session UUID format");
    json = nlohmann::json::parse(good);
    json["valid"] = 1;
    check(!decode("flight.command", json.dump(), parsed, error), "boolean constraint");
    json.erase("valid");
    check(!decode("flight.command", json.dump(), parsed, error), "required field");
    message.header.version = "2.0";
    check(!encode(message, payload, error), "unsupported major");
    message.header.version = "1.12";
    message.body["optional_extension"] = true;
    check(encode(message, payload, error), "future minor optional field");
    message.body["valid"] = true;
    check(!encode(message, payload, error), "body shadows header");
    message = flight();
    message.body["control"]["roll"] = 1.01;
    check(!encode(message, payload, error), "axis range");
    message.body["control"]["roll"] = std::numeric_limits<double>::quiet_NaN();
    check(!encode(message, payload, error), "NaN cannot silently become null");
    message = flight();
    nlohmann::json deep = 1;
    for (int i = 0; i < 20; ++i) deep = nlohmann::json{{"nested", deep}};
    json = nlohmann::json::parse(good);
    json["extension"] = deep;
    check(!decode("flight.command", json.dump(), parsed, error), "nesting bound");
    check(parsed.body == flight().body, "failed decode preserves output");
    // All registered topic/type mappings; business validation remains node-owned.
    for (int i = 0; i < 11; ++i) {
        message = flight();
        message.topic = static_cast<Topic>(i);
        if (message.topic == Topic::arm_command || message.topic == Topic::hand_command) {
            message.body["mode"] = "JOINT_POSITION";
            message.body["control_epoch"] = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa";
            message.body["origin"] = {{"publisher_id", "flight_gateway"},
                {"session_id", flight().header.session_id}, {"sequence", 1},
                {"sample_mono_us", 1000000}, {"clock_id", "host-boot"}};
        }
        check(encode(message, payload, error), "registered topic encode");
        check(decode(topic_name(message.topic), payload, parsed, error), "registered topic decode");
    }
}
void runtime_tests() {
    auto message = flight();
    InputGuard guard(flight_policy());
    std::string error;
    check(guard.expired(1000000), "guard starts expired");
    check(guard.accept(message, 1000000, error), "first authorized sample");
    check(!guard.expired(1099999) && guard.expired(1100000), "exact timeout boundary");
    check(!guard.accept(message, 1010000, error), "duplicate rejected");
    message.header.sequence = 2;
    check(!guard.accept(message, 1100000, error), "old input with new sequence rejected");
    message.header.sample_mono_us = 1100001;
    check(!guard.accept(message, 1100000, error), "future sample rejected");
    message.header.sample_mono_us = 1100000;
    message.header.session_id = "new-session";
    check(!guard.accept(message, 1100000, error), "restart cannot authorize itself");
    message.header.session_id = flight().header.session_id;
    message.header.clock_id = "other-boot";
    check(!guard.accept(message, 1100000, error), "cross-clock rejected");
    message.header.clock_id = "host-boot";
    message.body["source"] = "FLIGHT";
    check(!guard.accept(message, 1100000, error), "source cannot seize control");
    message.body["source"] = "JOYSTICK";
    check(guard.accept(message, 1100000, error), "rejection did not advance sequence");
    message.header.sequence = 3;
    message.header.valid = false;
    check(!guard.accept(message, 1100000, error) && guard.expired(1100000), "invalid revokes immediately");
    message.header.valid = true;
    check(!guard.accept(message, 1100000, error), "invalid report advanced observed sequence");

    auto policy = flight_policy();
    policy.topic = Topic::arm_command;
    policy.publisher_id = "aviator_core";
    policy.control_epoch = "epoch-1";
    policy.origin_publisher_id = "flight_gateway";
    policy.origin_session_id = flight().header.session_id;
    InputGuard arm(policy);
    message.topic = Topic::arm_command;
    message.header.publisher_id = "aviator_core";
    message.body = {{"control_epoch", "epoch-1"}, {"origin", {
        {"publisher_id", "flight_gateway"}, {"session_id", flight().header.session_id},
        {"sequence", 1}, {"sample_mono_us", 1000000}, {"clock_id", "host-boot"}}}};
    check(!arm.accept(message, 1100000, error), "fresh command cannot hide stale origin");
    message.body["origin"]["sample_mono_us"] = 1090000;
    message.body["control_epoch"] = "epoch-2";
    check(!arm.accept(message, 1100000, error), "epoch cannot authorize itself");
    message.body["control_epoch"] = "epoch-1";
    check(arm.accept(message, 1100000, error), "motion authorization");
    check(arm.expired(1190000), "origin expires independently");

    LatestMailbox<int> left, right;
    left.store(1); right.store(2); left.store(3);
    check(left.load() == 3 && right.load() == 2, "latest topic slots independent");
    std::thread writer([&] { for (int i = 0; i < 10000; ++i) left.store(i); });
    for (int i = 0; i < 10000; ++i) (void)left.load();
    writer.join();
    check(left.load() == 9999, "mailbox concurrent copies");
    left.clear(); check(!left.load(), "mailbox clear");
    check(utc_us() > 0 && monotonic_us() > 0, "Linux clocks");
    check(local_clock_id() == local_clock_id(), "shared boot clock identity");
    check(new_session_id() != new_session_id(), "new UUID per session");
}

ReceiveResult await_receive(zmq::socket_t& socket, ReceiveState& state, WireMessage& message) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    std::string error;
    while (std::chrono::steady_clock::now() < deadline) {
        auto result = receive(socket, state, message, error);
        if (result != ReceiveResult::empty) return result;
        zmq::pollitem_t item{socket.handle(), 0, ZMQ_POLLIN, 0};
        zmq::poll(&item, 1, 10ms);
    }
    throw std::runtime_error("receive timed out");
}
void framing_tests() {
    zmq::context_t context(1);
    zmq::socket_t tx(context, zmq::socket_type::pair), rx(context, zmq::socket_type::pair);
    configure(tx); configure(rx);
    tx.bind("tcp://127.0.0.1:*");
    rx.connect(tx.get(zmq::sockopt::last_endpoint));
    // Poll writable: bounded handshake, no fixed startup sleep.
    zmq::pollitem_t writable{tx.handle(), 0, ZMQ_POLLOUT, 0};
    zmq::poll(&writable, 1, 2s);
    check(writable.revents & ZMQ_POLLOUT, "TCP peer ready");
    check(tx.get(zmq::sockopt::conflate) == 0, "multipart conflate disabled");
    WireMessage wire{"unchanged", "unchanged"}; ReceiveState state;
    tx.send(zmq::str_buffer("one"), zmq::send_flags::none);
    check(await_receive(rx, state, wire) == ReceiveResult::rejected, "one frame rejected");
    check(wire.topic == "unchanged", "reject preserves wire output");
    for (int i = 0; i < 70; ++i)
        check(tx.send(zmq::str_buffer("bad"), i == 69 ? zmq::send_flags::none : zmq::send_flags::sndmore).has_value(), "send malformed multipart");
    check(send(tx, "flight.command", "{}"), "send after malformed multipart");
    check(await_receive(rx, state, wire) == ReceiveResult::rejected && state.discarding, "drain first budget");
    check(await_receive(rx, state, wire) == ReceiveResult::rejected && state.discarding, "drain second budget");
    check(await_receive(rx, state, wire) == ReceiveResult::rejected && !state.discarding, "drain tail");
    check(await_receive(rx, state, wire) == ReceiveResult::received && wire.topic == "flight.command", "tail never becomes next message");
    tx.send(zmq::buffer(std::string(129, 'x')), zmq::send_flags::sndmore);
    tx.send(zmq::str_buffer("{}"), zmq::send_flags::none);
    check(await_receive(rx, state, wire) == ReceiveResult::rejected, "oversized topic");
}
void bus_tests() {
    zmq::context_t context(1);
    // Proxy lifecycle is checked over inproc; framing_tests exercises TCP.
    auto bus = std::async(std::launch::async, [&] {
        run_bus(context, "inproc://aviator-test-input", "inproc://aviator-test-output");
    });
    try {
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        configure(pub); configure(sub);
        subscribe(sub, "flight.command");
        sub.connect("inproc://aviator-test-output");
        pub.connect("inproc://aviator-test-input");
        std::string payload, error;
        check(encode(flight(), payload, error), "proxy payload");
        ReceiveState state; WireMessage wire;
        bool got = false;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!got && std::chrono::steady_clock::now() < deadline) {
            send(pub, "flight.command", payload);
            zmq::pollitem_t item{sub.handle(), 0, ZMQ_POLLIN, 0};
            zmq::poll(&item, 1, 10ms);
            got = receive(sub, state, wire, error) == ReceiveResult::received;
        }
        check(got && wire.payload == payload, "XSUB/XPUB forwards exact multipart");
        Message message;
        check(decode(wire.topic, wire.payload, message, error), "proxy decode");
        context.shutdown();
        check(bus.wait_for(2s) == std::future_status::ready, "bus bounded shutdown");
        bus.get();
    } catch (...) { context.shutdown(); bus.wait(); throw; }
}
} // namespace
int main() {
    try {
        codec_tests(); runtime_tests(); framing_tests(); bus_tests();
        std::cout << "communication tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}

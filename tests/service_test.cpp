#include "gateway.hpp"
#include "logger.hpp"
#include "runtime.hpp"
#include "service.hpp"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <iostream>
#include <mcap/mcap.hpp>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
namespace {
void check(bool value, const char* reason) {
    if (!value)
        throw std::runtime_error(reason);
}
template <class F> void rejects(F f) {
    bool caught = false;
    try {
        f();
    } catch (const std::exception&) {
        caught = true;
    }
    check(caught, "invalid message accepted");
}
input_event event(unsigned type, unsigned code, int value, std::uint64_t time) {
    input_event e{};
    e.type = type;
    e.code = code;
    e.value = value;
    e.input_event_sec = time / 1000000;
    e.input_event_usec = time % 1000000;
    return e;
}
void buttons() {
    flight_gateway::JoystickButtons b;
    for (unsigned i = 0; i < 11; ++i)
        b.codes[i] = BTN_TRIGGER + i;
    for (unsigned i = 0; i < 11; ++i) {
        check(b.update(event(EV_KEY, b.codes[i], 1, 1000000), 1000000, true).empty(),
              "partial report fired");
        b.update(event(EV_KEY, b.codes[i], 2, 1000000), 1000000, true);
        auto pressed = b.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000, true);
        check(pressed.size() == 1 && pressed[0] == i, "button missing");
        b.update(event(EV_KEY, b.codes[i], 1, 1000000), 1000000, true);
        check(b.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000, true).empty(),
              "held button repeated");
        b.update(event(EV_KEY, b.codes[i], 0, 1000000), 1000000, true);
        check(b.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000, true).empty(),
              "release fired");
    }
    b.update(event(EV_KEY, b.codes[0], 1, 1000000), 1000000, true);
    b.update(event(EV_SYN, SYN_DROPPED, 0, 1000000), 1000000, false);
    check(b.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1000000, false).empty(),
          "dropped report fired");
    b.held.fill(false);
    b.update(event(EV_KEY, b.codes[0], 1, 1000000), 1200000, true);
    check(b.update(event(EV_SYN, SYN_REPORT, 0, 1000000), 1200000, true).empty(),
          "stale button fired");
    b.held.fill(false);
    b.update(event(EV_KEY, b.codes[0], 1, 1000000), 1000000, true);
    check(b.update(event(EV_SYN, SYN_REPORT, 0, 1200000), 1200000, true).empty(),
          "delayed report refreshed old press");
    b.held[0] = true; // Key already down at startup.
    b.update(event(EV_KEY, b.codes[0], 1, 1300000), 1300000, true);
    check(b.update(event(EV_SYN, SYN_REPORT, 0, 1300000), 1300000, true).empty(),
          "startup held key fired");
    for (unsigned i = 0; i < 6; ++i)
        check(aviator::state_operation(b.operations[i]), "missing external event");
    check(!aviator::state_operation("Boot") && !aviator::state_operation("Emergency"),
          "internal event exposed");
}
void protocol() {
    const auto session = aviator::new_session_id();
    auto req =
        aviator::make_service_request("flight_gateway", session, "aviator_core", "grasp_wheel");
    auto rep =
        aviator::make_service_reply(req, "aviator_core", aviator::new_session_id(), "ACCEPTED");
    check(aviator::decode_service(req.dump()) == req && aviator::decode_service(rep.dump()) == rep,
          "codec");
    check(aviator::matches_service_reply(req, rep), "correlation");
    for (const char* key : {"request_id", "client_session_id", "client_id", "server_id"}) {
        auto bad = rep;
        bad[key] = aviator::new_session_id();
        check(!aviator::matches_service_reply(req, bad), "bad correlation");
    }
    auto pinned = req;
    pinned["parameters"]["server_session_id"] = aviator::new_session_id();
    check(!aviator::matches_service_reply(pinned, rep), "wrong server session");
    for (const char* key : {"request_id", "operation", "deadline_ms", "parameters"}) {
        auto bad = req;
        bad.erase(key);
        rejects([&] { aviator::decode_service(bad.dump()); });
    }
    auto bad = req;
    bad["deadline_ms"] = 0;
    rejects([&] { aviator::decode_service(bad.dump()); });
    bad = rep;
    bad["status"] = "SUCCESS";
    rejects([&] { aviator::decode_service(bad.dump()); });
    auto duplicate = req.dump();
    duplicate.insert(1, "\"version\":\"1.0\",");
    rejects([&] { aviator::decode_service(duplicate); });
    bad = req;
    bad["parameters"]["huge"] = 1e100;
    rejects([&] { aviator::decode_service(bad.dump()); });
    rejects([&] { aviator::decode_service(std::string(65537, ' ')); });
    zmq::context_t ctx(1);
    zmq::socket_t dealer(ctx, zmq::socket_type::dealer), router(ctx, zmq::socket_type::router);
    aviator::configure(dealer);
    aviator::configure(router);
    router.set(zmq::sockopt::rcvtimeo, 1000);
    router.bind("inproc://service-test");
    dealer.connect("inproc://service-test");
    check(aviator::send_service(dealer, req), "request send");
    zmq::message_t route, frame;
    check(bool(router.recv(route)) && route.more() && bool(router.recv(frame)) && !frame.more(),
          "router framing");
    check(frame.to_string() == req.dump(), "request byte preservation");
    const auto respond = [&](const std::string& raw, bool multipart = false) {
        router.send(zmq::buffer(route.data(), route.size()), zmq::send_flags::sndmore);
        router.send(zmq::buffer(raw), multipart ? zmq::send_flags::sndmore : zmq::send_flags::none);
        if (multipart)
            router.send(zmq::str_buffer("tail"));
    };
    respond(rep.dump(), true);
    respond(rep.dump());
    aviator::ReceiveState state;
    std::string raw, error;
    nlohmann::json received;
    unsigned rejected = 0;
    bool got = false;
    for (unsigned i = 0; i < 100 && !got; ++i) {
        auto result = aviator::receive_service(dealer, state, raw, received, error);
        if (result == aviator::ReceiveResult::rejected)
            ++rejected;
        if (result == aviator::ReceiveResult::received)
            got = true;
        if (!got)
            std::this_thread::sleep_for(1ms);
    }
    check(got && rejected == 1 && received == rep, "multipart drain/reply recovery");
    check(aviator::receive_service(dealer, state, raw, received, error) ==
              aviator::ReceiveResult::empty,
          "receive blocks");
    zmq::socket_t offline(ctx, zmq::socket_type::dealer);
    aviator::configure(offline);
    offline.set(zmq::sockopt::immediate, 1);
    offline.connect("tcp://127.0.0.1:1");
    check(!aviator::send_service(offline, req), "offline request queued for later execution");
}
void recording(const std::filesystem::path& path) {
    zmq::context_t ctx(1);
    zmq::socket_t pub(ctx, zmq::socket_type::xpub);
    aviator::configure(pub, {64, 64, 0});
    pub.set(zmq::sockopt::rcvtimeo, 3000);
    pub.bind("tcp://127.0.0.1:*");
    const auto endpoint = pub.get(zmq::sockopt::last_endpoint);
    std::atomic<bool> stop{false};
    std::promise<void> ready;
    auto worker = std::async(std::launch::async, [&] {
        aviator::RecorderOptions options;
        return aviator::record_bus(endpoint, path.string(), aviator::new_session_id(), stop,
                                   options, [&] { ready.set_value(); });
    });
    try {
        check(ready.get_future().wait_for(3s) == std::future_status::ready, "logger startup");
        zmq::message_t subscription;
        check(bool(pub.recv(subscription)), "logger subscription");
        auto req = aviator::make_service_request("flight_gateway", aviator::new_session_id(),
                                                 "aviator_core", "start_control");
        auto rep = aviator::make_service_reply(req, "aviator_core", aviator::new_session_id(),
                                               "COMPLETED");
        const auto request_bytes = " \n" + req.dump() + "\t";
        const auto reply_bytes = rep.dump();
        check(aviator::send(pub, aviator::service_request_topic, request_bytes),
              "record request send");
        check(aviator::send(pub, aviator::service_reply_topic, reply_bytes), "record reply send");
        aviator::send(pub, aviator::service_request_topic, reply_bytes); // topic/type mismatch
        std::this_thread::sleep_for(200ms);
        stop = true;
        const auto summary = worker.get();
        check(summary.messages == 2 && summary.invalid == 1 && summary.sequence_gaps == 0,
              "service MCAP counts");
        mcap::McapReader reader;
        check(reader.open(path.string()).ok(), "MCAP open");
        unsigned count = 0;
        for (const auto& view : reader.readMessages()) {
            const std::string raw(reinterpret_cast<const char*>(view.message.data),
                                  view.message.dataSize);
            const bool reply = view.channel->topic == aviator::service_reply_topic;
            check(raw == (reply ? reply_bytes : request_bytes), "MCAP original bytes");
            check(view.channel->messageEncoding == "json", "MCAP encoding");
            const std::string schema(reinterpret_cast<const char*>(view.schema->data.data()),
                                     view.schema->data.size());
            auto definition = nlohmann::json::parse(schema);
            check(definition.at("properties").contains("request_id") &&
                      !definition.at("properties").contains("sequence"),
                  "wrong service schema");
            check(aviator::decode_service(raw).at("request_id") == req.at("request_id"),
                  "lost MCAP correlation");
            ++count;
        }
        check(count == 2, "MCAP count");
    } catch (...) {
        stop = true;
        if (worker.valid())
            worker.wait();
        throw;
    }
}
} // namespace
int main() {
    const auto dir = std::filesystem::temp_directory_path() /
                     ("aviator-service-test-" + std::to_string(getpid()));
    try {
        std::filesystem::create_directories(dir);
        buttons();
        protocol();
        recording(dir / "service.mcap");
        std::filesystem::remove_all(dir);
        std::cout << "service tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

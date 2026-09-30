#include "monitor.hpp"
#include "runtime.hpp"
#include "service.hpp"
#include "transport.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
aviator::Message message() {
    aviator::Message m;
    m.topic = aviator::Topic::flight_state;
    m.header = {"1.0", 1, 1, 1000000, "clock", "aviator_core",
        "11111111-1111-4111-8111-111111111111", true};
    m.body = {{"system", {{"state", "CONTROL"}, {"control_source", "JOYSTICK"},
        {"current_error_code", 0}, {"last_error_code", 0}}},
        {"freshness", {{"arm", {{"valid", true}, {"age_ms", 4}}}}}};
    return m;
}
std::string encoded(const aviator::Message& m) {
    std::string payload, error;
    check(aviator::encode(m, payload, error), "encode fixture");
    return payload;
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--publish") {
            zmq::context_t context{1}; zmq::socket_t pub(context, zmq::socket_type::pub);
            aviator::configure(pub); pub.bind(argv[2]);
            auto m = message();
            m.header.clock_id = aviator::local_clock_id();
            auto request = aviator::make_service_request(
                "flight_gateway", aviator::new_session_id(), "aviator_core", "grasp_wheel");
            request["gateway_observation"] = "QUEUED";
            auto reply = aviator::make_service_reply(request, "aviator_core",
                                                     aviator::new_session_id(), "ACCEPTED");
            for (unsigned i = 1; i <= 500; ++i) {
                m.header.sequence = i; m.header.sample_mono_us = aviator::monotonic_us();
                m.header.timestamp = aviator::utc_us();
                aviator::send(pub, "flight.state", encoded(m));
                aviator::send(pub, aviator::service_request_topic, request.dump());
                aviator::send(pub, aviator::service_reply_topic, reply.dump());
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            return 0;
        }
        monitor::State state;
        check(state.snapshot(1000000, "clock")["streams"].empty(), "empty snapshot");
        auto m = message();
        state.ingest("flight.state", encoded(m), 1000000);
        auto snap = state.snapshot(1010000, "clock");
        check(snap["streams"][0]["status"] == "FRESH", "fresh state");
        check(snap["streams"][0]["summary"]["arm"]["age_ms"] == 14, "aggregate age advances");
        check(state.snapshot(1100000, "clock")["streams"][0]["status"] == "STALE", "timeout without messages");
        check(state.snapshot(1010000, "other")["streams"][0]["age_ms"].is_null(), "cross-clock age not fabricated");
        state.ingest("flight.state", encoded(m), 1020000);
        check(state.snapshot(1020000, "clock")["streams"][0]["duplicates"] == 1, "duplicate counted");
        m.header.sequence = 4;
        state.ingest("flight.state", encoded(m), 1030000);
        check(state.snapshot(1030000, "clock")["streams"][0]["gaps"] == 2, "gap accounting");
        check(state.snapshot(1200000, "clock")["streams"][0]["status"] == "STALE", "new sequence cannot hide old sample");
        m.header.sequence = 5; m.header.valid = false;
        state.ingest("flight.state", encoded(m), 1040000);
        check(state.snapshot(1040000, "clock")["streams"][0]["status"] == "INVALID", "invalid status");
        check(state.detail(1) == encoded(m) && state.detail(999).empty(), "raw payload and missing ID");
        state.ingest("flight.state.debug", "{}", 1040000);
        check(state.snapshot(1040000, "clock")["rejected"] == 1, "precise topics");
        for (unsigned i = 0; i < 65; ++i) {
            m.header.publisher_id = "source-" + std::to_string(i);
            state.ingest("flight.state", encoded(m), 1050000 + i);
        }
        snap = state.snapshot(1060000, "clock");
        check(snap["streams"].size() == 64 && snap["evicted"] == 2, "bounded stream cache");
        monitor::State services;
        auto request = aviator::make_service_request("flight_gateway", aviator::new_session_id(),
                                                     "aviator_core", "start_control");
        request["clock_id"] = "clock";
        request["issued_mono_us"] = 1000000;
        request["gateway_observation"] = "QUEUED";
        services.ingest(aviator::service_request_topic, request.dump(), 1000000);
        auto row = services.snapshot(1000000, "clock")["services"][0];
        check(row["status"] == "WAITING" && row["reply_status"].is_null(),
              "queued is not accepted");
        check(services.snapshot(1100000, "clock")["services"][0]["status"] == "TIMEOUT_UNKNOWN",
              "service timeout");
        check(services.snapshot(1100000, "other")["services"][0]["status"] == "CLOCK_UNKNOWN",
              "service clock mismatch");
        auto reply = aviator::make_service_reply(request, "aviator_core", aviator::new_session_id(),
                                                 "ACCEPTED");
        reply["timestamp"] = 200;
        auto wrong = reply;
        wrong["server_id"] = "other";
        services.ingest(aviator::service_reply_topic, wrong.dump(), 1010000);
        check(services.snapshot(1010000, "clock")["rejected"] == 1, "wrong server accepted");
        services.ingest(aviator::service_reply_topic, reply.dump(), 1020000);
        row = services.snapshot(1020000, "clock")["services"][0];
        check(row["status"] == "ACCEPTED" && row["observed_reply_ms"] == 20,
              "reply not associated");
        auto complete = reply;
        complete["status"] = "COMPLETED";
        complete["timestamp"] = 300;
        services.ingest(aviator::service_reply_topic, complete.dump(), 1030000);
        services.ingest(aviator::service_reply_topic, reply.dump(), 1040000);
        auto timeout_copy = request;
        timeout_copy["gateway_observation"] = "TIMEOUT_UNKNOWN";
        services.ingest(aviator::service_request_topic, timeout_copy.dump(), 1100000);
        services.ingest(aviator::service_request_topic, request.dump(), 1100001);
        row = services.snapshot(1100001, "clock")["services"][0];
        check(row["status"] == "COMPLETED" && row["observation"] == "TIMEOUT_UNKNOWN" &&
                  row["reply_count"] == 3,
              "old records replaced known result");
        auto detail = nlohmann::json::parse(services.detail(row["id"]));
        check(detail["reply_raw"] == complete.dump() &&
                  detail["request"]["request_id"] == request["request_id"],
              "service detail");
        auto conflict = request;
        conflict["operation"] = "exit_control";
        services.ingest(aviator::service_request_topic, conflict.dump(), 1100002);
        check(services.snapshot(1100002, "clock")["rejected"] == 2, "conflicting request accepted");
        services.ingest(aviator::service_request_topic, reply.dump(), 1100002);
        check(services.snapshot(1100002, "clock")["rejected"] == 3, "topic/type mismatch accepted");
        monitor::State orphan;
        orphan.ingest(aviator::service_reply_topic, reply.dump(), 1000000);
        row = orphan.snapshot(1000000, "clock")["services"][0];
        check(row["observation"] == "REPLY_ONLY" && row["operation"].is_null(),
              "orphan reply hidden");
        orphan.ingest(aviator::service_request_topic, request.dump(), 1010000);
        row = orphan.snapshot(1010000, "clock")["services"][0];
        check(row["operation"] == "start_control" && row["observed_reply_ms"].is_null(),
              "out-of-order association fabricated latency");
        auto second = request;
        second["client_session_id"] = aviator::new_session_id();
        second["gateway_observation"] = "NOT_SENT";
        orphan.ingest(aviator::service_request_topic, second.dump(), 1020000);
        auto transactions = orphan.snapshot(1020000, "clock")["services"];
        check(transactions.size() == 2 && transactions[0]["status"] == "NOT_SENT",
              "sessions merged/not-sent hidden");
        for (unsigned i = 0; i < 64; ++i) {
            second["request_id"] = aviator::new_session_id();
            orphan.ingest(aviator::service_request_topic, second.dump(), 1030000 + i);
        }
        snap = orphan.snapshot(1040000, "clock");
        check(snap["services"].size() == 64 && snap["service_evicted"] == 2 &&
                  snap["streams"].empty(),
              "bounded service cache");
        std::cout << "monitor tests passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

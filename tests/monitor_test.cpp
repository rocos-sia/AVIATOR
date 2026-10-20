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
        // RS422 is optional, and its age must follow the serial sample rather
        // than the continuously refreshed ZMQ envelope.
        monitor::State serial;
        auto command = message();
        command.topic = aviator::Topic::flight_command;
        command.body = {{"source", "JOYSTICK"}, {"control", {{"roll", 0}, {"pitch", 0}}}};
        serial.ingest("flight.command", encoded(command), 1000000);
        check(!serial.snapshot(1000000, "clock")["streams"][0].contains("rs422"),
              "ordinary joystick message requires no serial metadata");
        auto bytes = nlohmann::json::array();
        for (unsigned i = 0; i < 32; ++i) bytes.push_back(i);
        command.body["source"] = "FLIGHT";
        command.body["rs422"] = {{"format", "aviator-downlink-32-v1"},
            {"selected_channel", "A"}, {"blocked", false}, {"receive_timeout_ms", 100},
            {"channels", nlohmann::json::array({
                {{"channel", "A"}, {"raw_bytes", bytes}, {"received_mono_us", 1000000},
                 {"healthy", true}, {"accepted_frames", 5}, {"rejected_bytes", 7}},
                {{"channel", "B"}, {"raw_bytes", nlohmann::json::array()}, {"received_mono_us", 0},
                 {"healthy", false}, {"accepted_frames", 0}, {"rejected_bytes", 0}}})}};
        command.header.sequence++;
        serial.ingest("flight.command", encoded(command), 1000000);
        auto observed = serial.snapshot(1010000, "clock")["streams"][0]["rs422"];
        check(observed["channels"][0]["raw_bytes"] == bytes &&
              observed["channels"][0]["age_ms"] == 10 &&
              observed["channels"][0]["fresh_for_ms"] == 90 &&
              observed["channels"][1]["status"] == "WAITING", "serial bytes/age/waiting");
        command.header.sequence++;
        command.header.sample_mono_us = 1100000;
        serial.ingest("flight.command", encoded(command), 1100000);
        check(serial.snapshot(1100000, "clock")["streams"][0]["rs422"]["channels"][0]["status"] == "STALE",
              "fresh envelope must not renew serial sample");
        check(serial.snapshot(1110000, "other")["streams"][0]["rs422"]["channels"][0]["status"] == "CLOCK_UNKNOWN",
              "serial cross-clock freshness");
        command.body["rs422"]["channels"][0]["raw_bytes"][0] = 256;
        command.header.sequence++;
        serial.ingest("flight.command", encoded(command), 1100000);
        auto malformed = serial.snapshot(1100000, "clock");
        check(malformed["rejected"] == 0 && malformed["streams"][0]["rs422"].contains("error"),
              "malformed optional data must not reject control message or crash monitor");
        monitor::State retained;
        command.header.publisher_id = "flight_gateway";
        command.header.session_id = "serial-client";
        command.header.sample_mono_us = 1000000;
        bytes = nlohmann::json::array();
        for (unsigned i = 0; i < 32; ++i) bytes.push_back(0);
        bytes[0] = 235; bytes[1] = 144; bytes[2] = 7; bytes[3] = 32; bytes[31] = 39;
        auto& channel = command.body["rs422"]["channels"][0];
        channel["raw_bytes"] = bytes;
        channel["last_operation"] = {{"raw_bytes", bytes}, {"received_mono_us", 1000000},
            {"received_utc_us", 1792466880000000ULL}, {"request_id", nullptr}};
        auto ingest_saved = [&] {
            command.header.sequence++;
            retained.ingest("flight.command", encoded(command), 1010000);
            return retained.snapshot(1010000, "clock")["streams"][0]["rs422"]["channels"][0];
        };
        auto held = ingest_saved();
        check(held["last_operation"]["execution"]["status"] == "RECEIVED" &&
              held["last_operation"]["age_ms"] == 10, "retained frame is historical reception only");
        auto req = aviator::make_service_request("flight_gateway", "serial-client", "aviator_core",
            "grasp_wheel", {{"source", "FLIGHT"}, {"operation_code", 32}}, 100);
        req["clock_id"] = "clock"; req["issued_mono_us"] = 1000000;
        req["gateway_observation"] = "QUEUED";
        channel["last_operation"]["request_id"] = req["request_id"];
        channel["raw_bytes"][3] = 0; channel["raw_bytes"][31] = 7;
        held = ingest_saved();
        check(held["raw_bytes"][3] == 0 && held["last_operation"]["raw_bytes"][3] == 32 &&
              held["last_operation"]["execution"]["status"] == "CONFIRMED",
              "idle live frame coexists with retained operation without fabricating a Core reply");
        auto other_req = req; other_req["request_id"] = aviator::new_session_id();
        retained.ingest(aviator::service_request_topic, other_req.dump(), 1010000);
        check(retained.snapshot(1010000, "clock")["rejected"] == 0 &&
              ingest_saved()["last_operation"]["execution"]["status"] == "CONFIRMED",
              "same operation name is not sufficient to associate a request");
        retained.ingest(aviator::service_request_topic, req.dump(), 1010000);
        check(ingest_saved()["last_operation"]["execution"]["status"] == "WAITING",
              "exact request association reports pending Core reply");
        auto rep = aviator::make_service_reply(req, "aviator_core", "core", "REJECTED", 1,
            {{"reason", "INVALID_STATE"}, {"state", "READY"}});
        retained.ingest(aviator::service_reply_topic, rep.dump(), 1010000);
        held = ingest_saved();
        check(held["last_operation"]["execution"]["status"] == "REJECTED" &&
              held["last_operation"]["execution"]["reason"] == "INVALID_STATE",
              "Core rejection is attached to the exact retained request");
        channel["last_operation"]["request_id"] = nullptr;
        check(ingest_saved()["last_operation"]["execution"]["status"] == "RECEIVED",
              "a later same-code operation does not inherit the previous reply");
        channel["last_operation"]["raw_bytes"][31] = 0;
        held = ingest_saved();
        check(held["last_operation"].contains("error") && held["status"] == "FRESH",
              "bad retained checksum is isolated from live reception");
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
        monitor::State relabeled;
        relabeled.ingest(aviator::service_request_topic, request.dump(), 1000000);
        auto changed_labels = request;
        changed_labels["client_session_id"] = "new-client-label";
        changed_labels["parameters"]["server_session_id"] = "ignored-server-label";
        relabeled.ingest(aviator::service_request_topic, changed_labels.dump(), 1000001);
        auto changed_reply = reply;
        changed_reply["client_session_id"] = "reply-client-label";
        changed_reply["server_session_id"] = "reply-server-label";
        relabeled.ingest(aviator::service_reply_topic, changed_reply.dump(), 1000002);
        check(relabeled.snapshot(1000002, "clock")["rejected"] == 0 &&
                  relabeled.snapshot(1000002, "clock")["services"].size() == 1,
              "session labels split or rejected a service transaction");
        orphan.ingest(aviator::service_reply_topic, reply.dump(), 1000000);
        row = orphan.snapshot(1000000, "clock")["services"][0];
        check(row["observation"] == "REPLY_ONLY" && row["operation"].is_null(),
              "orphan reply hidden");
        orphan.ingest(aviator::service_request_topic, request.dump(), 1010000);
        row = orphan.snapshot(1010000, "clock")["services"][0];
        check(row["operation"] == "start_control" && row["observed_reply_ms"].is_null(),
              "out-of-order association fabricated latency");
        auto second = request;
        second["client_session_id"] = "another-instance";
        second["request_id"] = aviator::new_session_id();
        second["gateway_observation"] = "NOT_SENT";
        orphan.ingest(aviator::service_request_topic, second.dump(), 1020000);
        auto transactions = orphan.snapshot(1020000, "clock")["services"];
        check(transactions.size() == 2 && transactions[0]["status"] == "NOT_SENT",
              "requests merged/not-sent hidden");
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

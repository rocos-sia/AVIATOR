#include "logger.hpp"
#include "protocol.hpp"
#include "recording.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <mcap/mcap.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
namespace {
void check(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
template <class F> void fails(F function, const char* message) {
    bool failed = false;
    try {
        function();
    } catch (const std::exception&) {
        failed = true;
    }
    check(failed, message);
}
aviator::Message sample(int topic = 0) {
    aviator::Message message;
    message.topic = static_cast<aviator::Topic>(topic);
    message.header = {"1.0",
                      1,
                      1790121600000000,
                      1000000,
                      "boot",
                      "producer",
                      "11111111-1111-4111-8111-111111111111",
                      true};
    message.body = {{"source", "JOYSTICK"}, {"control", {{"roll", 0.2}, {"pitch", 0.1}}}};
    if (topic == 2 || topic == 4) {
        message.body["mode"] = "JOINT_POSITION";
        message.body["control_epoch"] = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa";
        message.body["origin"] = {{"publisher_id", "producer"},
                                  {"session_id", message.header.session_id},
                                  {"sequence", 1},
                                  {"sample_mono_us", 1000000},
                                  {"clock_id", "boot"}};
    }
    return message;
}
std::string encode(const aviator::Message& message) {
    std::string payload, error;
    check(aviator::encode(message, payload, error), error.c_str());
    return " \n" + payload + "\t"; // Ensure stored bytes are never reserialized.
}
void storage(const std::filesystem::path& directory) {
    const auto path = (directory / "all.mcap").string();
    std::vector<std::pair<std::string, std::string>> expected;
    aviator::RecorderSummary summary;
    {
        aviator::RecordingWriter writer(path, "logger-session");
        for (int topic = 0; topic < 11; ++topic) {
            auto message = sample(topic);
            message.header.sequence = (1ULL << 32) + 1;
            message.header.valid = false;
            expected.emplace_back(aviator::topic_name(message.topic), encode(message));
        }
        auto message = sample();
        message.header.publisher_id = "second";
        expected.emplace_back("flight.command", encode(message));
        message.header.session_id = "22222222-2222-4222-8222-222222222222";
        expected.emplace_back("flight.command", encode(message));
        message.header.version = "1.1";
        message.header.sequence = 3; // one gap within this source session
        expected.emplace_back("flight.command", encode(message));
        expected.emplace_back("flight.command", encode(message)); // duplicate retained
        for (std::size_t i = 0; i < expected.size(); ++i)
            writer.append(expected[i].first, expected[i].second,
                          1000000 - i); // UTC rollback retained
        writer.append("unknown.topic", "{}", 5);
        writer.append("flight.command", "broken JSON", 6);
        summary = writer.finish(2, 3);
        fails([&] { writer.append("flight.command", "{}", 0); }, "append after finish");
    }
    check(summary.messages == expected.size() && summary.channels == 14, "counts/channels");
    check(summary.invalid == 2 && summary.rejected == 2 && summary.dropped == 3, "drop statistics");
    check(summary.sequence_gaps == 1 && summary.duplicate_or_reordered == 1,
          "full sequence statistics");
    check(!std::filesystem::exists(path + ".partial"), "partial finalized");
    mcap::McapReader reader;
    check(reader.open(path).ok(), "read recording");
    check(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok(), "read indexes");
    std::size_t count = 0;
    for (const auto& view : reader.readMessages(
             [](const mcap::Status& status) { check(status.ok(), "reader error"); })) {
        check(count < expected.size(), "unexpected message");
        check(view.channel->topic == expected[count].first, "topic mapping");
        check(view.schema && view.schema->encoding == "jsonschema", "schema mapping");
        check(view.channel->messageEncoding == "json", "channel encoding");
        const auto json = nlohmann::json::parse(expected[count].second);
        check(view.channel->metadata.at("publisher_id") == json.at("publisher_id"),
              "publisher metadata");
        check(view.channel->metadata.at("session_id") == json.at("session_id"),
              "source session metadata");
        check(view.schema->name == json.at("msg_type").get<std::string>() + "/" +
                                       json.at("version").get<std::string>(),
              "versioned schema");
        check(view.message.logTime == 1000000 - count, "receive timestamp retained");
        check(view.message.publishTime == view.message.logTime, "publish fallback");
        check(view.message.sequence ==
                  static_cast<std::uint32_t>(json.at("sequence").get<std::uint64_t>()),
              "sequence low32");
        check(std::string(reinterpret_cast<const char*>(view.message.data),
                          view.message.dataSize) == expected[count].second,
              "byte-identical payload");
        ++count;
    }
    check(count == expected.size(), "roundtrip count");
    reader.close();
    fails([&] { aviator::RecordingWriter writer(path, "session"); }, "existing file must survive");
    const auto abandoned = (directory / "abandoned.mcap").string();
    { aviator::RecordingWriter writer(abandoned, "session"); }
    check(std::filesystem::exists(abandoned + ".partial") && !std::filesystem::exists(abandoned),
          "abandoned recording retained");
    fails([&] { aviator::RecordingWriter writer(abandoned, "session"); },
          "partial must not be overwritten");
    const auto raced = (directory / "raced.mcap").string();
    {
        aviator::RecordingWriter writer(raced, "session");
        std::ofstream(raced) << "existing";
        fails([&] { writer.finish(); }, "concurrent final creation must not be overwritten");
    }
    check(std::filesystem::file_size(raced) == 8, "concurrent output preserved");
    const auto empty = (directory / "empty.mcap").string();
    {
        aviator::RecordingWriter writer(empty, "session");
        check(writer.finish().messages == 0, "empty recording");
    }
    // A symlink to /dev/full must never be followed/truncated by partial creation.
    const auto full = (directory / "full.mcap").string();
    std::filesystem::create_symlink("/dev/full", full + ".partial");
    fails([&] { aviator::RecordingWriter writer(full, "session"); }, "exclusive partial creation");
}
void transport(const std::filesystem::path& directory, bool overflow) {
    zmq::context_t context{1};
    zmq::socket_t publisher(context, zmq::socket_type::xpub);
    aviator::configure(publisher, {1024, 1024, 0});
    publisher.bind("tcp://127.0.0.1:*");
    const auto endpoint = publisher.get(zmq::sockopt::last_endpoint);
    std::atomic<bool> stop{false};
    aviator::RecorderOptions options;
    if (overflow)
        options.queue_bytes = 1;
    auto task = std::async(std::launch::async, [&] {
        return aviator::record_bus(endpoint,
                                   (directory / (overflow ? "overflow.mcap" : "bus.mcap")).string(),
                                   "logger-session", stop, options);
    });
    try {
        zmq::pollitem_t ready{publisher.handle(), 0, ZMQ_POLLIN, 0};
        zmq::poll(&ready, 1, 3s);
        check(ready.revents & ZMQ_POLLIN, "subscription handshake");
        zmq::message_t subscription;
        check(publisher.recv(subscription).has_value(), "subscription received");
        for (unsigned i = 1; i <= 360; ++i) {
            auto message = sample();
            message.header.sequence = i;
            check(aviator::send(publisher, "flight.command", encode(message)), "send bus traffic");
        }
        std::this_thread::sleep_for(500ms);
        stop.store(true);
        check(task.wait_for(5s) == std::future_status::ready, "shutdown drains queue");
        const auto summary = task.get();
        check(summary.messages == (overflow ? 0 : 360), "socket recording count");
        check(summary.dropped == (overflow ? 360 : 0), "bounded queue overflow count");
    } catch (...) {
        stop.store(true);
        if (task.valid())
            task.wait();
        throw;
    }
}
} // namespace
int main() {
    const auto directory =
        std::filesystem::temp_directory_path() / ("aviator-recording-" + std::to_string(getpid()));
    try {
        check(std::filesystem::create_directory(directory), "isolated test directory");
        storage(directory);
        transport(directory, false);
        transport(directory, true);
        std::filesystem::remove_all(directory);
        std::cout << "recording tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << " (artifacts: " << directory << ")\n";
        return 1;
    }
}

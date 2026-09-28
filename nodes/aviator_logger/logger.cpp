// aviator_logger recording loop: bus subscription -> single-file MCAP.
//
// Minimal implementation promoted from examples/mcap_recording: lossless
// bus-JSON capture to one MCAP file, no volume splitting. See SAD §10 for the
// full contract (volume/session/raw-channel upgrades) and the mapping kept
// here: Schema=jsonschema per message type, Channel=bus topic, Message.data =
// the original frame bytes (never re-encoded).

#define MCAP_IMPLEMENTATION  // single TU for the header-only MCAP library
#include <mcap/mcap.hpp>

#include "logger.hpp"
#include "protocol.hpp"
#include "runtime.hpp"
#include "transport.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace aviator {
namespace {

// One JSON Schema per message type. Real schemas are not frozen yet (schemas/
// is empty); this constrains the common header so readers can still validate
// the envelope. Replace with the frozen schemas once the ICD lands.
std::string make_schema(std::string_view message_type) {
    return std::string(
               "{\"$schema\":\"http://json-schema.org/draft-07/schema#\",\"type\":\"object\","
               "\"required\":[\"msg_type\",\"version\",\"sequence\",\"timestamp\","
               "\"sample_mono_us\",\"clock_id\",\"publisher_id\",\"session_id\",\"valid\"],"
               "\"properties\":{\"msg_type\":{\"const\":\"") +
           std::string(message_type) +
           "\"},\"version\":{\"type\":\"string\"},\"sequence\":{\"type\":\"integer\"},"
           "\"timestamp\":{\"type\":\"integer\"},\"sample_mono_us\":{\"type\":\"integer\"},"
           "\"clock_id\":{\"type\":\"string\"},\"publisher_id\":{\"type\":\"string\"},"
           "\"session_id\":{\"type\":\"string\"},\"valid\":{\"type\":\"boolean\"}}}";
}

// Nanosecond UTC receive time, clamped non-decreasing so MCAP chunk indexing
// (which sorts by log_time) stays valid across small wall-clock steps.
class ReceiveClock {
public:
    std::uint64_t now() {
        const std::uint64_t candidate = utc_us() * 1000ULL;
        last_ = std::max(candidate, last_ + 1);
        return last_;
    }

private:
    std::uint64_t last_ = 0;
};

} // namespace

RecorderSummary record_bus(const std::string& subscribe_endpoint,
                           const std::string& output_path,
                           const std::string& session_id,
                           const std::atomic<bool>& stop) {
    RecorderSummary summary;
    summary.path = output_path;
    const std::string partial = output_path + ".partial";

    // Receiver owns its context + SUB socket + writer, all in this thread.
    zmq::context_t context{1};
    zmq::socket_t sub(context, zmq::socket_type::sub);
    configure(sub, {8, 1024, 0});  // larger receive HWM: a logger is not latest-value
    subscribe(sub, "");
    sub.connect(subscribe_endpoint);

    mcap::McapWriterOptions options("aviator");
    options.compression = mcap::Compression::None;  // minimal: dependency-free
    options.chunkSize = 1024 * 1024;

    mcap::McapWriter writer;
    const auto open_status = writer.open(partial, options);
    if (open_status.code != mcap::StatusCode::Success)
        throw std::runtime_error("writer.open " + partial + ": " + open_status.message);

    mcap::Metadata metadata;
    metadata.name = "aviator";
    metadata.metadata = {{"node", "aviator_logger"}, {"session_id", session_id}};
    const auto metadata_status = writer.write(metadata);
    if (metadata_status.code != mcap::StatusCode::Success)
        throw std::runtime_error("writer.write(metadata): " + metadata_status.message);

    // Lazily registered on first sight: message type -> schema id, topic -> channel id.
    std::map<std::string, mcap::SchemaId> schemas;
    std::map<std::string, mcap::ChannelId> channels;

    ReceiveClock clock;
    ReceiveState receiving;
    std::string error;
    Message decoded;

    while (!stop.load()) {
        zmq::pollitem_t ready{sub.handle(), 0, ZMQ_POLLIN, 0};
        zmq::poll(&ready, 1, std::chrono::milliseconds(20));
        while (!stop.load()) {
            WireMessage wire;
            const auto result = receive(sub, receiving, wire, error);
            if (result == ReceiveResult::empty) break;
            if (result == ReceiveResult::rejected) { ++summary.rejected; continue; }
            if (!decode(wire.topic, wire.payload, decoded, error)) { ++summary.invalid; continue; }

            // Schema: one per message type.
            const std::string type(message_type(decoded.topic));
            auto schema_it = schemas.find(type);
            if (schema_it == schemas.end()) {
                mcap::Schema schema(type, "jsonschema", make_schema(type));
                writer.addSchema(schema);
                schema_it = schemas.emplace(type, schema.id).first;
            }

            // Channel: one per topic; metadata carries the first message's origin.
            const std::string topic(wire.topic);
            auto channel_it = channels.find(topic);
            if (channel_it == channels.end()) {
                mcap::Channel channel(topic, "json", schema_it->second,
                                      {{"publisher_id", decoded.header.publisher_id},
                                       {"session_id", decoded.header.session_id},
                                       {"clock_id", decoded.header.clock_id}});
                writer.addChannel(channel);
                channel_it = channels.emplace(topic, channel.id).first;
                ++summary.channels;
            }

            mcap::Message message;
            message.channelId = channel_it->second;
            message.sequence = static_cast<std::uint32_t>(decoded.header.sequence);
            message.logTime = clock.now();
            message.publishTime = decoded.header.timestamp * 1000ULL;
            message.data = reinterpret_cast<const std::byte*>(wire.payload.data());
            message.dataSize = wire.payload.size();
            const auto write_status = writer.write(message);
            if (write_status.code != mcap::StatusCode::Success)
                throw std::runtime_error("writer.write: " + write_status.message);
            ++summary.messages;
        }
    }

    writer.close();
    std::filesystem::rename(partial, output_path);
    return summary;
}

} // namespace aviator

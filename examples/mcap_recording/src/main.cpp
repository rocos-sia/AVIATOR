// MCAP full-recording example for AVIATOR.
//
// Demonstrates the SAD §10 recording contract end-to-end:
//   1. Write synthetic bus messages (3 topics) to a local MCAP file, mapping
//      Schema / Channel / Message as the architecture document specifies.
//   2. Finalize the file atomically (recording.mcap.partial -> recording.mcap).
//   3. Read the file back and verify the recording is lossless: total message
//      count, per-channel count, byte-identical payloads, monotonic per-channel
//      sequence, and presence of schema metadata.
//
// The example is self-contained (header-only MCAP, no external dependency
// beyond a C++17 standard library) so it can be built and tuned independently
// before being promoted into nodes/aviator_logger.

#define MCAP_IMPLEMENTATION  // single translation unit for the header-only MCAP library
#include <mcap/mcap.hpp>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

using std::string_literals::operator""s;

// ---------------------------------------------------------------------------
// Recording parameters
// ---------------------------------------------------------------------------

constexpr uint64_t kBaseNs = 1'700'000'000'000'000'000ULL;  // synthetic epoch (ns)

struct TopicSpec {
  const char* topic;   // bus topic (also the MCAP channel topic)
  const char* msgType; // PascalCase type name -> MCAP schema name
  int hz;              // nominal publish rate
  int messages;        // number of messages to record for this topic
};

// 2 s of traffic at the SAD §04 rates.
const TopicSpec kTopics[] = {
    {"flight.command", "FlightCommand", 50, 100},
    {"arm.state", "ArmState", 100, 200},
    {"camera.detection", "CameraDetection", 30, 60},
};

// ---------------------------------------------------------------------------
// Synthetic payload builders (real nodes would serialize their typed message)
// ---------------------------------------------------------------------------

std::string makePayload(const TopicSpec& spec, uint32_t seq, uint64_t tsUs) {
  if (spec.topic == "flight.command"s) {
    const double roll = 0.5 * (seq % 20) / 20.0 - 0.25;
    const double pitch = -0.3 + 0.6 * ((seq * 7) % 11) / 11.0;
    return "{\"msg_type\":\"FlightCommand\",\"version\":\"1.0\",\"sequence\":" +
           std::to_string(seq) + ",\"timestamp\":" + std::to_string(tsUs) +
           ",\"valid\":true,\"source\":\"JOYSTICK\",\"control\":{\"roll\":" +
           std::to_string(roll) + ",\"pitch\":" + std::to_string(pitch) + "}}";
  }
  if (spec.topic == "arm.state"s) {
    return "{\"msg_type\":\"ArmState\",\"version\":\"1.0\",\"sequence\":" +
           std::to_string(seq) + ",\"timestamp\":" + std::to_string(tsUs) +
           ",\"valid\":true,\"joint_position\":[0.1,0.2,0.3,0.4,0.5,0.6,0.7]}";
  }
  return "{\"msg_type\":\"CameraDetection\",\"version\":\"1.0\",\"sequence\":" +
         std::to_string(seq) + ",\"timestamp\":" + std::to_string(tsUs) +
         ",\"valid\":true,\"status\":\"TRACKING\",\"confidence\":0.95,\"yoke\":{"
         "\"detected\":true,\"roll\":0.1,\"pitch\":-0.1}}";
}

// Minimal JSON Schema per message type (encoding=jsonschema, SAD §10).
std::string makeSchema(const TopicSpec& spec) {
  return std::string("{\"$schema\":\"http://json-schema.org/draft-07/schema#\",\"type\":\"object\","
         "\"properties\":{\"msg_type\":{\"const\":\"") +
         spec.msgType +
         "\"},\"sequence\":{\"type\":\"integer\"},\"timestamp\":{\"type\":\"integer\"},"
         "\"valid\":{\"type\":\"boolean\"}}}";
}

// ---------------------------------------------------------------------------
// Lossless verification
// ---------------------------------------------------------------------------

// The recording we expect: channelId -> (sequence -> payload bytes).
struct Recording {
  std::map<uint16_t, std::map<uint32_t, std::string>> expected;
  std::size_t total = 0;
};

bool verify(const std::string& path, const Recording& rec) {
  mcap::McapReader reader;
  auto st = reader.open(path);
  if (st.code != mcap::StatusCode::Success) {
    std::cerr << "[FAIL] reader.open: " << st.message << "\n";
    return false;
  }

  bool ok = true;
  std::size_t seen = 0;
  std::map<uint16_t, uint32_t> lastSeq;

  auto view = reader.readMessages();
  for (const auto& msgView : view) {
    const auto& msg = msgView.message;
    ++seen;

    // Per-channel sequence must be strictly increasing.
    auto& last = lastSeq[msg.channelId];
    if (last != 0 && msg.sequence <= last) {
      std::cerr << "[FAIL] channel " << msg.channelId << " sequence not monotonic: "
                << last << " -> " << msg.sequence << "\n";
      ok = false;
    }
    last = msg.sequence;

    // Payload must be byte-identical.
    const auto ch = rec.expected.find(msg.channelId);
    if (ch == rec.expected.end()) {
      std::cerr << "[FAIL] unexpected channel " << msg.channelId << "\n";
      ok = false;
      continue;
    }
    const auto it = ch->second.find(msg.sequence);
    if (it == ch->second.end()) {
      std::cerr << "[FAIL] unexpected sequence " << msg.sequence << " on channel "
                << msg.channelId << "\n";
      ok = false;
      continue;
    }
    const std::string got(reinterpret_cast<const char*>(msg.data), msg.dataSize);
    if (got != it->second) {
      std::cerr << "[FAIL] payload mismatch on channel " << msg.channelId
                << " seq " << msg.sequence << "\n";
      ok = false;
    }

    // JSON messages must reference a schema (SAD §10).
    if (msgView.channel->schemaId == 0 || msgView.schema == nullptr) {
      std::cerr << "[FAIL] channel " << msg.channelId << " lacks schema\n";
      ok = false;
    }
  }

  if (seen != rec.total) {
    std::cerr << "[FAIL] message count: expected " << rec.total << ", got " << seen << "\n";
    ok = false;
  }
  reader.close();
  return ok;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string outPath = (argc > 1) ? argv[1] : "recording.mcap";
  const std::string partialPath = outPath + ".partial";

  // 1. Write to a .partial file.
  mcap::McapWriterOptions opts("aviator");  // recording profile (free-form)
  opts.compression = mcap::Compression::None;  // keep the example dependency-free
  opts.chunkSize = 64 * 1024;

  mcap::McapWriter writer;
  auto st = writer.open(partialPath, opts);
  if (st.code != mcap::StatusCode::Success) {
    std::cerr << "writer.open failed: " << st.message << "\n";
    return 1;
  }

  Recording rec;
  for (const auto& spec : kTopics) {
    // Schema: one per message type.
    mcap::Schema schema(spec.msgType, "jsonschema", makeSchema(spec));
    writer.addSchema(schema);

    // Channel: one per topic; metadata carries publisher/session (SAD §10).
    mcap::Channel channel(spec.topic, "json", schema.id,
                          {{"publisher_id", "aviator_core"},
                           {"session_id", "example-session"}});
    writer.addChannel(channel);

    const uint64_t periodNs = 1'000'000'000ULL / spec.hz;
    for (int i = 0; i < spec.messages; ++i) {
      const uint32_t seq = static_cast<uint32_t>(i + 1);
      const uint64_t publishNs = kBaseNs + static_cast<uint64_t>(i) * periodNs;
      const uint64_t tsUs = publishNs / 1000;
      std::string payload = makePayload(spec, seq, tsUs);

      mcap::Message msg;
      msg.channelId = channel.id;
      msg.sequence = seq;
      msg.publishTime = publishNs;
      msg.logTime = publishNs + 1'000'000;  // ingested 1 ms after publish
      msg.data = reinterpret_cast<const std::byte*>(payload.data());
      msg.dataSize = payload.size();
      if (writer.write(msg).code != mcap::StatusCode::Success) {
        std::cerr << "writer.write failed\n";
        return 1;
      }

      rec.expected[channel.id][seq] = std::move(payload);
      ++rec.total;
    }
  }
  writer.close();

  // 2. Atomic finalize (.partial -> final), SAD §10.
  std::filesystem::rename(partialPath, outPath);

  std::cout << "wrote " << rec.total << " messages to " << outPath << "\n";

  // 3. Read back and verify losslessness.
  const bool ok = verify(outPath, rec);
  if (ok) {
    std::cout << "[PASS] lossless round-trip verified: " << rec.total
              << " messages, byte-identical payloads, monotonic sequence, schema present\n";
    return 0;
  }
  std::cerr << "[FAIL] verification failed\n";
  return 1;
}

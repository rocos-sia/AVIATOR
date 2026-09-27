// foxglove_bridge — forward AVIATOR bus traffic (data + camera image) to
// Foxglove Studio over the foxglove.websocket.v1 protocol.
//
// This process is a pure forwarder: it does not capture or encode any camera
// frame itself. It subscribes to two ZMQ sources and re-publishes them to
// WebSocket clients:
//
//   * control bus XPUB  tcp://127.0.0.1:5556  — small JSON messages
//       (flight.command, arm.state, ...), each carrying a `timestamp` (µs).
//   * image channel       tcp://127.0.0.1:5558  — multipart
//       Frame0 = topic ("camera.image")
//       Frame1 = JPEG bytes
//       Frame2 = JSON metadata { frame, timestamp µs, width, height, format }
//
// Channel mapping to Foxglove:
//   * every bus topic          -> JSON channel (payload passed through as-is)
//   * camera.image             -> foxglove.CompressedImage (JSON encoding,
//                                 JPEG base64 in `data`)
// Every outgoing foxglove message carries its timestamp in the binary frame
// header (nanoseconds since Unix epoch), so Studio plots and the image panel
// are time-synchronized across topics.
//
// Wire protocol (foxglove.websocket.v1), server -> client:
//   * text   {"op":"serverInfo",...}
//   * text   {"op":"advertise","channels":[{id,topic,encoding,schemaName,schema}]}
//   * binary [0x01][u32 subscription_id LE][u64 timestamp_ns LE][payload]
// client -> server (text): {"op":"subscribe"|"unsubscribe",...}
//
// Usage:
//   foxglove_bridge [--data-endpoint tcp://127.0.0.1:5556]
//                   [--image-endpoint tcp://127.0.0.1:5558]
//                   [--listen 0.0.0.0 --port 8765]
// then in Foxglove Studio: Open connection -> WebSocket -> ws://localhost:8765

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

#include <zmq.hpp>
#include <nlohmann/json.hpp>

#include "base64.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace {

std::atomic<bool> g_running{true};

// ---------------------------------------------------------------------------
// Little-endian helpers for the binary message header
// ---------------------------------------------------------------------------

inline void append_le32(std::string& s, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

inline void append_le64(std::string& s, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

inline std::uint64_t now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

// ---------------------------------------------------------------------------
// Channel registry (owned by the io thread)
// ---------------------------------------------------------------------------

struct Channel {
  std::uint32_t id;
  std::string topic;        // foxglove topic (bus topic with a leading '/')
  std::string encoding;     // "json"
  std::string schema_name;  // "foxglove.CompressedImage" or "JsonMessage"
  std::string schema;       // JSON Schema text
  bool is_image = false;
};

// Foxglove JSON-encoded schemas. `data` carries the base64 JPEG bytes.
constexpr const char* kCompressedImageSchema =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp\":{\"type\":\"object\",\"properties\":{"
    "\"sec\":{\"type\":\"integer\"},\"nsec\":{\"type\":\"integer\"}}},"
    "\"frame_id\":{\"type\":\"string\"},"
    "\"data\":{\"type\":\"string\",\"contentEncoding\":\"base64\"},"
    "\"format\":{\"type\":\"string\"}}}";

constexpr const char* kJsonMessageSchema = "{\"type\":\"object\"}";

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

class Bridge;

class Session : public std::enable_shared_from_this<Session> {
 public:
  Session(tcp::socket socket, Bridge& bridge);
  void run();

  // Send one frame to the client (safe to call from the io thread).
  void send_json(std::string text);
  void send_message(std::uint32_t sub_id, std::uint64_t ts_ns,
                    const std::string& payload);

  std::map<std::uint32_t, std::uint32_t> subs;  // subscription id -> channel id

 private:
  void on_accept(beast::error_code ec);
  void do_read();
  void on_read(beast::error_code ec, std::size_t);
  void handle_client_message(const std::string& text);
  void do_write();

  struct Out {
    bool binary;
    std::string data;
  };

  websocket::stream<tcp::socket> ws_;
  beast::flat_buffer buffer_;
  std::deque<Out> queue_;
  bool writing_ = false;
  Bridge& bridge_;
};

class Bridge {
 public:
  // Register (or look up) the channel for a bus topic. Returns the channel.
  Channel& channel_for(const std::string& bus_topic);

  // Sessions live on the io thread; keyed by raw pointer for O(1) removal.
  void add_session(std::shared_ptr<Session> s) { sessions_.emplace(s.get(), std::move(s)); }
  void remove_session(Session* s) { sessions_.erase(s); }

  const std::map<std::uint32_t, Channel>& channels() const { return channels_; }

  // Handle a bus message received on the ZMQ thread (posted to the io thread).
  void handle_incoming(const std::string& bus_topic, const std::string& payload,
                       const std::string& meta, std::uint64_t ts_us);

 private:
  std::uint32_t next_id_ = 1;
  std::map<std::string, std::uint32_t> topic_to_id_;
  std::map<std::uint32_t, Channel> channels_;
  std::map<Session*, std::shared_ptr<Session>> sessions_;
};

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

Session::Session(tcp::socket socket, Bridge& bridge)
    : ws_(std::move(socket)), bridge_(bridge) {}

void Session::run() {
  ws_.set_option(websocket::stream_base::decorator(
      [](websocket::response_type& res) {
        res.set(http::field::server, "aviator-foxglove-bridge");
        // Negotiate the foxglove subprotocol so Studio accepts the handshake.
        res.set(http::field::sec_websocket_protocol, "foxglove.websocket.v1");
      }));
  ws_.async_accept(
      beast::bind_front_handler(&Session::on_accept, shared_from_this()));
}

void Session::on_accept(beast::error_code ec) {
  if (ec) {
    bridge_.remove_session(this);
    return;
  }

  // serverInfo first, then advertise every channel already known.
  send_json("{\"op\":\"serverInfo\",\"name\":\"aviator-foxglove-bridge\","
            "\"capabilities\":[],\"supportedEncodings\":[\"json\"],"
            "\"metadata\":{}}");
  for (const auto& [id, ch] : bridge_.channels()) {
    send_json("{\"op\":\"advertise\",\"channels\":[{\"id\":" +
              std::to_string(ch.id) + ",\"topic\":\"" + ch.topic +
              "\",\"encoding\":\"" + ch.encoding + "\",\"schemaName\":\"" +
              ch.schema_name + "\",\"schema\":" + ch.schema + "}]}");
  }
  do_read();
}

void Session::send_json(std::string text) {
  queue_.push_back({false, std::move(text)});
  do_write();
}

void Session::send_message(std::uint32_t sub_id, std::uint64_t ts_ns,
                           const std::string& payload) {
  std::string frame;
  frame.reserve(1 + 4 + 8 + payload.size());
  frame.push_back('\x01');  // message opcode
  append_le32(frame, sub_id);
  append_le64(frame, ts_ns);
  frame.append(payload);
  queue_.push_back({true, std::move(frame)});
  do_write();
}

void Session::do_write() {
  if (writing_ || queue_.empty()) return;
  writing_ = true;
  auto self = shared_from_this();
  auto& out = queue_.front();
  ws_.binary(out.binary);
  ws_.async_write(net::buffer(out.data),
                  [self](beast::error_code ec, std::size_t) {
                    if (ec) return;
                    self->queue_.pop_front();
                    self->writing_ = false;
                    self->do_write();
                  });
}

void Session::do_read() {
  ws_.async_read(
      buffer_, beast::bind_front_handler(&Session::on_read, shared_from_this()));
}

void Session::on_read(beast::error_code ec, std::size_t) {
  if (ec) {
    bridge_.remove_session(this);
    return;
  }
  const std::string text = beast::buffers_to_string(buffer_.data());
  buffer_.consume(buffer_.size());
  handle_client_message(text);
  do_read();
}

void Session::handle_client_message(const std::string& text) {
  const auto j = nlohmann::json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return;
  const std::string op = j.value("op", "");

  if (op == "subscribe" && j.contains("subscriptions")) {
    for (const auto& s : j["subscriptions"]) {
      subs[s.at("id").get<std::uint32_t>()] =
          s.at("channelId").get<std::uint32_t>();
    }
  } else if (op == "unsubscribe" && j.contains("subscriptionIds")) {
    for (const auto& id : j["subscriptionIds"]) {
      subs.erase(id.get<std::uint32_t>());
    }
  }
  // clientPublish / parameters / getParameters etc. are ignored: this bridge
  // is server-side publish only.
}

// ---------------------------------------------------------------------------
// Bridge
// ---------------------------------------------------------------------------

Channel& Bridge::channel_for(const std::string& bus_topic) {
  auto it = topic_to_id_.find(bus_topic);
  if (it != topic_to_id_.end()) return channels_[it->second];

  Channel ch;
  ch.id = next_id_++;
  ch.topic = "/" + bus_topic;
  ch.encoding = "json";
  if (bus_topic == "camera.image") {
    ch.schema_name = "foxglove.CompressedImage";
    ch.schema = kCompressedImageSchema;
    ch.is_image = true;
  } else {
    ch.schema_name = "JsonMessage";
    ch.schema = kJsonMessageSchema;
  }

  auto [inserted, _] =
      channels_.emplace(ch.id, ch);
  topic_to_id_[bus_topic] = ch.id;

  // Lazy advertise: tell every connected client about the new channel.
  const std::string adv =
      "{\"op\":\"advertise\",\"channels\":[{\"id\":" + std::to_string(ch.id) +
      ",\"topic\":\"" + ch.topic + "\",\"encoding\":\"json\",\"schemaName\":\"" +
      ch.schema_name + "\",\"schema\":" + ch.schema + "}]}";
  for (auto& [ptr, session] : sessions_) session->send_json(adv);

  return channels_[ch.id];
}

void Bridge::handle_incoming(const std::string& bus_topic,
                             const std::string& payload,
                             const std::string& meta, std::uint64_t ts_us) {
  Channel& ch = channel_for(bus_topic);

  const std::uint64_t ts_ns = (ts_us != 0) ? ts_us * 1000ULL : now_ns();

  std::string foxglove_payload;
  if (ch.is_image) {
    // Wrap JPEG + metadata into foxglove.CompressedImage (JSON encoding).
    const auto m = nlohmann::json::parse(meta, nullptr, false);
    const std::string frame_id =
        (m.is_object() && m.contains("frame"))
            ? ("camera/frame_" + std::to_string(m["frame"].get<std::uint64_t>()))
            : "camera";
    nlohmann::json img;
    img["timestamp"]["sec"] = ts_ns / 1'000'000'000ULL;
    img["timestamp"]["nsec"] = ts_ns % 1'000'000'000ULL;
    img["frame_id"] = frame_id;
    img["data"] = aviator::base64_encode(payload);
    img["format"] = "jpeg";
    foxglove_payload = img.dump();
  } else {
    foxglove_payload = payload;  // already a JSON document
  }

  for (auto& [ptr, session] : sessions_) {
    for (const auto& [sub_id, channel_id] : session->subs) {
      if (channel_id == ch.id) {
        session->send_message(sub_id, ts_ns, foxglove_payload);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Server (acceptor)
// ---------------------------------------------------------------------------

class Server {
 public:
  Server(net::io_context& io, const tcp::endpoint& ep, Bridge& bridge)
      : acceptor_(io, ep), bridge_(bridge) {
    do_accept();
  }

 private:
  void do_accept() {
    acceptor_.async_accept(
        beast::bind_front_handler(&Server::on_accept, this));
  }

  void on_accept(beast::error_code ec, tcp::socket socket) {
    if (!ec) {
      auto session = std::make_shared<Session>(std::move(socket), bridge_);
      bridge_.add_session(session);
      session->run();
    }
    do_accept();
  }

  tcp::acceptor acceptor_;
  Bridge& bridge_;
};

// ---------------------------------------------------------------------------
// ZMQ reader (runs on its own thread; posts parsed messages to the io thread)
// ---------------------------------------------------------------------------

std::uint64_t extract_ts_us(const std::string& topic, const std::string& payload,
                            const std::string& meta) {
  const std::string& src =
      (topic == "camera.image" && !meta.empty()) ? meta : payload;
  if (src.empty()) return 0;
  const auto j = nlohmann::json::parse(src, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return 0;
  const auto it = j.find("timestamp");
  if (it == j.end() || !it->is_number()) return 0;
  return it->get<std::uint64_t>();
}

void read_multipart(zmq::socket_t& sock, net::io_context& io, Bridge& bridge) {
  zmq::message_t topic_msg, payload_msg, meta_msg;

  auto got = sock.recv(topic_msg, zmq::recv_flags::dontwait);
  if (!got) return;
  const std::string topic(static_cast<const char*>(topic_msg.data()),
                          topic_msg.size());

  std::string payload;
  if (topic_msg.more()) {
    got = sock.recv(payload_msg, zmq::recv_flags::dontwait);
    if (!got) return;
    payload.assign(static_cast<const char*>(payload_msg.data()),
                   payload_msg.size());
  }

  std::string meta;
  if (payload_msg.more()) {
    got = sock.recv(meta_msg, zmq::recv_flags::dontwait);
    if (!got) return;
    meta.assign(static_cast<const char*>(meta_msg.data()), meta_msg.size());
  }

  // Drain any stray extra frames (robustness; the protocol sends at most 3).
  while (meta_msg.more()) {
    zmq::message_t extra;
    if (!sock.recv(extra, zmq::recv_flags::dontwait)) break;
  }

  const std::uint64_t ts_us = extract_ts_us(topic, payload, meta);

  net::post(io, [&bridge, topic, payload = std::move(payload),
                 meta = std::move(meta), ts_us]() {
    bridge.handle_incoming(topic, payload, meta, ts_us);
  });
}

void zmq_thread(net::io_context& io, Bridge& bridge,
                const std::string& data_ep, const std::string& image_ep) {
  zmq::context_t ctx(1);

  zmq::socket_t sub_data(ctx, zmq::socket_type::sub);
  sub_data.set(zmq::sockopt::subscribe, "");
  sub_data.set(zmq::sockopt::maxmsgsize,
               static_cast<std::int64_t>(1 * 1024 * 1024));
  sub_data.connect(data_ep);

  zmq::socket_t sub_image(ctx, zmq::socket_type::sub);
  sub_image.set(zmq::sockopt::subscribe, "");
  sub_image.set(zmq::sockopt::maxmsgsize,
                static_cast<std::int64_t>(20 * 1024 * 1024));
  sub_image.connect(image_ep);

  while (g_running) {
    zmq::pollitem_t items[] = {
        {sub_data.handle(), 0, ZMQ_POLLIN, 0},
        {sub_image.handle(), 0, ZMQ_POLLIN, 0},
    };
    zmq::poll(items, 2, std::chrono::milliseconds(100));
    if (items[0].revents & ZMQ_POLLIN) read_multipart(sub_data, io, bridge);
    if (items[1].revents & ZMQ_POLLIN) read_multipart(sub_image, io, bridge);
  }
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Args {
  std::string data_ep = "tcp://127.0.0.1:5556";
  std::string image_ep = "tcp://127.0.0.1:5558";
  std::string listen = "0.0.0.0";
  unsigned short port = 8765;
};

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << flag << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    const std::string arg = argv[i];
    if (arg == "--data-endpoint") a.data_ep = next("--data-endpoint");
    else if (arg == "--image-endpoint") a.image_ep = next("--image-endpoint");
    else if (arg == "--listen") a.listen = next("--listen");
    else if (arg == "--port") a.port = static_cast<unsigned short>(std::stoi(next("--port")));
    else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: foxglove_bridge [--data-endpoint URL] "
                   "[--image-endpoint URL] [--listen ADDR] [--port N]\n";
      std::exit(0);
    } else {
      std::cerr << "unknown arg: " << arg << "\n";
      std::exit(2);
    }
  }
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  const Args a = parse_args(argc, argv);

  net::io_context io{1};
  Bridge bridge;

  const auto address = net::ip::make_address(a.listen);
  Server server(io, tcp::endpoint{address, a.port}, bridge);

  std::thread zmq(zmq_thread, std::ref(io), std::ref(bridge), a.data_ep,
                  a.image_ep);

  std::cout << "foxglove_bridge: ws://" << a.listen << ":" << a.port
            << "  (data=" << a.data_ep << ", image=" << a.image_ep << ")\n";

  // On SIGINT/SIGTERM stop the io_context so io.run() returns and the ZMQ
  // reader thread (which polls g_running) exits too.
  net::signal_set signals(io, SIGINT, SIGTERM);
  signals.async_wait([&io](beast::error_code, int) {
    g_running = false;
    io.stop();
  });

  io.run();
  g_running = false;
  if (zmq.joinable()) zmq.join();
  return 0;
}

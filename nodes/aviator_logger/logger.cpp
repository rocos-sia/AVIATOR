#include "logger.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace aviator {
RecorderSummary record_bus(const std::string& endpoint, const std::string& output,
                           const std::string& session, const std::atomic<bool>& stop,
                           const RecorderOptions& options, const std::function<void()>& on_ready) {
    RecordingConfig config{endpoint, output, options};
    validate_recording_config(config);
    const bool cameras = options.camera.mode != "disabled";
    struct Entry {
        WireMessage wire;
        std::uint64_t log_ns = 0, mono_us = 0;
        std::size_t bytes() const {
            return sizeof(Entry) + wire.topic.size() + wire.payload.size();
        }
    };
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<Entry> bus_queue, camera_queue;
    std::size_t bus_bytes = 0, camera_bytes = 0;
    bool done = false, ready = false, receiver_failed = false;
    std::exception_ptr writer_error;
    std::uint64_t rejected = 0, dropped = 0, camera_rejected = 0, camera_dropped = 0;
    RecorderSummary summary;
    zmq::context_t context{1};
    zmq::socket_t sub(context, zmq::socket_type::sub);
    configure(sub, {8, options.receive_hwm, 0});
    subscribe(sub, "");
    sub.connect(endpoint);
    std::unique_ptr<zmq::socket_t> pull;
    if (cameras) {
        pull = std::make_unique<zmq::socket_t>(context, zmq::socket_type::pull);
        pull->set(zmq::sockopt::rcvhwm, options.camera.receive_hwm);
        pull->set(zmq::sockopt::linger, 0);
        pull->set(zmq::sockopt::maxmsgsize,
                  static_cast<std::int64_t>(options.camera.max_record_bytes));
        pull->bind(options.camera.record_endpoint);
    }
    std::thread writer([&] {
        try {
            CameraCompressor compressor(options.camera);
            const auto receive_clock = local_clock_id();
            RecordingWriter recording(output, session, options.chunk_size_bytes,
                                      recording_config_json(config).dump());
            {
                std::lock_guard<std::mutex> lock(mutex);
                ready = true;
            }
            changed.notify_all();
            std::set<std::string> streams;
            std::uint64_t invalid_camera = 0;
            bool prefer_camera = false;
            for (;;) {
                Entry entry;
                bool image;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    changed.wait(
                        lock, [&] { return done || !bus_queue.empty() || !camera_queue.empty(); });
                    if (bus_queue.empty() && camera_queue.empty())
                        break;
                    image = !camera_queue.empty() && (bus_queue.empty() || prefer_camera);
                    prefer_camera = !image;
                    auto& queue = image ? camera_queue : bus_queue;
                    entry = std::move(queue.front());
                    (image ? camera_bytes : bus_bytes) -= entry.bytes();
                    queue.pop_front();
                }
                if (!image) {
                    recording.append(entry.wire.topic, entry.wire.payload, entry.log_ns);
                    continue;
                }
                CameraFrame frame;
                std::string topic;
                try {
                    frame = parse_camera_frame(entry.wire.payload);
                    topic = validate_camera_frame(frame, options.camera);
                } catch (const std::invalid_argument& error) {
                    ++invalid_camera;
                    if (invalid_camera == 1)
                        std::cerr << "aviator_logger DEGRADED: invalid camera frame: "
                                  << error.what() << '\n';
                    continue;
                }
                frame.metadata["receive_mono_us"] = entry.mono_us;
                frame.metadata["receive_clock_id"] = receive_clock;
                frame = compressor.encode(std::move(frame));
                // Never discard an encoded reference frame and then write dependent frames.
                if (serialize_camera_frame(frame).size() > options.camera.max_record_bytes)
                    throw std::runtime_error(
                        "encoded camera packet exceeds max_record_bytes; partial retained");
                recording.append_camera(topic, frame, entry.log_ns);
                streams.insert(topic);
            }
            if (receiver_failed)
                throw std::runtime_error("receiver failed; recording retained as partial");
            nlohmann::json missing = nlohmann::json::array();
            if (cameras)
                for (const auto& source : options.camera.sources) {
                    if (!streams.count(source.rgb_topic))
                        missing.push_back(source.rgb_topic);
                    if (source.record_depth && !streams.count(source.depth_topic))
                        missing.push_back(source.depth_topic);
                }
            const bool degraded =
                invalid_camera || camera_rejected || camera_dropped || !missing.empty();
            recording.metadata(
                "camera_recording",
                {{"mode", options.camera.mode},
                 {"state", degraded ? "DEGRADED" : (cameras ? "RECORDING" : "DISABLED")},
                 {"invalid", invalid_camera},
                 {"rejected", camera_rejected},
                 {"queue_dropped", camera_dropped},
                 {"missing_streams", missing},
                 {"rgb_lossless", options.camera.mode != "compressed"}});
            if (degraded)
                std::cerr << "aviator_logger DEGRADED: camera invalid=" << invalid_camera
                          << " rejected=" << camera_rejected << " dropped=" << camera_dropped
                          << " missing=" << missing.dump() << '\n';
            summary = recording.finish(rejected + camera_rejected + invalid_camera,
                                       dropped + camera_dropped);
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex);
            writer_error = std::current_exception();
        }
        changed.notify_all();
    });
    std::exception_ptr receiver_error;
    try {
        {
            std::unique_lock<std::mutex> lock(mutex);
            changed.wait(lock, [&] { return ready || writer_error; });
            if (writer_error)
                std::rethrow_exception(writer_error);
        }
        if (on_ready)
            on_ready();
        ReceiveState receiving;
        bool draining_camera = false;
        std::string error;
        const auto enqueue = [&](Entry entry, bool image) {
            entry.log_ns = utc_us() * 1000ULL;
            entry.mono_us = monotonic_us();
            std::lock_guard<std::mutex> lock(mutex);
            auto& bytes = image ? camera_bytes : bus_bytes;
            const auto limit = image ? options.camera.queue_bytes : options.queue_bytes;
            if (entry.bytes() > limit - bytes) {
                auto& count = image ? camera_dropped : dropped;
                if (++count == 1)
                    std::cerr << "aviator_logger DEGRADED: " << (image ? "camera" : "bus")
                              << " queue overflow\n";
                return;
            }
            bytes += entry.bytes();
            (image ? camera_queue : bus_queue).push_back(std::move(entry));
            changed.notify_one();
        };
        while (!stop.load()) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (writer_error)
                    std::rethrow_exception(writer_error);
            }
            zmq::pollitem_t items[] = {{sub.handle(), 0, ZMQ_POLLIN, 0},
                                       {pull ? pull->handle() : nullptr, 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, cameras ? 2 : 1, std::chrono::milliseconds(20));
            for (unsigned i = 0; i < 32 && !stop.load(); ++i) {
                Entry entry;
                auto result = receive(sub, receiving, entry.wire, error);
                if (result == ReceiveResult::empty)
                    break;
                if (result == ReceiveResult::rejected) {
                    ++rejected;
                    continue;
                }
                enqueue(std::move(entry), false);
            }
            if (pull)
                for (unsigned i = 0; i < 32 && !stop.load(); ++i) {
                    zmq::message_t frame;
                    if (!pull->recv(frame, zmq::recv_flags::dontwait))
                        break;
                    if (draining_camera || frame.more()) {
                        draining_camera = frame.more();
                        if (!draining_camera)
                            ++camera_rejected;
                        continue;
                    }
                    if (frame.size() == 0 || frame.size() > options.camera.max_record_bytes) {
                        ++camera_rejected;
                        continue;
                    }
                    Entry entry;
                    entry.wire.payload = frame.to_string();
                    enqueue(std::move(entry), true);
                }
        }
        if (draining_camera)
            ++camera_rejected;
    } catch (...) {
        receiver_error = std::current_exception();
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        receiver_failed = bool(receiver_error);
        done = true;
    }
    changed.notify_one();
    writer.join();
    if (writer_error)
        std::rethrow_exception(writer_error);
    if (receiver_error)
        std::rethrow_exception(receiver_error);
    return summary;
}
} // namespace aviator

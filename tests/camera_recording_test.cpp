#include "camera_recording.hpp"
#include "logger.hpp"
#include "protocol.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <mcap/mcap.hpp>
#include <zstd.h>
extern "C" {
#include <libavcodec/avcodec.h>
}
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace std::chrono_literals;
namespace {
void check(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
template <class F> void rejects(F f) {
    bool bad = false;
    try {
        f();
    } catch (const std::exception&) {
        bad = true;
    }
    check(bad, "expected failure");
}
aviator::CameraFrame frame(bool rgb, unsigned sequence) {
    aviator::CameraFrame f;
    f.metadata = {{"version", 1},
                  {"camera_id", "cockpit"},
                  {"stream", rgb ? "rgb" : "depth"},
                  {"publisher_id", "camera"},
                  {"session_id", "run-camera-test"},
                  {"clock_id", "test-boot"},
                  {"config_id", "test-calibration-v1"},
                  {"pixel_format", rgb ? "RGB8" : "Z16"},
                  {"byte_order", "little"},
                  {"encoding", "raw"},
                  {"width", 64},
                  {"height", 64},
                  {"stride_bytes", 64 * (rgb ? 3 : 2)},
                  {"fps", 30},
                  {"sequence", sequence},
                  {"timestamp_us", 1700000000000000ULL + sequence},
                  {"sample_mono_us", 1000 + sequence},
                  {"depth_scale", 0.001},
                  {"calibration", {{"test_pattern", true}}}};
    f.data.resize(64 * 64 * (rgb ? 3 : 2));
    for (std::size_t i = 0; i < f.data.size(); ++i)
        f.data[i] = static_cast<char>((i + sequence) % 251);
    return aviator::parse_camera_frame(aviator::serialize_camera_frame(f));
}
struct Decoder {
    AVCodecContext* c;
    AVFrame* image = av_frame_alloc();
    AVPacket* packet = av_packet_alloc();
    int frames = 0;
    explicit Decoder(const std::string& codec) {
        c = avcodec_alloc_context3(
            avcodec_find_decoder(codec == "h265" ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264));
        check(c && image && packet && avcodec_open2(c, c->codec, nullptr) >= 0, "open decoder");
    }
    ~Decoder() {
        av_packet_free(&packet);
        av_frame_free(&image);
        avcodec_free_context(&c);
    }
    void append(const std::string& bytes) {
        check(av_new_packet(packet, bytes.size()) >= 0, "allocate packet");
        std::memcpy(packet->data, bytes.data(), bytes.size());
        check(avcodec_send_packet(c, packet) >= 0, "decode packet");
        av_packet_unref(packet);
        while (avcodec_receive_frame(c, image) == 0) {
            check(image->width == 64 && image->height == 64, "decoded dimensions");
            ++frames;
        }
    }
};
void config_tests(const std::filesystem::path& dir, const char* config_path) {
    auto cfg = aviator::load_recording_config(config_path);
    check(cfg.options.camera.mode == "compressed", "sample config enables compressed images");
    check(cfg.options.camera.codec == "h264" && cfg.options.camera.encoder == "software",
          "sample config supports CPU-only recording");
    check(!cfg.options.camera.sources[0].record_depth, "sample config disables depth");
    const auto file = dir / "config.yaml";
    for (const auto* bad :
         {"config_version: 2", "config_version: 1\nconfig_version: 1",
          "config_version: 1\nunknown: 1", "config_version: 1\ncamera: {mode: false}",
          "config_version: 1\nbus: {receive_hwm: -1}",
          "config_version: 1\nbus: {receive_hwm: '32'}",
          "config_version: 1\ncamera: {compressed: {rgb: {b_frames: 1}}}",
          "config_version: 1\ncamera: {sources: []}"}) {
        {
            std::ofstream out(file);
            out << bad;
        }
        rejects([&] { aviator::load_recording_config(file.string()); });
    }
    {
        std::ofstream out(file);
        out << "config_version: 1\ncamera: {mode: raw}\noutput: {chunk_size_bytes: 65536, image_path: custom.mcap}\n";
    }
    cfg = aviator::load_recording_config(file.string());
    check(cfg.options.camera.mode == "raw" && cfg.options.chunk_size_bytes == 65536,
          "partial config merges defaults");
    check(cfg.options.image_output == "custom.mcap", "parse image output");
    cfg.options.image_output.clear();
    check(cfg.options.camera.sources[0].record_depth, "omitted record_depth stays enabled");
    {
        std::ofstream out(file);
        out << "config_version: 1\ncamera:\n  mode: raw\n  sources:\n"
               "    - camera_id: cockpit\n      rgb_topic: record.camera.cockpit.rgb\n"
               "      depth_topic: record.camera.cockpit.depth\n"
               "      rgb_pixel_format: RGB8\n      depth_pixel_format: Z16\n"
               "      record_depth: false\n";
    }
    auto rgb_only = aviator::load_recording_config(file.string());
    check(!rgb_only.options.camera.sources[0].record_depth, "parse RGB-only source");
    check(aviator::recording_config_json(rgb_only)["camera"]["sources"][0]["record_depth"] == false,
          "effective config records depth choice");
    rejects([&] {
        aviator::validate_camera_frame(frame(false, 1), rgb_only.options.camera);
    });
    {
        std::ofstream out(file);
        out << "config_version: 1\ncamera:\n  sources:\n"
               "    - camera_id: cockpit\n      rgb_topic: record.camera.cockpit.rgb\n"
               "      depth_topic: record.camera.cockpit.depth\n"
               "      rgb_pixel_format: RGB8\n      depth_pixel_format: Z16\n"
               "      record_depth: 'false'\n";
    }
    rejects([&] { aviator::load_recording_config(file.string()); });
    {
        zmq::context_t context(1);
        zmq::socket_t occupied(context, zmq::socket_type::pull);
        occupied.bind("tcp://127.0.0.1:*");
        auto options = cfg.options;
        options.camera.record_endpoint = occupied.get(zmq::sockopt::last_endpoint);
        std::atomic<bool> stop{true};
        const auto output = (dir / "occupied.mcap").string();
        rejects([&] {
            aviator::record_bus("tcp://127.0.0.1:59999", output, "session", stop, options);
        });
        check(!std::filesystem::exists(output + ".partial"),
              "bind failure must not create recording");
    }
    for (const auto& image_path : {std::string("data.mcap"), std::string("./data.mcap"),
                                   std::string("data.mcap.partial")}) {
        auto conflict = cfg;
        conflict.output = "data.mcap";
        conflict.options.image_output = image_path;
        rejects([&] { aviator::validate_recording_config(conflict); });
    }
    auto good = frame(true, 1);
    check(aviator::validate_camera_frame(good, cfg.options.camera) == "record.camera.cockpit.rgb",
          "valid ingress");
    auto bad = good;
    bad.data.pop_back();
    rejects([&] { aviator::validate_camera_frame(bad, cfg.options.camera); });
    bad = good;
    bad.metadata["camera_id"] = "unknown";
    rejects([&] { aviator::validate_camera_frame(bad, cfg.options.camera); });
    bad = good;
    bad.metadata["encoding"] = "h265";
    rejects([&] { aviator::validate_camera_frame(bad, cfg.options.camera); });
    rejects([&] { aviator::parse_camera_frame("not protobuf"); });
}
void transport(const std::filesystem::path& dir, const std::string& mode, const std::string& codec,
               bool overflow = false, bool rgb_only = false) {
    aviator::RecorderOptions options;
    options.camera.mode = mode;
    if (rgb_only)
        options.image_output = (dir / (mode + codec + "-custom-images.mcap")).string();
    options.camera.codec = codec;
    options.camera.encoder = "software";
    options.camera.keyframe_interval = 3;
    options.camera.sources[0].record_depth = !rgb_only;
    if (overflow)
        options.camera.max_record_bytes = options.camera.queue_bytes = 20000;
    zmq::context_t context(1);
    zmq::socket_t reserve(context, zmq::socket_type::pull);
    reserve.bind("tcp://127.0.0.1:*");
    options.camera.record_endpoint = reserve.get(zmq::sockopt::last_endpoint);
    reserve.close();
    zmq::socket_t push(context, zmq::socket_type::push);
    push.set(zmq::sockopt::sndtimeo, 3000);
    push.set(zmq::sockopt::linger, 0);
    zmq::socket_t pub(context, zmq::socket_type::pub);
    pub.bind("tcp://127.0.0.1:*");
    const auto bus_endpoint = pub.get(zmq::sockopt::last_endpoint);
    std::atomic<bool> stop{false};
    std::promise<void> ready;
    auto ready_future = ready.get_future();
    const auto path = (dir / (mode + codec + (overflow ? "-overflow" : "") +
                              (rgb_only ? "-rgb-only" : "") + ".mcap")).string();
    auto task = std::async(std::launch::async, [&] {
        return aviator::record_bus(bus_endpoint, path, "logger", stop, options,
                                   [&] { ready.set_value(); });
    });
    try {
        check(ready_future.wait_for(5s) == std::future_status::ready, "camera logger startup");
        std::this_thread::sleep_for(200ms);
        aviator::Message message;
        message.topic = aviator::Topic::flight_command;
        message.header = {"1.0", 1, 1790121600000000, 1000000, "boot", "producer",
                          "11111111-1111-4111-8111-111111111111", true};
        message.body = {{"source", "JOYSTICK"}, {"control", {{"roll", 0.2}, {"pitch", 0.1}}}};
        std::string bus_payload, error;
        check(aviator::encode(message, bus_payload, error), "encode business fixture");
        pub.send(zmq::buffer(std::string("flight.command")), zmq::send_flags::sndmore);
        pub.send(zmq::buffer(bus_payload));
        std::this_thread::sleep_for(100ms);
        if (mode == "disabled") {
            reserve = zmq::socket_t(context, zmq::socket_type::pull);
            reserve.bind(options.camera.record_endpoint); // Disabled mode did not bind it.
        } else {
            push.connect(options.camera.record_endpoint);
            for (unsigned sequence = 1; sequence <= 8; ++sequence) {
                for (bool rgb : {true, false}) {
                    if (!rgb && rgb_only)
                        continue;
                    auto input = frame(rgb, sequence);
                    if (overflow) {
                        input.metadata["padding"] = "";
                        const auto base = aviator::serialize_camera_frame(input).size();
                        input.metadata["padding"] = std::string(20000 - base, 'x');
                    }
                    auto data = aviator::serialize_camera_frame(input);
                    if (overflow)
                        check(data.size() == 20000, "overflow fixture size");
                    check(push.send(zmq::buffer(data)).has_value(), "send image");
                }
            }
            check(push.send(zmq::buffer(std::string("malformed"))).has_value(), "send invalid");
            push.send(zmq::buffer(std::string("bad")), zmq::send_flags::sndmore);
            push.send(zmq::buffer(std::string("multipart")));
            std::this_thread::sleep_for(1s);
        }
        stop.store(true);
        check(task.wait_for(10s) == std::future_status::ready, "camera drain timeout");
        const auto summary = task.get();
        check(summary.camera_messages == (mode == "disabled" || overflow ? 0 :
                                          (rgb_only ? 8 : 16)),
              "camera message count");
        check(summary.dropped == (overflow ? 16 : 0), "image queue overflow count");
        check(summary.rejected == (mode == "disabled" ? 0 : 2), "camera reject count");
        mcap::McapReader reader;
        check(reader.open(path).ok(), "data MCAP open");
        check(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok(), "data MCAP index");
        int bus_count = 0;
        for (const auto& view : reader.readMessages()) {
            check(view.schema->encoding == "jsonschema" && view.channel->topic == "flight.command",
                  "only business messages in data file");
            check(std::string(reinterpret_cast<const char*>(view.message.data), view.message.dataSize)
                      == bus_payload, "business payload unchanged");
            ++bus_count;
        }
        check(bus_count == 1, "business message retained with image recording");
        reader.close();
        const auto image_path = aviator::image_output_path(path, options);
        check(std::filesystem::exists(image_path) == (mode != "disabled"), "image file lifecycle");
        if (mode == "disabled")
            return;
        check(reader.open(image_path).ok(), "MCAP open");
        check(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok(), "MCAP index");
        Decoder decoder(codec);
        int rgb_count = 0, depth_count = 0;
        for (const auto& view : reader.readMessages()) {
            check(view.schema->name == aviator::camera_schema_name &&
                      view.schema->encoding == "protobuf",
                  "camera schema");
            auto f = aviator::parse_camera_frame(std::string_view(
                reinterpret_cast<const char*>(view.message.data), view.message.dataSize));
            const bool rgb = f.metadata.at("stream") == "rgb";
            const auto original = frame(rgb, f.metadata.at("sequence"));
            if (mode == "raw")
                check(f.data == original.data, "raw bytes identical");
            else if (!rgb) {
                std::string result(original.data.size(), '\0');
                check(ZSTD_decompress(result.data(), result.size(), f.data.data(), f.data.size()) ==
                          result.size(),
                      "depth decompression");
                check(result == original.data, "depth lossless");
            } else {
                check(f.metadata.at("encoder") == (codec == "h265" ? "libx265" : "libx264"),
                      "software encoder selected");
                check(f.metadata.at("encoder_preset") == (codec == "h265" ? "ultrafast" : "veryfast"),
                      "software preset recorded");
                if (!rgb_count)
                    check(f.metadata.at("keyframe").get<bool>(), "first frame is keyframe");
                decoder.append(f.data);
            }
            if (rgb)
                ++rgb_count;
            else
                ++depth_count;
        }
        if (mode == "compressed")
            check(decoder.frames == 8, "all RGB frames decode");
        check(depth_count == (rgb_only ? 0 : rgb_count) &&
                  rgb_count == (mode == "disabled" || overflow ? 0 : 8),
              "all frames retained");
    } catch (...) {
        stop.store(true);
        if (task.valid())
            task.wait();
        throw;
    }
}
} // namespace
int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--python-packet") {
        try {
            std::ifstream input(argv[2], std::ios::binary);
            const std::string bytes((std::istreambuf_iterator<char>(input)), {});
            auto frame = aviator::parse_camera_frame(bytes);
            check(aviator::validate_camera_frame(frame, {}) == "record.camera.cockpit.depth",
                  "Python metadata contract");
            check(frame.metadata.at("sequence").get<std::uint64_t>() == 4294967297ULL,
                  "full source sequence");
            check(frame.data == std::string("\x00\xff\x01\x80\x02\x00\xff\x7f", 8),
                  "binary payload from Python");
            return 0;
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
            return 1;
        }
    }
    auto dir = std::filesystem::temp_directory_path() /
               ("aviator-camera-test-" + std::to_string(getpid()));
    try {
        check(argc == 2 && std::filesystem::create_directory(dir), "test setup");
        config_tests(dir, argv[1]);
        transport(dir, "disabled", "h264");
        transport(dir, "raw", "h264");
        transport(dir, "raw", "h264", true);
        transport(dir, "compressed", "h264");
        transport(dir, "compressed", "h265");
        transport(dir, "raw", "h264", false, true);
        transport(dir, "compressed", "h264", false, true);
        std::filesystem::remove_all(dir);
        std::cout << "camera recording tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << " artifacts: " << dir << '\n';
        return 1;
    }
}

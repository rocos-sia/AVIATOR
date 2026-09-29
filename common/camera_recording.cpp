#include "camera_recording.hpp"
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <zstd.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <climits>
#include <cmath>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>

namespace aviator {
namespace {
struct Protocol {
    google::protobuf::FileDescriptorProto file;
    google::protobuf::DescriptorPool pool;
    google::protobuf::DynamicMessageFactory factory;
    const google::protobuf::Message* prototype;
    Protocol() : factory(&pool) {
        file.set_name("camera_packet.proto");
        file.set_package("aviator.record.v1");
        file.set_syntax("proto3");
        auto* message = file.add_message_type();
        message->set_name("CameraPacket");
        for (int i = 1; i <= 2; ++i) {
            auto* field = message->add_field();
            field->set_number(i);
            field->set_name(i == 1 ? "metadata_json" : "data");
            field->set_type(i == 1 ? google::protobuf::FieldDescriptorProto::TYPE_STRING
                                   : google::protobuf::FieldDescriptorProto::TYPE_BYTES);
            field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
        }
        const auto* built = pool.BuildFile(file);
        if (!built)
            throw std::runtime_error("camera protobuf descriptor failed");
        prototype = factory.GetPrototype(built->message_type(0));
    }
};
const Protocol& protocol() {
    static const Protocol p;
    return p;
}
void valid(bool condition, const char* error) {
    if (!condition)
        throw std::invalid_argument(error);
}
std::uint64_t uint(const nlohmann::json& m, const char* field, std::uint64_t max = UINT64_MAX) {
    valid(m.contains(field) && m[field].is_number_unsigned(), field);
    auto n = m[field].get<std::uint64_t>();
    valid(n > 0 && n <= max, field);
    return n;
}
void avcheck(int code, const char* operation) {
    if (code >= 0)
        return;
    char error[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(code, error, sizeof(error));
    throw std::runtime_error(std::string(operation) + ": " + error);
}
struct Video {
    AVCodecContext* context = nullptr;
    AVFrame* image = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* scale = nullptr;
    std::string backend, preset, identity;
    std::int64_t index = 0;
    ~Video() {
        sws_freeContext(scale);
        av_packet_free(&packet);
        av_frame_free(&image);
        avcodec_free_context(&context);
    }
};
std::unique_ptr<Video> open_video(const CameraRecordingOptions& o, int width, int height, int fps) {
    std::vector<std::string> candidates;
    if (o.encoder != "software")
        candidates.push_back(o.codec == "h265" ? "hevc_nvenc" : "h264_nvenc");
    if (o.encoder != "hardware")
        candidates.push_back(o.codec == "h265" ? "libx265" : "libx264");
    std::string failures;
    for (const auto& name : candidates) {
        const auto* codec = avcodec_find_encoder_by_name(name.c_str());
        if (!codec) {
            failures += name + " not built; ";
            continue;
        }
        auto v = std::make_unique<Video>();
        v->context = avcodec_alloc_context3(codec);
        if (!v->context)
            throw std::bad_alloc();
        auto* c = v->context;
        c->width = width;
        c->height = height;
        c->pix_fmt = AV_PIX_FMT_YUV420P;
        c->time_base = AVRational{1, fps};
        c->framerate = AVRational{fps, 1};
        c->bit_rate = o.bitrate;
        c->gop_size = o.keyframe_interval;
        c->max_b_frames = 0;
        c->thread_count = 4;
        // Zero latency keeps the one-input-frame/one-output-packet contract.
        // x265 parallelizes rows via WPP; frame threads must stay at one.
        AVDictionary* dict = nullptr;
        if (name == "libx264" || name == "libx265") {
            v->preset = name == "libx265" ? "ultrafast" : "veryfast";
            av_dict_set(&dict, "preset", v->preset.c_str(), 0);
            av_dict_set(&dict, "tune", "zerolatency", 0);
            av_dict_set(
                &dict, name == "libx264" ? "x264-params" : "x265-params",
                name == "libx264"
                    ? "repeat-headers=1:scenecut=0"
                    : "repeat-headers=1:scenecut=0:pools=4:wpp=1:frame-threads=1:log-level=error",
                0);
        } else {
            v->preset = "default";
            av_dict_set(&dict, "zerolatency", "1", 0);
            av_dict_set(&dict, "delay", "0", 0);
            av_dict_set(&dict, "rc-lookahead", "0", 0);
            av_dict_set(&dict, "forced-idr", "1", 0);
        }
        const int result = avcodec_open2(c, codec, &dict);
        av_dict_free(&dict);
        if (result < 0) {
            failures += name + " unavailable; ";
            continue;
        }
        v->image = av_frame_alloc();
        v->packet = av_packet_alloc();
        if (!v->image || !v->packet)
            throw std::bad_alloc();
        v->image->format = c->pix_fmt;
        v->image->width = width;
        v->image->height = height;
        avcheck(av_frame_get_buffer(v->image, 32), "video frame buffer");
        v->scale = sws_getContext(width, height, AV_PIX_FMT_RGB24, width, height,
                                  AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!v->scale)
            throw std::runtime_error("RGB conversion unavailable");
        v->backend = name;
        return v;
    }
    throw std::runtime_error("no encoder for " + o.codec + ": " + failures);
}
} // namespace
std::string camera_schema_descriptor() {
    google::protobuf::FileDescriptorSet set;
    *set.add_file() = protocol().file;
    return set.SerializeAsString();
}
std::string serialize_camera_frame(const CameraFrame& frame) {
    const auto& p = protocol();
    std::unique_ptr<google::protobuf::Message> message(p.prototype->New());
    const auto* r = message->GetReflection();
    const auto* d = message->GetDescriptor();
    const auto header = frame.metadata.dump();
    valid(header.size() <= 65536, "encoded metadata too large");
    r->SetString(message.get(), d->FindFieldByNumber(1), header);
    r->SetString(message.get(), d->FindFieldByNumber(2), frame.data);
    return message->SerializeAsString();
}
CameraFrame parse_camera_frame(std::string_view bytes) {
    valid(bytes.size() <= INT_MAX, "camera packet too large");
    std::unique_ptr<google::protobuf::Message> message(protocol().prototype->New());
    valid(message->ParseFromArray(bytes.data(), bytes.size()), "invalid camera protobuf");
    const auto* r = message->GetReflection();
    const auto* d = message->GetDescriptor();
    const auto header = r->GetString(*message, d->FindFieldByNumber(1));
    valid(header.size() <= 65536, "camera metadata too large");
    CameraFrame frame;
    try {
        std::vector<std::set<std::string>> keys;
        frame.metadata = nlohmann::json::parse(
            header, [&](int depth, nlohmann::json::parse_event_t event, nlohmann::json& value) {
                valid(depth <= 32, "camera metadata nesting limit");
                if (event == nlohmann::json::parse_event_t::object_start)
                    keys.emplace_back();
                if (event == nlohmann::json::parse_event_t::key)
                    valid(keys.back().insert(value.get<std::string>()).second,
                          "duplicate camera metadata key");
                if (event == nlohmann::json::parse_event_t::object_end)
                    keys.pop_back();
                return true;
            });
    } catch (const nlohmann::json::exception&) {
        throw std::invalid_argument("invalid camera metadata JSON");
    }
    frame.data = r->GetString(*message, d->FindFieldByNumber(2));
    return frame;
}
std::string validate_camera_frame(const CameraFrame& f, const CameraRecordingOptions& o) {
    try {
        const auto& m = f.metadata;
        valid(m.is_object(), "camera metadata must be object");
        valid(uint(m, "version", 1) == 1, "unsupported camera version");
        for (auto key : {"camera_id", "stream", "publisher_id", "session_id", "clock_id",
                         "config_id", "pixel_format", "byte_order", "encoding"})
            valid(m.contains(key) && m[key].is_string() &&
                      !m[key].get_ref<const std::string&>().empty() &&
                      m[key].get_ref<const std::string&>().size() <= 256,
                  key);
        valid(
            std::regex_match(
                m["session_id"].get<std::string>(),
                std::regex(
                    "[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}")),
            "invalid source session UUID");
        valid(m["encoding"] == "raw", "camera ingress requires raw frames");
        valid(m["stream"] == "rgb" || m["stream"] == "depth", "unknown camera stream");
        const bool rgb = m["stream"] == "rgb";
        valid(m["pixel_format"] == (rgb ? "RGB8" : "Z16"), "pixel format mismatch");
        valid(m["byte_order"] == "little", "only little-endian supported");
        const auto width = uint(m, "width", 16384), height = uint(m, "height", 16384);
        const auto stride = uint(m, "stride_bytes", INT_MAX);
        valid(stride >= width * (rgb ? 3 : 2) && stride * height == f.data.size(),
              "stride or payload length mismatch");
        uint(m, "sequence");
        uint(m, "timestamp_us", UINT64_MAX / 1000);
        uint(m, "sample_mono_us");
        uint(m, "fps", 240);
        valid(m.contains("calibration") && m["calibration"].is_object() &&
                  !m["calibration"].empty(),
              "missing calibration snapshot");
        if (!rgb)
            valid(m.contains("depth_scale") && m["depth_scale"].is_number() &&
                      std::isfinite(m["depth_scale"].get<double>()) &&
                      m["depth_scale"].get<double>() > 0,
                  "invalid depth_scale");
        if (rgb && o.mode == "compressed")
            valid(width % 2 == 0 && height % 2 == 0, "yuv420p requires even dimensions");
        for (const auto& s : o.sources) {
            if (s.camera_id == m["camera_id"]) {
                if (rgb)
                    return s.rgb_topic;
                valid(s.record_depth, "depth recording disabled for camera_id");
                return s.depth_topic;
            }
        }
        throw std::invalid_argument("unregistered camera_id");
    } catch (const nlohmann::json::exception&) {
        throw std::invalid_argument("invalid camera metadata types");
    }
}
struct CameraCompressor::Impl {
    CameraRecordingOptions options;
    std::map<std::string, std::unique_ptr<Video>> videos;
    std::string backend = "none", preset = "none";
    explicit Impl(const CameraRecordingOptions& o) : options(o) {
        // Check actual encoder initialization before reporting the logger as listening.
        if (o.mode == "compressed") {
            auto probe = open_video(o, 64, 64, 30);
            backend = probe->backend;
            preset = probe->preset;
        }
    }
};
CameraCompressor::CameraCompressor(const CameraRecordingOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
CameraCompressor::~CameraCompressor() = default;
const std::string& CameraCompressor::encoder_backend() const { return impl_->backend; }
const std::string& CameraCompressor::encoder_preset() const { return impl_->preset; }
CameraFrame CameraCompressor::encode(CameraFrame frame) {
    const auto& o = impl_->options;
    if (o.mode != "compressed")
        return frame;
    auto& m = frame.metadata;
    m["original_size_bytes"] = frame.data.size();
    if (m["stream"] == "depth") {
        std::string compressed(ZSTD_compressBound(frame.data.size()), '\0');
        const auto size = ZSTD_compress(compressed.data(), compressed.size(), frame.data.data(),
                                        frame.data.size(), o.zstd_level);
        if (ZSTD_isError(size))
            throw std::runtime_error(ZSTD_getErrorName(size));
        compressed.resize(size);
        frame.data = std::move(compressed);
        m["encoding"] = "zstd";
        m["lossless"] = true;
        m["encoder"] = "libzstd";
    } else {
        auto& video = impl_->videos[m.at("camera_id").get<std::string>()];
        const auto identity =
            nlohmann::json::array({m["publisher_id"], m["session_id"], m["clock_id"], m["width"],
                                   m["height"], m["fps"], m["config_id"]})
                .dump();
        if (!video || video->identity != identity) {
            video = open_video(o, m.at("width"), m.at("height"), m.at("fps"));
            video->identity = identity;
            impl_->backend = video->backend;
            impl_->preset = video->preset;
        }
        avcheck(av_frame_make_writable(video->image), "writable video frame");
        const uint8_t* source[] = {reinterpret_cast<const uint8_t*>(frame.data.data())};
        const int stride[] = {m.at("stride_bytes").get<int>()};
        if (sws_scale(video->scale, source, stride, 0, video->context->height, video->image->data,
                      video->image->linesize) != video->context->height)
            throw std::runtime_error("incomplete RGB conversion");
        video->image->pts = video->index++;
        video->image->pict_type = (video->image->pts % o.keyframe_interval == 0)
                                      ? AV_PICTURE_TYPE_I
                                      : AV_PICTURE_TYPE_NONE;
        avcheck(avcodec_send_frame(video->context, video->image), "encode video");
        avcheck(avcodec_receive_packet(video->context, video->packet),
                "encoder must produce one packet without delay");
        m["keyframe"] = bool(video->packet->flags & AV_PKT_FLAG_KEY);
        frame.data.assign(reinterpret_cast<const char*>(video->packet->data), video->packet->size);
        av_packet_unref(video->packet);
        const auto extra = avcodec_receive_packet(video->context, video->packet);
        if (extra != AVERROR(EAGAIN)) {
            av_packet_unref(video->packet);
            throw std::runtime_error("unexpected extra video packet");
        }
        m["encoding"] = o.codec;
        m["lossless"] = false;
        m["encoder"] = video->backend;
        m["encoder_preset"] = video->preset;
        m["encoded_pixel_format"] = "yuv420p";
        m["bitstream_format"] = "annexb";
    }
    return frame;
}
} // namespace aviator

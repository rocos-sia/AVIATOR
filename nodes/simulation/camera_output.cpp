#include "camera_output.hpp"
#include "transport.hpp"
#include <yaml-cpp/yaml.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace simulation {
namespace {
void require(bool ok, const char* error) {
    if (!ok)
        throw std::runtime_error(error);
}
struct Jpeg {
    AVCodecContext* context = nullptr;
    AVFrame* frame = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* scale = nullptr;
    int width = 0, height = 0;
    std::int64_t sequence = 0;
    ~Jpeg() {
        sws_freeContext(scale);
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&context);
    }
    std::string encode(const std::vector<unsigned char>& rgb, int w, int h,
                       const PreviewSettings& settings) {
        if (!context) {
            const double ratio =
                std::min({1.0, double(settings.width) / w, double(settings.height) / h});
            width = std::max(1, int(std::lround(w * ratio)));
            height = std::max(1, int(std::lround(h * ratio)));
            const auto* codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
            require(codec != nullptr, "MJPEG encoder unavailable");
            context = avcodec_alloc_context3(codec);
            require(context != nullptr, "JPEG context allocation failed");
            context->width = width;
            context->height = height;
            context->pix_fmt = AV_PIX_FMT_YUVJ444P;
            context->color_range = AVCOL_RANGE_JPEG;
            context->time_base = {1, settings.fps};
            context->thread_count = 1;
            context->flags |= AV_CODEC_FLAG_QSCALE;
            context->global_quality =
                FF_QP2LAMBDA * std::clamp((100 - settings.quality) * 31 / 100, 2, 31);
            require(avcodec_open2(context, codec, nullptr) >= 0, "JPEG encoder open failed");
            frame = av_frame_alloc();
            packet = av_packet_alloc();
            require(frame && packet, "JPEG buffer allocation failed");
            frame->format = context->pix_fmt;
            frame->width = width;
            frame->height = height;
            frame->quality = context->global_quality;
            require(av_frame_get_buffer(frame, 32) >= 0, "JPEG frame buffer failed");
            scale = sws_getContext(w, h, AV_PIX_FMT_RGB24, width, height, context->pix_fmt,
                                   SWS_AREA, nullptr, nullptr, nullptr);
            require(scale != nullptr, "JPEG resize context failed");
        }
        require(av_frame_make_writable(frame) >= 0, "JPEG frame not writable");
        const unsigned char* input[] = {rgb.data()};
        const int stride[] = {w * 3};
        require(sws_scale(scale, input, stride, 0, h, frame->data, frame->linesize) == height,
                "JPEG resize failed");
        frame->pts = sequence++;
        require(avcodec_send_frame(context, frame) >= 0, "JPEG encode failed");
        require(avcodec_receive_packet(context, packet) >= 0, "JPEG packet unavailable");
        std::string result(reinterpret_cast<const char*>(packet->data), packet->size);
        av_packet_unref(packet);
        return result;
    }
};
} // namespace
PreviewSettings previewSettings(const std::string& path, const std::string& override_endpoint) {
    PreviewSettings result;
    if (!path.empty()) {
        const auto p = YAML::LoadFile(path)["preview"];
        if (p) {
            result.enabled = p["enabled"].as<bool>(true);
            result.endpoint = p["endpoint"].as<std::string>(result.endpoint);
            result.width = p["width"].as<int>(result.width);
            result.height = p["height"].as<int>(result.height);
            result.fps = p["fps"].as<int>(result.fps);
            result.quality = p["jpeg_quality"].as<int>(result.quality);
        }
    }
    if (!override_endpoint.empty()) {
        result.enabled = override_endpoint != "off";
        if (result.enabled)
            result.endpoint = override_endpoint;
    }
    require(result.endpoint.rfind("tcp://127.0.0.1:", 0) == 0, "preview must bind loopback TCP");
    require(result.width >= 16 && result.width <= 1920 && result.height >= 16 &&
                result.height <= 1080 && result.fps >= 1 && result.fps <= 30 &&
                result.quality >= 30 && result.quality <= 95,
            "invalid preview size/rate/JPEG quality");
    return result;
}
CameraOutput::CameraOutput(PreviewSettings preview, const std::string& recording_path,
                           const std::string& camera_id)
    : preview_(std::move(preview)), camera_id_(camera_id) {
    if (!recording_path.empty())
        recording_ = aviator::load_recording_config(recording_path).options.camera;
    if (recording_.mode != "disabled") {
        const auto source = std::find_if(recording_.sources.begin(), recording_.sources.end(),
                                         [&](const auto& s) { return s.camera_id == camera_id; });
        require(source != recording_.sources.end(),
                "--camera-id must match recording camera.sources");
        require(!source->record_depth,
                "simulation currently records RGB only; set record_depth: false");
    }
    if (!preview_.enabled && recording_.mode == "disabled")
        return;
    std::promise<void> ready;
    auto started = ready.get_future();
    thread_ = std::thread([this, &ready] { run(ready); });
    try {
        started.get();
    } catch (...) {
        thread_.join();
        throw;
    }
    std::cout << "Camera images: preview=" << (preview_.enabled ? preview_.endpoint : "off")
              << " recording="
              << (recording_.mode != "disabled" ? recording_.record_endpoint : "off")
              << " camera_id=" << camera_id_ << '\n';
}
CameraOutput::~CameraOutput() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        dropped_ += queue_.size();
        queue_.clear();
        latest_.reset();
        queue_bytes_ = 0;
    }
    condition_.notify_all();
    if (thread_.joinable())
        thread_.join();
    if (recording_.mode != "disabled")
        std::cout << "Camera recording: queued-to-zmq=" << sent_ << " dropped=" << dropped_ << '\n';
}
void CameraOutput::submit(const Camera& camera, const aviator::Message& detection) {
    if (!thread_.joinable() || failed_)
        return;
    auto f = std::make_shared<Frame>();
    const auto& h = detection.header;
    const auto& b = detection.body;
    f->rgb = camera.rgb();
    f->metadata = {{"version", 1u},
                   {"camera_id", camera_id_},
                   {"stream", "rgb"},
                   {"publisher_id", h.publisher_id},
                   {"session_id", h.session_id},
                   {"clock_id", h.clock_id},
                   {"config_id", b.at("config_id")},
                   {"frame_id", b.at("frame_id")},
                   {"sequence", b.at("frame_id")},
                   {"sample_mono_us", h.sample_mono_us},
                   {"timestamp_us", h.timestamp},
                   {"sample_time_basis", "host_simulation_snapshot"},
                   {"width", b.at("image_width").get<unsigned>()},
                   {"height", b.at("image_height").get<unsigned>()},
                   {"stride_bytes", b.at("image_width").get<unsigned>() * 3u},
                   {"fps", 30u},
                   {"pixel_format", "RGB8"},
                   {"byte_order", "little"},
                   {"encoding", "raw"},
                   {"calibration", camera.calibration()}};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (preview_.enabled)
            latest_ = f;
        if (recording_.mode != "disabled") {
            const auto bytes = f->rgb.size() + f->metadata.dump().size() + 32;
            if (bytes > recording_.max_record_bytes ||
                queue_bytes_ + bytes > recording_.queue_bytes)
                ++dropped_;
            else {
                queue_bytes_ += bytes;
                queue_.push_back(f);
            }
        }
    }
    condition_.notify_one();
}
void CameraOutput::run(std::promise<void>& ready) {
    bool initialized = false;
    try {
        zmq::context_t context(1);
        zmq::socket_t pub(context, zmq::socket_type::pub), push(context, zmq::socket_type::push);
        pub.set(zmq::sockopt::linger, 0);
        pub.set(zmq::sockopt::sndhwm, 2);
        push.set(zmq::sockopt::linger, 0);
        push.set(zmq::sockopt::sndhwm, recording_.receive_hwm);
        push.set(zmq::sockopt::immediate, 1);
        if (preview_.enabled)
            pub.bind(preview_.endpoint);
        if (recording_.mode != "disabled")
            push.connect(recording_.record_endpoint);
        Jpeg jpeg;
        ready.set_value();
        initialized = true;
        std::uint64_t next_preview = 0;
        while (true) {
            std::shared_ptr<Frame> preview, record;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (stopping_)
                    break;
                const auto now = aviator::monotonic_us();
                if (latest_ && now >= next_preview) {
                    preview.swap(latest_);
                    next_preview = now + 1000000 / preview_.fps;
                }
                if (!queue_.empty()) {
                    record = queue_.front();
                    queue_.pop_front();
                    queue_bytes_ -= record->rgb.size() + record->metadata.dump().size() + 32;
                }
                if (!preview && !record) {
                    condition_.wait_for(
                        lock, std::chrono::microseconds(
                                  latest_ && next_preview > now ? next_preview - now : 1000000));
                    continue;
                }
            }
            if (record) {
                aviator::CameraFrame raw{
                    record->metadata, std::string(reinterpret_cast<const char*>(record->rgb.data()),
                                                  record->rgb.size())};
                aviator::validate_camera_frame(raw, recording_);
                const auto packet = aviator::serialize_camera_frame(raw);
                if (packet.size() <= recording_.max_record_bytes &&
                    push.send(zmq::buffer(packet), zmq::send_flags::dontwait))
                    ++sent_;
                else
                    ++dropped_;
            }
            if (preview) {
                const int w = preview->metadata.at("width"), h = preview->metadata.at("height");
                const auto bytes = jpeg.encode(preview->rgb, w, h, preview_);
                if (bytes.size() > 2 * 1024 * 1024)
                    continue;
                auto meta = preview->metadata;
                // Same three-part protocol as nodes/camera/preview_client.py.
                for (const char* key :
                     {"calibration", "stride_bytes", "pixel_format", "stream", "byte_order"})
                    meta.erase(key);
                meta.update({{"encoding", "jpeg"},
                             {"width", jpeg.width},
                             {"height", jpeg.height},
                             {"original_width", w},
                             {"original_height", h},
                             {"resize_mode", "fit"},
                             {"preview_fps", preview_.fps}});
                const auto topic = "camera.rgb." + camera_id_, metadata = meta.dump();
                pub.send(zmq::buffer(topic), zmq::send_flags::sndmore | zmq::send_flags::dontwait);
                pub.send(zmq::buffer(metadata),
                         zmq::send_flags::sndmore | zmq::send_flags::dontwait);
                pub.send(zmq::buffer(bytes), zmq::send_flags::dontwait);
            }
        }
    } catch (...) {
        failed_ = true;
        if (!initialized)
            ready.set_exception(std::current_exception());
        else {
            try {
                throw;
            } catch (const std::exception& e) {
                std::cerr << "Camera image transmission disabled: " << e.what() << '\n';
            }
        }
    }
}
} // namespace simulation

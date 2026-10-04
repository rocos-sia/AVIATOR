#pragma once
#include "camera.hpp"
#include "camera_recording.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <thread>

namespace simulation {
struct PreviewSettings {
    bool enabled = true;
    std::string endpoint = "tcp://127.0.0.1:5561";
    int width = 640, height = 360, fps = 15, quality = 80;
};
PreviewSettings previewSettings(const std::string& path, const std::string& override_endpoint);

// GL stays on the main thread; encoding and both image sockets belong to this
// worker. Preview retains one latest frame; recording has a bounded FIFO.
class CameraOutput {
  public:
    CameraOutput(PreviewSettings preview, const std::string& recording_path,
                 const std::string& camera_id);
    ~CameraOutput();
    CameraOutput(const CameraOutput&) = delete;
    CameraOutput& operator=(const CameraOutput&) = delete;
    void submit(const Camera& camera, const aviator::Message& detection);

  private:
    struct Frame {
        Json metadata;
        std::vector<unsigned char> rgb;
    };
    void run(std::promise<void>& ready);
    PreviewSettings preview_;
    aviator::CameraRecordingOptions recording_;
    std::string camera_id_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_ = false;
    std::atomic<bool> failed_{false};
    std::shared_ptr<Frame> latest_;
    std::deque<std::shared_ptr<Frame>> queue_;
    std::size_t queue_bytes_ = 0;
    std::atomic<std::uint64_t> sent_{0}, dropped_{0};
};
} // namespace simulation

// camera_pub — capture one camera, publish JPEG frames on a dedicated image
// channel for the foxglove_bridge example.
//
// Why a dedicated channel: the AVIATOR control bus caps every message at
// 64 KiB (common/protocol.hpp max_payload_bytes, enforced via ZMQ maxmsgsize),
// while a color JPEG is typically 30-150 KiB. So the image rides its own PUB
// socket (default tcp://127.0.0.1:5558) with a large maxmsgsize, matching the
// SAD rule that camera frames stay off the control bus (recording 5557 / image
// channel), not the 64 KiB control bus.
//
// Message shape (ZMQ multipart), so a consumer can recover frame number and
// capture time even though ZMQ messages are otherwise untyped:
//   Frame0 = topic    ("camera.image")
//   Frame1 = JPEG bytes (binary)
//   Frame2 = JSON metadata { "frame": N, "timestamp": <us epoch>,
//                            "width": W, "height": H, "format": "jpeg" }
//
// Usage:
//   camera_pub [--camera 0] [--endpoint tcp://127.0.0.1:5558] [--fps 30]
//               [--width 1280] [--height 720] [--quality 80]

#include <zmq.hpp>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

struct Args {
  int camera = 0;
  std::string endpoint = "tcp://127.0.0.1:5558";
  int fps = 30;
  int width = 1280;
  int height = 720;
  int quality = 80;
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
    if (arg == "--camera") a.camera = std::stoi(next("--camera"));
    else if (arg == "--endpoint") a.endpoint = next("--endpoint");
    else if (arg == "--fps") a.fps = std::stoi(next("--fps"));
    else if (arg == "--width") a.width = std::stoi(next("--width"));
    else if (arg == "--height") a.height = std::stoi(next("--height"));
    else if (arg == "--quality") a.quality = std::stoi(next("--quality"));
    else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: camera_pub [--camera N] [--endpoint URL] [--fps N] "
                   "[--width N] [--height N] [--quality N]\n";
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
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  cv::VideoCapture cap(a.camera);
  if (!cap.isOpened()) {
    std::cerr << "failed to open camera index " << a.camera
              << " (try --camera 1/2/...; color nodes on this machine are "
                 "/dev/video4, /dev/video10, /dev/video16)\n";
    return 1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, a.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, a.height);
  cap.set(cv::CAP_PROP_FPS, a.fps);

  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::maxmsgsize,
          static_cast<std::int64_t>(20 * 1024 * 1024));  // 20 MiB headroom
  pub.bind(a.endpoint);

  std::cout << "camera_pub: camera=" << a.camera << " -> PUB " << a.endpoint
            << " @" << a.fps << " fps\n";

  const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, a.quality};
  std::uint64_t frame = 0;
  cv::Mat raw, bgr;
  std::vector<unsigned char> jpeg;

  while (g_running) {
    if (!cap.read(raw)) {
      std::cerr << "read failed; retrying\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Normalize to 3-channel BGR regardless of the source pixel format.
    if (raw.channels() == 4) {
      cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
    } else if (raw.channels() == 1) {
      cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    } else {
      bgr = raw;
    }

    cv::imencode(".jpg", bgr, jpeg, jpeg_params);

    const auto now_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    nlohmann::json meta;
    meta["frame"] = ++frame;
    meta["timestamp"] = now_us;  // microseconds since Unix epoch
    meta["width"] = bgr.cols;
    meta["height"] = bgr.rows;
    meta["format"] = "jpeg";
    const std::string meta_str = meta.dump();

    const std::string jpeg_str(reinterpret_cast<const char*>(jpeg.data()),
                               jpeg.size());

    pub.send(zmq::buffer(std::string("camera.image")), zmq::send_flags::sndmore);
    pub.send(zmq::buffer(jpeg_str), zmq::send_flags::sndmore);
    pub.send(zmq::buffer(meta_str), zmq::send_flags::none);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / a.fps));
  }

  std::cout << "camera_pub: sent " << frame << " frames, exiting\n";
  return 0;
}

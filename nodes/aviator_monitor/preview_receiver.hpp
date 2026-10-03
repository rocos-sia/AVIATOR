#pragma once
#include "preview.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <atomic>
#include <memory>
#include <thread>

namespace monitor {
// Socket use and replacement are serialized. The control-bus receiver is independent.
class PreviewReceiver {
  public:
    explicit PreviewReceiver(Preview& preview) : preview_(preview), context_(1) {}
    std::unique_ptr<zmq::socket_t> prepare(const Json& settings) {
        const auto address = settings.at("endpoint").get<std::string>();
        if (address.empty())
            return {};
        auto socket = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::sub);
        socket->set(zmq::sockopt::rcvhwm, 4);
        socket->set(zmq::sockopt::linger, 0);
        socket->set(zmq::sockopt::maxmsgsize, std::int64_t{2 * 1024 * 1024});
        socket->set(zmq::sockopt::subscribe,
                    "camera.rgb." + settings.at("camera_id").get<std::string>());
        socket->connect(address);
        return socket;
    }
    void apply(Json settings, std::unique_ptr<zmq::socket_t> socket) {
        std::lock_guard<std::mutex> lock(mutex_);
        preview_.configure(std::move(settings));
        socket_.swap(socket);
        parts_.clear();
        oversized_ = false;
    }
    void run(const std::atomic<bool>& stop) {
        while (!stop.load()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                try {
                    receive();
                } catch (const std::exception& e) {
                    preview_.reject(e.what());
                    socket_.reset();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

  private:
    void receive() {
        if (!socket_)
            return;
        const auto until = aviator::monotonic_us() + 5000;
        for (unsigned i = 0; i < 128 && aviator::monotonic_us() < until; ++i) {
            zmq::message_t part;
            if (!socket_->recv(part, zmq::recv_flags::dontwait))
                break;
            const bool more = socket_->get(zmq::sockopt::rcvmore);
            const std::size_t limit =
                parts_.empty() ? 128 : (parts_.size() == 1 ? 8192 : 2 * 1024 * 1024);
            if (!oversized_ && parts_.size() < 3 && part.size() <= limit)
                parts_.emplace_back(static_cast<const char*>(part.data()), part.size());
            else
                oversized_ = true;
            if (!more) {
                if (!oversized_ && parts_.size() == 3)
                    preview_.ingest(parts_[0], parts_[1], std::move(parts_[2]),
                                    aviator::monotonic_us());
                else
                    preview_.reject("preview must contain topic, metadata and JPEG");
                parts_.clear();
                oversized_ = false;
            }
        }
    }
    Preview& preview_;
    zmq::context_t context_;
    std::mutex mutex_;
    std::unique_ptr<zmq::socket_t> socket_;
    std::vector<std::string> parts_;
    bool oversized_ = false;
};
} // namespace monitor

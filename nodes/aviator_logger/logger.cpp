#include "logger.hpp"
#include "runtime.hpp"
#include "transport.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace aviator {
RecorderSummary record_bus(const std::string& endpoint, const std::string& output,
                           const std::string& session, const std::atomic<bool>& stop,
                           const RecorderOptions& options, const std::function<void()>& on_ready) {
    if (options.queue_bytes == 0 || options.receive_hwm <= 0)
        throw std::invalid_argument("recording queue limits must be positive");
    struct Entry {
        WireMessage wire;
        std::uint64_t log_ns;
        std::size_t bytes() const {
            return sizeof(Entry) + wire.topic.size() + wire.payload.size();
        }
    };
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<Entry> queue;
    std::size_t queued_bytes = 0;
    bool done = false, ready = false, receiver_failed = false;
    std::exception_ptr writer_error;
    std::uint64_t rejected = 0, dropped = 0;
    RecorderSummary summary;

    zmq::context_t context{1};
    zmq::socket_t sub(context, zmq::socket_type::sub);
    configure(sub, {8, options.receive_hwm, 0});
    subscribe(sub, "");
    sub.connect(endpoint);

    std::thread writer([&] {
        try {
            RecordingWriter recording(output, session);
            {
                std::lock_guard<std::mutex> lock(mutex);
                ready = true;
            }
            changed.notify_all();
            for (;;) {
                Entry entry;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    changed.wait(lock, [&] { return done || !queue.empty(); });
                    if (queue.empty())
                        break;
                    entry = std::move(queue.front());
                    queued_bytes -= entry.bytes();
                    queue.pop_front();
                }
                recording.append(entry.wire.topic, entry.wire.payload, entry.log_ns);
            }
            if (receiver_failed)
                throw std::runtime_error("receiver failed; recording retained as partial");
            summary = recording.finish(rejected, dropped);
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
        std::string error;
        while (!stop.load()) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (writer_error)
                    std::rethrow_exception(writer_error);
            }
            zmq::pollitem_t poll{sub.handle(), 0, ZMQ_POLLIN, 0};
            zmq::poll(&poll, 1, std::chrono::milliseconds(20));
            // Bounded batch checks stop/writer failures regularly under saturation.
            for (unsigned i = 0; i < 256 && !stop.load(); ++i) {
                Entry entry;
                const auto result = receive(sub, receiving, entry.wire, error);
                if (result == ReceiveResult::empty)
                    break;
                if (result == ReceiveResult::rejected) {
                    ++rejected;
                    continue;
                }
                entry.log_ns = utc_us() * 1000ULL;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (entry.bytes() > options.queue_bytes - queued_bytes) {
                        ++dropped;
                        continue;
                    }
                    queued_bytes += entry.bytes();
                    queue.push_back(std::move(entry));
                }
                changed.notify_one();
            }
        }
    } catch (...) {
        receiver_error = std::current_exception();
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        receiver_failed = static_cast<bool>(receiver_error);
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

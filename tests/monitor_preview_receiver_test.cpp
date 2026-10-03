#include "preview_receiver.hpp"
#include <fstream>
#include <iostream>
using monitor::Json;
void check(bool value, const char* why) {
    if (!value)
        throw std::runtime_error(why);
}
int main(int argc, char** argv) {
    try {
        check(argc == 2, "JPEG fixture required");
        std::ifstream file(argv[1], std::ios::binary);
        const std::string jpeg(std::istreambuf_iterator<char>(file), {});
        zmq::context_t context(1);
        zmq::socket_t first(context, zmq::socket_type::pub), second(context, zmq::socket_type::pub);
        first.set(zmq::sockopt::linger, 0);
        second.set(zmq::sockopt::linger, 0);
        first.bind("tcp://127.0.0.1:*");
        second.bind("tcp://127.0.0.1:*");
        auto settings = monitor::default_config()["preview"];
        settings["endpoint"] = "";
        monitor::Preview preview(settings);
        monitor::PreviewReceiver receiver(preview);
        receiver.apply(settings, receiver.prepare(settings));
        std::atomic<bool> stop{false};
        std::thread thread([&] { receiver.run(stop); });
        struct Join {
            std::atomic<bool>& stop;
            std::thread& thread;
            ~Join() {
                stop = true;
                thread.join();
            }
        } join{stop, thread};
        Json meta{{"version", 1},
                  {"encoding", "jpeg"},
                  {"publisher_id", "camera"},
                  {"camera_id", "cockpit"},
                  {"session_id", "11111111-1111-4111-8111-111111111111"},
                  {"clock_id", "clock"},
                  {"sequence", 1},
                  {"frame_id", 1},
                  {"sample_mono_us", 0},
                  {"width", 16},
                  {"height", 16}};
        auto receive = [&](zmq::socket_t& publisher) {
            const auto deadline = aviator::monotonic_us() + 2000000;
            while (aviator::monotonic_us() < deadline) {
                meta["sample_mono_us"] = aviator::monotonic_us();
                meta["sequence"] = meta["sequence"].get<unsigned>() + 1;
                const auto topic = "camera.rgb." + meta["camera_id"].get<std::string>();
                publisher.send(zmq::buffer(topic), zmq::send_flags::sndmore);
                const auto payload = meta.dump();
                publisher.send(zmq::buffer(payload), zmq::send_flags::sndmore);
                publisher.send(zmq::buffer(jpeg));
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                auto data = preview.latest(aviator::monotonic_us(), "clock");
                if (data["state"] == "FRESH")
                    return data["current"]["token"].get<std::string>();
            }
            throw std::runtime_error("no frame received after preview switch");
        };
        settings["endpoint"] = first.get(zmq::sockopt::last_endpoint);
        receiver.apply(settings, receiver.prepare(settings));
        const auto old_token = receive(first);
        settings["endpoint"] = second.get(zmq::sockopt::last_endpoint);
        settings["camera_id"] = "new_camera";
        settings["publisher_id"] = "new_publisher";
        receiver.apply(settings, receiver.prepare(settings));
        check(preview.frame(old_token).empty(), "old preview cache survived configuration switch");
        check(preview.latest(aviator::monotonic_us(), "clock")["state"] == "WAITING",
              "switch did not clear frame");
        meta["camera_id"] = "new_camera";
        meta["publisher_id"] = "new_publisher";
        check(receive(second) != old_token, "preview token reused");
        settings["endpoint"] = "";
        receiver.apply(settings, receiver.prepare(settings));
        check(preview.latest(aviator::monotonic_us(), "clock")["state"] == "UNCONFIGURED",
              "preview not disabled");
        settings["endpoint"] = first.get(zmq::sockopt::last_endpoint);
        receiver.apply(settings, receiver.prepare(settings));
        receive(first);
        std::cout << "Preview enable, reconnect, identity, disable and re-enable passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

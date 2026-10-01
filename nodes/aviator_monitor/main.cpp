#include "assets.hpp"
#include "monitor.hpp"
#include "page.hpp"
#include "preview.hpp"
#include "runtime.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include <filesystem>

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {
struct Fd {
    int value = -1;
    ~Fd() {
        if (value >= 0)
            close(value);
    }
    void reset() {
        if (value >= 0)
            close(value);
        value = -1;
    }
};
struct Client {
    Fd fd;
    std::string request, response;
    std::size_t sent = 0;
    std::uint64_t deadline = 0;
};
unsigned number(const std::string& text, unsigned maximum) {
    if (text.empty() || text.size() > 10 ||
        text.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("invalid numeric argument");
    const auto value = std::stoul(text);
    if (value == 0 || value > maximum)
        throw std::runtime_error("numeric argument out of range");
    return static_cast<unsigned>(value);
}
void response(Client& client, monitor::State& state, monitor::Preview& preview,
              const monitor::Assets& assets, const std::string& clock) {
    std::istringstream line(client.request.substr(0, client.request.find("\r\n")));
    std::string method, path, version, extra;
    line >> method >> path >> version;
    std::string status = "200 OK", body, type = "application/json; charset=utf-8";
    if ((version != "HTTP/1.1" && version != "HTTP/1.0") || (line >> extra)) {
        status = "400 Bad Request";
        body = "{}";
    } else if (method != "GET") {
        status = "405 Method Not Allowed";
        body = "{}";
    } else if (path == "/" || path == "/index.html") {
        body = monitor_page;
        type = "text/html; charset=utf-8";
    } else if (path == "/api/model-manifest") {
        body = assets.manifest.dump();
    } else if (path == "/api/overview") {
        const auto now = aviator::monotonic_us();
        auto data = state.overview(now, clock);
        data["camera_preview"] = preview.latest(now, clock);
        body = data.dump();
    } else if (path == "/api/camera/latest") {
        body = preview.latest(aviator::monotonic_us(), clock).dump();
    } else if (path.rfind("/api/camera/frame/", 0) == 0) {
        body = preview.frame(path.substr(18));
        type = "image/jpeg";
        if (body.empty()) {
            status = "404 Not Found";
            body = "{}";
            type = "application/json";
        }
    } else if (path.rfind("/assets/", 0) == 0 || path.rfind("/models/", 0) == 0) {
        monitor::Asset asset;
        if (assets.read(path, asset)) {
            body = std::move(asset.body);
            type = std::move(asset.type);
        } else {
            status = "404 Not Found";
            body = "{}";
        }
    } else if (path == "/api/state") {
        body = state.snapshot(aviator::monotonic_us(), clock).dump();
    } else if (path.rfind("/api/message?id=", 0) == 0) {
        try {
            body = state.detail(number(path.substr(16), 0xffffffffU));
        } catch (const std::exception&) {
            status = "400 Bad Request";
            body = "{}";
        }
        if (body.empty()) {
            status = "404 Not Found";
            body = "{}";
        }
    } else {
        status = "404 Not Found";
        body = "{}";
    }
    client.response =
        "HTTP/1.1 " + status + "\r\nContent-Type: " + type +
        "\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n"
        "Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self' "
        "'unsafe-inline'; img-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'\r\n"
        "Connection: close\r\nContent-Length: " +
        std::to_string(body.size()) + "\r\n\r\n" + body;
}
} // namespace
int main(int argc, char** argv) {
    try {
        unsigned port = 8081;
        std::string bind_address = "0.0.0.0";
        std::string endpoint = aviator::subscribe_endpoint, config_path, preview_endpoint;
        namespace fs = std::filesystem;
        const auto deployed =
            fs::weakly_canonical(fs::path("/proc/self/exe")).parent_path().parent_path() /
            AVIATOR_MONITOR_INSTALL_ROOT;
        fs::path web_root =
            fs::exists(deployed / "web/app.js") ? deployed / "web" : fs::path(AVIATOR_MONITOR_WEB);
        fs::path model_root = fs::exists(deployed / "models/urdf/aviator.urdf")
                                  ? deployed / "models"
                                  : fs::path(AVIATOR_MONITOR_MODELS);
        if (fs::exists(deployed / "monitor.json"))
            config_path = (deployed / "monitor.json").string();
        else if (fs::exists(AVIATOR_MONITOR_CONFIG))
            config_path = AVIATOR_MONITOR_CONFIG;
        bool preview_override = false;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                std::cout
                    << "Usage: aviator_monitor [--bind 0.0.0.0] [--port 8081]\n"
                       "  [--subscribe tcp://127.0.0.1:5556]\n"
                       "  [--config monitor.json] [--model-root MODELS] [--preview "
                       "tcp://127.0.0.1:5561|off]\n"
                       "Read-only dual-tab Web UI; HTTP listens on all IPv4 interfaces by "
                       "default.\n"
                       "Open http://<server-LAN-IP>:PORT/ from another computer; use "
                       "--bind 127.0.0.1 for local-only access.\n"
                       "SIGINT/SIGTERM to stop.\n";
                return 0;
            }
            if (++i == argc)
                throw std::runtime_error("missing option value");
            if (key == "--port")
                port = number(argv[i], 65535);
            else if (key == "--bind")
                bind_address = argv[i];
            else if (key == "--subscribe")
                endpoint = argv[i];
            else if (key == "--config")
                config_path = argv[i];
            else if (key == "--model-root")
                model_root = argv[i];
            else if (key == "--preview") {
                preview_endpoint = argv[i];
                preview_override = true;
            } else
                throw std::runtime_error("unknown option: " + key);
        }
        if (endpoint.rfind("tcp://", 0) != 0)
            throw std::runtime_error("subscription must use TCP");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1)
            throw std::runtime_error("--bind must be an IPv4 address: " + bind_address);
        address.sin_port = htons(static_cast<unsigned short>(port));
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0)
            throw std::runtime_error("signal mask failed");
        Fd signal_fd{signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC)};
        Fd server{socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)};
        if (server.value < 0 || signal_fd.value < 0)
            throw std::runtime_error("cannot create server/signal fd");
        int reuse = 1;
        setsockopt(server.value, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        if (bind(server.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
            listen(server.value, 8) < 0)
            throw std::runtime_error(std::strerror(errno));
        monitor::State state;
        state.config = monitor::load_config(config_path);
        if (preview_override)
            state.config["preview"]["endpoint"] = preview_endpoint == "off" ? "" : preview_endpoint;
        monitor::validate_config(state.config);
        monitor::Preview preview(state.config.at("preview"));
        monitor::Assets assets(web_root, model_root, state.config);
        const auto clock = aviator::local_clock_id();
        std::atomic<bool> stop{false};
        std::thread receiver([&] {
            try {
                zmq::context_t context{1};
                zmq::socket_t sub(context, zmq::socket_type::sub);
                aviator::configure(sub, {8, 256, 0});
                aviator::subscribe(sub, "");
                sub.connect(endpoint);
                aviator::ReceiveState receiving;
                while (!stop.load()) {
                    zmq::pollitem_t ready{sub.handle(), 0, ZMQ_POLLIN, 0};
                    zmq::poll(&ready, 1, std::chrono::milliseconds(20));
                    const auto deadline = aviator::monotonic_us() + 5000;
                    for (int i = 0; i < 128 && !stop.load() && aviator::monotonic_us() < deadline;
                         ++i) {
                        aviator::WireMessage wire;
                        std::string error;
                        const auto result = aviator::receive(sub, receiving, wire, error);
                        if (result == aviator::ReceiveResult::empty)
                            break;
                        if (result == aviator::ReceiveResult::rejected)
                            state.reject(error);
                        else
                            state.ingest(wire.topic, wire.payload, aviator::monotonic_us());
                    }
                }
            } catch (const std::exception& error) {
                std::cerr << "monitor receiver: " << error.what() << '\n';
                stop.store(true);
            }
        });
        struct Join {
            std::atomic<bool>& stop;
            std::thread& thread;
            ~Join() {
                stop.store(true);
                thread.join();
            }
        } join{stop, receiver};
        std::thread images([&] {
            const auto address = state.config.at("preview").at("endpoint").get<std::string>();
            if (address.empty())
                return;
            try {
                zmq::context_t context{1};
                zmq::socket_t sub(context, zmq::socket_type::sub);
                sub.set(zmq::sockopt::rcvhwm, 4);
                sub.set(zmq::sockopt::linger, 0);
                sub.set(zmq::sockopt::maxmsgsize, std::int64_t{2 * 1024 * 1024});
                sub.set(zmq::sockopt::subscribe,
                        "camera.rgb." + state.config["preview"]["camera_id"].get<std::string>());
                sub.connect(address);
                std::vector<std::string> parts;
                bool oversized = false;
                while (!stop.load()) {
                    zmq::pollitem_t ready{sub.handle(), 0, ZMQ_POLLIN, 0};
                    zmq::poll(&ready, 1, std::chrono::milliseconds(20));
                    const auto until = aviator::monotonic_us() + 5000;
                    for (unsigned i = 0; i < 128 && !stop.load() && aviator::monotonic_us() < until;
                         ++i) {
                        zmq::message_t part;
                        if (!sub.recv(part, zmq::recv_flags::dontwait))
                            break;
                        const bool more = sub.get(zmq::sockopt::rcvmore);
                        if (parts.size() < 3 && !oversized) {
                            const auto limit = parts.size() == 0
                                                   ? 128
                                                   : (parts.size() == 1 ? 8192 : 2 * 1024 * 1024);
                            if (part.size() > static_cast<std::size_t>(limit))
                                oversized = true;
                            else
                                parts.emplace_back(static_cast<const char*>(part.data()),
                                                   part.size());
                        } else
                            oversized = true;
                        if (!more) {
                            if (!oversized && parts.size() == 3)
                                preview.ingest(parts[0], parts[1], std::move(parts[2]),
                                               aviator::monotonic_us());
                            else
                                preview.reject("preview must contain topic, metadata and JPEG");
                            parts.clear();
                            oversized = false;
                        }
                    }
                }
            } catch (const std::exception& e) {
                preview.reject(e.what());
            }
        });
        Join image_join{stop, images};
        std::array<Client, 8> clients;
        const auto listen_url = "http://" + bind_address + ":" + std::to_string(port) + "/";
        const auto local_url = "http://" +
                               (bind_address == "0.0.0.0" ? "127.0.0.1" : bind_address) + ":" +
                               std::to_string(port) + "/";
        std::cout << "aviator_monitor 已启动，HTTP 监听：" << listen_url << '\n'
                  << "本机浏览器打开：" << local_url << '\n';
        if (bind_address == "0.0.0.0")
            std::cout << "局域网浏览器打开：http://<本机局域网IP>:" << port << "/\n";
        std::cout << "subscribe=" << endpoint << std::endl;
        aviator::print_startup(
            "aviator_monitor",
            {{"HTTP listen", listen_url},
             {"SUB connect", endpoint},
             {"SUB topics", "* (all topics; empty ZMQ subscription filter)"},
             {"PUB topics", "None (read-only monitor)"},
             {"Transport", "Async connect; HTTP availability does not confirm bus traffic."},
             {"Inspect", "/api/overview  |  /api/state  |  dual-tab Web UI"},
             {"Exit", "Ctrl+C"}});
        while (!stop.load()) {
            pollfd ready[]{{server.value, POLLIN, 0}, {signal_fd.value, POLLIN, 0}};
            if (::poll(ready, 2, 10) < 0 && errno != EINTR)
                throw std::runtime_error("HTTP poll failed");
            if (ready[1].revents)
                return 0;
            if (ready[0].revents & POLLIN) {
                const int fd =
                    accept4(server.value, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
                if (fd >= 0) {
                    auto slot = std::find_if(clients.begin(), clients.end(),
                                             [](const Client& c) { return c.fd.value < 0; });
                    if (slot == clients.end())
                        close(fd);
                    else {
                        slot->fd.value = fd;
                        slot->request.clear();
                        slot->response.clear();
                        slot->sent = 0;
                        slot->deadline = aviator::monotonic_us() + 2000000;
                    }
                }
            }
            for (auto& client : clients) {
                if (client.fd.value < 0)
                    continue;
                if (aviator::monotonic_us() >= client.deadline) {
                    client.fd.reset();
                    continue;
                }
                if (client.response.empty()) {
                    char buffer[4096];
                    const auto n = recv(client.fd.value, buffer, sizeof(buffer), MSG_DONTWAIT);
                    if (n == 0 ||
                        (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                        client.fd.reset();
                        continue;
                    }
                    if (n > 0)
                        client.request.append(buffer, static_cast<std::size_t>(n));
                    if (client.request.size() > 4096) {
                        client.fd.reset();
                        continue;
                    }
                    if (client.request.find("\r\n\r\n") != std::string::npos)
                        response(client, state, preview, assets, clock);
                }
                if (!client.response.empty()) {
                    const auto n =
                        send(client.fd.value, client.response.data() + client.sent,
                             std::min<std::size_t>(65536, client.response.size() - client.sent),
                             MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (n > 0)
                        client.sent += static_cast<std::size_t>(n);
                    if ((n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ||
                        client.sent == client.response.size())
                        client.fd.reset();
                }
            }
        }
        return 1; // Receiver failure must not leave a healthy-looking service running.
    } catch (const std::exception& error) {
        std::cerr << "aviator_monitor: " << error.what() << '\n';
        return 1;
    }
}

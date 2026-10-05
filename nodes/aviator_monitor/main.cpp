#include "Logger.hpp"
#include "assets.hpp"
#include "monitor.hpp"
#include "page.hpp"
#include "preview.hpp"
#include "preview_receiver.hpp"
#include "runtime.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include <filesystem>

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <map>
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
    std::map<std::string, std::string> headers;
    std::string parse_error;
    std::size_t body_start = 0;
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
bool request_ready(Client& client) {
    const auto end = client.request.find("\r\n\r\n");
    if (end == std::string::npos) {
        if (client.request.size() > 4096)
            throw std::runtime_error("headers exceed 4 KiB");
        return false;
    }
    if (end + 4 > 4096)
        throw std::runtime_error("headers exceed 4 KiB");
    client.body_start = end + 4;
    client.headers.clear();
    std::istringstream input(client.request.substr(0, end));
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            throw std::runtime_error("malformed header");
        auto key = line.substr(0, colon), value = line.substr(colon + 1);
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        const auto begin = value.find_first_not_of(" \t"), last = value.find_last_not_of(" \t");
        value = begin == std::string::npos ? "" : value.substr(begin, last - begin + 1);
        if (!client.headers.emplace(key, value).second)
            throw std::runtime_error("duplicate header");
    }
    if (client.headers.count("transfer-encoding"))
        throw std::runtime_error("chunked requests unsupported");
    std::size_t size = 0;
    const auto length = client.headers.find("content-length");
    if (length != client.headers.end()) {
        const auto& text = length->second;
        if (text.empty() || text.size() > 6 ||
            text.find_first_not_of("0123456789") != std::string::npos)
            throw std::runtime_error("invalid content length");
        size = std::stoul(text);
    }
    if (size > 131072)
        throw std::runtime_error("body exceeds 128 KiB");
    return client.request.size() >= client.body_start + size;
}
void response(Client& client, monitor::State& state, monitor::Preview& preview,
              const monitor::Assets& assets, const std::string& clock,
              const std::function<monitor::Json(const monitor::Json*)>& configuration) {
    std::istringstream line(client.request.substr(0, client.request.find("\r\n")));
    std::string method, path, version, extra;
    line >> method >> path >> version;
    std::string status = "200 OK", body, type = "application/json; charset=utf-8";
    if (!client.parse_error.empty()) {
        status = "400 Bad Request";
        body = monitor::Json{{"error", client.parse_error}}.dump();
    } else if ((version != "HTTP/1.1" && version != "HTTP/1.0") || (line >> extra)) {
        status = "400 Bad Request";
        body = "{}";
    } else if (path == "/api/config" && (method == "GET" || method == "PUT")) {
        try {
            if (method == "GET")
                body = configuration(nullptr).dump();
            else {
                const auto& headers = client.headers;
                const auto header = [&](const char* key) {
                    auto found = headers.find(key);
                    return found == headers.end() ? std::string() : found->second;
                };
                const auto origin = header("origin");
                if (header("content-type") != "application/json" ||
                    header("x-monitor-config") != "1" ||
                    (!origin.empty() && origin != "http://" + header("host") &&
                     origin != "https://" + header("host"))) {
                    status = "403 Forbidden";
                    body = monitor::Json{{"error", "configuration writes require same-origin JSON "
                                                   "and X-Monitor-Config: 1"}}
                               .dump();
                } else {
                    const auto payload = monitor::Json::parse(
                        client.request.substr(client.body_start),
                        [](int depth, monitor::Json::parse_event_t, monitor::Json&) {
                            if (depth > 32)
                                throw std::runtime_error("configuration nesting limit");
                            return true;
                        });
                    body = configuration(&payload).dump();
                }
            }
        } catch (const std::logic_error& error) {
            status = "409 Conflict";
            body = monitor::Json{{"error", error.what()}}.dump();
        } catch (const std::exception& error) {
            status = "400 Bad Request";
            body = monitor::Json{{"error", error.what()}}.dump();
        }
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
    // Request upload has a short absolute deadline; a response gets an idle timeout.
    // Large meshes must keep streaming for as long as the client makes progress.
    client.deadline = aviator::monotonic_us() + 30000000;
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
        if (fs::exists(deployed / "monitor.yaml"))
            config_path = (deployed / "monitor.yaml").string();
        else if (fs::exists(AVIATOR_MONITOR_CONFIG))
            config_path = AVIATOR_MONITOR_CONFIG;
        bool preview_override = false;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                aviator::Logger::info("Usage: aviator_monitor [--bind 0.0.0.0] [--port 8081]\n"
                    "  [--subscribe tcp://127.0.0.1:5556]\n"
                    "  [--config monitor.yaml] [--model-root MODELS] [--preview tcp://127.0.0.1:5561|off]\n"
                    "Monitoring and live configuration Web UI; HTTP listens on all IPv4 interfaces by default.\n"
                    "Open http://<server-LAN-IP>:PORT/ from another computer; use --bind 127.0.0.1 for "
                    "local-only access.\n"
                    "SIGINT/SIGTERM to stop.");
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
        monitor::PreviewReceiver preview_receiver(preview);
        preview_receiver.apply(state.config.at("preview"),
                               preview_receiver.prepare(state.config.at("preview")));
        const auto configuration = [&](const monitor::Json* request) -> monitor::Json {
            if (request) {
                const auto revision =
                    state.session_id + "/" + std::to_string(state.config_revision);
                if (request->at("revision") != revision)
                    throw std::logic_error("配置已被更新或服务已重启，请重新读取后再保存。");
                if (request->contains("yaml") == request->contains("config"))
                    throw std::runtime_error("provide either yaml or config");
                auto candidate = request->contains("yaml")
                                     ? monitor::parse_config(request->at("yaml").get<std::string>())
                                     : request->at("config");
                monitor::validate_config(candidate);
                // Prepare all fallible resources before persisting or replacing the active
                // configuration.
                monitor::Assets next_assets(web_root, model_root, candidate);
                const bool preview_changed = candidate.at("preview") != state.config.at("preview");
                auto next_preview = candidate.at("preview");
                auto next_socket =
                    preview_changed ? preview_receiver.prepare(next_preview) : nullptr;
                monitor::save_config(config_path, candidate);
                {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.config.swap(candidate);
                    ++state.config_revision;
                }
                if (preview_changed)
                    preview_receiver.apply(std::move(next_preview), std::move(next_socket));
                assets = std::move(next_assets);
            }
            return {{"config", state.config},
                    {"yaml", monitor::config_yaml(state.config)},
                    {"revision", state.session_id + "/" + std::to_string(state.config_revision)},
                    {"path", config_path}};
        };
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
                aviator::Logger::error("monitor receiver: {}", error.what());
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
        std::thread images([&] { preview_receiver.run(stop); });
        Join image_join{stop, images};
        std::array<Client, 8> clients;
        const auto listen_url = "http://" + bind_address + ":" + std::to_string(port) + "/";
        const auto local_url = "http://" +
                               (bind_address == "0.0.0.0" ? "127.0.0.1" : bind_address) + ":" +
                               std::to_string(port) + "/";
        aviator::Logger::info("aviator_monitor 已启动，HTTP 监听：{}\n本机浏览器打开：{}", listen_url, local_url);
        if (bind_address == "0.0.0.0")
            aviator::Logger::info("局域网浏览器打开：http://<本机局域网IP>:{}/", port);
        aviator::Logger::info("subscribe={}", endpoint);
        aviator::print_startup(
            "aviator_monitor",
            {{"HTTP listen", listen_url},
             {"SUB connect", endpoint},
             {"SUB topics", "* (all topics; empty ZMQ subscription filter)"},
             {"PUB topics", "None (read-only monitor)"},
             {"Transport", "Async connect; HTTP availability does not confirm bus traffic."},
             {"Inspect", "/api/overview  |  /api/state  |  Web UI + /api/config"},
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
                        slot->parse_error.clear();
                        slot->headers.clear();
                        slot->body_start = 0;
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
                    bool complete = false;
                    try {
                        complete = request_ready(client);
                    } catch (const std::exception& error) {
                        client.parse_error = error.what();
                        complete = true;
                    }
                    if (complete)
                        response(client, state, preview, assets, clock, configuration);
                }
                if (!client.response.empty()) {
                    const auto n =
                        send(client.fd.value, client.response.data() + client.sent,
                             std::min<std::size_t>(65536, client.response.size() - client.sent),
                             MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (n > 0) {
                        client.sent += static_cast<std::size_t>(n);
                        client.deadline = aviator::monotonic_us() + 30000000;
                    }
                    if ((n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ||
                        client.sent == client.response.size())
                        client.fd.reset();
                }
            }
        }
        return 1; // Receiver failure must not leave a healthy-looking service running.
    } catch (const std::exception& error) {
        aviator::Logger::error("aviator_monitor: {}", error.what());
        return 1;
    }
}

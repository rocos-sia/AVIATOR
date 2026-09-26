#include "monitor.hpp"
#include "page.hpp"
#include "runtime.hpp"
#include "transport.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <arpa/inet.h>
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
    ~Fd() { if (value >= 0) close(value); }
    void reset() { if (value >= 0) close(value); value = -1; }
};
struct Client {
    Fd fd;
    std::string request, response;
    std::size_t sent = 0;
    std::uint64_t deadline = 0;
};
unsigned number(const std::string& text, unsigned maximum) {
    if (text.empty() || text.size() > 10 || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("invalid numeric argument");
    const auto value = std::stoul(text);
    if (value == 0 || value > maximum) throw std::runtime_error("numeric argument out of range");
    return static_cast<unsigned>(value);
}
void response(Client& client, monitor::State& state, const std::string& clock) {
    std::istringstream line(client.request.substr(0, client.request.find("\r\n")));
    std::string method, path, version, extra;
    line >> method >> path >> version;
    std::string status = "200 OK", body, type = "application/json; charset=utf-8";
    if ((version != "HTTP/1.1" && version != "HTTP/1.0") || (line >> extra)) {
        status = "400 Bad Request"; body = "{}";
    } else if (method != "GET") {
        status = "405 Method Not Allowed"; body = "{}";
    } else if (path == "/" || path == "/index.html") {
        body = monitor_page; type = "text/html; charset=utf-8";
    } else if (path == "/api/state") {
        body = state.snapshot(aviator::monotonic_us(), clock).dump();
    } else if (path.rfind("/api/message?id=", 0) == 0) {
        try { body = state.detail(number(path.substr(16), 0xffffffffU)); }
        catch (const std::exception&) { status = "400 Bad Request"; body = "{}"; }
        if (body.empty()) { status = "404 Not Found"; body = "{}"; }
    } else { status = "404 Not Found"; body = "{}"; }
    client.response = "HTTP/1.1 " + status + "\r\nContent-Type: " + type +
        "\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n"
        "Content-Security-Policy: default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; frame-ancestors 'none'\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
}
}
int main(int argc, char** argv) {
    try {
        unsigned port = 8081;
        std::string endpoint = aviator::subscribe_endpoint;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                std::cout << "Usage: aviator_monitor [--port 8081] [--subscribe tcp://127.0.0.1:5556]\n"
                             "Read-only Web UI on http://127.0.0.1:PORT (SIGINT/SIGTERM to stop).\n";
                return 0;
            }
            if (++i == argc) throw std::runtime_error("missing option value");
            if (key == "--port") port = number(argv[i], 65535);
            else if (key == "--subscribe") endpoint = argv[i];
            else throw std::runtime_error("unknown option: " + key);
        }
        if (endpoint.rfind("tcp://", 0) != 0) throw std::runtime_error("subscription must use TCP");
        sigset_t signals;
        sigemptyset(&signals); sigaddset(&signals, SIGINT); sigaddset(&signals, SIGTERM);
        if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0) throw std::runtime_error("signal mask failed");
        Fd signal_fd{signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC)};
        Fd server{socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)};
        if (server.value < 0 || signal_fd.value < 0) throw std::runtime_error("cannot create server/signal fd");
        int reuse = 1;
        setsockopt(server.value, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<unsigned short>(port));
        if (bind(server.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(server.value, 8) < 0)
            throw std::runtime_error(std::strerror(errno));
        monitor::State state;
        const auto clock = aviator::local_clock_id();
        std::atomic<bool> stop{false};
        std::thread receiver([&] {
            try {
                zmq::context_t context{1};
                zmq::socket_t sub(context, zmq::socket_type::sub);
                aviator::configure(sub, {8, 256, 0});
                aviator::subscribe(sub, ""); sub.connect(endpoint);
                aviator::ReceiveState receiving;
                while (!stop.load()) {
                    zmq::pollitem_t ready{sub.handle(), 0, ZMQ_POLLIN, 0};
                    zmq::poll(&ready, 1, std::chrono::milliseconds(20));
                    const auto deadline = aviator::monotonic_us() + 5000;
                    for (int i = 0; i < 128 && !stop.load() && aviator::monotonic_us() < deadline; ++i) {
                        aviator::WireMessage wire; std::string error;
                        const auto result = aviator::receive(sub, receiving, wire, error);
                        if (result == aviator::ReceiveResult::empty) break;
                        if (result == aviator::ReceiveResult::rejected) state.reject(error);
                        else state.ingest(wire.topic, wire.payload, aviator::monotonic_us());
                    }
                }
            } catch (const std::exception& error) {
                std::cerr << "monitor receiver: " << error.what() << '\n';
                stop.store(true);
            }
        });
        struct Join {
            std::atomic<bool>& stop; std::thread& thread;
            ~Join() { stop.store(true); thread.join(); }
        } join{stop, receiver};
        std::array<Client, 8> clients;
        std::cout << "Open http://127.0.0.1:" << port << " subscribe=" << endpoint << std::endl;
        while (!stop.load()) {
            pollfd ready[]{{server.value, POLLIN, 0}, {signal_fd.value, POLLIN, 0}};
            if (::poll(ready, 2, 10) < 0 && errno != EINTR) throw std::runtime_error("HTTP poll failed");
            if (ready[1].revents) return 0;
            if (ready[0].revents & POLLIN) {
                const int fd = accept4(server.value, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
                if (fd >= 0) {
                    auto slot = std::find_if(clients.begin(), clients.end(), [](const Client& c) { return c.fd.value < 0; });
                    if (slot == clients.end()) close(fd);
                    else { slot->fd.value = fd; slot->request.clear(); slot->response.clear(); slot->sent = 0;
                           slot->deadline = aviator::monotonic_us() + 2000000; }
                }
            }
            for (auto& client : clients) {
                if (client.fd.value < 0) continue;
                if (aviator::monotonic_us() >= client.deadline) { client.fd.reset(); continue; }
                if (client.response.empty()) {
                    char buffer[4096];
                    const auto n = recv(client.fd.value, buffer, sizeof(buffer), MSG_DONTWAIT);
                    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                        client.fd.reset(); continue;
                    }
                    if (n > 0) client.request.append(buffer, static_cast<std::size_t>(n));
                    if (client.request.size() > 4096) { client.fd.reset(); continue; }
                    if (client.request.find("\r\n\r\n") != std::string::npos) response(client, state, clock);
                }
                if (!client.response.empty()) {
                    const auto n = send(client.fd.value, client.response.data() + client.sent,
                        std::min<std::size_t>(65536, client.response.size() - client.sent), MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (n > 0) client.sent += static_cast<std::size_t>(n);
                    if ((n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ||
                        client.sent == client.response.size()) client.fd.reset();
                }
            }
        }
        return 1; // Receiver failure must not leave a healthy-looking service running.
    } catch (const std::exception& error) {
        std::cerr << "aviator_monitor: " << error.what() << '\n'; return 1;
    }
}

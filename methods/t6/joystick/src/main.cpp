#include "joystick.hpp"
#include "page.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
std::string quote(const std::string& value) {
    std::string out = "\"";
    constexpr char hex[] = "0123456789abcdef";
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c < 32) {
            out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15];
        } else out += c; // UTF-8 多字节原样保留，不能逐字节转为 \u00XX。
    }
    return out + '"';
}
std::string json(const joystick::State& s) {
    std::ostringstream out;
    out << "{\"connected\":" << (s.connected ? "true" : "false")
        << ",\"rumble_available\":" << (s.rumble_available ? "true" : "false")
        << ",\"rumble_error\":" << quote(s.rumble_error)
        << ",\"name\":" << quote(s.name) << ",\"error\":" << quote(s.error);
    auto array = [&](const char* key, const std::vector<int>& values) {
        out << ",\"" << key << "\":[";
        for (size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << values[i];
        out << ']';
    };
    array("axes", s.axes); array("buttons", s.buttons);
    return out.str() + '}';
}
void serve(int client, joystick::Device& device) {
    const timeval timeout{0, 200000};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::string request;
    char buffer[1024];
    while (request.find("\r\n\r\n") == std::string::npos) {
        const auto n = recv(client, buffer, sizeof(buffer), 0);
        if (n <= 0) return;
        request.append(buffer, n);
        if (request.size() > 8192) return;
    }
    std::istringstream line(request);
    std::string method, path;
    line >> method >> path;
    const bool control = method == "POST" && (path == "/api/rumble/start" || path == "/api/rumble/stop");
    const auto& state = device.poll();
    const bool ok = !control || device.rumble(path == "/api/rumble/start");
    const bool api = path == "/api/state" || control;
    const bool found = control || (method == "GET" && (path == "/api/state" || path == "/" || path == "/index.html"));
    const std::string body = found ? (api ? json(state) : kPage) : "Not found\n";
    const std::string response = std::string("HTTP/1.1 ") + (found ? (ok ? "200 OK" : "503 Service Unavailable") : "404 Not Found") +
        "\r\nContent-Type: " + (api ? "application/json; charset=utf-8" : "text/html; charset=utf-8") +
        "\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " +
        std::to_string(body.size()) + "\r\n\r\n" + body;
    size_t sent = 0;
    while (sent < response.size()) {
        const auto n = send(client, response.data() + sent, response.size() - sent, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        sent += n;
    }
}
}
int main(int argc, char** argv) {
    int port = 8080;
    try {
        if (argc > 3) throw 0;
        if (argc > 2) {
            size_t used = 0;
            port = std::stoi(argv[2], &used);
            if (argv[2][used] != '\0' || port < 1 || port > 65535) throw 0;
        }
    } catch (...) {
        std::cerr << "Usage: joystick_web [/dev/input/js0] [port:1..65535]\n";
        return 1;
    }
    joystick::Device device(argc > 1 ? argv[1] : "/dev/input/js0");
    const int server = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (server < 0) { std::cerr << std::strerror(errno) << '\n'; return 1; }
    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<unsigned short>(port));
    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(server, 8) < 0) {
        std::cerr << std::strerror(errno) << '\n'; close(server); return 1;
    }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    std::cout << "Open http://127.0.0.1:" << port << " (Ctrl+C to stop)" << std::endl;
    while (running) {
        device.poll();
        pollfd ready{server, POLLIN, 0};
        if (::poll(&ready, 1, 10) > 0 && (ready.revents & POLLIN)) {
            const int client = accept4(server, nullptr, nullptr, SOCK_CLOEXEC);
            if (client >= 0) { serve(client, device); close(client); }
        }
    }
    close(server);
    return 0;
}

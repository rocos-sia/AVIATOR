#include "transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <future>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
void usage() {
    std::cout << "Usage: aviator_bus [--input tcp://address:port]\n"
                 "                   [--output tcp://address:port] [--lock-file path]\n"
                 "Defaults: input tcp://127.0.0.1:5555, output tcp://127.0.0.1:5556\n"
                 "SIGINT/SIGTERM stop the proxy. Use a separate lock file for replay.\n";
}
}

int main(int argc, char** argv) {
    int lock_fd = -1;
    int result = 0;
    try {
        std::string input = aviator::publish_endpoint;
        std::string output = aviator::subscribe_endpoint;
        std::string lock_path = "/tmp/aviator_bus-" + std::to_string(getuid()) + ".lock";
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") { usage(); return 0; }
            if (argument != "--input" && argument != "--output" && argument != "--lock-file")
                throw std::runtime_error("unknown argument: " + argument);
            if (++i == argc || std::string(argv[i]).empty())
                throw std::runtime_error("missing value for " + argument);
            if (argument == "--input") input = argv[i];
            else if (argument == "--output") output = argv[i];
            else lock_path = argv[i];
        }
        if (input.rfind("tcp://", 0) != 0 || output.rfind("tcp://", 0) != 0 || input == output)
            throw std::runtime_error("input/output must be distinct TCP endpoints");

        // Block before creating any threads (including libzmq IO threads).
        // No signal handler calls ZMQ or touches C++ objects.
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        const int mask_error = pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        if (mask_error != 0) throw std::runtime_error(std::strerror(mask_error));

        lock_fd = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_fd < 0) throw std::runtime_error("open lock: " + std::string(std::strerror(errno)));
        struct stat info{};
        if (fstat(lock_fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != getuid())
            throw std::runtime_error("lock must be a regular file owned by this user");
        if (flock(lock_fd, LOCK_EX | LOCK_NB) != 0)
            throw std::runtime_error("cannot acquire instance lock: " + std::string(std::strerror(errno)));

        zmq::context_t context{1};
        auto proxy = std::async(std::launch::async, [&] {
            aviator::run_bus(context, input, output, [&] {
                std::cout << "READY input=" << input << " output=" << output << std::endl;
            });
        });
        while (proxy.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            const timespec timeout{0, 100000000};
            const int signal = sigtimedwait(&signals, nullptr, &timeout);
            if (signal == SIGINT || signal == SIGTERM) {
                context.shutdown();
                break;
            }
            if (signal < 0 && errno != EAGAIN && errno != EINTR) {
                const int wait_error = errno;
                context.shutdown();
                proxy.wait();
                throw std::runtime_error("signal wait: " + std::string(std::strerror(wait_error)));
            }
        }
        proxy.get(); // Bind/proxy failure is reported, never mistaken for readiness.
    } catch (const std::exception& error) {
        std::cerr << "aviator_bus: " << error.what() << '\n';
        result = 1;
    }
    // Keep the file: unlinking a flock file can allow two different lock inodes.
    if (lock_fd >= 0) close(lock_fd);
    return result;
}

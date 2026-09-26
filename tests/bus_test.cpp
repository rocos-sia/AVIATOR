#include "transport.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
namespace {
void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

// Test-only child ownership: always reap, including assertion failures.
struct Child {
    pid_t pid = -1;
    int output = -1;
    Child(const char* executable, const std::string& input,
          const std::string& target, const std::string& lock) {
        int pipe_fd[2];
        check(pipe(pipe_fd) == 0, "pipe failed");
        pid = fork();
        if (pid == 0) {
            dup2(pipe_fd[1], STDOUT_FILENO);
            dup2(pipe_fd[1], STDERR_FILENO);
            close(pipe_fd[0]); close(pipe_fd[1]);
            execl(executable, executable, "--input", input.c_str(), "--output",
                  target.c_str(), "--lock-file", lock.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }
        close(pipe_fd[1]);
        output = pipe_fd[0];
        if (pid < 0) { close(output); throw std::runtime_error("fork failed"); }
    }
    ~Child() {
        if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, nullptr, 0); }
        close(output);
    }
    void ready() {
        std::string text;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd fd{output, POLLIN, 0};
            if (poll(&fd, 1, 100) <= 0) continue;
            char buffer[512];
            const auto size = read(output, buffer, sizeof(buffer));
            if (size <= 0) break;
            text.append(buffer, static_cast<std::size_t>(size));
            if (text.find("READY ") != std::string::npos) return;
        }
        throw std::runtime_error("bus not ready: " + text);
    }
    int wait() {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        int status = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            if (waitpid(pid, &status, WNOHANG) == pid) {
                pid = -1;
                check(WIFEXITED(status), "bus exited by signal instead of cleanup");
                return WEXITSTATUS(status);
            }
            std::this_thread::sleep_for(5ms);
        }
        throw std::runtime_error("bus exit timed out");
    }
    void stop(int signal) { check(kill(pid, signal) == 0, "kill failed"); check(wait() == 0, "unclean stop"); }
};

void tests(const char* executable) {
    const auto lock = "/tmp/aviator-bus-test-" + std::to_string(getpid()) + ".lock";
    const auto other_lock = lock + ".other";
    struct Cleanup {
        std::string first, second;
        ~Cleanup() { unlink(first.c_str()); unlink(second.c_str()); }
    } cleanup{lock, other_lock};

    zmq::context_t context{1};
    zmq::socket_t first(context, zmq::socket_type::pub), second(context, zmq::socket_type::pub);
    first.bind("tcp://127.0.0.1:*"); second.bind("tcp://127.0.0.1:*");
    const auto input = first.get(zmq::sockopt::last_endpoint);
    const auto output = second.get(zmq::sockopt::last_endpoint);
    // Keep output occupied: failure of the SECOND bind must exit without READY.
    first.close();
    {
        Child conflict(executable, input, output, lock);
        check(conflict.wait() != 0, "occupied output must fail");
        char buffer[1024];
        const auto count = read(conflict.output, buffer, sizeof(buffer));
        check(count >= 0 && std::string(buffer, static_cast<std::size_t>(count)).find("READY ") == std::string::npos,
              "failed bind must not report READY");
    }
    second.close();
    {
        Child bus(executable, input, output, lock);
        bus.ready(); // Also verifies failed startup released input and lock.
        {
            Child duplicate(executable, input, output, lock);
            check(duplicate.wait() != 0, "duplicate instance must fail");
        }
        zmq::socket_t pub(context, zmq::socket_type::pub), sub(context, zmq::socket_type::sub);
        aviator::configure(pub); aviator::configure(sub);
        aviator::subscribe(sub, "flight.command");
        pub.connect(input); sub.connect(output);
        aviator::ReceiveState state;
        aviator::WireMessage wire;
        std::string error;
        const std::string payload = "{\"opaque\":true}";
        bool received = false;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!received && std::chrono::steady_clock::now() < deadline) {
            aviator::send(pub, "flight.command", payload);
            zmq::pollitem_t item{sub.handle(), 0, ZMQ_POLLIN, 0};
            zmq::poll(&item, 1, 10ms);
            received = aviator::receive(sub, state, wire, error) == aviator::ReceiveResult::received;
        }
        check(received && wire.topic == "flight.command" && wire.payload == payload,
              "process must forward opaque multipart without business parsing");
        bus.stop(SIGTERM);
    }
    {
        Child restarted(executable, input, output, lock);
        restarted.ready();
        restarted.stop(SIGINT);
    }
    {
        Child invalid(executable, "inproc://not-allowed", output, other_lock);
        check(invalid.wait() != 0, "non-TCP endpoint must fail");
    }
}
}
int main(int argc, char** argv) {
    try {
        check(argc == 2, "expected bus executable path");
        tests(argv[1]);
        std::cout << "bus process tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#include "release_identity.hpp"
#include "Logger.hpp"
#include "aviator/version.hpp"
#include "runtime.hpp"
#include <fcntl.h>
#include <filesystem>
#include <poll.h>
#include <signal.h>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

namespace flight_gateway {
ReleaseIdentity releaseIdentity(const Rs422Config& config) {
    const auto exe = std::filesystem::canonical("/proc/self/exe");
    const auto installed = exe.parent_path() / "../share/aviator/scripts/aviator_release.py";
    const auto tool =
        std::filesystem::exists(installed) ? installed.string() : std::string(AVIATOR_RELEASE_TOOL);
    const auto root = config.release_root.empty() ? exe.parent_path().parent_path()
                                                  : std::filesystem::path(config.release_root);
    // A verified identity must cover the running executable, not some other valid installation.
    const auto relative = std::filesystem::relative(exe, root);
    if (relative.empty() || *relative.begin() == "..")
        throw std::runtime_error("release_root must contain the running flight_gateway executable");
    int pipes[2];
    if (pipe2(pipes, O_CLOEXEC | O_NONBLOCK) < 0)
        throw std::runtime_error("identity pipe failed");
    const auto child = fork();
    if (child == 0) {
        sigset_t empty;
        sigemptyset(&empty);
        sigprocmask(SIG_SETMASK, &empty, nullptr);
        dup2(pipes[1], STDOUT_FILENO);
        close(pipes[0]);
        close(pipes[1]);
        if (config.allow_test_identity)
            execlp("python3", "python3", tool.c_str(), "identity", root.c_str(),
                   "--allow-test-fallback", "--version", aviator::release::version, nullptr);
        else
            execlp("python3", "python3", tool.c_str(), "identity", root.c_str(), nullptr);
        _exit(127);
    }
    close(pipes[1]);
    if (child < 0) {
        close(pipes[0]);
        throw std::runtime_error("identity fork failed");
    }
    std::string output;
    int status = 0;
    bool done = false;
    const auto began = aviator::monotonic_us();
    while (aviator::monotonic_us() - began < 30000000) {
        char buffer[4096];
        const auto n = read(pipes[0], buffer, sizeof(buffer));
        if (n > 0)
            output.append(buffer, n);
        if (output.size() > 65536)
            break;
        const auto result = waitpid(child, &status, WNOHANG);
        if (result == child) {
            while (true) {
                const auto m = read(pipes[0], buffer, sizeof(buffer));
                if (m <= 0)
                    break;
                output.append(buffer, m);
            }
            done = true;
            break;
        }
        pollfd fd{pipes[0], POLLIN, 0};
        poll(&fd, 1, 20);
    }
    if (!done) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
    }
    close(pipes[0]);
    if (!done || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("release identity verification failed or timed out");
    const auto value = nlohmann::json::parse(output);
    const auto version = value.at("version_u16").get<unsigned>();
    const auto checksum = value.at("checksum_u16").get<unsigned>();
    const bool bench = value.at("mode") == "bench-test";
    if (value.at("version").get<std::string>() != aviator::release::version ||
        version != aviator::release::version_u16 || checksum > 65535 ||
        (bench ? !config.allow_test_identity : !value.at("verified").get<bool>()))
        throw std::runtime_error("release identity does not match running binary");
    aviator::Logger::info("RS422 identity mode={} version={} checksum={:04X}",
                          bench ? "bench-test" : "release", aviator::release::version, checksum);
    return {uint16_t(version), uint16_t(checksum), bench};
}
} // namespace flight_gateway

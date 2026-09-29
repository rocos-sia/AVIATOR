#include "motion.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace aviator;
namespace fs = std::filesystem;
void check(bool ok, const std::string &why) {
    if (!ok)
        throw std::runtime_error(why);
}
struct Child {
    pid_t pid = -1;
    fs::path log;
    int status = -1, input_fd = -1;
    Child(const std::vector<std::string> &args, const fs::path &path, bool input = false) : log(path) {
        int pipes[2]{-1, -1};
        if (input)
            check(pipe(pipes) == 0, "stdin pipe failed");
        pid = fork();
        check(pid >= 0, "fork failed");
        if (pid == 0) {
            if (input) {
                dup2(pipes[0], STDIN_FILENO);
                close(pipes[0]);
                close(pipes[1]);
            }
            int fd = open(log.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0600);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
            std::vector<char *> argv;
            for (const auto &a : args)
                argv.push_back(const_cast<char *>(a.c_str()));
            argv.push_back(nullptr);
            execv(argv[0], argv.data());
            _exit(127);
        }
        if (input) {
            close(pipes[0]);
            input_fd = pipes[1];
        }
    }
    void command(const std::string &line) {
        check(::write(input_fd, line.data(), line.size()) == ssize_t(line.size()),
              "Cannot send console command");
    }
    bool done() {
        if (status >= 0)
            return true;
        int s;
        auto result = waitpid(pid, &s, WNOHANG);
        if (result == pid) {
            status = WIFEXITED(s) ? WEXITSTATUS(s) : 128 + WTERMSIG(s);
            return true;
        }
        return false;
    }
    void terminate(int signal = SIGTERM) {
        if (!done())
            kill(pid, signal);
    }
    ~Child() {
        if (input_fd >= 0)
            close(input_fd);
        if (!done()) {
            terminate();
            for (int i = 0; i < 100 && !done(); ++i)
                usleep(10000);
            if (!done()) {
                kill(pid, SIGKILL);
                int s;
                waitpid(pid, &s, 0);
            }
        }
    }
    std::string text() const {
        std::ifstream f(log);
        return {(std::istreambuf_iterator<char>(f)), {}};
    }
};
int port() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    check(fd >= 0, "socket");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    check(bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0, "bind");
    socklen_t size = sizeof(a);
    getsockname(fd, reinterpret_cast<sockaddr *>(&a), &size);
    close(fd);
    return ntohs(a.sin_port);
}
void write(const fs::path &p, const YAML::Node &y) {
    std::ofstream f(p);
    f << y;
}
int main(int argc, char **argv) {
    fs::path directory;
    try {
        check(argc == 6, "Expected bus manipulator core source_root mode");
        const fs::path source = argv[4];
        const std::string mode = argv[5];
        char pattern[] = "/tmp/aviator-control-test-XXXXXX";
        directory = mkdtemp(pattern);
        std::cout << "Test logs: " << directory << std::endl;
        const auto endpoint = [](int p) { return "tcp://127.0.0.1:" + std::to_string(p); };
        auto system = YAML::LoadFile((source / "config/system.yaml").string());
        std::string input = endpoint(port()), output = endpoint(port()), service = endpoint(port());
        check(input != output && input != service && output != service, "Port allocation collision");
        system["bus"]["publish"] = input;
        system["bus"]["subscribe"] = output;
        system["manipulator_service"] = service;
        system["robot"] = "robot.yaml";
        auto robot = YAML::LoadFile((source / "config/robot.yaml").string());
        robot["backend"] = "mujoco";
        for (const auto *key : {"model", "urdf", "collision_urdf", "grasp", "posture"})
            robot[key] = fs::weakly_canonical(source / "config" / robot[key].as<std::string>()).string();
        // The regression uses explicit faster simulation limits, never edits deployment files.
        robot["wheel_angular_speed"] = .4;
        robot["wheel_linear_speed"] = .04;
        robot["settle_duration"] = .15;
        write(directory / "system.yaml", system);
        write(directory / "robot.yaml", robot);
        Child bus(
            {argv[1], "--input", input, "--output", output, "--lock-file", (directory / "bus.lock").string()},
            directory / "bus.log");
        Child manipulator({argv[2], "--config", (directory / "system.yaml").string(), "--headless"},
                          directory / "manipulator.log");
        const auto deadline = monotonic_us() + 15000000;
        while (manipulator.text().find("READY manipulator") == std::string::npos) {
            check(!manipulator.done(), manipulator.text());
            check(monotonic_us() < deadline, "Manipulator startup timeout");
            usleep(10000);
        }
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        configure(sub);
        subscribe(sub, "arm.state");
        subscribe(sub, "arm.command");
        sub.connect(output);
        if (mode == "service") {
            const auto config = loadMotionConfig(directory / "system.yaml");
            const auto client = new_session_id();
            auto info = callService(ctx, config, serviceRequest(client, "describe", Json::object()));
            const Json parameters = {{"server_session", info.at("server_session")},
                                     {"config_id", config.config_id}};
            auto request = serviceRequest(client, "authorize", parameters);
            auto first = callService(ctx, config, request), second = callService(ctx, config, request);
            check(first == second, "Duplicate authorize generated a different epoch");
            auto query = parameters;
            query["original_request_id"] = request.at("request_id");
            query["original_client_session_id"] = client;
            auto result = callService(ctx, config, serviceRequest(client, "get_result", query));
            check(result.at("status") == "COMPLETED" && result.at("result") == first,
                  "Result query lost original outcome");
            auto conflict = request;
            conflict["operation"] = "enable";
            bool rejected = false;
            try {
                callService(ctx, config, conflict);
            } catch (const std::exception &) {
                rejected = true;
            }
            check(rejected, "Conflicting request identity accepted");
            zmq::socket_t malformed(ctx, zmq::socket_type::req);
            configure(malformed);
            malformed.set(zmq::sockopt::rcvtimeo, 1000);
            malformed.connect(service);
            const std::string invalid = "{\"x\":1,\"x\":2}";
            malformed.send(zmq::buffer(invalid));
            zmq::message_t reply;
            check(bool(malformed.recv(reply)), "Malformed request killed service");
            check(parseService(reply.to_string()).at("status") == "REJECTED",
                  "Duplicate service keys accepted");
            manipulator.terminate();
            while (!manipulator.done())
                usleep(10000);
            Child restarted({argv[2], "--config", (directory / "system.yaml").string(), "--headless"},
                            directory / "restarted.log");
            auto new_info = callService(ctx, config, serviceRequest(client, "describe", Json::object()));
            check(new_info.at("server_session") != info.at("server_session"),
                  "Restart reused server session");
            rejected = false;
            try {
                callService(ctx, config, serviceRequest(client, "enable", parameters));
            } catch (const std::exception &) {
                rejected = true;
            }
            check(rejected, "Old session enabled restarted device");
            std::cout
                << "PASS service: deduplication, conflicting identities, malformed input, restart fencing\n";
            return 0;
        }
        std::vector<std::string> core_args{argv[3], "--config", (directory / "system.yaml").string()};
        if (mode != "interactive")
            core_args.push_back(mode == "demo" ? "--demo" : "--servo-demo");
        Child core(core_args, directory / "core.log", mode == "interactive");
        int console_stage = 0;
        uint64_t servo_at = 0;
        if (mode == "interactive")
            core.command("enable\n");
        ReceiveState receiver;
        size_t states = 0, commands = 0;
        bool enabled = false, locked = false, killed = false, fault = false;
        double max_angle = -1, min_angle = 1, min_displacement = 1, max_error = 0;
        uint64_t killed_at = 0;
        auto until = monotonic_us() + 180000000;
        while (monotonic_us() < until) {
            WireMessage wire;
            std::string error;
            while (receive(sub, receiver, wire, error) == ReceiveResult::received) {
                Message m;
                check(decode(wire.topic, wire.payload, m, error), error);
                if (m.topic == Topic::arm_command) {
                    ++commands;
                    continue;
                }
                ++states;
                const auto &body = m.body;
                enabled = enabled || body["arms"]["left"]["enabled"].get<bool>();
                locked = locked || body["software_lock"].get<bool>();
                fault = body["execution"]["fault"].get<bool>();
                const auto q = body["execution"]["target"].get<Joints>();
                for (double x : q)
                    check(std::isfinite(x), "Nonfinite target");
                check(q[1] >= 85 * M_PI / 180 - 1e-6 && q[1] <= 94 * M_PI / 180 + 1e-6 &&
                          q[8] >= 85 * M_PI / 180 - 1e-6 && q[8] <= 94 * M_PI / 180 + 1e-6,
                      "J2 target outside configured limits");
                if (body.contains("wheel_measurement") && body["software_lock"].get<bool>()) {
                    const double a = body["wheel_measurement"]["angle"],
                                 d = body["wheel_measurement"]["displacement"];
                    max_angle = std::max(max_angle, a);
                    min_angle = std::min(min_angle, a);
                    min_displacement = std::min(min_displacement, d);
                    max_error =
                        std::max(max_error, std::abs(a - body["wheel_reference"]["angle"].get<double>()));
                }
                if (mode == "watchdog" && enabled && !killed && commands > 10) {
                    core.terminate(SIGKILL);
                    killed = true;
                    killed_at = monotonic_us();
                }
                if (mode == "watchdog" && killed && fault) {
                    check(monotonic_us() - killed_at < 1500000, "Watchdog did not stop within test budget");
                    check(!body["arms"]["left"]["enabled"].get<bool>() &&
                              !body["arms"]["right"]["enabled"].get<bool>(),
                          "Fault did not disable both arms");
                    std::cout << "Core loss: fault latched and both arms disabled\n";
                    return 0;
                }
                if (mode != "watchdog")
                    check(!fault, body["execution"]["error"].get<std::string>());
            }
            if (mode == "interactive") {
                const auto text = core.text();
                if (console_stage == 0 && text.find("[ok] ENABLED") != std::string::npos) {
                    core.command("approach\n");
                    ++console_stage;
                } else if (console_stage == 1 && text.find("[ok] APPROACHED") != std::string::npos) {
                    core.command("lock\n");
                    ++console_stage;
                } else if (console_stage == 2 && text.find("[ok] LOCKED") != std::string::npos) {
                    core.command("servo 0.02 -0.002 0.5\n");
                    servo_at = monotonic_us();
                    ++console_stage;
                } else if (console_stage == 3 && monotonic_us() - servo_at > 800000) {
                    core.command("status\n");
                    ++console_stage;
                } else if (console_stage == 4 && text.find("Servo command timeout") != std::string::npos) {
                    core.command("unlock\n");
                    ++console_stage;
                } else if (console_stage == 5 && text.find("[ok] ENABLED") != text.rfind("[ok] ENABLED")) {
                    core.command("disable\n");
                    ++console_stage;
                } else if (console_stage == 6 && text.find("[ok] DISABLED") != std::string::npos) {
                    core.command("quit\n");
                    ++console_stage;
                }
            }
            if (mode != "watchdog" && core.done())
                break;
            check(!manipulator.done(), manipulator.text());
            check(!bus.done(), bus.text());
            usleep(1000);
        }
        check(mode != "watchdog", "Watchdog test timed out");
        check(core.done() && core.status == 0, core.text() + "\n" + manipulator.text());
        if (mode == "interactive")
            check(console_stage == 7, "Interactive sequence incomplete");
        else
            check(core.text().find("Demo completed") != std::string::npos, "Demo did not complete");
        check(enabled && locked && states > 100 && commands > 100, "Missing lifecycle/traffic");
        if (mode == "demo")
            check(max_angle > .84 && min_angle < -.84 && min_displacement < -.16,
                  "Physical wheel did not follow full demo");
        else
            check(max_angle > .001, "Physical wheel did not move in Servo mode");
        check(max_error < .08, "Unexpected simulation wheel tracking error (test assertion only)");
        std::cout << "PASS " << mode << " states=" << states << " commands=" << commands
                  << " measured angle range=" << min_angle << "," << max_angle
                  << " min displacement=" << min_displacement << " max wheel error=" << max_error << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << "\nLogs: " << directory << '\n';
        return 1;
    }
}

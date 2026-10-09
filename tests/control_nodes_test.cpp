#include "motion.hpp"
#include "gateway.hpp"
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
void servoWindowTest(zmq::context_t& ctx, zmq::socket_t& sub, const MotionConfig& config) {
    check(config.timeout_us == 50000, "Regression requires unchanged 50 ms watchdog");
    const auto client = new_session_id();
    const auto info = callService(ctx, config, serviceRequest(client, "describe", Json::object()));
    const Json params = {{"server_session", info.at("server_session")}, {"config_id", config.config_id}};
    const std::string epoch = callService(ctx, config, serviceRequest(client, "authorize", params)).at("control_epoch");
    zmq::socket_t pub(ctx, zmq::socket_type::pub);
    configure(pub);
    pub.connect(config.publish);
    usleep(200000); // Establish PUB/SUB before enable starts its first-command deadline.
    auto initial = callService(ctx, config, serviceRequest(client, "enable", params)).at("target").get<Joints>();
    ReceiveState receiver;
    uint64_t cursor = 0, sequence = 0, last_sample = 0, accepted = 0, trajectory_id = 1, total = 10000;
    Json state;
    auto read = [&](bool allow_fault = false) {
        WireMessage wire;
        std::string error;
        while (receive(sub, receiver, wire, error) == ReceiveResult::received) {
            if (wire.topic != "arm.state") continue;
            Message m;
            check(decode(wire.topic, wire.payload, m, error), error);
            if (m.topic != Topic::arm_state) continue;
            state = m.body;
            cursor = state["execution"]["tick"];
            if (!state["accepted_command"].is_null()) accepted = state["accepted_command"]["sequence"];
            if (!allow_fault) check(!state["execution"]["fault"].get<bool>(), state["execution"]["error"]);
        }
    };
    auto publish = [&] {
        TrajectoryWindow w;
        w.streaming = true;
        w.id = trajectory_id;
        // Match Core's four history ticks; cursor also carries real feedback/bus delay.
        w.first = cursor > 4 ? cursor - 4 : 0;
        w.count = servo_window_points;
        w.total = total;
        w.sequence = ++sequence;
        w.sample = w.origin_sample = w.start = monotonic_us();
        last_sample = w.sample;
        for (size_t k = 0; k < w.count; ++k)
            for (size_t j = 0; j < 14; ++j) {
                const double t = (w.first + k) * .001, jerk = .01234567891234567;
                w.frames[k].q[j] = initial[j] + jerk * t * t * t / 6;
                w.frames[k].dq[j] = jerk * t * t / 2;
                w.frames[k].ddq[j] = jerk * t;
            }
        auto m = motionMessage(Topic::arm_command, "aviator_core", client, sequence);
        m.header.sample_mono_us = w.sample;
        m.body = encodeWindow(w, client, epoch);
        m.body["config_id"] = config.config_id;
        publishMessage(pub, m); // Full precision q/dq/ddq travels through the actual bus.
    };
    for (int i = 0; i < 20; ++i) {
        read(); publish(); usleep(10000);
    }
    read();
    check(cursor > 50, "Servo did not advance");
    const auto before = cursor;
    publish();
    while (monotonic_us() - last_sample < 42000) { read(); usleep(500); }
    read();
    check(accepted == sequence && cursor > before + 25, "Gap did not exercise continued buffered execution");
    publish();
    for (int i = 0; i < 10; ++i) { usleep(10000); read(); publish(); }
    check(cursor > before + 100, "Servo failed to resume continuously after 42 ms gap");
    // Advertise long horizons through the real bus/decoder/executor and verify
    // they do not interrupt accepted commands. Large rolling first_tick values
    // are covered by motion_protocol; this is not a wall-clock soak test.
    for (uint64_t horizon : {3600001ULL, 86400001ULL, 172800001ULL}) {
        total = horizon;
        const auto previous_cursor = cursor, previous_sequence = sequence;
        for (int i = 0; i < 10; ++i) { read(); publish(); usleep(10000); }
        read();
        check(accepted > previous_sequence && cursor > previous_cursor + 50,
              "Long-running Servo horizon stopped accepted commands/execution");
    }
    initial = callService(ctx, config, serviceRequest(client, "stop", params)).at("target").get<Joints>();
    const auto stopped_at = monotonic_us();
    while (monotonic_us() - stopped_at < 1200000) {
        read();
        if (monotonic_us() - stopped_at > 100000) {
            check(state["execution"]["trajectory_id"] == 0 && !state["execution"]["stopping"].get<bool>(),
                  "Explicit stop did not retire the trajectory");
            check(state["execution"]["target"].get<Joints>() == initial, "Stopped hold target changed");
            check(state["arms"]["left"]["enabled"].get<bool>() && state["arms"]["right"]["enabled"].get<bool>(),
                  "Explicit stop unexpectedly disabled arms");
        }
        usleep(1000);
    }
    ++trajectory_id;
    for (int i = 0; i < 10; ++i) { read(); publish(); usleep(10000); }
    read();
    check(cursor > 50, "New trajectory did not restart from local hold");
    publish();
    const auto final_sequence = sequence;
    const auto until = last_sample + 1000000;
    while (monotonic_us() < until) {
        read(true);
        if (!state.is_null() && state["execution"]["fault"].get<bool>()) {
            const std::string error = state["execution"]["error"];
            check(accepted == final_sequence, "Last window did not reach executor");
            check(error.find("Local command watchdog expired") != std::string::npos, error);
            const auto age_at = error.find("sample_age_us=");
            check(age_at != std::string::npos, "Missing watchdog age");
            const auto age = std::stoull(error.substr(age_at + 14));
            check(age >= 50000 && age < 70000, "Watchdog trigger moved outside 50..70 ms");
            if (state["arms"]["left"]["enabled"].get<bool>() ||
                state["arms"]["right"]["enabled"].get<bool>()) continue;
            std::cout << "PASS Servo window: 81 points via bus, 42 ms gap/resume, 1/24/48-hour horizons, explicit stop/hold/restart, watchdog age=" << age << " us\n";
            return;
        }
        usleep(1000);
    }
    throw std::runtime_error("Persistent stream loss did not trip watchdog");
}
int main(int argc, char **argv) {
    fs::path directory;
    try {
        check(argc == 6, "Expected bus manipulator core source_root mode");
        const fs::path source = argv[4];
        const std::string mode = argv[5];
        const bool position_hold = mode == "flight_hold";
        const bool console = mode == "interactive" || mode == "grasp";
        const bool flight = mode == "flight" || mode == "flight_auto" || position_hold;
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
        for (const auto *key : {"model", "urdf", "grasp", "posture"})
            robot[key] = fs::weakly_canonical(source / "config" / robot[key].as<std::string>()).string();
        const auto posture = YAML::LoadFile(robot["posture"].as<std::string>());
        const auto j2 = posture["joint2_limits_deg"].as<std::vector<double>>();
        // The regression uses explicit faster simulation limits, never edits deployment files.
        robot["wheel_angular_speed"] = .4;
        robot["wheel_linear_speed"] = .04;
        robot["settle_duration"] = .15;
        write(directory / "system.yaml", system);
        write(directory / "robot.yaml", robot);
        Child bus(
            {argv[1], "--input", input, "--output", output, "--lock-file", (directory / "bus.lock").string()},
            directory / "bus.log");
        Child manipulator({argv[2], "--config", (directory / "system.yaml").string(), "--headless", "--no-camera"},
                          directory / "manipulator.log");
        const auto deadline = monotonic_us() + 15000000;
        while (manipulator.text().find("READY simulation") == std::string::npos) {
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
        if (mode == "servo_window") {
            servoWindowTest(ctx, sub, loadMotionConfig(directory / "system.yaml"));
            return 0;
        }
        if (mode == "service") {
            const auto config = loadMotionConfig(directory / "system.yaml");
            const auto client = new_instance_id();
            auto info = callService(ctx, config, serviceRequest(client, "describe", Json::object()));
            const Json parameters = {{"server_session", info.at("server_session")},
                                     {"config_id", config.config_id}};
            auto request = serviceRequest(client, "authorize", parameters);
            auto first = callService(ctx, config, request), second = callService(ctx, config, request);
            check(first == second, "Duplicate authorize generated a different epoch");
            auto relabeled = request;
            relabeled["client_session_id"] = "restarted-core";
            relabeled["parameters"].erase("server_session");
            check(callService(ctx, config, relabeled) == first, "Session label broke request deduplication");
            auto query = parameters;
            query["original_request_id"] = request.at("request_id");
            query["original_client_session_id"] = "ignored-origin-marker";
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
            Child restarted({argv[2], "--config", (directory / "system.yaml").string(), "--headless", "--no-camera"},
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
            check(rejected, "Device enabled without a new control authorization");
            std::cout
                << "PASS service: deduplication, conflicting identities, malformed input, restart fencing\n";
            return 0;
        }
        const auto gateway_session = new_session_id(), wrong_session = new_session_id();
        zmq::socket_t flight_pub(ctx, zmq::socket_type::pub);
        configure(flight_pub);
        if (flight) flight_pub.connect(input);
        std::vector<std::string> core_args{argv[3], "--config", (directory / "system.yaml").string()};
        if (mode == "flight") {
            core_args.push_back("--gateway-session");
            core_args.push_back(gateway_session);
        } else if (!flight && !console)
            core_args.push_back(mode == "demo" ? "--demo" : "--servo-demo");
        Child core(core_args, directory / "core.log", console);
        int console_stage = 0;
        uint64_t servo_at = 0;
        if (console)
            core.command("enable\n");
        ReceiveState receiver;
        size_t states = 0, commands = 0;
        bool enabled = false, locked = false, killed = false, fault = false;
        double max_angle = -1, min_angle = 1, min_displacement = 1, max_error = 0;
        uint64_t killed_at = 0;
        int flight_stage = 0;
        uint64_t flight_at = 0, flight_publish_at = 0, flight_seq = 0;
        double reference_angle = 0, reference_displacement = 0, held_angle = 0, held_displacement = 0;
        bool hold_sampled = false;
        uint64_t hold_at = 0;
        flight_gateway::JoystickSample joystick{{ABS_X, -1000, 1000, 0}, {ABS_Y, -1000, 1000, 0}};
        int last_input_stage = 0;
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
                reference_angle = body["wheel_reference"]["angle"];
                reference_displacement = body["wheel_reference"]["displacement"];
                enabled = enabled || body["arms"]["left"]["enabled"].get<bool>();
                locked = locked || body["software_lock"].get<bool>();
                fault = body["execution"]["fault"].get<bool>();
                const auto q = body["execution"]["target"].get<Joints>();
                for (double x : q)
                    check(std::isfinite(x), "Nonfinite target");
                check(q[1] >= j2[0] * M_PI / 180 - 1e-6 && q[1] <= j2[1] * M_PI / 180 + 1e-6 &&
                          q[8] >= j2[0] * M_PI / 180 - 1e-6 && q[8] <= j2[1] * M_PI / 180 + 1e-6,
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
                    // Fault is visible during braking, before disable completes.
                    if (!body["arms"]["left"]["enabled"].get<bool>() &&
                        !body["arms"]["right"]["enabled"].get<bool>()) {
                        std::cout << "Core loss: fault latched and both arms disabled\n";
                        return 0;
                    }
                }
                if (mode != "watchdog")
                    check(!fault, body["execution"]["error"].get<std::string>());
            }
            if (flight) {
                const auto now = monotonic_us();
                const auto text = core.text();
                if (flight_stage == 0 && text.find("Waiting for flight.command") != std::string::npos) {
                    flight_stage = 1;
                    flight_at = now;
                }
                const int last_stage = position_hold ? 9 : 6;
                if (flight_stage >= 1 && flight_stage <= last_stage &&
                    !(position_hold && flight_stage == 7) && now >= flight_publish_at) {
                    // Match flight_gateway's envelope and preserve the original sample time.
                    auto message = motionMessage(Topic::flight_command, "flight_gateway",
                        flight_stage == 1 ? wrong_session : gateway_session, ++flight_seq);
                    const double roll = flight_stage == 2 ? (position_hold ? .5 : .03) :
                                        flight_stage == 3 ? (position_hold ? -.2 : -.03) :
                                        flight_stage >= 8 ? .1 :
                                        flight_stage == 6 ? -1 : 1;
                    const double pitch = flight_stage == 3 ? (position_hold ? -.1 : -.02) :
                                         flight_stage == 6 ? -1 : 1;
                    message.body = {{"source", "JOYSTICK"}, {"control", {{"roll", roll}, {"pitch", pitch}}}};
                    if (flight_stage == 1) {
                        // Invalid input must not activate control, regardless of startup marker.
                        switch (flight_seq % 5) {
                        case 0: message.header.valid = false; break;
                        case 1: message.header.sample_mono_us -= 1000000; break;
                        case 2: message.header.publisher_id = "unrelated_node"; break;
                        case 3: message.header.clock_id = "different_clock"; break;
                        case 4: message.body["source"] = "FLIGHT"; break;
                        }
                    }
                    if (flight_stage == 4) message.header.sample_mono_us = flight_at;
                    if (flight_stage == 4 && flight_seq % 2 == 0) {
                        // A different startup marker must not bypass sample freshness.
                        message.header.session_id = wrong_session;
                        message.header.sample_mono_us = flight_at;
                    }
                    if (position_hold && flight_stage >= 2) {
                        if (last_input_stage != flight_stage && flight_stage != 9) {
                            // One hardware report per target change, then no new axis events for seconds.
                            for (const auto axis : {ABS_X, ABS_Y}) {
                                input_event event{};
                                event.type = EV_ABS; event.code = axis;
                                event.value = int(std::lround((axis == ABS_X ? roll : pitch) * 1000));
                                joystick.update(event, now);
                            }
                            input_event report{};
                            report.type = EV_SYN; report.code = SYN_REPORT;
                            report.input_event_sec = now / 1000000;
                            report.input_event_usec = now % 1000000;
                            joystick.update(report, now);
                        }
                        // Stage 4 keeps transmitting but its device checks stop. Stage 9 disconnects.
                        if (flight_stage == 9) joystick.invalidate();
                        else if (flight_stage != 4 || last_input_stage != 4) joystick.deviceChecked(now);
                        message = flight_gateway::command(joystick, gateway_session, local_clock_id(),
                                                           flight_seq, now, utc_us(), 100000);
                        if (flight_stage == 4 && flight_seq % 2 == 0) {
                            message.header.session_id = wrong_session;
                            message.header.valid = true;
                        }
                        last_input_stage = flight_stage;
                    }
                    publishMessage(flight_pub, message);
                    flight_publish_at = now + 20000;
                }
                // Wait for the completed jerk-limited stop, then observe a stable reference.
                // 10 s bounds v/a + a/j plus input timeout/buffer at this test configuration.
                const uint64_t duration[] = {0, 400000, position_hold ? 20000000ULL : 10000000ULL,
                    position_hold ? 30000000ULL : 10000000ULL, 10000000, 150000, 150000, 10000000, 1000000, 10000000};
                const bool stopping_stage = flight_stage == 4 || (position_hold && (flight_stage == 7 || flight_stage == 9));
                const auto locked_at = text.rfind("state=LOCKED"), servo_state_at = text.rfind("state=SERVO");
                const bool stop_completed = locked_at != std::string::npos && servo_state_at != std::string::npos &&
                                            locked_at > servo_state_at;
                if (stopping_stage && stop_completed && !hold_at) hold_at = now;
                if (stopping_stage && hold_at && now - hold_at > 100000 && !hold_sampled) {
                    held_angle = reference_angle;
                    held_displacement = reference_displacement;
                    hold_sampled = true;
                }
                // Wait for actual execution progress, allowing joint limits and ZMQ segment overhead.
                const bool step_reached = position_hold && now - flight_at >= 3000000 &&
                    ((flight_stage == 2 && std::abs(reference_angle - .43633) < .003 && max_angle > .42 &&
                      std::abs(reference_displacement) < 1e-8) ||
                     (flight_stage == 3 && std::abs(reference_angle + .174532) < .003 &&
                      std::abs(reference_displacement + .0935) < .0005 && min_angle < -.16));
                if (flight_stage >= 1 && flight_stage <= last_stage &&
                    (step_reached || (stopping_stage && hold_sampled && now - hold_at > 400000) ||
                     now - flight_at >= duration[flight_stage])) {
                    if (flight_stage == 1)
                        check(text.find("Servo target") == std::string::npos, "Invalid gateway input moved robot");
                    if (flight_stage == 2) {
                        if (mode == "flight_auto" || position_hold)
                            check(text.find("Received valid flight_gateway input") != std::string::npos,
                                  "Valid gateway input was not received");
                        check(max_angle > .01, "Positive roll did not move physical wheel");
                        check(std::abs(reference_displacement) < 1e-8, "Positive pitch produced pull target");
                        if (position_hold) {
                            check(std::abs(reference_angle - .43633) < .003 && max_angle > .42,
                                  "Static positive step did not reach its target: reference=" + std::to_string(reference_angle));
                            check(text.find("Servo command timeout") == std::string::npos,
                                  "Static healthy joystick timed out");
                        }
                    }
                    if (flight_stage == 3)
                        check(std::abs(reference_displacement - (position_hold ? -.0935 : -.0867)) < .0005,
                              "Negative pitch did not reach mapped absolute displacement");
                    if (flight_stage == 3 && position_hold)
                        check(std::abs(reference_angle + .174532) < .003 &&
                              std::abs(reference_displacement + .0935) < .0005 && min_angle < -.16,
                              "Static negative step did not reach its target: reference=" + std::to_string(reference_angle) +
                                  " displacement=" + std::to_string(reference_displacement) + " measured_min=" + std::to_string(min_angle));
                    if (stopping_stage) {
                        check(text.find("Servo command timeout") != std::string::npos,
                              "Replayed old samples kept Servo alive");
                        check(hold_sampled && std::abs(reference_angle - held_angle) < 1e-6 &&
                              std::abs(reference_displacement - held_displacement) < 1e-6,
                              "Servo did not hold after stale input");
                    }
                    if (flight_stage == 6) {
                        check(text.find("Servo target angle=0.87266 displacement=0 v=1") != std::string::npos &&
                              text.find("Servo target angle=-0.87266 displacement=-0.17 v=1") != std::string::npos,
                              "Full-scale mapping or fresh-input resume failed");
                    }
                    if (flight_stage == last_stage) core.terminate(SIGINT);
                    ++flight_stage;
                    flight_at = now;
                    hold_sampled = false;
                    hold_at = 0;
                }
            }
            if (console) {
                const auto text = core.text();
                if (console_stage == 0 && text.find("[ok] ENABLED") != std::string::npos) {
                    core.command("approach\n");
                    ++console_stage;
                } else if (console_stage == 1 && text.find("[ok] APPROACHED") != std::string::npos) {
                    core.command("lock\n");
                    ++console_stage;
                } else if (console_stage == 2 && text.find("[ok] LOCKED") != std::string::npos && mode == "grasp") {
                    core.command("unlock\n");
                    console_stage = 5;
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
        if (flight)
            check(flight_stage == (position_hold ? 10 : 7), "Flight control sequence incomplete");
        else if (console)
            check(console_stage == 7, "Interactive sequence incomplete");
        else
            check(core.text().find("Demo completed") != std::string::npos, "Demo did not complete");
        check(enabled && locked && states > 100 && commands > 100, "Missing lifecycle/traffic");
        if (mode == "demo")
            check(max_angle > .84 && min_angle < -.84 && min_displacement < -.16,
                  "Physical wheel did not follow full demo");
        else if (mode != "grasp")
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

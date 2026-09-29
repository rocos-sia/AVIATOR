#include "aviator/backend.hpp"
#include "motion.hpp"
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <map>
#include <thread>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#ifdef AVIATOR_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif
#ifdef AVIATOR_HAVE_GLFW
#include "Viewer.hpp"
#endif
using namespace aviator;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
void require(bool ok, const std::string &why) {
    if (!ok)
        throw std::runtime_error(why);
}
pinocchio::SE3 frame(const YAML::Node &n) {
    auto p = n["position"].as<std::vector<double>>(), q = n["quaternion"].as<std::vector<double>>();
    require(p.size() == 3 && q.size() == 4, "Invalid wxyz pose");
    for (double value : p)
        require(std::isfinite(value), "Invalid pose translation");
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    require(std::isfinite(rotation.norm()) && std::abs(rotation.norm() - 1) < 1e-6, "Invalid quaternion");
    return {rotation, Eigen::Vector3d(p[0], p[1], p[2])};
}
struct Operation {
    Json request;
    std::string status = "ACCEPTED", error;
    Json result = Json::object();
};
struct Shared {
    std::mutex mutex;
    std::map<std::string, Operation> operations;
    std::string pending;
    DeviceState state;
    ArmFeedback feedback;
    TrajectoryWindow window;
    bool has_window = false, revoke = false;
    std::atomic<bool> quit{false};
    std::atomic<uint64_t> accept_after{0};
};
Json armState(const DeviceState &s, const ArmFeedback &f, const std::string &backend) {
    Json b;
    for (int side = 0; side < 2; ++side) {
        auto &a = b["arms"][side ? "right" : "left"];
        std::array<double, 7> q{}, dq{};
        for (int j = 0; j < 7; ++j) {
            q[j] = f.q[side * 7 + j];
            dq[j] = f.dq[side * 7 + j];
        }
        const bool valid = f.valid[side];
        a = {{"valid", valid},
             {"status", s.fault ? "ERROR" : (s.stopping ? "SAFE" : (f.enabled[side] ? "ACTIVE" : "READY"))},
             {"enabled", f.enabled[side]},
             {"error_code", s.fault ? 28673 : 0},
             {"joint_position", q},
             {"joint_velocity", valid ? Json(dq) : Json(nullptr)},
             {"sample_mono_us",
              f.sample_time[side] > 0 ? Json(uint64_t(f.sample_time[side] * 1e6)) : Json(nullptr)}};
        if (valid) {
            const auto *p = f.tcp.data() + side * 7;
            a["tcp_pose"] = {
                {"frame_id", backend == "mujoco" ? "aircraft" : (side ? "right_base" : "left_base")},
                {"position", {{"x", p[0]}, {"y", p[1]}, {"z", p[2]}}},
                {"orientation", {{"qx", p[3]}, {"qy", p[4]}, {"qz", p[5]}, {"qw", p[6]}}}};
        } else
            a["tcp_pose"] = nullptr;
    }
    b["execution"] = {{"trajectory_id", s.id},  {"tick", s.cursor}, {"target", s.target},
                      {"stopping", s.stopping}, {"fault", s.fault}, {"error", s.error}};
    b["software_lock"] = s.locked;
    b["wheel_reference"] = {{"angle", s.angle}, {"displacement", s.displacement}};
    if (backend == "mujoco")
        b["wheel_measurement"] = {{"angle", s.measured_angle}, {"displacement", s.measured_displacement}};
    return b;
}
void executor(Shared &shared, DataLink &device, const MotionConfig &config, const Joints &lo,
              const Joints &hi, const Joints &speed, double braking) {
    DeviceState state;
    TrajectoryWindow active{};
    bool have = false;
    uint64_t last_message = 0, highest_trajectory = 0;
    auto snapshot = [&] {
        auto f = device.armFeedback();
        const auto g = device.graspState();
        state.measured_angle = g.angle;
        state.measured_displacement = g.displacement;
        state.q = f.q;
        state.dq = f.dq;
        state.target = f.target;
        state.enabled = f.enabled;
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.state = state;
        shared.feedback = f;
    };
    auto disable = [&] {
        device.commandDeadline(0);
        std::exception_ptr failure;
        for (auto side : {Side::Right, Side::Left})
            try {
                device.disable(side);
            } catch (...) {
                failure = std::current_exception();
            }
        have = false;
        state.id = state.cursor = 0;
        if (failure)
            std::rethrow_exception(failure);
    };
    Joints velocity{};
    auto stop = [&] {
        // Local bounded deceleration from the last command; never jump to measured q.
        state.stopping = true;
        double duration = 0;
        for (double v : velocity)
            duration = std::max(duration, std::abs(v) / braking);
        const size_t ticks = std::max<size_t>(1, std::ceil(duration / .001));
        device.commandDeadline(double(monotonic_us()) / 1e6 + duration + 0.5);
        const auto initial = velocity;
        for (size_t k = 1; k <= ticks; ++k) {
            device.waitTick();
            auto next = state.target;
            for (size_t j = 0; j < 14; ++j) {
                next[j] += initial[j] * (1 - double(k) / ticks) * .001;
                require(next[j] >= lo[j] && next[j] <= hi[j], "Local braking joint limit");
            }
            device.setJointPositions(next);
            state.target = next;
        }
        device.waitTick();
        velocity = {};
        have = false;
        state.id = state.cursor = 0;
        state.stopping = false;
        device.commandDeadline(0);
    };
    uint64_t last_snapshot = 0;
    while (!shared.quit) {
        std::string key;
        Json request;
        TrajectoryWindow incoming;
        bool new_window = false, revoke = false;
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            key = shared.pending;
            if (!key.empty()) {
                auto &operation = shared.operations.at(key);
                request = operation.request;
                if (monotonic_us() >= request.at("issued_mono_us").get<uint64_t>() +
                                          request.at("deadline_ms").get<uint64_t>() * 1000) {
                    operation.status = "EXPIRED";
                    operation.error = "Request expired before execution";
                    key.clear();
                } else
                    operation.status = "RUNNING";
                shared.pending.clear();
            }
            if (shared.has_window) {
                incoming = shared.window;
                shared.has_window = false;
                new_window = true;
            }
            revoke = shared.revoke;
            shared.revoke = false;
        }
        try {
            if (!key.empty()) {
                const auto op = request.at("operation").get<std::string>();
                Json result = Json::object();
                if (op == "enable") {
                    require(!state.fault, "Reset fault before enabling");
                    try {
                        device.enable(Side::Left);
                        device.enable(Side::Right);
                    } catch (...) {
                        disable();
                        throw;
                    }
                    state.target = device.jointTargets();
                    velocity = {};
                    state.id = state.cursor = 0;
                    highest_trajectory = 0;
                    have = false;
                    last_message = monotonic_us();
                    device.commandDeadline(double(last_message) / 1e6 + 1.0);
                    result["target"] = state.target;
                } else if (op == "disable") {
                    stop();
                    disable();
                    state.locked = false;
                } else if (op == "stop") {
                    stop();
                    result["target"] = state.target;
                } else if (op == "lock" || op == "unlock") {
                    if (op == "lock")
                        require(device.isEnabled(Side::Left) && device.isEnabled(Side::Right),
                                "Enable both arms first");
                    device.sendGraspCommand(op == "lock" ? GraspCommand::Lock : GraspCommand::Unlock);
                    state.locked = op == "lock";
                } else if (op == "reset_fault") {
                    require(!device.isEnabled(Side::Left) && !device.isEnabled(Side::Right),
                            "Disable before reset");
                    device.sendGraspCommand(GraspCommand::Unlock);
                    device.sendGraspCommand(GraspCommand::ResetFault);
                    state.fault = false;
                    state.error.clear();
                    state.locked = false;
                    have = false;
                } else
                    throw std::runtime_error("Unknown device operation");
                if (op == "stop" || op == "disable" || op == "enable" || op == "reset_fault") {
                    shared.accept_after = monotonic_us();
                    new_window = false;
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    shared.has_window = false;
                }
                snapshot();
                std::lock_guard<std::mutex> lock(shared.mutex);
                auto &record = shared.operations.at(key);
                record.status = "COMPLETED";
                record.result = result;
            }
            if (revoke && (state.enabled[0] || state.enabled[1]))
                throw std::runtime_error("Core command invalid or expired");
            const auto now = monotonic_us();
            if (new_window && !state.fault) {
                require(now >= incoming.sample && now - incoming.sample < config.timeout_us &&
                            now >= incoming.origin_sample &&
                            now - incoming.origin_sample < config.origin_timeout_us,
                        "Stale executor command");
                require(device.isEnabled(Side::Left) && device.isEnabled(Side::Right),
                        "Command received while disabled");
                if (incoming.id != state.id) {
                    require(incoming.id > highest_trajectory, "Retired trajectory ID");
                    require(incoming.first == 0, "New trajectory must start at tick zero");
                    require(!have || state.cursor >= active.total, "Previous trajectory still active");
                    for (size_t j = 0; j < 14; ++j)
                        require(std::abs(incoming.frames[0].q[j] - state.target[j]) < 1e-7,
                                "New trajectory starts away from last command");
                    state.id = incoming.id;
                    highest_trajectory = incoming.id;
                    state.cursor = 0;
                } else {
                    require(incoming.streaming == active.streaming, "Trajectory mode changed");
                    if (active.streaming) require(incoming.total >= active.total, "Servo total regressed");
                    require(incoming.first <= state.cursor && incoming.first + incoming.count > state.cursor,
                            "Replacement window does not cover execution cursor");
                    for (size_t j = 0; j < 14; ++j)
                        require(std::abs(incoming.frames[state.cursor - incoming.first].q[j] -
                                         state.target[j]) < 1e-7,
                                "Window replacement position discontinuity");
                    if (active.streaming) {
                        const auto begin = std::max(active.first, incoming.first);
                        const auto end = std::min(active.first + active.count, incoming.first + incoming.count);
                        for (auto tick = begin; tick < end; ++tick)
                            for (size_t j = 0; j < 14; ++j) {
                                const auto &a = active.frames[tick - active.first];
                                const auto &b = incoming.frames[tick - incoming.first];
                                require(std::abs(a.q[j] - b.q[j]) < 1e-9 && std::abs(a.dq[j] - b.dq[j]) < 1e-8 &&
                                        std::abs(a.ddq[j] - b.ddq[j]) < 1e-7, "Servo rewrote committed samples");
                            }
                    }
                }
                active = incoming;
                have = true;
                last_message = now;
                state.sequence = incoming.sequence;
                device.commandDeadline(double(std::min(incoming.sample + config.timeout_us,
                                                       incoming.origin_sample + config.origin_timeout_us)) /
                                           1e6 +
                                       0.02);
            }
            if (have && !state.fault) {
                require(now >= active.sample && now - active.sample < config.timeout_us &&
                            now >= active.origin_sample &&
                            now - active.origin_sample < config.origin_timeout_us,
                        "Local command watchdog expired: sample_age_us=" + std::to_string(now - active.sample) +
                        " origin_age_us=" + std::to_string(now - active.origin_sample) +
                        " tick=" + std::to_string(state.cursor) + " total=" + std::to_string(active.total));
                if (state.cursor < active.total) {
                    require(state.cursor + 1 < active.first + active.count,
                            "Trajectory window exhausted: tick=" + std::to_string(state.cursor) + " first=" +
                                std::to_string(active.first) + " count=" + std::to_string(active.count) +
                                " age_us=" + std::to_string(now - active.sample));
                    device.waitTick(); // both SDK callbacks consumed previous command
                    const auto &f = active.frames[state.cursor + 1 - active.first];
                    const auto &previous = active.frames[state.cursor - active.first];
                    for (size_t j = 0; j < 14; ++j) {
                        require(std::isfinite(f.q[j]) && f.q[j] >= lo[j] && f.q[j] <= hi[j] &&
                                    (active.streaming || std::abs(f.q[j] - state.target[j]) <= speed[j] * .001 + 1e-8),
                                "Local command continuity/limit violation");
                        if (active.streaming)
                            require(std::isfinite(f.dq[j]) && std::isfinite(f.ddq[j]) &&
                                    std::abs(f.q[j] - previous.q[j] - .0005 * (f.dq[j] + previous.dq[j])) < 1e-7,
                                    "Local Servo sample/derivative mismatch");
                        velocity[j] = active.streaming ? f.dq[j] : (f.q[j] - state.target[j]) * 1000;
                    }
                    device.setJointPositions(f.q);
                    device.setWheelReference(f.angle, f.displacement);
                    state.target = f.q;
                    state.angle = f.angle;
                    state.displacement = f.displacement;
                    ++state.cursor;
                } else {
                    require(!active.streaming || active.finished, "Servo buffer underrun");
                    velocity = {};
                    device.waitTick();
                }
            } else {
                if (state.fault)
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                else
                    device.waitTick();
                if ((state.enabled[0] || state.enabled[1]) && last_message && now - last_message > 1000000)
                    throw std::runtime_error("No command after enable/stop");
            }
            if (now - last_snapshot >= 5000) {
                snapshot();
                last_snapshot = now;
                if (state.enabled[0] && state.enabled[1]) {
                    const auto health = device.graspState();
                    require(!health.fault, "Device feedback/SDK fault: " + health.fault_reason);
                }
            }
        } catch (const std::exception &e) {
            state.fault = true;
            state.error = e.what();
            try {
                stop();
            } catch (...) {
            }
            try {
                disable();
            } catch (const std::exception &stop_error) {
                state.error += "; " + std::string(stop_error.what());
            }
            std::cerr << "manipulator: " << state.error << std::endl;
            if (!key.empty()) {
                std::lock_guard<std::mutex> lock(shared.mutex);
                auto &record = shared.operations.at(key);
                record.status = "FAILED";
                record.error = state.error;
            }
            try {
                snapshot();
            } catch (...) {
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.state = state;
            }
        }
    }
    try {
        stop();
        disable();
    } catch (const std::exception &e) {
        std::cerr << "manipulator shutdown: " << e.what() << '\n';
    }
}
} // namespace
int main(int argc, char **argv) {
    try {
        auto file = defaultSystemConfig();
        bool headless = false;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--help") {
                std::cout << "manipulator [--config system.yaml] [--headless]\nBackend from robot.yaml; "
                             "starts disabled.\n";
                return 0;
            }
            if (a == "--headless")
                headless = true;
            else if (a == "--config" && i + 1 < argc)
                file = argv[++i];
            else
                throw std::runtime_error("Unknown option: " + a);
        }
        const auto config = loadMotionConfig(file);
        const auto y = YAML::LoadFile(config.robot.string());
        auto path = [&](const char *key) {
            std::filesystem::path p = y[key].as<std::string>();
            return p.is_absolute() ? p : config.robot.parent_path() / p;
        };
        const std::string backend = y["backend"].as<std::string>();
        const auto urdf = urdf::parseURDFFile(path("urdf").string());
        require(bool(urdf), "Cannot load control URDF");
        Joints lo{}, hi{}, speed{};
        for (int side = 0; side < 2; ++side)
            for (int j = 0; j < 7; ++j) {
                auto joint = urdf->getJoint(std::string("AR5-5_07") + (side ? "R" : "L") + "-W4C4A2_joint_" +
                                            std::to_string(j + 1));
                require(joint && joint->limits, "Missing joint limits");
                const int i = side * 7 + j;
                lo[i] = joint->limits->lower;
                hi[i] = joint->limits->upper;
                speed[i] = std::min(joint->limits->velocity, y["joint_speed"].as<double>());
            }
        const auto posture = YAML::LoadFile(path("posture").string());
        for (int i : {1, 8}) {
            lo[i] = std::max(lo[i], posture["joint2_limits_deg"][0].as<double>() * M_PI / 180);
            hi[i] = std::min(hi[i], posture["joint2_limits_deg"][1].as<double>() * M_PI / 180);
        }
        const double braking = y["stop_acceleration"].as<double>();
        require(std::isfinite(braking) && braking > 0, "Invalid braking acceleration");
        auto g = YAML::LoadFile(path("grasp").string());
        GraspGeometry geometry;
        geometry.tool = frame(g["tool"]);
        geometry.wheel_origin = frame(g["wheel_origin"]);
        geometry.handles[0] = frame(g["left"]);
        geometry.handles[1] = frame(g["right"]);
        RokaeConfig rk;
        auto r = y["rokae"];
        if (backend == "rokae") {
            rk.grasp_mode = r["grasp_mode"].as<std::string>();
            rk.left_ip = r["left_ip"].as<std::string>();
            rk.right_ip = r["right_ip"].as<std::string>();
            rk.left_local_ip = r["left_local_ip"].as<std::string>();
            rk.right_local_ip = r["right_local_ip"].as<std::string>();
            auto stiffness = r["joint_stiffness"].as<std::vector<double>>();
            require(stiffness.size() == 7, "Seven stiffness values required");
            std::copy(stiffness.begin(), stiffness.end(), rk.joint_stiffness.begin());
        }
        BackendContext context;
#ifdef AVIATOR_HAVE_MUJOCO
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(nullptr, mj_deleteModel);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(nullptr, mj_deleteData);
        if (backend == "mujoco") {
            char error[1024]{};
            model.reset(mj_loadXML(path("model").c_str(), nullptr, error, sizeof(error)));
            require(bool(model), error);
            data.reset(mj_makeData(model.get()));
            require(bool(data), "mj_makeData failed");
            int key = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
            require(key >= 0, "Missing aviator_home");
            mj_resetDataKeyframe(model.get(), data.get(), key);
            mj_forward(model.get(), data.get());
            context = {model.get(), data.get()};
        }
#endif
        auto device = makeDataLink(backend, context, path("urdf").string(), geometry, rk);
        device->setRealTime(true);
#ifdef AVIATOR_HAVE_GLFW
        std::unique_ptr<Viewer> viewer;
        if (context.model && !headless && y["viewer"].as<bool>(true))
            viewer = std::make_unique<Viewer>(context.model);
#endif
        const std::string session = new_session_id();
        std::string core_session, epoch;
        zmq::context_t zmq_context(1);
        zmq::socket_t pub(zmq_context, zmq::socket_type::pub), sub(zmq_context, zmq::socket_type::sub),
            rep(zmq_context, zmq::socket_type::rep);
        configure(pub);
        configure(sub);
        configure(rep);
        subscribe(sub, "arm.command");
        rep.set(zmq::sockopt::maxmsgsize, int64_t(max_payload_bytes));
        pub.connect(config.publish);
        sub.connect(config.subscribe);
        rep.bind(config.service);
        Shared shared;
        shared.feedback = device->armFeedback();
        shared.state.q = shared.feedback.q;
        shared.state.target = shared.feedback.target;
        std::thread worker([&] { executor(shared, *device, config, lo, hi, speed, braking); });
        struct Join {
            Shared &s;
            std::thread &t;
            ~Join() {
                s.quit = true;
                t.join();
            }
        } join{shared, worker};
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        ReceiveState receive_state;
        std::unique_ptr<InputGuard> guard;
        uint64_t seq = 0, hand_seq = 0, next = 0, rejected = 0;
        std::cout << "READY manipulator backend=" << backend << " session=" << session
                  << " service=" << config.service << std::endl;
        while (!interrupted) {
            DeviceState state;
            ArmFeedback feedback;
            {
                std::lock_guard<std::mutex> lock(shared.mutex);
                state = shared.state;
                feedback = shared.feedback;
            }
            zmq::message_t request_frame;
            if (rep.recv(request_frame, zmq::recv_flags::dontwait)) {
                Json req = Json::object(), reply;
                try {
                    require(!rep.get(zmq::sockopt::rcvmore), "Service requires one frame");
                    req = parseService(request_frame.to_string());
                    require(req.at("msg_type") == "ServiceRequest" && req.at("version") == "1.0" &&
                                req.at("client_id") == "aviator_core" && req.at("target") == "manipulator" &&
                                req.at("clock_id") == local_clock_id(),
                            "Unauthorized service envelope");
                    for (const auto *name : {"timestamp", "issued_mono_us", "deadline_ms"}) {
                        const auto &value = req.at(name);
                        require(value.is_number_integer() && value.get<double>() >= 0 &&
                                    value.get<double>() <= max_json_integer,
                                "Expected uint53 service time");
                    }
                    require(req.at("parameters").is_object(), "Service parameters must be an object");
                    const auto now = monotonic_us(), issued = req.at("issued_mono_us").get<uint64_t>(),
                               deadline = req.at("deadline_ms").get<uint64_t>();
                    require(deadline > 0 && deadline <= 10000 && issued <= now &&
                                now - issued < deadline * 1000,
                            "Expired service request");
                    const std::string op = req.at("operation"), client = req.at("client_session_id"),
                                      id = req.at("request_id");
                    auto uuid = [](const std::string &value) {
                        if (value.size() != 36)
                            return false;
                        for (size_t n = 0; n < 36; ++n) {
                            const char c = value[n];
                            if (n == 8 || n == 13 || n == 18 || n == 23) {
                                if (c != '-')
                                    return false;
                            } else if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
                                return false;
                        }
                        return true;
                    };
                    require(uuid(client) && uuid(id), "Invalid request/session UUID");
                    reply = {{"msg_type", "ServiceReply"},
                             {"version", "1.0"},
                             {"request_id", id},
                             {"client_id", "aviator_core"},
                             {"client_session_id", client},
                             {"server_id", "manipulator"},
                             {"server_session_id", session},
                             {"timestamp", utc_us()},
                             {"status", "COMPLETED"},
                             {"error_code", 0},
                             {"result", Json::object()}};
                    if (op == "describe")
                        reply["result"] = {{"server_session", session},
                                           {"target", state.target},
                                           {"q", state.q},
                                           {"speed", speed},
                                           {"config_id", config.config_id},
                                           {"backend", backend}};
                    else if (op == "get_result") {
                        const auto &params = req.at("parameters");
                        require(params.size() == 4 && params.at("server_session") == session &&
                                    params.at("config_id") == config.config_id,
                                "Invalid result query context");
                        const std::string original_id = params.at("original_request_id"),
                                          original_session = params.at("original_client_session_id");
                        require(uuid(original_id) && uuid(original_session),
                                "Invalid original request identity");
                        std::lock_guard<std::mutex> lock(shared.mutex);
                        const auto found = shared.operations.find(original_session + ":" + original_id);
                        reply["result"] = {{"request_id", original_id},
                                           {"status", "UNKNOWN"},
                                           {"error_code", 0},
                                           {"result", Json::object()}};
                        if (found != shared.operations.end()) {
                            reply["result"]["status"] = found->second.status;
                            reply["result"]["error_code"] = found->second.error.empty() ? 0 : 4097;
                            reply["result"]["result"] = found->second.result;
                        }
                    } else {
                        const auto &params = req.at("parameters");
                        require(params.is_object() && params.size() == 2, "Unexpected service parameters");
                        require(params.at("server_session") == session,
                                "Stale server session; rediscover before acting");
                        require(params.at("config_id") == config.config_id, "Configuration mismatch");
                        const auto key = client + ":" + id;
                        std::lock_guard<std::mutex> lock(shared.mutex);
                        auto found = shared.operations.find(key);
                        if (found != shared.operations.end()) {
                            require(found->second.request == req, "Same request_id with different content");
                        } else {
                            if (shared.operations.size() >= 1024)
                                throw std::runtime_error(
                                    "Service dedup capacity exhausted; restart while disabled");
                            require(shared.pending.empty(), "Another service operation is pending");
                            if (op == "authorize") {
                                require(!shared.state.enabled[0] && !shared.state.enabled[1] &&
                                            !shared.state.fault,
                                        "Authorize only while disabled and healthy");
                                for (const auto &entry : shared.operations)
                                    require(entry.second.status != "RUNNING" &&
                                                entry.second.status != "ACCEPTED",
                                            "Operation in progress");
                                core_session = client;
                                epoch = new_session_id();
                                InputPolicy policy;
                                policy.topic = Topic::arm_command;
                                policy.publisher_id = "aviator_core";
                                policy.session_id = client;
                                policy.clock_id = local_clock_id();
                                policy.control_epoch = epoch;
                                policy.origin_topic = "local.task";
                                policy.origin_publisher_id = "aviator_core";
                                policy.origin_session_id = client;
                                policy.timeout_us = config.timeout_us;
                                policy.origin_timeout_us = config.origin_timeout_us;
                                guard = std::make_unique<InputGuard>(policy);
                                shared.has_window = false;
                                Operation operation;
                                operation.request = req;
                                operation.status = "COMPLETED";
                                operation.result = {{"control_epoch", epoch}};
                                shared.operations.emplace(key, std::move(operation));
                            } else {
                                require(client == core_session && !epoch.empty(),
                                        "Unauthorized client session");
                                require(op == "enable" || op == "disable" || op == "stop" || op == "lock" ||
                                            op == "unlock" || op == "reset_fault",
                                        "Unknown operation");
                                shared.operations.emplace(key, Operation{req});
                                shared.pending = key;
                            }
                            found = shared.operations.find(key);
                        }
                        reply["status"] = found->second.status;
                        reply["result"] = found->second.result;
                        reply["message"] = found->second.error;
                        reply["error_code"] = found->second.error.empty() ? 0 : 4097;
                    }
                } catch (const std::exception &e) {
                    for (int frames = 0; frames < 32 && rep.get(zmq::sockopt::rcvmore); ++frames) {
                        zmq::message_t discard;
                        (void)rep.recv(discard);
                    }
                    if (rep.get(zmq::sockopt::rcvmore)) {
                        rep.close();
                        rep = zmq::socket_t(zmq_context, zmq::socket_type::rep);
                        configure(rep);
                        rep.set(zmq::sockopt::maxmsgsize, int64_t(max_payload_bytes));
                        rep.bind(config.service);
                        continue;
                    }
                    if (!req.is_object())
                        req = Json::object();
                    if (!req.contains("request_id") || !req["request_id"].is_string())
                        req["request_id"] = "";
                    if (!req.contains("client_session_id") || !req["client_session_id"].is_string())
                        req["client_session_id"] = "";
                    reply = {{"msg_type", "ServiceReply"},
                             {"version", "1.0"},
                             {"request_id", req.value("request_id", "")},
                             {"client_session_id", req.value("client_session_id", "")},
                             {"status", "REJECTED"},
                             {"message", e.what()},
                             {"error_code", 4097},
                             {"result", Json::object()}};
                }
                const auto bytes = reply.dump();
                rep.send(zmq::buffer(bytes));
            }
            for (int n = 0; n < 64; ++n) {
                WireMessage wire;
                std::string error;
                auto received = receive(sub, receive_state, wire, error);
                if (received == ReceiveResult::empty)
                    break;
                Message m;
                if (received != ReceiveResult::received || !decode(wire.topic, wire.payload, m, error)) {
                    ++rejected;
                    continue;
                }
                try {
                    require(bool(guard), "Not authorized");
                    require(m.body.at("config_id") == config.config_id, "config_id mismatch");
                    require(m.body.at("origin").at("topic") == "local.task",
                            "Only explicitly authorized local tasks supported");
                    auto window = decodeWindow(m, lo, hi, speed);
                    require(window.sample >= shared.accept_after.load(),
                            "Command predates lifecycle barrier");
                    if (!guard->accept(m, monotonic_us(), error)) {
                        if (error == "invalid business data") {
                            std::lock_guard<std::mutex> lock(shared.mutex);
                            shared.revoke = true;
                        }
                        throw std::runtime_error(error);
                    }
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    shared.window = window;
                    shared.has_window = true;
                } catch (const std::exception &e) {
                    ++rejected;
                    if (rejected < 5)
                        std::cerr << "Rejected arm.command: " << e.what() << '\n';
                }
            }
            const auto now = monotonic_us();
            if (now >= next) {
                next =
                    next && now - next < config.period_us ? next + config.period_us : now + config.period_us;
                auto m = motionMessage(Topic::arm_state, "manipulator", session, ++seq,
                                       feedback.valid[0] && feedback.valid[1]);
                if (feedback.sample_time[0] > 0 && feedback.sample_time[1] > 0)
                    m.header.sample_mono_us =
                        uint64_t(std::min(feedback.sample_time[0], feedback.sample_time[1]) * 1e6);
                m.body = armState(state, feedback, backend);
                m.body["config_id"] = config.config_id;
                m.body["accepted_command"] = state.sequence ? Json{{"publisher_id", "aviator_core"},
                                                                   {"session_id", core_session},
                                                                   {"sequence", state.sequence},
                                                                   {"control_epoch", epoch}}
                                                            : Json(nullptr);
                publishMessage(pub, m);
                auto hand = motionMessage(Topic::hand_state, "manipulator", session, ++hand_seq, false);
                for (auto side : {"left", "right"})
                    hand.body["hands"][side] = {{"valid", false},
                                                {"status", "OFFLINE"},
                                                {"enabled", false},
                                                {"error_code", 0},
                                                {"sample_mono_us", nullptr},
                                                {"joint_position", nullptr},
                                                {"joint_velocity", nullptr},
                                                {"grasp_verified", false}};
                publishMessage(pub, hand);
            }
#ifdef AVIATOR_HAVE_GLFW
            static uint64_t draw_at = 0;
            if (viewer && now >= draw_at) {
                draw_at = now + 16666;
                if (!viewer->draw(context.data, *device->physicsMutex()))
                    break;
            }
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "manipulator: " << e.what() << '\n';
        return 1;
    }
}

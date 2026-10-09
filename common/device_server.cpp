#include "Logger.hpp"
#include "device_server.hpp"
#include <atomic>
#include <cmath>
#include <map>
#include <thread>
using namespace aviator;
namespace {
void require(bool ok, const std::string& why) {
    if (!ok)
        throw std::runtime_error(why);
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
    std::string window_instance, accepted_instance;
    bool has_window = false, revoke = false;
    std::atomic<bool> quit{false};
    std::atomic<uint64_t> accept_after{0}, control_alive{0}, retired_trajectory{0};
};
Json armState(const DeviceState& s, const ArmFeedback& f, const DeviceServerOptions& options) {
    Json b;
    for (int side = 0; side < 2; ++side) {
        auto& a = b["arms"][side ? "right" : "left"];
        std::array<double, 7> q{}, dq{};
        for (int j = 0; j < 7; ++j) {
            q[j] = f.q[side * 7 + j];
            dq[j] = f.dq[side * 7 + j];
        }
        const bool valid = f.valid[side];
        a = {{"valid", valid},
             {"status",
              s.fault ? "ERROR" : (s.stopping ? "SAFE" : (f.enabled[side] ? "ACTIVE" : "READY"))},
             {"enabled", f.enabled[side]},
             {"error_code", s.fault ? 28673 : 0},
             {"joint_position", q},
             {"joint_velocity", valid ? Json(dq) : Json(nullptr)},
             {"sample_mono_us",
              f.sample_time[side] > 0 ? Json(uint64_t(f.sample_time[side] * 1e6)) : Json(nullptr)}};
        if (valid) {
            const auto* p = f.tcp.data() + side * 7;
            a["tcp_pose"] = {
                {"frame_id", options.tcp_frames[side]},
                {"position", {{"x", p[0]}, {"y", p[1]}, {"z", p[2]}}},
                {"orientation", {{"qx", p[3]}, {"qy", p[4]}, {"qz", p[5]}, {"qw", p[6]}}}};
        } else
            a["tcp_pose"] = nullptr;
    }
    b["execution"] = {{"trajectory_id", s.id},  {"tick", s.cursor}, {"target", s.target},
                      {"stopping", s.stopping}, {"fault", s.fault}, {"error", s.error}};
    b["execution"]["impedance_switching"] = s.impedance_switching;
    b["execution"]["impedance_profile"] = s.impedance_profile;
    b["software_lock"] = s.locked;
    b["wheel_reference"] = {{"angle", s.angle}, {"displacement", s.displacement}};
    if (options.wheel_measurement)
        b["wheel_measurement"] = {{"angle", s.measured_angle},
                                  {"displacement", s.measured_displacement}};
    return b;
}
void executor(Shared& shared, DataLink& device, const MotionConfig& config, const Joints& lo,
              const Joints& hi, const Joints& speed, double braking, WheelReference initial,
              const DeviceSettings& settings) {
    DeviceState state;
    state.angle = initial.angle;
    state.displacement = initial.displacement;
    auto current_stiffness = settings.default_stiffness;
    TrajectoryWindow active{};
    std::string active_instance;
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
        shared.accepted_instance = active_instance;
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
        snapshot();
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
            if (k % 5 == 0)
                snapshot(); // Keep feedback/fault status fresh throughout local braking.
        }
        device.waitTick();
        velocity = {};
        have = false;
        state.id = state.cursor = 0;
        state.stopping = false;
        device.commandDeadline(0);
        // Explicit stop has reached a local hold. Only enable arms the first-command
        // deadline; the next accepted window re-arms the normal command watchdog.
        last_message = 0;
    };
    uint64_t last_snapshot = 0;
    while (!shared.quit) {
        std::string key;
        Json request;
        TrajectoryWindow incoming;
        std::string incoming_instance;
        bool new_window = false, revoke = false;
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            key = shared.pending;
            if (!key.empty()) {
                auto& operation = shared.operations.at(key);
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
                incoming_instance = shared.window_instance;
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
                    // Every enable starts with the configured default, including simulation.
                    device.setJointStiffness(settings.default_stiffness, [] {});
                    current_stiffness = settings.default_stiffness;
                    state.impedance_profile = "default";
                    state.target = device.jointTargets();
                    velocity = {};
                    state.id = state.cursor = 0;
                    highest_trajectory = 0;
                    shared.retired_trajectory = 0;
                    have = false;
                    last_message = monotonic_us();
                    device.commandDeadline(double(last_message) / 1e6 + 1.0);
                    result["target"] = state.target;
                } else if (op == "set_impedance_profile") {
                    require(!state.fault && state.enabled[0] && state.enabled[1],
                            "Impedance update requires healthy enabled arms");
                    require(!have || (state.cursor >= active.total && (!active.streaming || active.finished)),
                            "Finish the trajectory before changing impedance");
                    for (double v : velocity) require(std::abs(v) < 1e-8, "Impedance update requires a held target");
                    const std::string profile = request.at("parameters").at("profile");
                    const auto& stiffness = profile == "following" ? settings.following_stiffness : settings.default_stiffness;
                    Logger::info("Impedance request profile={} mono_us={} trajectory_id={} tick={} target_rad={} "
                        "wheel_angle_rad={} wheel_displacement_m={}", profile, monotonic_us(), state.id, state.cursor,
                        Json(state.target).dump(), state.angle, state.displacement);
                    const bool force_reapply = profile == "following";
                    if (force_reapply || stiffness != current_stiffness) {
                        const uint64_t deadline = request.at("issued_mono_us").get<uint64_t>() +
                            request.at("deadline_ms").get<uint64_t>() * 1000;
                        const auto check = [&] {
                            const auto now = monotonic_us();
                            const auto alive = shared.control_alive.load();
                            require(!shared.quit && now < deadline, "Impedance update cancelled or expired");
                            require(alive && now >= alive && now - alive < config.origin_timeout_us,
                                    "Core heartbeat lost during impedance update");
                            std::lock_guard<std::mutex> lock(shared.mutex);
                            require(!shared.revoke, "Core authorization revoked during impedance update");
                        };
                        check();
                        state.impedance_switching = true;
                        snapshot();
                        // Stop consuming windows, but their validated origin heartbeat stays live.
                        have = false;
                        shared.retired_trajectory = highest_trajectory;
                        device.commandDeadline(double(deadline) / 1e6);
                        device.setJointStiffness(stiffness, check, force_reapply);
                        check();
                        current_stiffness = stiffness;
                        state.impedance_switching = false;
                        state.id = state.cursor = 0;
                        last_message = 0;
                        device.commandDeadline(0);
                    }
                    state.impedance_profile = profile;
                    result = {{"profile", profile}, {"stiffness", stiffness}, {"target", state.target}};
                    Logger::info("Joint stiffness update profile={} values={}", profile, Json(stiffness).dump());
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
                    device.sendGraspCommand(op == "lock" ? GraspCommand::Lock
                                                         : GraspCommand::Unlock);
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
                if (op == "stop" || op == "disable" || op == "enable" || op == "reset_fault" || op == "set_impedance_profile") {
                    shared.accept_after = monotonic_us();
                    new_window = false;
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    shared.has_window = false;
                }
                snapshot();
                std::lock_guard<std::mutex> lock(shared.mutex);
                auto& record = shared.operations.at(key);
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
                    require(!have || state.cursor >= active.total,
                            "Previous trajectory still active");
                    for (size_t j = 0; j < 14; ++j)
                        require(std::abs(incoming.frames[0].q[j] - state.target[j]) < 1e-7,
                                "New trajectory starts away from last command");
                    state.id = incoming.id;
                    highest_trajectory = incoming.id;
                    state.cursor = 0;
                } else {
                    require(incoming.streaming == active.streaming, "Trajectory mode changed");
                    if (active.streaming)
                        require(incoming.total >= active.total, "Servo total regressed");
                    require(incoming.first <= state.cursor &&
                                incoming.first + incoming.count > state.cursor,
                            "Replacement window does not cover execution cursor");
                    for (size_t j = 0; j < 14; ++j)
                        require(std::abs(incoming.frames[state.cursor - incoming.first].q[j] -
                                         state.target[j]) < 1e-7,
                                "Window replacement position discontinuity");
                    if (active.streaming) {
                        const auto begin = std::max(active.first, incoming.first);
                        const auto end =
                            std::min(active.first + active.count, incoming.first + incoming.count);
                        for (auto tick = begin; tick < end; ++tick)
                            for (size_t j = 0; j < 14; ++j) {
                                const auto& a = active.frames[tick - active.first];
                                const auto& b = incoming.frames[tick - incoming.first];
                                require(std::abs(a.q[j] - b.q[j]) < 1e-9 &&
                                            std::abs(a.dq[j] - b.dq[j]) < 1e-8 &&
                                            std::abs(a.ddq[j] - b.ddq[j]) < 1e-7,
                                        "Servo rewrote committed samples");
                            }
                    }
                }
                active = incoming;
                have = true;
                last_message = now;
                state.sequence = incoming.sequence;
                active_instance = incoming_instance;
                device.commandDeadline(
                    double(std::min(incoming.sample + config.timeout_us,
                                    incoming.origin_sample + config.origin_timeout_us)) /
                        1e6 +
                    0.02);
            }
            if (have && !state.fault) {
                require(now >= active.sample && now - active.sample < config.timeout_us &&
                            now >= active.origin_sample &&
                            now - active.origin_sample < config.origin_timeout_us,
                        "Local command watchdog expired: sample_age_us=" +
                            std::to_string(now - active.sample) +
                            " origin_age_us=" + std::to_string(now - active.origin_sample) +
                            " tick=" + std::to_string(state.cursor) +
                            " total=" + std::to_string(active.total));
                if (state.cursor < active.total) {
                    require(state.cursor + 1 < active.first + active.count,
                            "Trajectory window exhausted: tick=" + std::to_string(state.cursor) +
                                " first=" + std::to_string(active.first) +
                                " count=" + std::to_string(active.count) +
                                " age_us=" + std::to_string(now - active.sample));
                    device.waitTick(); // both SDK callbacks consumed previous command
                    const auto& f = active.frames[state.cursor + 1 - active.first];
                    const auto& previous = active.frames[state.cursor - active.first];
                    for (size_t j = 0; j < 14; ++j) {
                        require(std::isfinite(f.q[j]) && f.q[j] >= lo[j] && f.q[j] <= hi[j] &&
                                    (active.streaming ||
                                     std::abs(f.q[j] - state.target[j]) <= speed[j] * .001 + 1e-8),
                                "Local command continuity/limit violation");
                        if (active.streaming)
                            require(std::isfinite(f.dq[j]) && std::isfinite(f.ddq[j]) &&
                                        std::abs(f.q[j] - previous.q[j] -
                                                 .0005 * (f.dq[j] + previous.dq[j])) < 1e-7,
                                    "Local Servo sample/derivative mismatch");
                        velocity[j] =
                            active.streaming ? f.dq[j] : (f.q[j] - state.target[j]) * 1000;
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
                if ((state.enabled[0] || state.enabled[1]) && last_message &&
                    now - last_message > 1000000)
                    throw std::runtime_error("No command after enable");
            }
            if (now - last_snapshot >= 5000) {
                snapshot();
                last_snapshot = now;
                if (state.enabled[0] && state.enabled[1]) {
                    const auto health = device.graspState();
                    require(!health.fault, "Device feedback/SDK fault: " + health.fault_reason);
                }
            }
        } catch (const std::exception& e) {
            state.impedance_switching = false;
            state.fault = true;
            state.error = e.what();
            try {
                stop();
            } catch (...) {
            }
            try {
                disable();
            } catch (const std::exception& stop_error) {
                state.error += "; " + std::string(stop_error.what());
            }
            Logger::error("manipulator: {}", state.error);
            if (!key.empty()) {
                std::lock_guard<std::mutex> lock(shared.mutex);
                auto& record = shared.operations.at(key);
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
    } catch (const std::exception& e) {
        Logger::error("manipulator shutdown: {}", e.what());
    }
}
} // namespace
namespace aviator {
int runDeviceServer(DataLink& device, const MotionConfig& config, const DeviceSettings& settings,
                    const DeviceServerOptions& options,
                    const volatile std::sig_atomic_t& interrupted) {
    const auto& lo = settings.lower;
    const auto& hi = settings.upper;
    const auto& speed = settings.speed;
    const auto braking = settings.braking;
    const auto initial_wheel = settings.initial_wheel;
    const std::string session = new_instance_id();
    std::string epoch;
    zmq::context_t zmq_context(1);
    zmq::socket_t pub(zmq_context, zmq::socket_type::pub), sub(zmq_context, zmq::socket_type::sub),
        rep(zmq_context, zmq::socket_type::rep);
    configure(pub);
    configure(sub);
    configure(rep);
    subscribe(sub, "arm.command");
    for (const auto& topic : options.topics)
        subscribe(sub, topic);
    rep.set(zmq::sockopt::maxmsgsize, int64_t(max_payload_bytes));
    pub.connect(config.publish);
    sub.connect(config.subscribe);
    rep.bind(config.service);
    Shared shared;
    shared.feedback = device.armFeedback();
    shared.state.q = shared.feedback.q;
    shared.state.target = shared.feedback.target;
    shared.state.angle = initial_wheel.angle;
    shared.state.displacement = initial_wheel.displacement;
    std::thread worker(
        [&] { executor(shared, device, config, lo, hi, speed, braking, initial_wheel, settings); });
    struct Join {
        Shared& s;
        std::thread& t;
        ~Join() {
            s.quit = true;
            t.join();
        }
    } join{shared, worker};
    ReceiveState receive_state;
    std::unique_ptr<InputGuard> guard;
    uint64_t seq = 0, next = 0, rejected = 0;
    Logger::info("READY {} session={} service={}", options.name, session, config.service);
    while (!interrupted) {
        DeviceState state;
        ArmFeedback feedback;
        std::string accepted_instance;
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            state = shared.state;
            feedback = shared.feedback;
            accepted_instance = shared.accepted_instance;
        }
        zmq::message_t request_frame;
        if (rep.recv(request_frame, zmq::recv_flags::dontwait)) {
            Json req = Json::object(), reply;
            try {
                require(!rep.get(zmq::sockopt::rcvmore), "Service requires one frame");
                req = parseService(request_frame.to_string());
                require(req.at("msg_type") == "ServiceRequest" && req.at("version") == "1.0" &&
                            req.at("client_id") == "aviator_core" &&
                            req.at("target") == "manipulator" &&
                            req.at("clock_id") == local_clock_id(),
                        "Unauthorized service envelope");
                for (const auto* name : {"timestamp", "issued_mono_us", "deadline_ms"}) {
                    const auto& value = req.at(name);
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
                auto uuid = [](const std::string& value) {
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
                require(uuid(id), "Invalid request UUID");
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
                                       {"capabilities", {{"status_before_enable", true}, {"impedance_profiles", true}}}};
                else if (op == "get_result") {
                    const auto& params = req.at("parameters");
                    require(params.size() == 2u + params.count("server_session") +
                                                 params.count("original_client_session_id") &&
                                params.at("config_id") == config.config_id,
                            "Invalid result query context");
                    const std::string original_id = params.at("original_request_id");
                    require(uuid(original_id), "Invalid original request identity");
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    const auto found = shared.operations.find(original_id);
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
                    const auto& params = req.at("parameters");
                    require(params.is_object() &&
                                params.size() == 1u + params.count("server_session") + (op == "set_impedance_profile" ? 1u : 0u),
                            "Unexpected service parameters");
                    require(params.at("config_id") == config.config_id, "Configuration mismatch");
                    if (op == "set_impedance_profile")
                        require(params.at("profile") == "default" || params.at("profile") == "following",
                                "Unknown impedance profile");
                    const auto key = id;
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    auto found = shared.operations.find(key);
                    if (found != shared.operations.end()) {
                        auto previous = found->second.request, incoming = req;
                        for (auto* value : {&previous, &incoming}) {
                            value->erase("client_session_id");
                            (*value)["parameters"].erase("server_session");
                        }
                        require(previous == incoming, "Same request_id with different content");
                    } else {
                        if (shared.operations.size() >= 1024)
                            throw std::runtime_error(
                                "Service dedup capacity exhausted; restart while disabled");
                        require(shared.pending.empty(), "Another service operation is pending");
                        if (op == "authorize") {
                            require(!shared.state.enabled[0] && !shared.state.enabled[1] &&
                                        !shared.state.fault,
                                    "Authorize only while disabled and healthy");
                            for (const auto& entry : shared.operations)
                                require(entry.second.status != "RUNNING" &&
                                            entry.second.status != "ACCEPTED",
                                        "Operation in progress");
                            shared.retired_trajectory = shared.control_alive = 0;
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
                            require(!epoch.empty(), "Control has not been authorized");
                            require(op == "enable" || op == "disable" || op == "stop" ||
                                        op == "lock" || op == "unlock" || op == "reset_fault" || op == "set_impedance_profile",
                                    "Unknown operation");
                            // A protection request must cancel any running reconfiguration before restart.
                            if ((op == "stop" || op == "disable") && shared.state.impedance_switching) shared.revoke = true;
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
            } catch (const std::exception& e) {
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
        // Large command windows can keep the receive queue busy. Bound each batch
        // so decoding cannot starve feedback publication and trigger its watchdog.
        const auto receive_deadline = monotonic_us() + 2000;
        for (int n = 0; n < 64 && (n == 0 || monotonic_us() < receive_deadline); ++n) {
            WireMessage wire;
            std::string error;
            auto received = receive(sub, receive_state, wire, error);
            if (received == ReceiveResult::empty)
                break;
            Message m;
            if (received != ReceiveResult::received ||
                !decode(wire.topic, wire.payload, m, error)) {
                ++rejected;
                continue;
            }
            if (m.topic != Topic::arm_command) {
                if (options.message)
                    options.message(m, monotonic_us());
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
                shared.control_alive = window.origin_sample;
                if (shared.state.impedance_switching || window.id <= shared.retired_trajectory.load()) continue;
                shared.window = window;
                shared.window_instance = m.header.session_id;
                shared.has_window = true;
            } catch (const std::exception& e) {
                ++rejected;
                if (rejected < 5)
                    Logger::warn("Rejected arm.command: {}", e.what());
            }
        }
        const auto now = monotonic_us();
        if (now >= next) {
            next = next && now - next < config.period_us ? next + config.period_us
                                                         : now + config.period_us;
            // Input decoding can consume a publication period. Sample the executor
            // mailbox at publication, not the snapshot used for earlier RPC handling.
            {
                std::lock_guard<std::mutex> lock(shared.mutex);
                state = shared.state;
                feedback = shared.feedback;
                accepted_instance = shared.accepted_instance;
            }
            auto m = motionMessage(Topic::arm_state, "manipulator", session, ++seq,
                                   feedback.valid[0] && feedback.valid[1] && !state.impedance_switching);
            if (feedback.sample_time[0] > 0 && feedback.sample_time[1] > 0)
                m.header.sample_mono_us =
                    uint64_t(std::min(feedback.sample_time[0], feedback.sample_time[1]) * 1e6);
            m.body = armState(state, feedback, options);
            // Device status remains available before Rokae's enabled RT callbacks exist.
            // This is NOT a replacement for the original joint/TCP sample timestamp.
            m.body["status_mono_us"] = now;
            m.body["config_id"] = config.config_id;
            m.body["accepted_command"] = state.sequence ? Json{{"publisher_id", "aviator_core"},
                                                               {"session_id", accepted_instance},
                                                               {"sequence", state.sequence},
                                                               {"control_epoch", epoch}}
                                                        : Json(nullptr);
            publishMessage(pub, m);
        }
        if (options.tick && !options.tick(pub, now))
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
} // namespace aviator

#include "motion.hpp"
#include <cmath>
#include <thread>
#include <unistd.h>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

namespace aviator {
namespace {
void require(bool ok, const char *reason) {
    if (!ok)
        throw std::runtime_error(reason);
}
uint64_t integer(const Json &j) {
    require(j.is_number_integer() && j.get<double>() >= 0 && j.get<double>() <= max_json_integer,
            "Expected uint53");
    return j.get<uint64_t>();
}
} // namespace
WheelReference loadInitialWheel(const std::filesystem::path& robot_file) {
    const auto config = YAML::LoadFile(robot_file.string());
    const auto initial = config["wheel_initial"];
    WheelReference value;
    if (initial) {
        require(initial.IsMap() && initial.size() == 2 && initial["angle"] && initial["displacement"],
                "wheel_initial requires angle (rad) and displacement (m)");
        value.angle = initial["angle"].as<double>();
        value.displacement = initial["displacement"].as<double>();
    }
    require(std::isfinite(value.angle) && std::abs(value.angle) <= .87266 &&
            std::isfinite(value.displacement) && value.displacement >= -.170 && value.displacement <= 0,
            "Invalid wheel_initial: angle must be in [-0.87266,0.87266] rad, displacement in [-0.170,0] m");
    return value;
}
double joystickWheelDisplacement(double pitch) {
    require(std::isfinite(pitch) && pitch >= -1 && pitch <= 1, "Joystick pitch must be in [-1,1]");
    return .085 * (pitch - 1.0);
}
std::filesystem::path defaultSystemConfig() {
    auto exe = std::filesystem::read_symlink("/proc/self/exe");
    auto installed = exe.parent_path() / "../share/aviator/config/system.yaml";
    return std::filesystem::exists(installed) ? installed : std::filesystem::path(AVIATOR_SOURCE_CONFIG);
}
MotionConfig loadMotionConfig(const std::filesystem::path &path) {
    MotionConfig c;
    c.system = std::filesystem::absolute(path);
    auto y = YAML::LoadFile(c.system.string());
    c.robot = y["robot"].as<std::string>();
    if (c.robot.is_relative())
        c.robot = c.system.parent_path() / c.robot;
    c.publish = y["bus"]["publish"].as<std::string>();
    c.subscribe = y["bus"]["subscribe"].as<std::string>();
    c.service = y["manipulator_service"].as<std::string>();
    c.config_id = y["config_id"].as<std::string>();
    require(!c.config_id.empty(), "Missing config_id");
    c.timeout_us = y["command_timeout_ms"].as<unsigned>() * 1000;
    c.origin_timeout_us = y["origin_timeout_ms"].as<unsigned>() * 1000;
    require(c.timeout_us >= 30000 && c.timeout_us <= 100000 && c.origin_timeout_us >= c.timeout_us &&
                c.origin_timeout_us <= 1000000,
            "Invalid watchdog configuration");
    for (const auto *endpoint : {&c.publish, &c.subscribe, &c.service})
        require(endpoint->rfind("tcp://127.0.0.1:", 0) == 0,
                "Motion nodes currently require same-host loopback TCP");
    require(c.publish != c.subscribe && c.publish != c.service && c.subscribe != c.service,
            "Endpoints must differ");
    return c;
}
Message motionMessage(Topic t, const std::string &publisher, const std::string &session, uint64_t seq,
                      bool valid) {
    Message m;
    m.topic = t;
    m.header.sequence = seq;
    m.header.timestamp = utc_us();
    m.header.sample_mono_us = monotonic_us();
    m.header.clock_id = local_clock_id();
    m.header.publisher_id = publisher;
    m.header.session_id = session;
    m.header.valid = valid;
    return m;
}
void publishMessage(zmq::socket_t &socket, const Message &m) {
    std::string payload, error;
    if (!encode(m, payload, error))
        throw std::runtime_error(error);
    send(socket, topic_name(m.topic), payload);
}
Json parseService(std::string_view text) {
    require(!text.empty() && text.size() <= max_payload_bytes && text.find('\0') == text.npos &&
                text.substr(0, 3) != "\xef\xbb\xbf",
            "Invalid service payload");
    std::vector<std::unordered_set<std::string>> keys;
    auto j = Json::parse(text.begin(), text.end(), [&](int depth, Json::parse_event_t event, Json &value) {
        require(depth < 16, "Service nesting limit");
        if (event == Json::parse_event_t::object_start)
            keys.emplace_back();
        if (event == Json::parse_event_t::key)
            require(keys.back().insert(value.get<std::string>()).second, "Duplicate service key");
        if (event == Json::parse_event_t::object_end)
            keys.pop_back();
        if (value.is_number())
            require(std::isfinite(value.get<double>()) && std::abs(value.get<double>()) <= max_json_integer,
                    "Invalid service number");
        return true;
    });
    require(j.is_object(), "Service must be an object");
    return j;
}
Json serviceRequest(const std::string &session, const std::string &op, const Json &params) {
    return {{"msg_type", "ServiceRequest"},
            {"version", "1.0"},
            {"request_id", new_session_id()},
            {"client_id", "aviator_core"},
            {"client_session_id", session},
            {"timestamp", utc_us()},
            {"issued_mono_us", monotonic_us()},
            {"clock_id", local_clock_id()},
            {"deadline_ms", 10000},
            {"operation", op},
            {"target", "manipulator"},
            {"parameters", params}};
}
Json callService(zmq::context_t &ctx, const MotionConfig &cfg, const Json &request) {
    // Retrying never creates another operation identity. Each timeout uses a fresh REQ socket.
    const auto deadline =
        request.at("issued_mono_us").get<uint64_t>() + request.at("deadline_ms").get<uint64_t>() * 1000;
    while (monotonic_us() < deadline) {
        zmq::socket_t socket(ctx, zmq::socket_type::req);
        configure(socket);
        socket.set(zmq::sockopt::rcvtimeo, 250);
        socket.set(zmq::sockopt::sndtimeo, 250);
        socket.set(zmq::sockopt::maxmsgsize, int64_t(max_payload_bytes));
        socket.connect(cfg.service);
        const auto payload = request.dump();
        if (!socket.send(zmq::buffer(payload)))
            continue;
        zmq::message_t frame;
        if (!socket.recv(frame))
            continue;
        require(!socket.get(zmq::sockopt::rcvmore), "Multipart service reply rejected");
        auto reply = parseService(frame.to_string());
        require(reply.at("msg_type") == "ServiceReply" &&
                    reply.at("request_id") == request.at("request_id") &&
                    reply.at("client_session_id") == request.at("client_session_id"),
                "Service correlation mismatch");
        const auto status = reply.at("status").get<std::string>();
        if (status == "COMPLETED")
            return reply.at("result");
        if (status != "ACCEPTED" && status != "RUNNING")
            throw std::runtime_error(reply.value("message", "Service failed"));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error(
        "Service deadline expired; operation result is UNKNOWN, do not repeat with a new request_id");
}
Json encodeWindow(const TrajectoryWindow &w, const std::string &session, const std::string &epoch) {
    Json b = {{"mode", "JOINT_TRAJECTORY"},
              {"execution", "SYNCHRONIZED_TICKS"},
              {"control_epoch", epoch},
              {"trajectory_id", w.id},
              {"first_tick", w.first},
              {"total_ticks", w.total},
              {"trajectory_start_mono_us", w.start},
              {"origin",
               {{"publisher_id", "aviator_core"},
                {"session_id", session},
                {"sequence", w.sequence},
                {"sample_mono_us", w.origin_sample},
                {"clock_id", local_clock_id()},
                {"topic", "local.task"}}}};
    const size_t stride = w.streaming ? 1 : 2;
    if (w.streaming) {
        b["streaming"] = true;
        b["finished"] = w.finished;
    }
    for (int side = 0; side < 2; ++side) {
        Json points = Json::array();
        for (size_t k = 0; k < w.count; k += stride) {
            std::array<double, 7> q{}, v{};
            for (int j = 0; j < 7; ++j) {
                q[j] = w.frames[k].q[7 * side + j];
                if (w.streaming) v[j] = w.frames[k].dq[7 * side + j];
                else if (k + 2 < w.count)
                    v[j] = (w.frames[k + 2].q[7 * side + j] - q[j]) * 500;
            }
            points.push_back(
                {{"time_from_start_us", k * 1000}, {"joint_position", q}, {"joint_velocity", v}});
        }
        if (w.streaming) {
            for (size_t k = 0; k < w.count; ++k) {
                std::array<double, 7> a{};
                for (int j = 0; j < 7; ++j) a[j] = w.frames[k].ddq[7 * side + j];
                points[k]["joint_acceleration"] = a;
            }
        }
        b["arms"][side ? "right" : "left"]["points"] = points;
    }
    b["wheel_reference"] = Json::array();
    for (size_t k = 0; k < w.count; k += stride)
        b["wheel_reference"].push_back({w.frames[k].angle, w.frames[k].displacement});
    return b;
}
TrajectoryWindow decodeWindow(const Message &m, const Joints &lo, const Joints &hi, const Joints &speed) {
    const auto &b = m.body;
    require(b.at("mode") == "JOINT_TRAJECTORY" && b.at("execution") == "SYNCHRONIZED_TICKS",
            "Unsupported trajectory mode");
    TrajectoryWindow w;
    w.streaming = b.value("streaming", false);
    w.finished = b.value("finished", false);
    const size_t stride = w.streaming ? 1 : 2;
    w.id = integer(b.at("trajectory_id"));
    w.first = integer(b.at("first_tick"));
    w.total = integer(b.at("total_ticks"));
    w.start = integer(b.at("trajectory_start_mono_us"));
    w.sequence = m.header.sequence;
    w.sample = m.header.sample_mono_us;
    w.origin_sample = integer(b.at("origin").at("sample_mono_us"));
    require(w.id > 0 && w.total >= 2 && w.total <= 3600000 && w.first <= w.total && w.first % stride == 0 &&
                w.total % stride == 0,
            "Trajectory range exceeds budget or tick grid");
    for (int side = 0; side < 2; ++side) {
        const auto &points = b.at("arms").at(side ? "right" : "left").at("points");
        require(points.is_array() && points.size() >= 2 && points.size() <= (w.streaming ? servo_window_points : 32),
                "Invalid trajectory point count");
        if (!side)
            w.count = stride * (points.size() - 1) + 1;
        else
            require(stride * (points.size() - 1) + 1 == w.count, "Dual-arm time grids differ");
        for (size_t k = 0; k < points.size(); ++k) {
            require(integer(points[k].at("time_from_start_us")) == k * stride * 1000, "Unexpected wire time grid");
            const auto &q = points[k].at("joint_position");
            const auto &v = points[k].at("joint_velocity");
            require(q.is_array() && v.is_array() && q.size() == 7 && v.size() == 7, "Expected seven joints");
            for (int j = 0; j < 7; ++j) {
                require(q[j].is_number() && v[j].is_number(), "Non-numeric joint");
                const int i = 7 * side + j;
                double x = q[j].get<double>();
                require(std::isfinite(x) && x >= lo[i] && x <= hi[i] &&
                            std::isfinite(v[j].get<double>()) &&
                            (w.streaming || std::abs(v[j].get<double>()) <= speed[i] + 1e-7),
                        "Joint limit/velocity violation");
                if (k && !w.streaming)
                    require(std::abs(x - w.frames[stride * (k - 1)].q[i]) <= speed[i] * stride * .001 + 1e-8,
                            "Discontinuous segment");
                w.frames[stride * k].q[i] = x;
                if (w.streaming) {
                    const auto &a = points[k].at("joint_acceleration");
                    require(a.is_array() && a.size() == 7 && a[j].is_number(), "Expected joint acceleration");
                    w.frames[k].dq[i] = v[j].get<double>();
                    w.frames[k].ddq[i] = a[j].get<double>();
                    require(std::isfinite(w.frames[k].dq[i]) && std::isfinite(w.frames[k].ddq[i]), "Nonfinite derivatives");
                }
            }
        }
    }
    require(w.first + w.count <= w.total + 1, "Window extends past trajectory end");
    if (w.streaming && (w.first == 0 || (w.finished && w.first + w.count == w.total + 1))) {
        const auto &f = w.first == 0 ? w.frames[0] : w.frames[w.count - 1];
        for (size_t j = 0; j < 14; ++j)
            require(std::abs(f.dq[j]) < 1e-8 && std::abs(f.ddq[j]) < 1e-8,
                    "Servo start/end must be at planned rest");
    }
    const auto &wheel = b.at("wheel_reference");
    require(wheel.is_array() && stride * (wheel.size() - 1) + 1 == w.count, "Missing wheel references");
    for (size_t k = 0; k < wheel.size(); ++k) {
        require(wheel[k].is_array() && wheel[k].size() == 2, "Invalid wheel reference");
        w.frames[stride * k].angle = wheel[k][0].get<double>();
        w.frames[stride * k].displacement = wheel[k][1].get<double>();
        require(std::isfinite(w.frames[stride * k].angle) && std::abs(w.frames[stride * k].angle) <= .87266 + 1e-8 &&
                    std::isfinite(w.frames[stride * k].displacement) &&
                    w.frames[stride * k].displacement >= -.170 - 1e-8 && w.frames[stride * k].displacement <= 1e-8,
                "Wheel reference out of range");
    }
    if (!w.streaming) for (size_t k = 1; k < w.count; k += 2) {
        for (size_t j = 0; j < 14; ++j)
            w.frames[k].q[j] = (w.frames[k - 1].q[j] + w.frames[k + 1].q[j]) * .5;
        w.frames[k].angle = (w.frames[k - 1].angle + w.frames[k + 1].angle) * .5;
        w.frames[k].displacement = (w.frames[k - 1].displacement + w.frames[k + 1].displacement) * .5;
    }
    return w;
}
} // namespace aviator

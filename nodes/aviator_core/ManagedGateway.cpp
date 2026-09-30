#include "ManagedGateway.hpp"
#include <algorithm>
#include <iostream>

namespace aviator {
ManagedGateway::ManagedGateway(const MotionConfig& config, const ManagedGatewayOptions& options) {
    const std::string prefix = "tcp://127.0.0.1:";
    const auto port = options.endpoint.substr(std::min(prefix.size(), options.endpoint.size()));
    if (options.endpoint.compare(0, prefix.size(), prefix) != 0 || port.empty() || port.size() > 5 ||
        port.find_first_not_of("0123456789") != std::string::npos ||
        std::stoul(port) == 0 || std::stoul(port) > 65535 ||
        options.endpoint == config.publish || options.endpoint == config.subscribe || options.endpoint == config.service)
        throw std::runtime_error("Operation service must use a distinct tcp://127.0.0.1:<port> endpoint");
    policy_.publisher_id = "flight_gateway";
    policy_.session_id = options.session;
    policy_.clock_id = local_clock_id();
    policy_.source = "JOYSTICK";
    policy_.allow_joystick_position_hold = true;
    policy_.timeout_us = std::min<uint64_t>(100000, config.origin_timeout_us);
    if (!options.session.empty()) input_.emplace(policy_);
    configure(sub_, {64, 64, 0});
    subscribe(sub_, "flight.command");
    sub_.connect(config.subscribe);
    configure(router_, {16, 16, 0});
    router_.set(zmq::sockopt::maxmsgsize, static_cast<int64_t>(max_payload_bytes));
    router_.set(zmq::sockopt::router_mandatory, 1);
    router_.bind(options.endpoint);
    std::cout << "Managed Gateway ROUTER bind=" << options.endpoint << " gateway_session="
              << (options.session.empty() ? "first-valid" : options.session) << std::endl;
}

void ManagedGateway::receiveInput() {
    for (unsigned i = 0; i < 64; ++i) {
        WireMessage wire;
        Message message;
        std::string error;
        const auto result = receive(sub_, receiving_, wire, error);
        if (result == ReceiveResult::empty) break;
        if (result != ReceiveResult::received || !decode(wire.topic, wire.payload, message, error)) continue;
        if (!input_) {
            auto candidate_policy = policy_;
            candidate_policy.session_id = message.header.session_id;
            InputGuard candidate(candidate_policy);
            if (!candidate.accept(message, monotonic_us(), error)) continue;
            policy_ = std::move(candidate_policy);
            input_ = std::move(candidate);
            std::cout << "Bound flight_gateway session=" << policy_.session_id << std::endl;
        } else if (!input_->accept(message, monotonic_us(), error)) continue;
        effective_sample_ = message.header.sample_mono_us;
        if (message.body.contains("input_state")) {
            bool connected;
            // InputGuard already validated this extension. Preserve the original event timestamp;
            // only the validated device-check timestamp leases a POSITION_HOLD target.
            read_position_hold(message, effective_sample_, connected, error);
        }
        roll_ = message.body.at("control").at("roll").get<double>();
        pitch_ = message.body.at("control").at("pitch").get<double>();
    }
    const bool current = fresh();
    if (current != was_fresh_) {
        std::cout << "Managed flight input: " << (current ? "valid" : "stale/invalid") << std::endl;
        was_fresh_ = current;
    }
}

Json ManagedGateway::request(Aviator& robot, const std::string& session, const Json& req) {
    const auto reply = [&](const std::string& status, const std::string& reason, uint64_t error = 0) {
        return make_service_reply(req, "aviator_core", session, status, error,
            {{"reason", reason}, {"state", robot.GetSystemState()},
             {"expected_gateway_session", policy_.session_id}, {"expected_clock_id", policy_.clock_id},
             {"generation", robot.GetSystemStatus().generation}});
    };
    const auto& params = req.at("parameters");
    if (req.at("client_id") != "flight_gateway") return reply("REJECTED", "UNAUTHORIZED_CLIENT", 1);
    if (req.at("target") != "aviator_core") return reply("REJECTED", "WRONG_TARGET", 1);
    if (req.at("clock_id") != policy_.clock_id) return reply("REJECTED", "CLOCK_DOMAIN_MISMATCH", 1);
    if (!input_) return reply("REJECTED", "GATEWAY_NOT_BOUND", 1);
    if (req.at("client_session_id") != policy_.session_id)
        return reply("REJECTED", "GATEWAY_SESSION_MISMATCH", 1);
    if (!params.contains("server_session_id")) return reply("REJECTED", "CORE_SESSION_REQUIRED", 1);
    if (params.at("server_session_id") != session) return reply("REJECTED", "CORE_SESSION_MISMATCH", 1);
    if (params.size() != 3 || !params.contains("source") || params.at("source") != "JOYSTICK" ||
        !params.contains("server_session_id") || params.at("server_session_id") != session || !params.contains("button") ||
        !params.at("button").is_number_unsigned() || params.at("button").get<uint64_t>() < 1 ||
        params.at("button").get<uint64_t>() > 11 || req.contains("gateway_observation"))
        return reply("REJECTED", "INVALID_PARAMETERS_OR_CORE_SESSION", 1);
    const auto op = req.at("operation").get<std::string>();
    if (!state_operation(op)) return reply("REJECTED", "UNKNOWN_OPERATION", 1);
    const auto now = monotonic_us();
    const auto issued = req.at("issued_mono_us").get<uint64_t>();
    const auto budget = req.at("deadline_ms").get<uint64_t>();
    if (budget > 10000 || issued > now) return reply("REJECTED", "INVALID_DEADLINE", 1);
    const auto key = policy_.session_id + "/" + req.at("request_id").get<std::string>();
    const auto body = req.dump();
    if (auto it = cache_.find(key); it != cache_.end())
        return it->second.body == body ? it->second.reply : reply("REJECTED", "REQUEST_ID_CONFLICT", 1);
    if (now - issued >= budget * 1000) return reply("EXPIRED", "REQUEST_EXPIRED", 1);
    // Never evict an executed identity and accidentally execute it again with altered timestamps.
    if (cache_.size() >= 1024) return reply("REJECTED", "REQUEST_CACHE_FULL", 1);
    auto entry = cache_.emplace(key, Cached{body, reply("UNKNOWN", "DISPATCHING")}).first;
    using Operation = fsm::Reply (Aviator::*)();
    const std::pair<const char*, Operation> operations[] = {
        {"enter_standby", &Aviator::EnterStandby}, {"grasp_wheel", &Aviator::GraspWheel},
        {"start_control", &Aviator::StartControl}, {"exit_control", &Aviator::ExitControl},
        {"leave_wheel", &Aviator::LeaveWheel}, {"reset_error", &Aviator::ResetError}};
    for (const auto& [name, operation] : operations) if (op == name) {
        const auto result = (robot.*operation)();
        const bool accepted = result == fsm::Reply::accepted;
        const bool completed = result == fsm::Reply::completed;
        entry->second.reply = reply(accepted ? "ACCEPTED" : completed ? "COMPLETED" : "REJECTED",
                                    fsm::replyName(result), accepted || completed ? 0 : 1);
        std::cout << "Gateway " << op << " " << fsm::replyName(result)
                  << " " << robot.GetSystemState() << std::endl;
        break;
    }
    return entry->second.reply;
}

void ManagedGateway::receiveRequests(Aviator& robot, const std::string& session) {
    // ROUTER adds exactly one routing frame to the DEALER single-frame envelope. Incremental,
    // bounded draining prevents malformed multipart input from monopolizing the owner heartbeat.
    for (unsigned i = 0; i < 32; ++i) {
        zmq::message_t frame;
        if (!router_.recv(frame, zmq::recv_flags::dontwait)) break;
        if (frames_ == 0) route_ = frame.to_string();
        else if (frames_ == 1) payload_ = frame.to_string();
        frames_ = std::min(3u, frames_ + 1);
        if (frame.more()) continue;
        const bool valid = frames_ == 2;
        frames_ = 0;
        if (!valid) continue;
        try {
            const auto req = decode_service(payload_);
            if (req.at("msg_type") != "ServiceRequest") continue;
            const auto response = request(robot, session, req).dump();
            if (router_.send(zmq::buffer(route_), zmq::send_flags::sndmore | zmq::send_flags::dontwait))
                router_.send(zmq::buffer(response), zmq::send_flags::dontwait);
        } catch (const std::exception& e) {
            std::cerr << "Gateway service rejected/undelivered: " << e.what() << '\n';
        }
    }
}

void ManagedGateway::drive(Aviator& robot) {
    const auto now = monotonic_us();
    if (robot.GetSystemState() != "CONTROL" || !fresh() || effective_sample_ <= forwarded_ || now < next_servo_)
        return;
    if (robot.ServoWheel(roll_ * 0.87266, std::min(pitch_, 0.0) * 0.170, 1.0, effective_sample_)) {
        forwarded_ = effective_sample_;
        next_servo_ = now + 20000;
    }
}
} // namespace aviator

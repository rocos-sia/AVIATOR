#include "RemoteLink.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
namespace aviator {
RemoteLink::RemoteLink(const MotionConfig &c, bool authorize) : config_(c), session_(new_session_id()) {
    const auto info = callService(context_, c, serviceRequest(session_, "describe", Json::object()));
    if (info.at("config_id") != c.config_id)
        throw std::runtime_error("Core/device config_id mismatch");
    server_ = info.at("server_session");
    backend_ = info.at("backend");
    hand_.configure(c.system, backend_);
    std::cout << "Core hand control: " << (hand_.enabled() ? "dedicated ZMQ worker (50 Hz)" : "disabled / simulation") << std::endl;
    state_.q = info.at("q").get<Joints>();
    state_.target = info.at("target").get<Joints>();
    speed_ = info.at("speed").get<Joints>();
    motion_allowed_ = authorize;
    if (authorize) epoch_ = operation("authorize").at("control_epoch");
    heartbeat();
    try {
        hand_.start(context_, config_, session_, heartbeat_, authorize);
        thread_ = std::thread([this] { io(); });
        std::unique_lock<std::mutex> lock(mutex_);
        if (!changed_.wait_for(lock, std::chrono::seconds(5), [&] { return received_ || !error_.empty(); }))
            throw std::runtime_error("No arm.state via bus; start aviator_bus and manipulator");
        if (!error_.empty()) throw std::runtime_error(error_);
    } catch (...) {
        quit_ = true;
        hand_.stop();
        if (thread_.joinable()) thread_.join();
        throw;
    }
}
RemoteLink::~RemoteLink() {
    if (protective_stop_.valid()) protective_stop_.wait();
    quit_ = true;
    hand_.stop();
    if (thread_.joinable()) thread_.join();
}
void RemoteLink::heartbeat() {
    heartbeat_ = monotonic_us();
}
void RemoteLink::report(const std::string &state, const std::string &source) {
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = state;
    source_ = source;
}
void RemoteLink::reportSystem(const Json& body) {
    std::lock_guard<std::mutex> lock(mutex_);
    system_body_ = body;
}
DeviceState RemoteLink::snapshot(bool& fresh, bool* status_fresh) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = monotonic_us();
    fresh = feedback_valid_ && received_ && now >= received_ && now >= sample_ &&
            now - received_ < config_.timeout_us && now - sample_ < config_.timeout_us && error_.empty();
    if (status_fresh) *status_fresh = received_ && status_sample_ && now >= received_ && now >= status_sample_ &&
        now - received_ < config_.timeout_us && now - status_sample_ < config_.timeout_us && error_.empty();
    auto result = state_;
    if (protective_stop_.valid() && protective_stop_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        result.stopping = true;
    if (!error_.empty()) { result.fault = true; result.error = error_; }
    if (hand_.enabled()) {
        const auto reason = hand_.fault();
        if (!reason.empty()) { result.fault = true; result.error = reason; }
    }
    return result;
}
void RemoteLink::allowMotion(bool allowed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (allowed && protective_stop_.valid() &&
        protective_stop_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        throw std::runtime_error("Device protective stop is still pending");
    const bool stop = !allowed && motion_allowed_ && enabled_;
    motion_allowed_ = allowed;
    hand_.allow(allowed);
    if (!allowed) { publishing_ = false; changed_.notify_all(); }
    if (stop) {
        // Keep owner heartbeats and feedback IO alive while the device decelerates.
        // This also covers an idle executor, which has no motion loop to observe cancel.
        protective_stop_ = std::async(std::launch::async, [this] {
            try { stopTrajectory(); }
            catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = std::string("Device protective stop failed: ") + e.what();
                changed_.notify_all();
            }
        });
    }
}
Json RemoteLink::operation(const std::string &op) {
    std::lock_guard<std::mutex> lock(service_mutex_);
    return callService(
        context_, config_,
        serviceRequest(session_, op, {{"server_session", server_}, {"config_id", config_.config_id}}));
}
double RemoteLink::getJointPosition(Side s, int j) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.q.at(int(s) * 7 + j);
}
double RemoteLink::getJointVelocity(Side s, int j) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.dq.at(int(s) * 7 + j);
}
Joints RemoteLink::jointTargets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.target;
}
double RemoteLink::jointVelLimit(Side s, int j) const {
    return speed_.at(int(s) * 7 + j);
}
bool RemoteLink::isEnabled(Side s) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_ && state_.enabled[int(s)];
}
void RemoteLink::enable(Side) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_allowed_) throw std::runtime_error("Core motion authorization revoked");
        if (enabled_)
            return;
    }
    handTarget(false); // Open and confirm actual position before any arm approach.
    auto response = operation("enable");
    std::unique_lock<std::mutex> lock(mutex_);
    if (!motion_allowed_) throw std::runtime_error("Core motion authorization revoked during enable");
    state_.target = response.at("target").get<Joints>();
    state_.fault = false;
    state_.error.clear();
    enabled_ = true;
    trajectory_ = std::make_shared<const std::vector<JointFrame>>(
        3, JointFrame{state_.target, state_.angle, state_.displacement});
    ++trajectory_id_;
    start_ = monotonic_us();
    publishing_ = true;
    if (!changed_.wait_for(lock, std::chrono::milliseconds(500),
                           [&] { return feedback_valid_ && state_.enabled[0] && state_.enabled[1]; }))
        throw std::runtime_error("No fresh dual-arm feedback after enable");
}
void RemoteLink::disable(Side) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
            return;
        publishing_ = false;
    }
    operation("disable");
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
    state_.enabled = {false, false};
    trajectory_.reset();
    streaming_ = false;
    stream_.clear();
}
GraspState RemoteLink::graspState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    GraspState g;
    g.open_loop = true;
    g.heartbeat = double(received_) / 1e6;
    g.locked = state_.locked ? 3 : 0;
    g.ready = enabled_;
    g.fault = state_.fault || !error_.empty();
    g.fault_reason = error_.empty() ? state_.error : error_;
    if (enabled_ && (!feedback_valid_ || monotonic_us() - received_ >= config_.timeout_us ||
                     monotonic_us() - sample_ >= config_.timeout_us)) {
        g.fault = 1;
        g.fault_reason = "arm.state feedback expired";
    }
    if (hand_.enabled()) {
        const auto reason = hand_.fault();
        if (!reason.empty()) { g.fault = 1; g.fault_reason = reason; }
    }
    g.angle = state_.angle;
    g.displacement = state_.displacement;
    g.ack = ack_;
    return g;
}
void RemoteLink::handTarget(bool close) {
    if (!hand_.enabled()) return;
    uint64_t version;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_allowed_) throw std::runtime_error("Core hand authorization revoked");
        if (!error_.empty()) throw std::runtime_error(error_);
        if (close && synchronized_hand_version_) version = synchronized_hand_version_;
        else {
            synchronized_hand_version_ = 0;
            version = hand_.request(close);
        }
    }
    // Do not hold the arm mutex while waiting for physical hand feedback.
    hand_.wait(version);
}

uint64_t RemoteLink::sendGraspCommand(GraspCommand c) {
    if (c == GraspCommand::Lock) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_allowed_) throw std::runtime_error("Core motion authorization revoked before lock");
    }
    // ResetFault used to implicitly unlock the arm backend; open the fingers first too.
    handTarget(c == GraspCommand::Lock);
    try {
        operation(c == GraspCommand::Lock ? "lock" : c == GraspCommand::Unlock ? "unlock" : "reset_fault");
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hand_.enabled()) hand_.fail("Manipulator grasp operation failed; hand watchdog applies safe_pose");
        throw;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (c == GraspCommand::Lock && !motion_allowed_)
        throw std::runtime_error("Core motion authorization revoked during lock");
    state_.locked = c == GraspCommand::Lock;
    if (c == GraspCommand::ResetFault) {
        enabled_ = false;
        publishing_ = false;
        trajectory_.reset();
        streaming_ = false;
        stream_.clear();
        state_.fault = false;
        state_.error.clear();
        error_.clear();
    }
    return ++ack_;
}
void RemoteLink::setJointPositions(const Joints &) {
    throw std::runtime_error("Core must stream a planned trajectory");
}
void RemoteLink::beginStream(const std::vector<JointFrame> &frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_allowed_) throw MotionCancelled();
    if (!enabled_ || state_.fault || !error_.empty() || !hand_.fault().empty() || frames.size() < 61 || frames.size() > 251)
        throw std::runtime_error("Servo requires a healthy device and 60..250 ms prefill");
    for (size_t j = 0; j < 14; ++j)
        if (std::abs(frames.front().q[j] - state_.target[j]) > 1e-7)
            throw std::runtime_error("Servo prefill does not start at last device command");
    stream_.assign(frames.begin(), frames.end());
    stream_first_ = 0;
    streaming_ = true;
    stream_finished_ = false;
    ++trajectory_id_;
    start_ = monotonic_us();
    publishing_ = true;
}
void RemoteLink::appendStream(const std::vector<JointFrame> &frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_allowed_) throw MotionCancelled();
    if (!streaming_ || stream_finished_ || state_.fault || !error_.empty() || !hand_.fault().empty())
        throw std::runtime_error("Servo stream unavailable: " + error_ + state_.error);
    if (frames.size() < 2 || stream_.size() + frames.size() > 251)
        throw std::runtime_error("Servo queue exceeds 250 ms budget");
    for (size_t j = 0; j < 14; ++j)
        if (std::abs(stream_.back().q[j] - frames.front().q[j]) > 1e-9 ||
            std::abs(stream_.back().dq[j] - frames.front().dq[j]) > 1e-8 ||
            std::abs(stream_.back().ddq[j] - frames.front().ddq[j]) > 1e-7)
            throw std::runtime_error("Servo append is not C2 continuous");
    if (std::abs(stream_.back().angle - frames.front().angle) > 1e-9 ||
        std::abs(stream_.back().displacement - frames.front().displacement) > 1e-9)
        throw std::runtime_error("Servo append wheel reference changed");
    stream_.insert(stream_.end(), frames.begin() + 1, frames.end());
}
size_t RemoteLink::streamAhead() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = monotonic_us();
    if (!motion_allowed_) throw MotionCancelled();
    if (!streaming_ || state_.fault || !error_.empty() || !hand_.fault().empty() || !feedback_valid_ ||
        now - received_ >= config_.timeout_us || now - sample_ >= config_.timeout_us)
        throw std::runtime_error("Servo feedback unavailable: receive_age_us=" + std::to_string(now - received_) +
                                 " sample_age_us=" + std::to_string(now - sample_) + " " + error_ + state_.error);
    const uint64_t cursor = state_.id == trajectory_id_ ? state_.cursor : 0;
    if (cursor > stream_first_ + stream_.size() - 1) throw std::runtime_error("Servo cursor beyond queue");
    return stream_first_ + stream_.size() - 1 - cursor;
}
void RemoteLink::finishStream() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!motion_allowed_) throw MotionCancelled();
    if (!streaming_ || stream_.empty()) throw std::runtime_error("Servo stream unavailable");
    for (size_t j = 0; j < 14; ++j)
        if (std::abs(stream_.back().dq[j]) > 1e-8 || std::abs(stream_.back().ddq[j]) > 1e-8)
            throw std::runtime_error("Cannot finish Servo before planned rest");
    stream_finished_ = true;
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (state_.id != trajectory_id_ || state_.cursor < stream_first_ + stream_.size() - 1) {
        if (!motion_allowed_) throw MotionCancelled();
        if (state_.fault || !error_.empty() || !hand_.fault().empty())
            throw std::runtime_error("Servo drain failed: " + error_ + state_.error + hand_.fault());
        if (std::chrono::steady_clock::now() > until) throw std::runtime_error("Servo drain timeout");
        changed_.wait_for(lock, std::chrono::milliseconds(5));
    }
}
void RemoteLink::runTrajectory(const std::vector<JointFrame> &frames, const std::atomic<bool> &cancel) {
    if (frames.empty())
        throw std::runtime_error("Empty trajectory");
    auto data = std::make_shared<std::vector<JointFrame>>(frames);
    if (data->size() == 1)
        data->push_back(data->back());
    if (data->size() % 2 == 0)
        data->push_back(data->back());
    std::unique_lock<std::mutex> lock(mutex_);
    if (!motion_allowed_ || !enabled_ || state_.fault || !hand_.fault().empty())
        throw std::runtime_error("Manipulator is not enabled/healthy");
    const bool synchronized = frames.front().hand_closure[0] >= 0;
    uint64_t hand_version = 0;
    if (hand_.enabled() && synchronized) {
        synchronized_hand_version_ = 0;
        hand_version = hand_.beginApproach(); // Validate close targets before starting arm motion.
        hand_.approachProgress(hand_version, frames.front().hand_closure);
    }
    streaming_ = false;
    stream_.clear();
    trajectory_ = data;
    const auto id = ++trajectory_id_;
    start_ = monotonic_us();
    publishing_ = true;
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(data->size() * 4 + 10000);
    while (true) {
        if (!motion_allowed_) throw std::runtime_error("Core motion authorization revoked");
        if (const auto reason = hand_.fault(); !reason.empty()) throw std::runtime_error(reason);
        if (cancel) {
            lock.unlock();
            stopTrajectory();
            throw std::runtime_error("Motion stopped");
        }
        if (!error_.empty() || state_.fault)
            throw std::runtime_error(error_.empty() ? state_.error : error_);
        const auto now = monotonic_us();
        if (!feedback_valid_ || now - received_ >= config_.timeout_us || now - sample_ >= config_.timeout_us)
            throw std::runtime_error(
                "arm.state feedback timeout: receive_age_us=" + std::to_string(now - received_) +
                " sample_age_us=" + std::to_string(now - sample_));
        if (hand_version && state_.id == id)
            hand_.approachProgress(hand_version, data->at(std::min<uint64_t>(state_.cursor, data->size() - 1)).hand_closure);
        if (state_.id == id && state_.cursor >= data->size() - 1)
            break;
        if (std::chrono::steady_clock::now() > until)
            throw std::runtime_error("Trajectory execution progress timeout");
        changed_.wait_for(lock, std::chrono::milliseconds(5));
    }
    if (hand_version) {
        lock.unlock();
        hand_.wait(hand_version, &cancel); // Endpoint CAN-write ACK; fingers were already closing during approach.
        lock.lock();
        if (!motion_allowed_ || cancel) throw std::runtime_error("Motion stopped during hand completion");
        synchronized_hand_version_ = hand_version;
    }
}
void RemoteLink::stopTrajectory() {
    // Cancellation may also be observed by a trajectory/Servo worker.
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
            return;
        publishing_ = false;
    }
    auto result = operation("stop");
    std::lock_guard<std::mutex> lock(mutex_);
    streaming_ = false;
    stream_.clear();
    state_.target = result.at("target").get<Joints>();
    trajectory_ = std::make_shared<const std::vector<JointFrame>>(
        3, JointFrame{state_.target, state_.angle, state_.displacement});
    ++trajectory_id_;
    start_ = monotonic_us();
    publishing_ = motion_allowed_;
}
void RemoteLink::waitTick() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
std::string RemoteLink::diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return "manipulator: " + state_.error + "; bus: " + error_ + "; hand: " + hand_.fault();
}
void RemoteLink::io() {
    try {
        zmq::socket_t pub(context_, zmq::socket_type::pub), sub(context_, zmq::socket_type::sub);
        configure(pub);
        configure(sub);
        subscribe(sub, "arm.state");
        if (!hand_.enabled()) subscribe(sub, "hand.state");
        pub.connect(config_.publish);
        sub.connect(config_.subscribe);
        ReceiveState receiver;
        uint64_t command_seq = 0, flight_seq = 0, system_seq = 0, last_arm_seq = 0, next = 0, flight_at = 0,
                 system_at = 0;
        while (!quit_) {
            for (int n = 0; n < 64; ++n) {
                WireMessage wire;
                std::string error;
                auto received = receive(sub, receiver, wire, error);
                if (received == ReceiveResult::empty)
                    break;
                Message m;
                if (received != ReceiveResult::received || !decode(wire.topic, wire.payload, m, error))
                    continue;
                if (m.header.publisher_id != "manipulator" || m.header.session_id != server_ ||
                    m.header.clock_id != local_clock_id())
                    continue;
                const auto now = monotonic_us();
                if (m.header.sample_mono_us > now ||
                    (m.header.valid && now - m.header.sample_mono_us >= config_.timeout_us))
                    continue;
                if (m.topic == Topic::hand_state) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    hand_body_ = m.body;
                    continue;
                }
                if (m.topic != Topic::arm_state || m.header.sequence <= last_arm_seq)
                    continue;
                if (m.body.value("config_id", "") != config_.config_id)
                    continue;
                uint64_t status_sample = 0;
                if (m.body.contains("status_mono_us")) {
                    const auto& stamp = m.body.at("status_mono_us");
                    if (!stamp.is_number_unsigned() || stamp.get<uint64_t>() > max_json_integer ||
                        stamp.get<uint64_t>() > now) continue;
                    status_sample = stamp.get<uint64_t>();
                }
                DeviceState state;
                for (int side = 0; side < 2; ++side) {
                    const auto &a = m.body.at("arms").at(side ? "right" : "left");
                    auto q = a.at("joint_position").get<std::array<double, 7>>();
                    std::array<double, 7> dq{};
                    if (!a.at("joint_velocity").is_null())
                        dq = a.at("joint_velocity").get<std::array<double, 7>>();
                    for (int j = 0; j < 7; ++j) {
                        if (!std::isfinite(q[j]) || !std::isfinite(dq[j]))
                            throw std::runtime_error("Nonfinite arm feedback");
                        state.q[7 * side + j] = q[j];
                        state.dq[7 * side + j] = dq[j];
                    }
                    state.enabled[side] = a.at("enabled");
                }
                const auto &e = m.body.at("execution");
                state.id = e.at("trajectory_id");
                state.cursor = e.at("tick");
                state.target = e.at("target").get<Joints>();
                state.fault = e.at("fault");
                state.error = e.at("error");
                state.stopping = e.at("stopping");
                state.locked = m.body.at("software_lock");
                state.angle = m.body.at("wheel_reference").at("angle");
                state.displacement = m.body.at("wheel_reference").at("displacement");
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = state;
                feedback_valid_ = m.header.valid;
                arm_body_ = m.body;
                received_ = now;
                sample_ = m.header.sample_mono_us;
                status_sample_ = status_sample;
                last_arm_seq = m.header.sequence;
                changed_.notify_all();
            }
            const auto now = monotonic_us();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto reason = hand_.fault();
                if (hand_.enabled() && !reason.empty()) {
                    // Let the independent arm watchdog stop motion; never stream through hand loss.
                    publishing_ = false;
                    changed_.notify_all();
                }
            }
            if (now >= next) {
                next = next && now - next < config_.period_us ? next + config_.period_us
                                                              : now + config_.period_us;
                TrajectoryWindow w;
                bool send_window = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (publishing_ && streaming_) {
                        const uint64_t cursor = state_.id == trajectory_id_ ? state_.cursor : 0;
                        while (stream_.size() > 2 && stream_first_ + 4 < cursor) {
                            stream_.pop_front();
                            ++stream_first_;
                        }
                        w.streaming = true;
                        w.finished = stream_finished_;
                        w.id = trajectory_id_;
                        w.first = stream_first_;
                        w.total = stream_first_ + stream_.size() - 1;
                        // 80 ms span leaves room for history/feedback lag before the 50 ms watchdog.
                        w.count = std::min<size_t>(servo_window_points, stream_.size());
                        for (size_t k = 0; k < w.count; ++k) w.frames[k] = stream_[k];
                        w.sequence = ++command_seq;
                        w.origin_sample = heartbeat_;
                        w.sample = monotonic_us();
                        w.start = start_ + w.first * 1000;
                        send_window = true;
                    } else if (publishing_ && trajectory_) {
                        w.id = trajectory_id_;
                        w.total = trajectory_->size() - 1;
                        const uint64_t cursor = state_.id == w.id ? std::min(state_.cursor, w.total) : 0;
                        w.first = cursor > 4 ? ((cursor - 4) / 2) * 2 : 0;
                        w.count = std::min<uint64_t>(63, trajectory_->size() - w.first);
                        if (w.count < 3) {
                            w.first = w.first >= 2 ? w.first - 2 : 0;
                            w.count = 3;
                        }
                        for (size_t k = 0; k < w.count; ++k)
                            w.frames[k] = trajectory_->at(std::min<uint64_t>(w.first + k, w.total));
                        for (size_t k = 1; k < w.count; k += 2) {
                            for (size_t j = 0; j < 14; ++j)
                                w.frames[k].q[j] = (w.frames[k - 1].q[j] + w.frames[k + 1].q[j]) * .5;
                        }
                        w.sequence = ++command_seq;
                        w.origin_sample = heartbeat_;
                        w.sample = monotonic_us();
                        w.start = start_ + w.first * 1000;
                        send_window = true;
                    }
                }
                if (send_window) {
                    // Serialize the final publication with cancellation. Once allowMotion(false)
                    // returns, no previously prepared ordinary window can be published.
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!motion_allowed_ || !hand_.fault().empty()) continue;
                    const auto publish_time = monotonic_us();
                    auto m = motionMessage(Topic::arm_command, "aviator_core", session_, w.sequence,
                                           publish_time >= w.origin_sample &&
                                               publish_time - w.origin_sample < config_.origin_timeout_us);
                    m.header.sample_mono_us = w.sample;
                    m.body = encodeWindow(w, session_, epoch_);
                    m.body["config_id"] = config_.config_id;
                    publishMessage(pub, m);
                }
            }
            if (now >= flight_at) {
                flight_at = now + 20000;
                auto m = motionMessage(Topic::flight_state, "aviator_core", session_, ++flight_seq);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const bool fresh = feedback_valid_ && received_ && now - received_ < config_.timeout_us &&
                                       now - sample_ < config_.timeout_us;
                    m.header.valid = fresh && !state_.fault;
                    std::string phase =
                        state_.fault
                            ? "ERROR"
                            : (phase_ == "APPROACHING"
                                   ? "GRASPING"
                                   : ((phase_ == "MOVING" || phase_ == "SERVO") ? "CONTROL" : "STANDBY"));
                    m.body["system"] = {{"state", phase},
                                        {"control_source", source_},
                                        {"current_error_code", state_.fault ? 28673 : 0},
                                        {"last_error_code", state_.fault ? 28673 : 0},
                                        {"task_phase", phase_}};
                    if (!system_body_.is_null()) {
                        m.body["system"] = system_body_;
                        // Managed status can be trustworthy while disabled Rokae RT samples are
                        // unavailable. Arm sample usability is still reported in freshness.arm.
                        const auto owner_at = heartbeat_.load();
                        const auto status_now = monotonic_us(); // Read after the concurrent owner heartbeat.
                        m.header.valid = received_ && status_sample_ && owner_at && error_.empty() &&
                            status_now >= received_ && status_now >= status_sample_ && status_now >= owner_at &&
                            status_now - received_ < config_.timeout_us && status_now - status_sample_ < config_.timeout_us &&
                            status_now - owner_at < 100000;
                    }
                    m.body["arms"] = arm_body_.value("arms", Json::object());
                    m.body["hands"] = hand_body_.value("hands", Json::object());
                    m.body["vision"] = {
                        {"status", "OFFLINE"},
                        {"confidence", 0},
                        {"yoke", {{"detected", false}, {"roll", nullptr}, {"pitch", nullptr}}}};
                    m.body["freshness"]["arm"] = {
                        {"valid", fresh},
                        {"age_ms", sample_ ? Json(double(now - sample_) / 1000) : Json(nullptr)},
                        {"sequence", last_arm_seq}};
                    for (const char *group : {"hand", "camera"})
                        m.body["freshness"][group] = {
                            {"valid", false}, {"age_ms", nullptr}, {"sequence", nullptr}};
                    if (hand_.enabled()) {
                        const auto hand = hand_.status(); // One coherent hand snapshot, timed under its own lock.
                        m.body["hands"] = hand.body.value("hands", Json::object());
                        m.body["freshness"]["hand"] = {
                            {"valid", hand.fresh}, {"age_ms", hand.age_ms}, {"sequence", hand.sequence}};
                        m.body["hand_control"] = {{"enabled", true}, {"error", hand.error},
                                                   {"target_version", hand.target_version}};
                        if (!hand.error.empty() && system_body_.is_null()) m.header.valid = false;
                    }
                    m.body["software_lock"] = state_.locked;
                    m.body["wheel_reference"] = {{"angle", state_.angle},
                                                 {"displacement", state_.displacement}};
                }
                publishMessage(pub, m);
            }
            if (now >= system_at) {
                system_at = now + 200000;
                auto m = motionMessage(Topic::system_state, "aviator_core", session_, ++system_seq);
                m.body = {
                    {"node", "aviator_core"}, {"lifecycle", "RUNNING"}, {"ready", true}, {"uptime_ms", 0}};
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!system_body_.is_null()) {
                        m.body["system"] = system_body_;
                        m.body["ready"] = system_body_.value("ready", false);
                    }
                }
                publishMessage(pub, m);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = e.what();
        if (hand_.enabled()) hand_.fail("Core bus IO failed: " + error_);
        changed_.notify_all();
    }
}
} // namespace aviator

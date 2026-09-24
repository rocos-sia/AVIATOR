#include "rokae_sdk.hpp"
#include <rokae/robot.h>
#include <stdexcept>
#include <vector>
namespace aviator {
namespace {
void check(const std::error_code &ec, const char *action) {
    if (ec) throw std::runtime_error(std::string(action) + ": " + ec.message());
}
}
class RokaeArm::Impl {
public:
    rokae::xMateErProRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt;
    std::array<double, 7> stiffness{};
    bool powered = false, receiving = false, moving = false, looping = false;
    void stop() {
        std::exception_ptr error;
        auto attempt = [&](auto fn) { try { fn(); } catch (...) { if (!error) error = std::current_exception(); } };
        if (looping) attempt([&] { rt->stopLoop(); looping = false; });
        if (moving) attempt([&] { rt->stopMove(); moving = false; });
        if (receiving) { robot.stopReceiveRobotState(); receiving = false; }
        if (powered) attempt([&] {
            std::error_code ec;
            robot.setPowerState(false, ec);
            check(ec, "setPowerState(false)");
            powered = false;
        });
        if (error) std::rethrow_exception(error);
    }
    ~Impl() {
        try { stop(); } catch (...) {}
        std::error_code ec;
        robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
        robot.disconnectFromRobot(ec);
    }
};
RokaeArm::RokaeArm(const std::string &ip, const std::string &local_ip,
                   const std::array<double, 16> &tool, const std::array<double, 7> &stiffness) : impl_(std::make_unique<Impl>()) {
    auto &s = *impl_;
    s.stiffness = stiffness;
    s.robot.connectToRobot(ip, local_ip);
    std::error_code ec;
    s.robot.setOperateMode(rokae::OperateMode::automatic, ec);
    check(ec, "setOperateMode");
    s.robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    check(ec, "setMotionControlMode");
    s.robot.setRtNetworkTolerance(20, ec);
    check(ec, "setRtNetworkTolerance");
    auto toolset = s.robot.toolset(ec);
    check(ec, "toolset");
    toolset.end = rokae::Frame(tool);
    toolset.ref = rokae::Frame();
    // Retain the controller's calibrated load; grasp.json has no load calibration.
    s.robot.setToolset(toolset, ec);
    check(ec, "setToolset");
    s.rt = s.robot.getRtMotionController().lock();
    if (!s.rt) throw std::runtime_error("getRtMotionController returned null");
}
RokaeArm::~RokaeArm() = default;
std::array<double, 7> RokaeArm::position() const {
    std::error_code ec;
    auto q = impl_->robot.jointPos(ec);
    check(ec, "jointPos");
    return q;
}
void RokaeArm::start(std::function<std::array<double, 7>(const RokaeSample &)> callback) {
    auto &s = *impl_;
    try {
        std::error_code ec;
        s.robot.setPowerState(true, ec);
        check(ec, "setPowerState(true)");
        s.powered = true;
        s.receiving = true;
        s.robot.startReceiveRobotState(std::chrono::milliseconds(1),
            {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
             rokae::RtSupportedFields::tcpPose_m});
        s.rt->setControlLoop(std::function<rokae::JointPosition()>([&s, callback] {
            RokaeSample sample;
            if (s.robot.getStateData(rokae::RtSupportedFields::jointPos_m, sample.position) != 0 ||
                s.robot.getStateData(rokae::RtSupportedFields::jointVel_m, sample.velocity) != 0 ||
                s.robot.getStateData(rokae::RtSupportedFields::tcpPose_m, sample.tcp) != 0)
                throw std::runtime_error("Incomplete xCore realtime state");
            auto q = callback(sample);
            return rokae::JointPosition(std::vector<double>(q.begin(), q.end()));
        }), 0, true);
        // One mode for approach, wheel paths, streaming and holding; no position-mode pre-move.
        s.rt->setJointImpedance(s.stiffness, ec);
        check(ec, "setJointImpedance");
        s.moving = true;
        s.rt->startMove(rokae::RtControllerMode::jointImpedance);
        s.looping = true;
        s.rt->startLoop(false);
    } catch (...) {
        auto error = std::current_exception();
        try { s.stop(); } catch (...) {}
        std::rethrow_exception(error);
    }
}
void RokaeArm::stop() { impl_->stop(); }
}

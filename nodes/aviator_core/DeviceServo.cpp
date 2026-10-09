#include "DeviceServo.hpp"
#include "ServoPlanner.hpp"
#include "aviator/GraspTools.hpp"
#include "aviator/backend.hpp"
#include <cmath>

namespace aviator {
namespace {
void require(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
pinocchio::SE3 frame(const YAML::Node& n) {
    auto p = n["position"].as<std::array<double, 3>>();
    auto q = n["quaternion"].as<std::array<double, 4>>();
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    require(std::isfinite(rotation.norm()) && std::abs(rotation.norm() - 1) < 1e-6,
            "Invalid servo frame");
    for (double v : p)
        require(std::isfinite(v), "Invalid servo translation");
    return {rotation, Eigen::Vector3d(p[0], p[1], p[2])};
}
struct Resources {
    std::unique_ptr<Kinematics> kinematics;
    std::unique_ptr<CollisionChecker> collision;
    pinocchio::SE3 origin, handles[2], tools[2];
    std::array<double, 2> speed, acceleration, jerk;
    explicit Resources(const DeviceSettings& settings) {
        const auto posture = YAML::LoadFile(settings.path("posture").string());
        const double margin = posture["joint2_planning_margin_deg"].as<double>(.5) * M_PI / 180;
        const double lo = posture["joint2_limits_deg"][0].as<double>() * M_PI / 180 + margin;
        const double hi = posture["joint2_limits_deg"][1].as<double>() * M_PI / 180 - margin;
        require(std::isfinite(lo) && std::isfinite(hi) && lo < hi, "Invalid servo J2 limits");
        kinematics = makePinIkKinematics(settings.path("urdf").string(), lo, hi);
        if (settings.robot["collision_check_enabled"].as<bool>(true))
            collision = makePinocchioCollisionChecker(settings.path("urdf").string());
        auto grasp = YAML::LoadFile(settings.path("grasp").string());
        origin = frame(grasp["wheel_origin"]);
        for (int side = 0; side < 2; ++side) {
            handles[side] = frame(grasp[side ? "right" : "left"]);
            tools[side] = frame(toolFrameConfig(grasp, side));
        }
        speed = {settings.robot["wheel_angular_speed"].as<double>(.4),
                 settings.robot["wheel_linear_speed"].as<double>(.08)};
        acceleration = {settings.robot["wheel_angular_acceleration"].as<double>(.05),
                        settings.robot["wheel_linear_acceleration"].as<double>(.005)};
        jerk = {settings.robot["wheel_angular_jerk"].as<double>(.05),
                settings.robot["wheel_linear_jerk"].as<double>(.005)};
        for (const auto& values : {speed, acceleration, jerk})
            for (double v : values)
                require(std::isfinite(v) && v > 0, "Invalid servo constraints");
    }
    pinocchio::SE3 target(int side, double a, double d) const {
        return origin *
               pinocchio::SE3(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                              Eigen::Vector3d(0, 0, d)) *
               handles[side] * tools[side].inverse();
    }
    void check(const JointFrame& f) {
        for (int j = 0; j < 14; ++j)
            require(std::isfinite(f.q[j]) && f.q[j] >= kinematics->jointLower(Side(j / 7), j % 7) &&
                        f.q[j] <= kinematics->jointUpper(Side(j / 7), j % 7),
                    "Device servo joint limit");
        if (collision)
            collision->check(f.q, f.angle, f.displacement);
    }
};
class OnlineServo final : public DeviceServo {
    std::shared_ptr<Resources> resources_;
    ServoPlanner planner_;

  public:
    OnlineServo(std::shared_ptr<Resources> r, const JointFrame& start)
        : resources_(r),
          planner_(
              *r->kinematics, [r](int side, double a, double d) { return r->target(side, a, d); },
              [r](const JointFrame& f) { r->check(f); }, r->origin, start, .001, r->speed,
              r->acceleration, r->jerk) {}
    JointFrame step(const ServoGoal& g) override {
        // Two samples: current and next. No future trajectory queue is retained.
        return planner_.advance(g.angle, g.displacement, g.speed_ratio, g.stop).back();
    }
    bool stopped() const override { return planner_.stopped(); }
};
} // namespace
void configureDeviceServo(DeviceServerOptions& options, const DeviceSettings& settings) {
    auto resources = std::make_shared<Resources>(settings);
    options.servo = [resources](const JointFrame& start) {
        return std::make_unique<OnlineServo>(resources, start);
    };
}
} // namespace aviator

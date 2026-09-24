// MoveWheel 专项测试：速度倍率、实际下发速度、关节限速自动延时。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace aviator;

static void check(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

// 只为逐周期测量下发速度保留这个记录器；控制流程直接写在 main 中。
// Record commands at the backend boundary, not the planner's computed durations.
struct RecordingLink final : DataLink {
    std::unique_ptr<DataLink> io;
    std::array<double, 14> previous{};
    double last_time = 0, last_angle = 0, last_displacement = 0;
    double peak_joint = 0, peak_angular = 0, peak_linear = 0;
    double speed_cap = std::numeric_limits<double>::infinity();
    bool record = false;
    double tick_time = 0;
    explicit RecordingLink(std::unique_ptr<DataLink> link) : io(std::move(link)) {}
    double getJointPosition(Side s, int j) const override { return io->getJointPosition(s,j); }
    double getJointVelocity(Side s, int j) const override { return io->getJointVelocity(s,j); }
    double jointVelLimit(Side s, int j) const override { return std::min(speed_cap, io->jointVelLimit(s,j)); }
    bool isEnabled(Side s) const override { return io->isEnabled(s); }
    void enable(Side s) override { io->enable(s); }
    void disable(Side s) override { io->disable(s); }
    GraspState graspState() const override { return io->graspState(); }
    uint64_t sendGraspCommand(GraspCommand c) override { return io->sendGraspCommand(c); }
    // Freeze one clock sample per control tick so physics advancing concurrently cannot
    // skew the reference/command timestamps used for finite-difference velocity checks.
    void waitTick() override { io->waitTick(); tick_time = io->time(); }
    double time() const override { return tick_time; }
    void setRealTime(bool value) override { io->setRealTime(value); }
    std::mutex *physicsMutex() override { return io->physicsMutex(); }
    void setWheelReference(double a, double d) override {
        const double dt = time() - last_time;
        if (record && dt > 0) {
            peak_angular = std::max(peak_angular, std::abs(a-last_angle)/dt);
            peak_linear = std::max(peak_linear, std::abs(d-last_displacement)/dt);
        }
        last_angle = a; last_displacement = d;
        io->setWheelReference(a,d);
    }
    void setJointPositions(const std::array<double,14> &q) override {
        const double dt = time() - last_time;
        if (record && dt > 0) {
            for (int i = 0; i < 14; ++i)
                peak_joint = std::max(peak_joint, std::abs(q[i]-previous[i])/dt);
        }
        previous = q; last_time = time();
        io->setJointPositions(q);
    }
    void begin(const Status &start) {
        peak_joint = peak_angular = peak_linear = 0;
        last_angle = start.angle; last_displacement = start.displacement;
        for (int i = 0; i < 14; ++i) previous[i] = getJointPosition(static_cast<Side>(i/7),i%7);
        last_time = time(); record = true;
    }
};

int main() {
    try {
        char error[1000]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        check(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int home = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
        check(data && home >= 0, "Missing simulation data or aviator_home");
        mj_resetDataKeyframe(model.get(), data.get(), home);
        mj_forward(model.get(), data.get());
        auto link = std::make_unique<RecordingLink>(makeMuJoCoDirectDataLink(
            model.get(), data.get(), AVIATOR_CONFIG_DIR "/aviator_control.urdf"));
        auto *io = link.get();
        Aviator robot(std::move(link), nullptr, nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.SetRealTime(false);
        robot.Enable();
        robot.ApproachHandles();
        robot.LockHandles();

        for (double v : {0., -0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
            bool rejected = false;
            try { robot.MoveWheel(0, 0, v); } catch (const std::exception &) { rejected = true; }
            check(rejected, "Invalid speed accepted");
        }
        double duration[2]{};
        for (int run = 0; run < 2; ++run) {
            const double v = run == 0 ? 1. : 0.5;
            io->begin(robot.GetStatus());
            const double start = io->time();
            robot.MoveWheel(0.1,-0.01,v);
            duration[run] = io->time()-start-1.5; // omit configured final dwell
            io->record = false;
            std::cout << "v=" << v << " duration=" << duration[run] << " joint_rad_s=" << io->peak_joint
                      << " wheel_rad_s=" << io->peak_angular << " wheel_m_s=" << io->peak_linear << std::endl;
            check(io->peak_joint <= 1.9*v+0.002, "joint speed exceeded v-scaled limit");
            check(io->peak_angular <= 0.4*v+0.0002, "angular speed exceeded v-scaled limit");
            check(io->peak_linear <= 0.08*v+0.0002, "linear speed exceeded v-scaled limit");
            const auto s = robot.GetStatus();
            check(std::abs(s.angle-0.1)<1e-9 && std::abs(s.displacement+0.01)<1e-9, "MoveWheel final target wrong");
            robot.MoveWheel(0,0,1);
        }
        check(duration[1]/duration[0] > 1.9 && duration[1]/duration[0] < 2.1, "half speed did not double motion time");
        // Force a joint to determine the trajectory duration instead of the wheel limit.
        io->speed_cap = 0.08;
        for (double v : {1., 0.5}) {
            io->begin(robot.GetStatus());
            const double start = io->time();
            robot.MoveWheel(0.1,-0.01,v);
            io->record = false;
            std::cout << "joint-limited v=" << v << " duration=" << io->time()-start-1.5
                      << " peak_joint=" << io->peak_joint << std::endl;
            check(io->peak_joint <= io->speed_cap*v+0.0002, "joint-driven timing failed");
            check(io->time()-start-1.5 > 0.5/v, "joint constraint did not extend duration");
            robot.MoveWheel(0,0,1);
        }

        robot.UnlockHandles();
        robot.Disable();
        std::cout << "MoveWheel speed ratio and velocity limits passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

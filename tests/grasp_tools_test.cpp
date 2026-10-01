#include "aviator/Aviator.hpp"
#include "aviator/GraspTools.hpp"
#include "aviator/backend.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#ifdef AVIATOR_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif

using namespace aviator;
namespace fs = std::filesystem;
void check(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
pinocchio::SE3 frame(const YAML::Node &n) {
    auto p = n["position"].as<std::vector<double>>(), q = n["quaternion"].as<std::vector<double>>();
    return {Eigen::Quaterniond(q[0], q[1], q[2], q[3]), Eigen::Vector3d(p[0], p[1], p[2])};
}
void samePose(const pinocchio::SE3 &a, const pinocchio::SE3 &b) {
    check((a.translation() - b.translation()).norm() < 1e-10 && rotationError(a, b) < 1e-10,
          "Wrong per-arm tool pose");
}
struct RecordingIk : Kinematics {
    std::vector<pinocchio::SE3> targets[2];
    bool solveIk(Side side, const std::array<double, 7> &seed, const pinocchio::SE3 &pose,
                 std::array<double, 7> &out) override {
        targets[int(side)].push_back(pose); out = seed; return true;
    }
    bool solveFk(Side side, const std::array<double, 7> &, pinocchio::SE3 &out) override {
        out = targets[int(side)].empty() ? pinocchio::SE3::Identity() : targets[int(side)].back(); return true;
    }
    double jointLower(Side, int) const override { return -10; }
    double jointUpper(Side, int) const override { return 10; }
};
struct Device : DataLink {
    std::array<double, 14> q{};
    bool enabled[2]{};
    double now = 0, displacement = 0;
    std::vector<std::vector<JointFrame>> trajectories;
    std::vector<GraspCommand> commands;
    double getJointPosition(Side s, int j) const override { return q[int(s)*7+j]; }
    double getJointVelocity(Side, int) const override { return 0; }
    void setJointPositions(const std::array<double, 14> &v) override { q = v; }
    std::array<double, 14> jointTargets() const override { return q; }
    double jointVelLimit(Side, int) const override { return 10; }
    bool isEnabled(Side s) const override { return enabled[int(s)]; }
    void enable(Side s) override { enabled[int(s)] = true; }
    void disable(Side s) override { enabled[int(s)] = false; }
    GraspState graspState() const override { GraspState g; g.heartbeat = now; g.displacement = displacement; return g; }
    uint64_t sendGraspCommand(GraspCommand c) override { commands.push_back(c); return 0; }
    void waitTick() override { now += .001; }
    double time() const override { return now; }
    void runTrajectory(const std::vector<JointFrame> &v, const std::atomic<bool> &) override {
        trajectories.push_back(v);
        q = v.back().q; now += v.size() * .001;
    }
};
int main(int argc, char **argv) try {
    check(argc == 2, "Expected project root");
    const fs::path root = argv[1];
    const auto dir = fs::temp_directory_path() / ("grasp-tools-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(dir);
    struct Cleanup { fs::path p; ~Cleanup() { fs::remove_all(p); } } cleanup{dir};
    auto grasp = YAML::LoadFile((root / "config/grasp.json").string());
    const auto shared = YAML::Clone(toolFrameConfig(grasp, 0));
    auto legacy = YAML::Clone(grasp);
    legacy["tool"] = shared;
    samePose(frame(toolFrameConfig(legacy, 0)), frame(toolFrameConfig(legacy, 1)));
    auto split = YAML::Clone(grasp);
    split["tool"].remove("position"); split["tool"].remove("quaternion");
    split["tool"]["left"] = YAML::Clone(shared);
    split["tool"]["right"] = YAML::Clone(shared);
    split["tool"]["left"]["position"] = std::vector<double>{.01, -.02, .14};
    split["tool"]["right"]["position"] = std::vector<double>{-.03, .06, .11};
    split["tool"]["right"]["quaternion"] = std::vector<double>{1, 0, 0, 0};
    for (bool mixed : {false, true}) {
        auto invalid = YAML::Clone(split);
        if (mixed) invalid["tool"]["position"] = std::vector<double>{0, 0, 0};
        else invalid["tool"].remove("right");
        bool rejected = false;
        try { toolFrameConfig(invalid, 0); } catch (const std::runtime_error &) { rejected = true; }
        check(rejected, "Ambiguous or incomplete tools accepted");
    }
    GraspGeometry geometry;
    geometry.wheel_origin = frame(split["wheel_origin"]);
    for (int s = 0; s < 2; ++s) {
        geometry.tools[s] = frame(toolFrameConfig(split, s));
        geometry.handles[s] = frame(split[s ? "right" : "left"]);
    }
    auto config = YAML::LoadFile((root / "config/robot.yaml").string());
    config["posture"] = (root / "config/posture.json").string();
    config["grasp"] = (dir / "grasp.yaml").string();
    config["collision_check_enabled"] = false;
    auto runCore = [&](const YAML::Node &g) {
        std::ofstream(dir / "grasp.yaml") << g;
        std::ofstream(dir / "robot.yaml") << config;
        auto device = std::make_unique<Device>();
        device->q[1] = device->q[8] = M_PI / 2;
        auto *executed = device.get();
        auto ik = std::make_unique<RecordingIk>();
        auto *record = ik.get();
        Aviator core(std::move(device), std::move(ik), nullptr, (dir / "robot.yaml").string());
        core.init(); core.enable(); core.approachHandles();
        for (int s = 0; s < 2; ++s) {
            const auto expected = frame(g["wheel_origin"]) * frame(g[s ? "right" : "left"]) *
                                  frame(toolFrameConfig(g, s)).inverse();
            check(record->targets[s].size() == 1, "Approach retained an intermediate IK target");
            samePose(record->targets[s].back(), expected);
            const auto& trajectory = executed->trajectories.back();
            check(trajectory.front().hand_closure[s] == 0 && trajectory.back().hand_closure[s] == 1,
                  "Hand endpoints not synchronized with approach");
        }
        const auto before_disable = executed->commands.size();
        core.disable();
        check(executed->commands.size() == before_disable + 1 && executed->commands.back() == GraspCommand::Unlock,
              "Disable after approach did not open unlatched hands");
    };
    // Independent thresholds, quintic endpoints, configurable range and continuous progress.
    for (double threshold : {.07, .04}) {
        std::vector<JointFrame> frames(101);
        for (size_t k = 0; k < frames.size(); ++k) frames[k].q[0] = double(k) / 100;
        planHandClosure(frames, threshold, [](int side, const JointFrame& f) {
            return (side ? .1 : .2) * (1 - f.q[0]);
        });
        for (int side = 0; side < 2; ++side) {
            bool partial = false;
            for (size_t k = 0; k < frames.size(); ++k) {
                const auto progress = frames[k].hand_closure[side];
                if ((side ? .1 : .2) * (1 - frames[k].q[0]) > threshold)
                    check(progress == 0, "Hand closed outside configured tool distance");
                if (k) check(progress >= frames[k-1].hand_closure[side], "Hand progress reversed");
                partial |= progress > 0 && progress < 1;
            }
            check(partial && frames.back().hand_closure[side] == 1, "Missing smooth hand trajectory");
        }
        check(frames[70].hand_closure[1] > frames[70].hand_closure[0], "Hands shared a trigger");
    }
    for (double bad : {0., -.01, std::numeric_limits<double>::infinity()}) {
        bool rejected = false;
        std::vector<JointFrame> frames(2);
        try { planHandClosure(frames, bad, [](int, const JointFrame&) { return 0.; }); }
        catch (const std::runtime_error&) { rejected = true; }
        check(rejected, "Invalid closing distance accepted");
    }
    runCore(split); runCore(legacy);
    // Real deployment geometry + Pinocchio FK/IK, without connecting any device.
    const auto posture = YAML::LoadFile((root / "config/posture.json").string());
    for (double threshold : {.07, .04}) {
        auto actual = YAML::Clone(grasp);
        actual["hand_closing_distance"] = threshold;
        std::ofstream(dir / "grasp.yaml") << actual;
        config["urdf"] = (root / "models/urdf/aviator.urdf").string();
        std::ofstream(dir / "robot.yaml") << config;
        auto device = std::make_unique<Device>();
        device->displacement = -.085;
        for (int side = 0; side < 2; ++side) {
            const auto home = posture[side ? "right_home_deg" : "left_home_deg"].as<std::vector<double>>();
            for (int j = 0; j < 7; ++j) device->q[side*7+j] = home[j]*M_PI/180;
        }
        auto *executed = device.get();
        const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        auto ik = makePinIkKinematics(config["urdf"].as<std::string>(), limits[0]*M_PI/180, limits[1]*M_PI/180);
        auto *fk = ik.get();
        Aviator core(std::move(device), std::move(ik), nullptr, (dir / "robot.yaml").string());
        core.init(); core.enable(); core.approachHandles();
        check(executed->trajectories.size() == 2, "Expected home and one continuous approach only");
        const auto& path = executed->trajectories.back();
        const auto wheel = frame(actual["wheel_origin"]) *
            pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,-.085));
        for (int side = 0; side < 2; ++side) {
            bool partial = false;
            for (size_t k = 0; k < path.size(); ++k) {
                std::array<double,7> q{};
                std::copy_n(path[k].q.begin()+side*7, 7, q.begin());
                pinocchio::SE3 flange;
                check(fk->solveFk(static_cast<Side>(side), q, flange), "Real FK failed");
                const auto distance = ((flange * frame(toolFrameConfig(actual, side))).translation() -
                                      (wheel * frame(actual[side ? "right" : "left"])).translation()).norm();
                if (distance > threshold) check(path[k].hand_closure[side] == 0, "Real tool closed outside range");
                partial |= path[k].hand_closure[side] > 0 && path[k].hand_closure[side] < 1;
                if (k + 1 == path.size()) check(distance < 1e-6, "Real tool did not reach grasp target");
            }
            check(partial && path.front().hand_closure[side] == 0 && path.back().hand_closure[side] == 1,
                  "Real arm/hand trajectory endpoints incorrect");
        }
        core.unlockHandles();
        check(core.GetState() == "ENABLED", "Unlatched approached hands could not be opened");
        core.disable();
    }
    // Geometry is configured before device threads start; exercise the actual collision loader.
    const auto collision = root / "models/control/aviator_collision.urdf";
    makePinocchioCollisionChecker(collision.string(), (root / "models/control/aviator_collision.srdf").string(),
                                 GraspCylinder{}, geometry.tools);
#ifdef AVIATOR_HAVE_MUJOCO
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML((root / "models/control/aviator.xml").c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    check(bool(model), error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "aviator_home"));
    const int wheel = mj_name2id(model.get(), mjOBJ_JOINT, "pitch_input_joint");
    data->qpos[model->jnt_qposadr[wheel]] = -.085;
    const std::vector<mjtNum> initial(data->qpos, data->qpos + model->nq);
    auto device = makeDataLink("mujoco", {model.get(), data.get()},
                               (root / "models/urdf/aviator.urdf").string(), geometry);
    device->setRealTime(false);
    std::lock_guard<std::mutex> lock(*device->physicsMutex());
    // Physics may have advanced a tick, so restore the known test posture under its lock.
    for (int i = 0; i < model->nq; ++i)
        check(std::abs(data->qpos[i] - initial[i]) < .01, "Tool setup reset home or wheel_initial");
    std::copy(initial.begin(), initial.end(), data->qpos);
    mj_forward(model.get(), data.get());
    for (int s = 0; s < 2; ++s) {
        const std::string name = s ? "right" : "left";
        const int tcp = mj_name2id(model.get(), mjOBJ_SITE, (name + "_tcp").c_str());
        const int flange = mj_name2id(model.get(), mjOBJ_BODY,
            (std::string("AR5-5_07") + (s ? "R" : "L") + "-W4C4A2_flan_link").c_str());
        Eigen::Matrix3d r, rt;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
            r(i,j) = data->xmat[9*flange+3*i+j]; rt(i,j) = data->site_xmat[9*tcp+3*i+j];
        }
        samePose({rt, Eigen::Map<Eigen::Vector3d>(data->site_xpos+3*tcp)},
                 pinocchio::SE3(r, Eigen::Map<Eigen::Vector3d>(data->xpos+3*flange)) * geometry.tools[s]);
    }
#endif
    std::cout << "PASS independent tool positions/rotations, approach, legacy config, collision and simulation TCP\n";
    return 0;
} catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }

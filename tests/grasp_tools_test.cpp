#include "aviator/Aviator.hpp"
#include "aviator/GraspTools.hpp"
#include "aviator/backend.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
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
    bool solveFk(Side, const std::array<double, 7> &, pinocchio::SE3 &out) override {
        out = pinocchio::SE3::Identity(); return true;
    }
    double jointLower(Side, int) const override { return -10; }
    double jointUpper(Side, int) const override { return 10; }
};
struct Device : DataLink {
    std::array<double, 14> q{};
    bool enabled[2]{};
    double now = 0;
    double getJointPosition(Side s, int j) const override { return q[int(s)*7+j]; }
    double getJointVelocity(Side, int) const override { return 0; }
    void setJointPositions(const std::array<double, 14> &v) override { q = v; }
    std::array<double, 14> jointTargets() const override { return q; }
    double jointVelLimit(Side, int) const override { return 10; }
    bool isEnabled(Side s) const override { return enabled[int(s)]; }
    void enable(Side s) override { enabled[int(s)] = true; }
    void disable(Side s) override { enabled[int(s)] = false; }
    GraspState graspState() const override { GraspState g; g.heartbeat = now; return g; }
    uint64_t sendGraspCommand(GraspCommand) override { return 0; }
    void waitTick() override { now += .001; }
    double time() const override { return now; }
    void runTrajectory(const std::vector<JointFrame> &v, const std::atomic<bool> &) override {
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
        auto ik = std::make_unique<RecordingIk>();
        auto *record = ik.get();
        Aviator core(std::move(device), std::move(ik), nullptr, (dir / "robot.yaml").string());
        core.init(); core.enable(); core.approachHandles();
        for (int s = 0; s < 2; ++s) {
            const auto expected = frame(g["wheel_origin"]) * frame(g[s ? "right" : "left"]) *
                                  frame(toolFrameConfig(g, s)).inverse();
            check(record->targets[s].size() > 1, "Approach did not call IK");
            samePose(record->targets[s].back(), expected);
            auto retreat = expected;
            retreat.translation() -= expected.rotation() * Eigen::Vector3d(0, 0, g["approach_distance"].as<double>());
            samePose(record->targets[s].front(), retreat);
        }
        core.disable();
    };
    runCore(split); runCore(legacy);
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

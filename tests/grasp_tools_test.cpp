#include "aviator/Aviator.hpp"
#include "aviator/GraspTools.hpp"
#include "aviator/backend.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#ifdef AVIATOR_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#include "DataLink_direct.hpp"
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
    std::atomic<bool> fail_fk{false};
    std::vector<pinocchio::SE3> targets[2];
    bool solveIk(Side side, const std::array<double, 7> &seed, const pinocchio::SE3 &pose,
                 std::array<double, 7> &out) override {
        targets[int(side)].push_back(pose); out = seed; return true;
    }
    bool solveFk(Side side, const std::array<double, 7> &, pinocchio::SE3 &out) override {
        if (fail_fk) return false;
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
    void setImpedanceProfile(bool) override {}
    GraspState graspState() const override { GraspState g; g.heartbeat = now; g.displacement = displacement; return g; }
    uint64_t sendGraspCommand(GraspCommand c) override { commands.push_back(c); return 0; }
    void waitTick() override { now += .001; }
    double time() const override { return now; }
    void runTrajectory(const std::vector<JointFrame> &v, const std::atomic<bool> &) override {
        trajectories.push_back(v);
        q = v.back().q; now += v.size() * .001;
    }
};
// Let the executor enter FAULT before the owner consumes the failed future. This
// reproduces the release exception being masked by SafetyLost and a new generation.
void managedReleaseFailure(const fs::path& config) {
    auto device = std::make_unique<Device>();
    device->q[1] = device->q[8] = M_PI / 2;
    auto ik = std::make_unique<RecordingIk>();
    auto* solver = ik.get();
    bool allowed = false;
    ManagedOptions options;
    options.snapshot = [] {
        fsm::Snapshot s;
        s.ready = s.settled = s.clear_of_wheel = s.following_authorized = true;
        s.release_authorized = s.source_authorized = s.input_ready = s.fault_cleared = s.emergency_known = true;
        return s;
    };
    options.allow_motion = [&](bool value) { allowed = value; };
    options.heartbeat = [] {};
    options.request_brake = [] {};
    Aviator core(std::move(device), std::move(ik), nullptr, config.string(), options);
    const auto wait = [&](const char* state) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while ((core.GetSystemState() != state || !core.GetSystemStatus().conditions.settled) && std::chrono::steady_clock::now() < deadline) {
            core.Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        check(core.GetSystemState() == state, "Managed release setup failed");
    };
    core.Init(); wait("READY");
    check(core.EnterStandby() == fsm::Reply::accepted, "Home was not accepted");
    wait("STANDBY");
    check(core.GraspWheel() == fsm::Reply::accepted, "Grasp was not accepted");
    wait("FOLLOWING");
    solver->fail_fk = true;
    check(core.LeaveWheel() == fsm::Reply::accepted, "Release was not accepted");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (core.GetState() != "FAULT" && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    check(core.GetState() == "FAULT", "Release FK failure was not injected");
    core.Update();
    check(core.GetSystemState() == "ERROR", "Release exception was masked by SAFE");
    check(core.GetSystemStatus().current_error.find("Release FK failed") != std::string::npos,
          "Release failure lost the original diagnostic");
    check(!allowed, "Release failure left commands authorized");
}
// Hold a profile RPC open to exercise CONTROL entry independently of SDK/hardware.
struct ProfileDevice : Device {
    std::atomic<int> following_calls{0}, default_calls{0};
    bool fail_default = false;
    void setImpedanceProfile(bool following) override {
        if (following) { ++following_calls; return; }
        ++default_calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        if (fail_default) throw std::runtime_error("Injected default impedance failure");
    }
};
void managedControlEntry(const fs::path& config, const std::string& outcome) {
    auto device = std::make_unique<ProfileDevice>();
    auto* observed = device.get();
    device->q[1] = device->q[8] = M_PI / 2;
    device->fail_default = outcome == "failure";
    bool allowed = false, input_ready = true;
    int heartbeats = 0;
    ManagedOptions options;
    options.snapshot = [&] {
        fsm::Snapshot s;
        s.ready = s.settled = s.clear_of_wheel = s.following_authorized = true;
        s.release_authorized = s.source_authorized = s.fault_cleared = s.emergency_known = true;
        s.input_ready = input_ready;
        return s;
    };
    options.allow_motion = [&](bool value) { allowed = value; };
    options.heartbeat = [&] { ++heartbeats; };
    options.request_brake = [] {};
    Aviator core(std::move(device), std::make_unique<RecordingIk>(), nullptr, config.string(), options);
    const auto wait = [&](auto predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            core.Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        check(predicate(), "Managed control entry timed out");
    };
    const auto settled = [&](const char* state) {
        return core.GetSystemState() == state && core.GetSystemStatus().conditions.settled;
    };
    core.Init(); wait([&] { return settled("READY"); });
    core.EnterStandby(); wait([&] { return settled("STANDBY"); });
    core.GraspWheel(); wait([&] { return settled("FOLLOWING"); });
    check(observed->following_calls == 1, "Following profile missing");
    const auto before_target = observed->q;
    const auto sample = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto began = std::chrono::steady_clock::now();
    check(core.StartControl() == fsm::Reply::completed, "START_CONTROL reply changed");
    check(std::chrono::steady_clock::now() - began < std::chrono::milliseconds(100), "CONTROL entry blocked owner");
    check(core.GetSystemState() == "CONTROL", "START_CONTROL stayed in FOLLOWING");
    check(!core.GetSystemStatus().accepts_control, "Targets allowed during profile restoration");
    check(!core.ServoWheel(.1, 0, .5, sample), "Target accepted before default stiffness confirmation");
    // Ensure the call is in flight before injecting protection / an exit request.
    wait([&] { return observed->default_calls == 1; });
    const int before_heartbeats = heartbeats;
    if (outcome == "emergency") core.EmergencyStop("During profile restoration");
    if (outcome == "input_lost") { input_ready = false; core.Update(); }
    if (outcome == "exit") {
        check(core.ExitControl() == fsm::Reply::accepted, "Exit during restoration rejected");
        check(core.GetSystemState() == "FOLLOWING", "Exit did not leave CONTROL immediately");
    }
    if (outcome == "success") {
        wait([&] { return core.GetSystemStatus().accepts_control; });
        check(core.GetSystemState() == "CONTROL", "Restoration changed CONTROL state");
        check(!core.ServoWheel(.1, 0, .5, sample), "Pre-restoration target replayed");
    } else if (outcome == "exit") {
        wait([&] { return settled("FOLLOWING"); });
        check(observed->following_calls == 2, "Cancelled restoration lost Following update");
    } else {
        const auto expected = outcome == "failure" ? "ERROR" : outcome == "emergency" ? "EMERGENCY_STOP" : "SAFE";
        wait([&] { return core.GetSystemState() == expected && core.GetSystemStatus().conditions.executor_idle; });
        check(!allowed && !core.GetSystemStatus().accepts_control, "Late restoration escaped protection");
    }
    check(heartbeats > before_heartbeats + 2, "Restoration blocked supervision/heartbeats");
    check(observed->default_calls == 1, "Repeated default restoration in CONTROL");
    check(observed->q == before_target, "Profile restoration reset hold target");
}
// Check collision behavior independently of the real model's initial contacts.
void collisionModelRegression(const fs::path &dir) {
    const auto path = dir / "collision.urdf";
    std::ofstream out(path);
    const std::string sphere =
        "<collision><geometry><sphere radius='0.1'/></geometry></collision>";
    out << "<robot name='collision_fixture'><link name='aircraft'>" << sphere << "</link>";
    for (int side = 0; side < 2; ++side) {
        const std::string prefix = std::string("AR5-5_07") + (side ? "R" : "L") + "-W4C4A2";
        std::string parent = "aircraft";
        for (int j = 1; j <= 7; ++j) {
            const auto link = prefix + "_link" + std::to_string(j);
            out << "<link name='" << link << "'>" << (j >= 6 ? sphere : "") << "</link>"
                << "<joint name='" << prefix << "_joint_" << j << "' type='prismatic'>"
                << "<parent link='" << parent << "'/><child link='" << link << "'/>"
                << "<origin xyz='" << (j == 1 ? (side ? 2 : -2) : 0) << " 0 0'/>"
                << "<axis xyz='1 0 0'/><limit lower='-10' upper='10' effort='1' velocity='1'/>"
                << "</joint>";
            parent = link;
        }
    }
    // An extra moving joint exercises complete models with more than 16 DOFs.
    out << "<link name='finger'>" << sphere << "</link>"
        << "<joint name='finger_joint' type='revolute'><parent link='AR5-5_07R-W4C4A2_link7'/>"
        << "<child link='finger'/><origin xyz='0 1 0'/><axis xyz='0 0 1'/>"
        << "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint>"
        << "<link name='dummy'/><link name='wheel'>" << sphere << "</link>"
        << "<joint name='roll_input_joint' type='revolute'><parent link='aircraft'/>"
        << "<child link='dummy'/><origin xyz='0 4 0'/><axis xyz='0 0 1'/>"
        << "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        << "<joint name='pitch_input_joint' type='prismatic'><parent link='dummy'/>"
        << "<child link='wheel'/><axis xyz='0 1 0'/>"
        << "<limit lower='-10' upper='10' effort='1' velocity='1'/></joint></robot>";
    out.close();
    auto checker = makePinocchioCollisionChecker(path.string());
    std::array<double, 14> q{};
    checker->check(q, 0, 0); // Coincident adjacent link6/link7 are intentional.
    const auto mustCollide = [&](double wheel) {
        bool collided = false;
        try { checker->check(q, 0, wheel); }
        catch (const std::runtime_error &e) {
            collided = std::string(e.what()).find("Planned collision:") == 0;
        }
        check(collided, "Collision checker missed nonadjacent geometry");
    };
    q[6] = 2; mustCollide(0); // Left arm against aircraft.
    q[6] = 4; mustCollide(0); // Left arm against right arm.
    q[6] = 0; mustCollide(-4); // Wheel against aircraft; no legacy SRDF exclusion.
    checker->check(q, 0, 0); // Clear previous collision results on the next call.
}

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
    managedReleaseFailure(dir / "robot.yaml");
    for (const auto* outcome : {"success", "failure", "emergency", "input_lost", "exit"})
        managedControlEntry(dir / "robot.yaml", outcome);
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
    collisionModelRegression(dir);
    // Load complete geometry without a legacy collision URDF or SRDF.
    makePinocchioCollisionChecker((root / "models/urdf/aviator.urdf").string());
#ifdef AVIATOR_HAVE_MUJOCO
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML((root / "models/mjcf/aviator.xml").c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    check(bool(model), error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "aviator_home"));
    const int wheel = mj_name2id(model.get(), mjOBJ_JOINT, "pitch_input_joint");
    data->qpos[model->jnt_qposadr[wheel]] = -.085;
    const std::vector<mjtNum> initial(data->qpos, data->qpos + model->nq);
    SimulationTools sim_tools;
    for (int side = 0; side < 2; ++side) {
        const auto& t = geometry.tools[side];
        const Eigen::Quaterniond q(t.rotation());
        sim_tools[side].position = {t.translation()[0], t.translation()[1], t.translation()[2]};
        sim_tools[side].quaternion = {q.w(), q.x(), q.y(), q.z()};
    }
    auto device = std::make_unique<MuJoCoDirectDataLink>(model.get(), data.get(),
                               (root / "models/urdf/aviator.urdf").string(), sim_tools,
                               config["rokae"]["joint_stiffness"].as<std::array<double, 7>>());
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

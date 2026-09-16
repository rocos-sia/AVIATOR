#include "rocos_mujoco/model_loader.hpp"
#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unistd.h>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct TestBus {
    const int id = static_cast<int>(getpid());
    ~TestBus() {
        for (const char* prefix : {"/ecm", "/pd_input", "/pd_output"})
            shm_unlink((std::string(prefix) + std::to_string(id)).c_str());
        for (int i = 0; i < EC_SEM_NUM; ++i)
            sem_unlink(("/sync" + std::to_string(id) + "_" + std::to_string(i)).c_str());
    }
};

void checkPassiveJoints(const char* path) {
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        rocos_mujoco::loadSimulationModel(path, error, sizeof(error)), mj_deleteModel);
    require(model != nullptr, error);
    auto* m = model.get();
    require(m->nq == 16 && m->nv == 16 && m->njnt == 16, "Expected 16 scalar joints and fixed aircraft");
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
    auto* d = data.get();
    require(d != nullptr, "Could not allocate model data");
    const int home = mj_name2id(m, mjOBJ_KEY, "aviator_home");
    require(home >= 0, "Missing elbow-down startup keyframe");
    mj_resetDataKeyframe(m, d, home);
    mj_forward(m, d);
    require(d->ncon == 0, "Unexpected initial collision penetration");
    auto geom = [&](const char *name) {
        int id = mj_name2id(m, mjOBJ_GEOM, name);
        require(id >= 0, std::string("Missing geometry: ") + name);
        return id;
    };
    auto eligible = [&](int a, int b) {
        return (m->geom_contype[a] & m->geom_conaffinity[b]) ||
               (m->geom_contype[b] & m->geom_conaffinity[a]);
    };
    const int left_cylinder = geom("left_grasp_cylinder"), right_cylinder = geom("right_grasp_cylinder");
    require(m->geom_type[left_cylinder] == mjGEOM_CYLINDER && m->geom_type[right_cylinder] == mjGEOM_CYLINDER,
            "Both grasp proxies must be cylinders");
    require(!eligible(left_cylinder, geom("steering_wheel_collision_31")) &&
            !eligible(right_cylinder, geom("steering_wheel_collision_33")), "Intentional grasp overlap must be allowed");
    require(eligible(left_cylinder, right_cylinder) && eligible(left_cylinder, geom("steering_wheel_collision_33")) &&
            eligible(right_cylinder, geom("steering_wheel_collision_31")), "Opposite-hand collisions must remain active");
    for (int cylinder : {left_cylinder, right_cylinder}) {
        require(eligible(cylinder, geom("floor")) && eligible(cylinder, geom("aircraft_collision_0")) &&
                eligible(cylinder, geom("steering_wheel_collision_30")) &&
                eligible(cylinder, geom("steering_wheel_collision_32")), "Cylinder environment collisions were disabled");
        require(m->body_mass[m->geom_bodyid[cylinder]] > 0, "Cylinder payload must have inertia");
    }
    for (const char side : {'L', 'R'}) {
        const std::string prefix = std::string("AR5-5_07") + side + "-W4C4A2_";
        const int joint = mj_name2id(m, mjOBJ_JOINT, (prefix + "joint_2").c_str());
        const int elbow = mj_name2id(m, mjOBJ_BODY, (prefix + "link4").c_str());
        const int shoulder = mj_name2id(m, mjOBJ_BODY, (prefix + "link2").c_str());
        require(std::abs(d->qpos[m->jnt_qposadr[joint]] - std::acos(-1.) / 2) < 1e-9,
                "Joint 2 must start at +90 deg");
        require(d->xpos[3 * elbow + 2] < d->xpos[3 * shoulder + 2] - .25,
                "Initial elbow must be below shoulder");
    }

    const int roll = mj_name2id(m, mjOBJ_JOINT, "roll_input_joint");
    const int pitch = mj_name2id(m, mjOBJ_JOINT, "pitch_input_joint");
    require(roll >= 0 && pitch >= 0, "Missing wheel joints");
    require(m->jnt_type[roll] == mjJNT_HINGE && m->jnt_type[pitch] == mjJNT_SLIDE,
            "Wheel must rotate and translate");
    require(std::abs(m->jnt_range[2 * roll] + 0.87266) < 1e-8 &&
            std::abs(m->jnt_range[2 * roll + 1] - 0.87266) < 1e-8 &&
            std::abs(m->jnt_range[2 * pitch] + 0.165) < 1e-8 &&
            std::abs(m->jnt_range[2 * pitch + 1]) < 1e-8, "Incorrect wheel limits");
    for (int a = 0; a < m->nu; ++a)
        require(m->actuator_trnid[2 * a] != roll && m->actuator_trnid[2 * a] != pitch,
                "Passive wheel has an actuator");

    // Apply only external generalized force, without any motor command.
    // Hold the arms with the same PD law used by the simulator while testing
    // each wheel DOF independently. Start away from the slide's end stop.
    const int roll_q = m->jnt_qposadr[roll], pitch_q = m->jnt_qposadr[pitch];
    const int roll_v = m->jnt_dofadr[roll], pitch_v = m->jnt_dofadr[pitch];
    for (int v = 0; v < m->nv; ++v)
        if (v != roll_v && v != pitch_v) m->dof_damping[v] += 80;
    for (int selected : {roll, pitch}) {
        mj_resetDataKeyframe(m, d, home);
        d->qpos[pitch_q] = -0.08;
        mj_forward(m, d);
        for (int step = 0; step < 200; ++step) {
            for (int joint = 0; joint < m->njnt; ++joint) {
                const int v = m->jnt_dofadr[joint], q = m->jnt_qposadr[joint];
                d->qfrc_applied[v] = (joint == roll || joint == pitch)
                    ? 0 : d->qfrc_bias[v] + 1000 * (m->key_qpos[home * m->nq + q] - d->qpos[q]);
            }
            d->qfrc_applied[m->jnt_dofadr[selected]] = selected == roll ? 0.02 : -0.1;
            mj_step(m, d);
        }
        require(selected == roll ? d->qpos[roll_q] > 0.005 : d->qpos[pitch_q] < -0.081,
                "Passive wheel did not respond to external force");
        require(selected == roll ? std::abs(d->qpos[pitch_q] + 0.08) < 0.001
                                 : std::abs(d->qpos[roll_q]) < 0.001,
                "Wheel DOFs unexpectedly coupled");
        for (int w = 0; w < mjNWARNING; ++w)
            require(d->warning[w].number == 0, "MuJoCo warning during passive-joint test");
    }
    std::cout << "Both passive wheel joints respond to external forces.\n";
}

void checkDrives(const char* model, const char* config) {
    TestBus bus;
    rocos_mujoco::MujocoSimulator sim(model, config, bus.id);
    sim.setVerbose(false);
    require(sim.initialize() && sim.getJointCount() == 14, "Expected 14 configured drives");
    rocos::SharedMemoryConfig client(bus.id);
    require(client.getSharedMemory() && client.getPdDataMemoryProvider(), "Could not attach test client");
    require(client.getSlaveNum() == 14, "Passive joints must not appear on EtherCAT bus");
    const auto hardware = YAML::LoadFile(config)["hardware"];
    require(hardware.size() == 14, "YAML must contain 14 drives");
    double home[14]{};
    for (int axis = 0; axis < 14; ++axis) {
        const std::string expected = std::string("AR5-5_07") + (axis < 7 ? "L" : "R") +
            "-W4C4A2_joint_" + std::to_string(axis % 7 + 1);
        require(hardware[axis]["id"].as<int>() == axis &&
                hardware[axis]["joint_name"].as<std::string>() == expected,
                "Incorrect left/right arm slave order");
        client.setSlaveOutputVarValueByName<int8_t>(axis, "Mode of operation", 8);
        const auto initial = client.getSlaveInputVarValueByName<int32_t>(axis, "Position actual value");
        home[axis] = initial / hardware[axis]["transform"]["cnt_per_unit"].as<double>();
        client.setSlaveOutputVarValueByName<int32_t>(axis, "Target Position", initial);
    }
    for (uint16_t control : {uint16_t(0), uint16_t(6), uint16_t(7), uint16_t(15)}) {
        for (int axis = 0; axis < 14; ++axis)
            client.setSlaveOutputVarValueByName<uint16_t>(axis, "Control word", control);
        sim.step();
    }
    // Distinct targets exercise name/address mapping past the two passive
    // joints; reverse direction to detect frozen or wrongly mapped feedback.
    for (double direction : {1.0, -1.0}) {
        for (int axis = 0; axis < 14; ++axis) {
            const double scale = hardware[axis]["transform"]["cnt_per_unit"].as<double>();
            client.setSlaveOutputVarValueByName<int32_t>(axis, "Target Position",
                static_cast<int32_t>((home[axis] + direction * 0.001 * (axis + 1)) * scale));
        }
        for (int step = 0; step < 2000; ++step) sim.step();
        for (int axis = 0; axis < 14; ++axis) {
            const double scale = hardware[axis]["transform"]["cnt_per_unit"].as<double>();
            const auto status = client.getSlaveInputVarValueByName<uint16_t>(axis, "Status word");
            const double actual = client.getSlaveInputVarValueByName<int32_t>(
                axis, "Position actual value") / scale;
            const double target = home[axis] + direction * 0.001 * (axis + 1);
            std::cout << "axis " << axis << ": target=" << target << " actual=" << actual << '\n';
            require((status & 0x006f) == 0x0027, "Drive failed to enable");
            require(std::abs(actual - target) < 0.0005, "Drive failed to track target");
        }
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    try {
        checkPassiveJoints(argv[1]);
        checkDrives(argv[1], argv[2]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}

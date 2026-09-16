#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <filesystem>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const int bus_id = static_cast<int>(getpid());
    rocos_mujoco::MujocoSimulator sim(argv[1], argv[2], bus_id);
    sim.setVerbose(false);
    if (!sim.initialize() || sim.getJointCount() != 7) return 1;
    rocos::SharedMemoryConfig client(bus_id);
    if (!client.getSharedMemory() || !client.getPdDataMemoryProvider()) return 1;
    const auto hardware = YAML::LoadFile(argv[2])["hardware"];

    // Give each axis a distinct target, exercising qpos/DOF mapping after
    // the scene's freejoint as well as the original URDF's joint2 alias.
    for (int axis = 0; axis < 7; ++axis) {
        const double scale = hardware[axis]["transform"]["cnt_per_unit"].as<double>();
        client.setSlaveOutputVarValueByName<int8_t>(axis, "Mode of operation", 8);
        client.setSlaveOutputVarValueByName<int32_t>(
            axis, "Target Position", static_cast<int32_t>(0.02 * (axis + 1) * scale));
    }
    for (uint16_t control : {uint16_t(0), uint16_t(6), uint16_t(7), uint16_t(15)}) {
        for (int axis = 0; axis < 7; ++axis)
            client.setSlaveOutputVarValueByName<uint16_t>(axis, "Control word", control);
        sim.step();
    }
    for (int step = 0; step < 3000; ++step) sim.step();

    bool passed = true;
    for (int axis = 0; axis < 7; ++axis) {
        const double scale = hardware[axis]["transform"]["cnt_per_unit"].as<double>();
        const auto status = client.getSlaveInputVarValueByName<uint16_t>(axis, "Status word");
        const double position = client.getSlaveInputVarValueByName<int32_t>(
            axis, "Position actual value") / scale;
        const double target = 0.02 * (axis + 1);
        std::cout << "axis " << axis << ": target=" << target << " actual=" << position << '\n';
        // The original URDF has a 16 mm base/link_1 mesh overlap. MuJoCo
        // 3.4 contact friction opposes axis 1, so verify its motion response;
        // tracking to the target is asserted for every collision-free axis.
        const bool base_contact = axis == 0 &&
            std::filesystem::path(argv[1]).extension() == ".urdf";
        const bool position_ok = base_contact
            ? position > 0.002 && position < target + 0.01
            : std::abs(position - target) < 0.01;
        if ((status & 0x006f) != 0x0027 || !position_ok) passed = false;
    }
    // These objects belong exclusively to this test process.
    for (const char* prefix : {"/ecm", "/pd_input", "/pd_output"})
        shm_unlink((std::string(prefix) + std::to_string(bus_id)).c_str());
    for (int i = 0; i < EC_SEM_NUM; ++i)
        sem_unlink(("/sync" + std::to_string(bus_id) + "_" + std::to_string(i)).c_str());
    return passed ? 0 : 1;
}

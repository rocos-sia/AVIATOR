// 启动姿态与基础接口：读模型 → 对比配置 → 初始化 → 使能 → 失能。
#include "aviator/Aviator.hpp"
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

static void check(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

int main() {
    try {
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        check(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int home = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
        check(data && home >= 0, "Missing simulation data or aviator_home");
        mj_resetDataKeyframe(model.get(), data.get(), home);
        mj_forward(model.get(), data.get());

        const auto posture = YAML::LoadFile(AVIATOR_CONFIG_DIR "/posture.json");
        const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        const auto grasp = YAML::LoadFile(AVIATOR_CONFIG_DIR "/grasp.json");
        for (int side = 0; side < 2; ++side) {
            const std::string side_name = side == 0 ? "left" : "right";
            const int site = mj_name2id(model.get(), mjOBJ_SITE, (side_name + "_handle").c_str());
            check(site >= 0, "Missing handle site");
            const auto p = grasp[side_name]["position"].as<std::vector<double>>();
            const auto q = grasp[side_name]["quaternion"].as<std::vector<double>>();
            for (int j = 0; j < 3; ++j)
                check(std::abs(p[j] - model->site_pos[3*site+j]) < 1e-10, "Grasp/site position mismatch");
            double dot = 0;
            for (int j = 0; j < 4; ++j) dot += q[j] * model->site_quat[4*site+j];
            check(std::abs(std::abs(dot)-1) < 1e-10, "Grasp/site orientation mismatch");
            const auto expected = posture[side == 0 ? "left_home_deg" : "right_home_deg"].as<std::vector<double>>();
            check(expected.size() == 7, "Home posture must contain seven joints");
            for (int axis = 0; axis < 7; ++axis) {
                const auto name = std::string("AR5-5_07") + (side == 0 ? "L" : "R") +
                                  "-W4C4A2_joint_" + std::to_string(axis + 1);
                const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
                check(id >= 0, "Missing joint: " + name);
                const double angle = data->qpos[model->jnt_qposadr[id]];
                check(std::abs(angle - expected[axis] * M_PI / 180) < 1e-6, "Home mismatch: " + name);
                if (axis == 1) check(angle >= limits[0]*M_PI/180 && angle <= limits[1]*M_PI/180,
                                     "J2 out of configured range");
            }
        }

        aviator::Aviator robot(model.get(), data.get(), AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.SetRealTime(false);
        check(robot.GetState() == "INITIALIZED", "Init failed");
        check(!robot.GetStatus().locked && !robot.GetStatus().fault, "Unexpected initial grasp state");
        robot.Enable();
        check(robot.GetState() == "ENABLED", "Enable failed");
        robot.Disable();
        check(robot.GetState() == "DISABLED", "Disable failed");
        std::cout << "Home posture and basic control passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

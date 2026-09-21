// 完整仿真验收：顺序调用公开接口，并检查实际轮盘位置和全程 J2 范围。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include "../src/DataLink_direct.hpp"
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
        const int roll = mj_name2id(model.get(), mjOBJ_JOINT, "roll_input_joint");
        const int slide = mj_name2id(model.get(), mjOBJ_JOINT, "pitch_input_joint");
        check(roll >= 0 && slide >= 0, "Missing wheel joints");

        auto link = std::make_unique<aviator::MuJoCoDirectDataLink>(
            model.get(), data.get(), AVIATOR_CONFIG_DIR "/aviator_control.urdf");
        auto *simulation = link.get(); // 只用于读取逐物理步统计的 J2 范围。
        aviator::Aviator robot(std::move(link), nullptr, nullptr, AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.SetRealTime(false);

        bool rejected = false;
        try { robot.MoveWheel(.1, 0, .5); } catch (const std::exception &) { rejected = true; }
        check(rejected, "MoveWheel accepted before enable/lock");
        rejected = false;
        try { robot.LockHandles(); } catch (const std::exception &) { rejected = true; }
        check(rejected, "LockHandles accepted before approach");

        robot.Enable();
        robot.ApproachHandles();
        const auto approach = robot.GetStatus();
        for (int side = 0; side < 2; ++side) {
            check(approach.position_error[side] < .002, "Approach position error >= 2 mm");
            check(approach.rotation_error[side] < .01, "Approach rotation error >= .01 rad");
        }
        robot.LockHandles();
        check(robot.GetStatus().locked == 3, "Both handles must be locked");

        for (const auto target : {std::array<double, 3>{.87266, 0, .5},
                                  {-.87266, 0, .5},
                                  {0, 0, .5},
                                  {0, -.170, .5},
                                  {0, 0, .5}}) {
            robot.MoveWheel(target[0], target[1], target[2]);
            const auto s = robot.GetStatus();
            check(std::abs(s.angle-target[0]) < 1e-9 && std::abs(s.displacement-target[1]) < 1e-9,
                  "Reference did not reach target");
            // Status 是规划参考；验收误差必须读取 MuJoCo 的被动轮盘关节。
            std::lock_guard<std::mutex> lock(*robot.PhysicsMutex());
            const double angle_error = std::abs(data->qpos[model->jnt_qposadr[roll]]-target[0]);
            const double displacement_error = std::abs(data->qpos[model->jnt_qposadr[slide]]-target[1]);
            std::cout << "target=" << target[0] << ',' << target[1]
                      << " actual_error=" << angle_error << " rad, " << displacement_error*1000 << " mm\n";
            check(angle_error < .005, "Wheel rotation error >= .005 rad");
            check(displacement_error < .001, "Wheel displacement error >= 1 mm");
        }

        const auto range = simulation->elbowRange();
        std::cout << "J2 range=" << range[0]*180/M_PI << " ... " << range[1]*180/M_PI << " deg\n";
        check(range[0] >= 85*M_PI/180 && range[1] <= 95*M_PI/180, "J2 outside [85,95] degrees");
        rejected = false;
        try { robot.MoveWheel(2, 0, .5); } catch (const std::exception &) { rejected = true; }
        check(rejected, "Out-of-range target accepted");

        robot.UnlockHandles();
        check(robot.GetStatus().locked == 0, "Unlock failed");
        robot.Disable();
        check(robot.GetState() == "DISABLED", "Disable failed");
        std::cout << "Full simulation acceptance passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

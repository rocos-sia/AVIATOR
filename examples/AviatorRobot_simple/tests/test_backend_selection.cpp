// 后端配置测试：非法配置必须在联网前拒绝；仿真走实际配置入口。
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <memory>
#include <stdexcept>

static void check(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

int main() {
    try {
        const std::string urdf = AVIATOR_CONFIG_DIR "/aviator_control.urdf";
        for (const std::string backend : {"unknown", "mujoco", "rokae"}) {
            std::string error;
            try { auto link = aviator::makeDataLink(backend, {}, urdf, {}); }
            catch (const std::exception &e) { error = e.what(); }
            std::cout << backend << ": " << error << '\n';
            check(!error.empty(), "Invalid backend configuration accepted");
            if (backend == "unknown")
                check(error.find("mujoco") != std::string::npos && error.find("rokae") != std::string::npos,
                      "Unknown-backend error must list supported backends");
            if (backend == "mujoco")
                check(error.find("mjModel") != std::string::npos, "Error must identify missing simulation handles");
            if (backend == "rokae") {
#ifdef AVIATOR_HAVE_ROKAE
                check(error.find("未配置") != std::string::npos, "Error must identify missing IPs");
#else
                check(error.find("xCore SDK") != std::string::npos, "Error must identify missing SDK");
#endif
            }
        }
#ifdef AVIATOR_HAVE_ROKAE
        aviator::RokaeConfig invalid;
        invalid.left_ip = "192.0.2.1";
        invalid.right_ip = "198.51.100.2";
        invalid.left_local_ip = "192.0.2.3";
        invalid.right_local_ip = "198.51.100.3";
        invalid.joint_stiffness[6] = 301;
        std::string error;
        try { auto link = aviator::makeRokaeDataLink(urdf, invalid, {}); }
        catch (const std::exception &e) { error = e.what(); }
        check(error.find("stiffness") != std::string::npos, "Invalid stiffness must be rejected before connecting");
        // 保留非法刚度作为联网前的保护，分别检查两侧缺失地址和地址冲突。
        for (int side = 0; side < 2; ++side) {
            auto missing = invalid;
            (side == 0 ? missing.left_local_ip : missing.right_local_ip).clear();
            error.clear();
            try { auto link = aviator::makeRokaeDataLink(urdf, missing, {}); }
            catch (const std::exception &e) { error = e.what(); }
            check(error.find(side == 0 ? "left_local_ip" : "right_local_ip") != std::string::npos,
                  "Missing local IP must identify the corresponding arm");
            for (const auto &controller_ip : {invalid.left_ip, invalid.right_ip}) {
                auto conflict = invalid;
                (side == 0 ? conflict.left_local_ip : conflict.right_local_ip) = controller_ip;
                error.clear();
                try { auto link = aviator::makeRokaeDataLink(urdf, conflict, {}); }
                catch (const std::exception &e) { error = e.what(); }
                check(error.find("上位机 IP") != std::string::npos, "Local/controller IP conflict accepted");
            }
        }
        auto duplicate = invalid;
        duplicate.right_ip = duplicate.left_ip;
        error.clear();
        try { auto link = aviator::makeRokaeDataLink(urdf, duplicate, {}); }
        catch (const std::exception &e) { error = e.what(); }
        check(error.find("左右臂控制器 IP") != std::string::npos, "Duplicate controller IP accepted");
#endif
        char mj_error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, mj_error, sizeof(mj_error)), mj_deleteModel);
        check(bool(model), mj_error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int home = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
        check(data && home >= 0, "Missing simulation data or aviator_home");
        mj_resetDataKeyframe(model.get(), data.get(), home);
        mj_forward(model.get(), data.get());
        {
            auto link = aviator::makeDataLink("mujoco", {model.get(), data.get()}, urdf, {});
            check(bool(link), "MuJoCo factory returned no backend");
        }

        aviator::Aviator robot(model.get(), data.get(), AVIATOR_CONFIG_DIR "/aviator.yaml");
        robot.Init();
        robot.SetRealTime(false);
        check(robot.GetState() == "INITIALIZED", "Configuration-driven Init failed");
        robot.Enable();
        check(robot.GetState() == "ENABLED", "Configuration-driven Enable failed");
        robot.Disable();
        std::cout << "Backend selection passed without hardware connections\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

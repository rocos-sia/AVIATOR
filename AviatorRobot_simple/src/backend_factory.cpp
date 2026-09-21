#include "aviator/backend.hpp"

#include <stdexcept>
#include <string>

// 后端分发的唯一入口。
//
// 由 config 的 backend 字段选择实现，算法层与主程序都不直接引用具体后端类型，
// 因此"仿真 / 真机"的切换点收敛成一个配置字段。
//
namespace aviator {

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
} // namespace

std::unique_ptr<DataLink> makeDataLink(const std::string &backend, const BackendContext &context,
                                       const std::string &urdf_path,
                                       const GraspGeometry &geometry,
                                       const RokaeConfig &rokae) {
    if (backend == "mujoco") {
        if (!context.hasSimulation())
            fail("backend: mujoco 需要 mjModel/mjData 句柄；"
                 "请从 aviator 启动（它会加载模型并传入句柄）");
#ifdef AVIATOR_HAVE_MUJOCO
        return makeMuJoCoDirectDataLink(context.model, context.data, urdf_path);
#else
        fail("本构建未包含 MuJoCo 后端；请启用 AVIATOR_WITH_SIMULATION");
#endif
    }

    if (backend == "rokae") {
#ifdef AVIATOR_HAVE_ROKAE
        return makeRokaeDataLink(urdf_path, rokae, geometry);
#else
        fail("backend: rokae 已配置，但本次构建未包含 Rokae xCore SDK。"
             "请确认 SDK 的头文件与预编译库就位后重新构建 "
             "（重新配置 -DAVIATOR_WITH_ROKAE=ON）");
#endif
    }

    fail("未知 backend: '" + backend + "'（可选: mujoco / rokae）");
}

} // namespace aviator

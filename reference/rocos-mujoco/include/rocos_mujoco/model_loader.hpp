#pragma once

#include <mujoco/mujoco.h>

#include <cstdio>
#include <filesystem>
#include <string>

namespace rocos_mujoco {

// Share model loading between the headless engine and the viewer. CAD URDFs
// use package:// mesh names; MuJoCo strips those paths and needs a meshdir.
inline mjModel* loadSimulationModel(const std::string& path, char* error,
                                    int error_size) {
    namespace fs = std::filesystem;
    mjSpec* spec = mj_parseXML(path.c_str(), nullptr, error, error_size);
    if (!spec) return nullptr;
    const fs::path model_path = fs::absolute(path);
    const fs::path mesh_dir = model_path.parent_path().parent_path() / "meshes";
    if (model_path.extension() == ".urdf" && fs::is_directory(mesh_dir) &&
        std::string(mjs_getString(spec->compiler.meshdir)).empty()) {
        mjs_setString(spec->compiler.meshdir, mesh_dir.string().c_str());
    }
    mjModel* model = mj_compile(spec, nullptr);
    if (!model) std::snprintf(error, error_size, "%s", mjs_getError(spec));
    mj_deleteSpec(spec);
    return model;
}

}  // namespace rocos_mujoco

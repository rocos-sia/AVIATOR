#pragma once
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace aviator {

// Per-arm transforms must be supplied as a pair. Legacy shared poses remain valid.
inline YAML::Node toolFrameConfig(const YAML::Node &grasp, int side) {
    const auto tool = grasp["tool"];
    if (!tool || !tool.IsMap())
        throw std::runtime_error("grasp.tool must be a map");
    if (tool["left"] || tool["right"]) {
        if (!tool["left"] || !tool["right"])
            throw std::runtime_error("grasp.tool requires both left and right transforms");
        if (tool["position"] || tool["quaternion"])
            throw std::runtime_error("grasp.tool cannot mix shared and per-arm transforms");
        return tool[side == 0 ? "left" : "right"];
    }
    return tool;
}

} // namespace aviator

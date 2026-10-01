#pragma once
#include "DataLink.hpp"
#include <cmath>
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

// FK distances correspond to the actual 1 ms arm samples. Each hand uses its
// own trigger and a quintic trajectory ending at the common arm endpoint.
template<class Distance>
void planHandClosure(std::vector<JointFrame>& frames, double threshold, Distance distance) {
    if (!std::isfinite(threshold) || threshold <= 0 || frames.empty())
        throw std::runtime_error("Invalid hand closing distance/trajectory");
    for (int side = 0; side < 2; ++side) {
        size_t start = frames.size();
        for (size_t k = 0; k < frames.size(); ++k) {
            const auto d = distance(side, frames[k]);
            if (!std::isfinite(d) || d < 0)
                throw std::runtime_error("Invalid tool-to-grasp distance");
            if (d <= threshold && start == frames.size()) start = k;
            if (start != frames.size() && d > threshold)
                throw std::runtime_error("Approach leaves hand closing range after entering it");
        }
        if (start >= frames.size() - 1 && frames.size() > 1)
            throw std::runtime_error("No remaining approach trajectory for synchronized hand closure");
        if (start == frames.size())
            throw std::runtime_error("Approach never enters hand closing range");
        for (size_t k = 0; k < frames.size(); ++k) {
            const double u = frames.size() == 1 ? 1 : k <= start ? 0 :
                double(k - start) / (frames.size() - 1 - start);
            frames[k].hand_closure[side] = u * u * u * (10 + u * (-15 + 6 * u));
        }
    }
}

} // namespace aviator

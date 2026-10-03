#include "assets.hpp"
#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
namespace monitor {
namespace fs = std::filesystem;
namespace {
bool beneath(const fs::path& child, const fs::path& parent) {
    auto c = child.begin();
    for (auto p = parent.begin(); p != parent.end(); ++p, ++c)
        if (c == child.end() || *c != *p)
            return false;
    return true;
}
std::string content(const fs::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file || file.tellg() < 0 || file.tellg() > 32 * 1024 * 1024)
        return {};
    std::string bytes(static_cast<std::size_t>(file.tellg()), '\0');
    file.seekg(0);
    if (!bytes.empty())
        file.read(bytes.data(), bytes.size());
    return file ? bytes : std::string();
}
} // namespace
Assets::Assets(const fs::path& web, const fs::path& models, const Json& config) {
    for (auto* name : {"app.js", "settings.js", "viewer.js", "vendor.js", "style.css"}) {
        files_["/assets/" + std::string(name)] = {
            web / name, std::string(name).find(".css") != std::string::npos
                            ? "text/css; charset=utf-8"
                            : "text/javascript; charset=utf-8"};
    }
    manifest = {{"model_url", "/models/urdf/aviator.urdf"},
                {"aliases", Json::object()},
                {"arm_joints", config.at("arm_joints")},
                {"hand_calibration_id", config.at("hand_calibration").is_null()
                                            ? Json(nullptr)
                                            : config.at("hand_calibration").at("id")},
                {"yoke_calibration_id", config.at("yoke_calibration").is_null()
                                            ? Json(nullptr)
                                            : config.at("yoke_calibration").at("id")},
                {"resource_errors", Json::array()}};
    const auto& calibration = config.at("yoke_calibration");
    if (!calibration.is_null()) {
        const auto& display = calibration.at("model");
        const double rsign = display.at("roll_sign"), roffset = display.at("roll_offset_rad");
        const double psign = display.at("pitch_sign"), poffset = display.at("pitch_offset_m");
        const double r1 = rsign * -52 * 3.14159265358979323846 / 180 + roffset;
        const double r2 = rsign * 52 * 3.14159265358979323846 / 180 + roffset;
        const double p1 = psign * -.005 + poffset, p2 = psign * .175 + poffset;
        manifest["yoke_display_limits"] = {
            {"roll_input_joint", {{"lower", std::min(r1, r2)}, {"upper", std::max(r1, r2)}}},
            {"pitch_input_joint", {{"lower", std::min(p1, p2)}, {"upper", std::max(p1, p2)}}}};
    }
    const auto root = fs::weakly_canonical(models);
    const auto urdf = fs::weakly_canonical(root / "urdf/aviator.urdf");
    if (!beneath(urdf, root)) {
        manifest["resource_errors"].push_back("URDF outside model root");
        return;
    }
    files_["/models/urdf/aviator.urdf"] = {urdf, "application/xml; charset=utf-8"};
    const auto xml = content(urdf);
    if (xml.empty()) {
        manifest["resource_errors"].push_back("Cannot load aviator.urdf");
        return;
    }
    const std::regex mesh(R"MESH(<mesh\s+[^>]*filename\s*=\s*"([^"]+)")MESH");
    for (auto it = std::sregex_iterator(xml.begin(), xml.end(), mesh); it != std::sregex_iterator();
         ++it) {
        const auto name = (*it)[1].str();
        auto relative = (fs::path("urdf") / name).lexically_normal();
        if (relative == "meshes/aircraft.STL")
            relative = "meshes/Cessna/aircraft.STL";
        if (relative == "meshes/steering_wheel.STL" || relative == "urdf/steering_wheel.STL")
            relative = "meshes/Cessna/steering_wheel.STL";
        auto path = fs::weakly_canonical(root / relative);
        if (!beneath(path, root) || !fs::is_regular_file(path)) {
            manifest["resource_errors"].push_back(name);
            continue;
        }
        auto original = "/models/" + (fs::path("urdf") / name).lexically_normal().generic_string();
        auto url = "/models/" + relative.generic_string();
        manifest["aliases"][original] = url;
        files_[url] = {path, "application/octet-stream"};
    }
}
bool Assets::read(const std::string& url, Asset& result) const {
    const auto found = files_.find(url);
    if (found == files_.end())
        return false;
    result = {content(found->second.first), found->second.second};
    return !result.body.empty();
}
} // namespace monitor

#pragma once
// SDK isolation boundary: no KDL/Eigen/SDK types cross this interface.
#include <array>
#include <functional>
#include <memory>
#include <string>
namespace aviator {
struct RokaeSample {
    std::array<double, 7> position{}, velocity{};
    std::array<double, 16> tcp{};
};
class RokaeArm {
public:
    RokaeArm(const std::string &ip, const std::string &local_ip,
             const std::array<double, 16> &tool, const std::array<double, 7> &stiffness);
    ~RokaeArm();
    std::array<double, 7> position() const;
    void start(std::function<std::array<double, 7>(const RokaeSample &)> callback);
    void stop();
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}

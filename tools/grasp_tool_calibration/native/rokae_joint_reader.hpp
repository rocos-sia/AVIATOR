#pragma once

// Keep SDK/Eigen/KDL types inside the vendor bridge, as in manipulator's SDK
// isolation boundary. This reader deliberately does not use RokaeArm.
#include <array>
#include <memory>
#include <string>

namespace aviator::calibration {

class RokaeJointReader {
 public:
    explicit RokaeJointReader(const std::string& ip);
    ~RokaeJointReader();
    RokaeJointReader(const RokaeJointReader&) = delete;
    RokaeJointReader& operator=(const RokaeJointReader&) = delete;
    std::array<double, 7> position();

 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace aviator::calibration

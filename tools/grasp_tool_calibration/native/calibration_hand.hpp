#pragma once
#include "HandRuntime.hpp"

namespace aviator::calibration {
// Independent per-side polling and explicit operator commands, matching the
// RH56FTP example. The owning session must outlive these workers.
class CalibrationHands final : public hand::Endpoint {
 public:
    CalibrationHands(hand::Config, std::array<hand::Transfer, 2>,
                     std::array<hand::Values, 2> open);
    ~CalibrationHands();
    void receive(const Message&) override;
    unsigned periodUs() const override;
    Message state() override;
 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace aviator::calibration

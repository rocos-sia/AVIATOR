#pragma once
#include "config.hpp"
#include "protocol.hpp"
#include "rs422.hpp"
#include "runtime.hpp"
namespace flight_gateway {
void runRs422(const Config&, int signals_fd);
// Adds only fresh, validated business feedback; default fields remain explicitly unknown.
void populateTelemetry(rs422::Uplink&, const aviator::Message&, uint64_t now);
// Independent visual measurement; never clamped to the command travel range.
class CameraTelemetry {
  public:
    CameraTelemetry(std::string clock, std::string camera_id = "cockpit", std::string publisher = "camera");
    bool accept(const aviator::Message&, uint64_t now);
    void populate(rs422::Uplink&, uint64_t now) const;
  private:
    std::string camera_id_, publisher_;
    aviator::InputGuard guard_;
    uint64_t sample_ = 0;
    int16_t roll_ = rs422::invalid_position, pitch_ = rs422::invalid_position;
};
} // namespace flight_gateway

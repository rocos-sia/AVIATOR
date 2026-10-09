#pragma once
#include "motion.hpp"
#include <string_view>
namespace aviator {
// Rokae RT feedback starts inside enable(). Initialization must not require enable first.
// Only pre-enable executor phases may use fresh device-status feedback instead of RT samples.
inline bool managedResourcesReady(std::string_view phase,
                                  const DeviceState& device, bool feedback_fresh, bool status_fresh) {
    if (device.fault) return false;
    const bool before_enable = phase == "UNINITIALIZED" || phase == "INITIALIZED" || phase == "DISABLED";
    if (device.impedance_switching)
        return status_fresh && device.enabled[0] && device.enabled[1] && !device.stopping;
    if (feedback_fresh) return before_enable || (device.enabled[0] && device.enabled[1]);
    return status_fresh && !device.stopping && device.id == 0 &&
           before_enable;
}
}

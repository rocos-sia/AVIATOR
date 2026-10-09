#pragma once
#include "device_server.hpp"
namespace aviator {
// Preload robot geometry before enable. Both hardware and simulation use this factory.
void configureDeviceServo(DeviceServerOptions&, const DeviceSettings&);
} // namespace aviator

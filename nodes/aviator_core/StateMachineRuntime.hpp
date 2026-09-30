#pragma once
#include "motion.hpp"
#include <csignal>
namespace aviator {
struct ManagedGatewayOptions {
    bool enabled = false;
    std::string endpoint = "tcp://127.0.0.1:5559";
    std::string session; // Empty: bind the first valid flight.command session once.
};
// Device/console adapter for Aviator managed API. main_sml remains offline only.
int runStateMachine(const MotionConfig&, const volatile std::sig_atomic_t& interrupted,
                    const ManagedGatewayOptions& gateway = {});
}

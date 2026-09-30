#pragma once
#include "motion.hpp"
#include <csignal>
namespace aviator {
int runStateMachine(const MotionConfig&, const std::string& safety_file, bool simulation,
                    const volatile std::sig_atomic_t& interrupted);
}

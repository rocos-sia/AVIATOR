#pragma once
#include "aviator/DataLink.hpp"
#include <cmath>
namespace aviator {
// Software grasp state for a robot with arm feedback only. No external latch is actuated.
// The caller supplies TCP error relative to the commanded wheel pose, never measured wheel data.
class OpenLoopGrasp {
    GraspState state_;
    double aligned_since_ = -1, lost_since_ = -1;
    bool enabled_ = false;
public:
    OpenLoopGrasp() { state_.open_loop = true; }
    void reference(double angle, double displacement) {
        state_.angle = angle; state_.displacement = displacement;
    }
    const GraspState &state() const { return state_; }
    GraspState update(double now, double feedback_time, bool enabled,
                      const double position[2], const double rotation[2], double speed) {
        enabled_ = enabled;
        state_.heartbeat = feedback_time;
        bool aligned = enabled && std::isfinite(speed) && speed < 0.02;
        bool lost = !enabled;
        if (enabled && now - feedback_time > 0.1) state_.fault = 1;
        for (int side = 0; side < 2; ++side) {
            state_.position_error[side] = position[side];
            state_.rotation_error[side] = rotation[side];
            aligned = aligned && position[side] < 0.003 && rotation[side] < 0.035;
            lost = lost || !(position[side] <= 0.02 && rotation[side] <= 0.15);
        }
        if (!aligned) aligned_since_ = -1;
        else if (aligned_since_ < 0) aligned_since_ = now;
        state_.ready = !state_.fault && aligned_since_ >= 0 && now - aligned_since_ >= 0.1;
        if (!state_.locked || !lost) lost_since_ = -1;
        else if (lost_since_ < 0) lost_since_ = now;
        if (state_.locked && lost_since_ >= 0 && now - lost_since_ > 0.1) {
            state_.fault = 1; state_.ready = 0;
        }
        return state_;
    }
    uint64_t command(GraspCommand command) {
        ++state_.ack;
        state_.result = GraspResult::Ok;
        switch (command) {
        case GraspCommand::Lock:
            if (state_.fault) state_.result = GraspResult::Fault;
            else if (!enabled_) state_.result = GraspResult::NotEnabled;
            else if (!state_.ready) state_.result = GraspResult::NotAligned;
            else state_.locked = 3;
            break;
        case GraspCommand::Unlock:
            state_.locked = 0;
            lost_since_ = -1;
            break;
        case GraspCommand::ResetFault:
            if (state_.locked) state_.result = GraspResult::Fault;
            else { state_.fault = 0; state_.ready = 0; aligned_since_ = lost_since_ = -1; }
            break;
        }
        return state_.ack;
    }
};
}

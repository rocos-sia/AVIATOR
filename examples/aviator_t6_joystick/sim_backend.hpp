#pragma once
#include <mujoco/mujoco.h>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace aviator {

using Vec14 = std::array<double, 14>;
using Vec12 = std::array<double, 12>;

// Snapshot of the sim's observable state for the control loop.
struct SimState {
    double theta = 0;       // wheel roll (rad)
    double s = 0;           // wheel slide (m)
    double theta_vel = 0;   // rad/s
    double s_vel = 0;       // m/s
    Vec14 q{};              // 14 arm joint angles (rad)
    double time = 0;        // sim time (s)
    double position_error[2]{};   // |tcp - handle| per side (m)
    double rotation_error[2]{};   // per side (rad)
    bool aligned = false;
    bool fault = false;
    bool locked = false;
};

// In-process MuJoCo sim: qfrc_applied position servo on the 14 arm joints and
// 12 independent hand joints (grip pose), a passive steering wheel, and the
// left/right grasp welds. Adapted from
// examples/AviatorRobot_simple/src/DataLink_direct.cpp (MuJoCoDirectDataLink) —
// same gravity-feedforward + proportional servo, joint damping, and TCP/handle
// alignment + lost-grip detection. This backend also servos the full RH56E2
// hand; callers must check alignment before activating the grasp welds.
class Sim {
  public:
    // hand_grip: 12 independent hand-joint targets, order
    //   [L index_1, L middle_1, L ring_1, L little_1, L thumb_1, L thumb_2,
    //    R index_1, ... ]  (the *_2 / thumb_3 / thumb_4 joints follow via mimic
    //    equality constraints in the MJCF).
    Sim(const std::string& model_path, const Vec12& hand_grip,
        const Vec14& initial_arm_q, bool realtime = true,
        bool start_at_grasp = false);
    ~Sim();

    void setArmTarget(const Vec14& q);
    void lock();        // caller checks alignment before activating welds
    void unlock();
    void resetFault();
    SimState state() const;   // thread-safe snapshot
    double time() const;
    std::mutex& mutex() { return mutex_; }
    mjModel* model() { return m_; }
    mjData* data() { return d_; }

  private:
    void initMapping();
    void updateAlignmentLocked();
    void physicsLoop();

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;

    std::array<int, 14> arm_qadr_{}, arm_dofadr_{};
    std::array<int, 12> hand_qadr_{}, hand_dofadr_{};
    std::array<int, 12> mimic_qadr_{}, mimic_dofadr_{};
    Vec12 mimic_target_{};
    int roll_qadr_ = -1, roll_dofadr_ = -1, pitch_qadr_ = -1, pitch_dofadr_ = -1;
    int tcp_[2]{}, handle_[2]{}, weld_[2]{};

    Vec14 arm_target_{};
    Vec12 hand_grip_{};

    mutable std::mutex mutex_;
    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> realtime_{true};

    double position_error_[2]{}, rotation_error_[2]{};
    double stable_time_ = 0.0;
    double lost_time_ = 0.0;
    bool fault_ = false;
    bool grasp_probe_ = false;
};

}  // namespace aviator

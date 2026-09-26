#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aviator::t6 {

using Vec2 = std::array<double, 2>;
using Joints = std::array<double, 14>;

struct LookupResult {
    Joints q{};
    Vec2 margin_minus{}, margin_plus{};
    double clearance = 0;
    double joint_margin = 0;
    std::uint8_t branch = 0;
};

struct PhaseEstimate {
    Vec2 phi{};
    Vec2 rms_joint_error{}; // radians, one seven-joint RMS per arm
    Vec2 max_joint_error{}; // radians, max absolute error per arm
};

struct PhaseSearch {
    std::optional<Vec2> previous;
    double max_phase_change = std::numeric_limits<double>::infinity();
    double continuity_weight = 0; // penalty in joint-error squared per phase-radian squared
};

// Immutable after construction; all files are checked for the expected size.
class Lookup {
public:
    explicit Lookup(const std::string& directory);
    LookupResult query(Vec2 x, Vec2 phi) const;
    PhaseEstimate estimate_phase(Vec2 x, const Joints& measured_q,
                                 PhaseSearch search = {}) const;
    std::array<double, 4> safe_interval(Vec2 x) const; // Llo,Lhi,Rlo,Rhi
    Vec2 initial_phase(Vec2 x) const;

private:
    struct Grid { int index; double weight; };
    Grid theta(double v) const;
    Grid slide(double v) const;
    Grid phase(double v) const;
    std::size_t point(int s, int th, int p) const;
    std::array<double, 7> joints(const std::vector<float>& data, Grid th, Grid s, Grid p) const;
    double scalar(const std::vector<float>& data, Grid th, Grid s, Grid p) const;
    int n_th_ = 0, n_s_ = 0, n_p_ = 0;
    double th_min_ = 0, th_max_ = 0, s_min_ = 0, s_max_ = 0;
    std::vector<float> phi_, q_l_, q_r_, d_l_, d_r_, safe_;
    std::vector<std::uint8_t> branch_;
};

class Actor {
public:
    explicit Actor(const std::string& onnx_path);
    ~Actor();
    Actor(const Actor&) = delete;
    Actor& operator=(const Actor&) = delete;
    Vec2 predict(const std::array<float, 40>& observation) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

enum class Status { ok, grid_exit, branch, clearance, joint_limit, speed,
                    unsafe_phase, reconstruction_error, phase_jump, timing,
                    invalid_feedback, invalid_target, actor_error };

// Where a rejection occurred. Only policy_command means the RL proposal was
// evaluated and rejected; feedback and reset failures are different events.
enum class CheckStage { none, reset, feedback, target, policy_command };
const char* status_name(Status status);

struct StepResult {
    Status status = Status::grid_exit;
    CheckStage stage = CheckStage::none;
    bool new_failure = false; // false on repeated calls while fault-latched
    bool has_command = false;
    bool has_action = false;
    bool has_evaluated_sample = false;
    Joints q{};
    Vec2 action{}, executed_action{}, phi{}; // raw actor action and checked action
    bool intervened = false;
    Vec2 candidate_x{}, candidate_phi{}; // proposed next task state and phase
    LookupResult evaluated_sample{};     // sample checked at the failure stage
    double clearance = 0, max_joint_speed = 0;
    int limiting_joint = -1; // 0..13 when the speed gate is evaluated
    PhaseEstimate estimate{};
};

struct Feedback {
    Vec2 x{}, xdot{}; // synchronized actual task state and velocity
    Joints q{};       // synchronized measured robot joint angles
    double time = 0; // monotonic seconds
};

struct FeedbackLimits {
    double max_rms_joint_error = 0;
    double max_joint_error = 0;
    double max_phase_change = 0;
    double continuity_weight = 0;
    double max_joint_speed = 0;
    double min_clearance = 0;
    double time_tolerance = 0;
};

// Fail-closed feedback controller. reset() is a global inverse-LUT acquisition;
// each step() uses measured feedback and a bounded inverse-LUT re-acquisition.
// A non-ok result contains no command to send; reset() is then required.
class FeedbackController {
public:
    FeedbackController(const Lookup& lookup, const Actor& actor, FeedbackLimits limits);
    StepResult reset(const Feedback& measured);
    StepResult step(const Feedback& measured, Vec2 x_target_next);
    std::array<float, 40> observation() const;

private:
    std::array<float, 40> make_observation(const Feedback& measured,
                                           const PhaseEstimate& phase) const;
    void shift_history(const Feedback& measured);
    Status check_state(const LookupResult& sample, const PhaseEstimate& phase) const;
    const Lookup& lookup_;
    const Actor& actor_;
    FeedbackLimits limits_;
    bool ready_ = false;
    bool raw_ = false; // AVIATOR_T6_RAW: bypass safety gates, keep non-finite/bounds guards
    Feedback last_{};
    PhaseEstimate last_phase_{};
    Vec2 prev_action_{};
    std::array<Vec2, 21> x_hist_{}, xdot_hist_{}, action_hist_{};
    std::array<float, 40> last_observation_{};
};

// Public one-call interface. The first call acquires phase and returns no
// command. A failure latches until the caller explicitly passes rearm=true.
class T6 {
public:
    T6(const std::string& onnx_path, const std::string& lut_directory,
       FeedbackLimits limits);
    StepResult operator()(const Feedback& measured, Vec2 x_target_next,
                          bool rearm = false);

private:
    Lookup lookup_;
    Actor actor_;
    FeedbackController controller_;
    bool initialized_ = false;
    bool faulted_ = false;
    StepResult last_failure_{};
};

// One instance per trajectory/control stream. The caller supplies measured
// task state x=(theta,s) and xdot each 10 ms, in the same units as training.
class Controller {
public:
    Controller(const Lookup& lookup, const Actor& actor);
    StepResult reset(Vec2 x, Vec2 xdot);
    StepResult reset(Vec2 x, Vec2 xdot, Vec2 registered_phase);
    StepResult step(Vec2 x_next, Vec2 xdot_next);
    std::array<float, 40> observation() const;

private:
    const Lookup& lookup_;
    const Actor& actor_;
    bool ready_ = false;
    Vec2 x_{}, xdot_{}, phi_{}, prev_action_{};
    std::array<Vec2, 21> x_hist_{}, xdot_hist_{}, action_hist_{};
    int hist_count_ = 0;
};

} // namespace aviator::t6

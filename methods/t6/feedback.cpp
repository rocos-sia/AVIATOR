#include "t6.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace aviator::t6 {
namespace {
constexpr double dt = 0.01;
constexpr double phase_rate = 1.5;
constexpr double tol = 1e-9;

bool finite(const Feedback& f) {
    if (!std::isfinite(f.time)) return false;
    for (double v : f.x) if (!std::isfinite(v)) return false;
    for (double v : f.xdot) if (!std::isfinite(v)) return false;
    for (double v : f.q) if (!std::isfinite(v)) return false;
    return true;
}

bool finite_positive(double v) { return std::isfinite(v) && v > 0; }
} // namespace

const char* status_name(Status status) {
    switch (status) {
    case Status::ok: return "ok";
    case Status::grid_exit: return "grid_exit";
    case Status::branch: return "branch";
    case Status::clearance: return "clearance";
    case Status::joint_limit: return "joint_limit";
    case Status::speed: return "speed";
    case Status::unsafe_phase: return "unsafe_phase";
    case Status::reconstruction_error: return "reconstruction_error";
    case Status::phase_jump: return "phase_jump";
    case Status::timing: return "timing";
    case Status::invalid_feedback: return "invalid_feedback";
    case Status::invalid_target: return "invalid_target";
    case Status::actor_error: return "actor_error";
    }
    return "unknown";
}

FeedbackController::FeedbackController(const Lookup& lookup, const Actor& actor,
                                       FeedbackLimits limits)
    : lookup_(lookup), actor_(actor), limits_(limits) {
    if (!finite_positive(limits_.max_rms_joint_error) ||
        !finite_positive(limits_.max_joint_error) ||
        !finite_positive(limits_.max_phase_change) ||
        !finite_positive(limits_.max_joint_speed) ||
        !finite_positive(limits_.min_clearance) ||
        !finite_positive(limits_.time_tolerance) || limits_.time_tolerance >= dt ||
        !std::isfinite(limits_.continuity_weight) || limits_.continuity_weight < 0)
        throw std::invalid_argument("Feedback limits must be finite and explicitly configured");
    const char *raw_env = std::getenv("AVIATOR_T6_RAW");
    raw_ = raw_env && raw_env[0] != '\0' && raw_env[0] != '0';
}

Status FeedbackController::check_state(const LookupResult& sample,
                                        const PhaseEstimate& phase) const {
    if (!raw_) {
        for (int arm = 0; arm < 2; ++arm) {
            if (!std::isfinite(phase.rms_joint_error[arm]) ||
                !std::isfinite(phase.max_joint_error[arm]) ||
                phase.rms_joint_error[arm] > limits_.max_rms_joint_error ||
                phase.max_joint_error[arm] > limits_.max_joint_error)
                return Status::reconstruction_error;
            if (sample.margin_minus[arm] < -tol || sample.margin_plus[arm] < -tol)
                return Status::unsafe_phase;
        }
        if (sample.branch >= 2) return Status::branch;
        if (!std::isfinite(sample.clearance) || sample.clearance < limits_.min_clearance)
            return Status::clearance;
    }
    // Kept in raw mode: out-of-bounds / non-finite guard (joint_margin < 0 means
    // the LUT output crosses a joint limit; non-finite q would crash dispatch).
    if (!std::isfinite(sample.joint_margin) || sample.joint_margin < 0 ||
        !std::all_of(sample.q.begin(), sample.q.end(),
                     [](double q) { return std::isfinite(q); }))
        return Status::joint_limit;
    return Status::ok;
}

std::array<float, 40> FeedbackController::make_observation(
    const Feedback& measured, const PhaseEstimate& phase) const {
    const auto sample = lookup_.query(measured.x, phase.phi);
    std::array<float, 40> o{};
    o[0] = measured.x[0]; o[1] = measured.x[1];
    o[2] = measured.xdot[0]; o[3] = measured.xdot[1];
    o[4] = std::sin(phase.phi[0]); o[5] = std::cos(phase.phi[0]);
    o[6] = std::sin(phase.phi[1]); o[7] = std::cos(phase.phi[1]);
    o[8] = sample.margin_minus[0]; o[9] = sample.margin_minus[1];
    o[10] = sample.margin_plus[0]; o[11] = sample.margin_plus[1];
    o[12] = sample.clearance; o[13] = sample.joint_margin;
    o[14] = prev_action_[0]; o[15] = prev_action_[1];
    constexpr int lags[4] = {2, 5, 10, 20};
    for (int i = 0; i < 4; ++i) {
        const int h = 20 - lags[i];
        for (int k = 0; k < 2; ++k) {
            o[16 + i * 2 + k] = x_hist_[h][k];
            o[24 + i * 2 + k] = xdot_hist_[h][k];
            o[32 + i * 2 + k] = action_hist_[h][k];
        }
    }
    return o;
}

void FeedbackController::shift_history(const Feedback& measured) {
    for (int i = 0; i < 20; ++i) {
        x_hist_[i] = x_hist_[i + 1];
        xdot_hist_[i] = xdot_hist_[i + 1];
        action_hist_[i] = action_hist_[i + 1];
    }
    x_hist_[20] = measured.x;
    xdot_hist_[20] = measured.xdot;
    action_hist_[20] = prev_action_;
}

StepResult FeedbackController::reset(const Feedback& measured) {
    ready_ = false;
    StepResult out;
    out.stage = CheckStage::reset;
    if (!finite(measured)) { out.status = Status::invalid_feedback; return out; }
    LookupResult sample;
    try {
        out.estimate = lookup_.estimate_phase(measured.x, measured.q);
        sample = lookup_.query(measured.x, out.estimate.phi);
    } catch (const std::out_of_range&) {
        out.status = Status::grid_exit; return out;
    }
    out.phi = out.estimate.phi;
    out.clearance = sample.clearance;
    out.evaluated_sample = sample;
    out.has_evaluated_sample = true;
    out.status = check_state(sample, out.estimate);
    if (out.status != Status::ok) return out;
    last_ = measured;
    last_phase_ = out.estimate;
    prev_action_ = {};
    x_hist_.fill(measured.x);
    xdot_hist_.fill(measured.xdot);
    action_hist_.fill({});
    last_observation_ = make_observation(measured, out.estimate);
    ready_ = true;
    return out; // acquisition only: no command on reset
}

std::array<float, 40> FeedbackController::observation() const {
    if (!ready_) throw std::logic_error("Feedback controller is not ready");
    return last_observation_;
}

StepResult FeedbackController::step(const Feedback& measured, Vec2 x_target_next) {
    if (!ready_) throw std::logic_error("Feedback controller requires reset");
    StepResult out;
    out.stage = CheckStage::feedback;
    auto fail = [&](Status status) {
        out.status = status;
        ready_ = false;
        return out;
    };
    if (!finite(measured)) return fail(Status::invalid_feedback);
    if (std::abs(measured.time - last_.time - dt) > limits_.time_tolerance)
        return fail(Status::timing);
    try {
        PhaseSearch search;
        search.previous = last_phase_.phi;
        search.max_phase_change = limits_.max_phase_change;
        search.continuity_weight = limits_.continuity_weight;
        out.estimate = lookup_.estimate_phase(measured.x, measured.q, search);
    } catch (const std::out_of_range&) {
        return fail(Status::phase_jump);
    }
    out.phi = out.estimate.phi;
    LookupResult actual;
    try {
        actual = lookup_.query(measured.x, out.estimate.phi);
    } catch (const std::out_of_range&) {
        return fail(Status::grid_exit);
    }
    out.clearance = actual.clearance;
    out.evaluated_sample = actual;
    out.has_evaluated_sample = true;
    const Status actual_status = check_state(actual, out.estimate);
    if (actual_status != Status::ok) return fail(actual_status);
    if (!std::isfinite(x_target_next[0]) || !std::isfinite(x_target_next[1])) {
        out.stage = CheckStage::target;
        out.candidate_x = x_target_next;
        return fail(Status::invalid_target);
    }
    shift_history(measured);
    last_observation_ = make_observation(measured, out.estimate);
    out.stage = CheckStage::policy_command;
    out.candidate_x = x_target_next;
    try {
        out.action = actor_.predict(last_observation_);
    } catch (const std::exception&) {
        return fail(Status::actor_error);
    }
    out.has_action = true;
    const Vec2 target_phi = {
        out.estimate.phi[0] + out.action[0] * phase_rate * dt,
        out.estimate.phi[1] + out.action[1] * phase_rate * dt};
    out.candidate_phi = target_phi;
    if (raw_) {
        // Raw path: dispatch the RL action unchanged (scale=1), no intervention
        // loop and no speed gate. Only the LUT grid_exit + joint_limit guards apply.
        StepResult trial = out;
        trial.executed_action = out.action;
        trial.phi = target_phi;
        LookupResult next;
        try {
            next = lookup_.query(x_target_next, trial.phi);
        } catch (const std::out_of_range&) {
            return fail(Status::grid_exit);
        }
        trial.evaluated_sample = next;
        trial.has_evaluated_sample = true;
        trial.clearance = next.clearance;
        trial.status = check_state(next, PhaseEstimate{});
        if (trial.status != Status::ok) return fail(trial.status);
        trial.q = next.q;
        trial.has_command = true;
        last_ = measured;
        last_phase_ = trial.estimate;
        prev_action_ = trial.executed_action;
        return trial;
    }
    constexpr double scales[] = {1, .5, .25, .125, .0625, .03125, .015625, 0};
    StepResult nominal_failure = out;
    for (double scale : scales) {
        StepResult trial = out;
        trial.intervened = scale < 1;
        trial.executed_action = {out.action[0] * scale, out.action[1] * scale};
        trial.phi = {out.estimate.phi[0] + trial.executed_action[0] * phase_rate * dt,
                     out.estimate.phi[1] + trial.executed_action[1] * phase_rate * dt};
        trial.has_evaluated_sample = false;
        LookupResult next;
        try {
            next = lookup_.query(x_target_next, trial.phi);
        } catch (const std::out_of_range&) {
            trial.status = Status::grid_exit;
            if (scale == 1) nominal_failure = trial;
            continue;
        }
        trial.evaluated_sample = next;
        trial.has_evaluated_sample = true;
        trial.clearance = next.clearance;
        trial.status = check_state(next, PhaseEstimate{});
        if (trial.status == Status::ok) {
            for (int j = 0; j < 14; ++j) {
                const double speed = std::abs(next.q[j] - measured.q[j]) / dt;
                if (speed > trial.max_joint_speed) {
                    trial.max_joint_speed = speed;
                    trial.limiting_joint = j;
                }
            }
            if (trial.max_joint_speed > limits_.max_joint_speed)
                trial.status = Status::speed;
        }
        if (scale == 1) nominal_failure = trial;
        if (trial.status != Status::ok) continue;
        trial.q = next.q;
        trial.has_command = true;
        last_ = measured;
        last_phase_ = trial.estimate;
        prev_action_ = trial.executed_action;
        return trial;
    }
    out = nominal_failure;
    return fail(out.status);
}

T6::T6(const std::string& onnx_path, const std::string& lut_directory,
       FeedbackLimits limits)
    : lookup_(lut_directory), actor_(onnx_path), controller_(lookup_, actor_, limits) {}

StepResult T6::operator()(const Feedback& measured, Vec2 x_target_next, bool rearm) {
    if (rearm || (!initialized_ && !faulted_)) {
        auto result = controller_.reset(measured);
        initialized_ = result.status == Status::ok;
        faulted_ = !initialized_;
        result.new_failure = faulted_;
        if (faulted_) last_failure_ = result;
        return result;
    }
    if (faulted_) {
        auto result = last_failure_;
        result.new_failure = false;
        return result;
    }
    auto result = controller_.step(measured, x_target_next);
    if (result.status != Status::ok) {
        result.new_failure = true;
        faulted_ = true;
        last_failure_ = result;
    }
    return result;
}

} // namespace aviator::t6

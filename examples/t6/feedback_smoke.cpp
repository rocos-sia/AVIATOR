#include "t6.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace t6 = aviator::t6;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: t6_feedback_smoke MODEL.onnx LUT_DIRECTORY\n";
        return 2;
    }
    try {
        const t6::Vec2 x{0, -0.08};
        t6::Lookup lookup(argv[2]);
        t6::FeedbackLimits limits;
        limits.max_rms_joint_error = 0.02; // illustrative test values only
        limits.max_joint_error = 0.05;
        limits.max_phase_change = 0.025;
        limits.continuity_weight = 0.001;
        limits.max_joint_speed = 1.5;
        limits.min_clearance = 0.005;
        limits.time_tolerance = 0.002;
        t6::T6 run(argv[1], argv[2], limits);

        t6::Feedback feedback;
        feedback.x = x;
        feedback.q = lookup.query(x, {0, 0}).q;
        const auto initial_q = feedback.q;
        auto initial = run(feedback, x);
        require(initial.status == t6::Status::ok && !initial.has_command,
                "First call must acquire phase without a command");
        require(std::abs(initial.estimate.phi[0]) < 1e-6 &&
                std::abs(initial.estimate.phi[1]) < 1e-6,
                "Initial inverse-LUT phase is wrong");

        feedback.time = 0.01;
        auto command = run(feedback, x);
        require(command.status == t6::Status::ok && command.has_command,
                "Healthy feedback did not produce a command");
        std::cout << "healthy action=" << command.action[0] << "," << command.action[1]
                  << " rms=" << command.estimate.rms_joint_error[0] << ","
                  << command.estimate.rms_joint_error[1] << "\n";

        feedback.time = 0.02;
        feedback.q = command.q; // ideal tracking of prior command
        auto tracked = run(feedback, x);
        require(tracked.status == t6::Status::ok && tracked.has_command,
                "Closed-loop phase re-acquisition failed");
        require(std::abs(tracked.estimate.phi[0] - command.phi[0]) < 1e-4,
                "Measured phase did not follow previous command");

        auto reduced_limits = limits;
        double nominal_step_speed = 0;
        for (int j = 0; j < 14; ++j)
            nominal_step_speed = std::max(nominal_step_speed,
                std::abs(command.q[j] - initial_q[j]) / 0.01);
        require(nominal_step_speed > 0, "Policy produced no testable joint step");
        reduced_limits.max_joint_speed = nominal_step_speed * 0.75;
        t6::T6 reduced(argv[1], argv[2], reduced_limits);
        t6::Feedback reduced_feedback;
        reduced_feedback.x = x;
        reduced_feedback.q = initial_q;
        require(reduced(reduced_feedback, x).status == t6::Status::ok,
                "Reduced-speed controller could not acquire phase");
        reduced_feedback.time = 0.01;
        auto protected_step = reduced(reduced_feedback, x);
        require(protected_step.has_command && protected_step.intervened &&
                protected_step.max_joint_speed <= reduced_limits.max_joint_speed &&
                std::abs(protected_step.executed_action[0]) <
                    std::abs(protected_step.action[0]),
                "Unsafe raw policy action was not reduced before execution");

        feedback.time = 0.03;
        feedback.q = tracked.q;
        feedback.q[0] += 0.3;
        auto bad = run(feedback, x);
        require(bad.status == t6::Status::reconstruction_error && !bad.has_command,
                "Off-manifold feedback was not rejected");
        require(bad.stage == t6::CheckStage::feedback && !bad.has_action && bad.new_failure &&
                bad.has_evaluated_sample,
                "Feedback rejection was mislabeled as a policy rejection");
        auto latched = run(feedback, x);
        require(latched.status == bad.status && !latched.has_command && !latched.new_failure,
                "Failure did not latch");

        feedback.q = lookup.query(x, {0, 0}).q;
        auto rearmed = run(feedback, x, true);
        require(rearmed.status == t6::Status::ok && !rearmed.has_command,
                "Explicit rearm failed");
        feedback.time = 0.05;
        auto late = run(feedback, x);
        require(late.status == t6::Status::timing && !late.has_command,
                "Feedback timing violation was not rejected");
        require(run(feedback, x, true).status == t6::Status::ok,
                "Second rearm failed");
        feedback.time = 0.06;
        auto infeasible = run(feedback, {100, -0.08});
        require(infeasible.status == t6::Status::grid_exit && !infeasible.has_command &&
                infeasible.new_failure && infeasible.has_action &&
                infeasible.stage == t6::CheckStage::policy_command &&
                !infeasible.has_evaluated_sample &&
                infeasible.candidate_x[0] == 100 &&
                std::isfinite(infeasible.candidate_phi[0]) &&
                std::isfinite(infeasible.candidate_phi[1]),
                "Infeasible policy proposal was not recorded");
        require(!run(feedback, x).new_failure,
                "Latched failure was incorrectly reported as a new event");
        require(run(feedback, x, true).status == t6::Status::ok,
                "Third rearm failed");
        feedback.time = 0.07;
        auto invalid_target = run(feedback, {NAN, -0.08});
        require(invalid_target.status == t6::Status::invalid_target &&
                invalid_target.stage == t6::CheckStage::target &&
                !invalid_target.has_action && !invalid_target.has_command,
                "Invalid target was not rejected before inference");
        std::cout << "reconstruction, latch, rearm, and timing gates passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}

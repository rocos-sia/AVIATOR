# N17 protocol: velocity-damper and full/reduced QP comparison

## Question

N16 showed that the endpoint-aware online governor is more robust than the
stale table under the tested installation errors, but it did not isolate which
part of the online method is responsible. N17 therefore compares the frozen
reduced controller with two representation controls and a classical local
velocity-damper control.

The node does **not** run rebuilt LUT, RL, TSR, or GSR. Rebuilt-LUT is omitted
because the purpose of N16 is precisely to test errors that are not known well
enough to rebuild a table.

## Frozen inputs

- Paired states and perturbed models: the N16 formal raw JSON
  `outputs/differential_lift_feasibility_20261011/n16_installation_error_formal/n16_installation_error_raw.json`.
- `dt=0.005`, `steps=100`, `d_safe=0.005 m`, `beta_stop=0.125`.
- Swept checker subdivisions: 5.
- Online reduced controller margin: `m_curv=0.50 mm`, and the frozen N8
  endpoint-aware schedule.
- All methods use the same MuJoCo model, initial `(x,q,phi)`, task velocity,
  endpoint checker, swept checker, and joint limits. The reduced governor and
  velocity-damper control use midpoint re-solving; the full-QP control uses
  the existing canonical `full_joint_qp_step` directly. This integrator
  difference is recorded rather than hidden, because the full-QP implementation
  is an existing baseline and the velocity-damper midpoint correction avoids
  measuring trivial Euler closure drift.

## Methods

1. `online_endpoint_governor`: frozen reduced action
   `y=[r_L,r_R,beta]`, using the N8 schedule and independent nonlinear
   endpoint/swept acceptance.
2. `full_joint_qp_governor`: the same schedule, thresholds, margin, and
   checker, but the candidate is solved directly in `(qdot,beta)` with all 14
   joint velocities. This is a full joint-space equality-constrained QP
   representation control; it is not called hierarchical unless separate
   priority levels are implemented.
3. `velocity_damper`: a classical full joint-space velocity-damper QP with
   fixed task scale and no task-scale backtracking. It uses the frozen
   `eta=5 s^-1` damping inequality
   `d_dot + eta (d-d_safe) >= 0`, then the same nonlinear checker.
4. `velocity_damper_governed`: the same velocity-damper QP with task-scale
   candidates `(1, 0.75, 0.5, 0.25, 0.125)` and no phase-scale candidates.

The velocity-damper variants use no curvature margin in their linear rate
inequality. This is recorded explicitly so that safety differences are not
silently attributed to representation. The independent nonlinear checker is
the acceptance authority for every method.

## Primary comparisons

- `velocity_damper` versus `velocity_damper_governed`: isolates the value of
  task-speed backtracking in a classical local avoidance controller.
- `velocity_damper_governed` versus `online_endpoint_governor`: tests whether
  a classical governed full-space controller reproduces the proposed method's
  safety/progress behavior.
- `full_joint_qp_governor` versus `online_endpoint_governor`: isolates the
  reduced `[r_L,r_R,beta]` representation while holding the governor and
  nonlinear acceptance protocol fixed.

## Reported metrics

For each paired trial and method: completion, accepted steps, task progress,
stop reason, endpoint/swept minimum clearance, closure residual, minimum
joint-limit margin, minimum singular value of `A_q`, governor attempts,
governed steps, solver failures, endpoint rejections, and per-step latency
mean/P95/P99. The report also gives paired differences and a solver-status
audit. A solver failure is never counted as a branch jump.

## Interpretation boundary

- If `velocity_damper_governed` is close to the reduced method, most measured
  gain may come from task-speed backtracking and nonlinear acceptance rather
  than the reduced representation.
- If it fails while the reduced method succeeds under the same paired states,
  the phase-coupled reduced lift contributes beyond generic velocity damping.
- If full and reduced QP match in safety/progress but full QP is slower, the
  reduced representation's evidence is computational/maintenance-related,
  not a claim of unique safety capability.
- A positive result remains conditional on these tested local states and
  installation-error models; it is not a global safety or IK-branch theorem.

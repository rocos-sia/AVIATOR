# N17 velocity-damper and full/reduced QP comparison

Raw input: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261011/n17_velocity_qp_formal_v2/n17_velocity_damper_full_qp_raw.json` (SHA-256 `aab52f8803eb23c49ad6d39eb88b4c01a8393480d1c57f9c5f86273bb2528307`).
Paired groups: **48**; rows: **192**; instrumentation_unknown: **0**.

## Summary

| method | completion | median progress | median min clearance (mm) | median max closure | median P95 step (ms) | stop reasons |
|---|---:|---:|---:|---:|---:|---|
| online_endpoint_governor | 100.0% | 0.9997 | 5.222 | 3.87e-05 | 69.3 | complete:48 |
| full_joint_qp_governor | 100.0% | 0.9997 | 5.222 | 7.55e-05 | 69.2 | complete:48 |
| velocity_damper | 20.8% | 0.4650 | 6.844 | 0.0137 | 129.0 | complete:10, velocity_damper_endpoint_validation:16, velocity_damper_midpoint_infeasible:22 |
| velocity_damper_governed | 54.2% | 0.6675 | 5.433 | 0.014 | 305.3 | complete:26, velocity_damper_endpoint_validation:10, velocity_damper_midpoint_infeasible:12 |

## Interpretation

The velocity-damper rows are a classical local rate-damper control with `eta=5 s^-1`; the governed variant adds task-speed backtracking but no phase-scale search.
The full-QP rows use the existing 14-joint equality-constrained QP, while the reduced rows use the frozen `[r_L,r_R,beta]` governor. Every method uses the independent nonlinear endpoint/swept checker.
Solver failure, endpoint rejection, or closure-residual failure is not a branch-jump observation.

## Files

- `n17_trial_summary.csv`
- `n17_pairwise_summary.csv`
- `n17_stop_reason_summary.csv`
- `n17_latency_summary.csv`
- `n17_summary.json`

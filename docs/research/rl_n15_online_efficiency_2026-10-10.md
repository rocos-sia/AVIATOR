# RL-N15 Online Efficiency (2026-10-10)

## Scope

This node measures deterministic RL phase scheduling against fixed-phase and greedy-phase baselines on the same 50 validation trajectories. It evaluates simulation-side latency and the unshielded manifold environment only.

The current `AviatorManifoldEnv` performs manifold lookup and kinematic termination checks, but does not invoke the N7/N8 nonlinear endpoint/swept checker or endpoint-aware governor. Those stages were not skipped in a claimed full pipeline; they were unavailable and are marked as such in every raw row. Therefore this node cannot establish full endpoint-aware closed-loop speedup.

The compatibility probe is recorded in `outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_safety_stage_compatibility.json`. Under the frozen `serl_clean` interpreter, MuJoCo 2.3.7 cannot load the frozen model because of its `weld site1` schema. The separate `mujo` interpreter (MuJoCo 3.3.6) can load it, but mixing that interpreter into this timing run would violate the frozen N15 runtime. This is why the nonlinear stages are reported as unavailable rather than timed through a different environment.

## Timing protocol

- Validation episodes per method/seed: 50
- Warm-up steps discarded from timing: 100
- Environment dt: 0.01 s
- RL action: deterministic SAC checkpoint at step 60000 (`argmax=True`)
- Checkpoint continuation/training: none
- Device: cuda:0
- Cold-start/XLA timings are recorded separately in `n15_cold_start.json` and excluded from formal P95/P99.

## Measured results

| method | seed | completion | clear. fail | speed fail | cycle P95 (ms) | cycle P99 (ms) | control Hz (P95) |
|---|---:|---:|---:|---:|---:|---:|---:|
| RL_seed_1 | 1 | 96.0% | 0.0% | 4.0% | 1.556 | 1.958 | 642.8 |
| RL_seed_2 | 2 | 98.0% | 0.0% | 2.0% | 1.573 | 2.391 | 635.7 |
| fixed_phase | 1 | 2.0% | 96.0% | 2.0% | 1.085 | 1.103 | 921.7 |
| fixed_phase | 2 | 2.0% | 96.0% | 2.0% | 1.086 | 1.118 | 920.5 |
| greedy_phase | 1 | 42.0% | 0.0% | 58.0% | 1.131 | 1.170 | 884.0 |
| greedy_phase | 2 | 42.0% | 0.0% | 58.0% | 1.129 | 1.151 | 885.8 |

## Interpretation

The measured cycle includes policy/action processing and `env.step`, which contains the lookup, reward, observation, and kinematic checks. The endpoint checker and governor columns are `null` because the frozen RL runner does not expose those stages. Accordingly, any lower RL latency is evidence only about the simulation-side unshielded cycle; it is not evidence that the full endpoint-aware system is faster.

The machine-readable efficiency gate is `summary['efficiency_gate']['status']`. The gate is intentionally false when the full safety stages are unavailable, even if the unshielded timing is lower.

In the measured unshielded cycle, RL P95 latency is 1.556--1.573 ms, compared with 1.085--1.086 ms for fixed phase and 1.129--1.131 ms for greedy phase. Thus this run provides no evidence of a lower total computational latency for RL. RL does have much higher completion on these trajectories, but the large completion difference means the predeclared efficiency comparison is not a matched quality comparison either.

## Artifacts

- Raw JSON: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_latency_raw.json`
- Per-step timing: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_latency_per_step.csv`
- Online execution summary: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_online_execution_summary.csv`
- Baseline comparison: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_baseline_comparison.csv`
- Safety-stage compatibility: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_safety_stage_compatibility.json`
- Cold-start timing: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_cold_start.json`
- Latency decomposition: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_latency_decomposition.csv`
- Safe-stop summary: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_safe_stop_summary.csv`


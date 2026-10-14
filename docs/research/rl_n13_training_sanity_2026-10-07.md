# RL-N13 training sanity result

## Decision

N13 is complete as a negative training gate. The three-seed RLPD processes
started and wrote checkpoints, but the policy did not obtain a useful
long-horizon training distribution. Formal N14 paired policy comparison is
blocked.

## Frozen evaluation

- Action: `[r_L,r_R]`.
- Task path: fixed nominal trajectory.
- Manifold: `manifold_phi_stale_pitch-10deg`.
- Seeds: `0,1,2`.
- Training cap: 20,000 learner/actor steps; 16 online episodes; 2 actor envs.
- Evaluation: 50 validation trajectories in the unshielded `AviatorManifoldEnv`.
- No N1--N10 result was modified.

## Observed results

| seed | checkpoint | online transitions | RL completion | main terminations |
|---:|---:|---:|---:|---|
| 0 | 67 | 67 | 0/50 | 60% grid exit, 40% speed |
| 1 | 51 | 51 | 0/50 | 66% grid exit, 34% speed |
| 2 | 103 | 103 | 0/50 | 60% grid exit, 40% speed |

The same stale manifold and unshielded environment gave `fixed_phase=1/50`
and `greedy_phase=21/50`; greedy had a 58% speed-termination rate. The
stale-compatible greedy replay prior also contained 7 one-step failures among
16 generated trajectories.

## Cause

The training environment does not provide a sufficiently valid exploration
distribution for this stale manifold. The 1.5 rad/s action scale and hard
termination on phase-grid or joint-speed violations end most episodes before
the actor collects long-horizon experience. This is a training-environment and
data-coverage failure, not evidence against the two-dimensional action
parameterization and not a safety result.

## Required repair before N14

Run a separately frozen RL-N13R node. It may add either a feasible-start
curriculum or a training-only action shield, but it must:

1. keep the deployment endpoint/swept checker unchanged;
2. log raw and applied actions separately;
3. use held-out validation states;
4. avoid changing `d_safe`, `beta_stop`, `m_curv`, or N7/N8 data;
5. compare the repaired training environment against the unshielded result.

No RL performance claim is added to the paper at this stage.

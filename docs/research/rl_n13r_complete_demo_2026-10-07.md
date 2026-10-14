# RL-N13R: complete demonstration transitions and convergence repair

## Question

Can complete stale-manifold trajectories be written as per-step transitions
for the independent RLPD `demo_buffer`, and does a better replay prior make the
two-dimensional policy train stably?

## Demonstration protocol

The generator checked 66 `rl_train` trajectories on the stale manifold and
retained 64 that reached the environment's `termination="end"`. Each retained
file is a list of transitions with the same schema as the earlier DP demos:

```text
observations, actions, next_observations, rewards, masks, dones
```

The terminal rule is explicit:

- all intermediate transitions: `dones=False`, `masks=1`;
- final transition: `dones=True`, `masks=0`;
- the final step is accepted whether Gym reports `terminated` or `truncated`.

The learner loads every transition into a separate `demo_buffer`. The online
actor continues writing to `replay_buffer`; the two stores are never merged.
The 64 retained files contain 64,595 transitions.

## Training repair

The first long run used a BC warm-start and the complete demos as both the
ordinary replay prior and a fixed extra prior. The run used one seed, 60,000
learner steps, and 60,000 online transitions. Checkpoints were evaluated on 20
held-out validation trajectories.

The curve was:

| checkpoint | completion | clearance stop | speed stop |
|---:|---:|---:|---:|
| 10k | 95% | 5% | 0% |
| 20k | 90% | 5% | 5% |
| 30k | 90% | 5% | 5% |
| 40k | 95% | 0% | 5% |
| 50k | 20% | 75% | 5% |
| 60k | 95% | 0% | 5% |

The final 60k checkpoint evaluated on 50 held-out trajectories reached 94%
completion, 4% clearance termination, and 2% speed termination. It had no
grid, branch, or joint-limit terminations.

## Interpretation

The transition repair is correct and materially improves the replay prior.
BC warm-start plus a fixed extra prior also makes the final checkpoint useful.
However, the 50k collapse demonstrates non-monotonic training, so this node
does not establish strict stable convergence. The result is suitable for a
repair pilot and checkpoint-selection study, not yet for the formal paired
policy comparison or an RL performance claim in the paper.

## Files

- Generator: `/home/rocos/sia/AVIATOR/hil-serl/tools/run_rl_n13r_complete_demo.py`
- Audit: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261007/rl_n13r_complete_demo/complete_demo_audit.json`
- Training run: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/train_seed_0/`
- Convergence curve: `convergence_eval.json`
- 50-trajectory final evaluation: `final_heldout_50.json`

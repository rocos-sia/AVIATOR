# Continuous phase registration test (2026-09-23)

The 97 DP demonstrations were registered to the existing RL manifold phase
axis with [the sequence registration tool](../tools/register_dp_phase.py).
The search minimizes the worst joint reconstruction error over each **whole
trajectory**, then cumulative squared error and phase movement. Every edge
obeys `|phi[t+1]-phi[t]| <= 1.5 * (t[t+1]-t[t])`; it uses the RL environment's
ordinary phase axis, with no periodic action wrap. Both arms are registered
independently because the lookup has one phase coordinate per arm.

Two diagnostic modes were tested:

1. **Representation:** only the grid and action limit constrain phase. This
   asks whether the LUT chart can continuously reproduce DP configurations.
2. **Environment feasible:** also requires the LUT safe-phase interval,
   interpolated `d >= 5 mm`, and `branch == 0` grid exclusion flag. This is
   the state region currently accepted by the RL environment.

The native 128-point phase grid gave 97/97 complete representation paths.
The maximum reconstruction error per trajectory had a median of 0.0161 rad
for the left arm (0.00338 rad for the right arm). Only 50/97 trajectories
had complete paths after current environment feasibility conditions were
added; 47 met excluded grid cells. The 50 feasible paths had minimum
trajectory-level maximum joint error of 0.0256 rad, with median left and
right maxima of 0.0849 and 0.0526 rad. Thus completing a path in the
environment's accepted phase region does not preserve the DP configuration.

Actions were recomputed from the registered phase differences, and the
environment was reset to each path's registered initial phase. The actual
replay results were:

| Registration mode | Complete | First failure |
| --- | ---: | --- |
| Representation | 0/97 | 90 clearance, 7 grid exclusion |
| Environment feasible | 33/97 | 47 no complete registration, 17 speed |

The 33 successful environment replays still had a median **maximum** joint
reconstruction error of 0.0839 rad. They cannot be relabeled as faithful DP
demonstrations. They are diagnostic paths only, and no demonstration PKL was
replaced. The existing launcher gate continues to block RLPD.

To check phase discretization, the same search was repeated with four linear
subcells per native phase interval (509 phase candidates). The number of
environment-feasible complete paths stayed **50/97**. The median trajectory
maximum reconstruction error for the unrestricted representation path stayed
0.0161 rad; only 22/97 paths had a maximum error at or below 0.005 rad. The
median trajectory maximum error among feasible paths fell only from 0.0914
to 0.0886 rad. The refined paths replayed to the end in 35/97 cases (47 had
no complete registration and 15 terminated on joint speed). The obstacle is
therefore not explained by the native 128-point phase resolution.

The earlier 26,696 LUT false clearance failures were measured on **the same
LUT configuration**, comparing interpolated distance with the physical C++
distance. Therefore phase mismatch alone does not explain that count. There
are two distinct problems: different phase coordinates, and an unreliable
interpolated distance outside the LUT safe interval. The DP physical distance
itself was already reproduced to numerical precision.

Reproduce from `hil-serl/` after activating `serl_clean`:

```bash
PYTHONPATH=. python tools/register_dp_phase.py
PYTHONPATH=. python tools/audit_registered_phase.py --mode representation
PYTHONPATH=. python tools/audit_registered_phase.py --mode feasible
PYTHONPATH=. python tools/register_dp_phase.py --phase-subdiv 4 --output-dir data/aviator/phase_registration_sub4
PYTHONPATH=. python tools/audit_registered_phase.py --registration-dir data/aviator/phase_registration_sub4 --mode feasible
```

The generated per-trajectory arrays and CSV/JSON summaries are under
`data/aviator/phase_registration/`. The RL observation and branch state were
not expanded; the evidence does not identify a new IK branch identity.

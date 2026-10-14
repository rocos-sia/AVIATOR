# N17 audit and paper-boundary decision

## Dataset and coverage

The formal N17 run contains 48 paired groups: the 16 N16 installation-error
states (four error families and four levels) at three task speeds. Each group
has four method rows, so the raw file contains 192 rows. The initial state,
perturbed MuJoCo model, task speed, checker, and step budget are identical
within each group. `instrumentation_unknown=0`, no duplicate method key exists,
and the N16 raw-input hash recorded in the N17 file matches the current N16
file.

This is broad error-family coverage with one sampled initial state per
error-level cell. It is sufficient as a controlled mechanism comparison, but
it is not a 10-replicate success-rate estimate. Replication should be added if
the result is used as a primary statistical claim.

## Results

| method | complete | median progress | median minimum clearance | median max closure | median step P95 |
|---|---:|---:|---:|---:|---:|
| online endpoint governor | 48/48 (100%) | 0.9997 | 5.222 mm | 3.87e-5 | 69.34 ms |
| full joint-space QP governor | 48/48 (100%) | 0.9997 | 5.222 mm | 7.55e-5 | 69.18 ms |
| velocity damper | 10/48 (20.8%) | 0.4650 | 6.844 mm | 1.37e-2 | 129.01 ms |
| velocity damper + task governor | 26/48 (54.2%) | 0.6675 | 5.433 mm | 1.40e-2 | 305.32 ms |

The classical velocity damper had 16 endpoint-validation stops and 22
midpoint-infeasible stops. Adding task-speed backtracking improved completion
from 20.8% to 54.2%, but did not approach the reduced governor and increased
the median P95 cycle latency. The velocity-damper rows also accumulated a
closure residual around `1.4e-2` in the mixed pose norm, despite passing the
clearance checker on accepted steps. This separates local obstacle clearance
from maintaining the closed-chain task manifold.

The reduced and full-QP governors both completed all 48 groups and had nearly
identical progress and clearance. Their median P95 latency differed by about
0.16 ms, which is not evidence that the reduced QP is faster in this Python
implementation. The full-QP closure residual was slightly larger but had the
same high-level completion result.

## What N17 supports

1. A classical velocity damper plus task-speed scaling does not reproduce the
   observed online-governor behavior on these closed-chain installation-error
   states. The measured difference is not only “slowing down”: the baseline
   remains vulnerable to closure drift and infeasible midpoint continuation.
2. The endpoint-aware governor is a system-level composition of differential
   closure lift, phase-coupled reduced action, candidate backtracking, and an
   independent nonlinear endpoint/swept checker. N17 supports this combined
   mechanism claim on the tested envelope.
3. The full joint-space QP is a strong representation control. Its agreement
   with the reduced QP supports numerical consistency and shows that current
   safety/progress gains are not proven to arise from dimensionality reduction
   alone.

## What N17 does not support

- It does not prove that the reduced QP is faster; the measured latency is
  essentially tied in this runner.
- It does not prove a universal advantage over every velocity-damper design.
  The damper coefficient `eta=5 s^-1`, midpoint re-solve, and no curvature
  margin are frozen protocol choices.
- It does not establish a formal continuous-time safety guarantee, global IK
  branch preservation, or a global topology result.
- It does not justify calling the current full-QP implementation a
  hierarchical QP with multiple lexicographic priority levels. It is an
  equality-constrained full joint-space QP baseline.

## Paper wording decision

Keep the innovation at the level of a **closed-chain, endpoint-aware online
continuation architecture**. Describe the reduced representation as a
structured implementation and reuse mechanism, not as a demonstrated runtime
winner. State explicitly that classical velocity damping handles local
clearance but does not by itself reproduce the closed-chain continuation and
acceptance behavior measured here.

The N17 result is conditional on the N16 paired states, four installation-error
families, three speeds, `dt=0.005`, the frozen checker, and the tested local
model. It is not branch-jump evidence: solver failures, endpoint rejection,
and closure-residual growth are reported as execution outcomes.

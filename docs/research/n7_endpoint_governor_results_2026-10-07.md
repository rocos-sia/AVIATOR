# N7 Endpoint-aware governor and matched LUT results

## Scope

N7 implemented a frozen endpoint-aware backtracking governor. At each step the reduced QP was re-solved over the predeclared task/phase scale schedule

\[
\{(1,1),(0.75,1),(0.5,1),(0.25,1),(0.125,1),
(1,0.75),(1,0.5),(1,0.25),(1,0.125),
(0.75,0.75),(0.5,0.5),(0.25,0.25),(0.125,0.125)\}.
\]

The first candidate that passed the independent nonlinear endpoint and swept checker was accepted. If all candidates failed, the trial stopped. The schedule, `beta_stop=0.125`, `d_safe=5` mm, `dt=0.005`, and all numerical budgets were frozen before the formal runs.

The same 90 initial states were replayed through:

1. stale safe-scaled LUT;
2. rebuilt safe-scaled LUT;
3. the existing online lift with the linear QP and nonlinear checker;
4. the endpoint-aware online governor.

The state manifest hash is:

```text
09ee436d3f7ec423368c435f5f94ca6e5b0ca6944324c2271a5f553270b39f5c
```

## Important metric definition

`complete` means that 100 candidate integration steps were accepted. It does not by itself mean that the nominal task displacement was reached, because governor scaling may reduce task speed. Task progress is therefore reported separately.

## Formal results

| Linear margin | Method | 100-step completion | Median progress | P05 progress | Median step latency |
|---|---|---:|---:|---:|---:|
| 0 mm | Online lift | 42/90 (46.7%) | 0.950 | 0.349 | 66.27 ms |
| 0 mm | Endpoint-aware governor | 49/90 (54.4%) | 0.993 | 0.430 | 74.05 ms |
| 0.25 mm | Online lift | 45/90 (50.0%) | 0.989 | 0.420 | 66.50 ms |
| 0.25 mm | Endpoint-aware governor | 90/90 (100%) | 1.000 | 0.918 | 67.92 ms |
| 0.50 mm | Online lift | 88/90 (97.8%) | 1.000 | 0.998 | 67.52 ms |
| 0.50 mm | Endpoint-aware governor | 90/90 (100%) | 1.000 | 0.998 | 67.42 ms |
| 0.50 mm | Rebuilt LUT | 70/90 (77.8%) | 1.000 | 0.250 | 10.51 ms |
| 0.50 mm | Stale LUT | 52/90 (57.8%) | 1.000 | 0.130 | 10.50 ms |

At zero margin, the governor recovered 7 trials that failed with the existing online method and caused no regression among the 90 paired trials. At 0.25 mm it recovered 45 trials. At 0.50 mm it recovered the remaining two baseline failures. No accepted governor step violated the endpoint or swept clearance threshold; `instrumentation_unknown=0`.

The 0.25-mm governor used more aggressive backtracking and had a p95 latency of approximately 168 ms. The 0.50-mm governor used fewer retries, had p95 latency near 68 ms, and retained P05 progress near 0.998. This makes 0.50 mm a candidate for a new frozen protocol, not a retroactively selected universal parameter.

## Interpretation

N7 provides stronger evidence for the mechanism that was missing in N6:

> A linear QP candidate can be recovered by finite-step task/phase backtracking before declaring a safety stop.

The evidence supports the following conditional statement for the tested MuJoCo scenes, speeds, and `dt=0.005`:

> Endpoint-aware backtracking combined with a predeclared linear clearance margin improves accepted progress and reduces premature safety stops while retaining independent nonlinear acceptance.

The result does not establish a universal curvature bound. The margin is still an empirical finite-step design parameter. N4 already showed that finite-step behavior changes with `dt`; N7 did not yet repeat the governor comparison at other step sizes.

The matched state-manifest comparison also shows a useful trade-off. In this protocol the 0.50-mm endpoint-aware online method completed all 90 states, while the rebuilt LUT completed 70/90, but the online method cost about 67 ms per step versus about 10.5 ms for the LUT. The result supports a safety/progress advantage under the tested finite-step policy, not a runtime or total-cost advantage.

## What changes in the paper

1. The governor can be promoted from a conditional description to an experimentally supported algorithm component: QP candidate, nonlinear checker, finite backtracking, then safe stop.
2. The clearance margin should be described as an empirical finite-step margin. Do not claim a universal curvature guarantee.
3. Report completion and task progress separately. A 100-step accepted run with reduced task rate is not equivalent to nominal full-speed completion.
4. Keep the global IK branch and topology claims excluded. N7 does not provide branch-jump evidence.
5. Report the latency trade-off against stale and rebuilt LUTs.

## Limitations and next node

- Rebuilt LUT rows use the same common `q0` state manifest. A separate warm-start-to-rebuilt-table study is still needed before making a production LUT claim.
- N7 uses `dt=0.005` only. The next validation should freeze the governor and margin and test `dt` sensitivity, especially `dt=0.010`.
- Physical execution remains untested.

Next node: **N8 finite-step robustness and frozen-parameter confirmation**. Freeze `m_curv=0.50` mm and the endpoint-aware schedule, test multiple `dt` values and held-out seeds, and do not change the paper claim until that result is available.


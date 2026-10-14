# N8 finite-step robustness and held-out validation

## Scope and frozen protocol

N8 replayed the frozen N7 endpoint-aware governor with `m_curv=0.50 mm`,
`d_safe=5 mm`, `beta_stop=0.125`, the N7 task/phase schedule, and five swept
subdivisions.  The total nominal task time was held at approximately `0.5 s`:

| `dt` (s) | steps |
|---:|---:|
| 0.0025 | 200 |
| 0.0050 | 100 |
| 0.0100 | 50 |

The N7 90-state manifest was replayed exactly, and a new seed-5200 manifest
provided 90 held-out states (10 per scene/speed cell).  Every method used the
same state, scene, speed, and checker within a cell.  The four methods were
`online_lift_filtered`, `online_endpoint_governor`, `stale_lut_safe_scaled`,
and `rebuilt_lut_safe_scaled`.

## Outputs and audit

- Runner: `hil-serl/tools/run_n8_dt_heldout.py`
- Summarizer: `hil-serl/tools/summarize_n8_dt_heldout.py`
- Output directory: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/`
- Raw rows: `n8_raw_trials.json` (2160 rows; SHA-256
  `5ea54e28da77f6ad55d34dd467a5b00854b302bf6dc28c03de0c57912dd5253b`)
- Held-out manifest: `n8_heldout_state_manifest.json`
- Exact N7 replay manifest: `n8_n7_replay_state_manifest.json`
- Summary: `n8_dt_summary.json`
- Step trace: `n8_step_trace.csv`

The N7 replay states are byte-for-byte equal as state records to the frozen
N7 manifest (90 states).  The formal N7 input hashes recorded during N8 were:

```text
n7_paired_state_manifest.json       09ee436d3f7ec423368c435f5f94ca6e5b0ca6944324c2271a5f553270b39f5c
n7_endpoint_governor_raw.json      1f822b744cbd466ea58647f668ef44880c3818d0ce08eed54a2eea404a7bffaf
n7_qualification_summary_final.json ec4c2c1643cc14973db9bf391c73ab2e91c80fe404a09b17749f2af07f97c11d
```

`instrumentation_unknown=0` and no accepted online step violated either the
endpoint or swept clearance threshold.

## Completion, progress, and latency

| manifest | `dt` | online lift | endpoint governor | stale LUT | rebuilt LUT |
|---|---:|---:|---:|---:|---:|
| N7 replay | 0.0025 | 88/90 | 90/90 | 52/90 | 70/90 |
| N7 replay | 0.0050 | 88/90 | 90/90 | 52/90 | 70/90 |
| N7 replay | 0.0100 | 45/90 | 89/90 | 52/90 | 70/90 |
| held-out | 0.0025 | 88/90 | 90/90 | 52/90 | 70/90 |
| held-out | 0.0050 | 88/90 | 90/90 | 52/90 | 70/90 |
| held-out | 0.0100 | 41/90 | 89/90 | 52/90 | 70/90 |

The governor's median progress was approximately `0.9998--0.9999` at all
tested `dt` values.  Its P05 progress was `0.9987` (replay) and `0.9973`
(held-out) at `dt=0.0025`, `0.9986` and `0.9971` at `dt=0.005`, and
`0.9228` and `0.8945` at `dt=0.010`.  Thus, the larger step remains usable in
this test but has a visible progress/latency tail.

Across 540 paired online trials, the governor recovered 100 baseline
incomplete trials and caused zero complete-to-incomplete regressions.  The
recoveries were concentrated at `dt=0.010` (44 replay and 48 held-out); there
were two recoveries per manifest at each smaller `dt`.

The online governor had approximately 67--68 ms median mean-step latency at
`dt=0.0025` and `dt=0.005`.  At `dt=0.010`, p95 latency increased to about
172 ms (replay) and 176 ms (held-out).  LUT queries stayed near 10.4--10.7 ms
per step.  This is a safety/progress trade-off, not evidence that online
continuation is faster than LUT execution.

## Finite-step error

The descriptive log-log slopes of median absolute `Delta d` were:

| manifest | online lift | endpoint governor |
|---|---:|---:|
| N7 replay | 1.92 | 2.01 |
| held-out | 1.91 | 2.01 |

The slopes are consistent with the earlier finite-step diagnostic, but they
are empirical slopes over three step sizes.  They do not establish a universal
second-order theorem, a curvature bound, or continuous-time safety invariance.
At `dt=0.010`, the unbacktracked online method still showed linearization
optimism (about 1.09% of trace samples on replay and 1.23% on held-out), while
the endpoint-aware governor had zero optimism samples in the recorded trace.
The independent nonlinear checker remains necessary.

## Paper interpretation

N8 supports the following conditional statement:

> Under the tested scenes, speeds, local task region, three finite step sizes,
> and the frozen 0.50-mm empirical margin, endpoint-aware backtracking can
> convert many finite-step candidate failures into safe lower-rate execution
> without accepting an endpoint or swept-clearance violation.  The result
> reproduces on the held-out state manifest.

The claim remains bounded by the tested operating range.  In particular,
`dt=0.010` has lower P05 progress and one governor stop per manifest, so the
margin is not promoted to a universal finite-step guarantee.  N8 provides no
global IK branch-preservation evidence, no topology result, and no physical
robot validation.  The LUT comparison reports a latency/progress trade-off;
it does not establish online non-inferiority or a reconstruction-cost
advantage.

## Next node

N9 should integrate N1--N8 into the final Results, Discussion, Conclusion, and
abstract, explicitly adding the held-out and finite-step operating-range
qualification while retaining N1's negative branch-jump result.

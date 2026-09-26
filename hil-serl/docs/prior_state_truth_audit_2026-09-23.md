# DP prior state truth audit (2026-09-23)

Reproduce from `hil-serl/` with the `serl_clean` Python environment:

```bash
PYTHONPATH=. python tools/audit_prior_states.py
```

The audit paired 97 prior PKLs with their DP CSVs and evaluated all 98,898
recorded states. It uses `ManifoldLookup` for `Q(x, phi)` and the existing C++
DP wall-cloud evaluator for the **same explicit joint configurations**. The
full per-state table and first replay-stop rows are written under
`data/aviator/prior_state_audit/`.

| Diagnostic | Result |
| --- | ---: |
| Direct `Q(x, DP phi)` vs DP `q`, median / p95 max-joint error | 0.0409 / 0.1448 rad |
| Nearest tabulated LUT phase vs DP `q`, median / p95 max-joint error | 0.00184 / 0.00331 rad |
| DP recorded `d` vs recomputed on identical DP `q`, maximum absolute error | 1.70e-10 m |
| DP configurations below the 5 mm hard margin | 0 |
| DP configurations with active geometry contact | 0 |
| DP phases outside the LUT safe interval | 67,451 / 98,898 |
| LUT interpolated `d` vs physical `d` on identical LUT `q`, p95 absolute error | 6.79 mm |
| LUT states falsely called below 5 mm by interpolated `d` | 26,696 |
| Replay stops | 71 clearance, 20 grid exclusion flag, 6 end |

The DP CSV phase is an arc coordinate relative to its locally selected
`self_motion_arc` seed. The manifold LUT transports its own `phi=0` seed across
the task grid. The same numerical `(x, phi)` therefore often names different
configurations. A nearest-phase scan recovers a close LUT configuration for
most states, supporting a coordinate-chart mismatch. It does **not** establish
that the resulting phase trajectory respects the 1.5 rad/s action limit:
nearest-grid labels have 1,743 left-arm and 5,046 right-arm adjacent changes
above 0.015 rad per 0.01 s, even after a simple 0.3 rad periodic wrap. These
counts are diagnostic; chart periodicity and continuity need explicit handling.

The C++ evaluator reproduces the DP distances to numerical precision, so
the current evidence does not support changing the wall gap or DP geometry.
The 5 mm policy is also not the reason that recorded DP states fail: every DP
state meets it. The LUT's stored distance is especially unreliable when the
queried phase is outside its declared safe interval; inside the interval its
p95 same-`q` distance error is about 0.1 mm.

The current Python environment has only an interpolated `d(x, phi)` value;
it cannot evaluate clearance at an arbitrary `q_DP`. The audit records that
requested field as null rather than claiming a same-`q` DP/environment
comparison. The C++ physical result is separately recorded for both `q_DP`
and `q_LUT`.

`branch.bin` is a grid exclusion code for loop-open, unreachable, or cycle
inconsistency. It is **not** a per-trajectory IK branch identifier. The DP CSV
does not store a branch ID, so the two branch identity fields remain null.

The launcher continues to reject the prior at the dynamic replay gate. Do not
resume RLPD with these demonstrations until one consistent phase chart yields
accurate configuration reconstruction, physically evaluated feasibility, and
complete action-driven replay. A relabeling of the existing DP CSV by independent
nearest-phase matches is insufficient because it can create velocity spikes.

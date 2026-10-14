# N10 table integration (2026-10-07)

## Scope

N10 embeds the frozen N1--N8 simulation results as LaTeX tables in the manuscript. No figures were created or modified. No experiment was rerun, and no N1--N8 JSON or CSV was changed.

## Tables added

| Table | Manuscript label | Evidence | Placement |
|---|---|---|---|
| 1 | `tab:node0_validation` | Node 0 numerical gate | Results / Node 0 |
| 2 | `tab:n1_branch_negative` | N1 strict branch-search summary | Results / N1 |
| 3 | `tab:n2_stop_classification` | N2 stop classification summary | Results / N2 |
| 4 | `tab:n3_finite_step` | N3 finite-step summary | Results / N3 |
| 5 | `tab:n4_margin` | N4 margin summary and CSV | Results / N4 |
| 6 | `tab:n6_lut` | N6 stale/rebuilt LUT qualification | Results / N6 |
| 7 | `tab:n7_n8_governor` | N7 governor and N8 replay/held-out summary | Results / N7--N8 |
| 8 | `tab:claim_boundary` | N1--N8 claim-evidence matrix | Results / evidence boundary |

The prose now cites each table with `\\ref`. Tables use `booktabs` and `tabularx`; no figure package or figure content was added.

## Evidence boundary retained

- N1 remains a negative branch-search control: zero strict valid branch-jump candidates.
- N2 preserves the endpoint-mismatch stop taxonomy and the reduced/full agreement.
- N3 reports descriptive finite-step scaling only.
- N4 reports margin as an empirical trade-off, not a universal curvature parameter.
- N6 preserves the negative stale/rebuilt LUT comparison and the latency/reconstruction costs.
- N7--N8 report endpoint-aware recovery, held-out completion, accepted-violation count, and latency.
- Physical execution remains unverified.

## Consistency checks

- All table values were copied from the frozen summaries or the audited N3/N4 CSVs.
- The manuscript's N1--N8 raw result hashes remain unchanged.
- Table labels are unique and every new table is referenced in Results.
- No new theoretical claim or figure was added.
- Full PDF compilation still depends on a TeX engine not installed in the current environment; structural LaTeX checks are required before submission.

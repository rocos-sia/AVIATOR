# N9 manuscript consolidation (2026-10-07)

## Scope

N9 integrates the frozen N1--N8 evidence into the manuscript. No experiment was added, and no N1--N8 JSON or CSV was modified. The manuscript was rewritten around the supported claim of safety-governed local online continuation in a tested regular locally connected component.

## Changes made

- Replaced the abstract with the final evidence boundary and N8 held-out counts.
- Rewrote Results to report Node 0, the N1 negative branch control, N2--N4 endpoint and finite-step evidence, N6 LUT qualification, and N7--N8 governor/held-out results.
- Rewrote Discussion to separate linear candidate generation from nonlinear acceptance, state the safety/progress/latency trade-off, retain N1's negative result, and mark physical execution as future work.
- Rewrote Conclusion to state only the supported local finite-step result.
- Replaced outdated planned adaptation/ablation text with completed N6--N8 qualification language.
- Tightened the contribution list, research questions, and physical-experiment description.
- Created `claim_evidence_matrix_n1_n8_2026-10-07.md`.

## Final claim boundary

The manuscript may claim local online continuation, analytic-Jacobian and reduced/full-QP consistency on tested states, candidate/acceptance separation, endpoint-aware governed recovery on the frozen replay and held-out protocols, and the measured finite-step safety/progress/latency trade-off. It may not claim global IK branch preservation, branch-jump prevention, global topology, continuous-time safety, reduced-QP speedup, universal curvature margins, online dominance over rebuilt LUTs, or physical deployment.

## N8 quantitative record retained

- 2160 trial rows: two 90-state manifests, three step sizes, four methods.
- Endpoint-aware governor completion: 90/90 at $dt=0.0025$ and $0.005$ s, 89/90 at $dt=0.010$ s on both manifests.
- 100/540 paired baseline-incomplete trials recovered; 0 complete-to-incomplete regressions.
- 0 accepted online endpoint/swept-clearance violations.
- Descriptive finite-step slopes: approximately 1.92 for unbacktracked lift and 2.01 for governor.
- Median online governor latency about 67--68 ms; p95 at $dt=0.010$ about 172--176 ms; LUT about 10.4--10.7 ms per step.

## Raw-evidence hash audit (before N9 edits)

The following hashes were recorded before editing the manuscript and must remain unchanged:

```text
f7211234b972bfb880114176d544fac479ced380a5055c0f2d0e57a22b9cff0e  N1 n1_strict_summary.json
71bf79ebef7d6cfe45ec528ef168d22f21c2f7745be09354ef8e5fe66d2bb6bc  N2 n2_stop_classification_summary.json
2236ca266ce3d86d57ab54b0d93b6524c8ccfbb73975452d71190186d5a0d7de  N3 n3_dt_sensitivity_summary.json
8642336acc627d8304e2081b0834cbedca9cb3aaac6d8cbbed7c7fe1e6abe108  N3 n3_classification_summary.csv
2ca82a10ccc112056336056e94ea2dfc4a003369e24618cad569ef9cc8a19503  N3 n3_clearance_error_statistics.csv
42d8e312c7863ceb0688ebbc5a79c5b3bf02225ca82b5c365594d869ffc37c69  N4 n4_curvature_margin_summary.json
b004ffe2acb4804f03cb3cf8258b78b9f1d93036dfa2e89e0543413abc58d0b9  N6 n6_qualification_summary_final_v2.json
ec4c2c1643cc14973db9bf391c73ab2e91c80fe404a09b17749f2af07f97c11d  N7 n7_qualification_summary_final.json
1887555f43356e1bf9f1b9be736b792abb8fada009b6f67d4294d6ea9682ba44  N8 n8_dt_summary.json
5ea54e28da77f6ad55d34dd467a5b00854b302bf6dc28c03de0c57912dd5253b  N8 n8_raw_trials.json
```

## Verification status

The strong-claim scan, structural LaTeX checks, `git diff --check`, focused tests, LUT consistency check, and frozen-file hash check passed. A full LaTeX compile could not run because this environment has no `latexmk`, `pdflatex`, `xelatex`, `lualatex`, or `tectonic`; the manuscript passed brace/environment-balance checks instead. The full pytest collection is blocked by missing `h5py` and a broken optional `narwhals` plugin dependency, while 71 focused G-TSR tests passed with plugin autoload disabled.

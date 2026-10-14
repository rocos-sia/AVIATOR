# Long-horizon experiment status

## Current node

- Node: RL-N13R — complete demonstration replay and convergence repair
- Status: complete with strong final held-out checkpoint but non-monotonic training; formal N14 remains blocked
- Started: 2026-10-07
- Output record: /home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/n13r_extra_prior_summary.json

## Frozen protocol

- Python: /home/rocos/miniconda3/envs/mujo/bin/python
- Manifold: hil-serl/data/aviator/manifold_phi_stale_pitch-10deg
- Model: AviatorRobot/build/bin/model/aviator.xml
- Decision variable: y=[r_L,r_R,beta]
- dt=0.005 for the nominal comparison
- beta_stop=0.125
- Atlas starts: at least 128 per arm for the first search; 192 for confirmation
- Closure tolerance: 2e-4 in the existing mixed pose-error norm
- Label margin: 0.05 in normalized joint-space distance
- Independent descriptor gap: 0.04 rad
- A valid branch jump requires an unambiguous label change and small closure residual at both endpoints.
- Solver failure, ambiguous labels, large residual, or a filtered safety stop is not a branch jump.

## Evidence already accepted before N1

- Node 0 shared formulation: outputs/differential_lift_feasibility_20261004/v3/node0_shared.json
- Frozen 90-trial matrix: outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json
- Those results support local safety filtering and full/reduced QP agreement.

## Claims under test

- Supported target: local continuation inside a validated regular component.
- N1 success condition: an independently atlas-validated segment where Pointwise or DLS has a valid branch jump, while lift_filtered has no valid jump and reaches the corresponding step.
- If the bounded search finds no such segment, the paper claim remains safety-governed local continuation; no branch-superiority claim is added.

## N1 outputs

1. Independent dual-arm atlas: outputs/differential_lift_feasibility_20261005/n1_branch_search/dual_atlas_state6_128.json
2. Independent right-arm atlas: outputs/differential_lift_feasibility_20261005/n1_branch_search/right_atlas_state6_128.json
3. Raw Pointwise/DLS/lift_filtered probes: outputs/differential_lift_feasibility_20261005/n1_branch_search/auto_component/
4. Strict frozen summary: outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json

## N1 result

- Right-arm atlas: 2 base components from 128 starts; both had local continuation samples, but the strict 5x5 extension covered only 5/25 common grid cells per seed and did not pass the wide stability gate.
- Dual-arm atlas: no segment with two stable components on both arms (stable_component_counts=[0,0] for the wide 5x5 test).
- Stress cases: 24 frozen combinations; 20 reached comparison and 4 were classified as atlas_insufficient.
- Strict candidates: 0.
- Strict valid branch jumps: Pointwise 0, DLS 0, lift_filtered 0.
- The only raw DLS/H+Nr label change had an independent descriptor change of about 0.0075 rad and was rejected by the frozen 0.04 rad descriptor-gap rule.
- lift_filtered stopped by endpoint validation before the raw label-change step in the near-miss case; this is not branch-preservation evidence.

## N1 paper impact and next node

N1 does not support a comparative claim that the proposed lift prevents a Pointwise/DLS branch jump. It supports the narrower statement that the tested local component can be monitored with an independent descriptor and that safety stops are separated from branch labels. The next node is N2: classify endpoint-validation stops and quantify progress, beta, clearance, and failure causes. No additional branch-superiority wording is added.

## N2 outputs

- Trace manifest: `outputs/differential_lift_feasibility_20261005/n2_endpoint_trace_v2/n2_trace_manifest.json`
- Raw merged rows: `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_endpoint_stop_raw.json`
- Classification summary: `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json`
- Predicted-vs-actual CSV/figure: `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_predicted_vs_actual_clearance.csv` and `.png`

## N2 result

- The frozen v3 matrix contains 90 paired trials; 92 filtered-stop rows were traced (46 `lift_filtered` and 46 `full_joint_qp`).
- Each method has 44 `nonlinear_endpoint_mismatch`, 1 `swept_clearance_failure`, and 1 `joint_limit_failure`; `true_local_infeasibility=0` and `instrumentation_unknown=0`.
- All required stop fields are present: progress, beta, beta minimum/time below one, QP feasibility, predicted clearance, endpoint/swept clearance, joint margin, closure residual, and `sigma_min(A_q)`.
- Reduced and full-QP classifications agree for every paired stop; there are no unexplained representation differences.
- The dominant case has a feasible linear QP and predicted clearance at 5 mm, but the independent nonlinear endpoint is below 5 mm. This is linearization optimism at the boundary, not evidence that the filter is overly conservative or that the local task set is globally infeasible.

## N2 paper impact and next node

Keep the lift decomposition, convex reduced-QP theorem, full/reduced equivalence, and conditional governor logic. Add an explicit two-layer safety statement: linear QP feasibility produces a candidate, while independent nonlinear endpoint/swept validation is required for acceptance. Do not interpret endpoint rejection as branch-preservation evidence or as a proof of true local infeasibility. N3 should quantify finite-step clearance prediction error versus `dt`, curvature, and initial clearance without changing the frozen N2 thresholds.

## N3 outputs

- Diagnostic manifest: `outputs/differential_lift_feasibility_20261005/n3_dt_sensitivity/n3_dt_sensitivity_manifest.json`
- Raw per-step JSON: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_step_trace_raw.json`
- Finite-step summary: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_dt_sensitivity_summary.json`
- Per-step clearance CSV: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_clearance_error_steps.csv`
- Margin-bin table: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_clearance_error_statistics.csv`
- Classification table: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_classification_summary.csv`
- Reduced/full overall comparison: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_reduced_full_comparison.csv`
- Reduced/full margin-bin comparison: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_reduced_full_comparison_by_margin_bin.csv`
- Predicted-vs-actual and error plot: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_predicted_actual_delta_figures.png`
- Research note: `docs/research/differential_lift_n3_finite_step_analysis_2026-10-05.md`

## N3 result

- The 27-cell diagnostic run (three `dt` values, three scenes, three speed settings) produced 53,200 complete step records, 26,600 per filtered method, with `instrumentation_unknown=0`.
- At `dt=0.0025, 0.005, 0.01`, the median absolute `Delta d` was `3.74e-9`, `1.60e-8`, and `6.76e-8` m. The descriptive log-log slope was `2.087`; it is reported as finite-step sensitivity, not a general theorem.
- Each method had 140 linearization-optimism samples (0.526%) and 3 conservative rejections (0.0113%). The conservative cases were post-integrator joint-limit checks with clearance well above `d_safe`, not clearance-filter rejection.
- Optimism and endpoint/swept failures concentrated in the `[0,0.05) mm` predicted-margin bin and increased with `dt`; the independent nonlinear checker remains necessary near the boundary.
- Reduced and full joint-space QP traces had identical aggregate classifications and indistinguishable `Delta d` distributions at floating-point precision. This supports numerical consistency, not a runtime advantage for the reduced representation.

## N3 paper impact and next node

N3 does not require changing the lift equations. The paper should state that the linear QP generates a candidate and the independent nonlinear endpoint/swept checker decides acceptance; finite-step optimism is an observed boundary effect. The approximate second-order scaling is descriptive only and does not establish a curvature bound or continuous-time invariance. N3 does not provide branch-jump or global IK-branch evidence. The next node is either a separately frozen curvature-margin ablation or a manuscript consolidation node that integrates the N1 negative result, N2 stop taxonomy, and N3 finite-step analysis.

## N4 outputs

- Cell reports and manifest: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin/`
- Raw merged trace: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_step_trace_raw.json`
- Summary JSON: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_curvature_margin_summary.json`
- Per-step CSV: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_clearance_margin_steps.csv`
- Classification CSV: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_classification_summary.csv`
- Margin summary CSV: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_margin_summary.csv`
- Reduced/full comparison: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_reduced_full_comparison.csv`
- Figure: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_margin_optimism_progress_stop_figures.png`
- Research note: `docs/research/differential_lift_n4_curvature_margin_2026-10-05.md`

## N4 result

- The 135-cell run (five margins, three `dt` values, three scenes, three speed settings) produced 281,262 step rows and 2,700 filtered/full trial rows; `instrumentation_unknown=0`.
- Margin 0 reproduces N3 exactly on all 53,200 shared step records. A 0.50-mm margin reduced optimism from 0.526% to 0.133%, increased complete trials from 48.15% to 83.70%, and raised median task progress from 0.9746 to 0.9999.
- The improvement is not uniform across step sizes: with 0.50 mm, completion was 89/90 at `dt=0.0025`, 89/90 at `dt=0.005`, and 48/90 at `dt=0.01`. No tested margin eliminated finite-step optimism at all `dt` values.
- Clearance-only conservative rejection was zero for all margins. The three rejections per method were the same small joint-limit post-check events seen in N3.
- Governor use increased and median step latency rose from about 72 ms at zero margin to 79–80 ms at 0.50 mm. Reduced/full QP classification and `Delta d` distributions remained numerically identical.

## N4 paper impact and next node

N4 supports reporting curvature margin as an empirical safety--progress trade-off, but does not justify adding a universal margin to the core method or claiming a curvature guarantee. Keep the independent nonlinear endpoint/swept checker as the final acceptance mechanism. N4 is not branch evidence. The next node should be N5 manuscript consolidation: integrate N1's negative branch result, N2's stop taxonomy, N3's finite-step analysis, and N4's margin trade-off into the claim--evidence matrix and Results/Discussion.


## N5 outputs

- Claim--evidence matrix: `docs/research/claim_evidence_matrix_n1_n3_2026-10-05.md`
- Consolidation note: `docs/research/n5_claim_boundary_consolidation_2026-10-06.md`
- Updated manuscript: `/home/rocos/sia/tsr/aviator_branch_preserving_differential_lift_draft.tex`

## N5 result

- The supported paper claim is restricted to safety-governed local online continuation inside a regular locally connected component.
- N1 remains a negative branch-jump control: no strict Pointwise/DLS jump candidate was found, so no branch-superiority claim is made.
- N2 is interpreted as evidence that independent nonlinear endpoint/swept validation is required after linear QP candidate generation; endpoint rejection is not treated as proof of true local infeasibility.
- N3 is interpreted as finite-step diagnostic evidence: the observed log--log slope is about 2.09, but the negative tail prevents a universal second-order or curvature guarantee.
- Reduced/full QP agreement supports numerical consistency on tested states only; no runtime advantage is claimed.
- N4 is kept as a supplementary empirical margin trade-off and does not change the core lift theorem.
- Physical execution, global IK branch preservation, global topology, continuous-time invariance, and LUT cost advantage remain unverified.

## N5 paper impact and next node

The Results, Discussion, Conclusion, and abstract now state the candidate/acceptance split, retain N1's negative result, and avoid unsupported global or formal safety language. The next node is a manuscript audit/compile gate; new experiments should only be started after this audit confirms that all quantitative statements resolve to frozen evidence files.


## N6-A protocol

- Protocol: `docs/research/n6_online_lut_qualification_protocol_2026-10-06.md`
- Frozen inputs: stale-pitch manifold, Aviator MuJoCo model, `dt=0.005`, `steps=100`, `beta_stop=0.125`, `d_safe=0.005 m`.
- Methods: `frozen_lut` and `online_lift_filtered`; `full_joint_qp` is optional representation control.
- Scope: 3 v3 scenes × 3 v3 speeds × 10 paired trials; same initial `(x,q,phi)` and independent checker.
- N1–N4 JSON files are read-only and must not be overwritten.
- N6-A cannot claim rebuilt-LUT cost savings; that requires N6-B.

## N6 results

- Corrected N6-A formal output: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/`
- N6-B rebuilt-table outputs: `outputs/differential_lift_feasibility_20261006/n6_rebuilt_compare_v2_nominal/`, `n6_rebuilt_compare_v2_tight/`, and `n6_rebuilt_compare_v2_shifted/`
- Combined summary: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_qualification_summary_final_v2.json`
- Results note: `docs/research/n6_online_lut_qualification_results_2026-10-06.md`

### N6-A stale-table result

- Across 90 paired trials, the safe-scaled frozen LUT completed 52/90 (57.8%) and the online filtered lift completed 42/90 (46.7%).
- The online method had zero rejected clearance candidates and the LUT had 38 candidate endpoint/swept clearance rejections; rejected candidates were not accepted physical collisions.
- The online method used 717 governed steps and had median mean step latency 67.15 ms versus 10.46 ms for the LUT.
- In the tight stale-table perturbation, online completion was 10/30 versus 8/30 for the frozen LUT. This is conditional evidence of local online robustness under one perturbation, not a general superiority result.

### N6-B rebuilt-table result

- Rebuilt LUT completion was 22/30 nominal, 21/30 tight, and 22/30 shifted, exceeding online completion of 15/30, 13/30, and 15/30 respectively.
- Each rebuilt table was about 186.03 MiB and took 51.75--52.23 s to construct in the recorded environment.
- Rebuilt runs were independently sampled per table and qualify recovery/cost, not a paired stale-versus-rebuilt hypothesis test.

### N6 paper impact

N6 does not support online non-inferiority to a rebuilt LUT, lower latency, or a universal adaptation/cost advantage. It supports the narrower statement that online continuation can avoid table reconstruction and provide auditable nonlinear safety stops; one tight stale-table perturbation showed higher progress than the frozen table at a substantial latency cost. The old fixed-phase `n6_smoke2` and `n6_online_lut_qualification_v1` results remain diagnostic and are not final evidence.

## N6 next-node record

- Node N7: manuscript audit and claim decision. Integrate N6 as a conditional/negative LUT qualification, keep the safety-governed local-continuation claim, and remove any unsupported adaptation, completion, latency, or reconstruction-cost superiority wording. Do not modify N1--N4 JSON files.

## N7 outputs

- Runner: `hil-serl/tools/run_n7_endpoint_governor.py`
- Summarizer: `hil-serl/tools/summarize_n7_endpoint_governor.py`
- Zero-margin formal raw result: `outputs/differential_lift_feasibility_20261006/n7_formal_v1/n7_endpoint_governor_raw.json`
- 0.25-mm formal raw result: `outputs/differential_lift_feasibility_20261006/n7_formal_margin025/n7_endpoint_governor_raw.json`
- 0.50-mm formal raw result: `outputs/differential_lift_feasibility_20261006/n7_formal_margin050/n7_endpoint_governor_raw.json`
- Combined summary: `outputs/differential_lift_feasibility_20261006/n7_formal_v1/n7_qualification_summary_final.json`
- Results note: `docs/research/n7_endpoint_governor_results_2026-10-07.md`

## N7 result

- The same 90-state manifest was replayed through stale LUT, rebuilt LUT, current online lift, and endpoint-aware online governor.
- At `m_curv=0`, endpoint-aware backtracking improved online completion from 42/90 to 49/90, recovered 7 paired trials, caused no paired regression, and produced no accepted endpoint/swept clearance violation.
- At `m_curv=0.25 mm`, the governor accepted all 90/90 100-step executions; P05 task progress was 0.918 and p95 latency was about 168 ms.
- At `m_curv=0.50 mm`, the governor also accepted 90/90 executions; P05 task progress was about 0.998 and median latency was about 67.4 ms. The current online baseline completed 88/90 under the same margin.
- Rebuilt LUT completed 70/90 and stale LUT 52/90 on the common N7 state manifest, with about 10.5 ms median step latency.
- `instrumentation_unknown=0`; no accepted governor step violated endpoint or swept clearance.

## N7 paper impact

N7 supports promoting endpoint-aware finite-step backtracking to an experimentally supported algorithm component. The result remains conditional on the tested scenes, speeds, `dt=0.005`, checker, and frozen margins. The 0.50-mm margin is an empirical design parameter, not a universal curvature bound. N7 provides no global IK branch-preservation evidence and no continuous-time invariance theorem.

## Next node

- Node N8: freeze the endpoint-aware schedule and `m_curv=0.50 mm`, test held-out seeds and `dt` sensitivity (especially `dt=0.010`), and verify that the progress/safety result persists before rewriting the paper's main theory claim.

## N8 outputs

- Runner: `hil-serl/tools/run_n8_dt_heldout.py`
- Summarizer: `hil-serl/tools/summarize_n8_dt_heldout.py`
- Formal output directory: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/`
- Raw trials: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_raw_trials.json`
- Held-out manifest: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_heldout_state_manifest.json`
- Summary: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_dt_summary.json`
- Results note: `docs/research/n8_dt_heldout_results_2026-10-07.md`

## N8 result

- The formal matrix contains 2160 rows: two 90-state manifests (the exact N7 replay manifest and a seed-5200 held-out manifest), three `dt` values (`0.0025`, `0.005`, `0.010`), and four methods.
- `instrumentation_unknown=0`; no accepted online step violated endpoint or swept clearance.
- Endpoint-aware governor completion was 90/90 at `dt=0.0025` and `0.005`, and 89/90 at `dt=0.010` on both manifests.  It recovered 100 of 540 paired baseline-incomplete trials and caused zero complete-to-incomplete regressions.
- The governor P05 progress was approximately `0.9987/0.9973` at `dt=0.0025`, `0.9986/0.9971` at `dt=0.005`, and `0.9228/0.8945` at `dt=0.010` for replay/held-out manifests.  This is evidence of a finite-step operating range, not a universal guarantee.
- Descriptive log-log slopes of median absolute clearance error were about `1.92` for the unbacktracked online lift and `2.01` for the endpoint-aware governor on replay and held-out states.  These do not establish a universal second-order theorem.
- At `dt=0.010`, the unbacktracked lift retained linearization-optimism samples (about 1.1--1.2%), while the endpoint-aware governor had none in the recorded trace.  The nonlinear endpoint/swept checker remains the final acceptance authority.
- LUT execution remained about 10.4--10.7 ms per step, while online governor latency was about 67--68 ms median and up to about 172--176 ms p95 at `dt=0.010`.  This is a safety/progress trade-off, not an online speed or reconstruction-cost advantage.

## N8 paper impact

N8 supports a conditional finite-step statement: under the tested local task region, scenes, speeds, step sizes, and frozen 0.50-mm empirical margin, endpoint-aware backtracking can recover many candidate failures while retaining auditable nonlinear acceptance, and the result reproduces on held-out states.  The paper must retain the operating-range qualifier and must not turn the margin into a universal curvature bound or continuous-time safety theorem.  N8 provides no global IK branch, topology, or physical-execution evidence.

## Next node

- Node N9: integrate N1--N8 into the final Results, Discussion, Conclusion, and abstract.  Preserve N1's negative branch-jump result, add N8 held-out and finite-step qualification, and keep the safety/progress/latency trade-off explicit.

## N9: final manuscript consolidation (2026-10-07)

### Inputs

- N1 strict branch-search summary: `outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json`
- N2 stop classification: `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json`
- N3 finite-step summary and CSVs: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/`
- N4 curvature-margin summary: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_curvature_margin_summary.json`
- N6 LUT qualification: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_qualification_summary_final_v2.json`
- N7 endpoint-governor summary: `outputs/differential_lift_feasibility_20261006/n7_formal_v1/n7_qualification_summary_final.json`
- N8 held-out and step-size summary: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_dt_summary.json`

### Work completed

- Rewrote the manuscript abstract, Results, Discussion, Conclusion, contribution wording, research questions, and physical-validation scope in `/home/rocos/sia/tsr/aviator_branch_preserving_differential_lift_draft.tex`.
- Created `docs/research/claim_evidence_matrix_n1_n8_2026-10-07.md`.
- Created `docs/research/n9_manuscript_consolidation_2026-10-07.md`.
- Preserved N1--N8 raw JSON/CSV files as read-only; the N9 note records their pre-edit hashes.

### Frozen final claim

> In a tested regular locally connected component, the analytic differential lift and three-variable linear QP generate an online candidate continuation, while independent nonlinear endpoint/swept validation and endpoint-aware backtracking preserve accepted clearance and maintain task progress over a finite tested operating range.

### Claims explicitly excluded

- Global IK branch preservation or branch-jump prevention.
- Global topology or continuous-time safety invariance.
- Universal curvature margin or universal second-order accuracy.
- Reduced-QP runtime superiority.
- Online non-inferiority to a rebuilt LUT or universal reconstruction-cost advantage.
- Physical execution or force-regulation validation.

### N9 status

Manuscript integration is complete. Structural LaTeX checks, focused tests, LUT consistency, diff checks, strong-claim scan, and raw-hash audit passed; a full LaTeX compile is unavailable because no TeX engine is installed in this environment. The simulation evidence is internally connected from Node 0 through N8. The next work should be submission-quality figure/table cleanup or a separately scoped physical-validation node; no new theoretical guarantee should be added without new evidence.

## N10: experiment-table integration (2026-10-07)

### Work completed

- Embedded eight review tables into `/home/rocos/sia/tsr/aviator_branch_preserving_differential_lift_draft.tex`.
- Added `tabularx` support and in-text `\ref` citations.
- Added `docs/research/n10_table_integration_2026-10-07.md`.
- Did not create or modify figures.
- Did not rerun experiments or modify N1--N8 JSON/CSV files.

### Table mapping

- `tab:node0_validation`: Node 0 numerical contract and reduced/full QP.
- `tab:n1_branch_negative`: N1 strict branch-search negative control.
- `tab:n2_stop_classification`: N2 endpoint stop taxonomy.
- `tab:n3_finite_step`: N3 finite-step clearance error.
- `tab:n4_margin`: N4 empirical margin trade-off.
- `tab:n6_lut`: N6 stale/rebuilt LUT qualification.
- `tab:n7_n8_governor`: N7 endpoint-aware governor and N8 held-out replay.
- `tab:claim_boundary`: final N1--N8 claim boundary.

### N10 status

Table integration is complete for author review. The next step is visual review after a TeX engine is available; figures remain intentionally deferred. No new theoretical claim should be added during table review.

## RL-N11: RL extension protocol freeze (2026-10-07)

### Scope

RL-N11 freezes a separate extension experiment. The policy action is
`[r_L,r_R]` on the fixed nominal task trajectory; `beta` remains owned by the
endpoint-aware governor. No RL training or new safety result was run in this
node. Existing N1--N10 JSON/CSV files remain read-only.

### Frozen protocol

- Training environment: current `AviatorManifoldEnv`, `dt=0.010`.
- Evaluation step sizes: `dt=0.005` and `dt=0.010`.
- `d_safe=0.005 m`, `m_curv=0.50 mm`, and the N7/N8 checker/governor are frozen.
- Methods: `fixed_phase`, `greedy_phase`, `RL_raw`, and `RL_governed`.
- Training seeds: `0, 1, 2`.
- Pilot size: 10 paired trials per scene/speed cell; formal size is 30--50 only
  after a pilot go/no-go decision.
- No global branch, continuous-time safety, physical zero-collision, or 100-Hz
  claim is enabled by this node.

### Outputs

- Protocol: `outputs/differential_lift_feasibility_20261007/rl_n11_protocol/n11_protocol.json`
- Input hashes: `outputs/differential_lift_feasibility_20261007/rl_n11_protocol/input_hashes.txt`
- Readme: `outputs/differential_lift_feasibility_20261007/rl_n11_protocol/README.md`
- Protocol script: `hil-serl/tools/run_rl_n11_protocol.py`

The protocol records 32 input hashes and reports no missing inputs. It refuses
to overwrite a non-empty output directory. N11 has `training_not_run=true`.

### N11 decision

The RL extension is ready for the decoder geometry gate. The current evidence
supports only the scoped hypothesis that RL can schedule redundancy phase on a
fixed task trajectory; it does not yet support an RL performance claim. The
next node is RL-N12: stratified decoder closure, branch-validity, safety-margin,
and latency validation before policy training.

## RL-N12: decoder geometry gate (2026-10-07)

### Protocol

- 10,000 stratified samples from the interior of the stale-pitch local valid
  phase domain, seed `1207`.
- Decoder: current trilinear `ManifoldLookup`.
- Independent checks: MuJoCo FK closure and the independent static collision
  checker using the frozen Aviator model.
- `d_safe=0.005 m`; closure diagnostic tolerance `2e-4` in the existing mixed
  pose-error norm.
- No policy was trained and no N1--N11 raw result was modified.

### Result

- Samples recorded: `10000/10000`.
- Branch code 0: `10000/10000`.
- Closure pass rate: `100%`.
- Independent endpoint clearance pass rate: `100%`.
- Independent contact samples: `0`.
- Closure residual infinity norm: median `3.51e-5`, P95 `5.91e-5`, maximum
  `1.95e-4`.
- LUT clearance: median `8.25e-3 m`.
- Independently checked clearance: median `8.26e-3 m`.
- Decoder plus independent checks: median `2.20 ms`, P95 `2.44 ms`.
- `instrumentation_unknown=0` and no sampled failure category occurred.

### Interpretation

N12 supports a local engineering statement: within the tested interior domain,
the decoder produced branch-valid references with controlled closure residual
and positive independently checked clearance. It does not establish the exact
identity `F(Q,x)=0` over the full table, global branch completeness, swept
continuous-time safety, or physical execution. The maximum residual is retained
as a measured boundary of this local qualification rather than rounded to zero.

### Outputs

- Script: `hil-serl/tools/run_rl_n12_decoder_gate.py`
- Raw samples: `outputs/differential_lift_feasibility_20261007/rl_n12_decoder_gate/n12_raw_samples.json`
- Summary: `outputs/differential_lift_feasibility_20261007/rl_n12_decoder_gate/n12_summary.json`
- Trace: `outputs/differential_lift_feasibility_20261007/rl_n12_decoder_gate/n12_decoder_trace.csv`

### N12 decision

The decoder gate passes for the tested local interior domain. RL training may
start under the frozen two-dimensional action contract. The next node is
RL-N13: three-seed policy-training sanity evaluation with fixed train/validation
splits and no safety claim from the training environment alone.

## RL-N13: training sanity gate (2026-10-07)

### Protocol

- SAC/RLPD implementation: existing `examples/train_rlpd.py`.
- Three frozen seeds: `0, 1, 2`.
- Training manifold: stale-pitch manifold used by N7/N8.
- Action: `[r_L,r_R]`; beta remains outside the actor and belongs to the governor.
- Training budget: 20,000 learner/actor step cap, 16 online episodes, two actor
  environments per seed.
- Replay prior: 16 stale-manifold-compatible greedy demonstrations generated for
  this node; they were not mixed with the old default-manifold demonstration files.
- Held-out evaluation: 50 validation trajectories, unshielded
  `AviatorManifoldEnv`.

### Result

- All three learner/actor processes exited cleanly and produced checkpoints:
  `checkpoint_67`, `checkpoint_51`, and `checkpoint_103`.
- The actor collected only 67, 51, and 103 online transitions respectively
  before the episode limit was exhausted.
- RL validation completion was `0/50` for all three seeds.
- RL termination was dominated by `grid_exit` (`60%`, `66%`, and `60%`) and
  `speed` (`40%`, `34%`, and `40%`).
- Under the same stale manifold and unshielded environment, `fixed_phase`
  completed `1/50` and `greedy_phase` completed `21/50`; greedy also produced
  a `58%` speed-termination rate.
- The greedy replay prior itself contained 7 one-step failures among 16
  trajectories, confirming that the training prior did not provide a uniformly
  valid starting distribution.

### Failure classification

This is a training-environment/data-coverage failure, not evidence that the
low-dimensional action contract is mathematically invalid and not evidence for
or against physical safety. The current unshielded environment exposes a
1.5-rad/s action scale and terminates on speed or phase-grid violations before
the learner can collect a useful long-horizon distribution. N13 therefore does
not support an RL performance claim.

### Outputs

- Runner: `hil-serl/tools/run_rl_n13_training_sanity.py`
- Summary: `outputs/differential_lift_feasibility_20261007/rl_n13_training/n13_training_summary.json`
- Policy evaluation: `outputs/differential_lift_feasibility_20261007/rl_n13_training/n13_policy_eval.json`
- Logs/checkpoints: `outputs/differential_lift_feasibility_20261007/rl_n13_training/`
- Protocol report: `docs/research/rl_n13_training_sanity_2026-10-07.md`

### N13 decision

The formal N14 paired policy comparison is blocked. Do not claim that RL
improves completion or reduces governor use. The next permissible node is a
small, separately frozen **RL-N13R training-environment repair** that adds a
documented feasible-start curriculum or a training-only action shield while
preserving the deployment safety checker and the N1--N10 evidence. N13 raw
outputs must remain unchanged.

## RL-N13R: complete demonstration replay and convergence repair (2026-10-07)

### Demonstration buffer repair

- Runner: `hil-serl/tools/run_rl_n13r_complete_demo.py`
- Complete-demo directory: `outputs/differential_lift_feasibility_20261007/rl_n13r_complete_demo/complete_demo/`
- Audit: `outputs/differential_lift_feasibility_20261007/rl_n13r_complete_demo/complete_demo_audit.json`
- 66 stale-manifold source trajectories were checked; 64 complete trajectories were retained and 2 speed-terminated trajectories were discarded.
- Retained lengths were 818--1190 steps. Every file is a list of step transitions. Every retained file has exactly one final `dones=True, masks=0` transition; all earlier transitions have `dones=False, masks=1`.
- The learner inserted these transitions into its independent `demo_buffer`; they were not copied into online replay. The learner log records `demo buffer size: 64595` and, in the fixed-extra-prior run, `extra demo buffer size: 64595`.
- The generation policy is a training-only slow phase-centre controller (`demo_scale=0.10`). It does not modify the deployment checker, governor, or N1--N10 evidence.

### Convergence repair run

- BC warm-start: `outputs/differential_lift_feasibility_20261007/rl_n13r_bc_warmstart/bc_seed_0/`
- RLPD run with the complete demos as both the ordinary and fixed extra prior: `outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/train_seed_0/`
- Checkpoint curve: `outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/train_seed_0/convergence_eval.json`
- Final held-out result: `outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/train_seed_0/final_heldout_50.json`
- Summary: `outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/n13r_extra_prior_summary.json`

The 20-trajectory held-out curve was 95% completion at 10k, 90% at 20k,
90% at 30k, 95% at 40k, 20% at 50k, and 95% at 60k. The final 50-trajectory
evaluation at checkpoint 60k was 94% completion, 4% clearance termination,
2% speed termination, and 0% grid/branch/joint-limit termination. This is a
strong final checkpoint, but the 50k collapse shows that training is
non-monotonic; it is not evidence of strict stable convergence. The final
checkpoint must therefore be treated as a selected held-out result, with the
training instability reported explicitly.

### RL-N13R decision

The complete-transition repair is valid and materially improves the replay
prior. BC warm-start plus a fixed extra demonstration quota makes the held-out
policy reach about 90--95% completion on the tested validation split, while
the no-warm-start run was unstable. However, this is still a single seed and
the training curve has a large 50k failure basin. N14 remains blocked until a
new frozen multi-seed stability gate, with predeclared checkpoint selection or
a stabilised learner schedule, is passed. This node does not enable RL safety,
physical execution, global branch, or governor-performance claims.

## RL-N13R-RCA: 50k root-cause analysis and repaired seeds (2026-10-08)

### Repaired protocol

- Runner: `hil-serl/tools/run_rl_n13r_complete_demo.py`
- Repaired output: `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/`
- Seeds: `1, 2`, each with a 60,000 learner/online-transition budget.
- Replay composition: fixed `128 online + 128 demo` transitions per batch; no extra demo buffer.
- Critic regularization: global gradient-norm clipping at `1.0`.
- Diagnostics: one JSONL record per 100 learner updates, including critic/Q statistics,
  actor loss/entropy, temperature, target entropy, and actual buffer composition.
- Shutdown: learner waits for the explicit actor-done request after the nominal cap,
  writes the final checkpoint, then stops the trainer server. Both repaired runs
  exited with actor and learner return code 0.

### RCA result

The immutable old Seed-0 50k held-out result was dominated by clearance failure:
75% clearance, 5% speed, and 0% grid/phase-boundary, branch, or joint-limit exits.
Its learner log did not contain update-level critic/Q/entropy/temperature values,
so a Q explosion or temperature instability cannot be claimed retrospectively.

The repaired Seed-1 run had 90% completion at 50k and 90% at 60k. Seed 2 had a
smaller 50k dip (65% completion, 30% clearance) and recovered to 95% at 60k.
In both repaired runs, Q tracked target Q, entropy stayed near the target, alpha
remained bounded, and the demo fraction was exactly 0.5 in every diagnostic record.
The evidence supports a seed-dependent policy/distribution drift near the clearance
boundary as the best current explanation. It does not isolate critic targets,
demo-ratio scheduling, or gradient clipping as a unique cause.

### Outputs and decision

- RCA JSON: `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/root_cause_analysis.json`
- Window statistics: `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/root_cause_metric_windows.csv`
- Report: `docs/research/rl_n13r_root_cause_analysis_2026-10-08.md`
- Analysis script: `hil-serl/tools/analyze_rl_n13r_root_cause.py`

The repaired runs pass the process and held-out recovery gate, but they do not
establish monotonic convergence or a universal RL guarantee. N14 remains blocked
until a predeclared checkpoint-selection rule and a 2x2 demo-ratio/clipping
ablation are run. N13 and the original N13R JSON artifacts remain immutable.

## RL-N13R 2x2 pilot (2026-10-08)

A 10k-step same-seed pilot was run for the four combinations of historical
versus fixed-50/50 demo prior and critic clipping on/off. The output is
`outputs/differential_lift_feasibility_20261008/rl_n13r_2x2_pilot/`, with the
protocol report in `docs/research/rl_n13r_2x2_pilot_2026-10-08.md`.

At 10k, fixed-50/50 plus clipping reached 75% held-out completion with 10%
clearance failure. Historical prior plus clipping reached 60% completion with
35% clearance failure; fixed-50/50 without clipping reached 30% completion
with 65% clearance failure. The pilot nominates fixed-50/50 plus clipping for
the next full-budget run, but does not establish a causal 2x2 conclusion or
50k stability.


## RL clearance-margin pilot (2026-10-08)

The existing reward already contained the clipped quadratic warning-layer term over 5--10 mm; this node exposed its weight as `AVIATOR_MARGIN_WEIGHT` without changing the safety termination rule. A same-seed 10k-step pilot compared `w_m=0.1` (baseline) with `w_m=0.5` using fixed 50/50 demo/online sampling, critic clip 1.0, BC warm start, and 20 held-out episodes.

- Output: `outputs/differential_lift_feasibility_20261008/rl_n13r_margin_pilot_w05/`; report: `docs/research/rl_n13r_clearance_margin_pilot_2026-10-08.md`.
- At checkpoint 10k, completion changed 75% -> 80%, but clearance failures changed 10% -> 20%; speed/grid failures decreased in this sample. Mean minimum clearance was 6.564 -> 6.587 mm.
- Decision: runnable, but no demonstrated safety improvement. Keep `w_m=0.1` as the formal baseline. Do not promote `w_m=0.5` or claim reduced boundary drift without a predeclared multi-seed, longer-horizon ablation. N13/N13R/N1--N10 artifacts remain immutable.


## RL-N14 exploration continuation pilot (2026-10-08)

N14 was initialized from the complete SAC state at seed 2, checkpoint 60000, rather than only copying the actor. A 10k continuation used the frozen `w_m=0.1`, fixed 50/50 demo/online, critic clip 1.0 protocol and randomized feasible initial phases in `[0.2, 0.8]`.

- Output: `outputs/differential_lift_feasibility_20261008/rl_n14_reset_exploration_from_seed2_60k_retry/`; report: `docs/research/rl_n14_exploration_pilot_2026-10-08.md`.
- Random-reset evaluation at 10k: 95% completion, 0% clearance, 5% speed on 20 held-out trajectories. Fixed-reset evaluation of the same checkpoint: 100% completion, 0% clearance, 0% speed.
- The run confirms that full-state continuation is operational and does not immediately damage safety. It is a single-seed, short continuation and does not establish exploration causality or generalization. Run the predeclared multi-seed gate before N15.
- An earlier failed launch used a checkpoint subdirectory instead of its parent restore directory; it produced no checkpoint and is retained separately as a setup-failure artifact.


## RL-N14 multi-seed exploration screening (2026-10-10)

Four 20k continuation runs were completed serially from the seed-specific 60k SAC checkpoints: seed1/seed2 with fixed reset and with random feasible phase reset `[0.2,0.8]`. All actor and learner processes exited successfully and produced 5k--20k checkpoints.

- Output: `outputs/differential_lift_feasibility_20261008/rl_n14_multiseed_gate/`; report: `docs/research/rl_n14_multiseed_exploration_2026-10-08.md`.
- Evaluations used 50 fixed held-out validation trajectories under both reset modes.
- The gate failed: seed1/fixed-reset final fixed evaluation was 80% completion with 14% clearance failure; seed2/random-reset final random evaluation was 86% completion with 12% speed failure. Seed2/random also fell to 50% completion and 44% clearance failure at 10k before partial recovery.
- No 60k continuation was started, and N15 is blocked. The result supports operational checkpoint continuation, but not stable multi-seed exploration benefit or generalization.

## RL-N15 — online efficiency (2026-10-10)

- Status: measured simulation-side unshielded manifold cycle; full endpoint-aware efficiency gate blocked.
- Output: `outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_summary.json`.
- The frozen RL environment did not invoke N7/N8 nonlinear endpoint/swept checker or governor; no full online efficiency claim is made.
- Differential-lift baseline was not run because no compatible current runner was directly callable.
- Compatibility probe: `serl_clean` MuJoCo 2.3.7 cannot load the frozen model (`weld site1` schema); `mujo` MuJoCo 3.3.6 can load it, but was not mixed into the frozen N15 timing protocol.
- Cold-start/XLA timings are stored separately in `outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_cold_start.json` and excluded from formal P95/P99.
- Next: integrate the same endpoint-aware checker/governor into a timing runner before claiming full closed-loop efficiency; keep this diagnostic immutable.

## N16 — installation-error robustness (2026-10-11)

- Protocol: `docs/research/n16_installation_error_protocol_2026-10-11.md`.
- Runner: `hil-serl/tools/run_n16_installation_error.py`; analyzer: `hil-serl/tools/analyze_n16_installation_error.py`.
- Formal output: `outputs/differential_lift_feasibility_20261011/n16_installation_error_formal/`.
- Conditions: fixture translation `{0,1,2,5} mm`, fixture rotation `{0,0.25,0.5,1} deg`, differential base translation `{0,0.5,1,2} mm`, differential base rotation `{0,0.1,0.25,0.5} deg`; gentle/nominal/stress task speeds; 10 paired trials per cell.
- Every formal cell had the target initial-state count; `sampling_failures=0`. The endpoint-aware online governor completed 475/480 trials (98.96%), with median task-progress 0.9997 and no accepted endpoint/swept clearance violation. It stopped 5 times with `governor_exhausted`.
- The stale safe-scaled LUT completed 315/480 trials (65.63%); it had 163 `lut_endpoint_validation` stops and 2 `lut_domain_exit` stops. Its median task-progress among all trials was 1.0000, but its lower-tail progress was much worse because endpoint rejection stops early.
- Median trial P95 step latency was 75.36 ms for the online governor and 11.65 ms for stale LUT. The safety/progress gain therefore carries a substantial online-computation cost.
- Interpretation: within the tested perturbation envelope, evidence supports a conditional claim that online endpoint-aware adaptation is more robust to installation/model mismatch than a stale LUT, at higher latency. This is not a global safety guarantee, branch-preservation evidence, rebuilt-LUT comparison, or RL-generalization result.
- Report: `docs/research/n16_installation_error_results_2026-10-11.md`; tables: `outputs/differential_lift_feasibility_20261011/n16_installation_error_formal/n16_cell_summary.csv`, `n16_method_summary.csv`, and `n16_stop_reason_summary.csv`.


## N17 result (velocity damper and full/reduced QP)

- Protocol: `docs/research/n17_velocity_damper_full_qp_protocol_2026-10-11.md`.
- Raw output: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261011/n17_velocity_qp_formal_v2/n17_velocity_damper_full_qp_raw.json`; summary: `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261011/n17_velocity_qp_formal_v2/n17_summary.json`.
- Paired groups: 48; instrumentation_unknown=0.
- `online_endpoint_governor`: completion 48/48 (100.0%), median P95 step 69.34 ms, median max closure 3.87e-05.
- `full_joint_qp_governor`: completion 48/48 (100.0%), median P95 step 69.18 ms, median max closure 7.55e-05.
- `velocity_damper`: completion 10/48 (20.8%), median P95 step 129.01 ms, median max closure 0.0137.
- `velocity_damper_governed`: completion 26/48 (54.2%), median P95 step 305.32 ms, median max closure 0.014.
- This node does not run rebuilt-LUT, RL, TSR, or GSR. It tests whether a classical velocity damper plus task-speed backtracking reproduces the online governor and whether full joint-space QP matches the reduced representation.
- Solver/endpoint failures are not branch-jump evidence. Final paper wording remains conditional on the tested paired states and installation-error envelope.
- Audit completed below; the detailed decision is recorded in the N17 audit note.

### N17 audit decision

- Audit note: `docs/research/n17_velocity_damper_full_qp_audit_2026-10-11.md`.
- The controlled comparison supports a combined closed-chain, endpoint-aware
  online-continuation claim. A classical velocity damper plus task-speed
  backtracking did not reproduce the result: completion was 10/48 without
  backtracking and 26/48 with it, with mixed closure residual around
  `1.4e-2`.
- The full joint-space equality-constrained QP matched the reduced
  `[r_L,r_R,beta]` governor at 48/48 completion and nearly identical progress
  and clearance. The measured P95 latency difference was about `0.16 ms`, so
  N17 provides no reduced-QP runtime advantage claim.
- The current full-QP implementation is not described as a lexicographic
  hierarchical QP. Solver failure, endpoint rejection, and closure drift are
  execution outcomes, not branch-jump evidence.
- N17 coverage is one initial state per error-level cell (48 paired groups);
  this is a mechanism comparison rather than a 10-replicate success-rate
  estimate. Any primary statistical claim should add replication.
- Next node: decide whether to add replication for the final paper table or
  proceed to manuscript integration with the innovation boundary frozen at the
  combined online governor/closed-chain continuation architecture.

## GFR-0/GFR-1 revised RL plan (2026-10-11)

- Revised protocol: `docs/research/gfr_rl_revised_plan_2026-10-11.md`.
- The first RL iteration is frozen to a consistent two-dimensional action contract: the governor selects `beta_g`, and RL selects `r=[r_L,r_R]` in the corresponding two-dimensional linear-feasible slice. RL does not directly control `beta` in GFR-1.
- GFR-1 keeps fixed duration, `dt`, safety thresholds, governor, and nonlinear checker. It tests error-conditioned observations, structured feasible-action mapping, and execution-feedback supervision separately.
- GFR-2 variable-duration actions are deferred until GFR-1 passes; requested duration must remain distinct from executed duration and cannot bypass verification.
- GFR-0 has now passed: the algebraic contract, the existing real MuJoCo Node-0
  contract, and a 64-state real MuJoCo fixed-`beta_g` two-dimensional slice
  probe all report `contract_status=pass`. Outputs are under
  `outputs/differential_lift_feasibility_20261011/gfr_gfr01/`.
- A separate `aviator_gfr` entry point and `GFRManifoldEnv` were added without
  modifying the historical LUT environment. A short GFR-1 smoke (seed 11,
  256 online transitions and updates per condition, fixed 50/50 demo/online
  sampling) completed B0--B4 with checkpoints and diagnostics. B4's optional
  correction loss was exercised with `lambda_corr=0.1` and produced finite
  `correction_loss` diagnostics.
- The short smoke is an implementation/preflight result only. B0--B3 were
  intentionally run with JAX CPU while the CUDA visibility issue was
  diagnosed; B4 and the backend probe used the RTX 6000 Ada GPU. The four-
  episode evaluation has zero completions in this short horizon and cannot
  support an RL superiority or generalization claim. A device-matched,
  multi-seed, longer GFR-1 run remains required.
- N14/N15/N16/N17 artifacts remain immutable.
- A reward-contract probe initially failed because the environment exposed only
  progress/intervention reward. The GFR environment was corrected to expose the
  frozen progress, intervention, recovery, compute, and terminal components;
  `tools/test_gfr_reward_contract.py` now passes. A post-contract B0--B4 smoke
  (GPU, seed 11, 256 transitions/updates per condition) completed normally.
  Four-episode deterministic evaluation remained at 0 completion for every
  condition. The result is therefore diagnostic: demos are incomplete,
  `random_steps=0` leaves exploration under-covered, and task-level oracle
  feasibility has not yet been separated from learning failure. The pre-
  contract runs are retained as implementation diagnostics, not final RL
  evidence.
- Interface audit after the reward-contract correction confirmed that the
  visible closure-error slice is `state[18:30]` (initial norm `6.56e-5`),
  while the same slice is exactly zero when error observation is disabled.
  Candidate and governor-exhaustion info now expose both
  `endpoint_validation` and `swept_validation` explicitly.
- A separate two-trajectory zero-phase feasibility probe was added at
  `outputs/differential_lift_feasibility_20261011/gfr_gfr01/feasibility_probe/`.
  Fixed zero phase stopped at progress `0.0258/0.0278`; structured zero phase
  reached `0.0567/0.0418`, with all four stops classified as
  `governor_exhausted`/`endpoint_validation` near `d_safe`. This is a backend
  diagnostic only: a feasible phase policy or an oracle reference is still
  needed before interpreting the zero-completion RL smoke as a learning result.
- The corrected GFR trace contains 333 rows across B0--B4, with no missing
  required action, duration, validation, progress, or termination fields. The
  reward-contract probe and the 17 existing manifold/safety tests pass, and
  `git diff --check` is clean.
- Runtime beta audit found that the pre-fix `_candidate` passed an already
  beta-scaled task velocity into `_lift`, applying beta twice for slowed
  candidates. After the fix, all five beta levels satisfy the tangent
  residual contract below `6e-17`; paired two-trajectory rollouts still stop
  early, but closure residual is reduced to the `1e-5` range. The paired audit
  is in `gfr_gfr01/runtime_audit_20261012T110330Z/beta_pair_summary.json`.
- B4 correction supervision now requires positive executed beta and no
  recovery, so governor exhaustion cannot be used as an action target. The
  dedicated correction-mask regression test passes 4/4. Existing B4 smoke
  outputs precede this fix and remain diagnostic only.

### GFR-1 post-runtime-fix B0--B4 smoke (2026-10-12)

- The formal runner was corrected to use an explicit transition budget and to
  collect the learner's `AVIATOR_DIAGNOSTIC_LOG`; the previous runner could
  report a false actor step-cap failure and no checkpoint because it treated
  `max_steps` as an episode-limited run and searched only a nested checkpoint
  directory.
- A fresh serial seed-11 B0--B4 smoke used the beta-corrected backend and
  post-fix demo buffers. All five actor/learner pairs exited with return code
  0 and produced `checkpoint_128`, `checkpoint_256`, and diagnostics JSONL.
- Output:
  `outputs/differential_lift_feasibility_20261011/gfr_gfr01/post_fix_smoke_B0_B4_runner_v2/`.
- Four deterministic val episodes at each checkpoint_256 still gave 0/4
  completion for B0--B4. Mean progress was B0 0.0311, B1 0.0181, B2
  0.0135, B3 0.0083, and B4 0.0729. These are short diagnostic evaluations,
  not RL effectiveness evidence.
- The result confirms that the training/checkpoint/diagnostic chain is
  operational after the runtime fixes. It does not distinguish insufficient
  demonstrations, insufficient safe exploration, or lack of a feasible
  phase-rate sequence. GFR-1 remains unverified until an oracle-feasible or
  complete demonstration reference is established under the same backend.

### GFR-1 cause analysis and local oracle probe (2026-10-12)

- Cause analysis: `docs/research/gfr1_cause_analysis_2026-10-12.md`.
- The post-fix demo manifests contain no successful episode: all four demos
  in each mode terminate with `governor_exhausted`; maximum progress is 0.301
  for structured-visible and 0.214 for raw-hidden. This makes the current
  50/50 demo/online short run unsuitable for judging convergence.
- A separate raw-mode local oracle scanned physical phase-rate candidates at
  every step for the first 100 steps of four val trajectories using the same
  nonlinear checker and beta schedule. Every step had at least one accepted
  candidate; minimum sampled feasible rates were 0.5625, 0.4125, 0.2125 and
  0.1750, and the fastest accepted candidate had beta=1 with about 8--9 mm
  clearance at step 100.
- This supports “short training and insufficient demonstration/exploration”
  as the leading explanation for the 0/4 evaluations. It does not establish
  full-horizon feasibility: the oracle covers only the first 100 steps and
  no claim of RL effectiveness is allowed yet.
- Oracle artifact:
  `outputs/differential_lift_feasibility_20261011/gfr_gfr01/feasibility_probe/gfr1_local_oracle_probe_100step_20261012.json`.
- Next node: obtain a full-horizon oracle-feasible or high-quality successful
  demonstration under the frozen checker/governor, then rerun a predeclared
  multi-seed B0--B4 training/evaluation gate.

### GFR-1 successful-demo correction (2026-10-12)

- The earlier GFR demo source was rejected: it was generated by zero/random
  phase actions and every episode ended at `governor_exhausted`.
- New data plan:
  `docs/research/gfr1_success_demo_100_400_plan_2026-10-12.md`.
- Target collection is 100 complete `online_endpoint_governor` demonstration
  episodes and 400 online exploration episodes. Existing N17 provides 48/48
  complete trials (4,800 accepted steps); N16 provides 475/480 complete
  trials, but these JSON traces need conversion or replay to produce the
  GFR-format observation/action transitions.
- Demo/online collection ratio is distinct from learner batch ratio. The
  proposed predeclared schedule is 50/50 demo/online for the first 10k
  updates, then 25/75 for exploration, with both ratios retained as an
  ablation. Failed and safety-stop transitions remain in online replay, not
  in the successful demo buffer.
- The detailed replay completed 96 trials from 32 states at three speeds and
  four additional gentle trials, all with `complete=true`, yielding exactly
  100 source-success demonstrations and 10,000 transitions. The converted
  buffers are under
  `outputs/differential_lift_feasibility_20261012/gfr_success_demo_100/`.
- The nominal backend audit passed 99.81% of endpoint steps and 99.73% of
  swept steps. Maximum nominal closure residual was `2.57e-3` because many
  source successes came from installation-error models; this is recorded as
  model-mismatch evidence, not hidden as perfect nominal data.
- The source traces now include `r_exec`, `x/q` before and after each accepted
  step, and `xdot_nom`. The old N17 JSON remains untouched.
- A seed-11 B0--B4 256-update smoke using the 100-success-demo buffers
  completed normally for all conditions. Four deterministic val episodes per
  condition still gave 0/4 completion, so the run is a training-chain and
  data-source validation, not an effectiveness result. The new B3/B4 outputs
  are under
  `outputs/differential_lift_feasibility_20261012/gfr_success_demo_smoke_B0_B4_v2/`.
- The next formal run must use a substantially longer frozen budget and
  source-task evaluation before held-out evaluation. A short checkpoint with
  successful demonstrations is still not evidence of RL convergence.

### GFR-1 B4 60k / 16-env run (2026-10-13)

- Output:
  `outputs/differential_lift_feasibility_20261012/gfr_success_demo_B4_60k_16env_seed11/`.
- B4 completed 60,000 online transitions with 16 environments, 100 success
  demos, 50/50 demo/online sampling, and checkpoints every 5,000 steps. Actor
  and learner both returned 0.
- The run is operational but not yet certified as converged. Mean predicted-Q
  dispersion increased from 32.3 in 0--10k to 92.5 in 50--60k; temperature
  increased from 0.108 to 0.372 and critic-loss spikes above 100 remained.
- A 50-episode single-environment evaluation was stopped because the
  endpoint/swept checker made it excessively slow and produced no valid output.
  Do not select checkpoint_60000 by training diagnostics alone.
- Diagnostic artifact:
  `gfr_success_demo_B4_60k_16env_seed11/training_diagnostic_windows.json`.
- Next: evaluate checkpoints 40k/50k/60k in smaller batches with an explicit
  per-episode step cap, then run at least one independent seed before claiming
  RL effectiveness or convergence.

### GFR-1 B4 checkpoint evaluation (2026-10-13)

- Fixed-val evaluation completed for checkpoint_40000, checkpoint_50000 and
  checkpoint_60000, 10 episodes each.
- Completion was 0/10 for all three. Mean progress was 0.2372, 0.3099 and
  0.2130 respectively; mean intervention steps were 162.4, 299.7 and 331.8.
- Every episode stopped with `governor_exhausted`. Endpoint rejection counts
  were 161.4, 298.7 and 330.8 on average.
- Checkpoint_50000 had the highest partial progress but still no completion;
  checkpoint_60000 regressed and intervened more often. No convergence or RL
  effectiveness claim is allowed.
- Summary:
  `gfr_success_demo_B4_60k_16env_seed11/eval_val_10/checkpoint_comparison_summary.csv`.

### GFR-1 B4 root-cause diagnosis (2026-10-13)

- Detailed diagnosis:
  `docs/research/gfr1_root_cause_b4_60k_2026-10-13.md`.
- The zero completion is caused by repeated rejection at the 5 mm clearance
  boundary, followed by `governor_exhausted`; it is not evidence of collision
  penetration or a branch jump.
- The 100 successful demos are valid source-task demonstrations, but their
  initial speed range (`0.03--0.12`) does not cover the val range
  (`0.015--0.661`), and their task starting states only partially overlap the
  val states. They therefore cannot be treated as pointwise labels for the
  current val trajectories.
- The nominal audit found endpoint/swept pass rates of `0.9981/0.9973` and a
  maximum nominal closure residual of `2.57e-3`, so a small source-model to
  nominal-backend mismatch remains.
- Training diagnostics show Q dispersion increasing from `32.3` to `92.5` and
  temperature from `0.108` to `0.372` over 0--60k. The 60k checkpoint is not
  certified as converged; 50k's higher partial progress is not a valid model
  selection criterion because completion remained 0/10.
- An independent oracle found accepted candidates throughout the first 100
  steps of four val trajectories, so initial local infeasibility is not the
  leading explanation. Full-horizon feasibility is still unverified.
- Next: generate a full-horizon reference under the same nominal checker,
  clean or recollect nominal-compatible demos, and compare source-task with
  held-out-val performance before another effectiveness claim.
- Correction: the existing three source-speed labels (`0.03/0.08/0.12`) are
  insufficiently diverse for the val range (`0.015--0.661`). New demos must
  be collected in val-derived speed bins, with multiple starts and directions,
  and must pass the same nominal endpoint/swept checker. Simple time scaling of
  old demos is not accepted as a substitute.

## RL speed-stratified demo retraining gate — 2026-10-13

- New demo set: 100 successful online-endpoint-governor episodes, 10,000 transitions, five speed bins with 20 episodes each.
- Training: B4, 16 environments, 50% demo / 50% online, fresh output under `outputs/differential_lift_feasibility_20261013/gfr_async_gate_16env_seed11_retry1`.
- Valid independent evaluation: checkpoint 20k = 0/10 completion, mean progress 0.1739; checkpoint 40k = 0/10 completion, mean progress 0.1884.
- Both checkpoints showed repeated endpoint rejection and governor exhaustion; no 60k continuation is justified.
- The first asynchronous evaluator had an import-path failure; its zero result was discarded. The values above come from the corrected evaluator and are the auditable gate result.

## GFR consistent-model environment and full teacher replay gate — 2026-10-13

- Added per-episode model manifest support to `GFRManifoldEnv`; FK, collision checker, initial q, and closure observation switch together and are cached by model path.
- Built a four-record manifest covering two val trajectories under nominal and 1 mm fixture-translation models. Initial closure P95 was `6.95e-8`; minimum initial clearance was `8.04 mm`; no initial contacts.
- Full teacher replay passed for one nominal episode and two fixture-translation episodes. All reached `termination=end`, progress `1.0`, and `instrumentation_unknown=0`.
- This gate validates the data interface and long-horizon terminal semantics. It does not yet establish RL effectiveness or installation-error generalization.

## Perturbation training gate started — 2026-10-14

- Train manifest: two models whose full teacher replay passed (nominal and 1 mm fixture translation), 2 complete teacher episodes / 3,392 transitions.
- Held-out manifest: 12 val records from four unseen installation-error models (fixture translation 2 mm, base translation 2 mm, fixture rotation 0.5 deg, base rotation 0.25 deg).
- Async B4 training started with 16 environments, 50/50 demo/online sampling, 60k upper budget, 20k/40k held-out evaluator gate.
- Base translation 1 mm and fixture rotation 0.25 deg teacher sequences stopped near progress 0.54 and were excluded from the demo buffer; they remain failure-boundary evidence.

## Perturbation gate correction — 2026-10-14

- The first perturbation SAC launch used only 2 complete teacher episodes (3,392 transitions) after two of four model teacher replays failed. This was a feasibility run, but it did not satisfy the planned 100-demo formal protocol.
- The run was stopped before any 20k checkpoint and is exploratory only; its checkpoints/logs must not be used as formal RL evidence.
- Formal perturbation training is blocked until 100 complete model-consistent full-trajectory demonstrations are available and teacher-replay audited.

## Formal 100-trajectory teacher collection started — 2026-10-14

- Manifest contains 100 model-consistent `rl_train` records: 50 nominal and 50 fixture-translation-1 mm.
- Full teacher replay is running; no RL process has been started for this formal collection.
- Formal gate remains blocked until the final replay reports 100/100 `termination=end`, zero instrumentation unknown, and all transitions accepted by endpoint/swept checks.

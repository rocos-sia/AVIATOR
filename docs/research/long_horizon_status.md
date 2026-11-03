## 2026-11-03 调度器配对与 G1–G3 泛化验收（运行中）

- 当前节点：冻结60k＋combined修复执行器，四种调度器开发集完整配对；随后冻结G1–G3清单并正式评估。
- 冻结参数：dt=.005、d_safe=.005、beta_stop=.125、r_max=qdot_max=1.5；CPU串行；无训练。
- 已验证：四方法双模型短预检通过；10项计时/缓存/几何测试通过；当前旧回归20/20且替代恢复0。
- 未验证：修复后RL独立增量、新扰动迁移、完整任务与周期计算加速。
- 新结果目录：outputs/differential_lift_feasibility_20261103/gfr_scheduler_generalization/study/；run_status.json可查进度。
- 下一节点：依结果决定多seed或策略调用频率研究；不自动重训。
- 旧结果边界：原20条现在为开发回归；历史N15不代表当前完整安全后端计时；所有旧JSON保留。

# Long-horizon experiment status

## Current node

- Node: frozen60k multi-clearance / affine-backtracking 2x2 ablation (2026-11-03).
- Status: clearance ablation completed; selected combined, completed 20/20, alternative recoveries 0; no training.
- Frozen: final60k checkpoint, original20 manifests, dt=.005, d_safe=.005, beta schedule, six projection trials, original midpoint and endpoint/discrete swept checks.
- Snapshots v1/v2 verified: original 0/198, multi 183/198, affine 181/198, combined 198/198 accepted policy schedules; combined all beta=1. All792 decision records identical after diagnostic assertion fix; 16 tests passed. No whole-task conclusion from snapshot acceptance.
- Correction: reviewer identified original polygon numerical tolerance vs overly strict diagnostic assertion. V2 changes only the assertion and verifies identical decisions before full execution.
- Output: outputs/differential_lift_feasibility_20261103/gfr_clearance_ablation/.
- Next: serial first2 timing pilots, then complete all original20 from initial state with predeclared selected variant; preserve negative outcomes, no retraining or new holdout claim.

## Previous node: final60k recovery cause audit

- Node: final60k alternative-recovery root-cause audit (2026-11-03).
- Status: completed; all 198 recovery states across 12 episodes reconstructed and replayed. No training or executor changes.
- Finding: all policy schedules failed clearance validation; 179 events included a candidate protecting the previously closest arm/wall while another arm/wall became unsafe. First r-halving left the affine set in 191 events and worsened clearance in 117 events (overlapping counts).
- Correction: the earlier 191 swept-only interpretation is withdrawn; swept_min=None was counted as an evaluated swept failure. Internal tracing confirms endpoint failures. Original JSON remains unchanged.
- Evidence: outputs/differential_lift_feasibility_20261103/gfr_recovery_root_cause/full/.
- Report: docs/research/gfr_final60k_recovery_root_cause_2026-11-03.md.
- Verification: accepted-control replay and final alternative candidates agree to floating-point precision; original input/source hashes unchanged.
- Next: separate same-state ablations of multiple near-obstacle constraints and feasibility-preserving backtracking with frozen 60k weights; no retraining yet. These test trajectories are now diagnostic/development data for future modifications.

## Previous node: prospective new-trajectory evaluation

- Node: GFR final60k prospective new-trajectory evaluation (2026-11-03).
- Status: completed; 20/20 complete, mean progress=1.000000; accepted-validation violations=0; protected input hashes unchanged.
- Checkpoint: final 60k from gfr_repaired_scratch/formal_seed11; no training or checkpoint selection on test.
- Executor: unchanged combined_repair_v1; original endpoint/discrete swept checks and safety thresholds.
- Data: test/traj_0010–0029; 4 existing perturbation models x 5 trajectories. Content-disjoint from 550 dp_train/rl_train/val trajectories and previously used test_0000–0009. 326 lineage metadata files audited; 20/20 starts valid.
- Boundary: new-trajectory evidence within previously development-tested perturbation models; no unseen-model or global safety claim.
- Output: outputs/differential_lift_feasibility_20261103/gfr_final60k_holdout/evaluation/.
- Report: docs/research/gfr_final60k_new_trajectory_validation_2026-11-03.md.
- Verification: 11 tests passed; evaluator control logic matches frozen development evaluator, only backend wiring and evidence-role metadata differ; checkpoint and source hashes checked before and after evaluation.
- Next: completed; subsequent recovery diagnosis is recorded above.

## Previous node: completed repaired-executor scratch training

- Node: GFR repaired-executor SAC from scratch (2026-11-02).
- Status: completed; 60,000 online transitions, Actor/Learner and all three evaluators returned 0. 20k/40k/60k each completed 10/10 development-val episodes with progress=1. Accepted-validation violations=0 in all three reports. This does not establish unseen-data generalization or policy-only safety.
- Initialization: random Actor/Critic/temperature, seed 11; no old SAC, BC, or smoke checkpoint initialization.
- Backend: frozen combined alternative recovery + joint-limit-only midpoint defect compensation; original safety thresholds/checker, reward coefficients and observation semantics retained.
- Data: existing 100 successful complete teacher episodes / 302,125 transitions; demo/online minibatch fixed 50/50. 100 reset/first-step compatibility checks passed; no claim of exhaustive interior re-replay.
- Budget: 16 logical environments, sequential geometry and batched CUDA policy; 60k transitions cap, checkpoints every 5k; 400 episode target subordinate to transition cap.
- Gate: CPU fixed10 development-val eval at 20k while training continues; at 40k wait for both reports; both zero stops, otherwise eligible for 60k. No training/test claim from nonzero alone.
- Test isolation: unseen test10 trajectories from 2026-11-01 remain excluded from demo/training/gate.
- Verification: 43 related tests passed; CUDA smoke 128 transitions passed, actor_rc=learner_rc=0; batch diagnostics confirm 128 demo + 128 online.
- Output: outputs/differential_lift_feasibility_20261102/gfr_repaired_scratch/formal_seed11/.
- Report: docs/research/gfr_repaired_executor_scratch_training_2026-11-02.md.
- Results: mean alternative recoveries per episode at 20k/40k/60k = 0.4/23.6/0.0; boundary corrections = 408.3/312.2/593.9; interventions = 747.5/1274.0/1019.2. Completion is consistent across these checkpoints, but intervention costs are not monotonically improving.
- Next: compare policy intervention costs under the same repaired executor and evaluate separately frozen test tasks; no additional training or evaluation launched by this status check.
- Historical artifacts: N1–N17, previous SAC/BC checkpoints, teacher buffer and repair regression JSON remain unchanged. Previous original-executor 0-completion evidence does not describe this newly repaired execution backend.

## Previous node: completed unseen-trajectory validation

- Node: GFR 40k unseen-trajectory recovery validation (2026-11-01)
- Status: completed and audited; original 0/10 complete, mean progress 0.3045; alternative_only 10/10 complete, mean progress 1.0000; combined 10/10 complete, mean progress 1.0000. No training.
- Frozen checkpoint: outputs/differential_lift_feasibility_20261030/gfr_sac_bc_warmstart/formal_seed11/training/b4_seed11/checkpoint_40000.
- Dataset: outputs/differential_lift_feasibility_20261101/gfr_unseen_recovery_selection/heldout_manifest.json. First 10 sorted content-disjoint test trajectories, balanced round-robin across the same four perturbation models (3/3/2/2); no outcome-based replacement. All 10 initial states valid.
- Leakage audit: exact trajectory-array content compared against all 550 dp_train/rl_train/val files; no overlap. Historical test-path reference search in current GFR lineage and repair records found none. This is new-trajectory evidence within existing geometry, not arbitrary unseen-model generalization.
- Execution: frozen recovery selector and original nonlinear safety checker; full progress horizon, identical reset per pair, deterministic actor, CPU serial, per-step JSONL and immediate episode results.
- Output: outputs/differential_lift_feasibility_20261101/gfr_unseen_recovery_validation/; console log: outputs/differential_lift_feasibility_20261101/gfr_unseen_recovery_validation.log. Final summary/report/verification are produced only after all 30 runs complete.
- Next: review completion, progress, accepted safety checks, recovery and unit-progress computation costs. Preserve remaining failure snapshots; keep this test set out of the demo buffer. Decide correction collection on separate training tasks, and do not start training automatically.
- Validation: 35 tests passed; Python compilation and git diff --check passed. Previous full regression remains immutable: original 0/10, alternative-only 10/10, combined 10/10 on diagnosed cases.

## Historical frozen protocol (N1)

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


## GFR formal training resumed with 100 complete teachers — 2026-10-28

- Current node: GFR-1 B4, seed 11, 20k/40k asynchronous checkpoint gate. Training was explicitly authorized after the 100-trajectory collection gate.
- Dataset: 98 successful shard episodes plus 2 successful replacements; the two `teacher_no_feasible_candidate` episodes are excluded. Exactly 100 full trajectories, 50 nominal and 50 fixture-translation-1 mm, with 302,125 transitions. Full-task terminal flags, finite values, state continuity, action/trace alignment, initial model consistency and recorded accepted-step clearance passed. This audit does not claim a new independent replay of every stored step.
- Immutable merged buffer and audit: `outputs/differential_lift_feasibility_20261028/gfr_complete_teacher_gate/demo_buffer/`. SHA-256 of `structured_visible.pkl`: `b070a7d5fc9c3c91f409bece86f9e93ea87c978ffb1a9b5f73499363ac2b9314`.
- Formal run: `outputs/differential_lift_feasibility_20261028/gfr_complete_teacher_gate/formal_seed11/`. `frozen_config.json` records input/code hashes, exact command and settings; `run_status.json` records live state.
- Frozen: Python `gfr_clean`, CUDA device 0, B4 correction weight 0.1, critic clipping 1.0, 50/50 demo/online batches, dt 0.005, d_safe 0.005, beta_stop 0.125, r_max/qdot_max 1.5, 16 logical environments. Existing vectorized actor batches policy inference but steps environments sequentially; no claim of 16 simulator worker processes.
- Budget: at most 60,000 online transitions, checkpoints every 5,000 updates. 400 exploration episodes remains a target; the transition cap and checkpoint stop gate take priority. Actual completed episodes are logged and must not be reported as 400 by assumption.
- Independent evaluation: fixed 10 held-out model/val-trajectory cases, disjoint from teacher models and training trajectories, deterministic actions, CPU execution, complete-path step budget derived from trajectory length and frozen beta floor. One JSONL row is flushed after each episode. Legacy projection/retry counts are distinguished from actual endpoint/swept failed-candidate counts and executed-state clearance.
- Gate: training continues during 20k evaluation; after the 40k checkpoint the learner waits for both evaluations while serving actor status requests. Two valid zero-completion results stop further updates. Any evaluation instrumentation error stops as an error, never as a zero success rate. Otherwise continuation to 60k is allowed without a convergence claim. A stop checkpoint retains the actual update count and is not relabelled 60k.
- Verification: 17 targeted tests passed. A separate 256-transition CUDA preflight loaded all 302,125 demos, sampled 128 demo + 128 online transitions per batch, saved checkpoints 128/256, and both actor/learner exited 0. Preflight is infrastructure evidence only.
- Previous N1–N17 results, old failed training runs and source teacher records were not overwritten. `gfr_env.py` and the teacher collector retain their pre-existing working-tree edits.
- Not yet validated: full-trajectory learned completion, convergence, generalization to held-out installation errors, or reduced intervention cost in this new run.
- Next node: inspect the automatic 20k/40k reports and their failure categories before deciding whether to continue beyond the gate.


## GFR complete-teacher run stopped at 40k — 2026-10-29

- Current node: the formal seed-11 B4 run is finished. The automatic gate returned `stop_both_zero`; no 60k checkpoint was produced. Actor and learner both returned 0, and both evaluators returned 0 with `instrumentation_unknown=0`.
- Actual budget consumed: 40,000 online transitions and 40,000 learner iterations. The runner's `steps=60000` / `steps_requested=60000` fields are requested upper bounds, not completed training steps. Actor assigned 99 episodes; the episode JSONL contains 83 terminal records. This is not a 400-completed-episode result.
- 20k held-out evaluation: 0/10 complete, mean progress 0.30968193, mean intervention steps 418.0; all 10 terminated with `governor_exhausted`.
- 40k held-out evaluation: 0/10 complete, mean progress 0.30683878, mean intervention steps 146.2; all 10 terminated with `governor_exhausted`.
- Sampling remained 128 demo / 128 online per batch. All 302,125 demonstration transitions remained in the independent demo buffer.
- The two fixed held-out evaluations use unseen models and val trajectories. Their zero completion does not alone isolate failed acquisition on the training distribution from model/trajectory generalization failure. Governor exhaustion is a finite-candidate rejection outcome, not proof of global infeasibility or a branch jump.
- Results: `outputs/differential_lift_feasibility_20261028/gfr_complete_teacher_gate/formal_seed11/gfr1_async_gate_summary.json`; detailed episodes are in its `evaluations/` directory. Raw JSON/checkpoints were not modified.
- Next diagnostic: evaluate the same checkpoints on teacher-covered training-model tasks, and test the deterministic teacher on the held-out tasks, before choosing another training revision. No further training has been started.

## Successful-paradigm comparison and local failure probes — 2026-10-29

- Current node: read-only diagnosis of the stopped October 28 run. No training was restarted and no safety, reward, sampling or checkpoint parameters were changed.
- New evidence: `outputs/differential_lift_feasibility_20261029/gfr_training_paradigm_audit/comparison.json` and `teacher_covered_probe_v2.json`. The comparison verifies source hashes; both checkpoint probes verify checkpoint hashes. The first local-probe attempt stopped while serializing undefined failure diagnostics (`d_min`/`max_qdot`); its empty JSONL is not experimental evidence. The v2 report preserves these undefined fields as null with explicit field paths.
- Confirmed historical difference: both successful N13R seeds loaded `rl_n13r_bc_warmstart/bc_seed_0/checkpoint_19990` into the SAC actor (confirmed by learner logs). Current GFR started a fresh SAC actor. A 50/50 replay mixture is not teacher-action cloning; B4 supervision targets executed corrections on eligible transitions, not all teacher raw actions.
- Demonstration audit: old 64 episodes / 64,595 transitions, mean length 1,009.30; current 100 / 302,125, mean length 3,021.25. Mean nonterminal reward changed from +1.14237 to -0.01231. At unchanged gamma=0.999, current successful demonstrations have mean discounted return -2.17254. This increases the credit-assignment concern, but negative return alone does not prove that failure is preferable.
- Current demonstrations are complete and model-consistent, but 49.817% of stored next states have clearance at most 5.01 mm (the threshold is 5 mm); 65.461% qualify for correction supervision. Successful demonstration is not evidence of a large imitation-error margin. The historical LUT clearances are a different checker, so their values are not a matched robustness comparison.
- Buffer dilution was not observed: every sampled diagnostic batch remains 128 demo / 128 online. No nonfinite actor/critic diagnostic values were observed at the 100-update logging cadence. This does not establish critic accuracy or rule out unlogged transient spikes.
- All 83 completed stochastic training episodes stopped with `governor_exhausted`; mean progress was 0.19219. Therefore held-out model mismatch is not the only issue. These training episode records are not a deterministic final-policy training-set evaluation.
- Bounded deterministic 40k check: selected the first teacher-covered task for each of the two training models before running, capped at 1,024 steps. Nominal stopped at step 124 / progress 0.06887; fixture translation 1 mm stopped at step 301 / progress 0.16254. At each exact stopped state, the existing 25-action teacher search found a candidate at beta=1 satisfying the same endpoint, swept, speed and joint-limit checks. Nominal raw witness [0,1], perturbed raw witness [0,0] (projected by the same filter). Closure residuals at the stops were about 3.5e-7. These witnesses establish only a legal next step, not recoverability of the entire trajectory.
- Mechanism: `GFRManifoldEnv.step` tries the policy's one nominal action across five beta values. `project_r` projects once and then halves the chosen r up to six times; it does not search other directions. The teacher additionally searches alternative nominal actions. The observed governor exhaustion is thus candidate-search exhaustion in these cases, not an empty nonlinear feasible set. Shrinking r toward zero need not stay in an affine safe slice that excludes zero.
- Policy probe: 800 evenly spaced teacher states (8 per episode), 40k mean raw-action L2 deviation 0.86090; only 1.625% within 0.1. Different actions can also be valid, so action disagreement is not itself proof of failure or critic overestimation.
- Next diagnostic/revision: first validate a GFR-compatible teacher imitation initialization on teacher-covered tasks; retain the same safety backend and evaluate checkpoint behavior before adding SAC exploration. Separately test teacher feasibility on the held-out tasks. BC initialization, reward/time-scale changes and stronger action recovery require separate controlled comparisons; this audit does not identify one exclusive causal factor. Do not copy the historical LUT BC weights directly into GFR, whose observation/action semantics differ.

## GFR-compatible BC initialization and execution gate — 2026-10-29

- Current node: execute the user-authorized missing imitation stage before any SAC exploration. All 100 audited full teachers / 302,125 transitions train the existing GFR-compatible Actor with its existing BC likelihood loss. Offline BC uses 100% demo; the subsequent SAC batch ratio remains fixed 50/50.
- Formal output: `outputs/differential_lift_feasibility_20261029/gfr_bc_teacher_gate/formal_seed11/`. `frozen_protocol.json` records data/code hashes and the predeclared task selection; `status.json` records progress; checkpoints are in `bc_seed11/`.
- Frozen BC: seed 11, 20k updates, batch 256, Adam 3e-4, unchanged 40-D observations and raw 2-D actions, original reward/governor/checker. No old LUT weights, observation normalization changes, reward shaping changes, or alternative-action recovery were introduced.
- Gate: deterministic full-horizon evaluation of 10k and 20k checkpoints on the same 10 teacher-covered tasks, five per model, selected before training by sorted task IDs 0/12/24/36/49. Both must complete at least 9/10, with at least 4/5 for each model and no accepted-check violation or instrumentation error. This is a development gate for imitation competence, not generalization evidence. Only final 20k BC may initialize a fresh SAC run.
- Instrumentation: per-episode JSONL plus per-step progress-aligned teacher action and joint-state comparisons; raw/executed actions, intervention flags and candidate rejection reasons remain distinct. A progress-aligned action difference is not an action-optimality label once rollout states diverge.
- Verification before the formal run: 17 gate/reward/runtime/correction tests passed; a separate 256-update CUDA BC smoke loaded all teachers, saved/restored its checkpoint, and transferred Actor weights into a fresh SAC with exactly identical deterministic actions on 800 probe states while leaving Critic weights unchanged. Smoke results are infrastructure evidence only.
- SAC conditional launch: requires the qualification report to pass again in the launcher and requires the selected BC checkpoint/demo hashes to match. Otherwise no SAC process starts. If qualified, the previous 20k/40k asynchronous held-out gate and 60k upper budget apply in a new `sac_from_bc/` directory.
- Old failed runs, teacher JSON/pickle data, and historical checkpoints remain unchanged. Reward/time-scale and alternative-action recovery ablations are deferred until the imitation gate result is known.

## GFR BC execution gate failed; SAC not started — 2026-10-29

- Current node: the 20k-update BC run and both full-horizon teacher-covered evaluations finished normally. Qualification is `stop_bc_not_stable`; `status.json` records `sac_started=false`. No SAC continuation was launched.
- Frozen inputs were all 100 teachers / 302,125 transitions, original 40-D observations, raw 2-D action labels, seed 11 and the existing likelihood objective. CUDA training completed 20,000 updates in 41.23 seconds; these are offline minibatch updates, not environment transitions. Both CPU evaluators returned 0 and reported zero instrumentation errors.
- Fixed 800-state in-sample reconstruction: mean raw-action L2 error decreased from 0.83232 at initialization to 0.31188 at 10k and 0.23065 at 20k. At 20k the P95 error remained 0.94567 and only 53.75% of probe states had error below 0.1. Better likelihood and reconstruction did not establish closed-loop competence.
- Teacher-covered evaluation (five tasks per model, selected before training): 10k completed 0/10, mean progress 0.0248803, mean intervention 4.7; 20k completed 0/10, mean progress 0.0442462, mean intervention 32.8. All 20 episodes terminated with `governor_exhausted`. No accepted-step checker violation was recorded. Actual endpoint-failed candidate counts were 0/2 and swept-failed counts 26/113 respectively; legacy projection/retry counters are not endpoint failure counts. These results do not establish continuous-time safety or global infeasibility.
- Concrete rollout evidence: on the first nominal task, the 20k teacher's reference raw action is zero for the first 26 steps. BC starts with raw-action L2 error 0.01399, reaches 1.40038 at step 20 and stops at step 26. At that stop clearance is approximately 8.3 mm, so the failures cannot all be attributed to crossing the 5 mm clearance threshold. Candidate rejection currently groups linear-slice and endpoint checks; a finer constraint-level explanation is still needed.
- Isolated diagnostic: `bc_feedback_probe_v2.json` keeps all recorded teacher observation fields fixed except previous executed r (indices 33:35), which is recursively replaced by 1.5 times the preceding policy output. On the first 26 nominal teacher states, the 20k policy's last action error is 0.01428 with teacher inputs but 1.40074 with this recursive channel alone. The corresponding 10k values are 0.06690 and 1.40620. This identifies a policy-input feedback amplification mechanism in a synthetic probe; it does not prove global robot-loop instability or identify the only failure cause.
- The v2 probe uses central finite differences at two increments. The older `bc_feedback_probe.json` automatic-Jacobian fields are invalid as inference sensitivity estimates because the Policy encoder uses stop_gradient; only those fields are superseded. Recursive predictions agree. No policy or encoder was modified. Checkpoint hashes are unchanged.
- New evidence directory: `outputs/differential_lift_feasibility_20261029/gfr_bc_teacher_gate/formal_seed11/`, including `bc_qualification.json`, `bc_execution_summary.csv`, per-step evaluation JSONL and `bc_feedback_probe_v2.json`. Source demo hash remains `b070a7d5fc9c3c91f409bece86f9e93ea87c978ffb1a9b5f73499363ac2b9314`.
- Post-run gate regression repair: validate instrumentation for both reports before returning a completion failure. The exact helper used for the completed run is preserved in `source_snapshot/gfr1_bc_support.py`. Both actual reports are valid, so the repaired gate gives the same decision. All 17 targeted tests pass after this repair.
- Verified: compatible BC training, checkpoint restore, exact Actor transfer into a fresh SAC object without changing Critic, and fail-closed execution gating. Not verified: stable imitation, SAC benefit from BC, convergence, or generalization. Missing BC initialization was not the only obstacle.
- Next node: isolate the previous-action observation feedback and obtain teacher corrections on policy-deviated states in separately frozen comparisons before resuming SAC. Keep reward/time-scale and alternative-action recovery as separate ablations; do not silently weaken the BC gate or select a favorable checkpoint. Historical N1–N17 and prior failed runs remain unchanged.

## BC previous-executed-action observation ablation started — 2026-10-30

- Current node: user-authorized single-variable ablation. The two previous executed r observation channels (zero-based indices 33, 34) are zeroed in both BC training inputs and policy inference inputs. The environment retains the actual previous executed action and unchanged dynamics, reward, governor and checker. Original replay files are read-only.
- Frozen comparison: reuse October 29 original BC as A; train B from scratch with the same seed 11, architecture, negative log-likelihood objective, batch 256, 20k updates and 100 teachers / 302,125 transitions. Evaluate 10k and 20k on the identical ten preselected teacher-covered tasks. No correction data or reward/time-scale/recovery changes are included. No SAC launch is authorized by this runner invocation.
- New output: `outputs/differential_lift_feasibility_20261030/gfr_bc_mask_previous_action/`. `ablation_protocol.json` freezes the comparison and SHA-256 of 39 protected historical/input files; `formal_seed11/frozen_protocol.json` freezes the new run. The previous completion gate remains both checkpoints >=9/10 and >=4/5 per model with no validation violation or instrumentation error.
- Preflight: existing environment/config/BC implementation and demonstration hashes match the baseline. Eighteen targeted tests pass, including source-array immutability and masking only the selected channels across observation batch shapes. Single-seed task-covered results will not be interpreted as generalization or convergence proof.

## BC previous-action ablation completed; execution gate still fails — 2026-10-30

- Current node: masked-observation BC completed all 20k offline updates; 10k and 20k evaluators each completed ten full-horizon attempts and returned 0. All twenty attempts stopped with `governor_exhausted`; `instrumentation_unknown=0`. Decision remains `stop_bc_not_stable`, and no SAC process was launched.
- Paired results (original / masked): at 10k, completion 0/10 / 0/10, mean progress 0.0248803 / 0.0658140, mean intervention 4.7 / 95.7; at 20k, completion 0/10 / 0/10, mean progress 0.0442462 / 0.0489335, mean intervention 32.8 / 18.6. Absolute intervention counts depend on travelled distance and cannot alone establish an efficiency gain.
- Masked 10k recorded 6 endpoint-failed candidates and 73 swept-failed candidates; masked 20k recorded 0 and 14. Both had zero recorded accepted validation violations, with mean episode minimum executed clearance 5.77688 mm and 6.31371 mm. These are frozen checker results, not continuous-time safety guarantees. Stopping reasons still do not establish global or local nonlinear infeasibility.
- Fixed 800-state teacher reconstruction worsened: final mean raw-action L2 0.510686 versus original 0.230652, and fraction below 0.1 was 18.25% versus 53.75%. The mask removes useful predictive information as well as the previously isolated feedback path; additional training was not added after observing these results.
- On the first nominal task, final-checkpoint action error at step 20 is 0.2255 versus original 1.4004; termination moves from step 26 / progress 0.01481 to step 185 / progress 0.10900. The masked policy begins with larger action error (0.1580 versus 0.0140) but avoids the same early saturation pattern. This supports an input-channel effect on this trajectory, not universal stabilization.
- Across paired tasks, progress improves in 8/10 at 10k and 4/10 at 20k (worsens in the remaining 2/10 and 6/10). Therefore the final-checkpoint improvement is not task-consistent, and the original feedback mechanism is not a sufficient explanation of all failures. No completion gain or convergence is established.
- Evidence: `outputs/differential_lift_feasibility_20261030/gfr_bc_mask_previous_action/ablation_summary.json`, `comparison.csv`, `paired_episodes.csv`; raw checkpoints, qualification and per-step logs are under `formal_seed11/`. The summary audit confirms identical evaluation records/budgets and frozen training settings apart from the mask, all 39 protected hashes unchanged, and exactly zero policy inputs at indices 33:35 on every evaluation step. The environment's true previous r is separately logged.
- Code: `gfr1_bc_support.py` provides a non-mutating input mask; `gfr1_bc_teacher_gate.py` and `gfr1_eval.py` apply it consistently behind an opt-in flag; `gfr1_bc_ablation_summary.py` audits the matched experiment. Original default behavior is retained. Masked BC cannot auto-launch SAC through this runner because the SAC observation pipeline would require a matching explicit change.
- Next node: separately test teacher-labelled corrections on states visited by the deviating policy, with the observation version and correction sampling ratio frozen before collection/training. Do not simultaneously change reward/time-scale or alternative-action recovery. No correction collection or new SAC training has started in this ablation.

## User-authorized SAC retry with GFR BC Actor initialization — 2026-10-30

- Current node: launch a fresh SAC run with the original full-observation GFR BC final 20k Actor. The user explicitly authorized BC initialization without requiring BC to complete tasks independently. Prior BC and masked-observation negative results remain unchanged; their gate decision is still `stop_bc_not_stable`.
- New root: `outputs/differential_lift_feasibility_20261030/gfr_sac_bc_warmstart/`. `launch.json` records the supervisor PID and exact command; `formal_seed11/frozen_config.json` records checkpoint/data/code hashes, and `run_status.json` reports live training/evaluation state.
- BC: `outputs/differential_lift_feasibility_20261029/gfr_bc_teacher_gate/formal_seed11/bc_seed11/checkpoint_20000`. Only the Actor is transferred into fresh SAC; no historical LUT weights or masked-observation weights are used. `--bc-initialization-only` retains integrity/instrumentation checks while removing the BC completion-rate prerequisite for this explicitly authorized experiment.
- Frozen: full 40-D observation, 100 complete teachers / 302,125 transitions, 128 demo + 128 online per batch, seed 11, 16 logical environments, B4 correction weight 0.1, critic clipping 1.0, original GFR reward/governor/checker. No correction samples, reward/time-scale changes, or alternative-action recovery changes are introduced. This reproduces the warm-start training pattern while retaining the GFR execution backend, not the old LUT environment.
- Maximum 60k online transitions/learner updates; 400 exploration episodes remains a budget-dependent target. Checkpoints every 5k; 20k and 40k evaluations use the same ten held-out model/trajectory cases as the failed fresh-Actor SAC run. Evaluations run on CPU while CUDA training continues; at 40k wait for both reports. Both zero stops; instrumentation error stops; otherwise continue to final 60k evaluation without a convergence claim.
- Preflight: demo/checkpoint/teacher-source hashes verified, nineteen gate/runtime/reward/correction tests passed, and diff whitespace check passed. The launcher forbids using an initialization-only exception to ignore instrumentation failure or observation-mask mismatch.
- Next node: inspect real Actor initialization, sampling diagnostics, and then the 20k/40k checkpoint results. This entry records launch, not completed training or effectiveness.
- Startup verified: learner log confirms `seeded SAC actor from BC checkpoint`, loads all 302,125 demo transitions and sends initial weights to the Actor. Both Actor and learner appear as CUDA processes. The first inspected diagnostic at update 400 records exactly 128 demo / 128 online per batch, online buffer size 400, and a live supervisor with no training return code yet. The 20k/40k reports are not available at this point.

## BC-initialized SAC stopped by the 20k/40k gate — verified 2026-11-01

- Training finished on 2026-10-30 at approximately 14:52 China time. Actor, learner and both evaluators returned 0. No training or evaluation process from this run remains active at verification. Gate decision: `stop_both_zero`; actual budget was 40,000 online transitions and 40,000 learner updates. No 60k checkpoint was produced; `steps=60000` in the wrapper summary is the requested cap.
- Held-out evaluation: 20k completed 0/10, mean progress 0.1297153, mean intervention 113.9; 40k completed 0/10, mean progress 0.3957522, mean intervention 299.3. All twenty attempts stopped with `governor_exhausted`, and both reports have `instrumentation_unknown=0`. Mean episode minimum clearance was approximately 0.00500003 m and 0.00500001 m. These minima do not identify every rejected constraint or establish global infeasibility.
- Sampling remained 128 demo / 128 online through the final diagnostic; all 302,125 teacher transitions were retained. Online episode logs contain 73 terminal records, all `governor_exhausted`; this is not a 400-completed-episode experiment.
- Compared with the earlier fresh-Actor run on the same held-out cases, final 40k mean progress increased from 0.3068388 to 0.3957522; completion remained 0/10. The new 20k mean progress was lower than the fresh-Actor 20k result. BC initialization therefore has not established reliable task completion or convergence in this single-seed experiment. Progress improvement alone is insufficient.
- Evidence: `outputs/differential_lift_feasibility_20261030/gfr_sac_bc_warmstart/formal_seed11/gfr1_async_gate_summary.json`, `run_status.json`, per-checkpoint evaluation JSON, final training diagnostics and `online_episodes.jsonl`. No retry, hyperparameter changes or continuation beyond the frozen stop gate was launched during this status check.


## 2026-11-01 — 最新 40k 策略失败状态的同状态 teacher 对照（已完成）

### 冻结输入与方法

- checkpoint：`outputs/differential_lift_feasibility_20261030/gfr_sac_bc_warmstart/formal_seed11/training/b4_seed11/checkpoint_40000`；SHA256（与既有 evaluator 口径一致）：`953f0c0c7da65c82b8fb5ac81f1f061dc121bb57c1b8030273c2e150fcc4e88d`。
- 复用原 held-out manifest 的全部 10 个 episode、初始化、确定性 Actor、CPU 评估和完整轨迹时间尺度。
- dt=0.005，d_safe=0.005 m，qdot_max=r_max=1.5，beta_stop=0.125，governor 速率档位保持不变。
- 停止前保存 q、x、任务索引 u、零空间基、历史动作/速率/干预标志、Actor 观测、wrapper 历史、RNG、完整任务数组及 MuJoCo 工作状态。
- 恢复原状态后运行策略分支及 teacher 分支。原 `project_r` 仅加 Python 诊断跟踪，逐候选结果与不开启跟踪时完全一致。
- teacher 初次搜索使用已有 25 个候选动作，固定 seed=20261101+episode，按 beta、clearance 排序；随后沿已有 teacher 规则复用上一动作，失败时才扩展搜索，最多接受 100 步。

### 核心结果

- 10/10 原始停止的步数和进度精确复现；10/10 同状态 teacher 找到合法动作；10/10 连续恢复 100 个接受步。
- 任务进度增加 5.37–6.23 个百分点，平均 5.87 个百分点；全部因预先冻结的 100 步诊断预算结束。没有开展恢复至终点的成功率测试。
- 1000 个恢复步均通过未修改的 endpoint/swept checker、速度和关节规划边界检查；最低 clearance=0.005000004699144724 m，最大闭合残差无穷范数=1.8602053199081033e-6（混合位姿误差，不能全部解释为米）。这是离散检查结果，不是连续时间安全证明。

| Episode | 直接拒绝约束 | 原停止进度 | teacher 100 步后进度 | 增加百分点 |
|---|---|---:|---:|---:|
| 0 | clearance（endpoint 与 swept） | 40.30% | 46.53% | 6.23 |
| 1 | clearance（endpoint 与 swept） | 40.38% | 46.61% | 6.23 |
| 2 | clearance（endpoint 与 swept） | 40.80% | 47.03% | 6.23 |
| 3 | clearance（endpoint 与 swept） | 40.64% | 46.87% | 6.23 |
| 4 | 左臂第 2 关节规划上限 | 54.89% | 60.26% | 5.37 |
| 5 | 左臂第 2 关节规划上限 | 54.93% | 60.30% | 5.37 |
| 6 | 左臂第 2 关节规划上限 | 55.01% | 60.38% | 5.37 |
| 7 | 左臂第 2 关节规划上限 | 54.82% | 60.19% | 5.37 |
| 8 | clearance（endpoint 与 swept） | 7.10% | 13.25% | 6.14 |
| 9 | clearance（endpoint 与 swept） | 6.88% | 13.04% | 6.16 |

### 已定位的机制

1. **6 个 clearance 案例**：策略在 5 档 beta 下均耗尽 Euler 候选的 6 次回退。30 个首次投影候选的线性 clearance 满足阈值，但非线性 endpoint/swept 检查未通过。后续 150 个缩幅候选中，144 个连线性 clearance 约束也不再满足。原因是速度中仍含闭合反馈和任务推进项；安全集对 r 是仿射约束，r 向零缩小并不保持可行性。teacher 改变动作方向后可以恢复。
2. **4 个关节边界案例**：Euler 投影接受，midpoint 更新后左臂第 2 关节超过当前规划上限 94.5°；越界量为 8.37e-9–2.17e-7 rad，高于冻结的 1e-9 rad 容差。此处是很小的数值/离散化边界超限，不应称为明显物理撞限位。原 `endpoint_validation` 标签没有准确表达这一原因。teacher 使用其他动作可通过相同约束。
3. **恢复动作选择存在差异**：当前策略路径对给定 r 做投影、缩幅和 beta 降速；teacher 搜索不同的 r。`governor_exhausted` 表示当前候选恢复路径耗尽，不是证明局部安全集合为空。
4. 首个案例：策略原始动作约 [-0.1745,-0.6750]；teacher 选择 [-1,0]（执行 r=[-1.5,0]，beta=1），从40.30%恢复到46.53%，第100步 clearance 约8.26 mm。

### 可以与不可以得出的结论

- 可以：在这 10 个失败状态上，合法的局部推进仍存在，teacher 不仅找到一步，还连续恢复了100步。不能把这些停止统一归因于任务几何不可行或示教本身不成功。
- 可以：失败链同时涉及策略给出的动作、边界上的线性/非线性检查差异，以及缺乏替代方向搜索的恢复路径。
- 尚不能：仅由本实验判断策略选错动作的训练原因究竟是示教覆盖、奖励、Critic、观测还是训练长度；也不能宣称已经解决全程任务、全局分支保持或泛化。
- teacher 搜索计算预算大于单次 Actor 推理，本实验提供可行性证据，不证明速度优势。
- 这些是 held-out 诊断状态。如果直接用于训练，必须将其从正式未见测试集中移除并另建测试集。

### 输出与验证

- 正式新目录：`outputs/differential_lift_feasibility_20261101/gfr_same_state_40k_v2/`。
- `summary.json`、`diagnostic_analysis.json`、`verification.json`；`same_state_comparison.csv`、`policy_constraints.csv`、`policy_linear_vs_actual.csv`。
- 每个 `episode_XX/`：`pre_stop_state.pkl/json`、`policy_rejection.json`、`teacher_search.json`、逐步追加的 `continuation.jsonl`、`result.json`。
- 23 项测试通过（其中新增诊断测试 4 项），instrumentation_unknown=0；快照哈希复核；原训练冻结的12个代码文件哈希匹配；checkpoint、旧评估文件、manifest 和执行器/求解器文件不变。
- 同级 `gfr_same_state_40k/` 是日志开发阶段中止的预检目录，没有最终结果，不作为证据。

### 下一节点

先用本次快照做候选恢复机制的独立回归：保持 checker/阈值不变，区分替代 r 搜索与积分边界一致性修正的收益；在训练任务上采集类似偏离状态的 teacher 纠偏标签。完成这些独立验收后再决定学习消融，不直接扩大训练时长。


## 2026-11-01 — 替代动作恢复 × 积分边界一致性：2×2 快照回归（已完成）

### 范围和冻结条件

使用上一节点的全部 10 个原始停止前快照。保持 SAC checkpoint_40000、观测、dt=0.005、d_safe=0.005 m、qdot_max=r_max=1.5、beta_stop=0.125、5 档 governor、midpoint 积分和 endpoint/swept checker 不变。关节容差仍为1e-9 rad，速度容差仍为2e-5 rad/s，swept subdivisions=5。

每组每个快照最多100个接受步。与上一节点不同，本轮每一步都重新调用当前40k策略，没有让teacher持续接管。没有训练，也没有修改正式GFR执行器；修改仅存在于隔离诊断工具内。

- 原流程：策略动作、原投影、原恢复逻辑。
- 替代动作恢复：仅当策略所有 beta 候选失败时，依次搜索上一接受候选的名义动作、零动作、正负坐标轴/角点及固定随机候选，总数最多25。按固定顺序取首个合法动作，然后下一周期重新交给策略。这里不是上轮teacher的最大beta/clearance择优搜索。
- 积分边界补偿：仅当候选在midpoint检查中只违反关节边界时启用；不改变策略给出的名义动作，不启用替代动作搜索。测量 delta=q_midpoint_endpoint-q_euler_endpoint，把候选生成用的lower增加max(-delta,0)、upper减去max(delta,0)，累积保留已测补偿，最多3次重算。最终midpoint仍用原始边界独立验收。补偿仅影响候选生成，不放宽物理/规划阈值，不改变积分器。
- 两者结合：先尝试边界补偿；策略候选仍全部失败时才进入替代动作搜索。

执行原始策略动作与实际接受动作分别记录；precomputed接口仍传入原始策略动作，因此执行修正没有从下一步观测中被隐藏。计时含诊断与额外复核，不用作部署速度结论。

### 结果

| 组别 | 首步恢复 | 连续恢复100步 | 平均进度增加（全部10例） | 替代动作介入总次数 | 边界补偿步数 | 候选评估次数 |
|---|---:|---:|---:|---:|---:|---:|
| 原流程 | 0/10 | 0/10 | 0.00 个百分点 | 0 | 0 | 50 |
| 仅替代动作恢复 | 10/10 | 10/10 | 5.76 个百分点 | 28 | 0 | 1345 |
| 仅积分边界补偿 | 4/10 | 4/10 | 2.15 个百分点 | 0 | 33 | 463 |
| 两者结合 | 10/10 | 10/10 | 5.78 个百分点 | 8 | 33 | 1170 |

注意：上表的恢复比例不是全程任务完成率。所有达到100步的运行都由诊断预算结束，没有测试到任务终点。仅边界补偿组的均值包含6个未恢复案例；其4个恢复案例各推进约5.37个百分点。

### 机制判断

1. **替代动作恢复的独立作用成立于这些快照**：10/10均恢复100步。1000个接受步里仅28步需要替代动作，其他972步由策略名义动作经原投影/governor接受。无需持续teacher接管，说明原先的候选耗尽并非这些状态下的真实局部不可行。
2. **积分边界补偿的独立作用成立于关节边界案例**：4/4关节边界案例恢复100步；6个clearance案例仍在原位置停止。它修正的是Euler投影与midpoint端点的关节边界差异，并没有顺便解决clearance问题。
3. **组合的作用**：1000个接受步中仅8步需要替代动作，另有33步使用边界补偿。与仅替代动作组相比，恢复比例同为10/10，替代动作介入从28次降到8次，候选评估数从1345降到1170。补偿本身还有额外lift计算，因此不能把调用数减少直接写成总计算提速。
4. 真实快照单步反例：episode04的原关节裕量=-8.6740715e-8 rad；一次测量补偿后为+2.2998936e-11 rad，通过同一checker。episode00的clearance失败在补偿开关前后均未通过，说明补偿没有放宽clearance阈值。
5. 这说明之前的0/10至少包含可修复的执行候选/恢复机制问题，不能仅凭旧completion=0认定需要更多训练步数。也不能因此声称策略已收敛或整条任务已学会。

### 验证与证据边界

- 40个组别×快照结果；每个组的初始Actor动作与快照完全一致。
- 原流程10个案例的全部5档候选结果，与上一节点原日志逐字段完全一致。
- 共2400个接受步额外调用原endpoint/swept checker复核，并核对速度、关节边界、beta和正任务进度；违规数0。
- 最低恢复clearance=0.005000004699144724 m；最大闭合残差无穷范数=1.298178943676525e-06，是混合位姿误差，不能全部解释为米。
- 27项测试通过（包含真实快照的原失败→补偿通过回归）；git diff --check通过；源快照树、checkpoint、manifest和正式执行器/投影文件哈希不变。
- 这些是已用于诊断的10个相关案例，不是新的独立泛化测试；swept检查是离散采样，不提供连续时间安全证明。
- 组合组与替代组达到相同的100步恢复上限，不能据此宣称组合组全程成功率更高。

### 训练决定与下一节点

本轮不启动训练。先冻结这一隔离实现，保持现有40k Actor，执行从轨迹初始状态到终点的开发回归，对照原执行器、替代恢复和组合方案。该步骤回答“先修执行器后，旧策略到底能完成多少”，而非再次混入学习更新。

若全程表现已有明显改善，优先完成执行器回归及独立测试；若仍有早停，再在训练任务上收集偏离状态的teacher纠偏标签。正式续训前必须检查成功示范在修复后后端的replay一致性、原始/执行动作与奖励计账，并保留50/50 replay及20k/40k门控。不得把本批held-out纠偏轨迹直接用于训练后仍称其为未见测试集。

### 新增文件

- `hil-serl/tools/gfr1_recovery_boundary_ablation.py`：隔离2×2回归及边界补偿。
- `hil-serl/tools/test_gfr_recovery_boundary_ablation.py`：补偿契约、替代动作触发和真实快照回归。
- 结果目录：`outputs/differential_lift_feasibility_20261101/gfr_recovery_boundary_ablation/`。
- `protocol.json`、`summary.json`、`verification.json`、`comparison.csv`、逐完成结果`episodes.jsonl`。
- 每个`episode_XX_方法/`保存首步所有候选`first_step.json`、逐步追加的`trace.jsonl`及`result.json`。

## 2026-11-01: 40k full-trajectory recovery regression completed

- Result directory: outputs/differential_lift_feasibility_20261101/gfr_full_recovery_regression_v2
- original: 0/10 completed; mean progress 0.395752; stops {'governor_exhausted': 10}.
- alternative_only: 10/10 completed; mean progress 1.000000; stops {'end': 10}.
- combined: 10/10 completed; mean progress 1.000000; stops {'end': 10}.
- Original historical stopping points reproduced in 10/10 cases; all accepted steps independently rechecked. Checkpoint, geometry, safety backend and prior evidence hashes unchanged.
- Results describe the unchanged policy PLUS execution recovery, not RL convergence or new held-out generalization. Discrete swept validation is not continuous-time safety proof.
- Next: inspect remaining_failure_constraints.csv and saved pre-stop states; decide correction-data collection on separate training tasks. No retraining was started.

## 2026-11-02: resume interrupted unseen validation

- Processes absent at status check; exit cause unconfirmed. 25 finished case folders retained and hashed; 5 missing cases resume from initial states with frozen seeds.
- Resume runner: hil-serl/tools/gfr1_resume_unseen_regression.py; 15 focused tests passed.
- Resume logs and launch record: outputs/differential_lift_feasibility_20261101/gfr_unseen_recovery_validation/resume_console.log and resume_launch.json.
- Incomplete attempt is not counted as a method failure. Final audit/report still required.

## 2026-11-01: 40k unseen-trajectory recovery validation completed

- Result directory: outputs/differential_lift_feasibility_20261101/gfr_unseen_recovery_validation
- original: 0/10 completed; mean progress 0.304507; stops {'governor_exhausted': 10}.
- alternative_only: 10/10 completed; mean progress 1.000000; stops {'end': 10}.
- combined: 10/10 completed; mean progress 1.000000; stops {'end': 10}.
- Identical paired initial states verified for all 10 new test trajectories; all accepted steps independently rechecked. Checkpoint, geometry, safety backend and prior evidence hashes unchanged.
- Results describe the unchanged policy PLUS execution recovery on 10 new trajectories within the same perturbation models, not RL-only convergence or generalization to arbitrary geometry. Discrete swept validation is not continuous-time safety proof.
- Next: read next_step_decision.json; keep test trajectories out of training, and decide correction-data collection on separate training tasks only after reviewing failures and intervention cost. No retraining was started.

# Claim--evidence matrix for N1--N8 (2026-10-07)

This matrix freezes the evidence boundary for the manuscript after N9 consolidation. N1--N8 result files are read-only inputs. “Supported” means supported only over the tested regular local region, scenes, thresholds, and numerical budgets.

| Claim or component | Status | Evidence | Allowed manuscript wording | Wording excluded |
|---|---|---|---|---|
| Analytic task Jacobian $A_q,A_x$ is implemented correctly on regular states | **Conditionally supported** | Node 0, 32 held-out MuJoCo states; max central-difference errors $3.84\times10^{-10}$ and $4.26\times10^{-10}$ | “Numerically validated on the tested regular states.” | “Correct for all configurations” or a global rank guarantee |
| $H+Nr$ differential lift gives a local tangent continuation | **Conditionally supported** | Node 0 residual/regularity checks; N1 local atlas diagnostics; N2--N8 accepted local traces | “Provides local online continuation inside a tested regular component.” | “Globally branch-preserving” |
| Local component continuity lemma | **Analytically supported under assumptions** | Manuscript lemma assumes a continuous accepted trajectory in one regular connected component | “A continuous accepted trajectory cannot leave its selected connected component without violating the assumptions.” | “The algorithm identifies or preserves every global IK branch.” |
| Three-variable safety QP over $(r_L,r_R,\beta)$ is a convex candidate generator | **Analytically supported under affine local constraints; numerically gated** | Node 0 full/reduced QP comparison; N2--N4 and N8 traces | “Generates a locally linearized candidate action.” | “The QP alone certifies nonlinear or continuous-time safety.” |
| Full joint-space QP and reduced QP are numerically equivalent | **Conditionally supported** | Node 0; N2--N4; N8 reduced/full comparisons, floating-point agreement | “Numerically consistent on the tested states and traces.” | “Reduced QP is faster” or universally equivalent |
| Independent endpoint/swept checker is necessary for acceptance | **Supported by diagnostic evidence** | N2: 44/46 stops per filtered method were nonlinear endpoint mismatches; N3 finite-step optimism; N8 no accepted violations with checker | “The checker is the final nonlinear acceptance authority.” | “The linear filter provides a formal safety guarantee.” |
| Governor can reduce task rate or stop near local infeasibility | **Conditionally supported** | Node 0 three governor states; N7/N8 endpoint-aware recovery; N8 0 accepted online violations | “On tested scenes and step sizes, the governor recovers progress or stops safely under the frozen protocol.” | “Always feasible,” “never stops,” or global safety guarantee |
| Curvature margin is part of the core method | **Not established** | N4 margin sweep: lower optimism but completion/latency trade-off; N8 fixed 0.50-mm schedule only in tested range | “An empirical finite-step design parameter; independent nonlinear checking remains necessary.” | “Universal curvature margin” or curvature theorem |
| Pointwise/DLS branch jump contrast exists and is prevented by the lift | **Negative result / unsupported** | N1: 24 cases, 0 strict candidates; raw 0.0075-rad near miss rejected by 0.04-rad rule | “N1 did not provide a valid comparative branch-jump case.” | “Prevents branch jumps” or “H+Nr is superior to DLS/Pointwise.” |
| Descriptor labels identify global IK branches | **Not proved** | N1 atlas coverage was insufficient for dual-arm stable multi-component contrast | “Descriptor is an independent local diagnostic under the frozen gate.” | “Descriptor is a global branch certificate.” |
| Finite-step clearance error decreases with smaller $dt$ | **Descriptive/conditionally supported** | N3 53,200 records; slope 2.087; N8 replay/held-out slopes about 1.92 (lift) and 2.01 (governor) | “Observed finite-step sensitivity over the tested range.” | “Universally second-order” or a model-independent bound |
| Endpoint-aware backtracking improves completion | **Conditionally supported** | N7: 49/90 vs 42/90 at zero margin; 90/90 at 0.25/0.50 mm; N8: 100 recovered incomplete paired trials, 0 regressions | “Improved completion on the frozen replay and held-out protocols.” | “Improves completion for all tasks.” |
| Online continuation dominates LUT execution | **Negative/conditional result** | N6/N7/N8: LUT about 10.4--10.7 ms per step; online governor about 67--68 ms median; rebuilt LUT completion higher in N6/N7 | “Offers a table-free, auditable alternative with a safety/progress/latency trade-off.” | “Lower latency,” “non-inferior to rebuilt LUT,” or universal adaptation advantage |
| Online continuation avoids table reconstruction | **Conditionally supported** | N6 rebuilt-table runs required about 51.75--52.23 s and 186 MiB in the recorded environment; online path does not rebuild | “Avoids reconstruction in the tested implementation.” | “Always cheaper” or hardware-independent cost superiority |
| N8 held-out finite-step qualification transfers the result | **Conditionally supported** | 2160 rows: exact N7 replay plus seed-5200 held-out, three $dt$, four methods; governor 90/90, 90/90, 89/90 | “Reproduced on the held-out manifest over the tested operating range.” | “Generalizes to arbitrary tasks or environments.” |
| Physical dual-arm execution is validated | **Unverified** | No physical trials in N1--N8 | “Physical deployment is future work.” | “Deployment-ready,” “industrial validation completed,” or force-regulation claims |

## Frozen final thesis sentence

> In a tested regular locally connected component, the analytic differential lift and three-variable linear QP generate an online candidate continuation, while independent nonlinear endpoint/swept validation and endpoint-aware backtracking preserve accepted clearance and maintain task progress over a finite tested operating range; the evidence does not establish global IK branch preservation, universal safety, runtime superiority, or physical deployment.

## Evidence files

- N1: `outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json`
- N2: `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json`
- N3: `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_dt_sensitivity_summary.json`
- N4: `outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_curvature_margin_summary.json`
- N6: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_qualification_summary_final_v2.json`
- N7: `outputs/differential_lift_feasibility_20261006/n7_formal_v1/n7_qualification_summary_final.json`
- N8: `outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_dt_summary.json`

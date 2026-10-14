#!/usr/bin/env python3
"""Summarize N4 linear clearance-margin ablations."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


D_SAFE = 0.005
TOL = 1e-9
MARGIN_BINS = ("<0mm", "[0,0.05)mm", "[0.05,0.10)mm", "[0.10,0.50)mm", ">=0.50mm")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input-dir", required=True)
    p.add_argument("--output-dir", required=True)
    return p.parse_args()


def margin_bin(v):
    if v < 0:
        return "<0mm"
    if v < 0.00005:
        return "[0,0.05)mm"
    if v < 0.00010:
        return "[0.05,0.10)mm"
    if v < 0.00050:
        return "[0.10,0.50)mm"
    return ">=0.50mm"


def stat(vals):
    vals = [float(v) for v in vals if v is not None and np.isfinite(v)]
    if not vals:
        return {"n": 0, "median": None, "p05": None, "p95": None, "min": None, "max": None}
    a = np.asarray(vals)
    return {
        "n": int(a.size), "median": float(np.median(a)),
        "p05": float(np.quantile(a, 0.05)),
        "p95": float(np.quantile(a, 0.95)),
        "min": float(np.min(a)), "max": float(np.max(a)),
    }


def classify(row):
    required = (
        "d_linear_m", "d_actual_m", "delta_d_m", "d_endpoint_m", "d_swept_m",
        "beta", "task_progress_before", "sigma_min_Aq", "closure_residual_inf",
        "joint_limit_margin_rad", "qp_feasible", "endpoint_pass", "swept_pass",
    )
    missing = [k for k in required if row.get(k) is None]
    if missing:
        # A QP infeasibility has no candidate endpoint by definition. Keep it
        # distinct from missing instrumentation; all fields are still audited.
        if row.get("qp_feasible") is False:
            return "qp_infeasibility_no_candidate", missing
        return "instrumentation_unknown", missing
    margin = float(row.get("clearance_margin_m", 0.0))
    required_clearance = D_SAFE + margin
    optimism = (
        float(row["d_linear_m"]) >= required_clearance - TOL
        and float(row["d_actual_m"]) < D_SAFE - TOL
    )
    joint_limit = (
        not bool(row.get("accepted", False))
        and float(row["joint_limit_margin_rad"]) < -TOL
    )
    swept_only = (
        bool(row.get("endpoint_pass", False))
        and not bool(row.get("swept_pass", False))
    )
    endpoint_fail = not bool(row.get("endpoint_pass", False))
    conservative = (
        not bool(row.get("accepted", False))
        and float(row["d_actual_m"]) >= D_SAFE - TOL
    )
    row["optimism_after_margin"] = bool(optimism)
    row["conservative_clearance_rejection"] = bool(conservative and not joint_limit)
    row["joint_limit_rejection"] = bool(joint_limit)
    row["swept_clearance_failure"] = bool(swept_only)
    row["nonlinear_endpoint_mismatch"] = bool(endpoint_fail and not optimism)
    if joint_limit:
        primary = "joint_limit_rejection"
    elif optimism:
        primary = "optimism_after_margin"
    elif swept_only:
        primary = "swept_clearance_failure"
    elif endpoint_fail:
        primary = "nonlinear_endpoint_mismatch"
    elif conservative:
        primary = "conservative_clearance_rejection"
    else:
        primary = "regular_finite_step_sample"
    return primary, []


def group(rows):
    if not rows:
        return {"n": 0}
    return {
        "n": len(rows),
        "complete_step_rate": float(np.mean([bool(r.get("accepted", False)) for r in rows])),
        "endpoint_failure_rate": float(np.mean([not bool(r.get("endpoint_pass", False)) for r in rows])),
        "swept_failure_rate": float(np.mean([not bool(r.get("swept_pass", False)) for r in rows])),
        "optimism_rate": float(np.mean([bool(r.get("optimism_after_margin", False)) for r in rows])),
        "conservative_clearance_rejection_rate": float(np.mean([bool(r.get("conservative_clearance_rejection", False)) for r in rows])),
        "joint_limit_rejection_rate": float(np.mean([bool(r.get("joint_limit_rejection", False)) for r in rows])),
        "delta_d_m": stat([r.get("delta_d_m") for r in rows]),
        "beta": stat([r.get("beta") for r in rows]),
        "task_progress_before": stat([r.get("task_progress_before") for r in rows]),
        "sigma_min_Aq": stat([r.get("sigma_min_Aq") for r in rows]),
        "closure_residual_inf": stat([r.get("closure_residual_inf") for r in rows]),
        "joint_limit_margin_rad": stat([r.get("joint_limit_margin_rad") for r in rows]),
        "latency_ms": stat([r.get("step_latency_ms") for r in rows]),
    }


def main():
    args = parse_args()
    in_dir = Path(args.input_dir).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(in_dir.glob("margin_*_dt_*.json"))
    if not files:
        raise RuntimeError(f"no N4 reports under {in_dir}")
    rows = []
    unknown = []
    source_reports = []
    trial_summaries = []
    for path in files:
        report = json.loads(path.read_text())
        source_reports.append(str(path))
        margin_m = float(report.get("clearance_margin_m", 0.0))
        dt = float(report["dt"])
        for tr in report["trial_rows"]:
            if tr["method"] not in ("lift_filtered", "full_joint_qp"):
                continue
            trace = tr.get("step_trace") or []
            trial_summaries.append({
                "margin_m": margin_m, "margin_mm": 1000.0 * margin_m,
                "dt": dt, "scene": report["scene_name"],
                "speed": report["task_velocity"], "method": tr["method"],
                "trial": tr["trial"], "complete": tr["complete"],
                "steps": tr["steps"],
                "task_progress_fraction": tr["task_progress_fraction"],
                "mean_step_latency_ms": tr["mean_step_latency_ms"],
                "stop_reason": tr["stop_reason"],
            })
            for step in trace:
                item = {
                    "source_report": str(path), "margin_m": margin_m,
                    "margin_mm": 1000.0 * margin_m, "dt": dt,
                    "scene": report["scene_name"], "speed": report["task_velocity"],
                    "method": tr["method"], "trial": tr["trial"], **step,
                }
                item["predicted_margin_m"] = (
                    None if item.get("d_linear_m") is None
                    else float(item["d_linear_m"] - D_SAFE)
                )
                item["predicted_excess_after_margin_m"] = (
                    None if item.get("d_linear_m") is None
                    else float(item["d_linear_m"] - D_SAFE - margin_m)
                )
                classification, missing = classify(item)
                item["n4_classification"] = classification
                if missing:
                    item["missing_fields"] = missing
                    if classification == "instrumentation_unknown":
                        unknown.append(item)
                rows.append(item)
    if not rows:
        raise RuntimeError("no N4 step traces")

    margins = sorted({r["margin_m"] for r in rows})
    dts = sorted({r["dt"] for r in rows})
    methods = ("lift_filtered", "full_joint_qp")
    classification_names = (
        "optimism_after_margin", "conservative_clearance_rejection",
        "joint_limit_rejection", "swept_clearance_failure",
        "nonlinear_endpoint_mismatch", "regular_finite_step_sample",
        "qp_infeasibility_no_candidate", "instrumentation_unknown",
    )
    by_margin_method = {}
    counts = {}
    for margin in margins:
        key = str(margin)
        by_margin_method[key] = {}
        counts[key] = {}
        for method in methods:
            subset = [r for r in rows if r["margin_m"] == margin and r["method"] == method]
            by_margin_method[key][method] = group(subset)
            counts[key][method] = dict(Counter(r["n4_classification"] for r in subset))

    by_margin_dt_method = {}
    for margin in margins:
        by_margin_dt_method[str(margin)] = {}
        for dt in dts:
            by_margin_dt_method[str(margin)][str(dt)] = {}
            for method in methods:
                subset = [r for r in rows if r["margin_m"] == margin and r["dt"] == dt and r["method"] == method]
                by_margin_dt_method[str(margin)][str(dt)][method] = group(subset)

    reduced_full = []
    for margin in margins:
        for dt in dts:
            a = [r for r in rows if r["margin_m"] == margin and r["dt"] == dt and r["method"] == "lift_filtered"]
            b = [r for r in rows if r["margin_m"] == margin and r["dt"] == dt and r["method"] == "full_joint_qp"]
            reduced_full.append({
                "margin_mm": 1000.0 * margin, "dt": dt,
                "n_reduced": len(a), "n_full": len(b),
                "median_delta_difference_m": (
                    float(np.median([r["delta_d_m"] for r in a]) - np.median([r["delta_d_m"] for r in b]))
                    if a and b and all(r.get("delta_d_m") is not None for r in a + b) else None
                ),
                "optimism_rate_difference": (
                    float(np.mean([r["optimism_after_margin"] for r in a]) - np.mean([r["optimism_after_margin"] for r in b]))
                    if a and b and all("optimism_after_margin" in r for r in a + b) else None
                ),
                "stop_count_difference": (
                    int(sum(not r["accepted"] for r in a) - sum(not r["accepted"] for r in b))
                    if a and b else None
                ),
            })

    summary = {
        "kind": "n4_curvature_margin_analysis",
        "source_reports": source_reports,
        "d_safe_m": D_SAFE,
        "margin_values_m": margins,
        "margin_values_mm": [1000.0 * x for x in margins],
        "dt_values": dts,
        "total_step_rows": len(rows),
        "trial_rows": len(trial_summaries),
        "instrumentation_unknown": len(unknown),
        "by_margin_method": by_margin_method,
        "by_margin_dt_method": by_margin_dt_method,
        "classification_counts": counts,
        "reduced_full_comparison": reduced_full,
        "interpretation": {
            "margin_scope": "m_curv is applied only to the linear candidate constraint d_linear >= d_safe + m_curv",
            "nonlinear_scope": "endpoint and swept validation keep d_actual >= d_safe",
            "optimism_after_margin": "d_linear >= d_safe + m_curv and d_actual < d_safe",
            "no_branch_claim": True,
        },
        "gates": {
            "instrumentation_unknown_zero": len(unknown) == 0,
            "all_candidate_fields_present": all(
                r.get("d_linear_m") is not None and r.get("d_actual_m") is not None and r.get("delta_d_m") is not None
                for r in rows if r["n4_classification"] != "qp_infeasibility_no_candidate"
            ),
            "reduced_full_present": all(x["n_reduced"] > 0 and x["n_full"] > 0 for x in reduced_full),
        },
    }
    (out_dir / "n4_curvature_margin_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (out_dir / "n4_step_trace_raw.json").write_text(json.dumps({
        "kind": "n4_step_trace_raw", "source_reports": source_reports,
        "rows": rows,
    }, indent=2) + "\n")
    (out_dir / "n4_trial_summary.json").write_text(json.dumps(trial_summaries, indent=2) + "\n")

    fields = [
        "margin_mm", "dt", "scene", "speed", "method", "trial", "step",
        "accepted", "stop_reason", "d0_m", "d_linear_m",
        "linear_required_clearance_m", "predicted_margin_m",
        "predicted_excess_after_margin_m", "d_actual_m", "d_endpoint_m",
        "d_swept_m", "delta_d_m", "beta", "task_progress_before",
        "task_progress_after", "sigma_min_Aq", "closure_residual_inf",
        "joint_limit_margin_rad", "qp_feasible", "endpoint_pass", "swept_pass",
        "n4_classification",
    ]
    with (out_dir / "n4_clearance_margin_steps.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in fields})

    class_fields = ["margin_mm", "dt", "method", "classification", "n", "rate"]
    with (out_dir / "n4_classification_summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=class_fields)
        writer.writeheader()
        for margin in margins:
            for dt in dts:
                for method in methods:
                    subset = [r for r in rows if r["margin_m"] == margin and r["dt"] == dt and r["method"] == method]
                    c = Counter(r["n4_classification"] for r in subset)
                    for name in classification_names:
                        writer.writerow({
                            "margin_mm": 1000.0 * margin, "dt": dt, "method": method,
                            "classification": name, "n": c.get(name, 0),
                            "rate": c.get(name, 0) / len(subset) if subset else None,
                        })

    with (out_dir / "n4_reduced_full_comparison.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(reduced_full[0].keys()))
        writer.writeheader(); writer.writerows(reduced_full)

    # Compact margin summary for plotting and manuscript tables.
    compact = []
    for margin in margins:
        for method in methods:
            subset = [r for r in rows if r["margin_m"] == margin and r["method"] == method]
            trials = [t for t in trial_summaries if t["margin_m"] == margin and t["method"] == method]
            compact.append({
                "margin_mm": 1000.0 * margin, "method": method,
                "step_count": len(subset),
                "optimism_rate": float(np.mean([r["optimism_after_margin"] for r in subset])),
                "conservative_rate": float(np.mean([r["conservative_clearance_rejection"] for r in subset])),
                "endpoint_failure_rate": float(np.mean([not r["endpoint_pass"] for r in subset])),
                "swept_failure_rate": float(np.mean([not r["swept_pass"] for r in subset])),
                "complete_trial_rate": float(np.mean([t["complete"] for t in trials])) if trials else None,
                "median_task_progress": float(np.median([t["task_progress_fraction"] for t in trials])) if trials else None,
                "median_latency_ms": float(np.median([t["mean_step_latency_ms"] for t in trials])) if trials else None,
                "median_delta_d_m": float(np.median([r["delta_d_m"] for r in subset if r.get("delta_d_m") is not None])),
            })
    (out_dir / "n4_margin_summary.csv").write_text(
        "" if not compact else ("margin_mm,method,step_count,optimism_rate,conservative_rate,endpoint_failure_rate,swept_failure_rate,complete_trial_rate,median_task_progress,median_latency_ms,median_delta_d_m\n" + "\n".join(
            ",".join(str(x[k]) for k in ("margin_mm","method","step_count","optimism_rate","conservative_rate","endpoint_failure_rate","swept_failure_rate","complete_trial_rate","median_task_progress","median_latency_ms","median_delta_d_m")) for x in compact
        ) + "\n")
    )

    try:
        import matplotlib.pyplot as plt
        colors = {"lift_filtered": "#1f77b4", "full_joint_qp": "#d62728"}
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
        for method in methods:
            vals = [x for x in compact if x["method"] == method]
            axes[0].plot([x["margin_mm"] for x in vals], [100.0 * x["optimism_rate"] for x in vals], "o-", label=method, color=colors[method])
            axes[1].plot([x["margin_mm"] for x in vals], [x["median_task_progress"] for x in vals], "o-", label=method, color=colors[method])
            axes[2].plot([x["margin_mm"] for x in vals], [100.0 * (1.0 - x["complete_trial_rate"]) for x in vals], "o-", label=method, color=colors[method])
        axes[0].set(xlabel="m_curv (mm)", ylabel="optimism rate (%)")
        axes[1].set(xlabel="m_curv (mm)", ylabel="median task progress")
        axes[2].set(xlabel="m_curv (mm)", ylabel="trial stop rate (%)")
        for ax in axes:
            ax.legend(frameon=False, fontsize=8)
            ax.grid(alpha=0.2)
        fig.savefig(out_dir / "n4_margin_optimism_progress_stop_figures.png", dpi=180)
        plt.close(fig)
    except Exception as exc:  # pragma: no cover
        summary["plot_error"] = repr(exc)
        (out_dir / "n4_curvature_margin_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(json.dumps({
        "reports": len(files), "rows": len(rows), "unknown": len(unknown),
        "summary": str(out_dir / "n4_curvature_margin_summary.json"),
        "plot": str(out_dir / "n4_margin_optimism_progress_stop_figures.png"),
        "classification_counts": counts,
    }, indent=2))


if __name__ == "__main__":
    main()

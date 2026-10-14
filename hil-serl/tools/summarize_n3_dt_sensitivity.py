#!/usr/bin/env python3
"""Summarize N3 finite-step clearance diagnostics and make figures."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


D_SAFE = 0.005
NUMERIC_TOL = 1e-9
REQUIRED = (
    "d0_m", "d_linear_m", "d_actual_m", "d_endpoint_m", "d_swept_m",
    "delta_d_m", "beta", "task_progress_before", "task_progress_after",
    "sigma_min_Aq", "closure_residual_inf", "joint_limit_margin_rad",
    "qp_feasible", "endpoint_pass", "swept_pass",
)
BIN_EDGES = (0.0, 0.00005, 0.00010, 0.00050, float("inf"))
BIN_NAMES = (
    "[0,0.05)mm", "[0.05,0.10)mm", "[0.10,0.50)mm", ">=0.50mm",
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input-dir", required=True)
    p.add_argument("--output-dir", required=True)
    return p.parse_args()


def margin_bin(margin):
    if margin < 0.0:
        return "<0mm"
    for lo, hi, name in zip(BIN_EDGES[:-1], BIN_EDGES[1:], BIN_NAMES):
        if lo <= margin < hi:
            return name
    return BIN_NAMES[-1]


def stats(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    if not vals:
        return {"n": 0}
    a = np.asarray(vals, dtype=float)
    return {
        "n": int(a.size),
        "median": float(np.median(a)),
        "p05": float(np.quantile(a, 0.05)),
        "p95": float(np.quantile(a, 0.95)),
        "min": float(np.min(a)),
        "max": float(np.max(a)),
    }


def group_summary(rows):
    if not rows:
        return {"n": 0}
    return {
        "n": len(rows),
        "endpoint_failure_rate": float(np.mean([not r["endpoint_pass"] for r in rows])),
        "swept_failure_rate": float(np.mean([not r["swept_pass"] for r in rows])),
        "linearization_optimism_rate": float(np.mean([r["linearization_optimism"] for r in rows])),
        "conservative_rejection_rate": float(np.mean([r["conservative_rejection"] for r in rows])),
        "d_linear_m": stats(rows, "d_linear_m"),
        "d_actual_m": stats(rows, "d_actual_m"),
        "d_endpoint_m": stats(rows, "d_endpoint_m"),
        "d_swept_m": stats(rows, "d_swept_m"),
        "delta_d_m": stats(rows, "delta_d_m"),
        "abs_delta_d_m": stats(rows, "abs_delta_d_m"),
        "beta": stats(rows, "beta"),
        "sigma_min_Aq": stats(rows, "sigma_min_Aq"),
        "closure_residual_inf": stats(rows, "closure_residual_inf"),
        "task_progress_before": stats(rows, "task_progress_before"),
        "task_progress_after": stats(rows, "task_progress_after"),
    }


def main():
    args = parse_args()
    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(input_dir.glob("dt_*.json"))
    if not files:
        raise RuntimeError(f"no dt reports under {input_dir}")

    rows = []
    source_reports = []
    unknown_rows = []
    for path in files:
        report = json.loads(path.read_text())
        source_reports.append(str(path))
        dt = float(report["dt"])
        for trial_row in report["trial_rows"]:
            trace = trial_row.get("step_trace") or []
            for step in trace:
                item = {
                    "source_report": str(path),
                    "dt": dt,
                    "steps_per_trial": report["steps_per_trial"],
                    "scene": report["scene_name"],
                    "speed": report["task_velocity"],
                    "method": trial_row["method"],
                    "trial": trial_row["trial"],
                    **step,
                }
                missing = [k for k in REQUIRED if item.get(k) is None]
                if missing:
                    item["n3_classification"] = "instrumentation_unknown"
                    item["missing_fields"] = missing
                    unknown_rows.append(item)
                    rows.append(item)
                    continue
                margin = float(item["d_linear_m"] - D_SAFE)
                item["predicted_margin_m"] = margin
                item["margin_bin"] = margin_bin(margin)
                item["abs_delta_d_m"] = abs(float(item["delta_d_m"]))
                item["linearization_optimism"] = bool(
                    item["d_linear_m"] >= D_SAFE - NUMERIC_TOL
                    and item["d_actual_m"] < D_SAFE - NUMERIC_TOL
                )
                item["conservative_rejection"] = bool(
                    (not item["accepted"])
                    and item["d_actual_m"] >= D_SAFE - NUMERIC_TOL
                )
                if item["linearization_optimism"]:
                    item["n3_classification"] = "linearization_optimism"
                elif item["conservative_rejection"]:
                    item["n3_classification"] = "conservative_rejection"
                else:
                    item["n3_classification"] = "regular_finite_step_sample"
                rows.append(item)

    if not rows:
        raise RuntimeError("reports contain no step traces")

    by_dt_method_bin = {}
    for dt in sorted({r["dt"] for r in rows}):
        by_dt_method_bin[str(dt)] = {}
        for method in ("lift_filtered", "full_joint_qp"):
            by_dt_method_bin[str(dt)][method] = {}
            for bin_name in ("<0mm",) + BIN_NAMES:
                subset = [
                    r for r in rows
                    if r["dt"] == dt and r["method"] == method
                    and r.get("margin_bin") == bin_name
                ]
                by_dt_method_bin[str(dt)][method][bin_name] = group_summary(subset)

    by_dt_method = {}
    for dt in sorted({r["dt"] for r in rows}):
        by_dt_method[str(dt)] = {}
        for method in ("lift_filtered", "full_joint_qp"):
            by_dt_method[str(dt)][method] = group_summary([
                r for r in rows if r["dt"] == dt and r["method"] == method
            ])

    # Empirical log-log slope of median absolute prediction error versus dt.
    error_order = {}
    for method in ("lift_filtered", "full_joint_qp"):
        points = []
        for dt in sorted({r["dt"] for r in rows}):
            subset = [r for r in rows if r["dt"] == dt and r["method"] == method]
            vals = [r["abs_delta_d_m"] for r in subset if "abs_delta_d_m" in r]
            if vals and np.median(vals) > 0:
                points.append((dt, float(np.median(vals))))
        if len(points) >= 2:
            slope, intercept = np.polyfit(
                np.log([p[0] for p in points]),
                np.log([p[1] for p in points]), 1,
            )
            error_order[method] = {
                "points": [{"dt": d, "median_abs_delta_m": e} for d, e in points],
                "empirical_loglog_slope": float(slope),
                "interpretation": "descriptive finite-step scaling; not a theorem",
            }
        else:
            error_order[method] = {"points": points, "empirical_loglog_slope": None}

    reduced_full = []
    for dt in sorted({r["dt"] for r in rows}):
        a = [r for r in rows if r["dt"] == dt and r["method"] == "lift_filtered"]
        b = [r for r in rows if r["dt"] == dt and r["method"] == "full_joint_qp"]
        reduced_full.append({
            "dt": dt,
            "n_reduced": len(a), "n_full": len(b),
            "median_delta_difference_m": float(np.median([r["delta_d_m"] for r in a]) - np.median([r["delta_d_m"] for r in b])) if a and b else None,
            "p95_abs_delta_difference_m": float(np.quantile([r["abs_delta_d_m"] for r in a], 0.95) - np.quantile([r["abs_delta_d_m"] for r in b], 0.95)) if a and b else None,
            "optimism_rate_difference": float(np.mean([r["linearization_optimism"] for r in a]) - np.mean([r["linearization_optimism"] for r in b])) if a and b else None,
        })

    # Per-margin-bin reduced/full comparison for an auditable boundary table.
    reduced_full_by_bin = []
    for dt in sorted({r["dt"] for r in rows}):
        for bin_name in ("<0mm",) + BIN_NAMES:
            a = [r for r in rows if r["dt"] == dt
                 and r["method"] == "lift_filtered"
                 and r.get("margin_bin") == bin_name]
            b = [r for r in rows if r["dt"] == dt
                 and r["method"] == "full_joint_qp"
                 and r.get("margin_bin") == bin_name]
            item = {"dt": dt, "margin_bin": bin_name,
                    "n_reduced": len(a), "n_full": len(b),
                    "median_delta_difference_m": None,
                    "p95_abs_delta_difference_m": None,
                    "optimism_rate_difference": None,
                    "conservative_rate_difference": None}
            if a and b:
                item.update({
                    "median_delta_difference_m": float(
                        np.median([r["delta_d_m"] for r in a])
                        - np.median([r["delta_d_m"] for r in b])
                    ),
                    "p95_abs_delta_difference_m": float(
                        np.quantile([r["abs_delta_d_m"] for r in a], 0.95)
                        - np.quantile([r["abs_delta_d_m"] for r in b], 0.95)
                    ),
                    "optimism_rate_difference": float(
                        np.mean([r["linearization_optimism"] for r in a])
                        - np.mean([r["linearization_optimism"] for r in b])
                    ),
                    "conservative_rate_difference": float(
                        np.mean([r["conservative_rejection"] for r in a])
                        - np.mean([r["conservative_rejection"] for r in b])
                    ),
                })
            reduced_full_by_bin.append(item)

    classification_counts = {
        method: dict(Counter(
            r.get("n3_classification") for r in rows if r["method"] == method
        ))
        for method in ("lift_filtered", "full_joint_qp")
    }
    classification_summary = []
    for dt in sorted({r["dt"] for r in rows}):
        for method in ("lift_filtered", "full_joint_qp"):
            for classification in (
                "linearization_optimism", "conservative_rejection",
                "regular_finite_step_sample", "instrumentation_unknown",
            ):
                subset = [r for r in rows if r["dt"] == dt
                          and r["method"] == method
                          and r.get("n3_classification") == classification]
                s = group_summary(subset)
                classification_summary.append({
                    "dt": dt, "method": method,
                    "classification": classification,
                    "n": s.get("n", 0),
                    "median_progress_before": s.get("task_progress_before", {}).get("median"),
                    "median_beta": s.get("beta", {}).get("median"),
                    "median_d_actual_m": s.get("d_actual_m", {}).get("median"),
                    "median_delta_d_m": s.get("delta_d_m", {}).get("median"),
                    "p05_delta_d_m": s.get("delta_d_m", {}).get("p05"),
                    "p95_delta_d_m": s.get("delta_d_m", {}).get("p95"),
                    "median_sigma_min_Aq": s.get("sigma_min_Aq", {}).get("median"),
                })
    summary = {
        "kind": "n3_finite_step_clearance_analysis",
        "source_reports": source_reports,
        "frozen_thresholds": {"d_safe_m": D_SAFE, "numeric_tol_m": NUMERIC_TOL},
        "dt_values": sorted({r["dt"] for r in rows}),
        "total_step_rows": len(rows),
        "instrumentation_unknown": len(unknown_rows),
        "classification_counts": classification_counts,
        "classification_summary": classification_summary,
        "by_dt_method": by_dt_method,
        "by_dt_method_margin_bin": by_dt_method_bin,
        "empirical_error_order": error_order,
        "reduced_full_comparison": reduced_full,
        "reduced_full_comparison_by_margin_bin": reduced_full_by_bin,
        "interpretation": {
            "d_linear": "d0 + dt*(grad_q(d)^T qdot + grad_x(d)^T beta*xdot_nom) from the filtered candidate",
            "d_actual": "min(independent endpoint clearance, swept clearance)",
            "linearization_optimism": "d_linear >= d_safe but d_actual < d_safe",
            "conservative_rejection": "candidate rejected while d_actual >= d_safe",
            "regular_finite_step_error": "descriptive delta_d scaling with dt; no invariance claim",
        },
        "gates": {
            "all_diagnostic_rows_instrumented": len(unknown_rows) == 0,
            "reduced_full_available_for_all_dt": all(
                all(x["n_reduced"] > 0 and x["n_full"] > 0 for x in reduced_full)
                for _ in [0]
            ),
        },
    }

    raw = {
        "kind": "n3_step_trace_raw",
        "source_reports": source_reports,
        "frozen_thresholds": summary["frozen_thresholds"],
        "rows": rows,
    }
    raw_path = output_dir / "n3_step_trace_raw.json"
    raw_path.write_text(json.dumps(raw, indent=2) + "\n")
    summary_path = output_dir / "n3_dt_sensitivity_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    fields = [
        "dt", "scene", "speed", "method", "trial", "step", "accepted",
        "stop_reason", "d0_m", "d_linear_m", "d_actual_m", "d_endpoint_m",
        "d_swept_m", "delta_d_m", "predicted_margin_m", "margin_bin", "beta",
        "task_progress_before", "task_progress_after", "sigma_min_Aq",
        "closure_residual_inf", "joint_limit_margin_rad", "qp_feasible",
        "endpoint_pass", "swept_pass", "n3_classification",
    ]
    csv_path = output_dir / "n3_clearance_error_steps.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fields})

    comp_path = output_dir / "n3_reduced_full_comparison.csv"
    with comp_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(reduced_full[0].keys()))
        writer.writeheader()
        writer.writerows(reduced_full)

    comp_bin_path = output_dir / "n3_reduced_full_comparison_by_margin_bin.csv"
    with comp_bin_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(reduced_full_by_bin[0].keys()))
        writer.writeheader()
        writer.writerows(reduced_full_by_bin)

    table_path = output_dir / "n3_clearance_error_statistics.csv"
    with table_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "dt", "method", "margin_bin", "n", "endpoint_failure_rate",
            "swept_failure_rate", "optimism_rate", "conservative_rate",
            "median_delta_m", "p05_delta_m", "p95_delta_m", "min_delta_m",
            "median_beta", "p05_beta", "p95_beta",
            "median_sigma_min_Aq", "p05_sigma_min_Aq", "p95_sigma_min_Aq",
        ])
        for dt in sorted({r["dt"] for r in rows}):
            for method in ("lift_filtered", "full_joint_qp"):
                for bin_name in ("<0mm",) + BIN_NAMES:
                    s = by_dt_method_bin[str(dt)][method][bin_name]
                    if not s.get("n"):
                        writer.writerow([dt, method, bin_name, 0, "", "", "", "", "", "", "", "", "", "", "", "", ""])
                        continue
                    writer.writerow([
                        dt, method, bin_name, s["n"],
                        s["endpoint_failure_rate"], s["swept_failure_rate"],
                        s["linearization_optimism_rate"], s["conservative_rejection_rate"],
                        s["delta_d_m"]["median"], s["delta_d_m"]["p05"],
                        s["delta_d_m"]["p95"], s["delta_d_m"]["min"],
                        s["beta"]["median"], s["beta"]["p05"], s["beta"]["p95"],
                        s["sigma_min_Aq"]["median"], s["sigma_min_Aq"]["p05"],
                        s["sigma_min_Aq"]["p95"],
                    ])

    class_path = output_dir / "n3_classification_summary.csv"
    class_fields = list(classification_summary[0].keys())
    with class_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=class_fields)
        writer.writeheader()
        writer.writerows(classification_summary)
    (output_dir / "n3_classification_summary.json").write_text(
        json.dumps(classification_summary, indent=2) + "\n"
    )

    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        axes = axes.ravel()
        colors = {"lift_filtered": "#1f77b4", "full_joint_qp": "#d62728"}
        plot_rows = rows
        for method in ("lift_filtered", "full_joint_qp"):
            data = [r for r in plot_rows if r["method"] == method]
            axes[0].scatter(
                [r["d_linear_m"] for r in data], [r["d_actual_m"] for r in data],
                s=4, alpha=0.18, color=colors[method], label=method,
            )
            med = []
            for dt in sorted({r["dt"] for r in data}):
                v = [r["delta_d_m"] for r in data if r["dt"] == dt]
                med.append((dt, float(np.median(v))))
            axes[1].plot([x[0] for x in med], [x[1] for x in med], "o-", color=colors[method], label=method)
            axes[2].scatter(
                [r["task_progress_before"] for r in data], [r["delta_d_m"] for r in data],
                s=4, alpha=0.18, color=colors[method], label=method,
            )
        lo = min(r["d_linear_m"] for r in plot_rows)
        hi = max(r["d_linear_m"] for r in plot_rows)
        axes[0].plot([lo, hi], [lo, hi], "k--", linewidth=0.7)
        axes[0].axvline(D_SAFE, color="0.5", linewidth=0.7)
        axes[0].axhline(D_SAFE, color="0.5", linewidth=0.7)
        axes[0].set(xlabel="linear predicted clearance (m)", ylabel="actual min clearance (m)")
        axes[1].set(xscale="log", xlabel="dt (s)", ylabel="median Δd (m)")
        axes[2].axhline(0.0, color="k", linewidth=0.7)
        axes[2].set(xlabel="task progress before step", ylabel="Δd (m)")
        for method in ("lift_filtered", "full_joint_qp"):
            data = [r for r in plot_rows if r["method"] == method]
            axes[3].scatter(
                [1000.0 * r["predicted_margin_m"] for r in data],
                [1e6 * r["delta_d_m"] for r in data],
                s=4, alpha=0.18, color=colors[method], label=method,
            )
        axes[3].axhline(0.0, color="k", linewidth=0.7)
        axes[3].set(
            xlabel="predicted safety margin (mm)",
            ylabel="Δd (µm)",
        )
        for ax in axes:
            ax.legend(frameon=False, fontsize=8)
        fig.savefig(output_dir / "n3_predicted_actual_delta_figures.png", dpi=180)
        plt.close(fig)
    except Exception as exc:  # pragma: no cover
        summary["plot_error"] = repr(exc)
        summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    print(json.dumps({
        "raw": str(raw_path), "summary": str(summary_path),
        "csv": str(csv_path), "plot": str(output_dir / "n3_predicted_actual_delta_figures.png"),
        "rows": len(rows), "unknown": len(unknown_rows),
        "classification_counts": classification_counts,
    }, indent=2))


if __name__ == "__main__":
    main()

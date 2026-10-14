#!/usr/bin/env python3
"""Classify and summarize N2 filtered-stop traces.

The classifier uses only frozen validation thresholds.  It treats a
QP-feasible action whose linear clearance is safe but whose independent
nonlinear endpoint is unsafe as ``nonlinear_endpoint_mismatch``.  A QP
infeasibility is reported as operational evidence of local infeasibility, not
as a global proof.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


D_SAFE = 0.005
BETA_STOP = 0.125
CLOSURE_TOL = 2e-4
NUMERIC_TOL = 1e-9
REQUIRED_TRACE_FIELDS = (
    "task_progress_fraction_before_stop",
    "beta",
    "beta_min_before_stop",
    "beta_time_below_one_before_stop",
    "qp_feasible",
    "linear_predicted_clearance_m",
    "endpoint_clearance_m",
    "swept_clearance_m",
    "joint_limit_margin_rad",
    "closure_residual_inf",
    "sigma_min_Aq",
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--matrix", required=True)
    p.add_argument("--trace-dir", required=True)
    p.add_argument("--output-dir", required=True)
    return p.parse_args()


def classify(trace):
    if trace is None:
        return "complete"
    missing = [k for k in REQUIRED_TRACE_FIELDS if trace.get(k) is None]
    if missing:
        return "instrumentation_unknown"
    beta = float(trace["beta"])
    predicted = float(trace["linear_predicted_clearance_m"])
    endpoint = float(trace["endpoint_clearance_m"])
    swept = float(trace["swept_clearance_m"])
    margin = float(trace["joint_limit_margin_rad"])
    closure = float(trace["closure_residual_inf"])
    qp_feasible = bool(trace["qp_feasible"])

    if margin < -NUMERIC_TOL:
        return "joint_limit_failure"
    if closure > CLOSURE_TOL:
        return "integration_or_residual_failure"
    if endpoint >= D_SAFE - NUMERIC_TOL and swept < D_SAFE - NUMERIC_TOL:
        return "swept_clearance_failure"
    if endpoint < D_SAFE - NUMERIC_TOL:
        if (
            qp_feasible
            and beta >= BETA_STOP - NUMERIC_TOL
            and predicted >= D_SAFE - NUMERIC_TOL
        ):
            return "nonlinear_endpoint_mismatch"
        if (not qp_feasible) and beta >= BETA_STOP - NUMERIC_TOL:
            return "true_local_infeasibility"
    return "instrumentation_unknown"


def summary_stats(rows):
    def vals(key):
        return np.asarray([r[key] for r in rows], dtype=float)

    if not rows:
        return {"n": 0}
    result = {"n": len(rows)}
    for key in (
        "task_progress_fraction_before_stop", "beta", "beta_min_before_stop",
        "beta_time_below_one_before_stop", "linear_predicted_clearance_m",
        "endpoint_clearance_m", "swept_clearance_m",
        "joint_limit_margin_rad", "closure_residual_inf", "sigma_min_Aq",
    ):
        a = vals(key)
        result[key] = {
            "median": float(np.median(a)),
            "p05": float(np.quantile(a, 0.05)),
            "p95": float(np.quantile(a, 0.95)),
            "min": float(np.min(a)),
            "max": float(np.max(a)),
        }
    return result


def main():
    args = parse_args()
    matrix_path = Path(args.matrix).resolve()
    trace_dir = Path(args.trace_dir).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(matrix_path.read_text())

    trace_by_key = {}
    trace_reports = []
    for path in sorted(trace_dir.glob("trace_*.json")):
        report = json.loads(path.read_text())
        trace_reports.append(str(path))
        for row in report["trial_rows"]:
            trace = row.get("stop_trace")
            if trace is not None:
                key = (report["scene_name"], row["method"],
                       report["task_velocity"][0], report["task_velocity"][1],
                       row["trial"])
                trace_by_key[key] = trace

    rows = []
    source_cell_reports = []
    for cell in matrix["cells"]:
        source = Path(cell["report"]).resolve()
        source_cell_reports.append(str(source))
        report = json.loads(source.read_text())
        for row in report["trial_rows"]:
            if row["method"] not in {"lift_filtered", "full_joint_qp"}:
                continue
            key = (report["scene_name"], row["method"],
                   report["task_velocity"][0], report["task_velocity"][1],
                   row["trial"])
            trace = trace_by_key.get(key)
            item = dict(row)
            item["scene"] = cell["scene"]
            item["speed"] = cell["speed"]
            item["source_cell_report"] = str(source)
            item["stop_trace"] = trace
            item["stop_classification"] = classify(trace)
            rows.append(item)

    stops = [r for r in rows if r["stop_classification"] != "complete"]
    counts_by_method = {
        method: dict(Counter(
            r["stop_classification"] for r in stops if r["method"] == method
        ))
        for method in ("lift_filtered", "full_joint_qp")
    }
    categories = [
        "true_local_infeasibility", "nonlinear_endpoint_mismatch",
        "swept_clearance_failure", "joint_limit_failure",
        "integration_or_residual_failure", "instrumentation_unknown",
    ]
    by_category = {
        method: {
            category: summary_stats([
                {
                    **r["stop_trace"],
                    "task_progress_fraction_before_stop": r["stop_trace"]["task_progress_fraction_before_stop"],
                }
                for r in stops
                if r["method"] == method and r["stop_classification"] == category
            ])
            for category in categories
        }
        for method in ("lift_filtered", "full_joint_qp")
    }

    # Pair the two representations by scene, speed and trial to expose any
    # unexplained classification disagreement.
    paired = defaultdict(dict)
    for r in stops:
        speed = (r["scene"], r["speed"], r["trial"])
        paired[speed][r["method"]] = r["stop_classification"]
    disagreements = [
        {"scene": k[0], "speed": k[1], "trial": k[2], **v}
        for k, v in paired.items()
        if v.get("lift_filtered") != v.get("full_joint_qp")
    ]

    raw = {
        "kind": "n2_endpoint_stop_raw",
        "source_matrix": str(matrix_path),
        "source_cell_reports": source_cell_reports,
        "trace_reports": trace_reports,
        "frozen_thresholds": {
            "d_safe_m": D_SAFE,
            "beta_stop": BETA_STOP,
            "closure_tol_inf": CLOSURE_TOL,
            "numeric_tol_m": NUMERIC_TOL,
        },
        "trial_rows": rows,
    }
    raw_path = out_dir / "n2_endpoint_stop_raw.json"
    raw_path.write_text(json.dumps(raw, indent=2) + "\n")

    summary = {
        "kind": "n2_endpoint_stop_classification",
        "source_matrix": str(matrix_path),
        "trace_reports": trace_reports,
        "frozen_thresholds": raw["frozen_thresholds"],
        "v3_counts": {
            "cells": len(matrix["cells"]),
            "trial_rows_total": len(matrix["trial_rows"]),
            "filtered_full_rows": len(rows),
            "stop_rows": len(stops),
        },
        "stop_counts_by_method": counts_by_method,
        "stop_statistics_by_method_and_category": by_category,
        "reduced_full_pair_disagreements": disagreements,
        "gates": {
            "all_stops_traced": all(r["stop_trace"] is not None for r in stops),
            "instrumentation_unknown": sum(
                sum(v.get("instrumentation_unknown", 0) for v in [counts_by_method[m]])
                for m in counts_by_method
            ),
            "reduced_full_same_classification": len(disagreements) == 0,
        },
        "interpretation": {
            "true_local_infeasibility": "Operational QP infeasibility at beta >= beta_stop; not a global feasibility proof.",
            "nonlinear_endpoint_mismatch": "QP and linear clearance prediction pass, but independent nonlinear endpoint fails.",
            "swept_clearance_failure": "Endpoint passes the threshold but an interpolated swept sample fails.",
            "joint_limit_failure": "The executed candidate crosses a joint bound beyond the frozen numeric tolerance.",
            "integration_or_residual_failure": "Closure residual exceeds the frozen N1 tolerance.",
        },
    }
    summary_path = out_dir / "n2_stop_classification_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    csv_path = out_dir / "n2_predicted_vs_actual_clearance.csv"
    fields = [
        "scene", "speed", "method", "trial", "stop_reason",
        "stop_classification", "stop_step",
        "task_progress_fraction_before_stop", "beta",
        "beta_min_before_stop", "beta_time_below_one_before_stop",
        "qp_feasible", "linear_predicted_clearance_m",
        "endpoint_clearance_m", "swept_clearance_m",
        "joint_limit_margin_rad", "closure_residual_inf", "sigma_min_Aq",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in stops:
            t = r["stop_trace"]
            writer.writerow({
                "scene": r["scene"], "speed": r["speed"],
                "method": r["method"], "trial": r["trial"],
                "stop_reason": r["stop_reason"],
                "stop_classification": r["stop_classification"],
                **{key: t.get(key) for key in fields if key not in {
                    "scene", "speed", "method", "trial", "stop_reason",
                    "stop_classification",
                }},
            })

    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(5.8, 4.8), constrained_layout=True)
        colors = {"lift_filtered": "#1f77b4", "full_joint_qp": "#d62728"}
        for method in ("lift_filtered", "full_joint_qp"):
            data = [r["stop_trace"] for r in stops if r["method"] == method]
            ax.scatter(
                [t["linear_predicted_clearance_m"] for t in data],
                [t["endpoint_clearance_m"] for t in data],
                s=26, alpha=0.8, label=method, color=colors[method],
            )
        lo, hi = D_SAFE - 0.0004, max(
            D_SAFE + 0.0004,
            max(t["linear_predicted_clearance_m"] for r in stops for t in [r["stop_trace"]]) + 0.0001,
        )
        ax.plot([lo, hi], [lo, hi], "k--", linewidth=0.8, label="y=x")
        ax.axvline(D_SAFE, color="0.5", linewidth=0.8)
        ax.axhline(D_SAFE, color="0.5", linewidth=0.8)
        ax.set_xlabel("linear predicted clearance (m)")
        ax.set_ylabel("independent endpoint clearance (m)")
        ax.set_title("N2 predicted vs actual clearance at filtered stops")
        ax.legend(frameon=False)
        fig.savefig(out_dir / "n2_predicted_vs_actual_clearance.png", dpi=180)
        plt.close(fig)
    except Exception as exc:  # pragma: no cover - plotting is supplementary
        summary["plot_error"] = repr(exc)
        summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    table_path = out_dir / "n2_stop_statistics.csv"
    with table_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "method", "category", "n", "fraction_of_stops",
            "median_progress", "median_beta", "median_predicted_clearance_m",
            "median_endpoint_clearance", "median_swept_clearance",
            "median_joint_limit_margin", "median_closure_residual",
        ])
        for method in ("lift_filtered", "full_joint_qp"):
            total = len([r for r in stops if r["method"] == method])
            for category in categories:
                s = by_category[method][category]
                if not s.get("n"):
                    writer.writerow([method, category, 0, 0.0, "", "", "", "", "", "", ""])
                    continue
                writer.writerow([
                    method, category, s["n"], s["n"] / total,
                    s["task_progress_fraction_before_stop"]["median"],
                    s["beta"]["median"],
                    s["linear_predicted_clearance_m"]["median"],
                    s["endpoint_clearance_m"]["median"],
                    s["swept_clearance_m"]["median"],
                    s["joint_limit_margin_rad"]["median"],
                    s["closure_residual_inf"]["median"],
                ])

    print(json.dumps({
        "raw": str(raw_path), "summary": str(summary_path),
        "csv": str(csv_path), "plot": str(out_dir / "n2_predicted_vs_actual_clearance.png"),
        "stop_counts": counts_by_method,
        "pair_disagreements": len(disagreements),
    }, indent=2))


if __name__ == "__main__":
    main()

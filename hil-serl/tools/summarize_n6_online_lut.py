#!/usr/bin/env python3
"""Summarize N6-A online versus frozen-LUT paired trials."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

METHODS = ("frozen_lut_safe_scaled", "online_lift_filtered")
METRICS = ("complete", "task_progress_fraction", "steps", "min_clearance_m",
           "collision_samples", "max_closure_error_mixed_m_rad",
           "mean_step_latency_ms", "p95_step_latency_ms")


def quantile(a, q):
    return float(np.quantile(np.asarray(a, dtype=float), q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    d = json.loads(args.input.read_text())
    rows = d["trial_rows"]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cells = []
    for scene in d["scenes"]:
        for speed in d["speeds"]:
            cell = {"scene": scene, "speed": speed}
            paired = {}
            for method in METHODS:
                rs = [r for r in rows if r["scene"] == scene and r["speed"] == speed and r["method"] == method]
                paired[method] = {int(r["trial"]): r for r in rs}
                cell[f"{method}_trials"] = len(rs)
                cell[f"{method}_complete"] = int(sum(r["complete"] for r in rs))
                cell[f"{method}_complete_rate"] = float(np.mean([r["complete"] for r in rs]))
                cell[f"{method}_median_progress"] = float(np.median([r["task_progress_fraction"] for r in rs]))
                cell[f"{method}_median_steps"] = float(np.median([r["steps"] for r in rs]))
                cell[f"{method}_median_latency_ms"] = float(np.median([r["mean_step_latency_ms"] for r in rs]))
                cell[f"{method}_p95_latency_ms"] = float(np.quantile([r["p95_step_latency_ms"] for r in rs], .95))
                cell[f"{method}_collision_samples"] = int(sum(r.get("collision_samples", 0) for r in rs))
                cell[f"{method}_joint_limit_samples"] = int(sum(r.get("joint_limit_samples", 0) for r in rs))
                cell[f"{method}_stop_reasons"] = json.dumps({k: sum(r["stop_reason"] == k for r in rs) for k in sorted({r["stop_reason"] for r in rs})}, sort_keys=True)
            common = sorted(set(paired[METHODS[0]]) & set(paired[METHODS[1]]))
            for metric in METRICS:
                a = [paired["online_lift_filtered"][t][metric] for t in common]
                b = [paired["frozen_lut_safe_scaled"][t][metric] for t in common]
                if metric == "complete":
                    aa, bb = np.asarray(a, dtype=int), np.asarray(b, dtype=int)
                    diff = aa - bb
                    cell[f"delta_{metric}_mean"] = float(np.mean(diff))
                    cell[f"delta_{metric}_median"] = float(np.median(diff))
                    cell[f"online_better_{metric}"] = int(np.sum(aa > bb))
                    cell[f"lut_better_{metric}"] = int(np.sum(bb > aa))
                else:
                    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
                    cell[f"delta_{metric}_mean"] = float(np.mean(diff))
                    cell[f"delta_{metric}_median"] = float(np.median(diff))
                    cell[f"online_better_{metric}"] = int(np.sum(diff > 0))
                    cell[f"lut_better_{metric}"] = int(np.sum(diff < 0))
            cells.append(cell)
    fields = sorted({k for c in cells for k in c})
    with (args.output_dir / "n6_cell_summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(cells)

    overall = {}
    for method in METHODS:
        rs = [r for r in rows if r["method"] == method]
        overall[method] = {
            "trials": len(rs),
            "complete": int(sum(r["complete"] for r in rs)),
            "complete_rate": float(np.mean([r["complete"] for r in rs])),
            "median_progress": float(np.median([r["task_progress_fraction"] for r in rs])),
            "p05_progress": quantile([r["task_progress_fraction"] for r in rs], .05),
            "median_steps": float(np.median([r["steps"] for r in rs])),
            "collision_samples": int(sum(r.get("collision_samples", 0) for r in rs)),
            "joint_limit_samples": int(sum(r.get("joint_limit_samples", 0) for r in rs)),
            "stop_reasons": {k: sum(r["stop_reason"] == k for r in rs) for k in sorted({r["stop_reason"] for r in rs})},
            "median_mean_latency_ms": float(np.median([r["mean_step_latency_ms"] for r in rs])),
            "p95_mean_latency_ms": quantile([r["mean_step_latency_ms"] for r in rs], .95),
        }
    out = {"kind": "n6_online_vs_frozen_lut_summary", "input": str(args.input.resolve()), "overall": overall, "cells": cells,
           "interpretation": [
               "frozen_lut_safe_scaled is a fixed phase replay and is not a rebuilt LUT or the full learned T6 policy.",
               "online_lift_filtered uses the v3 local QP/governor/checker implementation.",
               "The result qualifies an N6-A adaptation hypothesis; it does not yet establish rebuilt-LUT cost savings.",
           ]}
    (args.output_dir / "n6_qualification_summary.json").write_text(json.dumps(out, indent=2) + "\n")

    labels = [f"{c['scene']}\n{c['speed']}" for c in cells]
    x = np.arange(len(cells)); width = .38
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), constrained_layout=True)
    for j, method in enumerate(METHODS):
        vals = [c[f"{method}_complete_rate"] for c in cells]
        axes[0].bar(x + (j - .5) * width, vals, width, label=method)
        vals = [c[f"{method}_median_progress"] for c in cells]
        axes[1].bar(x + (j - .5) * width, vals, width, label=method)
        vals = [c[f"{method}_median_latency_ms"] for c in cells]
        axes[2].bar(x + (j - .5) * width, vals, width, label=method)
    axes[0].set_ylabel("complete rate"); axes[0].set_ylim(0, 1.05)
    axes[1].set_ylabel("median task progress"); axes[1].set_ylim(0, 1.05)
    axes[2].set_ylabel("median mean step latency (ms)")
    for ax in axes:
        ax.set_xticks(x, labels, rotation=45, ha="right")
        ax.grid(axis="y", alpha=.25)
    axes[0].legend(fontsize=8)
    fig.suptitle("N6-A: frozen LUT versus safety-supervised local continuation")
    fig.savefig(args.output_dir / "n6_completion_progress_latency.png", dpi=180)
    print(json.dumps(out["overall"], indent=2))


if __name__ == "__main__":
    main()

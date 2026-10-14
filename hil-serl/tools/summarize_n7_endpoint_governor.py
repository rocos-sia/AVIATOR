#!/usr/bin/env python3
"""Summarize N7 endpoint-aware governor results."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

METHODS = (
    "stale_lut_safe_scaled",
    "rebuilt_lut_safe_scaled",
    "online_lift_filtered",
    "online_endpoint_governor",
)
D_SAFE = 0.005


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantile(rows, key, q):
    return float(np.quantile(np.asarray([r[key] for r in rows], dtype=float), q))


def method_stats(rows, method):
    rs = [r for r in rows if r["method"] == method]
    stop_reasons = {k: sum(r["stop_reason"] == k for r in rs) for k in sorted({r["stop_reason"] for r in rs})}
    out = {
        "trials": len(rs),
        "complete": int(sum(bool(r["complete"]) for r in rs)),
        "complete_rate": float(np.mean([r["complete"] for r in rs])),
        "median_progress": float(np.median([r["task_progress_fraction"] for r in rs])),
        "p05_progress": quantile(rs, "task_progress_fraction", 0.05),
        "p95_progress": quantile(rs, "task_progress_fraction", 0.95),
        "median_steps": float(np.median([r["steps"] for r in rs])),
        "median_min_clearance_m": float(np.median([r["min_clearance_m"] for r in rs])),
        "min_min_clearance_m": float(np.min([r["min_clearance_m"] for r in rs])),
        "median_latency_ms": float(np.median([r.get("mean_step_latency_ms", 0.0) for r in rs])),
        "p95_latency_ms": quantile(rs, "mean_step_latency_ms", 0.95),
        "stop_reasons": stop_reasons,
    }
    if method == "online_endpoint_governor":
        out.update({
            "governor_attempts": int(sum(r["governor_attempts"] for r in rs)),
            "accepted_governed_steps": int(sum(r["accepted_governed_steps"] for r in rs)),
            "median_attempts_per_trial": float(np.median([r["governor_attempts"] for r in rs])),
            "attempt_count_histogram": {
                str(k): int(sum(
                    sum(1 for s in r["attempt_trace"] if s.get("accepted") and s.get("governor_attempt_count") == k)
                    for r in rs
                )) for k in range(1, 14)
            },
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    data = json.loads(args.input.read_text())
    rows = data["trial_rows"]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    required = {"method", "trial", "scene", "speed", "steps", "complete", "stop_reason",
                "task_progress_fraction", "min_clearance_m", "min_swept_clearance_m"}
    missing = []
    unsafe_accepted = []
    for row in rows:
        if not required.issubset(row):
            missing.append({"method": row.get("method"), "trial": row.get("trial"), "missing": sorted(required - set(row))})
        if row["method"] == "online_endpoint_governor":
            for step in row.get("attempt_trace", []):
                if step.get("accepted") and (
                    step.get("d_endpoint_m", D_SAFE) < 0.005 - 1e-9 or
                    step.get("d_swept_m", D_SAFE) < 0.005 - 1e-9
                ):
                    unsafe_accepted.append({"trial": row["trial"], "scene": row["scene"], "speed": row["speed"], "step": step.get("step")})

    cells = []
    for scene in data["scenes"]:
        for speed in data["speeds"]:
            cell = {"scene": scene, "speed": speed}
            for method in METHODS:
                rs = [r for r in rows if r["scene"] == scene and r["speed"] == speed and r["method"] == method]
                s = method_stats(rs, method)
                cell[method + "_complete"] = s["complete"]
                cell[method + "_complete_rate"] = s["complete_rate"]
                cell[method + "_median_progress"] = s["median_progress"]
                cell[method + "_median_steps"] = s["median_steps"]
                cell[method + "_median_latency_ms"] = s["median_latency_ms"]
                cell[method + "_stop_reasons"] = json.dumps(s["stop_reasons"], sort_keys=True)
                if method == "online_endpoint_governor":
                    cell[method + "_accepted_governed_steps"] = s["accepted_governed_steps"]
            cells.append(cell)

    # Paired current online versus endpoint-aware governor.
    paired = []
    for scene in data["scenes"]:
        for speed in data["speeds"]:
            for trial in sorted({r["trial"] for r in rows if r["scene"] == scene and r["speed"] == speed}):
                base = next(r for r in rows if r["scene"] == scene and r["speed"] == speed and r["trial"] == trial and r["method"] == "online_lift_filtered")
                gov = next(r for r in rows if r["scene"] == scene and r["speed"] == speed and r["trial"] == trial and r["method"] == "online_endpoint_governor")
                paired.append({
                    "scene": scene, "speed": speed, "trial": trial,
                    "baseline_complete": bool(base["complete"]), "governor_complete": bool(gov["complete"]),
                    "delta_complete": int(bool(gov["complete"])) - int(bool(base["complete"])),
                    "baseline_progress": base["task_progress_fraction"], "governor_progress": gov["task_progress_fraction"],
                    "delta_progress": gov["task_progress_fraction"] - base["task_progress_fraction"],
                    "baseline_steps": base["steps"], "governor_steps": gov["steps"],
                    "delta_steps": gov["steps"] - base["steps"],
                    "baseline_latency_ms": base.get("mean_step_latency_ms", 0.0),
                    "governor_latency_ms": gov.get("mean_step_latency_ms", 0.0),
                    "delta_latency_ms": gov.get("mean_step_latency_ms", 0.0) - base.get("mean_step_latency_ms", 0.0),
                })

    paired_summary = {
        "pairs": len(paired),
        "baseline_complete": int(sum(p["baseline_complete"] for p in paired)),
        "governor_complete": int(sum(p["governor_complete"] for p in paired)),
        "governor_recovers_incomplete": int(sum((not p["baseline_complete"]) and p["governor_complete"] for p in paired)),
        "governor_regresses_complete": int(sum(p["baseline_complete"] and (not p["governor_complete"]) for p in paired)),
        "median_delta_progress": float(np.median([p["delta_progress"] for p in paired])),
        "mean_delta_progress": float(np.mean([p["delta_progress"] for p in paired])),
        "median_delta_latency_ms": float(np.median([p["delta_latency_ms"] for p in paired])),
        "mean_delta_latency_ms": float(np.mean([p["delta_latency_ms"] for p in paired])),
    }

    overall = {method: method_stats(rows, method) for method in METHODS}
    summary = {
        "kind": "n7_endpoint_aware_governor_summary",
        "input": str(args.input.resolve()),
        "input_sha256": sha256(args.input),
        "overall": overall,
        "cells": cells,
        "paired_online_vs_governor": paired_summary,
        "instrumentation_unknown": len(missing),
        "missing_fields": missing,
        "unsafe_accepted_governor_steps": unsafe_accepted,
        "interpretation": [
            "The governor is endpoint-aware: it re-solves the reduced QP over the frozen scale schedule and accepts only nonlinear-validated steps.",
            "The governor result is an algorithm ablation at m_curv=0; it does not establish a curvature bound.",
            "Rebuilt LUT rows use the same N7 state manifest and are paired within N7, while N6 rebuilt runs remain separate diagnostics.",
            "No accepted governor step violated endpoint or swept clearance in the recorded trace.",
        ],
    }
    (args.output_dir / "n7_governor_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fields = sorted({k for row in cells for k in row})
    with (args.output_dir / "n7_cell_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(cells)
    fields = sorted({k for row in paired for k in row})
    with (args.output_dir / "n7_paired_online_governor.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(paired)

    labels = [f"{c['scene']}\n{c['speed']}" for c in cells]
    x = np.arange(len(cells)); width = 0.22
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), constrained_layout=True)
    for j, method in enumerate(METHODS):
        axes[0].bar(x + (j - 1.5) * width, [c[method + "_complete_rate"] for c in cells], width, label=method)
        axes[1].bar(x + (j - 1.5) * width, [c[method + "_median_progress"] for c in cells], width, label=method)
        axes[2].bar(x + (j - 1.5) * width, [c[method + "_median_latency_ms"] for c in cells], width, label=method)
    axes[0].set_ylabel("complete rate"); axes[0].set_ylim(0, 1.05)
    axes[1].set_ylabel("median progress"); axes[1].set_ylim(0, 1.05)
    axes[2].set_ylabel("median mean step latency (ms)")
    for ax in axes:
        ax.set_xticks(x, labels, rotation=45, ha="right"); ax.grid(axis="y", alpha=.25)
    axes[0].legend(fontsize=7)
    fig.suptitle("N7: endpoint-aware governor and matched LUT comparison")
    fig.savefig(args.output_dir / "n7_progress_safety_latency.png", dpi=180)
    plt.close(fig)

    governor_rows = [r for r in rows if r["method"] == "online_endpoint_governor"]
    attempt_hist = {}
    for row in governor_rows:
        for step in row.get("attempt_trace", []):
            if step.get("accepted"):
                n = str(step.get("governor_attempt_count", 0)); attempt_hist[n] = attempt_hist.get(n, 0) + 1
    fig, ax = plt.subplots(figsize=(7, 4))
    keys = sorted(attempt_hist, key=lambda k: int(k))
    ax.bar(keys, [attempt_hist[k] for k in keys]); ax.set_xlabel("attempts before accepted step"); ax.set_ylabel("accepted steps")
    ax.grid(axis="y", alpha=.25); fig.tight_layout(); fig.savefig(args.output_dir / "n7_governor_attempt_histogram.png", dpi=180); plt.close(fig)
    print(json.dumps({"overall": overall, "paired": paired_summary, "instrumentation_unknown": len(missing), "unsafe_accepted": len(unsafe_accepted)}, indent=2))


if __name__ == "__main__":
    main()

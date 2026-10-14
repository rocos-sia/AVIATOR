#!/usr/bin/env python3
"""Summarize N8 finite-step and held-out validation without changing raw data."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

D_SAFE = 0.005
ONLINE = ("online_lift_filtered", "online_endpoint_governor")
METHODS = ONLINE + ("stale_lut_safe_scaled", "rebuilt_lut_safe_scaled")
MARGIN_BINS = ((0.0, 0.00005), (0.00005, 0.00010), (0.00010, 0.00050), (0.00050, float("inf")))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantile(values, q):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    return float(np.quantile(a, q)) if len(a) else None


def write_csv(path, rows):
    fields = sorted({k for row in rows for k in row}) if rows else ["empty"]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)


def flatten_steps(rows):
    out = []
    for row in rows:
        if row["method"] not in ONLINE:
            continue
        trace = row.get("step_trace") or []
        for s in trace:
            item = dict(s)
            item.update({"manifest": row["manifest"], "method": row["method"],
                         "trial": row["trial"], "scene": row["scene"],
                         "speed": row["speed"], "dt": row["dt"],
                         "steps_target": row["steps_target"]})
            out.append(item)
    return out


def classify(step):
    linear = step.get("d_linear_m")
    actual = step.get("d_actual_m")
    accepted = bool(step.get("accepted", False))
    if linear is None or actual is None:
        return "instrumentation_non_applicable"
    if linear >= D_SAFE and actual < D_SAFE:
        return "linearization_optimism"
    if not accepted and actual >= D_SAFE:
        return "conservative_rejection"
    if not accepted and not bool(step.get("endpoint_pass", True)):
        return "nonlinear_endpoint_mismatch"
    if not accepted and not bool(step.get("swept_pass", True)):
        return "swept_clearance_failure"
    return "regular_finite_step_sample"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    data = json.loads(args.input.read_text())
    rows = data["trial_rows"]
    steps = flatten_steps(rows)

    required_online_trial = {"method", "manifest", "scene", "speed", "dt", "steps_target",
                             "complete", "task_progress_fraction", "min_clearance_m",
                             "min_swept_clearance_m", "stop_reason"}
    unknown = []
    for r in rows:
        if r["method"] in ONLINE:
            missing = sorted(required_online_trial - set(r))
            if missing:
                unknown.append({"scope": "trial", "method": r.get("method"), "trial": r.get("trial"), "missing": missing})
            for s in (r.get("step_trace") or []):
                if s.get("accepted") and any(s.get(k) is None for k in ("d_linear_m", "d_actual_m", "delta_d_m", "beta", "task_progress_after", "sigma_min_Aq", "step_latency_ms")):
                    unknown.append({"scope": "step", "method": r.get("method"), "trial": r.get("trial"), "step": s.get("step"), "missing": [k for k in ("d_linear_m", "d_actual_m", "delta_d_m", "beta", "task_progress_after", "sigma_min_Aq", "step_latency_ms") if s.get(k) is None]})

    for s in steps:
        s["classification"] = classify(s)
        margin = None if s.get("d_linear_m") is None else float(s["d_linear_m"] - D_SAFE)
        s["predicted_margin_m"] = margin
        s["predicted_margin_bin"] = "undefined"
        if margin is not None:
            for i, (lo, hi) in enumerate(MARGIN_BINS):
                if lo <= margin < hi:
                    s["predicted_margin_bin"] = ("[0,0.05)mm", "[0.05,0.10)mm", "[0.10,0.50)mm", ">=0.50mm")[i]
                    break
        s["optimism"] = bool(s["classification"] == "linearization_optimism")
        s["accepted_safety_violation"] = bool(s.get("accepted") and s.get("d_actual_m") is not None and s["d_actual_m"] < D_SAFE - 1e-9)
    write_csv(args.output_dir / "n8_step_trace.csv", steps)

    dt_rows, bin_rows, class_rows = [], [], []
    for manifest in sorted({r["manifest"] for r in rows}):
        for dt in sorted({float(r["dt"]) for r in rows if r["manifest"] == manifest}):
            for method in METHODS:
                rs = [r for r in rows if r["manifest"] == manifest and float(r["dt"]) == dt and r["method"] == method]
                ss = [s for s in steps if s["manifest"] == manifest and float(s["dt"]) == dt and s["method"] == method]
                dt_rows.append({
                    "manifest": manifest, "dt": dt, "method": method, "trials": len(rs),
                    "complete": sum(bool(r["complete"]) for r in rs),
                    "complete_rate": float(np.mean([r["complete"] for r in rs])) if rs else 0.0,
                    "median_progress": float(np.median([r["task_progress_fraction"] for r in rs])) if rs else None,
                    "p05_progress": quantile([r["task_progress_fraction"] for r in rs], .05),
                    "p95_progress": quantile([r["task_progress_fraction"] for r in rs], .95),
                    "median_min_clearance_m": quantile([r["min_clearance_m"] for r in rs], .5),
                    "median_min_swept_m": quantile([r["min_swept_clearance_m"] for r in rs], .5),
                    "median_latency_ms": quantile([r.get("mean_step_latency_ms") for r in rs], .5),
                    "p95_latency_ms": quantile([r.get("mean_step_latency_ms") for r in rs], .95),
                    "safe_stop_count": sum(bool(r.get("stopped")) for r in rs),
                    "governor_recovery_or_attempts": sum(r.get("governor_attempts", 0) for r in rs),
                    "optimism_rate": float(np.mean([s["optimism"] for s in ss])) if ss else None,
                    "median_abs_delta_d_m": quantile([abs(s["delta_d_m"]) for s in ss if s.get("delta_d_m") is not None], .5),
                    "delta_d_p05_m": quantile([s.get("delta_d_m") for s in ss], .05),
                    "delta_d_p95_m": quantile([s.get("delta_d_m") for s in ss], .95),
                    "delta_d_min_m": min([s["delta_d_m"] for s in ss if s.get("delta_d_m") is not None], default=None),
                    "accepted_safety_violations": sum(s["accepted_safety_violation"] for s in ss),
                })
                for label in ("[0,0.05)mm", "[0.05,0.10)mm", "[0.10,0.50)mm", ">=0.50mm"):
                    bs = [s for s in ss if s["predicted_margin_bin"] == label]
                    bin_rows.append({
                        "manifest": manifest, "dt": dt, "method": method, "predicted_margin_bin": label,
                        "samples": len(bs), "optimism_rate": float(np.mean([s["optimism"] for s in bs])) if bs else None,
                        "endpoint_failure_rate": float(np.mean([not bool(s.get("endpoint_pass", True)) for s in bs])) if bs else None,
                        "swept_failure_rate": float(np.mean([not bool(s.get("swept_pass", True)) for s in bs])) if bs else None,
                        "delta_d_median": quantile([s.get("delta_d_m") for s in bs], .5),
                        "delta_d_p05": quantile([s.get("delta_d_m") for s in bs], .05),
                        "delta_d_p95": quantile([s.get("delta_d_m") for s in bs], .95),
                        "delta_d_min": min([s["delta_d_m"] for s in bs if s.get("delta_d_m") is not None], default=None),
                        "median_beta": quantile([s.get("beta") for s in bs], .5),
                        "median_sigma_min_Aq": quantile([s.get("sigma_min_Aq") for s in bs], .5),
                    })
                counts = defaultdict(int)
                for s in ss: counts[s["classification"]] += 1
                for label, count in sorted(counts.items()):
                    class_rows.append({"manifest": manifest, "dt": dt, "method": method, "classification": label, "count": count, "rate": count / max(len(ss), 1)})
    write_csv(args.output_dir / "n8_dt_summary.csv", dt_rows)
    write_csv(args.output_dir / "n8_clearance_error_statistics.csv", bin_rows)
    write_csv(args.output_dir / "n8_classification_summary.csv", class_rows)

    # Paired online baseline versus governor on exactly the same manifest/state/dt.
    keyed = {(r["manifest"], r["scene"], r["speed"], r["trial"], float(r["dt"]), r["method"]): r for r in rows}
    recovery = []
    for key in sorted(k for k in keyed if k[-1] == "online_endpoint_governor"):
        base_key = (*key[:-1], "online_lift_filtered")
        if base_key not in keyed: continue
        b, g = keyed[base_key], keyed[key]
        recovery.append({
            "manifest": key[0], "scene": key[1], "speed": key[2], "trial": key[3], "dt": key[4],
            "baseline_complete": bool(b["complete"]), "governor_complete": bool(g["complete"]),
            "recovered_incomplete": int(not b["complete"] and g["complete"]),
            "regressed_complete": int(b["complete"] and not g["complete"]),
            "baseline_progress": b["task_progress_fraction"], "governor_progress": g["task_progress_fraction"],
            "delta_progress": g["task_progress_fraction"] - b["task_progress_fraction"],
            "governor_attempts": g.get("governor_attempts", 0),
            "baseline_latency_ms": b.get("mean_step_latency_ms", 0.0), "governor_latency_ms": g.get("mean_step_latency_ms", 0.0),
            "delta_latency_ms": g.get("mean_step_latency_ms", 0.0) - b.get("mean_step_latency_ms", 0.0),
        })
    write_csv(args.output_dir / "n8_governor_recovery_summary.csv", recovery)
    progress_rows = []
    for d in dt_rows:
        progress_rows.append({"manifest": d["manifest"], "dt": d["dt"], "method": d["method"], "complete_rate": d["complete_rate"], "median_progress": d["median_progress"], "p05_progress": d["p05_progress"], "median_latency_ms": d["median_latency_ms"], "p95_latency_ms": d["p95_latency_ms"], "safe_stop_count": d["safe_stop_count"], "accepted_safety_violations": d["accepted_safety_violations"]})
    write_csv(args.output_dir / "n8_progress_safety_latency.csv", progress_rows)

    # Finite-step slope per manifest/method using median absolute error.
    slopes = []
    for manifest in sorted({d["manifest"] for d in dt_rows}):
        for method in ONLINE:
            vals = [(d["dt"], d["median_abs_delta_d_m"]) for d in dt_rows if d["manifest"] == manifest and d["method"] == method and d["median_abs_delta_d_m"] and d["median_abs_delta_d_m"] > 0]
            slope = None
            if len(vals) >= 2:
                x = np.log([v[0] for v in vals]); y = np.log([v[1] for v in vals]); slope = float(np.polyfit(x, y, 1)[0])
            slopes.append({"manifest": manifest, "method": method, "loglog_slope_abs_delta": slope})

    summary = {
        "kind": "n8_dt_heldout_summary", "input": str(args.input.resolve()), "input_sha256": sha256(args.input),
        "instrumentation_unknown": len(unknown), "instrumentation_issues": unknown[:100],
        "accepted_safety_violations": int(sum(s["accepted_safety_violation"] for s in steps)),
        "dt_summary": dt_rows, "classification_summary": class_rows,
        "governor_recovery": {
            "pairs": len(recovery), "recovered_incomplete": sum(r["recovered_incomplete"] for r in recovery),
            "regressed_complete": sum(r["regressed_complete"] for r in recovery),
            "median_delta_progress": quantile([r["delta_progress"] for r in recovery], .5),
            "median_delta_latency_ms": quantile([r["delta_latency_ms"] for r in recovery], .5),
        },
        "finite_step_slopes": slopes,
        "interpretation": [
            "The linear QP is a candidate generator; independent nonlinear endpoint/swept checks decide acceptance.",
            "Accepted 100/200/50 steps are reported separately from nominal task progress.",
            "Any stable dt trend is conditional on the tested scenes, speeds, manifests, and margin.",
            "No N8 result is interpreted as global branch preservation or continuous-time safety.",
        ],
    }
    (args.output_dir / "n8_dt_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    # Publication-oriented figures.
    labels = sorted({f"{d['manifest']}\n{d['method']}" for d in dt_rows})
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    for manifest in sorted({d["manifest"] for d in dt_rows}):
        for method in METHODS:
            ds = sorted([d for d in dt_rows if d["manifest"] == manifest and d["method"] == method], key=lambda x: x["dt"])
            if not ds: continue
            lab = f"{manifest}:{method}"
            ax[0].plot([d["dt"] for d in ds], [d["complete_rate"] for d in ds], "o-", label=lab)
            ax[1].plot([d["dt"] for d in ds], [d["median_progress"] for d in ds], "o-", label=lab)
            ax[2].plot([d["dt"] for d in ds], [d["median_latency_ms"] for d in ds], "o-", label=lab)
    ax[0].set_ylabel("complete rate"); ax[1].set_ylabel("median task progress"); ax[2].set_ylabel("median mean step latency (ms)")
    for a in ax: a.set_xlabel("dt (s)"); a.grid(alpha=.25); a.set_xscale("log")
    ax[0].set_ylim(0, 1.05); ax[1].set_ylim(0, 1.05); ax[0].legend(fontsize=6, ncol=2)
    fig.savefig(args.output_dir / "n8_dt_sensitivity.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for method in ONLINE:
        ds = [d for d in dt_rows if d["method"] == method]
        x = np.arange(len(ds))
        ax[0].plot(x, [d["median_progress"] for d in ds], "o-", label=method)
        ax[1].plot(x, [d["median_latency_ms"] for d in ds], "o-", label=method)
    ax[0].set_ylabel("median task progress"); ax[1].set_ylabel("median step latency (ms)")
    ax[0].set_xticks(range(len(dt_rows)), [f"{d['manifest']}\n{d['method']}\ndt={d['dt']}" for d in dt_rows], rotation=75, ha="right", fontsize=6)
    ax[1].set_xticks(range(len(dt_rows)), [f"{d['manifest']}\n{d['method']}\ndt={d['dt']}" for d in dt_rows], rotation=75, ha="right", fontsize=6)
    for a in ax: a.grid(axis="y", alpha=.25); a.legend(fontsize=7)
    fig.savefig(args.output_dir / "n8_progress_safety_latency.png", dpi=180); plt.close(fig)

    attempts = [r.get("governor_attempts", 0) for r in rows if r["method"] == "online_endpoint_governor"]
    fig, ax = plt.subplots(figsize=(7, 4)); ax.hist(attempts, bins=max(1, min(30, len(set(attempts)))), align="left", rwidth=.8); ax.set_xlabel("total governor attempts per trial"); ax.set_ylabel("trials"); ax.grid(axis="y", alpha=.25); fig.tight_layout(); fig.savefig(args.output_dir / "n8_governor_attempt_histogram.png", dpi=180); plt.close(fig)

    print(json.dumps({"instrumentation_unknown": len(unknown), "accepted_safety_violations": int(sum(s["accepted_safety_violation"] for s in steps)), "pairs": len(recovery)}, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Audit and summarize the N17 paired velocity-damper/full-QP run."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


METHOD_ORDER = [
    "online_endpoint_governor",
    "full_joint_qp_governor",
    "velocity_damper",
    "velocity_damper_governed",
]


def stat(values):
    a = np.asarray(list(values), dtype=float)
    if not a.size:
        return {"n": 0, "median": None, "p05": None, "p95": None, "max": None}
    return {"n": int(a.size), "median": float(np.median(a)),
            "p05": float(np.quantile(a, .05)),
            "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--raw", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--status", required=True)
    return p.parse_args()


def main():
    args = parse_args()
    raw_path = Path(args.raw).resolve()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    raw = json.loads(raw_path.read_text())
    rows = raw["trial_rows"]
    methods = [m for m in METHOD_ORDER if m in {r["method"] for r in rows}]
    grouped = defaultdict(dict)
    for r in rows:
        key = (r["family"], float(r["error_level"]), r["speed"], int(r["trial"]))
        grouped[key][r["method"]] = r

    summary = {}
    for method in methods:
        rr = [r for r in rows if r["method"] == method]
        stop = Counter(str(r.get("stop_reason", "instrumentation_unknown")) for r in rr)
        required = [
            "complete", "steps", "task_progress_fraction", "min_clearance_m",
            "min_swept_clearance_m", "max_closure_error_mixed_m_rad",
            "p95_step_latency_ms", "p99_step_latency_ms", "stop_reason",
        ]
        unknown = sum(any(k not in r or r[k] is None for k in required) for r in rr)
        summary[method] = {
            "trials": len(rr), "complete": int(sum(bool(r["complete"]) for r in rr)),
            "completion_rate": float(np.mean([bool(r["complete"]) for r in rr])) if rr else None,
            "task_progress_fraction": stat(r["task_progress_fraction"] for r in rr),
            "steps": stat(r["steps"] for r in rr),
            "min_clearance_m": stat(r["min_clearance_m"] for r in rr),
            "min_swept_clearance_m": stat(r["min_swept_clearance_m"] for r in rr),
            "max_closure_error_mixed_m_rad": stat(r["max_closure_error_mixed_m_rad"] for r in rr),
            "p95_step_latency_ms": stat(r["p95_step_latency_ms"] for r in rr),
            "p99_step_latency_ms": stat(r["p99_step_latency_ms"] for r in rr),
            "governor_attempts": int(sum(r.get("governor_attempts", 0) for r in rr)),
            "accepted_governed_steps": int(sum(r.get("accepted_governed_steps", 0) for r in rr)),
            "stop_reasons": dict(sorted(stop.items())),
            "instrumentation_unknown": int(unknown),
        }

    trial_fields = [
        "family", "error_level", "speed", "trial", "method", "complete", "steps",
        "task_progress_fraction", "min_clearance_m", "min_swept_clearance_m",
        "max_closure_error_mixed_m_rad", "p95_step_latency_ms", "p99_step_latency_ms",
        "governor_attempts", "accepted_governed_steps", "stop_reason", "model_sha256",
    ]
    with (out / "n17_trial_summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=trial_fields); w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in trial_fields})

    pair_fields = ["family", "error_level", "speed", "trial", "method_a", "method_b",
                   "complete_a", "complete_b", "completion_delta", "progress_delta",
                   "p95_latency_delta_ms", "p99_latency_delta_ms", "closure_delta",
                   "min_clearance_delta_m", "stop_a", "stop_b"]
    pair_rows = []
    for key in sorted(grouped):
        g = grouped[key]
        for i, a in enumerate(methods):
            for b in methods[i + 1:]:
                if a not in g or b not in g: continue
                ra, rb = g[a], g[b]
                pair_rows.append({
                    "family": key[0], "error_level": key[1], "speed": key[2], "trial": key[3],
                    "method_a": a, "method_b": b,
                    "complete_a": int(bool(ra["complete"])), "complete_b": int(bool(rb["complete"])),
                    "completion_delta": int(bool(ra["complete"]) - bool(rb["complete"])),
                    "progress_delta": float(ra["task_progress_fraction"] - rb["task_progress_fraction"]),
                    "p95_latency_delta_ms": float(ra["p95_step_latency_ms"] - rb["p95_step_latency_ms"]),
                    "p99_latency_delta_ms": float(ra["p99_step_latency_ms"] - rb["p99_step_latency_ms"]),
                    "closure_delta": float(ra["max_closure_error_mixed_m_rad"] - rb["max_closure_error_mixed_m_rad"]),
                    "min_clearance_delta_m": float(ra["min_clearance_m"] - rb["min_clearance_m"]),
                    "stop_a": ra["stop_reason"], "stop_b": rb["stop_reason"],
                })
    with (out / "n17_pairwise_summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=pair_fields); w.writeheader(); w.writerows(pair_rows)

    stop_rows = []
    for method in methods:
        rr = [r for r in rows if r["method"] == method]
        for reason in sorted({r["stop_reason"] for r in rr}):
            rs = [r for r in rr if r["stop_reason"] == reason]
            stop_rows.append({"method": method, "stop_reason": reason, "count": len(rs),
                              "fraction": len(rs) / len(rr) if rr else None,
                              "median_progress": float(np.median([r["task_progress_fraction"] for r in rs])),
                              "median_clearance_m": float(np.median([r["min_clearance_m"] for r in rs])),
                              "median_closure": float(np.median([r["max_closure_error_mixed_m_rad"] for r in rs]))})
    with (out / "n17_stop_reason_summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(stop_rows[0]) if stop_rows else ["method", "stop_reason"])
        w.writeheader(); w.writerows(stop_rows)

    latency_rows = []
    for method in methods:
        s = summary[method]
        latency_rows.append({"method": method,
                             "p95_median_ms": s["p95_step_latency_ms"]["median"],
                             "p95_p95_ms": s["p95_step_latency_ms"]["p95"],
                             "p99_median_ms": s["p99_step_latency_ms"]["median"],
                             "p99_p95_ms": s["p99_step_latency_ms"]["p95"],
                             "completion_rate": s["completion_rate"]})
    with (out / "n17_latency_summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(latency_rows[0]) if latency_rows else ["method"])
        w.writeheader(); w.writerows(latency_rows)

    reduced, full = summary.get("online_endpoint_governor", {}), summary.get("full_joint_qp_governor", {})
    vd, vdg = summary.get("velocity_damper", {}), summary.get("velocity_damper_governed", {})
    report = {
        "kind": "n17_velocity_damper_full_qp_summary",
        "raw": str(raw_path), "raw_sha256": sha256(raw_path),
        "methods": methods, "rows": len(rows), "paired_groups": len(grouped),
        "summary": summary,
        "instrumentation_unknown": int(sum(v["instrumentation_unknown"] for v in summary.values())),
        "interpretation": {
            "governor_vs_velocity_damper": {
                "completion_difference": None if not vd else vd["completion_rate"] - vdg["completion_rate"],
                "same_as_velocity_damper": bool(vd and vdg and vd["completion_rate"] == vdg["completion_rate"]),
            },
            "reduced_vs_full": {
                "completion_difference": None if not reduced or not full else reduced["completion_rate"] - full["completion_rate"],
                "p95_latency_median_difference_ms": None if not reduced or not full else reduced["p95_step_latency_ms"]["median"] - full["p95_step_latency_ms"]["median"],
            },
        },
    }
    (out / "n17_summary.json").write_text(json.dumps(report, indent=2) + "\n")

    def pct(x): return "n/a" if x is None else f"{100*x:.1f}%"
    lines = [
        "# N17 velocity-damper and full/reduced QP comparison",
        "",
        f"Raw input: `{raw_path}` (SHA-256 `{report['raw_sha256']}`).",
        f"Paired groups: **{len(grouped)}**; rows: **{len(rows)}**; instrumentation_unknown: **{report['instrumentation_unknown']}**.",
        "",
        "## Summary",
        "",
        "| method | completion | median progress | median min clearance (mm) | median max closure | median P95 step (ms) | stop reasons |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for m in methods:
        s = summary[m]
        reasons = ", ".join(f"{k}:{v}" for k, v in s["stop_reasons"].items())
        lines.append(f"| {m} | {pct(s['completion_rate'])} | {s['task_progress_fraction']['median']:.4f} | {1000*s['min_clearance_m']['median']:.3f} | {s['max_closure_error_mixed_m_rad']['median']:.3g} | {s['p95_step_latency_ms']['median']:.1f} | {reasons} |")
    lines += [
        "", "## Interpretation", "",
        "The velocity-damper rows are a classical local rate-damper control with `eta=5 s^-1`; the governed variant adds task-speed backtracking but no phase-scale search.",
        "The full-QP rows use the existing 14-joint equality-constrained QP, while the reduced rows use the frozen `[r_L,r_R,beta]` governor. Every method uses the independent nonlinear endpoint/swept checker.",
        "Solver failure, endpoint rejection, or closure-residual failure is not a branch-jump observation.",
        "",
        "## Files",
        "",
        "- `n17_trial_summary.csv`",
        "- `n17_pairwise_summary.csv`",
        "- `n17_stop_reason_summary.csv`",
        "- `n17_latency_summary.csv`",
        "- `n17_summary.json`",
    ]
    report_path = Path(__file__).resolve().parents[2] / "docs/research/n17_velocity_damper_full_qp_results_2026-10-11.md"
    report_path.write_text("\n".join(lines) + "\n")

    status_path = Path(args.status)
    with status_path.open("a") as f:
        f.write("\n\n## N17 result (velocity damper and full/reduced QP)\n\n")
        f.write(f"- Protocol: `docs/research/n17_velocity_damper_full_qp_protocol_2026-10-11.md`.\n")
        f.write(f"- Raw output: `{out / 'n17_velocity_damper_full_qp_raw.json'}`; summary: `{out / 'n17_summary.json'}`.\n")
        f.write(f"- Paired groups: {len(grouped)}; instrumentation_unknown={report['instrumentation_unknown']}.\n")
        for m in methods:
            s = summary[m]
            f.write(f"- `{m}`: completion {s['complete']}/{s['trials']} ({pct(s['completion_rate'])}), median P95 step {s['p95_step_latency_ms']['median']:.2f} ms, median max closure {s['max_closure_error_mixed_m_rad']['median']:.3g}.\n")
        f.write("- This node does not run rebuilt-LUT, RL, TSR, or GSR. It tests whether a classical velocity damper plus task-speed backtracking reproduces the online governor and whether full joint-space QP matches the reduced representation.\n")
        f.write("- Solver/endpoint failures are not branch-jump evidence. Final paper wording remains conditional on the tested paired states and installation-error envelope.\n")
        f.write("- Next node: audit the paired results and decide whether the innovation claim belongs to the reduced representation, the endpoint-aware governor, or their combination.\n")
    print(json.dumps({"summary": summary, "report": str(report_path)}, indent=2))


if __name__ == "__main__":
    main()

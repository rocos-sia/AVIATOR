#!/usr/bin/env python3
"""Search for a fair branch-switch stress segment.

The search is deliberately conservative.  A candidate is only retained when
the initial continuation atlas has at least two components for each arm, all
reported jump endpoints have a non-ambiguous label margin, and both endpoints
have small closure error.  A solver failure or a large Euler step is therefore
reported as a rejected candidate rather than as evidence of branch switching.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--starts", type=int, default=128)
    p.add_argument("--steps", type=int, default=20)
    p.add_argument("--dt-values", type=float, nargs="+", default=[0.005, 0.01, 0.02])
    p.add_argument("--theta-dots", type=float, nargs="+", default=[-0.8, 0.8, 1.6])
    p.add_argument("--slide-dots", type=float, nargs="+", default=[-0.02, 0.02])
    p.add_argument("--r-scale-values", type=float, nargs="+", default=[0.8, 1.6])
    p.add_argument("--x-values", type=float, nargs=2, action="append", metavar=("THETA", "SLIDE"),
                   help="task state to test; may be repeated")
    p.add_argument("--seed", type=int, default=20261005)
    p.add_argument("--max-workers", type=int, default=4)
    p.add_argument("--label-margin", type=float, default=0.05)
    p.add_argument("--closure-tol", type=float, default=2e-4)
    p.add_argument("--workdir", default=".")
    return p.parse_args()


def jump_rows(rows, label_margin, closure_tol):
    out = []
    for a, b in zip(rows, rows[1:]):
        if a["labels"] == b["labels"]:
            continue
        if None in a["labels"] or None in b["labels"]:
            continue
        if min(a["label_margins"] + b["label_margins"]) < label_margin:
            continue
        if max(a["closure_error_inf"], b["closure_error_inf"]) > closure_tol:
            continue
        out.append({
            "step": a["step"],
            "from_labels": a["labels"],
            "to_labels": b["labels"],
            "margins": a["label_margins"] + b["label_margins"],
            "closure_error_inf": max(a["closure_error_inf"], b["closure_error_inf"]),
        })
    return out


def run_one(case, args, workdir, index):
    theta, slide, dt, theta_dot, slide_dot, r_scale = case
    out = Path(args.output).with_suffix("").parent / f"branch_stress_candidate_{index:04d}.json"
    cmd = [
        sys.executable, "hil-serl/tools/branch_label_probe.py",
        "--manifold", args.manifold, "--model", args.model,
        "--output", str(out), "--starts", str(args.starts),
        "--steps", str(args.steps), "--dt", str(dt),
        "--x-theta", str(theta), "--x-slide", str(slide),
        "--theta-dot", str(theta_dot), "--slide-dot", str(slide_dot),
        "--r-left", str(r_scale), "--r-right", str(-r_scale),
        "--pointwise-seed-mode", "warm_start", "--label-margin", str(args.label_margin),
        "--label-update-interval", "4", "--trajectory-solver-nfev", "40",
    ]
    proc = subprocess.run(cmd, cwd=workdir, text=True, capture_output=True, check=False)
    result = {
        "case": {"x": [theta, slide], "dt": dt, "theta_dot": theta_dot,
                 "slide_dot": slide_dot, "r_scale": r_scale},
        "returncode": proc.returncode,
        "stdout": proc.stdout[-2000:],
        "stderr": proc.stderr[-2000:],
        "probe_output": str(out),
    }
    if proc.returncode != 0 or not out.exists():
        result["status"] = "probe_failed"
        return result
    report = json.loads(out.read_text())
    result["static_failure_codes"] = report["static_failure_codes"]
    result["component_counts"] = [len(s["components"]) for s in report["static_components"]]
    jumps = {}
    for method, trajectory in report["trajectories"].items():
        jumps[method] = jump_rows(trajectory["rows"], args.label_margin, args.closure_tol)
    result["valid_jumps"] = jumps
    atlas_ok = all(n >= 2 for n in result["component_counts"])
    baseline_jumps = jumps["pointwise_ik"] + jumps["DLS"]
    last_baseline_jump = max((j["step"] for j in baseline_jumps), default=-1)
    filtered_rows = report["trajectories"]["lift_filtered"]["rows"]
    filtered_reaches_jump = len(filtered_rows) - 1 > last_baseline_jump
    contrast = (
        atlas_ok and len(baseline_jumps) > 0
        and len(jumps["lift_filtered"]) == 0
        and filtered_reaches_jump
    )
    result["status"] = "candidate" if contrast else ("atlas_insufficient" if not atlas_ok else "no_fair_contrast")
    return result


def main():
    args = parse_args()
    if not args.x_values:
        args.x_values = [[-0.35238328503565686, -0.13805786669420023]]
    cases = [
        (theta, slide, dt, theta_dot, slide_dot, r_scale)
        for theta, slide in args.x_values
        for dt in args.dt_values
        for theta_dot in args.theta_dots
        for slide_dot in args.slide_dots
        for r_scale in args.r_scale_values
    ]
    workdir = str(Path(args.workdir).resolve())
    results = []
    with ThreadPoolExecutor(max_workers=args.max_workers) as pool:
        futures = [pool.submit(run_one, case, args, workdir, i) for i, case in enumerate(cases)]
        for future in as_completed(futures):
            results.append(future.result())
    results.sort(key=lambda r: json.dumps(r["case"], sort_keys=True))
    candidates = [r for r in results if r.get("status") == "candidate"]
    summary = {
        "kind": "branch_stress_search",
        "cases": len(cases),
        "candidates": len(candidates),
        "status_counts": {status: sum(r.get("status") == status for r in results)
                           for status in sorted({r.get("status") for r in results})},
        "criteria": {
            "pointwise_or_DLS_valid_jump": "at least one",
            "lift_filtered_valid_jumps": 0,
            "lift_filtered_reaches_last_baseline_jump": True,
            "both_arm_component_count": ">=2",
            "label_margin": args.label_margin,
            "closure_tolerance_inf": args.closure_tol,
        },
        "candidate_cases": candidates,
        "results": results,
        "interpretation": [
            "A candidate is a hypothesis-generating stress segment, not proof of a global branch theorem.",
            "No candidate means the frozen task/controller family has not yet separated the methods fairly.",
            "Large closure error, ambiguous labels, or a one-component atlas are rejected as invalid evidence.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"cases": len(cases), "candidates": len(candidates),
                      "status_counts": summary["status_counts"]}, indent=2))


if __name__ == "__main__":
    main()

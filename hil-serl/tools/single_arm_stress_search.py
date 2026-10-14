#!/usr/bin/env python3
"""Search single-arm multi-component segments under a frozen validity rule."""

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
    p.add_argument("--x", type=float, nargs=2, action="append", required=True, metavar=("THETA", "S"))
    p.add_argument("--components", type=int, nargs="+", default=[0, 1, 2])
    p.add_argument("--starts", type=int, default=128)
    p.add_argument("--steps", type=int, default=40)
    p.add_argument("--dt-values", type=float, nargs="+", default=[0.01, 0.02, 0.03])
    p.add_argument("--theta-dots", type=float, nargs="+", default=[0.8, 2.0])
    p.add_argument("--slide-dots", type=float, nargs="+", default=[0.02, 0.05])
    p.add_argument("--r-scale-values", type=float, nargs="+", default=[0.8, 1.6])
    p.add_argument("--seed", type=int, default=20261010)
    p.add_argument("--max-workers", type=int, default=4)
    p.add_argument("--right-arm", type=int, default=1)
    p.add_argument("--label-margin", type=float, default=0.05)
    p.add_argument("--closure-tol", type=float, default=2e-4)
    p.add_argument("--workdir", default=".")
    return p.parse_args()


def valid_jumps(rows, arm, margin, closure_tol):
    jumps = []
    for a, b in zip(rows, rows[1:]):
        la, lb = a["labels"][arm], b["labels"][arm]
        if la is None or lb is None or la == lb:
            continue
        if min(a["label_margins"][arm], b["label_margins"][arm]) < margin:
            continue
        closure = max(a["closure_error_inf"], b["closure_error_inf"])
        if closure > closure_tol:
            continue
        jumps.append({"step": a["step"], "from": la, "to": lb, "closure": closure})
    return jumps


def run_case(case, args, workdir, index):
    theta, slide, component, dt, theta_dot, slide_dot, r_scale = case
    out = Path(args.output).with_suffix("").parent / f"single_arm_stress_{index:04d}.json"
    cmd = [
        sys.executable, "hil-serl/tools/branch_label_probe.py",
        "--manifold", args.manifold, "--model", args.model,
        "--output", str(out), "--starts", str(args.starts), "--steps", str(args.steps),
        "--dt", str(dt), "--x-theta", str(theta), "--x-slide", str(slide),
        "--theta-dot", str(theta_dot), "--slide-dot", str(slide_dot),
        "--r-left", str(r_scale), "--r-right", str(-r_scale),
        "--initial-component-right", "-1",
        "--preserve-left-lookup",
        "--pointwise-seed-mode", "warm_start", "--label-margin", str(args.label_margin),
        "--label-update-interval", "4", "--trajectory-solver-nfev", "40",
        "--seed", str(args.seed + index),
    ]
    proc = subprocess.run(cmd, cwd=workdir, text=True, capture_output=True, check=False)
    result = {"case": {"x": [theta, slide], "initial_component": component,
                        "dt": dt, "theta_dot": theta_dot,
                        "slide_dot": slide_dot, "r_scale": r_scale},
              "returncode": proc.returncode, "probe_output": str(out),
              "stdout": proc.stdout[-1000:], "stderr": proc.stderr[-1000:]}
    if proc.returncode != 0 or not out.exists():
        stderr = result.get("stderr", "")
        if "NO_SAFE_COMPONENT" in stderr or "initial component" in stderr:
            result["status"] = "atlas_insufficient"
        else:
            result["status"] = "probe_failed"
        return result
    d = json.loads(out.read_text())
    result["component_counts"] = [len(s["components"]) for s in d["static_components"]]
    result["static_failure_codes"] = d["static_failure_codes"]
    jumps = {}
    for method, traj in d["trajectories"].items():
        jumps[method] = valid_jumps(traj["rows"], args.right_arm, args.label_margin, args.closure_tol)
    result["valid_jumps"] = jumps
    baseline_jumps = jumps["pointwise_ik"] + jumps["DLS"]
    last_baseline_jump = max((j["step"] for j in baseline_jumps), default=-1)
    filtered_rows = d["trajectories"]["lift_filtered"]["rows"]
    filtered_reaches_jump = len(filtered_rows) - 1 > last_baseline_jump
    result["status"] = (
        "candidate"
        if result["component_counts"][args.right_arm] >= 2
        and len(baseline_jumps) > 0
        and len(jumps["lift_filtered"]) == 0
        and filtered_reaches_jump
        else "no_valid_contrast"
    )
    return result


def main():
    args = parse_args()
    cases = [
        (theta, slide, component, dt, theta_dot, slide_dot, r_scale)
        for theta, slide in args.x
        for component in args.components
        for dt in args.dt_values
        for theta_dot in args.theta_dots
        for slide_dot in args.slide_dots
        for r_scale in args.r_scale_values
    ]
    workdir = str(Path(args.workdir).resolve())
    results = []
    with ThreadPoolExecutor(max_workers=args.max_workers) as pool:
        futures = [pool.submit(run_case, c, args, workdir, i) for i, c in enumerate(cases)]
        for f in as_completed(futures):
            results.append(f.result())
    results.sort(key=lambda x: json.dumps(x["case"], sort_keys=True))
    summary = {
        "kind": "single_arm_stress_search",
        "right_arm": args.right_arm,
        "cases": len(cases),
        "candidates": sum(r.get("status") == "candidate" for r in results),
        "status_counts": {s: sum(r.get("status") == s for r in results)
                           for s in sorted({r.get("status") for r in results})},
        "frozen_criteria": {"label_margin": args.label_margin,
                             "closure_tolerance_inf": args.closure_tol,
                             "warm_start": True},
        "results": results,
        "interpretation": [
            "Only a valid jump with non-ambiguous labels and small closure error is counted.",
            "A candidate requires Pointwise or DLS jumps while lift_filtered has no valid jump and reaches the same step.",
            "No candidate is evidence that this frozen task/controller family does not separate the methods fairly.",
        ],
    }
    out = Path(args.output); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"cases": len(cases), "candidates": summary["candidates"],
                      "status_counts": summary["status_counts"]}, indent=2))


if __name__ == "__main__":
    main()

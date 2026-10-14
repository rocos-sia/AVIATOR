#!/usr/bin/env python3
"""Summarize frozen single-arm branch comparisons with strict jump validity."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--output", required=True)
    p.add_argument("--right-arm", type=int, default=1)
    p.add_argument("--closure-tol", type=float, default=2e-4)
    p.add_argument("--label-margin", type=float, default=0.05)
    p.add_argument("inputs", nargs="+")
    return p.parse_args()


def valid_jumps(rows, arm, closure_tol, label_margin):
    jumps = []
    for a, b in zip(rows, rows[1:]):
        la, lb = a["labels"][arm], b["labels"][arm]
        if la == lb or la is None or lb is None:
            continue
        margins = [a["label_margins"][arm], b["label_margins"][arm]]
        closure = max(a["closure_error_inf"], b["closure_error_inf"])
        if min(margins) < label_margin or closure > closure_tol:
            continue
        jumps.append({
            "step": a["step"],
            "from_label": la,
            "to_label": lb,
            "label_margins": margins,
            "closure_error_inf": closure,
            "descriptor_before": a["descriptor"][arm],
            "descriptor_after": b["descriptor"][arm],
        })
    return jumps


def main():
    args = parse_args()
    reports = []
    for path in args.inputs:
        d = json.loads(Path(path).read_text())
        methods = {}
        for method, traj in d["trajectories"].items():
            rows = traj["rows"]
            jumps = valid_jumps(rows, args.right_arm, args.closure_tol, args.label_margin)
            descriptors = [r["descriptor"][args.right_arm] for r in rows]
            closures = [r["closure_error_inf"] for r in rows]
            methods[method] = {
                "raw_branch_jumps": traj["branch_jumps"],
                "valid_branch_jumps": len(jumps),
                "valid_jump_rows": jumps,
                "descriptor_start_rad": descriptors[0],
                "descriptor_end_rad": descriptors[-1],
                "descriptor_range_rad": max(descriptors) - min(descriptors),
                "max_closure_error_inf": max(closures),
                "median_closure_error_inf": sorted(closures)[len(closures) // 2],
                "steps": len(rows) - 1,
            }
        reports.append({
            "input": str(Path(path).resolve()),
            "initial_x": d["initial_x"],
            "starts_per_arm": d["starts_per_arm"],
            "steps": d["steps"],
            "dt": d["dt"],
            "task_velocity": d["task_velocity"],
            "nominal_null_rate": d["nominal_null_rate"],
            "pointwise_seed_mode": d["pointwise_seed_mode"],
            "static_components": d["static_components"],
            "methods": methods,
        })
    summary = {
        "kind": "single_arm_branch_comparison",
        "arm": args.right_arm,
        "frozen_protocol": {
            "closure_tolerance_inf": args.closure_tol,
            "label_margin": args.label_margin,
            "same_warm_start_and_solver_budget": True,
            "descriptor": "independent signed elbow-swivel descriptor",
        },
        "reports": reports,
        "interpretation": [
            "A valid branch jump requires a label change, non-ambiguous margins, and small closure residual at both endpoints.",
            "Raw solver label changes with large closure error are numerical failures, not branch-switch evidence.",
            "This comparison is local to the selected arm and task segment.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"reports": len(reports), "valid_jump_counts": [
        {m: v["valid_branch_jumps"] for m, v in r["methods"].items()} for r in reports
    ]}, indent=2))


if __name__ == "__main__":
    main()

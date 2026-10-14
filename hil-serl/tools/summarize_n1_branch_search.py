#!/usr/bin/env python3
"""Apply the frozen independent-descriptor rule to N1 raw branch probes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input-summary", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--right-arm", type=int, default=1)
    p.add_argument("--label-margin", type=float, default=0.05)
    p.add_argument("--closure-tol", type=float, default=2e-4)
    p.add_argument("--descriptor-gap", type=float, default=0.04)
    return p.parse_args()


def valid_jumps(rows, arm, label_margin, closure_tol, descriptor_gap):
    jumps = []
    for a, b in zip(rows, rows[1:]):
        la, lb = a["labels"][arm], b["labels"][arm]
        if la is None or lb is None or la == lb:
            continue
        margin = min(a["label_margins"][arm], b["label_margins"][arm])
        closure = max(a["closure_error_inf"], b["closure_error_inf"])
        descriptor_delta = abs(b["descriptor"][arm] - a["descriptor"][arm])
        if margin < label_margin or closure > closure_tol:
            continue
        if descriptor_delta < descriptor_gap:
            continue
        jumps.append({
            "step": int(a["step"]),
            "from": int(la),
            "to": int(lb),
            "label_margin": float(margin),
            "closure_error_inf": float(closure),
            "descriptor_delta_rad": float(descriptor_delta),
        })
    return jumps


def main():
    args = parse_args()
    source = json.loads(Path(args.input_summary).read_text())
    results = []
    for entry in source["results"]:
        probe_path = Path(entry["probe_output"])
        row = dict(entry)
        if entry.get("status") == "atlas_insufficient":
            row["strict_status"] = "atlas_insufficient"
            results.append(row)
            continue
        if entry.get("status") == "probe_failed" or not probe_path.exists():
            row["strict_status"] = "probe_failed"
            results.append(row)
            continue
        report = json.loads(probe_path.read_text())
        strict_jumps = {}
        raw_label_changes = {}
        for method, trajectory in report["trajectories"].items():
            rows = trajectory["rows"]
            strict_jumps[method] = valid_jumps(
                rows, args.right_arm, args.label_margin,
                args.closure_tol, args.descriptor_gap,
            )
            raw_label_changes[method] = sum(
                a["labels"] != b["labels"]
                for a, b in zip(rows, rows[1:])
                if None not in a["labels"] and None not in b["labels"]
            )
        row["strict_valid_jumps"] = strict_jumps
        row["raw_label_changes"] = raw_label_changes
        baseline_jumps = strict_jumps["pointwise_ik"] + strict_jumps["DLS"]
        last_baseline_step = max(
            (j["step"] for j in baseline_jumps), default=-1
        )
        filtered_rows = report["trajectories"]["lift_filtered"]["rows"]
        reaches = len(filtered_rows) - 1 > last_baseline_step
        row["filtered_reaches_last_baseline_jump"] = bool(reaches)
        component_count = len(report["static_components"][args.right_arm]["components"])
        row["selected_initial_components"] = report.get("selected_initial_components")
        if component_count < 2:
            row["strict_status"] = "atlas_insufficient"
        elif (
            baseline_jumps
            and not strict_jumps["lift_filtered"]
            and reaches
        ):
            row["strict_status"] = "candidate"
        else:
            row["strict_status"] = "no_valid_contrast"
        results.append(row)

    counts = {}
    for row in results:
        counts[row["strict_status"]] = counts.get(row["strict_status"], 0) + 1
    summary = {
        "kind": "n1_branch_search_strict_summary",
        "input_summary": str(Path(args.input_summary).resolve()),
        "cases": len(results),
        "candidates": counts.get("candidate", 0),
        "status_counts": counts,
        "frozen_criteria": {
            "label_margin": args.label_margin,
            "closure_tolerance_inf": args.closure_tol,
            "independent_descriptor_gap_rad": args.descriptor_gap,
            "warm_start": True,
            "solver_failures_are_not_branch_jumps": True,
            "filtered_stop_before_baseline_jump_is_not_a_candidate": True,
        },
        "results": results,
        "interpretation": [
            "A valid jump requires label change, non-ambiguous margin, small closure residual, and an independent descriptor change.",
            "The filtered lift must reach the baseline jump step; an earlier safety stop is not evidence of branch preservation.",
            "This is a finite-resolution local test and does not claim global IK topology.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({
        "cases": summary["cases"],
        "candidates": summary["candidates"],
        "status_counts": summary["status_counts"],
    }, indent=2))


if __name__ == "__main__":
    main()

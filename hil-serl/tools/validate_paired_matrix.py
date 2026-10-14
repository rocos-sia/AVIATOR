#!/usr/bin/env python3
"""Run the frozen multi-speed, multi-scene paired differential-lift matrix.

The driver creates temporary MuJoCo wall-scene variants, runs the paired
baseline harness for each speed/scene cell, and merges trial-level records so
that the trial remains the statistical unit.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np


BASE_LEFT = np.array([-0.800308395631, 0.269737213437, -0.0180945147262])
BASE_RIGHT = np.array([-0.799691604369, -0.269737213437, -0.0419054852738])


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials-per-cell", type=int, default=5)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=4100)
    p.add_argument("--min-progress-scale", type=float, default=0.125)
    return p.parse_args()


def fmt_pos(p):
    return " ".join(f"{float(v):.15g}" for v in p)


def make_scene_model(base_path: Path, out_path: Path, scene: str):
    text = base_path.read_text()
    left = BASE_LEFT.copy()
    right = BASE_RIGHT.copy()
    if scene == "tight":
        # The frozen manifold only contains about 9 mm nominal clearance;
        # a 1 mm inward shift per wall gives a distinct near-boundary scene
        # while retaining valid initial states at d >= 7 mm.
        left[1] -= 0.001
        right[1] += 0.001
    elif scene == "shifted":
        left[0] += 0.015
        right[0] += 0.015
    elif scene != "nominal":
        raise ValueError(scene)

    def replace(name, position, source):
        pattern = rf'(<body name="{name}" pos=")[^"]+(" )'
        # The XML has no whitespace between the closing quote and >; use a
        # second expression to keep this robust to formatting changes.
        pattern = rf'(<body name="{name}" pos=")[^"]+("[^>]*>)'
        updated, count = re.subn(pattern, rf"\g<1>{fmt_pos(position)}\g<2>", source, count=1)
        if count != 1:
            raise RuntimeError(f"could not locate {name} in {base_path}")
        return updated

    text = replace("aviator_wall_left", left, text)
    text = replace("aviator_wall_right", right, text)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text)


def stat(rows, key):
    a = np.asarray([r[key] for r in rows], dtype=float)
    return {
        "median": float(np.median(a)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(np.max(a)),
    }


def aggregate(rows, method):
    rs = [r for r in rows if r["method"] == method]
    return {
        "trials": len(rs),
        "complete": int(sum(r["complete"] for r in rs)),
        "mean_steps": float(np.mean([r["steps"] for r in rs])),
        "min_swept_clearance_m": stat(rs, "min_swept_clearance_m"),
        "task_progress_fraction": stat(rs, "task_progress_fraction"),
        "max_descriptor_step_rad": stat(rs, "max_descriptor_step_rad"),
        "descriptor_jumps": int(sum(r["descriptor_jumps"] for r in rs)),
        "collision_samples": int(sum(r["collision_samples"] for r in rs)),
        "joint_limit_samples": int(sum(r["joint_limit_samples"] for r in rs)),
        "rejected_steps": int(sum(r["rejected_steps"] for r in rs)),
        "governed_steps": int(sum(r["governed_steps"] for r in rs)),
        "beta_min": stat(rs, "beta_min"),
        "beta_mean": stat(rs, "beta_mean"),
        "beta_time_below_one": stat(rs, "beta_time_below_one"),
        "mean_step_latency_ms": stat(rs, "mean_step_latency_ms"),
        "p95_step_latency_ms": stat(rs, "p95_step_latency_ms"),
        "p99_step_latency_ms": stat(rs, "p99_step_latency_ms"),
        "wall_time_s": stat(rs, "wall_time_s"),
        "stop_reasons": {
            reason: int(sum(r["stop_reason"] == reason for r in rs))
            for reason in sorted({r["stop_reason"] for r in rs})
        },
    }


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    base_model = Path(args.model).resolve()
    # MuJoCo resolves meshdir relative to the XML location, so variants must
    # live in the same directory as the frozen model rather than in a nested
    # output directory.
    scene_dir = base_model.parent
    python = sys.executable
    speeds = {
        "gentle": (0.03, 0.0005),
        "nominal": (0.08, 0.0020),
        "stress": (0.12, 0.0030),
    }
    scenes = ("nominal", "tight", "shifted")
    methods = ["lift_filtered", "full_joint_qp", "lift_unfiltered", "dls", "pointwise_projection"]
    cells = []
    all_rows = []

    for scene_index, scene in enumerate(scenes):
        scene_model = scene_dir / f"aviator_scene_{scene}.xml"
        make_scene_model(base_model, scene_model, scene)
        for speed_index, (speed_name, (theta_dot, slide_dot)) in enumerate(speeds.items()):
            cell_seed = args.seed + scene_index * 1000 + speed_index * 100
            cell_path = output.parent / f"paired_{scene}_{speed_name}.json"
            cmd = [
                python,
                str(root / "tools/validate_paired_baselines.py"),
                "--manifold", str(Path(args.manifold).resolve()),
                "--model", str(scene_model),
                "--output", str(cell_path),
                "--trials", str(args.trials_per_cell),
                "--steps", str(args.steps),
                "--dt", str(args.dt),
                "--seed", str(cell_seed),
                "--theta-dot", str(theta_dot),
                "--slide-dot", str(slide_dot),
                "--r-left", "0.4",
                "--r-right", "-0.4",
                "--governor",
                "--min-progress-scale", str(args.min_progress_scale),
                "--scene-name", scene,
            ]
            env = os.environ.copy()
            env["PYTHONPATH"] = str(root)
            result = subprocess.run(
                cmd, cwd=root, env=env, text=True, capture_output=True
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"cell {scene}/{speed_name} failed:\n{result.stdout}\n{result.stderr}"
                )
            report = json.loads(cell_path.read_text())
            rows = []
            for row in report["trial_rows"]:
                row = dict(row)
                row.update({"scene": scene, "speed": speed_name})
                rows.append(row)
            all_rows.extend(rows)
            cells.append({
                "scene": scene,
                "speed": speed_name,
                "theta_dot": theta_dot,
                "slide_dot": slide_dot,
                "seed": cell_seed,
                "report": str(cell_path),
                "aggregates": report["aggregates"],
            })
            print(f"completed {scene}/{speed_name}: {cell_path}")

    report = {
        "kind": "frozen_paired_differential_lift_matrix",
        "manifold": str(Path(args.manifold).resolve()),
        "base_model": str(base_model),
        "scenes": list(scenes),
        "speeds": {
            name: {"theta_dot": theta, "slide_dot": slide}
            for name, (theta, slide) in speeds.items()
        },
        "trials_per_cell": args.trials_per_cell,
        "total_paired_trials": len(all_rows) // len(methods),
        "steps_per_trial": args.steps,
        "dt": args.dt,
        "seed": args.seed,
        "min_progress_scale": args.min_progress_scale,
        "methods": methods,
        "cells": cells,
        "overall_aggregates": {m: aggregate(all_rows, m) for m in methods},
        "trial_rows": all_rows,
        "decision_gates": {
            "gate_A_protocol": "all cells completed and all trial rows contain progress, latency, descriptor and stop-reason fields",
            "gate_B_safety": "filtered method has zero collision and joint-limit samples",
            "gate_C_theme": "requires safety, nontrivial progress, and independently validated branch labels; descriptor continuity alone is insufficient",
        },
    }
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["overall_aggregates"], indent=2))


if __name__ == "__main__":
    main()

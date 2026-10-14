#!/usr/bin/env python3
"""N16 installation-error robustness experiment.

The experiment compares the frozen endpoint-aware online governor with the
frozen stale safe-scaled LUT under the *same perturbed MuJoCo model* and the
same paired initial states.  Installation errors are represented explicitly
in the model XML; the nominal stale manifold is never rebuilt in this node.

This node is intentionally independent of N6--N8 output files.  It writes a
new raw JSON containing all trial-level records and a compact CSV for review.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n6_online_lut_qualification import frozen_lut_row
from tools.run_n8_dt_heldout import run_endpoint_governor
from tools.validate_differential_lift import AviatorClosure, sample_state


D_SAFE = 0.005
BETA_STOP = 0.125
CURV_MARGIN = 0.0005
SWEPT_SUBDIVISIONS = 5
MIN_INITIAL_CLEARANCE = 0.007
SPEEDS = {"nominal": (0.08, 0.0020), "gentle": (0.03, 0.0005), "stress": (0.12, 0.0030)}

BASE_WALL_LEFT = np.array([-0.800308395631, 0.269737213437, -0.0180945147262])
BASE_WALL_RIGHT = np.array([-0.799691604369, -0.269737213437, -0.0419054852738])
BASE_WALL_ZAXIS = np.array([-0.00114220604093, 0.999026716432, 0.0440943899029])
BASE_LEFT_POS = np.array([-0.91333, 0.008, 0.12156])
BASE_RIGHT_POS = BASE_LEFT_POS.copy()
BASE_LEFT_QUAT = np.array([0.704449953431, -0.704382099293, -0.0616313760028, -0.0616254395358])
BASE_RIGHT_QUAT = np.array([0.704449953431, 0.704382099293, -0.0616313760028, 0.0616254395358])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fmt(v: np.ndarray) -> str:
    return " ".join(f"{float(x):.15g}" for x in np.asarray(v).reshape(-1))


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) * math.cos(angle) + (1.0 - math.cos(angle)) * np.outer(axis, axis) + math.sin(angle) * K


def quat_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.r_[math.cos(angle / 2.0), axis * math.sin(angle / 2.0)]


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def error_levels() -> dict[str, list[float]]:
    # User-selected installation-error families.  Translation is metres and
    # rotation is degrees in the public protocol; XML uses radians only for
    # quaternion construction.
    return {
        "fixture_translation": [0.0, 0.001, 0.002, 0.005],
        "fixture_rotation": [0.0, 0.25, 0.5, 1.0],
        "base_translation": [0.0, 0.0005, 0.001, 0.002],
        "base_rotation": [0.0, 0.1, 0.25, 0.5],
    }


def condition_spec(family: str, level: float) -> dict:
    # A single reproducible world-frame axis is used for each family.  The
    # differential base mode applies +half to L and -half to R.
    if family == "fixture_translation":
        return {"family": family, "level": level, "unit": "m", "axis": [1.0, 0.0, 0.0], "mode": "common_wall_shift"}
    if family == "fixture_rotation":
        return {"family": family, "level": level, "unit": "deg", "axis": [0.0, 0.0, 1.0], "mode": "common_wall_yaw"}
    if family == "base_translation":
        return {"family": family, "level": level, "unit": "m", "axis": [1.0, 0.0, 0.0], "mode": "differential_L_plus_R_minus"}
    if family == "base_rotation":
        return {"family": family, "level": level, "unit": "deg", "axis": [0.0, 0.0, 1.0], "mode": "differential_L_plus_R_minus"}
    raise ValueError(family)


def replace_attr(text: str, name: str, attr: str, value: np.ndarray, base: Path) -> str:
    pattern = rf'(<body name="{re.escape(name)}"[^>]*\s{attr}=\")[^\"]+(\")'
    updated, n = re.subn(pattern, rf"\g<1>{fmt(value)}\g<2>", text, count=1)
    if n != 1:
        raise RuntimeError(f"could not locate body={name} attr={attr} in {base}")
    return updated


def make_variant(base_model: Path, out_model: Path, family: str, level: float) -> dict:
    spec = condition_spec(family, level)
    axis = np.asarray(spec["axis"], dtype=float)
    angle = math.radians(level) if spec["unit"] == "deg" else level
    text = base_model.read_text()
    wall_left, wall_right = BASE_WALL_LEFT.copy(), BASE_WALL_RIGHT.copy()
    wall_zaxis = BASE_WALL_ZAXIS.copy()
    left_pos, right_pos = BASE_LEFT_POS.copy(), BASE_RIGHT_POS.copy()
    left_quat, right_quat = BASE_LEFT_QUAT.copy(), BASE_RIGHT_QUAT.copy()
    if family == "fixture_translation":
        shift = level * axis
        wall_left += shift
        wall_right += shift
    elif family == "fixture_rotation":
        wall_zaxis = rodrigues(axis, angle) @ wall_zaxis
    elif family == "base_translation":
        shift = 0.5 * level * axis
        left_pos += shift
        right_pos -= shift
    elif family == "base_rotation":
        left_quat = quat_mul(quat_axis_angle(axis, 0.5 * angle), left_quat)
        right_quat = quat_mul(quat_axis_angle(axis, -0.5 * angle), right_quat)
    text = replace_attr(text, "aviator_wall_left", "pos", wall_left, base_model)
    text = replace_attr(text, "aviator_wall_right", "pos", wall_right, base_model)
    text = replace_attr(text, "aviator_wall_left", "zaxis", wall_zaxis, base_model) if family == "fixture_rotation" else text
    text = replace_attr(text, "aviator_wall_right", "zaxis", wall_zaxis, base_model) if family == "fixture_rotation" else text
    text = replace_attr(text, "AR5-5_07L-W4C4A2_base", "pos", left_pos, base_model)
    text = replace_attr(text, "AR5-5_07R-W4C4A2_base", "pos", right_pos, base_model)
    text = replace_attr(text, "AR5-5_07L-W4C4A2_base", "quat", left_quat, base_model)
    text = replace_attr(text, "AR5-5_07R-W4C4A2_base", "quat", right_quat, base_model)
    out_model.parent.mkdir(parents=True, exist_ok=True)
    out_model.write_text(text)
    return {**spec, "angle_rad": angle, "model_sha256": sha256(out_model)}


def online_args(dt: float, steps: int, family: str, level: float) -> SimpleNamespace:
    return SimpleNamespace(theta_dot=SPEEDS["nominal"][0], slide_dot=SPEEDS["nominal"][1],
                           r_left=0.4, r_right=-0.4, feedback=2.0, dls_damping=1e-3,
                           dt=dt, steps=steps, clearance_margin_m=CURV_MARGIN,
                           swept_subdivisions=SWEPT_SUBDIVISIONS, governor=True,
                           min_progress_scale=BETA_STOP, trace_stops=True, trace_all_steps=True,
                           scene_name=f"n16_{family}_{level:g}", seed=0)


def sample_paired_states(lk: ManifoldLookup, checker: CollisionChecker, rng: np.random.Generator,
                         count: int, max_attempts: int) -> tuple[list[dict], int]:
    states, attempts = [], 0
    while len(states) < count and attempts < max_attempts:
        attempts += 1
        x0, phi0, lookup = sample_state(lk, rng)
        q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        d0, contact = checker.check(x0, q0)
        if contact or d0 < MIN_INITIAL_CLEARANCE:
            continue
        states.append({"trial": len(states), "x0": x0.tolist(), "q0": q0.tolist(),
                       "phi0": phi0.tolist(), "initial_clearance_m": float(d0)})
    return states, attempts


def summarize(rows: list[dict]) -> dict:
    out = {}
    for method in sorted({r["method"] for r in rows}):
        rs = [r for r in rows if r["method"] == method]
        def stat(key: str) -> dict:
            a = np.asarray([r[key] for r in rs], dtype=float)
            return {"median": float(np.median(a)), "p05": float(np.quantile(a, .05)),
                    "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}
        out[method] = {
            "trials": len(rs), "complete": int(sum(bool(r["complete"]) for r in rs)),
            "complete_rate": float(np.mean([r["complete"] for r in rs])),
            "mean_steps": float(np.mean([r["steps"] for r in rs])),
            "task_progress_fraction": stat("task_progress_fraction"),
            "min_clearance_m": stat("min_clearance_m"),
            "min_swept_clearance_m": stat("min_swept_clearance_m"),
            "max_closure_error_mixed_m_rad": stat("max_closure_error_mixed_m_rad"),
            "p95_step_latency_ms": stat("p95_step_latency_ms"),
            "p99_step_latency_ms": stat("p99_step_latency_ms"),
            "governor_attempts": int(sum(r.get("governor_attempts", 0) for r in rs)),
            "accepted_governed_steps": int(sum(r.get("accepted_governed_steps", 0) for r in rs)),
            "collision_samples": int(sum(r.get("collision_samples", 0) for r in rs)),
            "joint_limit_samples": int(sum(r.get("joint_limit_samples", 0) for r in rs)),
            "stop_reasons": {k: int(sum(r["stop_reason"] == k for r in rs))
                             for k in sorted({r["stop_reason"] for r in rs})},
        }
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials-per-cell", type=int, default=10)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=1611)
    p.add_argument("--speeds", default="nominal")
    p.add_argument("--families", default=",")
    p.add_argument("--levels", default="")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output must be empty/new: {output}")
    output.mkdir(parents=True, exist_ok=True)
    base_model = Path(args.model).resolve()
    manifold = Path(args.manifold).resolve()
    families = [x.strip() for x in args.families.split(",") if x.strip()] if args.families.strip(",") else list(error_levels())
    speeds = [x.strip() for x in args.speeds.split(",") if x.strip()]
    levels_map = error_levels()
    if any(f not in levels_map for f in families) or any(s not in SPEEDS for s in speeds):
        raise ValueError(f"families={families}, speeds={speeds}")
    if args.levels:
        # Optional one-level diagnostic override, still recorded explicitly.
        one = float(args.levels)
        levels_map = {f: [one] for f in families}
    lk = ManifoldLookup(str(manifold))
    lower = lk.joint_lower.reshape(14)
    upper = lk.joint_upper.reshape(14)
    rows, cells, sampling_failures, variants = [], [], [], []
    # The XML references meshes with paths relative to the original model
    # directory.  Keep generated variants beside aviator.xml so MuJoCo sees
    # exactly the same mesh assets; output metadata records their hashes.
    model_dir = base_model.parent
    for family_i, family in enumerate(families):
        for level_i, level in enumerate(levels_map[family]):
            model_path = model_dir / f"aviator_n16_{family}_{level:g}.xml"
            spec = make_variant(base_model, model_path, family, float(level))
            variants.append({**spec, "model": str(model_path.resolve())})
            # Reuse each state set across speeds so speed changes are paired as
            # well.  The nominal-error state is not silently substituted.
            checker = CollisionChecker(str(model_path))
            fk = AviatorClosure(str(model_path))
            seed = int(args.seed + family_i * 10000 + level_i * 100)
            states, attempts = sample_paired_states(lk, checker, np.random.default_rng(seed),
                                                    args.trials_per_cell, args.trials_per_cell * 4000)
            cell_meta = {"family": family, "level": float(level), "seed": seed,
                         "sampling_attempts": attempts, "states": len(states),
                         "target_states": args.trials_per_cell, "model": str(model_path.resolve()),
                         "model_sha256": sha256(model_path), "status": "ok" if len(states) == args.trials_per_cell else "sampling_incomplete"}
            if len(states) != args.trials_per_cell:
                sampling_failures.append(cell_meta)
                cells.append(cell_meta)
                continue
            for speed in speeds:
                theta_dot, slide_dot = SPEEDS[speed]
                cell_rows = []
                for st in states:
                    x0, q0, phi0 = (np.asarray(st[k], dtype=float) for k in ("x0", "q0", "phi0"))
                    # online governor takes the nominal speed from its explicit
                    # arguments; call it directly with this cell's speed.
                    gov = run_endpoint_governor(fk, checker, x0, q0, lower, upper,
                                                args.dt, args.steps, theta_dot, slide_dot)
                    lut_args = SimpleNamespace(dt=args.dt, steps=args.steps)
                    stale = frozen_lut_row(lk, fk, checker, x0, q0, phi0, lut_args,
                                           f"n16_{family}_{level:g}", speed, theta_dot, slide_dot, int(st["trial"]))
                    for method, row in (("online_endpoint_governor", gov), ("stale_lut_safe_scaled", stale)):
                        row.update({"method": method, "family": family, "error_level": float(level),
                                    "error_unit": condition_spec(family, level)["unit"],
                                    "speed": speed, "trial": int(st["trial"]),
                                    "dt": args.dt, "steps_target": args.steps,
                                    "initial_x": st["x0"], "initial_q": st["q0"],
                                    "initial_phi": st["phi0"],
                                    "initial_clearance_m": st["initial_clearance_m"],
                                    "d_safe_m": D_SAFE, "curvature_margin_m": CURV_MARGIN,
                                    "swept_subdivisions": SWEPT_SUBDIVISIONS,
                                    "model_sha256": sha256(model_path),
                                    "paired_state_seed": seed})
                        cell_rows.append(row)
                        rows.append(row)
                cell_meta_speed = {**cell_meta, "speed": speed, "trials": len(states),
                                   "summary": summarize(cell_rows)}
                cells.append(cell_meta_speed)
                (output / f"paired_{family}_{level:g}_{speed}.json").write_text(
                    json.dumps({"cell": cell_meta_speed, "trial_rows": cell_rows}, indent=2) + "\n")
                print(f"completed N16 {family} level={level:g} speed={speed} trials={len(states)}", flush=True)

    report = {
        "kind": "n16_installation_error_robustness_raw",
        "protocol": "N16 installation-error robustness 2026-10-11",
        "manifold": str(manifold), "manifold_sha256": None,
        "base_model": str(base_model), "base_model_sha256": sha256(base_model),
        "output": str(output), "families": families, "levels_magnitude_map": levels_map,
        "speeds": {s: {"theta_dot": SPEEDS[s][0], "slide_dot": SPEEDS[s][1]} for s in speeds},
        "trials_per_cell": args.trials_per_cell, "dt": args.dt, "steps": args.steps,
        "d_safe_m": D_SAFE, "beta_stop": BETA_STOP, "curvature_margin_m": CURV_MARGIN,
        "swept_subdivisions": SWEPT_SUBDIVISIONS, "min_initial_clearance_m": MIN_INITIAL_CLEARANCE,
        "methods": ["online_endpoint_governor", "stale_lut_safe_scaled"],
        "variants": variants, "cells": cells, "sampling_failures": sampling_failures,
        "summary": summarize(rows), "trial_rows": rows,
        "rebuilt_lut": {"status": "not_run", "reason": "N16 primary comparison freezes stale LUT; rebuilt LUT is a separate calibration condition"},
        "interpretation_boundary": [
            "Errors are injected into the true MuJoCo model; the stale LUT is not rebuilt.",
            "The online governor uses the same nonlinear endpoint/swept checker and frozen thresholds.",
            "Completion means all candidate steps accepted; task progress is reported separately.",
            "The result is an empirical robustness envelope, not a universal calibration or safety guarantee.",
            "This node does not test RL generalization or global IK branch preservation.",
        ],
    }
    (output / "n16_installation_error_raw.json").write_text(json.dumps(report, indent=2) + "\n")
    (output / "n16_installation_error_summary.json").write_text(json.dumps({
        "kind": report["kind"], "summary": report["summary"], "cells": cells,
        "sampling_failures": sampling_failures, "variants": variants,
    }, indent=2) + "\n")
    csv_path = output / "n16_trial_summary.csv"
    fields = ["family", "error_level", "error_unit", "speed", "trial", "method", "complete", "steps",
              "task_progress_fraction", "min_clearance_m", "min_swept_clearance_m",
              "max_closure_error_mixed_m_rad", "p95_step_latency_ms", "p99_step_latency_ms", "stop_reason",
              "model_sha256", "paired_state_seed"]
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k) for k in fields})
    print(json.dumps({"summary": report["summary"], "sampling_failures": sampling_failures}, indent=2))


if __name__ == "__main__":
    main()

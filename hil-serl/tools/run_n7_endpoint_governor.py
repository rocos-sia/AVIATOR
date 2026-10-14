#!/usr/bin/env python3
"""N7 endpoint-aware governor and matched LUT comparison.

The runner freezes one paired state manifest and replays the same states through
stale LUT, rebuilt LUT, the current online lift, and a backtracking online lift.
The backtracking method re-solves the reduced QP for a predeclared sequence of
task/phase scales and accepts the first candidate that passes the independent
nonlinear endpoint and swept check.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n6_online_lut_qualification import frozen_lut_row
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_paired_baselines import dls_step, metrics_row
from tools.validate_branch_connector import connector_step_y, swept_clearance
from tools.validate_paired_matrix import make_scene_model

D_SAFE = 0.005
BETA_STOP = 0.125
SPEEDS = {
    "gentle": (0.03, 0.0005),
    "nominal": (0.08, 0.0020),
    "stress": (0.12, 0.0030),
}
SCENES = ("nominal", "tight", "shifted")
# Frozen before the pilot.  The first entry is the nominal candidate.
GOVERNOR_SCHEDULE = (
    (1.0, 1.0),
    (0.75, 1.0), (0.5, 1.0), (0.25, 1.0), (0.125, 1.0),
    (1.0, 0.75), (1.0, 0.5), (1.0, 0.25), (1.0, 0.125),
    (0.75, 0.75), (0.5, 0.5), (0.25, 0.25), (0.125, 0.125),
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--stale-manifold", required=True)
    p.add_argument("--rebuilt-root", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials-per-cell", type=int, default=3)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=4100)
    p.add_argument("--clearance-margin-m", type=float, default=0.0)
    p.add_argument("--scenes", default=",".join(SCENES))
    p.add_argument("--speeds", default=",".join(SPEEDS))
    return p.parse_args()


def online_args(args, scene, speed_name, theta_dot, slide_dot):
    return SimpleNamespace(
        theta_dot=theta_dot, slide_dot=slide_dot,
        r_left=0.4, r_right=-0.4,
        feedback=2.0, dls_damping=1e-3,
        dt=args.dt, steps=args.steps,
        clearance_margin_m=args.clearance_margin_m,
        swept_subdivisions=5,
        governor=True,
        min_progress_scale=BETA_STOP,
        trace_stops=False, trace_all_steps=False,
        scene_name=scene, seed=args.seed,
    )


def state_manifest(stale_lk, scene, speed_name, model_path, args, scene_i, speed_i):
    theta_dot, slide_dot = SPEEDS[speed_name]
    seed = int(args.seed + scene_i * 1000 + speed_i * 100)
    rng = np.random.default_rng(seed)
    checker = CollisionChecker(str(model_path))
    states = []
    attempts = 0
    while len(states) < args.trials_per_cell and attempts < args.trials_per_cell * 2000:
        attempts += 1
        x0, phi0, lookup = sample_state(stale_lk, rng)
        q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        d0, contact = checker.check(x0, q0)
        if contact or d0 < 0.007:
            continue
        states.append({
            "trial": len(states), "scene": scene, "speed": speed_name,
            "seed": seed, "x0": x0.tolist(), "q0": q0.tolist(),
            "phi0": phi0.tolist(), "initial_clearance_m": float(d0),
            "theta_dot": theta_dot, "slide_dot": slide_dot,
        })
    if len(states) != args.trials_per_cell:
        raise RuntimeError(f"could not sample {scene}/{speed_name}: {len(states)}/{args.trials_per_cell}")
    return states


def _progress(x, x0, xdot, dt, steps):
    target = np.asarray(xdot, dtype=float) * dt * steps
    delta = np.asarray(x, dtype=float) - np.asarray(x0, dtype=float)
    return float(np.min(np.divide(delta, target, out=np.zeros(2), where=np.abs(target) > 1e-12)))


def run_endpoint_governor(fk, checker, x0, q0, lower, upper, args, theta_dot, slide_dot):
    x = np.asarray(x0, dtype=float).copy()
    q = np.asarray(q0, dtype=float).copy()
    xdot_nom = np.array([theta_dot, slide_dot], dtype=float)
    r_nom = np.array([0.4, -0.4], dtype=float)
    lower = np.asarray(lower, dtype=float).copy()
    upper = np.asarray(upper, dtype=float).copy()
    previous_n = None
    steps_done = 0
    stop_reason = "complete"
    min_clearance = float(checker.check(x, q)[0])
    min_swept = min_clearance
    max_closure = 0.0
    sigma_min = float("inf")
    governor_attempts = 0
    accepted_governed_steps = 0
    step_latencies = []
    trace = []
    stop_trace = None

    for step in range(args.steps):
        t0 = time.perf_counter()
        accepted = None
        attempts = []
        for task_scale, phase_scale in GOVERNOR_SCHEDULE:
            xdot_try = task_scale * xdot_nom
            r_try = phase_scale * r_nom
            qdot, n, info, sigma, beta, r_used = connector_step_y(
                fk, checker, x, q, xdot_try, r_try, previous_n,
                lower, upper, args.dt, beta_stop=BETA_STOP,
                clearance_margin_m=args.clearance_margin_m,
            )
            sigma_min = min(sigma_min, sigma)
            governor_attempts += 1
            attempt = {
                "attempt_index": len(attempts),
                "task_scale": task_scale, "phase_scale": phase_scale,
                "qp_feasible": bool(info.get("qp_feasible", info.get("feasible", False))),
                "candidate_feasible": bool(info.get("feasible", False)),
                "beta": info.get("beta", beta),
                "d0_m": info.get("d0"),
                "d_linear_m": info.get("linear_predicted_clearance_m"),
                "d_endpoint_qp_m": info.get("d_next"),
                "d_swept_qp_m": info.get("swept_min"),
                "reason": info.get("stop_reason", "none"),
            }
            if not info.get("feasible", False):
                attempts.append(attempt)
                continue

            xdot_used = float(beta) * xdot_try
            x_mid = x + 0.5 * args.dt * xdot_used
            q_mid = q + 0.5 * args.dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot_used, np.asarray(r_used), n,
                2.0, 0.0,
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + args.dt * xdot_used
            q_next = q + args.dt * qdot_mid
            d_next, contact_next = checker.check(x_next, q_next)
            swept_min, swept_contact = swept_clearance(
                checker, x, q, x_next, q_next, subdivisions=5
            )
            limit = bool(np.any(q_next < lower - 1e-9) or np.any(q_next > upper + 1e-9))
            closure = float(np.max(np.abs(fk.error(x_next, q_next))))
            accepted_ok = (
                not contact_next and not swept_contact and
                d_next >= D_SAFE - 1e-9 and swept_min >= D_SAFE - 1e-9 and
                not limit
            )
            attempt.update({
                "d_endpoint_m": float(d_next), "d_swept_m": float(swept_min),
                "d_actual_m": float(min(d_next, swept_min)),
                "closure_residual_inf": closure,
                "joint_limit_failure": limit,
                "endpoint_pass": bool(d_next >= D_SAFE - 1e-9 and not contact_next),
                "swept_pass": bool(swept_min >= D_SAFE - 1e-9 and not swept_contact),
                "midpoint_check": True,
                "midpoint_accepted": bool(accepted_ok),
                "reason": "accepted" if accepted_ok else "post_integrator_check",
            })
            attempts.append(attempt)
            if accepted_ok:
                accepted = {
                    "task_scale": task_scale, "phase_scale": phase_scale,
                    "beta": float(beta), "r": np.asarray(r_used).tolist(),
                    "x_next": x_next, "q_next": q_next,
                    "n_next": n_mid, "d_linear": info.get("linear_predicted_clearance_m"),
                    "d0": info.get("d0"), "d_endpoint": float(d_next),
                    "d_swept": float(swept_min), "d_actual": float(min(d_next, swept_min)),
                    "closure": closure, "joint_limit_margin": float(min(np.min(q_next - lower), np.min(upper - q_next))),
                }
                break

        if accepted is None:
            stop_reason = "governor_exhausted"
            stop_trace = {"step": step, "progress_before": _progress(x, x0, xdot_nom, args.dt, args.steps), "attempts": attempts}
            trace.append({"step": step, "accepted": False, "progress": stop_trace["progress_before"], "attempts": attempts, "stop_reason": stop_reason})
            step_latencies.append((time.perf_counter() - t0) * 1e3)
            break

        x, q, previous_n = accepted["x_next"], accepted["q_next"], accepted["n_next"]
        steps_done += 1
        if accepted["task_scale"] < 1.0 or accepted["phase_scale"] < 1.0 or accepted["beta"] < 1.0 - 1e-10:
            accepted_governed_steps += 1
        min_clearance = min(min_clearance, accepted["d_endpoint"])
        min_swept = min(min_swept, accepted["d_swept"])
        max_closure = max(max_closure, accepted["closure"])
        trace.append({
            "step": step, "accepted": True,
            "task_scale": accepted["task_scale"], "phase_scale": accepted["phase_scale"],
            "beta": accepted["beta"], "governor_attempt_count": len(attempts),
            "d0_m": accepted["d0"], "d_linear_m": accepted["d_linear"],
            "d_endpoint_m": accepted["d_endpoint"], "d_swept_m": accepted["d_swept"],
            "d_actual_m": accepted["d_actual"],
            "delta_d_m": None if accepted["d_linear"] is None else accepted["d_actual"] - accepted["d_linear"],
            "task_progress": _progress(x, x0, xdot_nom, args.dt, args.steps),
            "closure_residual_inf": accepted["closure"],
            "joint_limit_margin_rad": accepted["joint_limit_margin"],
            "attempts": attempts,
            "stop_reason": "none",
        })
        step_latencies.append((time.perf_counter() - t0) * 1e3)

    return {
        "method": "online_endpoint_governor",
        "steps": steps_done, "complete": bool(steps_done == args.steps),
        "stopped": bool(steps_done != args.steps), "stop_reason": stop_reason,
        "task_progress_fraction": _progress(x, x0, xdot_nom, args.dt, args.steps),
        "min_clearance_m": float(min_clearance), "min_swept_clearance_m": float(min_swept),
        "max_closure_error_mixed_m_rad": float(max_closure), "sigma_min_Aq": float(sigma_min),
        "governor_attempts": int(governor_attempts),
        "accepted_governed_steps": int(accepted_governed_steps),
        "mean_step_latency_ms": float(np.mean(step_latencies)) if step_latencies else 0.0,
        "p95_step_latency_ms": float(np.quantile(step_latencies, .95)) if step_latencies else 0.0,
        "p99_step_latency_ms": float(np.quantile(step_latencies, .99)) if step_latencies else 0.0,
        "attempt_trace": trace,
        "stop_trace": stop_trace,
    }


def main():
    args = parse_args()
    scenes = tuple(s.strip() for s in args.scenes.split(",") if s.strip())
    speeds = tuple(s.strip() for s in args.speeds.split(",") if s.strip())
    if not set(scenes).issubset(SCENES) or not set(speeds).issubset(SPEEDS):
        raise ValueError("unknown scene or speed")
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"output must be new/empty: {out}")
    out.mkdir(parents=True, exist_ok=True)
    base_model = Path(args.model).resolve()
    stale_lk = ManifoldLookup(str(Path(args.stale_manifold).resolve()))
    rebuilt_root = Path(args.rebuilt_root).resolve()
    rows = []
    manifest_rows = []
    for scene_i, scene in enumerate(scenes):
        scene_model = base_model.parent / f"aviator_n6_scene_{scene}.xml"
        if not scene_model.exists():
            make_scene_model(base_model, scene_model, scene)
        fk = AviatorClosure(str(scene_model))
        checker = CollisionChecker(str(scene_model))
        rebuilt_dir = rebuilt_root / f"n6_rebuilt_lut_{scene}" / "manifold_phi"
        rebuilt_lk = ManifoldLookup(str(rebuilt_dir))
        for speed_i, speed in enumerate(speeds):
            theta_dot, slide_dot = SPEEDS[speed]
            states = state_manifest(stale_lk, scene, speed, scene_model, args, scene_i, speed_i)
            manifest_rows.extend(states)
            for st in states:
                trial = int(st["trial"])
                x0 = np.asarray(st["x0"], dtype=float)
                q0 = np.asarray(st["q0"], dtype=float)
                phi0 = np.asarray(st["phi0"], dtype=float)
                oargs = online_args(args, scene, speed, theta_dot, slide_dot)
                current = metrics_row("lift_filtered", x0, q0, oargs, fk, checker,
                                      np.concatenate([stale_lk.joint_lower[0], stale_lk.joint_lower[1]]),
                                      np.concatenate([stale_lk.joint_upper[0], stale_lk.joint_upper[1]]))
                current["method"] = "online_lift_filtered"
                lower = np.concatenate([stale_lk.joint_lower[0], stale_lk.joint_lower[1]])
                upper = np.concatenate([stale_lk.joint_upper[0], stale_lk.joint_upper[1]])
                governed = run_endpoint_governor(fk, checker, x0, q0, lower, upper, args, theta_dot, slide_dot)
                stale = frozen_lut_row(stale_lk, fk, checker, x0, q0, phi0, args, scene, speed, theta_dot, slide_dot, trial)
                rebuilt = frozen_lut_row(rebuilt_lk, fk, checker, x0, q0, phi0, args, scene, speed, theta_dot, slide_dot, trial)
                for method, row in (("online_lift_filtered", current), ("online_endpoint_governor", governed),
                                    ("stale_lut_safe_scaled", stale), ("rebuilt_lut_safe_scaled", rebuilt)):
                    row.update({"method": method, "trial": trial, "scene": scene, "speed": speed,
                                "initial_x": st["x0"], "initial_q": st["q0"], "initial_phi": st["phi0"],
                                "initial_clearance_m": st["initial_clearance_m"],
                                "clearance_margin_m": args.clearance_margin_m})
                    rows.append(row)
            print(f"completed N7 {scene}/{speed} ({len(states)} paired states, four methods)", flush=True)
    manifest = {
        "kind": "n7_paired_state_manifest",
        "seed": args.seed, "scenes": list(scenes), "speeds": list(speeds),
        "trials_per_cell": args.trials_per_cell, "dt": args.dt, "steps": args.steps,
        "d_safe_m": D_SAFE, "beta_stop": BETA_STOP,
        "governor_schedule": [list(x) for x in GOVERNOR_SCHEDULE],
        "states": manifest_rows,
    }
    (out / "n7_paired_state_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    report = {
        "kind": "n7_endpoint_aware_governor_matched_lut",
        "stale_manifold": str(Path(args.stale_manifold).resolve()),
        "rebuilt_root": str(rebuilt_root), "base_model": str(base_model),
        "base_model_sha256": sha256(base_model),
        "scenes": list(scenes), "speeds": {s: dict(zip(("theta_dot", "slide_dot"), SPEEDS[s])) for s in speeds},
        "trials_per_cell": args.trials_per_cell, "total_states": len(manifest_rows),
        "methods": ["stale_lut_safe_scaled", "rebuilt_lut_safe_scaled", "online_lift_filtered", "online_endpoint_governor"],
        "dt": args.dt, "steps": args.steps, "d_safe_m": D_SAFE, "beta_stop": BETA_STOP,
        "clearance_margin_m": args.clearance_margin_m,
        "governor_schedule": [list(x) for x in GOVERNOR_SCHEDULE],
        "trial_rows": rows,
        "interpretation_boundary": [
            "N7 does not claim global branch preservation",
            "rejected candidates are not accepted collisions",
            "rebuilt and stale LUT rows share each N7 state manifest, but N6 rebuilt runs remain separately sampled",
            "curvature margin is an empirical ablation unless a local bound is independently established",
        ],
    }
    (out / "n7_endpoint_governor_raw.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({m: {"trials": sum(r["method"] == m for r in rows),
                         "complete": sum(bool(r.get("complete")) for r in rows if r["method"] == m),
                         "median_progress": float(np.median([r["task_progress_fraction"] for r in rows if r["method"] == m]))}
                     for m in report["methods"]}, indent=2))


if __name__ == "__main__":
    main()

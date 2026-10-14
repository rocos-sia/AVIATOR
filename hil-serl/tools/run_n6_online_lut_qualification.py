#!/usr/bin/env python3
"""N6-A paired qualification: frozen LUT versus online filtered continuation.

This experiment uses identical initial (x, q, phi) states, task commands, model,
and nonlinear checker for both methods.  It is deliberately separate from the
frozen N1-N4 runners and writes only to a new N6 output directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_paired_baselines import metrics_row
from tools.validate_branch_connector import swept_clearance
from tools.validate_paired_matrix import make_scene_model

D_SAFE = 0.005
BETA_STOP = 0.125

SPEEDS = {
    "gentle": (0.03, 0.0005),
    "nominal": (0.08, 0.0020),
    "stress": (0.12, 0.0030),
}
SCENES = ("nominal", "tight", "shifted")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials-per-cell", type=int, default=10)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=4100)
    p.add_argument("--min-progress-scale", type=float, default=BETA_STOP)
    p.add_argument("--scenes", default=",".join(SCENES))
    p.add_argument("--speeds", default=",".join(SPEEDS))
    return p.parse_args()


def online_args(args, scene, speed_name, theta_dot, slide_dot):
    # Namespace accepted by the already-frozen metrics_row implementation.
    return SimpleNamespace(
        theta_dot=theta_dot, slide_dot=slide_dot,
        r_left=0.4, r_right=-0.4,
        feedback=2.0, dls_damping=1e-3,
        dt=args.dt, steps=args.steps,
        clearance_margin_m=0.0,
        swept_subdivisions=5,
        governor=True,
        min_progress_scale=args.min_progress_scale,
        trace_stops=False, trace_all_steps=False,
        scene_name=scene, seed=args.seed,
    )


def _inside(v, lo, hi, tol=1e-9):
    return bool(lo - tol <= v <= hi + tol)


def phase_scale_to_box(phi, r_nom, dt, safe_lo, safe_hi):
    """Largest alpha in [0,1] keeping phi+alpha*dt*r inside LUT safe box."""
    alpha = 1.0
    for value, rate, lo, hi in zip(phi, r_nom, safe_lo, safe_hi):
        step = dt * rate
        if step > 0:
            alpha = min(alpha, (hi - value) / step)
        elif step < 0:
            alpha = min(alpha, (lo - value) / step)
    return float(np.clip(alpha, 0.0, 1.0))


def frozen_lut_row(lk, fk, checker, x0, q0, phi0, args, scene, speed_name,
                   theta_dot, slide_dot, trial):
    """Replay Q(x,phi) with native one-dimensional safe-phase scaling.

    This is a stronger LUT baseline than an unchecked fixed phase rate: each
    candidate phase increment is scaled toward zero until it remains inside the
    table's interpolated safe box.  No task-rate governor, Jacobian, or QP is
    used.  The actual MuJoCo endpoint/swept checker still decides acceptance.
    """
    x = np.asarray(x0, dtype=np.float64).copy()
    q = np.asarray(q0, dtype=np.float64).copy()
    phi = np.asarray(phi0, dtype=np.float64).copy()
    xdot = np.array([theta_dot, slide_dot], dtype=np.float64)
    r_nom = np.array([0.4, -0.4], dtype=np.float64)
    lower = lk.joint_lower.reshape(14)
    upper = lk.joint_upper.reshape(14)
    steps_done = 0
    stop_reason = "complete"
    min_clearance = float("inf")
    min_swept = float("inf")
    max_error = float(np.max(np.abs(fk.error(x, q))))
    min_limit_margin = float(min(np.min(q - lower), np.min(upper - q)))
    query_latencies, step_latencies = [], []
    domain_exits = collision_samples = joint_limit_samples = 0
    phase_intervened_steps, phase_scales = 0, []
    target_delta = xdot * args.dt * args.steps
    start_clearance, start_contact = checker.check(x, q)
    min_clearance = min(min_clearance, float(start_clearance))
    if start_contact or start_clearance < D_SAFE:
        stop_reason = "invalid_initial_clearance"
    for step in range(args.steps):
        if stop_reason != "complete":
            break
        t0 = time.perf_counter()
        x_next = x + args.dt * xdot
        if not (_inside(x_next[0], lk.th_min, lk.th_max) and
                _inside(x_next[1], lk.s_min, lk.s_max)):
            domain_exits += 1
            stop_reason = "lut_domain_exit"
            step_latencies.append((time.perf_counter() - t0) * 1e3)
            break
        query_t0 = time.perf_counter()
        try:
            box = lk.safe_interval(x_next[None, :])
            safe_lo, safe_hi = box["phi_safe_lo"][0], box["phi_safe_hi"][0]
            alpha = phase_scale_to_box(phi, r_nom, args.dt, safe_lo, safe_hi)
            phi_next = phi + alpha * args.dt * r_nom
            if not np.all([_inside(v, lk.phi_axis[0], lk.phi_axis[-1]) for v in phi_next]):
                raise ValueError("phase outside LUT grid")
            lut = lk.query(x_next[None, :], phi_next[None, :], check_safe=False)
        except (ValueError, FloatingPointError):
            domain_exits += 1
            stop_reason = "lut_query_failure"
            step_latencies.append((time.perf_counter() - t0) * 1e3)
            break
        query_latencies.append((time.perf_counter() - query_t0) * 1e3)
        phase_scales.append(alpha)
        if alpha < 1.0 - 1e-10:
            phase_intervened_steps += 1
        q_next = np.concatenate([lut["qL"][0], lut["qR"][0]])
        endpoint, endpoint_contact = checker.check(x_next, q_next)
        swept, swept_contact = swept_clearance(checker, x, q, x_next, q_next, subdivisions=5)
        limit_margin = float(min(np.min(q_next - lower), np.min(upper - q_next)))
        limit = bool(limit_margin < -1e-9)
        closure = float(np.max(np.abs(fk.error(x_next, q_next))))
        min_clearance = min(min_clearance, float(endpoint))
        min_swept = min(min_swept, float(swept))
        max_error = max(max_error, closure)
        min_limit_margin = min(min_limit_margin, limit_margin)
        collision_samples += int(endpoint_contact or swept_contact or endpoint < D_SAFE or swept < D_SAFE)
        joint_limit_samples += int(limit)
        if endpoint_contact or endpoint < D_SAFE:
            stop_reason = "lut_endpoint_validation"
        elif swept_contact or swept < D_SAFE:
            stop_reason = "lut_swept_clearance_failure"
        elif limit:
            stop_reason = "lut_joint_limit_failure"
        elif not np.isfinite(closure):
            stop_reason = "lut_invalid_closure"
        if stop_reason == "complete":
            x, q, phi = x_next, q_next, phi_next
            steps_done += 1
        step_latencies.append((time.perf_counter() - t0) * 1e3)
    actual_delta = x - x0
    progress = np.divide(actual_delta, target_delta, out=np.zeros(2), where=np.abs(target_delta) > 1e-12)
    lat = np.asarray(step_latencies, dtype=np.float64)
    qlat = np.asarray(query_latencies, dtype=np.float64)
    phase_scales = np.asarray(phase_scales, dtype=np.float64)
    return {
        "method": "frozen_lut_safe_scaled",
        "trial": int(trial), "scene": scene, "speed": speed_name,
        "steps": int(steps_done), "complete": bool(steps_done == args.steps),
        "stopped": bool(steps_done != args.steps), "stop_reason": stop_reason,
        "task_progress_fraction": float(np.min(progress)),
        "min_clearance_m": float(min_clearance), "min_swept_clearance_m": float(min_swept),
        "max_closure_error_mixed_m_rad": float(max_error), "min_joint_limit_margin_rad": float(min_limit_margin),
        "collision_samples": int(collision_samples), "joint_limit_samples": int(joint_limit_samples),
        "lut_safe_interval_exits": 0, "lut_domain_exits": int(domain_exits),
        "phase_rate": r_nom.tolist(),
        "phase_scale_min": float(np.min(phase_scales)) if len(phase_scales) else 0.0,
        "phase_scale_mean": float(np.mean(phase_scales)) if len(phase_scales) else 0.0,
        "phase_intervened_steps": int(phase_intervened_steps),
        "phase_speed_max_s1": float(np.max(np.abs(r_nom[:, None] * phase_scales[None, :]))) if len(phase_scales) else 0.0,
        "beta_min": 1.0, "beta_mean": 1.0, "beta_time_below_one": 0.0, "governed_steps": 0,
        "mean_step_latency_ms": float(np.mean(lat)) if len(lat) else 0.0,
        "p95_step_latency_ms": float(np.quantile(lat, .95)) if len(lat) else 0.0,
        "p99_step_latency_ms": float(np.quantile(lat, .99)) if len(lat) else 0.0,
        "mean_lut_query_latency_ms": float(np.mean(qlat)) if len(qlat) else 0.0,
        "p95_lut_query_latency_ms": float(np.quantile(qlat, .95)) if len(qlat) else 0.0,
        "initial_x": np.asarray(x0).tolist(), "initial_phi": np.asarray(phi0).tolist(),
        "initial_clearance_m": float(start_clearance),
    }

def summarize(rows):
    out = {}
    for method in sorted({r["method"] for r in rows}):
        rs = [r for r in rows if r["method"] == method]
        def stat(k):
            a = np.asarray([r[k] for r in rs], dtype=float)
            return {"median": float(np.median(a)), "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}
        out[method] = {
            "trials": len(rs),
            "complete": int(sum(r["complete"] for r in rs)),
            "complete_rate": float(np.mean([r["complete"] for r in rs])),
            "mean_steps": float(np.mean([r["steps"] for r in rs])),
            "task_progress_fraction": stat("task_progress_fraction"),
            "min_clearance_m": stat("min_clearance_m"),
            "min_swept_clearance_m": stat("min_swept_clearance_m"),
            "max_closure_error_mixed_m_rad": stat("max_closure_error_mixed_m_rad"),
            "collision_samples": int(sum(r["collision_samples"] for r in rs)),
            "joint_limit_samples": int(sum(r["joint_limit_samples"] for r in rs)),
            "lut_safe_interval_exits": int(sum(r.get("lut_safe_interval_exits", 0) for r in rs)),
            "lut_domain_exits": int(sum(r.get("lut_domain_exits", 0) for r in rs)),
            "governed_steps": int(sum(r.get("governed_steps", 0) for r in rs)),
            "beta_min": stat("beta_min"), "beta_mean": stat("beta_mean"),
            "mean_step_latency_ms": stat("mean_step_latency_ms"),
            "p95_step_latency_ms": stat("p95_step_latency_ms"),
            "p99_step_latency_ms": stat("p99_step_latency_ms"),
            "stop_reasons": {reason: int(sum(r["stop_reason"] == reason for r in rs))
                             for reason in sorted({r["stop_reason"] for r in rs})},
        }
    return out


def main():
    args = parse_args()
    scenes = tuple(x.strip() for x in args.scenes.split(",") if x.strip())
    speeds = tuple(x.strip() for x in args.speeds.split(",") if x.strip())
    if not scenes or not set(scenes).issubset(SCENES):
        raise ValueError(f"unknown scenes: {scenes}")
    if not speeds or not set(speeds).issubset(SPEEDS):
        raise ValueError(f"unknown speeds: {speeds}")
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"output must be new/empty: {out}")
    out.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    base_model = Path(args.model).resolve()
    lk = ManifoldLookup(str(Path(args.manifold).resolve()))
    rng_seed_base = int(args.seed)
    rows = []
    cells = []
    model_hashes = {}
    for scene_i, scene in enumerate(scenes):
        scene_model = base_model.parent / f"aviator_n6_scene_{scene}.xml"
        if not scene_model.exists():
            make_scene_model(base_model, scene_model, scene)
        model_hashes[scene] = sha256(scene_model)
        for speed_i, speed_name in enumerate(speeds):
            theta_dot, slide_dot = SPEEDS[speed_name]
            cell_seed = rng_seed_base + scene_i * 1000 + speed_i * 100
            fk = AviatorClosure(str(scene_model))
            checker = CollisionChecker(str(scene_model))
            lower = lk.joint_lower.reshape(14)
            upper = lk.joint_upper.reshape(14)
            cell_rows = []
            trial = 0
            attempts = 0
            cell_rng = np.random.default_rng(cell_seed)
            while trial < args.trials_per_cell and attempts < args.trials_per_cell * 2000:
                attempts += 1
                x0, phi0, lookup = sample_state(lk, cell_rng)
                q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
                d0, c0 = checker.check(x0, q0)
                if c0 or d0 < 0.007:
                    continue
                oargs = online_args(args, scene, speed_name, theta_dot, slide_dot)
                online = metrics_row("lift_filtered", x0, q0, oargs, fk, checker, lower, upper)
                online["method"] = "online_lift_filtered"
                online.update({"trial": trial, "scene": scene, "speed": speed_name,
                               "initial_x": x0.tolist(), "initial_phi": phi0.tolist(),
                               "initial_clearance_m": float(d0)})
                lut = frozen_lut_row(lk, fk, checker, x0, q0, phi0, args,
                                     scene, speed_name, theta_dot, slide_dot, trial)
                cell_rows.extend([online, lut])
                trial += 1
            if trial != args.trials_per_cell:
                raise RuntimeError(f"could not sample cell {scene}/{speed_name}: {trial}/{args.trials_per_cell}")
            rows.extend(cell_rows)
            cell_file = out / f"paired_{scene}_{speed_name}.json"
            cell_file.write_text(json.dumps({"scene": scene, "speed": speed_name,
                                             "seed": cell_seed, "trial_rows": cell_rows}, indent=2) + "\n")
            cells.append({"scene": scene, "speed": speed_name, "seed": cell_seed,
                          "trials": args.trials_per_cell, "report": str(cell_file),
                          "summary": summarize(cell_rows)})
            print(f"completed N6-A {scene}/{speed_name} ({trial} paired trials)", flush=True)
    report = {
        "kind": "n6_online_vs_frozen_lut_qualification",
        "protocol": "n6_online_lut_qualification_protocol_2026-10-06",
        "manifold": str(Path(args.manifold).resolve()),
        "base_model": str(base_model), "base_model_sha256": sha256(base_model),
        "scene_model_sha256": model_hashes,
        "scenes": list(scenes), "speeds": {s: dict(zip(("theta_dot", "slide_dot"), SPEEDS[s])) for s in speeds},
        "trials_per_cell": args.trials_per_cell, "total_paired_trials": len(rows) // 2,
        "steps_per_trial": args.steps, "dt": args.dt, "seed": args.seed,
        "d_safe_m": D_SAFE, "beta_stop": BETA_STOP,
        "methods": ["online_lift_filtered", "frozen_lut_safe_scaled"],
        "cells": cells, "summary": summarize(rows), "trial_rows": rows,
        "interpretation_boundary": [
            "frozen_lut_safe_scaled is not rebuilt under perturbations",
            "online_lift_filtered uses the frozen v3 safety/checker protocol",
            "N6-A does not claim LUT reconstruction cost savings",
            "all failures are retained by explicit method-specific stop reasons",
        ],
    }
    (out / "n6_online_vs_frozen_lut_raw.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "n6_online_vs_frozen_lut_summary.json").write_text(json.dumps({
        "kind": report["kind"], "summary": report["summary"], "cells": report["cells"],
        "base_model_sha256": report["base_model_sha256"], "scene_model_sha256": report["scene_model_sha256"],
    }, indent=2) + "\n")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()

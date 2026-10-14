#!/usr/bin/env python3
"""N8: frozen endpoint-aware governor, finite-step and held-out validation.

This runner replays the N7 state manifest and a new held-out manifest at three
time steps.  It deliberately imports the frozen N7/LUT primitives instead of
changing them.  Every online row keeps a per-step trace; nonlinear endpoint
and swept checks remain the acceptance authority.
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n6_online_lut_qualification import frozen_lut_row
from tools.validate_branch_connector import connector_step_y, swept_clearance
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_paired_baselines import dls_step, metrics_row
from tools.validate_paired_matrix import make_scene_model

D_SAFE = 0.005
BETA_STOP = 0.125
CURV_MARGIN = 0.0005
SWEPT_SUBDIVISIONS = 5
SPEEDS = {
    "gentle": (0.03, 0.0005),
    "nominal": (0.08, 0.0020),
    "stress": (0.12, 0.0030),
}
SCENES = ("nominal", "tight", "shifted")
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
    p.add_argument("--n7-manifest", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--heldout-seed", type=int, default=5200)
    p.add_argument("--trials-per-cell", type=int, default=10)
    p.add_argument("--max-states-per-cell", type=int, default=None)
    p.add_argument("--dt-list", default="0.0025,0.005,0.01")
    p.add_argument("--total-time", type=float, default=0.5)
    p.add_argument("--scenes", default=",".join(SCENES))
    p.add_argument("--speeds", default=",".join(SPEEDS))
    return p.parse_args()


def online_args(dt, steps, scene, speed_name, theta_dot, slide_dot):
    return SimpleNamespace(
        theta_dot=theta_dot, slide_dot=slide_dot,
        r_left=0.4, r_right=-0.4, feedback=2.0, dls_damping=1e-3,
        dt=dt, steps=steps, clearance_margin_m=CURV_MARGIN,
        swept_subdivisions=SWEPT_SUBDIVISIONS, governor=True,
        min_progress_scale=BETA_STOP, trace_stops=True, trace_all_steps=True,
        scene_name=scene, seed=0,
    )


def _progress(x, x0, xdot, dt, steps):
    target = np.asarray(xdot, dtype=float) * dt * steps
    delta = np.asarray(x, dtype=float) - np.asarray(x0, dtype=float)
    return float(np.min(np.divide(delta, target, out=np.zeros(2), where=np.abs(target) > 1e-12)))


def _trace_with_defaults(trace, dt, method):
    """Make the trace schema explicit without inventing unavailable values."""
    out = []
    for item in trace or []:
        row = dict(item)
        row.setdefault("method", method)
        row.setdefault("dt", float(dt))
        row.setdefault("task_scale", 1.0)
        row.setdefault("phase_scale", 1.0)
        row.setdefault("governor_attempt_count", 1)
        if row.get("d_actual_m") is None and row.get("d_endpoint_m") is not None and row.get("d_swept_m") is not None:
            row["d_actual_m"] = min(float(row["d_endpoint_m"]), float(row["d_swept_m"]))
        if row.get("delta_d_m") is None and row.get("d_actual_m") is not None and row.get("d_linear_m") is not None:
            row["delta_d_m"] = float(row["d_actual_m"] - row["d_linear_m"])
        row["diagnostic_status"] = (
            "available" if row.get("d_linear_m") is not None and row.get("d_actual_m") is not None
            else "qp_infeasible_or_no_candidate"
        )
        out.append(row)
    return out


def run_endpoint_governor(fk, checker, x0, q0, lower, upper, dt, steps, theta_dot, slide_dot):
    """Frozen N7 backtracking governor with an auditable per-step trace."""
    x, q = np.asarray(x0, dtype=float).copy(), np.asarray(q0, dtype=float).copy()
    xdot_nom = np.array([theta_dot, slide_dot], dtype=float)
    r_nom = np.array([0.4, -0.4], dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
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

    for step in range(steps):
        t0 = time.perf_counter()
        accepted = None
        attempts = []
        for task_scale, phase_scale in GOVERNOR_SCHEDULE:
            xdot_try = task_scale * xdot_nom
            r_try = phase_scale * r_nom
            qdot, n, info, sigma, beta, r_used = connector_step_y(
                fk, checker, x, q, xdot_try, r_try, previous_n,
                lower, upper, dt, beta_stop=BETA_STOP,
                clearance_margin_m=CURV_MARGIN,
            )
            sigma_min = min(sigma_min, sigma)
            governor_attempts += 1
            attempt = {
                "attempt_index": len(attempts), "task_scale": task_scale,
                "phase_scale": phase_scale, "beta": info.get("beta", beta),
                "sigma_min_Aq": float(sigma),
                "qp_feasible": bool(info.get("qp_feasible", info.get("feasible", False))),
                "candidate_feasible": bool(info.get("feasible", False)),
                "d0_m": info.get("d0"), "d_linear_m": info.get("linear_predicted_clearance_m"),
                "d_endpoint_qp_m": info.get("d_next"), "d_swept_qp_m": info.get("swept_min"),
                "reason": info.get("stop_reason", "none"),
            }
            if not info.get("feasible", False):
                attempts.append(attempt)
                continue

            xdot_used = float(beta) * xdot_try
            x_mid = x + 0.5 * dt * xdot_used
            q_mid = q + 0.5 * dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot_used, np.asarray(r_used), n, 2.0, 0.0
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + dt * xdot_used
            q_next = q + dt * qdot_mid
            d_next, contact_next = checker.check(x_next, q_next)
            swept_min, swept_contact = swept_clearance(
                checker, x, q, x_next, q_next, subdivisions=SWEPT_SUBDIVISIONS
            )
            limit = bool(np.any(q_next < lower - 1e-9) or np.any(q_next > upper + 1e-9))
            closure = float(np.max(np.abs(fk.error(x_next, q_next))))
            accepted_ok = (
                not contact_next and not swept_contact and
                d_next >= D_SAFE - 1e-9 and swept_min >= D_SAFE - 1e-9 and not limit
            )
            attempt.update({
                "d_endpoint_m": float(d_next), "d_swept_m": float(swept_min),
                "d_actual_m": float(min(d_next, swept_min)),
                "delta_d_m": None if info.get("linear_predicted_clearance_m") is None else float(min(d_next, swept_min) - info["linear_predicted_clearance_m"]),
                "closure_residual_inf": closure,
                "joint_limit_margin_rad": float(min(np.min(q_next - lower), np.min(upper - q_next))),
                "joint_limit_failure": limit,
                "endpoint_pass": bool(d_next >= D_SAFE - 1e-9 and not contact_next),
                "swept_pass": bool(swept_min >= D_SAFE - 1e-9 and not swept_contact),
                "midpoint_check": True, "midpoint_accepted": bool(accepted_ok),
                "reason": "accepted" if accepted_ok else "post_integrator_check",
            })
            attempts.append(attempt)
            if accepted_ok:
                accepted = {
                    "task_scale": task_scale, "phase_scale": phase_scale,
                    "beta": float(beta), "r": np.asarray(r_used).tolist(),
                    "x_next": x_next, "q_next": q_next, "n_next": n_mid,
                    "d_linear": info.get("linear_predicted_clearance_m"),
                    "d0": info.get("d0"), "d_endpoint": float(d_next),
                    "d_swept": float(swept_min), "d_actual": float(min(d_next, swept_min)),
                    "closure": closure,
                    "joint_limit_margin": float(min(np.min(q_next - lower), np.min(upper - q_next))),
                    "sigma": float(min(sigma, sigma_mid)),
                }
                break

        elapsed_ms = (time.perf_counter() - t0) * 1e3
        step_latencies.append(elapsed_ms)
        if accepted is None:
            stop_reason = "governor_exhausted"
            last = attempts[-1] if attempts else {}
            progress_before = _progress(x, x0, xdot_nom, dt, steps)
            stop_trace = {"step": step, "progress_before": progress_before, "attempts": attempts}
            trace.append({
                "step": step, "accepted": False, "task_progress_before": progress_before,
                "task_progress_after": progress_before, "step_latency_ms": elapsed_ms,
                "stop_reason": stop_reason, "attempts": attempts,
                "d0_m": last.get("d0_m"), "d_linear_m": last.get("d_linear_m"),
                "d_endpoint_m": last.get("d_endpoint_m"), "d_swept_m": last.get("d_swept_m"),
                "d_actual_m": last.get("d_actual_m"), "delta_d_m": last.get("delta_d_m"),
                "beta": last.get("beta"), "sigma_min_Aq": last.get("sigma_min_Aq"),
                "joint_limit_margin_rad": last.get("joint_limit_margin_rad"),
                "qp_feasible": last.get("qp_feasible"),
                "diagnostic_status": "available" if last.get("d_actual_m") is not None else "qp_infeasible_or_no_candidate",
            })
            break

        # Preserve the complete accepted transition for downstream GFR demo
        # conversion.  Historical JSON files are immutable because this
        # function only adds fields to future runs.
        x_before, q_before = x.copy(), q.copy()
        x, q, previous_n = accepted["x_next"], accepted["q_next"], accepted["n_next"]
        steps_done += 1
        if accepted["task_scale"] < 1.0 or accepted["phase_scale"] < 1.0 or accepted["beta"] < 1.0 - 1e-10:
            accepted_governed_steps += 1
        min_clearance = min(min_clearance, accepted["d_endpoint"])
        min_swept = min(min_swept, accepted["d_swept"])
        max_closure = max(max_closure, accepted["closure"])
        progress = _progress(x, x0, xdot_nom, dt, steps)
        trace.append({
            "step": step, "accepted": True, "task_scale": accepted["task_scale"],
            "phase_scale": accepted["phase_scale"], "governor_attempt_count": len(attempts),
            "beta": accepted["beta"], "d0_m": accepted["d0"], "d_linear_m": accepted["d_linear"],
            "d_endpoint_m": accepted["d_endpoint"], "d_swept_m": accepted["d_swept"],
            "d_actual_m": accepted["d_actual"],
            "delta_d_m": None if accepted["d_linear"] is None else accepted["d_actual"] - accepted["d_linear"],
            "task_progress_before": _progress(x - dt * accepted["beta"] * accepted["task_scale"] * xdot_nom, x0, xdot_nom, dt, steps),
            "task_progress_after": progress, "sigma_min_Aq": accepted["sigma"],
            "closure_residual_inf": accepted["closure"],
            "joint_limit_margin_rad": accepted["joint_limit_margin"],
            "qp_feasible": True, "endpoint_pass": True, "swept_pass": True,
            "accepted": True, "stop_reason": "none", "attempts": attempts,
            "step_latency_ms": elapsed_ms, "diagnostic_status": "available",
            "r_nom": (r_nom * float(accepted["phase_scale"])).tolist(),
            "r_exec": np.asarray(accepted["r"], dtype=float).tolist(),
            "x_before": x_before.tolist(), "q_before": q_before.tolist(),
            "x_after": np.asarray(x, dtype=float).tolist(),
            "q_after": np.asarray(q, dtype=float).tolist(),
            "xdot_nom": np.asarray(xdot_nom, dtype=float).tolist(),
        })

    latency = np.asarray(step_latencies, dtype=float)
    return {
        "method": "online_endpoint_governor", "steps": steps_done,
        "complete": bool(steps_done == steps), "stopped": bool(steps_done != steps),
        "stop_reason": stop_reason, "task_progress_fraction": _progress(x, x0, xdot_nom, dt, steps),
        "min_clearance_m": float(min_clearance), "min_swept_clearance_m": float(min_swept),
        "max_closure_error_mixed_m_rad": float(max_closure), "sigma_min_Aq": float(sigma_min),
        "governor_attempts": int(governor_attempts), "accepted_governed_steps": int(accepted_governed_steps),
        "mean_step_latency_ms": float(np.mean(latency)) if len(latency) else 0.0,
        "p95_step_latency_ms": float(np.quantile(latency, .95)) if len(latency) else 0.0,
        "p99_step_latency_ms": float(np.quantile(latency, .99)) if len(latency) else 0.0,
        "attempt_trace": trace, "step_trace": trace, "stop_trace": stop_trace,
    }


def make_manifest(stale_lk, base_model, scenes, speeds, seed, trials_per_cell):
    states = []
    for scene_i, scene in enumerate(scenes):
        scene_model = base_model.parent / f"aviator_n6_scene_{scene}.xml"
        if not scene_model.exists():
            make_scene_model(base_model, scene_model, scene)
        checker = CollisionChecker(str(scene_model))
        for speed_i, speed_name in enumerate(speeds):
            theta_dot, slide_dot = SPEEDS[speed_name]
            rng = np.random.default_rng(int(seed + scene_i * 1000 + speed_i * 100))
            cell = []
            attempts = 0
            while len(cell) < trials_per_cell and attempts < trials_per_cell * 2000:
                attempts += 1
                x0, phi0, lookup = sample_state(stale_lk, rng)
                q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
                d0, contact = checker.check(x0, q0)
                if contact or d0 < 0.007:
                    continue
                cell.append({
                    "trial": len(cell), "scene": scene, "speed": speed_name,
                    "seed": int(seed + scene_i * 1000 + speed_i * 100),
                    "x0": x0.tolist(), "q0": q0.tolist(), "phi0": phi0.tolist(),
                    "initial_clearance_m": float(d0), "theta_dot": theta_dot,
                    "slide_dot": slide_dot,
                })
            if len(cell) != trials_per_cell:
                raise RuntimeError(f"could not sample held-out {scene}/{speed_name}: {len(cell)}/{trials_per_cell}")
            states.extend(cell)
    return {
        "kind": "n8_heldout_state_manifest", "seed": seed, "scenes": list(scenes),
        "speeds": list(speeds), "trials_per_cell": trials_per_cell,
        "d_safe_m": D_SAFE, "states": states,
    }


def load_states(path, scenes, speeds, max_states_per_cell=None):
    data = json.loads(Path(path).read_text())
    selected = []
    for scene in scenes:
        for speed in speeds:
            cell = [s for s in data["states"] if s["scene"] == scene and s["speed"] == speed]
            if max_states_per_cell is not None:
                cell = cell[:max_states_per_cell]
            selected.extend(cell)
    return selected, data


def main():
    args = parse_args()
    scenes = tuple(x.strip() for x in args.scenes.split(",") if x.strip())
    speeds = tuple(x.strip() for x in args.speeds.split(",") if x.strip())
    dts = tuple(float(x) for x in args.dt_list.split(",") if x.strip())
    if not set(scenes).issubset(SCENES) or not set(speeds).issubset(SPEEDS):
        raise ValueError("unknown scene or speed")
    for dt in dts:
        if round(args.total_time / dt) < 1:
            raise ValueError(f"invalid dt {dt}")
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"output must be new/empty: {out}")
    out.mkdir(parents=True, exist_ok=True)
    stale_lk = ManifoldLookup(str(Path(args.stale_manifold).resolve()))
    base_model = Path(args.model).resolve()
    n7_states, n7_manifest = load_states(args.n7_manifest, scenes, speeds, args.max_states_per_cell)
    n7_replay = {
        **n7_manifest, "kind": "n8_n7_replay_state_manifest",
        "source_sha256": sha256(Path(args.n7_manifest)), "states": n7_states,
    }
    (out / "n8_n7_replay_state_manifest.json").write_text(json.dumps(n7_replay, indent=2) + "\n")
    heldout = make_manifest(stale_lk, base_model, scenes, speeds, args.heldout_seed,
                            args.max_states_per_cell or args.trials_per_cell)
    (out / "n8_heldout_state_manifest.json").write_text(json.dumps(heldout, indent=2) + "\n")

    lower = np.concatenate([stale_lk.joint_lower[0], stale_lk.joint_lower[1]])
    upper = np.concatenate([stale_lk.joint_upper[0], stale_lk.joint_upper[1]])
    rebuilt_root = Path(args.rebuilt_root).resolve()
    rows = []
    manifests = (("n7_replay", n7_states), ("heldout", heldout["states"]))
    fk_cache, checker_cache, rebuilt_cache = {}, {}, {}
    for manifest_name, states in manifests:
        for dt in dts:
            steps = int(round(args.total_time / dt))
            for st in states:
                scene, speed_name, trial = st["scene"], st["speed"], int(st["trial"])
                theta_dot, slide_dot = SPEEDS[speed_name]
                scene_model = base_model.parent / f"aviator_n6_scene_{scene}.xml"
                if scene not in fk_cache:
                    fk_cache[scene] = AviatorClosure(str(scene_model))
                    checker_cache[scene] = CollisionChecker(str(scene_model))
                    rebuilt_cache[scene] = ManifoldLookup(str(rebuilt_root / f"n6_rebuilt_lut_{scene}" / "manifold_phi"))
                fk, checker, rebuilt_lk = fk_cache[scene], checker_cache[scene], rebuilt_cache[scene]
                x0, q0, phi0 = (np.asarray(st[k], dtype=float) for k in ("x0", "q0", "phi0"))
                oargs = online_args(dt, steps, scene, speed_name, theta_dot, slide_dot)
                current = metrics_row("lift_filtered", x0, q0, oargs, fk, checker, lower, upper)
                current["method"] = "online_lift_filtered"
                current["step_trace"] = _trace_with_defaults(current.get("step_trace"), dt, current["method"])
                gov = run_endpoint_governor(fk, checker, x0, q0, lower, upper, dt, steps, theta_dot, slide_dot)
                stale = frozen_lut_row(stale_lk, fk, checker, x0, q0, phi0, oargs, scene, speed_name, theta_dot, slide_dot, trial)
                rebuilt = frozen_lut_row(rebuilt_lk, fk, checker, x0, q0, phi0, oargs, scene, speed_name, theta_dot, slide_dot, trial)
                for method, row in (("online_lift_filtered", current), ("online_endpoint_governor", gov),
                                    ("stale_lut_safe_scaled", stale), ("rebuilt_lut_safe_scaled", rebuilt)):
                    row.update({
                        "method": method, "manifest": manifest_name, "trial": trial,
                        "scene": scene, "speed": speed_name, "dt": dt, "steps_target": steps,
                        "initial_x": st["x0"], "initial_q": st["q0"], "initial_phi": st["phi0"],
                        "initial_clearance_m": st["initial_clearance_m"],
                        "clearance_margin_m": CURV_MARGIN,
                    })
                    if method == "online_endpoint_governor":
                        row["step_trace"] = _trace_with_defaults(row.get("step_trace"), dt, method)
                    rows.append(row)
            print(f"completed N8 {manifest_name} dt={dt:g} {len(states)} states", flush=True)

    report = {
        "kind": "n8_dt_heldout_raw",
        "stale_manifold": str(Path(args.stale_manifold).resolve()),
        "rebuilt_root": str(rebuilt_root), "base_model": str(base_model),
        "base_model_sha256": sha256(base_model), "n7_manifest_sha256": sha256(Path(args.n7_manifest)),
        "heldout_seed": args.heldout_seed, "scenes": list(scenes), "speeds": list(speeds),
        "dts": list(dts), "total_time_s": args.total_time, "trials_per_cell": args.trials_per_cell,
        "d_safe_m": D_SAFE, "beta_stop": BETA_STOP, "clearance_margin_m": CURV_MARGIN,
        "swept_subdivisions": SWEPT_SUBDIVISIONS, "governor_schedule": [list(x) for x in GOVERNOR_SCHEDULE],
        "methods": ["online_lift_filtered", "online_endpoint_governor", "stale_lut_safe_scaled", "rebuilt_lut_safe_scaled"],
        "n7_state_manifest": str((out / "n8_n7_replay_state_manifest.json").resolve()),
        "heldout_state_manifest": str((out / "n8_heldout_state_manifest.json").resolve()),
        "trial_rows": rows,
        "interpretation_boundary": [
            "N8 does not claim global IK branch preservation or continuous-time invariance.",
            "m_curv=0.50 mm is an empirical finite-step parameter.",
            "100 accepted steps do not imply nominal full-speed task completion; progress is reported separately.",
            "LUT and online latency are compared as a safety/progress trade-off, not universal superiority.",
        ],
    }
    (out / "n8_raw_trials.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({m: {"rows": sum(r["method"] == m for r in rows),
                         "complete": sum(bool(r.get("complete")) for r in rows if r["method"] == m)}
                     for m in report["methods"]}, indent=2))


if __name__ == "__main__":
    main()

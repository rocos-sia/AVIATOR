#!/usr/bin/env python3
"""N17 paired comparison: velocity damper, full QP, and reduced governor.

The runner reuses the N16 paired states and perturbed MuJoCo models.  It does
not alter any N16 file.  Every method is evaluated with the same nonlinear
endpoint and swept checker; only the candidate representation and governor
policy differ.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import LinearConstraint, minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n8_dt_heldout import run_endpoint_governor
from tools.validate_branch_connector import (
    clearance_gradient,
    swept_clearance,
    swivel_descriptor,
)
from tools.validate_differential_lift import AviatorClosure
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis
from tools.validate_paired_baselines import full_joint_qp_step


D_SAFE = 0.005
BETA_STOP = 0.125
CURV_MARGIN = 0.0005
SWEPT_SUBDIVISIONS = 5
VELOCITY_DAMPER_ETA = 5.0
QDOT_MAX = 1.5
R_NOM = np.array([0.4, -0.4], dtype=float)
GOVERNOR_SCHEDULE = (
    (1.0, 1.0),
    (0.75, 1.0), (0.5, 1.0), (0.25, 1.0), (0.125, 1.0),
    (1.0, 0.75), (1.0, 0.5), (1.0, 0.25), (1.0, 0.125),
    (0.75, 0.75), (0.5, 0.5), (0.25, 0.25), (0.125, 0.125),
)
TASK_ONLY_SCHEDULE = tuple((s, 1.0) for s in (1.0, 0.75, 0.5, 0.25, 0.125))
SPEEDS = {
    "gentle": (0.03, 0.0005),
    "nominal": (0.08, 0.0020),
    "stress": (0.12, 0.0030),
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def progress(x: np.ndarray, x0: np.ndarray, xdot: np.ndarray, dt: float, steps: int) -> float:
    target = np.asarray(xdot, dtype=float) * dt * steps
    delta = np.asarray(x, dtype=float) - np.asarray(x0, dtype=float)
    return float(np.min(np.divide(delta, target, out=np.zeros(2), where=np.abs(target) > 1e-12)))


def _grad_x(checker, x, q):
    out = np.zeros(2, dtype=float)
    for k, eps in enumerate((1e-5, 1e-6)):
        xp, xm = np.asarray(x, dtype=float).copy(), np.asarray(x, dtype=float).copy()
        xp[k] += eps
        xm[k] -= eps
        out[k] = (checker.check(xp, q)[0] - checker.check(xm, q)[0]) / (2.0 * eps)
    return out


def velocity_damper_step(fk, checker, x, q, xdot, r_nom, previous_n,
                         lower, upper, dt, eta=VELOCITY_DAMPER_ETA):
    """Solve a classical velocity-damper QP in all 14 joint velocities.

    The equality preserves the differential closure relation.  The clearance
    row is the standard rate damper
    ``grad(d) qdot + grad_x(d) xdot + eta (d-d_safe) >= 0``.
    Endpoint and swept checks are still performed after solving.
    """
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_n)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    rhs = -ax @ xdot - 2.0 * e
    horizontal = pinv @ rhs
    qdot_nom = horizontal + n @ r_nom
    d0, contact0 = checker.check(x, q)
    if contact0 or not np.isfinite(d0):
        return np.zeros(14), n, {
            "feasible": False, "qp_feasible": False, "d0": float(d0),
            "beta": 0.0, "stop_reason": "invalid_initial_clearance",
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": D_SAFE,
        }, float(singular[-1])

    grad = clearance_gradient(checker, x, q)
    grad_x = _grad_x(checker, x, q)
    # A qdot <= b, including joint-rate, one-step joint-limit, and rate-damper rows.
    A, b = [], []
    for j in range(14):
        row = np.zeros(14); row[j] = 1.0
        A.extend([row, -row, dt * row, -dt * row])
        b.extend([QDOT_MAX, QDOT_MAX, upper[j] - q[j], q[j] - lower[j]])
    # grad*qdot + grad_x*xdot + eta*(d0-dsafe) >= 0.
    A.append(-grad)
    b.append(float(grad_x @ xdot + eta * (d0 - D_SAFE)))
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float)

    def objective(z):
        diff = z - qdot_nom
        return 0.5 * float(diff @ diff)

    def objective_grad(z):
        return z - qdot_nom

    result = minimize(
        objective, np.clip(qdot_nom, -QDOT_MAX, QDOT_MAX), jac=objective_grad,
        method="SLSQP", bounds=[(-QDOT_MAX, QDOT_MAX)] * 14,
        constraints=[
            # Match the full-space QP's hard differential-closure equality;
            # a loose band here would measure solver tolerance rather than
            # the velocity-damper formulation.
            LinearConstraint(aq, -2.0 * e, -2.0 * e),
            LinearConstraint(A, -np.inf, b),
        ], options={"ftol": 1e-10, "maxiter": 250},
    )
    if not result.success or np.max(np.abs(aq @ result.x + 2.0 * e)) > 5e-5:
        return np.zeros(14), n, {
            "feasible": False, "qp_feasible": False, "d0": float(d0),
            "beta": 1.0, "stop_reason": "velocity_damper_infeasible",
            "optimizer_success": bool(result.success),
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": D_SAFE,
        }, float(singular[-1])

    qdot = np.asarray(result.x, dtype=float)
    x_next = x + dt * xdot
    q_next = q + dt * qdot
    d_next, contact_next = checker.check(x_next, q_next)
    swept_min, swept_contact = swept_clearance(
        checker, x, q, x_next, q_next, subdivisions=SWEPT_SUBDIVISIONS
    )
    limit = bool(np.any(q_next < lower - 1e-9) or np.any(q_next > upper + 1e-9))
    d_pred = float(d0 + dt * (grad @ qdot + grad_x @ xdot))
    limit_margin = float(min(np.min(q_next - lower), np.min(upper - q_next)))
    valid = (
        d_next >= D_SAFE - 1e-9 and swept_min >= D_SAFE - 1e-9
        and not contact_next and not swept_contact and not limit
        and np.max(np.abs(qdot)) <= QDOT_MAX + 2e-5
    )
    info = {
        "feasible": bool(valid), "qp_feasible": True, "d0": float(d0),
        "beta": 1.0, "governed": False, "r": n.T @ (qdot - horizontal),
        "linear_predicted_clearance_m": d_pred,
        "linear_required_clearance_m": D_SAFE,
        "d_next": float(d_next), "swept_min": float(swept_min),
        "joint_limit_margin_rad": limit_margin,
        "qdot_max_rad_s": float(np.max(np.abs(qdot))),
        "endpoint_contact": bool(contact_next), "swept_contact": bool(swept_contact),
        "endpoint_pass": bool(d_next >= D_SAFE - 1e-9 and not contact_next),
        "swept_pass": bool(swept_min >= D_SAFE - 1e-9 and not swept_contact),
        "stop_reason": "none" if valid else "velocity_damper_endpoint_validation",
        "eta_s_inv": eta,
    }
    return qdot if valid else np.zeros(14), n, info, float(singular[-1])


def _trace_row(step, accepted, method, elapsed_ms, x_before, x_after, x0, xdot,
               dt, steps, info=None, sigma=None, task_scale=1.0,
               phase_scale=1.0, stop_reason="none"):
    info = info or {}
    d_endpoint = info.get("d_next")
    d_swept = info.get("swept_min")
    d_actual = None if d_endpoint is None or d_swept is None else float(min(d_endpoint, d_swept))
    d_linear = info.get("linear_predicted_clearance_m")
    return {
        "step": int(step), "accepted": bool(accepted), "method": method,
        "task_scale": float(task_scale), "phase_scale": float(phase_scale),
        "beta": float(info.get("beta", 0.0)),
        "task_progress_before": progress(x_before, x0, xdot, dt, steps),
        "task_progress_after": progress(x_after, x0, xdot, dt, steps),
        "d0_m": info.get("d0"), "d_linear_m": d_linear,
        "d_endpoint_m": d_endpoint, "d_swept_m": d_swept,
        "d_actual_m": d_actual,
        "delta_d_m": None if d_actual is None or d_linear is None else float(d_actual - d_linear),
        "sigma_min_Aq": None if sigma is None else float(sigma),
        "closure_residual_inf": info.get("closure_residual_inf"),
        "joint_limit_margin_rad": info.get("joint_limit_margin_rad"),
        "qp_feasible": info.get("qp_feasible"),
        "endpoint_pass": info.get("endpoint_pass"),
        "swept_pass": info.get("swept_pass"),
        "accepted": bool(accepted), "stop_reason": stop_reason,
        "step_latency_ms": float(elapsed_ms),
    }


def run_full_or_damper(method, fk, checker, x0, q0, lower, upper, xdot_nom,
                       dt, steps, governed):
    x, q = np.asarray(x0, dtype=float).copy(), np.asarray(q0, dtype=float).copy()
    previous_n = None
    traces, latencies = [], []
    min_clearance = float(checker.check(x, q)[0])
    min_swept = min_clearance
    sigma_min = float("inf")
    max_closure = 0.0
    governed_steps = 0
    attempts_total = 0
    steps_done = 0
    stop_reason = "complete"
    r_nom = R_NOM.copy()
    schedule = TASK_ONLY_SCHEDULE if method == "velocity_damper_governed" else ((1.0, 1.0),)
    if method == "full_joint_qp_governor":
        schedule = GOVERNOR_SCHEDULE

    for step in range(steps):
        t0 = time.perf_counter()
        accepted = None
        attempts = []
        for task_scale, phase_scale in schedule:
            xdot_try = float(task_scale) * xdot_nom
            r_try = float(phase_scale) * r_nom
            if method == "velocity_damper" or method == "velocity_damper_governed":
                qdot, n, info, sigma = velocity_damper_step(
                    fk, checker, x, q, xdot_try, r_try, previous_n,
                    lower, upper, dt
                )
                beta = 1.0
            else:
                qdot, n, info, sigma, beta = full_joint_qp_step(
                    fk, checker, x, q, xdot_try, r_try, previous_n,
                    lower, upper, dt, beta_stop=BETA_STOP, qdot_max=QDOT_MAX,
                    d_safe=D_SAFE, clearance_margin_m=CURV_MARGIN
                )
            attempts_total += 1
            sigma_min = min(sigma_min, float(sigma))
            attempts.append({"task_scale": task_scale, "phase_scale": phase_scale,
                             "qp_feasible": bool(info.get("qp_feasible", False)),
                             "candidate_feasible": bool(info.get("feasible", False)),
                             "stop_reason": info.get("stop_reason", "unknown"),
                             "beta": float(info.get("beta", beta))})
            if not info.get("feasible", False):
                continue
            # The velocity-damper baseline gets a midpoint re-solve because a
            # rate damper otherwise accumulates visible nonlinear closure
            # drift.  The full-QP control uses the canonical
            # ``full_joint_qp_step`` candidate directly, matching the existing
            # full-QP baseline and keeping its numerical budget explicit.
            beta_first = float(info.get("beta", beta))
            if method in {"velocity_damper", "velocity_damper_governed"}:
                x_mid = x + 0.5 * dt * beta_first * xdot_try
                q_mid = q + 0.5 * dt * qdot
                qdot_mid, n_mid, mid_info, sigma_mid = velocity_damper_step(
                    fk, checker, x_mid, q_mid, xdot_try, r_try, n,
                    lower, upper, dt
                )
                beta_mid = 1.0
            else:
                qdot_mid, n_mid, mid_info, sigma_mid, beta_mid = (
                    qdot, n, info, sigma, beta_first
                )
            if not mid_info.get("feasible", False):
                attempts[-1].update({"midpoint_qp_feasible": bool(mid_info.get("qp_feasible", False)),
                                     "midpoint_stop_reason": mid_info.get("stop_reason", "unknown")})
                attempts[-1]["stop_reason"] = (
                    "velocity_damper_midpoint_infeasible"
                    if method in {"velocity_damper", "velocity_damper_governed"}
                    else "full_qp_midpoint_infeasible"
                )
                continue
            qdot = qdot_mid
            n = n_mid
            sigma = min(float(sigma), float(sigma_mid))
            info = dict(info)
            info["beta"] = float(beta_mid)
            info["qp_feasible"] = bool(info.get("qp_feasible", False) and mid_info.get("qp_feasible", False))
            x_next = x + dt * float(beta_mid) * xdot_try
            q_next = q + dt * qdot
            # Re-check the actual integrated candidate, rather than accepting
            # only the solver's internal candidate.
            d_next, contact_next = checker.check(x_next, q_next)
            swept_min, swept_contact = swept_clearance(
                checker, x, q, x_next, q_next, subdivisions=SWEPT_SUBDIVISIONS
            )
            closure = float(np.max(np.abs(fk.error(x_next, q_next))))
            limit = bool(np.any(q_next < lower - 1e-9) or np.any(q_next > upper + 1e-9))
            ok = (not contact_next and not swept_contact and not limit
                  and d_next >= D_SAFE - 1e-9 and swept_min >= D_SAFE - 1e-9)
            info = dict(info)
            info.update({"d_next": float(d_next), "swept_min": float(swept_min),
                         "endpoint_pass": bool(d_next >= D_SAFE - 1e-9 and not contact_next),
                         "swept_pass": bool(swept_min >= D_SAFE - 1e-9 and not swept_contact),
                         "joint_limit_margin_rad": float(min(np.min(q_next - lower), np.min(upper - q_next))),
                         "closure_residual_inf": closure,
                         "stop_reason": "none" if ok else "post_integrator_check"})
            attempts[-1].update({"d_endpoint_m": float(d_next), "d_swept_m": float(swept_min),
                                 "closure_residual_inf": closure, "accepted": bool(ok)})
            if ok:
                accepted = (q_next, n, info, float(sigma), task_scale, phase_scale)
                break
        elapsed = (time.perf_counter() - t0) * 1e3
        latencies.append(elapsed)
        if accepted is None:
            stop_reason = attempts[-1]["stop_reason"] if attempts else "no_candidate"
            traces.append(_trace_row(step, False, method, elapsed, x, x, x0,
                                     xdot_nom, dt, steps, info if attempts else {},
                                     sigma_min, stop_reason=stop_reason))
            break
        q_next, n_next, info, sigma, task_scale, phase_scale = accepted
        x_next = x + dt * float(info.get("beta", 1.0)) * task_scale * xdot_nom
        if task_scale < 1.0 - 1e-12 or phase_scale < 1.0 - 1e-12 or float(info.get("beta", 1.0)) < 1.0 - 1e-12:
            governed_steps += 1
        traces.append(_trace_row(step, True, method, elapsed, x, x_next, x0,
                                 xdot_nom, dt, steps, info, sigma,
                                 task_scale, phase_scale))
        x, q, previous_n = x_next, q_next, n_next
        steps_done += 1
        min_clearance = min(min_clearance, float(info["d_next"]))
        min_swept = min(min_swept, float(info["swept_min"]))
        max_closure = max(max_closure, float(info.get("closure_residual_inf", 0.0)))

    arr = np.asarray(latencies, dtype=float)
    return {
        "method": method, "steps": steps_done, "complete": steps_done == steps,
        "stopped": steps_done != steps, "stop_reason": stop_reason,
        "task_progress_fraction": progress(x, x0, xdot_nom, dt, steps),
        "min_clearance_m": float(min_clearance), "min_swept_clearance_m": float(min_swept),
        "max_closure_error_mixed_m_rad": float(max_closure),
        "sigma_min_Aq": float(sigma_min), "governor_attempts": int(attempts_total),
        "accepted_governed_steps": int(governed_steps),
        "mean_step_latency_ms": float(np.mean(arr)) if arr.size else 0.0,
        "p95_step_latency_ms": float(np.quantile(arr, .95)) if arr.size else 0.0,
        "p99_step_latency_ms": float(np.quantile(arr, .99)) if arr.size else 0.0,
        "step_trace": traces, "attempt_trace": traces,
    }


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--n16-raw", required=True)
    p.add_argument("--manifold", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--methods", default="online_endpoint_governor,full_joint_qp_governor,velocity_damper,velocity_damper_governed")
    p.add_argument("--families", default="")
    p.add_argument("--speeds", default="gentle,nominal,stress")
    p.add_argument("--max-trials-per-cell", type=int, default=None)
    p.add_argument("--max-cells", type=int, default=None)
    return p.parse_args()


def _stats(rows):
    out = {}
    for method in sorted({r["method"] for r in rows}):
        rr = [r for r in rows if r["method"] == method]
        def stat(k):
            a = np.asarray([r[k] for r in rr], dtype=float)
            return {"median": float(np.median(a)), "p05": float(np.quantile(a, .05)),
                    "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}
        out[method] = {
            "trials": len(rr), "complete": int(sum(bool(r["complete"]) for r in rr)),
            "completion_rate": float(np.mean([bool(r["complete"]) for r in rr])),
            "steps": stat("steps"), "task_progress_fraction": stat("task_progress_fraction"),
            "min_clearance_m": stat("min_clearance_m"),
            "min_swept_clearance_m": stat("min_swept_clearance_m"),
            "p95_step_latency_ms": stat("p95_step_latency_ms"),
            "p99_step_latency_ms": stat("p99_step_latency_ms"),
            "max_closure_error_mixed_m_rad": stat("max_closure_error_mixed_m_rad"),
            "governor_attempts": int(sum(r["governor_attempts"] for r in rr)),
            "accepted_governed_steps": int(sum(r["accepted_governed_steps"] for r in rr)),
            "stop_reasons": {k: int(sum(r["stop_reason"] == k for r in rr))
                             for k in sorted({r["stop_reason"] for r in rr})},
        }
    return out


def main():
    args = parse_args()
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"output must be empty/new: {out}")
    out.mkdir(parents=True, exist_ok=True)
    raw = json.loads(Path(args.n16_raw).read_text())
    variants = {v["model_sha256"]: Path(v["model"]) for v in raw["variants"]}
    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    speeds = [s.strip() for s in args.speeds.split(",") if s.strip()]
    families = set(x.strip() for x in args.families.split(",") if x.strip())
    # One state per (family,level,trial); N16 methods/speeds share these states.
    unique = {}
    for row in raw["trial_rows"]:
        key = (row["family"], float(row["error_level"]), int(row["trial"]))
        if families and row["family"] not in families:
            continue
        unique.setdefault(key, row)
    keys = sorted(unique)
    if args.max_cells is not None:
        keys = keys[: int(args.max_cells)]
    if args.max_trials_per_cell is not None:
        counts = {}
        trimmed = []
        for key in keys:
            cell = key[:2]
            if counts.get(cell, 0) >= args.max_trials_per_cell:
                continue
            counts[cell] = counts.get(cell, 0) + 1
            trimmed.append(key)
        keys = trimmed
    lk = ManifoldLookup(str(Path(args.manifold).resolve()))
    lower = lk.joint_lower.reshape(14)
    upper = lk.joint_upper.reshape(14)
    rows, cell_records = [], []
    for idx, key in enumerate(keys):
        template = unique[key]
        model = variants[template["model_sha256"]]
        family, level, trial = key
        fk = AviatorClosure(str(model))
        checker = CollisionChecker(str(model))
        x0 = np.asarray(template["initial_x"], dtype=float)
        q0 = np.asarray(template["initial_q"], dtype=float)
        for speed in speeds:
            if speed not in SPEEDS:
                raise ValueError(f"unknown speed {speed}")
            theta_dot, slide_dot = SPEEDS[speed]
            xdot = np.array([theta_dot, slide_dot], dtype=float)
            for method in methods:
                if method == "online_endpoint_governor":
                    result = run_endpoint_governor(
                        fk, checker, x0, q0, lower, upper, 0.005, 100,
                        theta_dot, slide_dot
                    )
                elif method in {"full_joint_qp_governor", "velocity_damper", "velocity_damper_governed"}:
                    result = run_full_or_damper(
                        method, fk, checker, x0, q0, lower, upper, xdot, 0.005, 100,
                        governed=method != "velocity_damper"
                    )
                else:
                    raise ValueError(f"unknown method {method}")
                result.update({"family": family, "error_level": level,
                               "speed": speed, "trial": trial, "model": str(model),
                               "model_sha256": template["model_sha256"],
                               "initial_x": template["initial_x"], "initial_q": template["initial_q"],
                               "initial_phi": template["initial_phi"],
                               "initial_clearance_m": template["initial_clearance_m"],
                               "dt": 0.005, "steps_target": 100,
                               "d_safe_m": D_SAFE, "beta_stop": BETA_STOP,
                               "curvature_margin_m": CURV_MARGIN,
                               "velocity_damper_eta_s_inv": VELOCITY_DAMPER_ETA,
                               "swept_subdivisions": SWEPT_SUBDIVISIONS})
                rows.append(result)
            print(f"N17 {idx+1}/{len(keys)} {family} level={level:g} trial={trial} speed={speed}", flush=True)

    report = {
        "kind": "n17_velocity_damper_full_qp_raw",
        "protocol": "N17 velocity-damper/full-QP/reduced comparison 2026-10-11",
        "n16_raw": str(Path(args.n16_raw).resolve()),
        "n16_raw_sha256": sha256(Path(args.n16_raw)),
        "manifold": str(Path(args.manifold).resolve()),
        "methods": methods, "families": sorted(families) if families else raw["families"],
        "speeds": speeds, "dt": 0.005, "steps": 100, "d_safe_m": D_SAFE,
        "beta_stop": BETA_STOP, "curvature_margin_m": CURV_MARGIN,
        "velocity_damper_eta_s_inv": VELOCITY_DAMPER_ETA,
        "swept_subdivisions": SWEPT_SUBDIVISIONS,
        "representation_boundary": [
            "No rebuilt LUT, RL, TSR, or GSR is evaluated in N17.",
            "All methods share the N16 paired state/model manifest and nonlinear checker.",
            "Full QP is an equality-constrained joint-space QP, not a claim of hierarchical QP.",
            "Velocity damper results are conditional on eta=5 s^-1 and the tested local states.",
        ],
        "summary": _stats(rows), "trial_rows": rows,
    }
    (out / "n17_velocity_damper_full_qp_raw.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "n17_velocity_damper_full_qp_summary.json").write_text(json.dumps({
        "kind": report["kind"], "summary": report["summary"],
        "methods": methods, "rows": len(rows),
    }, indent=2) + "\n")
    print(json.dumps({"rows": len(rows), "summary": report["summary"]}, indent=2))


if __name__ == "__main__":
    main()

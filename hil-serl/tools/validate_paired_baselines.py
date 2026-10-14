#!/usr/bin/env python3
"""Paired baseline experiment for the branch-preserving differential lift.

All methods replay the same sampled states, task velocity, nominal phase rate,
MuJoCo model, and independent clearance checker.  This is a first small
protocol run; it intentionally separates the proposed filtered connector from
unfiltered H/N, DLS differential IK, and warm-start pointwise projection.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import LinearConstraint, least_squares, minimize

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis
from tools.validate_branch_connector import (
    clearance_gradient,
    governor_candidates,
    swept_clearance,
    swivel_descriptor,
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials", type=int, default=4)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument(
        "--clearance-margin-m", type=float, default=0.0,
        help="extra margin used only by the linear candidate constraint; nonlinear validation keeps d_safe=0.005 m",
    )
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--theta-dot", type=float, default=0.08)
    p.add_argument("--slide-dot", type=float, default=0.002)
    p.add_argument("--r-left", type=float, default=0.4)
    p.add_argument("--r-right", type=float, default=-0.4)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--dls-damping", type=float, default=1e-3)
    p.add_argument("--swept-subdivisions", type=int, default=5)
    p.add_argument("--governor", action="store_true")
    p.add_argument("--min-progress-scale", type=float, default=0.125)
    p.add_argument("--scene-name", default="unspecified")
    p.add_argument(
        "--methods", default=None,
        help="comma-separated subset of methods to run (default: all)",
    )
    p.add_argument(
        "--trial-indices", default=None,
        help="comma-separated trial indices to run after generating states",
    )
    p.add_argument(
        "--trace-stops", action="store_true",
        help="retain last-state diagnostics for filtered stop cases",
    )
    p.add_argument(
        "--trace-all-steps", action="store_true",
        help="retain per-step clearance diagnostics for filtered/full-QP methods",
    )
    return p.parse_args()


def dls_step(fk, x, q, xdot, r_nom, previous_n, feedback, damping):
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_n)
    rhs = -ax @ xdot - feedback * e
    gram = aq @ aq.T + (damping ** 2) * np.eye(aq.shape[0])
    horizontal = aq.T @ np.linalg.solve(gram, rhs)
    qdot = horizontal + n @ r_nom
    return qdot, n, float(singular[-1]), float(np.max(np.abs(e)))


def pointwise_project(fk, x_next, q_pred, lower, upper, regularization=0.05):
    def objective(q):
        e = fk.error(x_next, q)
        return np.r_[10.0 * e[:3], e[3:], regularization * (q - q_pred)]

    result = least_squares(
        objective, np.clip(q_pred, lower, upper), bounds=(lower, upper),
        max_nfev=180, ftol=1e-10, xtol=1e-10, gtol=1e-10
    )
    q = result.x
    return q, float(np.max(np.abs(fk.error(x_next, q))))


def full_joint_qp_step(
    fk, checker, x, q, xdot_nom, r_nom, previous_n, lower, upper, dt,
    beta_stop=0.125, qdot_max=1.5, d_safe=0.005,
    clearance_margin_m=0.0,
):
    """Solve the same local linearized safety problem in ``(qdot,beta)``.

    This is the strong representation baseline for the reduced action theorem.
    It uses the same analytic Jacobian, clearance gradient, beta bounds, and
    independent endpoint checker as the reduced connector, but leaves all 14
    joint velocities as optimization variables.
    """
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_n)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    he = pinv @ (-2.0 * e)
    hx = pinv @ (-ax @ xdot_nom)
    qdot_nom = he + hx + n @ r_nom
    d0, contact0 = checker.check(x, q)
    if contact0 or not np.isfinite(d0):
        return np.zeros(14), n, {
            "feasible": False,
            "qp_feasible": False,
            "d0": float(d0),
            "stop_reason": "invalid_initial_clearance",
            "beta": 0.0,
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": d_safe + float(clearance_margin_m),
            "clearance_margin_m": float(clearance_margin_m),
        }, float(singular[-1]), 0.0
    grad = clearance_gradient(checker, x, q)
    grad_x = np.zeros(2, dtype=np.float64)
    for k, eps in enumerate((1e-5, 1e-6)):
        xp, xm = x.copy(), x.copy()
        xp[k] += eps; xm[k] -= eps
        grad_x[k] = (checker.check(xp, q)[0] - checker.check(xm, q)[0]) / (2.0 * eps)
    # Cq*qdot + Cbeta*beta <= b.
    Cq = []
    Cb = []
    b = []
    for j in range(14):
        row = np.zeros(14); row[j] = 1.0
        Cq.append(row); Cb.append(0.0); b.append(qdot_max)
        Cq.append(-row); Cb.append(0.0); b.append(qdot_max)
        Cq.append(dt * row); Cb.append(0.0); b.append(upper[j] - q[j])
        Cq.append(-dt * row); Cb.append(0.0); b.append(q[j] - lower[j])
    linear_required_clearance = d_safe + float(clearance_margin_m)
    Cq.append(-dt * grad); Cb.append(-dt * float(grad_x @ xdot_nom)); b.append(d0 - linear_required_clearance)
    Cq = np.asarray(Cq); Cb = np.asarray(Cb); b = np.asarray(b)
    C = np.column_stack([Cq, Cb])

    def objective(z):
        dq = z[:14] - qdot_nom
        return 0.5 * float(dq @ dq) + 10.0 * (z[14] - 1.0) ** 2

    def gradient_obj(z):
        g = np.zeros(15)
        g[:14] = z[:14] - qdot_nom
        g[14] = 20.0 * (z[14] - 1.0)
        return g

    equality_matrix = np.column_stack([aq, ax @ xdot_nom])

    result = minimize(
        objective, np.r_[qdot_nom, 1.0], jac=gradient_obj, method="SLSQP",
        bounds=[(-qdot_max, qdot_max)] * 14 + [(beta_stop, 1.0)],
        constraints=[
            LinearConstraint(equality_matrix, -2.0 * e, -2.0 * e),
            LinearConstraint(C, -np.inf, b),
        ],
        options={"ftol": 1e-10, "maxiter": 250},
    )
    if not result.success or np.max(np.abs(equality_matrix @ result.x + 2.0 * e)) > 5e-6:
        return np.zeros(14), n, {
            "feasible": False,
            "qp_feasible": False,
            "d0": float(d0),
            "stop_reason": "full_qp_infeasible",
            "beta": 0.0,
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": linear_required_clearance,
            "clearance_margin_m": float(clearance_margin_m),
            "optimizer_success": bool(result.success),
        }, float(singular[-1]), 0.0
    qdot = result.x[:14]
    beta = float(result.x[14])
    x_next = x + dt * beta * xdot_nom
    q_next = q + dt * qdot
    d_next, contact_next = checker.check(x_next, q_next)
    swept_min, swept_contact = swept_clearance(checker, x, q, x_next, q_next)
    valid = (
        d_next >= d_safe - 1e-9 and swept_min >= d_safe - 1e-9
        and not contact_next and not swept_contact
        and np.all(q_next >= lower - 1e-9) and np.all(q_next <= upper + 1e-9)
    )
    d_pred = float(d0 + dt * (grad @ qdot + float(grad_x @ (beta * xdot_nom))))
    limit_margin = float(min(np.min(q_next - lower), np.min(upper - q_next)))
    info = {
        "feasible": bool(valid),
        "qp_feasible": True,
        "d0": float(d0),
        "beta": beta,
        "governed": bool(beta < 1.0 - 1e-10),
        "r": ((n.T @ (qdot - he - beta * hx))).tolist(),
        "linear_predicted_clearance_m": d_pred,
        "linear_required_clearance_m": linear_required_clearance,
        "clearance_margin_m": float(clearance_margin_m),
        "d_next": float(d_next),
        "swept_min": float(swept_min),
        "joint_limit_margin_rad": limit_margin,
        "qdot_max_rad_s": float(np.max(np.abs(qdot))),
        "x_next_candidate": x_next.tolist(),
        "q_next_candidate": q_next.tolist(),
        "endpoint_contact": bool(contact_next),
        "swept_contact": bool(swept_contact),
        "stop_reason": "none" if valid else "full_qp_endpoint_validation",
    }
    return qdot if valid else np.zeros(14), n, info, float(singular[-1]), beta


def check_step(checker, x, q, x_next, q_next, lower, upper, subdivisions):
    d_next, contact = checker.check(x_next, q_next)
    swept_min, swept_contact = swept_clearance(
        checker, x, q, x_next, q_next, subdivisions=subdivisions
    )
    limit = bool(np.any(q_next < lower) or np.any(q_next > upper))
    return float(d_next), float(swept_min), bool(contact or swept_contact), limit


def metrics_row(method, x0, q0, args, fk, checker, lower, upper):
    x, q = x0.copy(), q0.copy()
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=np.float64)
    r_nom = np.array([args.r_left, args.r_right], dtype=np.float64)
    previous_n = None
    previous_desc = swivel_descriptor(fk, x, q)
    min_clearance = float(checker.check(x, q)[0])
    min_swept = min_clearance
    max_error = float(np.max(np.abs(fk.error(x, q))))
    max_speed = 0.0
    descriptor_jumps = 0
    max_descriptor_step = 0.0
    collision = 0
    limits = 0
    projections = 0
    rejected = 0
    governed_steps = 0
    governor_scale_min = 1.0
    beta_values = []
    completed = 0
    stopped = False
    sigma_min = np.inf
    stop_reason = "complete"
    step_latencies_ms = []
    stop_trace = None
    step_trace = []

    def capture_stop(step, reason, info, sigma, x_before, q_before,
                     x_candidate=None, q_candidate=None,
                     endpoint_clearance=None, swept_clearance_value=None,
                     limit_margin=None, qp_feasible=False):
        if not args.trace_stops:
            return None
        target_delta = xdot * args.dt * args.steps
        progress_vec = np.divide(
            x_before - x0, target_delta,
            out=np.zeros_like(x_before),
            where=np.abs(target_delta) > 1e-12,
        )
        closure = None
        if x_candidate is not None and q_candidate is not None:
            closure = float(np.max(np.abs(fk.error(x_candidate, q_candidate))))
        return {
            "stop_step": int(step),
            "stop_reason_raw": str(reason),
            "task_progress_fraction_before_stop": float(np.min(progress_vec)),
            "task_progress_vector_before_stop": progress_vec.tolist(),
            "beta": float(info["beta"]) if info.get("beta") is not None else None,
            "beta_min_before_stop": float(min(beta_values)) if beta_values else None,
            "beta_time_below_one_before_stop": (
                float(np.mean(np.asarray(beta_values) < 1.0 - 1e-10))
                if beta_values else None
            ),
            "qp_feasible": bool(qp_feasible),
            "linear_predicted_clearance_m": info.get("linear_predicted_clearance_m"),
            "linear_required_clearance_m": info.get("linear_required_clearance_m", 0.005 + args.clearance_margin_m),
            "clearance_margin_m": info.get("clearance_margin_m", args.clearance_margin_m),
            "endpoint_clearance_m": (
                None if endpoint_clearance is None else float(endpoint_clearance)
            ),
            "swept_clearance_m": (
                None if swept_clearance_value is None else float(swept_clearance_value)
            ),
            "joint_limit_margin_rad": (
                None if limit_margin is None else float(limit_margin)
            ),
            "closure_residual_inf": closure,
            "sigma_min_Aq": float(sigma),
            "qdot_max_rad_s": info.get("qdot_max_rad_s"),
            "endpoint_contact": bool(info.get("endpoint_contact", False)),
            "swept_contact": bool(info.get("swept_contact", False)),
            "x_before": np.asarray(x_before).tolist(),
            "q_before": np.asarray(q_before).tolist(),
            "x_candidate": (
                None if x_candidate is None else np.asarray(x_candidate).tolist()
            ),
            "q_candidate": (
                None if q_candidate is None else np.asarray(q_candidate).tolist()
            ),
            "trace_version": "n4-v1" if args.clearance_margin_m > 0.0 else "n2-v1",
        }

    def record_step(step, info, sigma, x_before, q_before, x_after, q_after,
                    endpoint_clearance, swept_clearance_value, limit_margin,
                    accepted, stop_reason_step="none"):
        if not args.trace_all_steps or method not in ("lift_filtered", "full_joint_qp"):
            return
        target_delta = xdot * args.dt * args.steps
        progress_before = np.divide(
            x_before - x0, target_delta,
            out=np.zeros_like(x_before),
            where=np.abs(target_delta) > 1e-12,
        )
        progress_after = np.divide(
            np.asarray(x_after) - x0, target_delta,
            out=np.zeros_like(x_before),
            where=np.abs(target_delta) > 1e-12,
        )
        predicted = info.get("linear_predicted_clearance_m")
        actual = None
        delta_d = None
        if endpoint_clearance is not None and swept_clearance_value is not None:
            actual = float(min(endpoint_clearance, swept_clearance_value))
            if predicted is not None:
                delta_d = float(actual - predicted)
        step_trace.append({
            "step": int(step),
            "accepted": bool(accepted),
            "stop_reason": str(stop_reason_step),
            "d0_m": info.get("d0"),
            "d_linear_m": predicted,
            "linear_required_clearance_m": info.get("linear_required_clearance_m", 0.005 + args.clearance_margin_m),
            "clearance_margin_m": info.get("clearance_margin_m", args.clearance_margin_m),
            "predicted_margin_m": None if predicted is None else float(predicted - 0.005),
            "predicted_excess_after_margin_m": (
                None if predicted is None else float(predicted - info.get("linear_required_clearance_m", 0.005 + args.clearance_margin_m))
            ),
            "d_actual_m": actual,
            "d_endpoint_m": None if endpoint_clearance is None else float(endpoint_clearance),
            "d_swept_m": None if swept_clearance_value is None else float(swept_clearance_value),
            "delta_d_m": delta_d,
            "beta": info.get("beta"),
            "task_progress_before": float(np.min(progress_before)),
            "task_progress_after": float(np.min(progress_after)),
            "sigma_min_Aq": float(sigma),
            "closure_residual_inf": (
                float(np.max(np.abs(fk.error(x_after, q_after))))
                if x_after is not None and q_after is not None else None
            ),
            "joint_limit_margin_rad": (
                None if limit_margin is None else float(limit_margin)
            ),
            "qp_feasible": info.get("qp_feasible"),
            "endpoint_pass": bool(endpoint_clearance is not None and endpoint_clearance >= 0.005 - 1e-9),
            "swept_pass": bool(swept_clearance_value is not None and swept_clearance_value >= 0.005 - 1e-9),
            "endpoint_contact": bool(info.get("endpoint_contact", False)),
            "swept_contact": bool(info.get("swept_contact", False)),
            "trace_version": "n4-v1" if args.clearance_margin_m > 0.0 else "n3-v1",
        })

    for _step in range(args.steps):
        step_start = time.perf_counter()
        if method == "lift_filtered":
            from tools.validate_branch_connector import connector_step_y
            beta_floor = args.min_progress_scale if args.governor else 1.0
            qdot, n, info, sigma, beta_used, r_used = connector_step_y(
                fk, checker, x, q, xdot, r_nom, previous_n,
                lower, upper, args.dt, beta_stop=beta_floor,
                clearance_margin_m=args.clearance_margin_m,
            )
            xdot_used = beta_used * xdot
            beta_logged = float(info.get("beta", beta_used))
            beta_values.append(beta_logged)
            if info.get("governed", False):
                governed_steps += 1
                governor_scale_min = min(governor_scale_min, beta_used)
            sigma_min = min(sigma_min, sigma)
            if not info["feasible"]:
                rejected += 1
                stopped = True
                stop_reason = info.get("stop_reason", "no_feasible_candidate")
                stop_trace = capture_stop(
                    _step, stop_reason, info, sigma, x, q,
                    info.get("x_next_candidate"), info.get("q_next_candidate"),
                    info.get("d_next"), info.get("swept_min"),
                    info.get("joint_limit_margin_rad"),
                    info.get("qp_feasible", False),
                )
                x_candidate = info.get("x_next_candidate")
                q_candidate = info.get("q_next_candidate")
                record_step(
                    _step, info, sigma, x, q,
                    x_candidate if x_candidate is not None else x,
                    q_candidate if q_candidate is not None else q,
                    info.get("d_next"), info.get("swept_min"),
                    info.get("joint_limit_margin_rad"), False, stop_reason,
                )
                elapsed_ms = (time.perf_counter() - step_start) * 1e3
                step_latencies_ms.append(elapsed_ms)
                if step_trace and step_trace[-1].get("step") == _step:
                    step_trace[-1]["step_latency_ms"] = float(elapsed_ms)
                break
            r_used = np.asarray(r_used, dtype=np.float64)
            x_mid = x + 0.5 * args.dt * xdot_used
            q_mid = q + 0.5 * args.dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot_used, r_used, n,
                args.feedback, 0.0
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + args.dt * xdot_used
            q_next = q + args.dt * qdot_mid
        elif method == "full_joint_qp":
            beta_floor = args.min_progress_scale if args.governor else 1.0
            qdot, n, info, sigma, beta_used = full_joint_qp_step(
                fk, checker, x, q, xdot, r_nom, previous_n,
                lower, upper, args.dt, beta_stop=beta_floor,
                clearance_margin_m=args.clearance_margin_m,
            )
            sigma_min = min(sigma_min, sigma)
            xdot_used = beta_used * xdot
            if info.get("governed", False):
                governed_steps += 1
                governor_scale_min = min(governor_scale_min, beta_used)
            beta_logged = float(info.get("beta", beta_used))
            beta_values.append(beta_logged)
            if not info["feasible"]:
                rejected += 1
                stopped = True
                stop_reason = info.get("stop_reason", "full_qp_infeasible")
                stop_trace = capture_stop(
                    _step, stop_reason, info, sigma, x, q,
                    info.get("x_next_candidate"), info.get("q_next_candidate"),
                    info.get("d_next"), info.get("swept_min"),
                    info.get("joint_limit_margin_rad"),
                    info.get("qp_feasible", False),
                )
                x_candidate = info.get("x_next_candidate")
                q_candidate = info.get("q_next_candidate")
                record_step(
                    _step, info, sigma, x, q,
                    x_candidate if x_candidate is not None else x,
                    q_candidate if q_candidate is not None else q,
                    info.get("d_next"), info.get("swept_min"),
                    info.get("joint_limit_margin_rad"), False, stop_reason,
                )
                elapsed_ms = (time.perf_counter() - step_start) * 1e3
                step_latencies_ms.append(elapsed_ms)
                if step_trace and step_trace[-1].get("step") == _step:
                    step_trace[-1]["step_latency_ms"] = float(elapsed_ms)
                break
            # Recover the null-space action selected by the full QP so the
            # midpoint evaluator integrates the same filtered velocity.  A
            # nominal ``r_nom`` here would silently mix two different
            # trajectories and invalidate the full/reduced comparison.
            r_used = np.asarray(info.get("r", r_nom), dtype=np.float64)
            x_mid = x + 0.5 * args.dt * xdot_used
            q_mid = q + 0.5 * args.dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot_used, r_used, n,
                args.feedback, 0.0
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + args.dt * xdot_used
            q_next = q + args.dt * qdot_mid
        elif method == "lift_unfiltered":
            qdot, n, sigma, _ = dls_step(
                fk, x, q, xdot, r_nom, previous_n,
                args.feedback, 0.0
            )
            sigma_min = min(sigma_min, sigma)
            x_mid = x + 0.5 * args.dt * xdot
            q_mid = q + 0.5 * args.dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot, r_nom, n,
                args.feedback, 0.0
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + args.dt * xdot
            q_next = q + args.dt * qdot_mid
        elif method == "dls":
            qdot, n, sigma, _ = dls_step(
                fk, x, q, xdot, r_nom, previous_n,
                args.feedback, args.dls_damping
            )
            sigma_min = min(sigma_min, sigma)
            x_mid = x + 0.5 * args.dt * xdot
            q_mid = q + 0.5 * args.dt * qdot
            qdot_mid, n_mid, sigma_mid, _ = dls_step(
                fk, x_mid, q_mid, xdot, r_nom, n,
                args.feedback, args.dls_damping
            )
            sigma_min = min(sigma_min, sigma_mid)
            x_next = x + args.dt * xdot
            q_next = q + args.dt * qdot_mid
        elif method == "pointwise_projection":
            q_task, n, sigma, _ = dls_step(
                fk, x, q, xdot, r_nom, previous_n,
                args.feedback, args.dls_damping
            )
            sigma_min = min(sigma_min, sigma)
            x_next = x + args.dt * xdot
            q_pred = q + args.dt * q_task
            q_next, projection_error = pointwise_project(
                fk, x_next, q_pred, lower, upper
            )
            projections += 1
            qdot_mid = (q_next - q) / args.dt
            n_mid = n
            if projection_error > 2e-5:
                stopped = True
                stop_reason = "projection_residual"
                elapsed_ms = (time.perf_counter() - step_start) * 1e3
                step_latencies_ms.append(elapsed_ms)
                if step_trace and step_trace[-1].get("step") == _step:
                    step_trace[-1]["step_latency_ms"] = float(elapsed_ms)
                break
        else:
            raise ValueError(method)

        d_next, swept_min, hit, limit = check_step(
            checker, x, q, x_next, q_next, lower, upper,
            args.swept_subdivisions
        )
        # connector_step certifies an Euler candidate.  The executed lift
        # uses the midpoint velocity above, so a filtered trial must recheck
        # that actual endpoint before accepting the step.  Otherwise the
        # filter and integrator would certify different trajectories.
        if method in ("lift_filtered", "full_joint_qp") and (hit or limit):
            rejected += 1
            stopped = True
            stop_reason = "post_integrator_check"
            endpoint_hit = bool(checker.check(x_next, q_next)[1])
            swept_hit = bool(swept_clearance(
                checker, x, q, x_next, q_next,
                subdivisions=args.swept_subdivisions,
            )[1])
            stop_info = dict(info)
            stop_info.update({
                "qdot_max_rad_s": float(np.max(np.abs(qdot_mid))),
                "endpoint_contact": endpoint_hit,
                "swept_contact": swept_hit,
            })
            limit_margin = float(min(np.min(q_next - lower), np.min(upper - q_next)))
            stop_trace = capture_stop(
                _step, stop_reason, stop_info, sigma, x, q, x_next, q_next,
                d_next, swept_min, limit_margin, info.get("qp_feasible", True),
            )
            record_step(
                _step, stop_info, sigma, x, q, x_next, q_next,
                d_next, swept_min, limit_margin, False, stop_reason,
            )
            elapsed_ms = (time.perf_counter() - step_start) * 1e3
            step_latencies_ms.append(elapsed_ms)
            if step_trace and step_trace[-1].get("step") == _step:
                step_trace[-1]["step_latency_ms"] = float(elapsed_ms)
            break
        if method in ("lift_filtered", "full_joint_qp"):
            limit_margin_step = float(min(np.min(q_next - lower), np.min(upper - q_next)))
            record_step(
                _step, info, sigma, x, q, x_next, q_next,
                d_next, swept_min, limit_margin_step, True, "none",
            )
        if hit:
            collision += 1
        if limit:
            limits += 1
        min_clearance = min(min_clearance, d_next)
        min_swept = min(min_swept, swept_min)
        max_speed = max(max_speed, float(np.max(np.abs(qdot_mid))))
        max_error = max(max_error, float(np.max(np.abs(fk.error(x_next, q_next)))))
        desc = swivel_descriptor(fk, x_next, q_next)
        delta = (desc - previous_desc + np.pi) % (2.0 * np.pi) - np.pi
        max_descriptor_step = max(max_descriptor_step, float(np.max(np.abs(delta))))
        descriptor_jumps += int(np.any(np.abs(delta) > 0.5))
        previous_desc = desc
        x, q, previous_n = x_next, q_next, n_mid
        completed += 1
        elapsed_ms = (time.perf_counter() - step_start) * 1e3
        step_latencies_ms.append(elapsed_ms)
        if step_trace and step_trace[-1].get("step") == _step:
            step_trace[-1]["step_latency_ms"] = float(elapsed_ms)

    if not stopped and (collision > 0 or limits > 0):
        stop_reason = "complete_with_violations"
    target_delta = xdot * args.dt * args.steps
    actual_delta = x - x0
    progress = np.divide(
        actual_delta, target_delta,
        out=np.zeros_like(actual_delta),
        where=np.abs(target_delta) > 1e-12,
    )
    latency = np.asarray(step_latencies_ms, dtype=np.float64)

    return {
        "method": method,
        "steps": completed,
        "complete": completed == args.steps,
        "stopped": stopped,
        "stop_reason": stop_reason,
        "rejected_steps": rejected,
        "governed_steps": governed_steps,
        "governor_scale_min": float(governor_scale_min),
        "beta_min": float(min(beta_values)) if beta_values else 0.0,
        "beta_mean": float(np.mean(beta_values)) if beta_values else 0.0,
        "beta_time_below_one": float(np.mean(np.asarray(beta_values) < 1.0 - 1e-10)) if beta_values else 0.0,
        "projection_calls": projections,
        "min_clearance_m": float(min_clearance),
        "min_swept_clearance_m": float(min_swept),
        "max_qdot_rad_s": float(max_speed),
        "max_closure_error_mixed_m_rad": float(max_error),
        "descriptor_jumps": descriptor_jumps,
        "max_descriptor_step_rad": float(max_descriptor_step),
        "collision_samples": collision,
        "joint_limit_samples": limits,
        "sigma_min_Aq": float(sigma_min),
        "task_delta": actual_delta.tolist(),
        "task_progress_fraction": float(np.min(progress)) if len(progress) else 0.0,
        "mean_step_latency_ms": float(np.mean(latency)) if len(latency) else 0.0,
        "p95_step_latency_ms": float(np.quantile(latency, 0.95)) if len(latency) else 0.0,
        "p99_step_latency_ms": float(np.quantile(latency, 0.99)) if len(latency) else 0.0,
        "wall_time_s": float(np.sum(latency) / 1e3),
        "stop_trace": stop_trace,
        "step_trace": step_trace if args.trace_all_steps else None,
    }


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    checker = CollisionChecker(args.model)
    lower = np.concatenate([lk.joint_lower[0], lk.joint_lower[1]])
    upper = np.concatenate([lk.joint_upper[0], lk.joint_upper[1]])
    rng = np.random.default_rng(args.seed)
    states = []
    for _ in range(args.trials):
        for _attempt in range(1000):
            x, _phi, lookup = sample_state(lk, rng)
            q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
            d, contact = checker.check(x, q)
            if d >= 0.007 and not contact:
                states.append((x, q))
                break
    if len(states) != args.trials:
        raise RuntimeError(
            f"could not sample {args.trials} valid initial states; got {len(states)}"
        )
    all_methods = [
        "lift_filtered", "full_joint_qp", "lift_unfiltered", "dls",
        "pointwise_projection",
    ]
    methods = (
        [m.strip() for m in args.methods.split(",") if m.strip()]
        if args.methods else all_methods
    )
    unknown = sorted(set(methods) - set(all_methods))
    if unknown:
        raise ValueError(f"unknown methods: {unknown}")
    selected_trials = (
        sorted({int(v) for v in args.trial_indices.split(",") if v.strip()})
        if args.trial_indices else list(range(len(states)))
    )
    if any(i < 0 or i >= len(states) for i in selected_trials):
        raise ValueError(f"trial index outside [0,{len(states)-1}]: {selected_trials}")
    rows = []
    for trial in selected_trials:
        x, q = states[trial]
        for method in methods:
            row = metrics_row(method, x, q, args, fk, checker, lower, upper)
            row["trial"] = trial
            rows.append(row)

    def aggregate(method):
        rs = [r for r in rows if r["method"] == method]
        def stat(key):
            a = np.asarray([r[key] for r in rs], dtype=np.float64)
            return {"median": float(np.median(a)), "p95": float(np.quantile(a, .95)),
                    "max": float(np.max(a))}
        return {
            "trials": len(rs),
            "complete": int(sum(r["complete"] for r in rs)),
            "mean_steps": float(np.mean([r["steps"] for r in rs])),
            "min_clearance_m": stat("min_clearance_m"),
            "min_swept_clearance_m": stat("min_swept_clearance_m"),
            "max_qdot_rad_s": stat("max_qdot_rad_s"),
            "max_closure_error_mixed_m_rad": stat("max_closure_error_mixed_m_rad"),
            "descriptor_jumps": int(sum(r["descriptor_jumps"] for r in rs)),
            "collision_samples": int(sum(r["collision_samples"] for r in rs)),
            "joint_limit_samples": int(sum(r["joint_limit_samples"] for r in rs)),
            "rejected_steps": int(sum(r["rejected_steps"] for r in rs)),
            "projection_calls": int(sum(r["projection_calls"] for r in rs)),
            "governed_steps": int(sum(r["governed_steps"] for r in rs)),
            "governor_scale_min": float(min(r["governor_scale_min"] for r in rs)),
            "beta_min": stat("beta_min"),
            "beta_mean": stat("beta_mean"),
            "beta_time_below_one": stat("beta_time_below_one"),
            "stop_reasons": {
                reason: int(sum(r["stop_reason"] == reason for r in rs))
                for reason in sorted({r["stop_reason"] for r in rs})
            },
            "task_progress_fraction": stat("task_progress_fraction"),
            "mean_step_latency_ms": stat("mean_step_latency_ms"),
            "p95_step_latency_ms": stat("p95_step_latency_ms"),
            "p99_step_latency_ms": stat("p99_step_latency_ms"),
            "wall_time_s": stat("wall_time_s"),
        }

    report = {
        "kind": "paired_differential_lift_baseline_experiment",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "trials": len(states),
        "steps_per_trial": args.steps,
        "dt": args.dt,
        "clearance_margin_m": args.clearance_margin_m,
        "seed": args.seed,
        "task_velocity": [args.theta_dot, args.slide_dot],
        "nominal_null_rate": [args.r_left, args.r_right],
        "phase_governor": bool(args.governor),
        "min_progress_scale": args.min_progress_scale,
        "scene_name": args.scene_name,
        "methods": methods,
        "selected_trial_indices": selected_trials,
        "trace_stops": bool(args.trace_stops),
        "aggregates": {m: aggregate(m) for m in methods},
        "trial_rows": rows,
        "interpretation": [
            "All methods replay the same sampled initial states and task commands.",
            "lift_filtered and full_joint_qp use the same linearized safety rows and independent endpoint checker.",
            "This is a protocol smoke test, not a statistically powered final comparison.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["aggregates"], indent=2))


if __name__ == "__main__":
    main()

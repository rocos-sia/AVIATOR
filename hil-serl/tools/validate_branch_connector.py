#!/usr/bin/env python3
"""Short branch-connector comparison with an N r phase filter.

The proposed connector uses analytic MuJoCo site Jacobians, the minimum-norm
horizontal lift, an oriented two-dimensional null basis, and linearized
joint-rate/joint-limit/clearance constraints in r.  Every accepted step is
then checked by the independent point-cloud/box clearance checker used by the
existing audit tool.  The unfiltered connector is replayed from the same
initial state and nominal r command for a diagnostic comparison.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from examples.experiments.aviator_manifold.safety_filter import (
    _clip_polygon,
    _project_onto_polygon,
    project_reduced_action,
)
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--trials", type=int, default=6)
    p.add_argument("--steps", type=int, default=120)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=21)
    p.add_argument("--theta-dot", type=float, default=0.16)
    p.add_argument("--slide-dot", type=float, default=0.004)
    p.add_argument("--r-left", type=float, default=1.8)
    p.add_argument("--r-right", type=float, default=-1.8)
    p.add_argument("--governor", action="store_true",
                   help="reduce task/null rates when the filtered step is infeasible")
    p.add_argument("--min-progress-scale", type=float, default=0.125,
                   help="minimum task/null-rate scale accepted by the governor")
    return p.parse_args()


def add_plane(planes, a, b):
    planes.append((np.asarray(a, dtype=np.float64), float(b)))


def clearance_gradient(checker, x, q):
    grad = np.zeros(14, dtype=np.float64)
    for j in range(14):
        h = 1e-5
        qp, qm = q.copy(), q.copy()
        qp[j] += h
        qm[j] -= h
        grad[j] = (checker.check(x, qp)[0] - checker.check(x, qm)[0]) / (2.0 * h)
    return grad


def swept_clearance(checker, x0, q0, x1, q1, subdivisions=5):
    """Sample the finite-step swept segment with the independent checker."""
    min_clearance = float("inf")
    contact = False
    for alpha in np.linspace(0.0, 1.0, subdivisions + 1):
        x = (1.0 - alpha) * x0 + alpha * x1
        q = (1.0 - alpha) * q0 + alpha * q1
        clearance, hit = checker.check(x, q)
        min_clearance = min(min_clearance, float(clearance))
        contact = contact or bool(hit)
    return min_clearance, contact


def project_r(
    checker,
    x,
    x_next,
    q,
    h,
    n,
    r_nom,
    lower,
    upper,
    dt,
    qdot_max=1.5,
    d_safe=0.005,
    r_max=3.0,
):
    """Project r_nom into the linearized N r safety polygon and recheck it."""
    planes = []
    for j in range(14):
        nj = n[j]
        add_plane(planes, nj, qdot_max - h[j])
        add_plane(planes, -nj, qdot_max + h[j])
        add_plane(planes, dt * nj, upper[j] - q[j] - dt * h[j])
        add_plane(planes, -dt * nj, q[j] - lower[j] + dt * h[j])

    d0, contact0 = checker.check(x, q)
    grad = clearance_gradient(checker, x, q)
    gn = grad @ n
    add_plane(planes, -dt * gn, d0 - d_safe + dt * float(grad @ h))

    poly = [
        np.array([-r_max, -r_max]),
        np.array([r_max, -r_max]),
        np.array([r_max, r_max]),
        np.array([-r_max, r_max]),
    ]
    for aa, bb in planes:
        poly = _clip_polygon(poly, aa, bb)
        if len(poly) == 0:
            return np.zeros(2), {
                "feasible": False,
                "intervened": True,
                "backtracked": 0,
                "r": [0.0, 0.0],
            }
    r = _project_onto_polygon(r_nom, poly)
    intervened = bool(np.linalg.norm(r - r_nom) > 1e-10)
    backtracked = 0
    for _ in range(6):
        q_next = q + dt * (h + n @ r)
        d_next, contact_next = checker.check(x_next, q_next)
        swept_min, swept_contact = swept_clearance(checker, x, q, x_next, q_next)
        qdot = (q_next - q) / dt
        if (
            np.max(np.abs(qdot)) <= qdot_max + 2e-5
            and np.all(q_next >= lower - 1e-9)
            and np.all(q_next <= upper + 1e-9)
            and d_next >= d_safe - 1e-9
            and swept_min >= d_safe - 1e-9
            and not contact_next
            and not swept_contact
        ):
            return r, {
                "feasible": True,
                "intervened": intervened,
                "backtracked": backtracked,
                "d_next": float(d_next),
                "swept_min": float(swept_min),
                "r": r.tolist(),
            }
        r *= 0.5
        backtracked += 1
        intervened = True
    return np.zeros(2), {
        "feasible": False,
        "intervened": True,
        "backtracked": backtracked,
        "swept_min": None,
        "r": [0.0, 0.0],
    }


def project_action_y(
    checker,
    x,
    q,
    h_e,
    h_x,
    n,
    r_nom,
    xdot_nom,
    lower,
    upper,
    dt,
    qdot_max=1.5,
    d_safe=0.005,
    r_max=3.0,
    beta_stop=0.125,
    clearance_margin_m=0.0,
):
    """Build and solve the unified ``y=[r_L,r_R,beta]`` safety QP.

    The rows are instantaneous joint-rate, one-step joint-limit, and local
    clearance half-spaces.  The independent checker validates the selected
    action at the beta-scaled task endpoint; it is not replaced by the linear
    filter.
    """
    planes = []
    for j in range(14):
        row = np.zeros(3)
        row[:2] = n[j]
        row[2] = h_x[j]
        add_plane(planes, row, qdot_max - h_e[j])
        add_plane(planes, -row, qdot_max + h_e[j])

        # q + dt*(h_e + beta*h_x + N*r) within joint limits.
        add_plane(planes, dt * row, upper[j] - q[j] - dt * h_e[j])
        add_plane(planes, -dt * row, q[j] - lower[j] + dt * h_e[j])

    d0, contact0 = checker.check(x, q)
    grad_x = np.zeros(2, dtype=np.float64)
    for k, eps in enumerate((1e-5, 1e-6)):
        xp, xm = np.asarray(x, dtype=np.float64).copy(), np.asarray(x, dtype=np.float64).copy()
        xp[k] += eps; xm[k] -= eps
        grad_x[k] = (checker.check(xp, q)[0] - checker.check(xm, q)[0]) / (2.0 * eps)
    grad = clearance_gradient(checker, x, q)
    row = np.zeros(3)
    row[:2] = -dt * (grad @ n)
    row[2] = -dt * float(grad @ h_x + grad_x @ xdot_nom)
    linear_required_clearance = d_safe + float(clearance_margin_m)
    add_plane(planes, row, d0 - linear_required_clearance + dt * float(grad @ h_e))

    if not np.isfinite(d0) or contact0:
        return np.zeros(3), {
            "feasible": False,
            "qp_feasible": False,
            "intervened": True,
            "beta": 0.0,
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": linear_required_clearance,
            "clearance_margin_m": float(clearance_margin_m),
            "stop_reason": "invalid_initial_clearance",
        }

    A = np.asarray([a for a, _ in planes], dtype=np.float64)
    b = np.asarray([bb for _, bb in planes], dtype=np.float64)
    y_nom = np.array([float(r_nom[0]), float(r_nom[1]), 1.0])
    # Match the full joint-space objective: ||qdot-qdot_nom||^2 with
    # qdot = h_e + [N,h_x] y, plus a penalty on beta deviation.
    lift_matrix = np.column_stack([n, h_x])
    action_metric = lift_matrix.T @ lift_matrix
    action_metric[2, 2] += 20.0
    y, info = project_reduced_action(
        y_nom, A, b, r_max=r_max, beta_lower=beta_stop, beta_upper=1.0,
        metric=action_metric,
    )
    if not info["feasible"]:
        info.update({
            "r": [0.0, 0.0], "beta": 0.0,
            "qp_feasible": False,
            "linear_predicted_clearance_m": None,
            "linear_required_clearance_m": linear_required_clearance,
            "clearance_margin_m": float(clearance_margin_m),
            "stop_reason": "no_feasible_y",
        })
        return np.zeros(3), info

    r = y[:2]
    beta = float(y[2])
    qdot = h_e + beta * h_x + n @ r
    x_next = x + dt * beta * xdot_nom
    q_next = q + dt * qdot
    d_next, contact_next = checker.check(x_next, q_next)
    swept_min, swept_contact = swept_clearance(checker, x, q, x_next, q_next)
    limit = bool(np.any(q_next < lower) or np.any(q_next > upper))
    d_pred = float(
        d0 + dt * (grad @ qdot + float(grad_x @ (beta * xdot_nom)))
    )
    limit_margin = float(min(np.min(q_next - lower), np.min(upper - q_next)))
    valid = (
        np.max(np.abs(qdot)) <= qdot_max + 2e-5
        and not limit
        and d_next >= d_safe - 1e-9
        and swept_min >= d_safe - 1e-9
        and not contact_next
        and not swept_contact
    )
    if not valid:
        info.update({
            "feasible": False,
            "qp_feasible": True,
            "d0": float(d0),
            "stop_reason": "endpoint_validation",
            "r": r.tolist(),
            "beta": beta,
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
        })
        return np.zeros(3), info
    info.update({
        "r": r.tolist(),
        "beta": beta,
        "qp_feasible": True,
        "d0": float(d0),
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
        "governed": bool(beta < 1.0 - 1e-10),
        "stop_reason": "none",
    })
    return y, info


def swivel_descriptor(fk: AviatorClosure, x, q):
    """Return two signed local elbow-swivel angles, relative to a fixed world frame.

    The descriptor is used only for local continuity.  It is not a proof that
    two disconnected global IK components have different labels.
    """
    fk.data.qpos[:] = 0.0
    fk.data.qpos[:2] = x
    fk.data.qpos[2:16] = q
    import mujoco

    mujoco.mj_forward(fk.model, fk.data)
    vals = []
    for side, tcp in enumerate(fk.tcp):
        base = 3 if side == 0 else 13
        elbow = 7 if side == 0 else 17
        shoulder_p = fk.data.xpos[base].copy()
        elbow_p = fk.data.xpos[elbow].copy()
        wrist_p = fk.data.site_xpos[tcp].copy()
        axis = wrist_p - shoulder_p
        axis /= max(np.linalg.norm(axis), 1e-12)
        vec = elbow_p - shoulder_p
        vec -= axis * float(axis @ vec)
        vec /= max(np.linalg.norm(vec), 1e-12)
        ref = np.array([0.0, 0.0, 1.0])
        ref -= axis * float(axis @ ref)
        if np.linalg.norm(ref) < 1e-8:
            ref = np.array([0.0, 1.0, 0.0])
            ref -= axis * float(axis @ ref)
        ref /= np.linalg.norm(ref)
        ortho = np.cross(axis, ref)
        vals.append(float(np.arctan2(ortho @ vec, ref @ vec)))
    return np.asarray(vals)


def connector_step(fk, checker, x, q, xdot, r_nom, previous_n, filtered, lower, upper, dt):
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_n)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    h = pinv @ (-ax @ xdot - 2.0 * e)
    x_next = x + dt * xdot
    if filtered:
        r, info = project_r(checker, x, x_next, q, h, n, r_nom, lower, upper, dt)
    else:
        r, info = r_nom, {
            "feasible": True,
            "intervened": False,
            "backtracked": 0,
            "r": r_nom.tolist(),
        }
    qdot = h + n @ r
    return qdot, n, info, float(singular[-1])


def connector_step_y(
    fk, checker, x, q, xdot_nom, r_nom, previous_n, lower, upper, dt,
    beta_stop=0.125, clearance_margin_m=0.0,
):
    """Unified beta-aware connector step used by the revised formal matrix."""
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_n)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    h_e = pinv @ (-2.0 * e)
    h_x = pinv @ (-ax @ xdot_nom)
    y, info = project_action_y(
        checker, x, q, h_e, h_x, n, r_nom, xdot_nom,
        lower, upper, dt, beta_stop=beta_stop,
        clearance_margin_m=clearance_margin_m,
    )
    if not info["feasible"]:
        return np.zeros(14), n, info, float(singular[-1]), 0.0, np.zeros(2)
    r = y[:2]
    beta = float(y[2])
    qdot = h_e + beta * h_x + n @ r
    return qdot, n, info, float(singular[-1]), beta, r


def summarize(values):
    if not values:
        return {"n": 0, "median": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"n": int(a.size), "median": float(np.median(a)),
            "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}


def governor_candidates(xdot, r_nom, min_progress_scale):
    """Try task-speed and null-rate reductions in increasing intervention order."""
    pairs = [
        (1.0, 1.0),
        (0.75, 1.0), (0.5, 1.0), (0.25, 1.0), (0.125, 1.0),
        (1.0, 0.75), (1.0, 0.5), (1.0, 0.25), (1.0, 0.125),
        (0.75, 0.75), (0.5, 0.5), (0.25, 0.25), (0.125, 0.125),
    ]
    for sx, sr in pairs:
        if max(sx, sr) < min_progress_scale:
            continue
        yield sx, sr, sx * xdot, sr * r_nom


def run_method(fk, checker, lk, rng, args, filtered):
    lower = np.concatenate([lk.joint_lower[0], lk.joint_lower[1]])
    upper = np.concatenate([lk.joint_upper[0], lk.joint_upper[1]])
    trial_rows = []
    for trial in range(args.trials):
        for _ in range(1000):
            x, phi, lookup = sample_state(lk, rng)
            q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
            d0, c0 = checker.check(x, q)
            if d0 >= 0.007 and not c0:
                break
        xdot = np.array([args.theta_dot, args.slide_dot], dtype=np.float64)
        r_nom = np.array([args.r_left, args.r_right], dtype=np.float64)
        previous_n = None
        previous_desc = swivel_descriptor(fk, x, q)
        min_clearance = d0
        min_swept_clearance = d0
        max_speed = 0.0
        max_task_error = float(np.max(np.abs(fk.error(x, q))))
        descriptor_jumps = 0
        max_descriptor_step = 0.0
        rejected = 0
        governed_steps = 0
        governor_scale_min = 1.0
        stalled_steps = 0
        feasible_steps = 0
        collision = 0
        limit = 0
        stopped = False
        for _step in range(args.steps):
            chosen = None
            if filtered and args.governor:
                for sx, sr, xdot_try, r_try in governor_candidates(
                    xdot, r_nom, args.min_progress_scale
                ):
                    qdot_try, n_try, info_try, sigma_try = connector_step(
                        fk, checker, x, q, xdot_try, r_try, previous_n,
                        True, lower, upper, args.dt
                    )
                    if info_try["feasible"]:
                        chosen = (qdot_try, n_try, info_try, sigma_try,
                                  xdot_try, r_try, min(sx, sr))
                        break
                if chosen is None:
                    rejected += 1
                    stopped = True
                    break
                qdot, n, info, sigma, xdot_used, r_nom_used, governor_scale = chosen
                if governor_scale < 1.0 - 1e-12:
                    governed_steps += 1
                    governor_scale_min = min(governor_scale_min, governor_scale)
                if governor_scale <= 1e-12:
                    stalled_steps += 1
            else:
                xdot_used = xdot
                r_nom_used = r_nom
                qdot, n, info, sigma = connector_step(
                    fk, checker, x, q, xdot_used, r_nom_used, previous_n,
                    filtered, lower, upper, args.dt
                )
            if filtered and not info["feasible"]:
                rejected += 1
                stopped = True
                break
            x_next = x + args.dt * xdot_used
            # The finite-step filter and midpoint integrator must use the same
            # null-space command.  Otherwise the filter certifies one Euler
            # candidate while the integrator executes a different midpoint
            # candidate.  Backtrack the shared r when the actual midpoint
            # endpoint fails the independent checker.
            r_used = np.asarray(info.get("r", r_nom_used), dtype=np.float64)
            scales = [1.0, 0.5, 0.25, 0.125, 0.0625] if filtered else [1.0]
            accepted = False
            d_next = float("nan")
            contact_next = False
            swept_min = float("inf")
            swept_contact = False
            limit_now = False
            speed_now = float("inf")
            qdot_mid = qdot
            n_mid = n
            for scale in scales:
                r_try = r_used * scale
                qdot_mid, n_mid, _, _ = connector_step(
                    fk, checker, x + 0.5 * args.dt * xdot_used,
                    q + 0.5 * args.dt * qdot, xdot_used, r_try, n, False,
                    lower, upper, args.dt
                )
                q_next = q + args.dt * qdot_mid
                d_next, contact_next = checker.check(x_next, q_next)
                swept_min, swept_contact = swept_clearance(checker, x, q, x_next, q_next)
                limit_now = bool(np.any(q_next < lower) or np.any(q_next > upper))
                speed_now = float(np.max(np.abs(qdot_mid)))
                if (
                    not filtered
                    or (
                        d_next >= 0.005 - 1e-9
                        and not contact_next
                        and swept_min >= 0.005 - 1e-9
                        and not swept_contact
                        and not limit_now
                        and speed_now <= 1.5 + 2e-5
                    )
                ):
                    accepted = True
                    break
            if filtered and not accepted:
                rejected += 1
                stopped = True
                break
            if d_next < 0.005 - 1e-9 or swept_min < 0.005 - 1e-9:
                collision += 1
            if contact_next or swept_contact:
                collision += 1
            if limit_now:
                limit += 1
            desc = swivel_descriptor(fk, x_next, q_next)
            desc_delta = (desc - previous_desc + np.pi) % (2.0 * np.pi) - np.pi
            max_descriptor_step = max(max_descriptor_step, float(np.max(np.abs(desc_delta))))
            descriptor_jumps += int(np.any(np.abs(desc_delta) > 0.5))
            previous_desc = desc
            q = q_next
            x = x_next
            previous_n = n_mid
            feasible_steps += 1
            min_clearance = min(min_clearance, d_next)
            min_swept_clearance = min(min_swept_clearance, swept_min)
            max_speed = max(max_speed, float(np.max(np.abs(qdot_mid))))
            max_task_error = max(max_task_error, float(np.max(np.abs(fk.error(x, q)))))
        trial_rows.append({
            "steps": feasible_steps,
            "complete": feasible_steps == args.steps,
            "stopped": stopped,
            "rejected": rejected,
            "governed_steps": governed_steps,
            "governor_scale_min": float(governor_scale_min),
            "stalled_steps": stalled_steps,
            "min_clearance_m": float(min_clearance),
            "min_swept_clearance_m": float(min_swept_clearance),
            "max_qdot_rad_s": float(max_speed),
            "max_closure_error_mixed_m_rad": float(max_task_error),
            "descriptor_jumps": descriptor_jumps,
            "max_descriptor_step_rad": float(max_descriptor_step),
            "collision_samples": collision,
            "joint_limit_samples": limit,
        })
    return trial_rows


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    checker = CollisionChecker(args.model)
    rows_filtered = run_method(fk, checker, lk, np.random.default_rng(args.seed), args, True)
    rows_raw = run_method(fk, checker, lk, np.random.default_rng(args.seed), args, False)

    def aggregate(rows):
        return {
            "trials": len(rows),
            "complete": int(sum(r["complete"] for r in rows)),
            "mean_steps": float(np.mean([r["steps"] for r in rows])),
            "min_clearance_m": summarize([r["min_clearance_m"] for r in rows]),
            "min_swept_clearance_m": summarize([r["min_swept_clearance_m"] for r in rows]),
            "max_qdot_rad_s": summarize([r["max_qdot_rad_s"] for r in rows]),
            "max_closure_error_mixed_m_rad": summarize([r["max_closure_error_mixed_m_rad"] for r in rows]),
            "descriptor_jumps": int(sum(r["descriptor_jumps"] for r in rows)),
            "max_descriptor_step_rad": summarize([r["max_descriptor_step_rad"] for r in rows]),
            "collision_samples": int(sum(r["collision_samples"] for r in rows)),
            "joint_limit_samples": int(sum(r["joint_limit_samples"] for r in rows)),
            "rejected_steps": int(sum(r["rejected"] for r in rows)),
            "governed_steps": int(sum(r["governed_steps"] for r in rows)),
            "governor_scale_min": float(min(r["governor_scale_min"] for r in rows)),
            "stalled_steps": int(sum(r["stalled_steps"] for r in rows)),
        }

    report = {
        "kind": "branch_connector_comparison",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "steps_per_trial": args.steps,
        "dt": args.dt,
        "seed": args.seed,
        "task_velocity": [args.theta_dot, args.slide_dot],
        "nominal_null_rate": [args.r_left, args.r_right],
        "phase_governor": bool(args.governor),
        "min_progress_scale": args.min_progress_scale,
        "filtered_Nr": aggregate(rows_filtered),
        "unfiltered_Nr": aggregate(rows_raw),
        "trial_rows_filtered": rows_filtered,
        "trial_rows_unfiltered": rows_raw,
        "interpretation": [
            "The swivel descriptor is a local continuity diagnostic, not a global IK component label.",
            "Clearance uses the independent point-cloud/box checker; midpoint checks are not a continuous-time proof.",
            "The filtered method uses linearized N r constraints plus finite-step validation.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k.startswith("filtered") or k.startswith("unfiltered")}, indent=2))


if __name__ == "__main__":
    main()

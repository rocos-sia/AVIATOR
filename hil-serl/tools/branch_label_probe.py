#!/usr/bin/env python3
"""Static multi-start branch labels plus DLS/pointwise trajectory replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis
from tools.validate_global_branch_descriptor import (
    ArmIK,
    descriptor,
    graph_components,
    solve_arm,
)
from tools.audit_static_collision import CollisionChecker
from tools.validate_branch_connector import connector_step_y


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--figure")
    p.add_argument("--starts", type=int, default=100)
    p.add_argument("--steps", type=int, default=160)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--seed", type=int, default=20261004)
    p.add_argument("--theta-dot", type=float, default=0.16)
    p.add_argument("--slide-dot", type=float, default=0.004)
    p.add_argument("--r-left", type=float, default=0.8)
    p.add_argument("--r-right", type=float, default=-0.8)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--dls-damping", type=float, default=1e-3)
    p.add_argument("--dedup", type=float, default=0.02)
    p.add_argument("--residual-tol", type=float, default=2e-5)
    p.add_argument("--edge-max-distance", type=float, default=0.25)
    p.add_argument("--edge-subdivisions", type=int, default=9)
    p.add_argument("--edge-residual-tol", type=float, default=5e-4)
    p.add_argument("--continuation-subdivisions", type=int, default=12)
    p.add_argument("--continuation-regularization", type=float, default=0.05)
    p.add_argument("--pointwise-seed-mode", choices=("warm_start", "anchor", "random"), default="warm_start")
    p.add_argument("--label-margin", type=float, default=0.05)
    p.add_argument("--label-update-interval", type=int, default=4)
    p.add_argument("--trajectory-solver-nfev", type=int, default=80)
    p.add_argument("--x-theta", type=float)
    p.add_argument("--x-slide", type=float)
    p.add_argument("--initial-component-left", type=int, default=0,
                   help="continuation component used as the initial left-arm IK state")
    p.add_argument("--initial-component-right", type=int, default=0,
                   help="continuation component used as the initial right-arm IK state")
    p.add_argument("--preserve-left-lookup", action="store_true",
                   help="keep the collision-checked LUT left-arm seed instead of replacing it")
    return p.parse_args()


def solve_from_seed(arm, x, seed, lower, upper, residual_tol, max_nfev=120):
    def objective(q):
        e = arm.error(x, q)
        return np.r_[10.0 * e[:3], e[3:]]

    result = least_squares(
        objective, np.clip(seed, lower, upper), bounds=(lower, upper),
        max_nfev=max_nfev, ftol=1e-8, xtol=1e-8, gtol=1e-8,
    )
    residual = float(np.max(np.abs(arm.error(x, result.x))))
    return result.x.copy(), residual, residual <= residual_tol


def branch_label(q, representatives, lower, upper, margin_threshold):
    if not representatives:
        return None, None, None
    scale = np.maximum(upper - lower, 1e-6)
    distances = [float(np.linalg.norm((q - rep) / scale)) for rep in representatives]
    order = np.argsort(distances)
    label = int(order[0])
    margin = float(distances[order[1]] - distances[order[0]]) if len(order) > 1 else float("inf")
    ambiguous = margin < margin_threshold
    return (None if ambiguous else label), margin, distances


def advance_representatives(arms, x, reps_by_side, lower, upper, residual_tol, max_nfev=120):
    updated = []
    ok = []
    for side, reps in enumerate(reps_by_side):
        side_updated = []
        side_ok = []
        for rep in reps:
            q_next, residual, success = solve_from_seed(
                arms[side], x, rep, lower[side], upper[side], residual_tol, max_nfev
            )
            side_updated.append(q_next)
            side_ok.append({"success": bool(success), "residual": residual})
        updated.append(side_updated)
        ok.append(side_ok)
    return updated, ok


def solve_horizontal(aq, ax, e, xdot, feedback, damping):
    rhs = -ax @ xdot - feedback * e
    if damping <= 0.0:
        return np.linalg.pinv(aq, rcond=1e-10) @ rhs
    gram = aq @ aq.T + damping**2 * np.eye(aq.shape[0])
    return aq.T @ np.linalg.solve(gram, rhs)


def run_dls(fk, x0, q0, args, reps_by_side, arms, lower, upper, method_name, damping):
    x, q = x0.copy(), q0.copy()
    previous_n = None
    rows = []
    jump_count = 0
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=float)
    r = np.array([args.r_left, args.r_right], dtype=float)
    reps = [[v.copy() for v in side] for side in reps_by_side]
    anchor = q0.copy()
    for step in range(args.steps + 1):
        labels, margins = [], []
        for side in range(2):
            label, margin, _ = branch_label(
                q[7 * side:7 * (side + 1)], reps[side], lower[side], upper[side], args.label_margin
            )
            labels.append(label)
            margins.append(margin)
        if rows and labels != rows[-1]["labels"] and None not in labels and None not in rows[-1]["labels"]:
            jump_count += 1
        rows.append({
            "method": method_name,
            "step": step,
            "x": x.tolist(),
            "q": q.tolist(),
            "labels": labels,
            "label_margins": margins,
            "descriptor": [descriptor(arms[s], x, q[7 * s:7 * (s + 1)]) for s in range(2)],
            "closure_error_inf": float(np.max(np.abs(fk.error(x, q)))),
        })
        if step == args.steps:
            break
        e, aq, ax = analytic_constraint_matrices(fk, x, q)
        _, _, vh = np.linalg.svd(aq, full_matrices=True)
        n = orient_basis(vh[-2:].T, previous_n)
        qdot = solve_horizontal(aq, ax, e, xdot, args.feedback, damping) + n @ r
        x_mid = x + 0.5 * args.dt * xdot
        q_mid = q + 0.5 * args.dt * qdot
        e_mid, aq_mid, ax_mid = analytic_constraint_matrices(fk, x_mid, q_mid)
        _, _, vh_mid = np.linalg.svd(aq_mid, full_matrices=True)
        n_mid = orient_basis(vh_mid[-2:].T, n)
        qdot_mid = solve_horizontal(aq_mid, ax_mid, e_mid, xdot, args.feedback, damping) + n_mid @ r
        x = x + args.dt * xdot
        q = q + args.dt * qdot_mid
        if (step + 1) % args.label_update_interval == 0:
            reps, _ = advance_representatives(
                arms, x, reps, lower, upper, args.residual_tol,
                args.trajectory_solver_nfev,
            )
        previous_n = n_mid
    return rows, jump_count


def run_pointwise(fk, x0, q0, args, reps_by_side, arms, lower, upper, rng):
    x, q = x0.copy(), q0.copy()
    rows = []
    jump_count = 0
    anchor = q0.copy()
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=float)
    reps = [[v.copy() for v in side] for side in reps_by_side]
    for step in range(args.steps + 1):
        labels, margins = [], []
        for side in range(2):
            label, margin, _ = branch_label(
                q[7 * side:7 * (side + 1)], reps[side], lower[side], upper[side], args.label_margin
            )
            labels.append(label)
            margins.append(margin)
        if rows and labels != rows[-1]["labels"] and None not in labels and None not in rows[-1]["labels"]:
            jump_count += 1
        rows.append({
            "method": "pointwise_ik",
            "step": step,
            "x": x.tolist(),
            "q": q.tolist(),
            "labels": labels,
            "label_margins": margins,
            "descriptor": [descriptor(arms[s], x, q[7 * s:7 * (s + 1)]) for s in range(2)],
            "closure_error_inf": float(np.max(np.abs(fk.error(x, q)))),
        })
        if step == args.steps:
            break
        x = x + args.dt * xdot
        q_next = []
        for side in range(2):
            if args.pointwise_seed_mode == "warm_start":
                seed = q[7 * side:7 * (side + 1)]
            elif args.pointwise_seed_mode == "anchor":
                seed = anchor[7 * side:7 * (side + 1)]
            else:
                seed = rng.uniform(lower[side], upper[side])
            solved, _, _ = solve_from_seed(
                arms[side], x, seed, lower[side], upper[side], args.residual_tol,
                args.trajectory_solver_nfev,
            )
            q_next.append(solved)
        q = np.concatenate(q_next)
        if (step + 1) % args.label_update_interval == 0:
            reps, _ = advance_representatives(
                arms, x, reps, lower, upper, args.residual_tol,
                args.trajectory_solver_nfev,
            )
    return rows, jump_count


def run_lift_filtered(
    fk, checker, x0, q0, args, reps_by_side, arms, lower, upper
):
    """Replay the actual beta-aware filtered lift on the branch probe.

    The branch labels use the same frozen atlas as the DLS and pointwise
    trajectories.  A filtered safety stop terminates this trajectory and is
    recorded explicitly; it is never counted as a branch jump.
    """
    x, q = x0.copy(), q0.copy()
    previous_n = None
    reps = [[v.copy() for v in side] for side in reps_by_side]
    rows = []
    jump_count = 0
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=float)
    r_nom = np.array([args.r_left, args.r_right], dtype=float)
    # The atlas helpers keep per-arm bounds as (2, 7) arrays, while the
    # unified QP operates on the concatenated 14-joint vector.
    lower_flat = np.concatenate(lower)
    upper_flat = np.concatenate(upper)
    stop_reason = "complete"
    for step in range(args.steps + 1):
        labels, margins = [], []
        for side in range(2):
            label, margin, _ = branch_label(
                q[7 * side:7 * (side + 1)], reps[side],
                lower[side], upper[side], args.label_margin
            )
            labels.append(label)
            margins.append(margin)
        if rows and labels != rows[-1]["labels"] and None not in labels and None not in rows[-1]["labels"]:
            jump_count += 1
        rows.append({
            "method": "lift_filtered",
            "step": step,
            "x": x.tolist(),
            "q": q.tolist(),
            "labels": labels,
            "label_margins": margins,
            "descriptor": [descriptor(arms[s], x, q[7 * s:7 * (s + 1)]) for s in range(2)],
            "closure_error_inf": float(np.max(np.abs(fk.error(x, q)))),
            "beta": 1.0,
            "stop_reason": stop_reason,
        })
        if step == args.steps:
            break

        qdot, n, info, sigma, beta, r_used = connector_step_y(
            fk, checker, x, q, xdot, r_nom, previous_n,
            lower_flat, upper_flat, args.dt, beta_stop=0.125,
        )
        if not info.get("feasible", False):
            stop_reason = info.get("stop_reason", "filtered_infeasible")
            rows[-1]["stop_reason"] = stop_reason
            break

        x_mid = x + 0.5 * args.dt * beta * xdot
        q_mid = q + 0.5 * args.dt * qdot
        qdot_mid, n_mid, info_mid, sigma_mid, beta_mid, r_mid = connector_step_y(
            fk, checker, x_mid, q_mid, xdot, r_used, n,
            lower_flat, upper_flat, args.dt, beta_stop=0.125,
        )
        if not info_mid.get("feasible", False):
            stop_reason = info_mid.get("stop_reason", "filtered_midpoint_infeasible")
            rows[-1]["stop_reason"] = stop_reason
            break

        x = x + args.dt * beta_mid * xdot
        q = q + args.dt * qdot_mid
        previous_n = n_mid
        if (step + 1) % args.label_update_interval == 0:
            reps, _ = advance_representatives(
                arms, x, reps, lower, upper, args.residual_tol,
                args.trajectory_solver_nfev,
            )

    return rows, jump_count


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    checker = CollisionChecker(args.model)
    arms = [ArmIK(args.model, 0), ArmIK(args.model, 1)]
    rng = np.random.default_rng(args.seed)
    lower, upper = lk.joint_lower, lk.joint_upper
    if (args.x_theta is None) != (args.x_slide is None):
        raise ValueError("--x-theta and --x-slide must be supplied together")
    if args.x_theta is None:
        x, _phi, lookup = sample_state(lk, rng)
    else:
        x = np.array([args.x_theta, args.x_slide], dtype=float)
        box = lk.safe_interval(x[None, :])
        if np.any(box["phi_safe_hi"][0] <= box["phi_safe_lo"][0]):
            raise RuntimeError("explicit task state has no safe phase interval")
        phi = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
        lookup = lk.query(x[None, :], phi[None, :], check_safe=False)
    q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
    static = []
    reps_by_side = []
    component_solutions = []
    static_codes = []
    for side, arm in enumerate(arms):
        q0_side = q0[7 * side:7 * (side + 1)]
        solutions, n_starts = solve_arm(
            arm, x, q0_side, lower[side], upper[side], rng,
            args.starts, args.residual_tol, args.dedup
        )
        components, edges = graph_components(
            arm, x, solutions, lower[side], upper[side], args.edge_max_distance,
            args.edge_subdivisions, args.edge_residual_tol,
            args.continuation_subdivisions, args.continuation_regularization,
        )
        reps = [solutions[c[0]][0].copy() for c in components]
        reps_by_side.append(reps)
        component_solutions.append(reps)
        if len(components) < 2:
            static_codes.append(f"SIDE_{side}_INSUFFICIENT_COMPONENTS")
        static.append({
            "side": side,
            "starts": n_starts,
            "valid_solutions": len(solutions),
            "graph_edges": len(edges),
            "components": [
                {
                    "component": i,
                    "size": len(c),
                    "descriptor_min_rad": float(min(descriptor(arm, x, solutions[j][0]) for j in c)),
                    "descriptor_max_rad": float(max(descriptor(arm, x, solutions[j][0]) for j in c)),
                    "representative": solutions[c[0]][0].tolist(),
                }
                for i, c in enumerate(components)
            ],
        })

    requested_components = [args.initial_component_left, args.initial_component_right]
    selected_components = ["lookup" if args.preserve_left_lookup else None, None]
    for side, requested in enumerate(requested_components):
        if side == 0 and args.preserve_left_lookup:
            continue
        if requested < 0:
            safe_candidates = []
            for candidate, rep in enumerate(component_solutions[side]):
                q_candidate = q0.copy()
                q_candidate[7 * side:7 * (side + 1)] = rep
                clearance, contact = checker.check(x, q_candidate)
                if clearance >= 0.005 and not contact:
                    safe_candidates.append((candidate, float(clearance)))
            if not safe_candidates:
                raise ValueError(
                    f"NO_SAFE_COMPONENT side={side}; "
                    f"found={len(component_solutions[side])}"
                )
            requested = max(safe_candidates, key=lambda item: item[1])[0]
        if requested < 0 or requested >= len(component_solutions[side]):
            raise ValueError(
                f"initial component {requested} unavailable for side {side}; "
                f"found {len(component_solutions[side])} components"
            )
        selected_components[side] = int(requested)
        q0[7 * side:7 * (side + 1)] = component_solutions[side][requested]

    hnr_rows, hnr_jumps = run_dls(
        fk, x, q0, args, reps_by_side, arms, lower, upper, "H+Nr", 0.0
    )
    dls_rows, dls_jumps = run_dls(
        fk, x, q0, args, reps_by_side, arms, lower, upper, "DLS", args.dls_damping
    )
    point_rows, point_jumps = run_pointwise(fk, x, q0, args, reps_by_side, arms, lower, upper, rng)
    filtered_rows, filtered_jumps = run_lift_filtered(
        fk, checker, x, q0, args, reps_by_side, arms, lower, upper
    )
    report = {
        "kind": "branch_label_probe",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "seed": args.seed,
        "initial_x": x.tolist(),
        "initial_q": q0.tolist(),
        "starts_per_arm": args.starts,
        "steps": args.steps,
        "dt": args.dt,
        "task_velocity": [args.theta_dot, args.slide_dot],
        "nominal_null_rate": [args.r_left, args.r_right],
        "pointwise_seed_mode": args.pointwise_seed_mode,
        "preserve_left_lookup": bool(args.preserve_left_lookup),
        "selected_initial_components": selected_components,
        "static_components": static,
        "static_failure_codes": static_codes,
        "trajectories": {
            "H+Nr": {"branch_jumps": hnr_jumps, "rows": hnr_rows},
            "DLS": {"branch_jumps": dls_jumps, "rows": dls_rows},
            "pointwise_ik": {"branch_jumps": point_jumps, "rows": point_rows},
            "lift_filtered": {"branch_jumps": filtered_jumps, "rows": filtered_rows},
        },
        "interpretation": [
            "Component labels are finite-resolution continuation labels at the sampled initial task state.",
            "A label transition is evidence of a branch change only when the component atlas remains valid and the label margin is non-ambiguous.",
            "This probe does not claim global IK completeness or continuous collision safety.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    if args.figure:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for method, color in (("H+Nr", "tab:green"), ("DLS", "tab:orange"), ("pointwise_ik", "tab:blue")):
            rr = report["trajectories"][method]["rows"]
            steps = [r["step"] for r in rr]
            label = [np.nan if None in r["labels"] else sum(2 ** i * r["labels"][i] for i in range(2)) for r in rr]
            desc = [r["descriptor"][0] for r in rr]
            axes[0].plot(steps, label, label=method, color=color)
            axes[1].plot(steps, desc, label=method, color=color)
        axes[0].set_ylabel("combined component label")
        axes[1].set_ylabel("left swivel descriptor (rad)")
        axes[1].set_xlabel("task step")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(args.figure, dpi=180)
        plt.close(fig)
    print(json.dumps({
        "static_failure_codes": static_codes,
        "H+Nr_branch_jumps": hnr_jumps,
        "DLS_branch_jumps": dls_jumps,
        "pointwise_branch_jumps": point_jumps,
        "lift_filtered_branch_jumps": filtered_jumps,
    }, indent=2))


if __name__ == "__main__":
    main()

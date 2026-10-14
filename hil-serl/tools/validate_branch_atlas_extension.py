#!/usr/bin/env python3
"""Dense-seed and local (theta, s, phi) atlas extension diagnostic.

This targets a selected state from the multi-start IK report.  It first adds
many IK seeds, then extends each valid right-arm solution along the one-
dimensional closure null direction (local phi) and across a small task-space
grid in (theta, s).  Each waypoint is corrected back onto the closure with a
bounded least-squares solve.  The result tests whether an apparently isolated
solution remains isolated under local atlas continuation.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import sample_state
from tools.validate_global_branch_descriptor import (
    ArmIK,
    descriptor,
    graph_components,
    solve_arm,
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--state-index", type=int, default=6)
    p.add_argument("--x", type=float, nargs=2, default=None,
                   metavar=("THETA", "S"),
                   help="explicit task state; overrides state-index sampling")
    p.add_argument("--starts", type=int, default=96)
    p.add_argument("--seed", type=int, default=91)
    p.add_argument("--theta-radius", type=float, default=0.025)
    p.add_argument("--s-radius", type=float, default=0.006)
    p.add_argument("--grid-points", type=int, default=3)
    p.add_argument("--phi-half-steps", type=int, default=3)
    p.add_argument("--phi-step", type=float, default=0.04)
    p.add_argument("--task-subdivisions", type=int, default=5)
    p.add_argument("--grid-connectivity", choices=("bfs", "direct"), default="bfs")
    p.add_argument("--residual-tol", type=float, default=2e-5)
    p.add_argument("--dedup", type=float, default=0.02)
    return p.parse_args()


def closure_jacobian(arm, x, q):
    jac = np.empty((6, 7), dtype=np.float64)
    h = 1e-6
    for j in range(7):
        qp, qm = q.copy(), q.copy()
        qp[j] += h
        qm[j] -= h
        jac[:, j] = (arm.error(x, qp) - arm.error(x, qm)) / (2.0 * h)
    return jac


def task_jacobian(arm, x, q):
    jac = np.empty((6, 2), dtype=np.float64)
    for j, h in enumerate((1e-6, 1e-6)):
        xp, xm = x.copy(), x.copy()
        xp[j] += h
        xm[j] -= h
        jac[:, j] = (arm.error(xp, q) - arm.error(xm, q)) / (2.0 * h)
    return jac


def correct(arm, x, q_pred, lower, upper, regularization=0.05):
    def objective(q):
        e = arm.error(x, q)
        return np.r_[10.0 * e[:3], e[3:], regularization * (q - q_pred)]

    result = least_squares(
        objective,
        np.clip(q_pred, lower, upper),
        bounds=(lower, upper),
        max_nfev=180,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )
    q = result.x
    return q, float(np.max(np.abs(arm.error(x, q))))


def phi_extend(arm, x, q0, lower, upper, half_steps, step, tol):
    paths = {"plus": [q0.copy()], "minus": [q0.copy()]}
    for label, direction in (("plus", 1.0), ("minus", -1.0)):
        q = q0.copy()
        for _ in range(half_steps):
            vh = np.linalg.svd(closure_jacobian(arm, x, q), full_matrices=True)[2]
            null = vh[-1]
            q_pred = q + direction * step * null
            q_new, residual = correct(arm, x, q_pred, lower, upper)
            if residual > tol:
                break
            paths[label].append(q_new)
            q = q_new
    return paths


def task_extend(arm, x0, q0, x1, lower, upper, tol, subdivisions=5):
    q = q0.copy()
    for alpha in np.linspace(1.0 / subdivisions, 1.0, subdivisions):
        x = (1.0 - alpha) * x0 + alpha * x1
        aq = closure_jacobian(arm, x, q)
        # Use the local horizontal lift for the task displacement.  This is
        # the same A_q^+(-A_x dx) structure used by the connector, so the
        # atlas extension tests the intended differential geometry rather
        # than relying on a fixed arbitrary null-space perturbation.
        ax = task_jacobian(arm, x, q)
        dx = x1 - x0
        q_pred = q + np.linalg.pinv(aq, rcond=1e-10) @ (-ax @ dx / subdivisions)
        q, residual = correct(arm, x, q_pred, lower, upper)
        if residual > tol:
            return None, residual
    return q, float(np.max(np.abs(arm.error(x1, q))))


def extend_grid(arm, x0, q_seed, theta_offsets, s_offsets, lower, upper,
                tol, subdivisions, mode):
    """Extend one phase seed over the task grid, directly or by 4-neighbor BFS."""
    n = len(theta_offsets)
    center = (n // 2, n // 2)
    states = {center: q_seed.copy()}
    errors = {center: float(np.max(np.abs(arm.error(x0, q_seed))))}
    queue = deque()
    if mode == "direct":
        for i in range(n):
            for j in range(n):
                if (i, j) != center:
                    queue.append((i, j))
    else:
        for neighbor in ((center[0] - 1, center[1]),
                         (center[0] + 1, center[1]),
                         (center[0], center[1] - 1),
                         (center[0], center[1] + 1)):
            if 0 <= neighbor[0] < n and 0 <= neighbor[1] < n:
                queue.append(neighbor)

    while queue:
        i, j = queue.popleft()
        x_target = x0 + np.array([theta_offsets[i], s_offsets[j]])
        if mode == "bfs":
            neighbors = [
                (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)
            ]
            source = next((v for v in neighbors if v in states), None)
            if source is None:
                continue
            x_source = x0 + np.array([theta_offsets[source[0]], s_offsets[source[1]]])
            q_source = states[source]
        else:
            x_source, q_source = x0, q_seed
        q_new, err = task_extend(
            arm, x_source, q_source, x_target, lower, upper, tol, subdivisions
        )
        if q_new is not None:
            if (i, j) not in states:
                states[(i, j)] = q_new
                errors[(i, j)] = err
                if mode == "bfs":
                    queue.append((i, j))
    return states, errors


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    rng = np.random.default_rng(args.seed)
    x = None
    lookup = None
    if args.x is None:
        for _ in range(args.state_index + 1):
            x, _phi, lookup = sample_state(lk, rng)
    else:
        x = np.asarray(args.x, dtype=np.float64)
        # Use the closest stored query as the anchor seed when the state is
        # supplied explicitly; dense IK will then explore around it.
        box = lk.safe_interval(x[None, :])
        lo = box["phi_safe_lo"][0]
        hi = box["phi_safe_hi"][0]
        phi = 0.5 * (lo + hi)
        lookup = lk.query(x[None, :], phi[None, :], check_safe=False)
    arm = ArmIK(args.model, 1)
    lower, upper = lk.joint_lower[1], lk.joint_upper[1]
    q_anchor = lookup["qR"][0].copy()
    solutions, n_starts = solve_arm(
        arm, x, q_anchor, lower, upper, rng, args.starts,
        args.residual_tol, args.dedup
    )
    components, edges = graph_components(
        arm, x, solutions, lower, upper, 0.25, 9, 5e-4, 12, 0.05
    )
    component_by_node = {}
    for cid, comp in enumerate(components):
        for idx in comp:
            component_by_node[idx] = cid

    theta_offsets = np.linspace(-args.theta_radius, args.theta_radius, args.grid_points)
    s_offsets = np.linspace(-args.s_radius, args.s_radius, args.grid_points)
    seed_rows = []
    for idx, (q0, residual, cost) in enumerate(solutions):
        phi_paths = phi_extend(arm, x, q0, lower, upper,
                               args.phi_half_steps, args.phi_step, args.residual_tol)
        grid_rows = [
            {"theta_offset": float(dtheta), "s_offset": float(ds)}
            for dtheta in theta_offsets for ds in s_offsets
        ]
        for label, path in phi_paths.items():
            states, errors = extend_grid(
                arm, x, path[-1], theta_offsets, s_offsets, lower, upper,
                args.residual_tol, args.task_subdivisions, args.grid_connectivity
            )
            for cell, row in enumerate(grid_rows):
                i, j = divmod(cell, args.grid_points)
                key = (i, j)
                q_new = states.get(key)
                row[f"{label}_ok"] = q_new is not None
                row[f"{label}_residual"] = float(errors.get(key, np.inf))
                if q_new is not None:
                    x_target = x + np.array([theta_offsets[i], s_offsets[j]])
                    row[f"{label}_descriptor"] = float(descriptor(arm, x_target, q_new))
                    row[f"{label}_q"] = q_new.tolist()
        seed_rows.append({
            "seed_index": idx,
            "base_component": component_by_node.get(idx),
            "base_descriptor_rad": float(descriptor(arm, x, q0)),
            "base_residual_m_rad": float(residual),
            "phi_plus_nodes": len(phi_paths["plus"]),
            "phi_minus_nodes": len(phi_paths["minus"]),
            "grid_success": int(sum(r["plus_ok"] and r["minus_ok"] for r in grid_rows)),
            "grid_total": len(grid_rows),
            "grid_rows": grid_rows,
        })

    cross_component_distances = {}
    if len(components) > 1:
        for cell in range(args.grid_points ** 2):
            by_component = {}
            for seed in seed_rows:
                comp = seed["base_component"]
                row = seed["grid_rows"][cell]
                for label in ("plus", "minus"):
                    q_key = f"{label}_q"
                    if q_key in row:
                        by_component.setdefault(comp, []).append(np.asarray(row[q_key]))
            comp_ids = sorted(by_component)
            pairs = {}
            for i, ci in enumerate(comp_ids):
                for cj in comp_ids[i + 1:]:
                    distances = [
                        float(np.max(np.abs(qa - qb)))
                        for qa in by_component[ci] for qb in by_component[cj]
                    ]
                    if distances:
                        pairs[f"{ci}-{cj}"] = min(distances)
            cross_component_distances[str(cell)] = pairs

    report = {
        "kind": "dense_branch_atlas_extension",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "state_index": args.state_index,
        "x": x.tolist(),
        "starts": n_starts,
        "valid_solutions": len(solutions),
        "base_graph_nodes": len(solutions),
        "base_graph_edges": len(edges),
        "base_graph_components": [
            {"size": len(c), "nodes": c,
             "descriptor_min_rad": float(min(descriptor(arm, x, solutions[i][0]) for i in c)),
             "descriptor_max_rad": float(max(descriptor(arm, x, solutions[i][0]) for i in c))}
            for c in components
        ],
        "theta_radius": args.theta_radius,
        "s_radius": args.s_radius,
        "grid_points": args.grid_points,
        "phi_half_steps": args.phi_half_steps,
        "phi_step": args.phi_step,
        "task_subdivisions": args.task_subdivisions,
        "grid_connectivity": args.grid_connectivity,
        "seed_rows": seed_rows,
        "cross_component_min_joint_distance_by_grid_cell": cross_component_distances,
        "interpretation": [
            "A successful edge is a finite-waypoint closure-corrected path within joint bounds.",
            "This local atlas tests sampled branch persistence around one state; it is not a global topology proof.",
            "An isolated node that remains isolated after dense seeds and task/phase continuation is a candidate branch, not yet a certified branch.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "state_index": args.state_index,
        "valid_solutions": len(solutions),
        "base_components": len(components),
        "grid_points_per_seed": args.grid_points ** 2,
        "phi_half_steps": args.phi_half_steps,
    }, indent=2))


if __name__ == "__main__":
    main()

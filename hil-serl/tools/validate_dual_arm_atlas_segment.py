#!/usr/bin/env python3
"""Dense dual-arm atlas continuation over a short task segment.

This is the gate before branch-stress trials.  Each arm is solved from a
large, deterministic multi-start set, finite continuation components are
formed, and every component representative is extended over a small
theta--s grid with local phi continuation.  A component is called stable only
when its representative reaches the required fraction of grid cells with
small closure residual and an unambiguous descriptor.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_global_branch_descriptor import (
    ArmIK,
    descriptor,
    graph_components,
    solve_arm,
)
from tools.validate_branch_atlas_extension import (
    extend_grid,
    phi_extend,
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--x", type=float, nargs=2, required=True, metavar=("THETA", "S"))
    p.add_argument("--starts", type=int, default=128)
    p.add_argument("--seed", type=int, default=20261006)
    p.add_argument("--theta-radius", type=float, default=0.08)
    p.add_argument("--s-radius", type=float, default=0.012)
    p.add_argument("--grid-points", type=int, default=5)
    p.add_argument("--phi-half-steps", type=int, default=4)
    p.add_argument("--phi-step", type=float, default=0.035)
    p.add_argument("--task-subdivisions", type=int, default=8)
    p.add_argument("--edge-max-distance", type=float, default=0.20)
    p.add_argument("--edge-subdivisions", type=int, default=11)
    p.add_argument("--edge-residual-tol", type=float, default=5e-4)
    p.add_argument("--continuation-subdivisions", type=int, default=14)
    p.add_argument("--continuation-regularization", type=float, default=0.05)
    p.add_argument("--residual-tol", type=float, default=2e-5)
    p.add_argument("--dedup", type=float, default=0.02)
    p.add_argument("--stable-cell-fraction", type=float, default=0.8)
    p.add_argument("--descriptor-separation", type=float, default=0.04)
    p.add_argument("--scan-only", action="store_true",
                   help="only build the dense dual-arm component atlas")
    return p.parse_args()


def component_extension(arm, x, q, lower, upper, args, theta_offsets, s_offsets):
    phi_paths = phi_extend(
        arm, x, q, lower, upper, args.phi_half_steps, args.phi_step,
        args.residual_tol,
    )
    best = None
    for direction, path in phi_paths.items():
        states, errors = extend_grid(
            arm, x, path[-1], theta_offsets, s_offsets, lower, upper,
            args.residual_tol, args.task_subdivisions, "bfs",
        )
        valid = []
        descriptors = []
        for cell, qcell in states.items():
            xcell = x + np.array([theta_offsets[cell[0]], s_offsets[cell[1]]])
            residual = float(errors[cell])
            if residual <= args.residual_tol:
                valid.append(cell)
                descriptors.append(float(descriptor(arm, xcell, qcell)))
        candidate = {
            "phi_direction": direction,
            "phi_nodes": len(path),
            "valid_cells": len(valid),
            "total_cells": len(theta_offsets) ** 2,
            "cell_fraction": len(valid) / float(len(theta_offsets) ** 2),
            "descriptor_min_rad": min(descriptors) if descriptors else None,
            "descriptor_max_rad": max(descriptors) if descriptors else None,
            "descriptor_values_rad": descriptors,
            "valid_cell_indices": [[int(i), int(j)] for i, j in valid],
        }
        if best is None or candidate["cell_fraction"] > best["cell_fraction"]:
            best = candidate
    return best


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    x = np.asarray(args.x, dtype=np.float64)
    box = lk.safe_interval(x[None, :])
    if np.any(box["phi_safe_hi"][0] <= box["phi_safe_lo"][0]):
        raise RuntimeError("task state has no safe phase interval")
    phi = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
    lookup = lk.query(x[None, :], phi[None, :], check_safe=False)
    rng = np.random.default_rng(args.seed)
    lower, upper = lk.joint_lower, lk.joint_upper
    theta_offsets = np.linspace(-args.theta_radius, args.theta_radius, args.grid_points)
    s_offsets = np.linspace(-args.s_radius, args.s_radius, args.grid_points)
    arm_reports = []
    for side in (0, 1):
        arm = ArmIK(args.model, side)
        q_anchor = lookup["qL" if side == 0 else "qR"][0].copy()
        solutions, n_starts = solve_arm(
            arm, x, q_anchor, lower[side], upper[side], rng,
            args.starts, args.residual_tol, args.dedup,
        )
        components, edges = graph_components(
            arm, x, solutions, lower[side], upper[side],
            args.edge_max_distance, args.edge_subdivisions,
            args.edge_residual_tol, args.continuation_subdivisions,
            args.continuation_regularization,
        )
        component_rows = []
        for cid, component in enumerate(components):
            rep_index = component[0]
            qrep = solutions[rep_index][0]
            ext = None if args.scan_only else component_extension(
                arm, x, qrep, lower[side], upper[side], args,
                theta_offsets, s_offsets,
            )
            component_rows.append({
                "component": cid,
                "size": len(component),
                "node_indices": component,
                "base_descriptor_min_rad": float(min(descriptor(arm, x, solutions[i][0]) for i in component)),
                "base_descriptor_max_rad": float(max(descriptor(arm, x, solutions[i][0]) for i in component)),
                "representative": qrep.tolist(),
                "extension": ext,
                "stable": bool(args.scan_only or (ext and ext["cell_fraction"] >= args.stable_cell_fraction)),
            })
        arm_reports.append({
            "side": side,
            "starts": n_starts,
            "valid_solutions": len(solutions),
            "graph_edges": len(edges),
            "components": component_rows,
        })

    stable_components = [
        [c for c in row["components"] if c["stable"]]
        for row in arm_reports
    ]
    separation_ok = True
    for comps in stable_components:
        centers = [0.5 * (c["base_descriptor_min_rad"] + c["base_descriptor_max_rad"])
                   for c in comps]
        if any(abs(a - b) < args.descriptor_separation
               for i, a in enumerate(centers) for b in centers[i + 1:]):
            separation_ok = False
    atlas_ok = all(len(comps) >= 2 for comps in stable_components) and separation_ok
    report = {
        "kind": "dual_arm_atlas_segment",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "x": x.tolist(),
        "starts_per_arm": args.starts,
        "theta_radius": args.theta_radius,
        "s_radius": args.s_radius,
        "grid_points": args.grid_points,
        "phi_half_steps": args.phi_half_steps,
        "phi_step": args.phi_step,
        "stable_cell_fraction": args.stable_cell_fraction,
        "descriptor_separation": args.descriptor_separation,
        "arms": arm_reports,
        "stable_component_counts": [len(v) for v in stable_components],
        "atlas_status": "pass" if atlas_ok else "fail",
        "interpretation": [
            "A stable component reaches the required fraction of the theta-s grid after local phi continuation.",
            "This is a finite atlas certificate for the selected segment, not a global topology proof.",
            "The branch-stress experiment may start only when both arms have at least two stable components.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"atlas_status": report["atlas_status"],
                      "stable_component_counts": report["stable_component_counts"]}, indent=2))


if __name__ == "__main__":
    main()

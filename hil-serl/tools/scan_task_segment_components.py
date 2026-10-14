#!/usr/bin/env python3
"""Scan a task-space segment for simultaneous multi-component IK structure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_global_branch_descriptor import ArmIK, descriptor, graph_components, solve_arm


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--x-center", type=float, nargs=2, required=True, metavar=("THETA", "S"))
    p.add_argument("--theta-radius", type=float, default=0.20)
    p.add_argument("--s-radius", type=float, default=0.025)
    p.add_argument("--grid-points", type=int, default=3)
    p.add_argument("--starts", type=int, default=100)
    p.add_argument("--seed", type=int, default=20261008)
    p.add_argument("--edge-max-distance", type=float, default=0.20)
    p.add_argument("--edge-subdivisions", type=int, default=11)
    p.add_argument("--edge-residual-tol", type=float, default=5e-4)
    p.add_argument("--continuation-subdivisions", type=int, default=14)
    p.add_argument("--continuation-regularization", type=float, default=0.05)
    p.add_argument("--residual-tol", type=float, default=2e-5)
    p.add_argument("--dedup", type=float, default=0.02)
    return p.parse_args()


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    center = np.asarray(args.x_center, dtype=float)
    theta_values = np.linspace(center[0] - args.theta_radius,
                               center[0] + args.theta_radius, args.grid_points)
    s_values = np.linspace(center[1] - args.s_radius,
                           center[1] + args.s_radius, args.grid_points)
    arms = [ArmIK(args.model, 0), ArmIK(args.model, 1)]
    rng = np.random.default_rng(args.seed)
    lower, upper = lk.joint_lower, lk.joint_upper
    cells = []
    for it, theta in enumerate(theta_values):
        for js, slide in enumerate(s_values):
            x = np.array([theta, slide], dtype=float)
            box = lk.safe_interval(x[None, :])
            if np.any(box["phi_safe_hi"][0] <= box["phi_safe_lo"][0]):
                cells.append({"grid": [it, js], "x": x.tolist(), "status": "unsafe_phase_interval"})
                continue
            phi = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
            lookup = lk.query(x[None, :], phi[None, :], check_safe=False)
            arm_rows = []
            for side, arm in enumerate(arms):
                qkey = "qL" if side == 0 else "qR"
                sols, nstarts = solve_arm(
                    arm, x, lookup[qkey][0], lower[side], upper[side], rng,
                    args.starts, args.residual_tol, args.dedup,
                )
                comps, edges = graph_components(
                    arm, x, sols, lower[side], upper[side],
                    args.edge_max_distance, args.edge_subdivisions,
                    args.edge_residual_tol, args.continuation_subdivisions,
                    args.continuation_regularization,
                )
                arm_rows.append({
                    "side": side,
                    "starts": nstarts,
                    "valid_solutions": len(sols),
                    "graph_edges": len(edges),
                    "component_count": len(comps),
                    "components": [
                        {
                            "size": len(c),
                            "descriptor_min_rad": float(min(descriptor(arm, x, sols[i][0]) for i in c)),
                            "descriptor_max_rad": float(max(descriptor(arm, x, sols[i][0]) for i in c)),
                            "representative": sols[c[0]][0].tolist(),
                        }
                        for c in comps
                    ],
                })
            counts = [a["component_count"] for a in arm_rows]
            cells.append({
                "grid": [it, js], "x": x.tolist(), "status": "ok",
                "arms": arm_rows, "component_counts": counts,
                "dual_multi_component": bool(all(c >= 2 for c in counts)),
            })

    dual_cells = [c for c in cells if c.get("dual_multi_component")]
    report = {
        "kind": "task_segment_component_scan",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "x_center": center.tolist(),
        "theta_radius": args.theta_radius,
        "s_radius": args.s_radius,
        "grid_points": args.grid_points,
        "starts_per_arm": args.starts,
        "cells": cells,
        "dual_multi_component_cells": len(dual_cells),
        "interpretation": [
            "This scan finds candidate task cells; it does not yet certify persistence across the whole segment.",
            "A candidate cell must be rechecked with local phi and theta--s atlas extension before branch stress testing.",
            "Component counts are finite-resolution continuation-graph results, not a global topology proof.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"cells": len(cells), "dual_multi_component_cells": len(dual_cells)}, indent=2))


if __name__ == "__main__":
    main()

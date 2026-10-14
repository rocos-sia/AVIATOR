#!/usr/bin/env python3
"""Compare the linearized phase polygon with the true one-step feasible slice."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from examples.experiments.aviator_manifold.safety_filter import _clip_polygon
from tools.audit_static_collision import CollisionChecker
from tools.validate_branch_connector import clearance_gradient, swept_clearance
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=20)
    p.add_argument("--grid-size", type=int, default=31)
    p.add_argument("--seed", type=int, default=20261003)
    p.add_argument("--dt", type=float, default=0.005)
    p.add_argument("--theta-dot", type=float, default=0.08)
    p.add_argument("--slide-dot", type=float, default=0.002)
    p.add_argument("--r-max", type=float, default=3.0)
    p.add_argument("--qdot-max", type=float, default=1.5)
    p.add_argument("--d-safe", type=float, default=0.005)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--iou-threshold", type=float, default=0.8)
    return p.parse_args()


def add_plane(planes, a, b):
    planes.append((np.asarray(a, dtype=float), float(b)))


def linear_planes(checker, x, q, h, n, lower, upper, dt, qdot_max, d_safe, r_max):
    planes = []
    for j in range(14):
        nj = n[j]
        add_plane(planes, nj, qdot_max - h[j])
        add_plane(planes, -nj, qdot_max + h[j])
        add_plane(planes, dt * nj, upper[j] - q[j] - dt * h[j])
        add_plane(planes, -dt * nj, q[j] - lower[j] + dt * h[j])
    d0, _ = checker.check(x, q)
    grad = clearance_gradient(checker, x, q)
    add_plane(planes, -dt * (grad @ n), d0 - d_safe + dt * float(grad @ h))
    polygon = [
        np.array([-r_max, -r_max]),
        np.array([r_max, -r_max]),
        np.array([r_max, r_max]),
        np.array([-r_max, r_max]),
    ]
    for a, b in planes:
        polygon = _clip_polygon(polygon, a, b)
        if len(polygon) == 0:
            break
    return planes, polygon


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    checker = CollisionChecker(args.model)
    lower = np.concatenate([lk.joint_lower[0], lk.joint_lower[1]])
    upper = np.concatenate([lk.joint_upper[0], lk.joint_upper[1]])
    rng = np.random.default_rng(args.seed)
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=float)
    r_values = np.linspace(-args.r_max, args.r_max, args.grid_size)
    rows = []
    failures = []

    for sample_id in range(args.samples):
        for _ in range(1000):
            x, _phi, lookup = sample_state(lk, rng)
            q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
            d0, contact0 = checker.check(x, q)
            if d0 >= args.d_safe + 0.002 and not contact0:
                break
        e, aq, ax = analytic_constraint_matrices(fk, x, q)
        _, singular, vh = np.linalg.svd(aq, full_matrices=True)
        n = orient_basis(vh[-2:].T, None)
        h = np.linalg.pinv(aq, rcond=1e-10) @ (-ax @ xdot - args.feedback * e)
        planes, polygon = linear_planes(
            checker, x, q, h, n, lower, upper, args.dt,
            args.qdot_max, args.d_safe, args.r_max,
        )
        linear_mask = []
        true_mask = []
        x_next = x + args.dt * xdot
        for rl in r_values:
            for rr in r_values:
                r = np.array([rl, rr], dtype=float)
                linear_ok = all(float(a @ r) <= b + 1e-10 for a, b in planes)
                qdot = h + n @ r
                q_next = q + args.dt * qdot
                d_next, contact_next = checker.check(x_next, q_next)
                swept_min, swept_contact = swept_clearance(
                    checker, x, q, x_next, q_next, subdivisions=5
                )
                true_ok = (
                    np.max(np.abs(qdot)) <= args.qdot_max + 2e-5
                    and np.all(q_next >= lower - 1e-9)
                    and np.all(q_next <= upper + 1e-9)
                    and d_next >= args.d_safe - 1e-9
                    and swept_min >= args.d_safe - 1e-9
                    and not contact_next
                    and not swept_contact
                )
                linear_mask.append(linear_ok)
                true_mask.append(true_ok)
        linear_mask = np.asarray(linear_mask, dtype=bool)
        true_mask = np.asarray(true_mask, dtype=bool)
        both = int(np.sum(linear_mask & true_mask))
        union = int(np.sum(linear_mask | true_mask))
        linear_only = int(np.sum(linear_mask & ~true_mask))
        true_only = int(np.sum(true_mask & ~linear_mask))
        iou = float(both / union) if union else 1.0
        row = {
            "sample": sample_id,
            "sigma_min_Aq": float(singular[-1]),
            "linear_polygon_vertices": int(len(polygon)),
            "linear_grid_points": int(np.sum(linear_mask)),
            "true_grid_points": int(np.sum(true_mask)),
            "intersection_grid_points": both,
            "union_grid_points": union,
            "grid_iou": iou,
            "linear_only_points": linear_only,
            "true_only_points": true_only,
        }
        codes = []
        if not polygon:
            codes.append("SAFE_SET_EMPTY")
        if union and iou < args.iou_threshold:
            codes.append("LINEARIZATION_MISMATCH")
        if linear_only:
            codes.append("LINEAR_FALSE_POSITIVE")
        if true_only:
            codes.append("LINEAR_FALSE_NEGATIVE")
        row["failure_codes"] = codes
        if codes:
            failures.append({"sample": sample_id, "codes": codes})
        rows.append(row)

    def stat(key):
        a = np.asarray([r[key] for r in rows], dtype=float)
        return {"median": float(np.median(a)), "p95": float(np.quantile(a, .95)), "max": float(np.max(a))}

    report = {
        "kind": "affine_phase_slice_contract_probe",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "samples": len(rows),
        "grid_size": args.grid_size,
        "seed": args.seed,
        "dt": args.dt,
        "xdot": xdot.tolist(),
        "r_max": args.r_max,
        "d_safe": args.d_safe,
        "iou_threshold": args.iou_threshold,
        "metrics": {
            "sigma_min_Aq": stat("sigma_min_Aq"),
            "grid_iou": stat("grid_iou"),
            "linear_only_points": stat("linear_only_points"),
            "true_only_points": stat("true_only_points"),
            "linear_polygon_vertices": stat("linear_polygon_vertices"),
        },
        "failed_samples": failures,
        "contract_status": "pass" if not failures else "fail",
        "scope": "one-step sampled slice; not a continuous-time safety proof",
        "rows": rows,
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"contract_status": report["contract_status"], "failed": len(failures), "metrics": report["metrics"]}, indent=2))


if __name__ == "__main__":
    main()

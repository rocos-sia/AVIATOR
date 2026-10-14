#!/usr/bin/env python3
"""Multi-start IK diagnostic for the local elbow-swivel branch descriptor.

For a fixed task state ``x``, each arm has a 6D rigid-grasp target and seven
joint variables.  This script solves that target from many deterministic local
and global seeds, deduplicates valid solutions, and compares their independent
elbow-swivel descriptors.  It is a branch-structure diagnostic: finding
separated descriptor values is evidence for a useful coordinate, but the
solver does not prove that two solution sets are disconnected globally.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure, sample_state


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--states", type=int, default=8)
    p.add_argument("--starts", type=int, default=24)
    p.add_argument("--seed", type=int, default=91)
    p.add_argument("--dedup", type=float, default=0.02,
                   help="joint-space distance for merging repeated solutions")
    p.add_argument("--residual-tol", type=float, default=2e-5)
    p.add_argument("--edge-max-distance", type=float, default=0.25,
                   help="maximum joint-space distance considered for a graph edge")
    p.add_argument("--edge-subdivisions", type=int, default=9)
    p.add_argument("--edge-residual-tol", type=float, default=5e-4)
    p.add_argument("--continuation-subdivisions", type=int, default=12)
    p.add_argument("--continuation-regularization", type=float, default=0.05)
    return p.parse_args()


class ArmIK:
    """Independent MuJoCo FK residual for one 7-DOF arm."""

    def __init__(self, model_path: str, side: int):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.side = side
        handle_name = ("left_handle", "right_handle")[side]
        tcp_name = ("left_tcp", "right_tcp")[side]
        self.handle = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, handle_name
        )
        self.tcp = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, tcp_name
        )
        if self.handle < 0 or self.tcp < 0:
            raise RuntimeError(f"missing sites for arm {side}")
        self.qslice = slice(2 + 7 * side, 2 + 7 * (side + 1))

    def error(self, x: np.ndarray, qside: np.ndarray) -> np.ndarray:
        self.data.qpos[:] = 0.0
        self.data.qpos[:2] = x
        self.data.qpos[self.qslice] = qside
        mujoco.mj_forward(self.model, self.data)
        p_tcp = self.data.site_xpos[self.tcp]
        p_handle = self.data.site_xpos[self.handle]
        r_tcp = self.data.site_xmat[self.tcp].reshape(3, 3)
        r_handle = self.data.site_xmat[self.handle].reshape(3, 3)
        return np.concatenate([
            p_tcp - p_handle,
            Rotation.from_matrix(r_tcp.T @ r_handle).as_rotvec(),
        ])


def descriptor(arm: ArmIK, x: np.ndarray, q: np.ndarray) -> float:
    """Signed elbow-swivel angle in the same local frame as the connector."""
    arm.data.qpos[:] = 0.0
    arm.data.qpos[:2] = x
    arm.data.qpos[arm.qslice] = q
    mujoco.mj_forward(arm.model, arm.data)
    base = 3 if arm.side == 0 else 13
    elbow = 7 if arm.side == 0 else 17
    shoulder = arm.data.xpos[base].copy()
    elbow_p = arm.data.xpos[elbow].copy()
    wrist = arm.data.site_xpos[arm.tcp].copy()
    axis = wrist - shoulder
    axis /= max(np.linalg.norm(axis), 1e-12)
    vec = elbow_p - shoulder
    vec -= axis * float(axis @ vec)
    vec /= max(np.linalg.norm(vec), 1e-12)
    ref = np.array([0.0, 0.0, 1.0])
    ref -= axis * float(axis @ ref)
    if np.linalg.norm(ref) < 1e-8:
        ref = np.array([0.0, 1.0, 0.0])
        ref -= axis * float(axis @ ref)
    ref /= np.linalg.norm(ref)
    ortho = np.cross(axis, ref)
    return float(np.arctan2(ortho @ vec, ref @ vec))


def angle_delta(a, b):
    return float((a - b + np.pi) % (2.0 * np.pi) - np.pi)


def deduplicate(solutions, threshold):
    unique = []
    for q, residual, cost in sorted(solutions, key=lambda z: z[1]):
        if all(np.max(np.abs(q - old[0])) > threshold for old in unique):
            unique.append((q, residual, cost))
    return unique


def continuation_edge(arm, x, qa, qb, lower, upper, subdivisions,
                      residual_tol, regularization):
    """Follow a joint-space chord with closure projection at each waypoint."""
    qcur = qa.copy()
    direction = qb - qa
    for step in range(1, subdivisions + 1):
        qref = qa + (step / subdivisions) * direction

        def objective(q):
            e = arm.error(x, q)
            return np.r_[10.0 * e[:3], e[3:], regularization * (q - qref)]

        result = least_squares(
            objective,
            qcur,
            bounds=(lower, upper),
            max_nfev=120,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        qcur = result.x
        if np.max(np.abs(arm.error(x, qcur))) > residual_tol:
            return False
    return np.max(np.abs(qcur - qb)) <= max(0.05, 2.0 * np.max(np.abs(direction)) / subdivisions)


def graph_components(arm, x, solutions, lower, upper, max_distance,
                     subdivisions, residual_tol, continuation_subdivisions,
                     continuation_regularization):
    """Build a finite-resolution graph of continuation-valid paths."""
    n = len(solutions)
    parent = list(range(n))
    edges = []

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    for i in range(n):
        qi = solutions[i][0]
        for j in range(i + 1, n):
            qj = solutions[j][0]
            if np.max(np.abs(qi - qj)) > max_distance:
                continue
            valid = continuation_edge(
                arm, x, qi, qj, lower, upper, continuation_subdivisions,
                residual_tol, continuation_regularization
            )
            if valid:
                edges.append([i, j])
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values()), edges


def solve_arm(arm, x, q0, lower, upper, rng, starts, residual_tol, dedup_tol):
    seeds = [q0.copy()]
    local_count = max(1, starts // 2)
    for _ in range(local_count - 1):
        seed = q0 + rng.normal(0.0, 0.35, size=7)
        seeds.append(np.clip(seed, lower, upper))
    for _ in range(starts - len(seeds)):
        seeds.append(rng.uniform(lower, upper))

    valid = []
    for seed in seeds:
        def scaled_error(q):
            e = arm.error(x, q)
            return np.r_[10.0 * e[:3], e[3:]]

        result = least_squares(
            scaled_error,
            seed,
            bounds=(lower, upper),
            max_nfev=350,
            ftol=1e-11,
            xtol=1e-11,
            gtol=1e-11,
        )
        raw = arm.error(x, result.x)
        max_residual = float(np.max(np.abs(raw)))
        if max_residual <= residual_tol:
            valid.append((result.x.copy(), max_residual, float(result.cost)))
    return deduplicate(valid, dedup_tol), len(seeds)


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    checker = CollisionChecker(args.model)
    arms = [ArmIK(args.model, 0), ArmIK(args.model, 1)]
    rng = np.random.default_rng(args.seed)
    lower = lk.joint_lower
    upper = lk.joint_upper
    rows = []

    for state_id in range(args.states):
        x, _phi, lookup = sample_state(lk, rng)
        q0 = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        arm_rows = []
        for side, arm in enumerate(arms):
            q0_side = q0[7 * side:7 * (side + 1)]
            sols, n_starts = solve_arm(
                arm, x, q0_side, lower[side], upper[side], rng,
                args.starts, args.residual_tol, args.dedup
            )
            values = [descriptor(arm, x, q) for q, _, _ in sols]
            anchor = descriptor(arm, x, q0_side)
            near = [float(np.max(np.abs(q - q0_side))) for q, _, _ in sols]
            far_values = [v for v, d in zip(values, near) if d > 0.15]
            pairwise = [
                abs(angle_delta(values[i], values[j]))
                for i in range(len(values)) for j in range(i + 1, len(values))
            ]
            components, edges = graph_components(
                arm, x, sols, lower[side], upper[side], args.edge_max_distance,
                args.edge_subdivisions, args.edge_residual_tol,
                args.continuation_subdivisions, args.continuation_regularization
            )
            component_rows = []
            anchor_idx = int(np.argmin(near)) if near else None
            for component in components:
                comp_values = [values[i] for i in component]
                component_rows.append({
                    "nodes": component,
                    "size": len(component),
                    "descriptor_min_rad": float(min(comp_values)),
                    "descriptor_max_rad": float(max(comp_values)),
                    "contains_anchor": anchor_idx in component if anchor_idx is not None else False,
                })
            arm_rows.append({
                "side": side,
                "starts": n_starts,
                "valid_solutions": len(sols),
                "anchor_descriptor_rad": float(anchor),
                "descriptor_values_rad": [float(v) for v in values],
                "descriptor_range_rad": (
                    float(max(values) - min(values)) if values else None
                ),
                "max_descriptor_pair_gap_rad": max(pairwise) if pairwise else 0.0,
                "far_solution_count": len(far_values),
                "far_descriptor_values_rad": [float(v) for v in far_values],
                "max_residual_m_rad": max((s[1] for s in sols), default=None),
                "joint_solutions": [q.tolist() for q, _, _ in sols],
                "graph_nodes": len(sols),
                "graph_edges": len(edges),
                "graph_components": component_rows,
                "graph_connected": len(components) <= 1,
            })

        # Combine one solution per arm to check pairwise collision feasibility.
        valid_pairs = 0
        safe_pairs = 0
        if arm_rows[0]["joint_solutions"] and arm_rows[1]["joint_solutions"]:
            for ql in arm_rows[0]["joint_solutions"]:
                for qr in arm_rows[1]["joint_solutions"]:
                    q = np.asarray(ql + qr, dtype=np.float64)
                    valid_pairs += 1
                    clearance, contact = checker.check(x, q)
                    if clearance >= 0.005 and not contact:
                        safe_pairs += 1
        rows.append({
            "state": state_id,
            "x": x.tolist(),
            "initial_closure_error_mixed_m_rad": float(np.max(np.abs(fk.error(x, q0)))),
            "arms": arm_rows,
            "valid_solution_pairs": valid_pairs,
            "safe_solution_pairs": safe_pairs,
        })

    report = {
        "kind": "multi_start_ik_branch_descriptor_diagnostic",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "states": args.states,
        "starts_per_arm": args.starts,
        "seed": args.seed,
        "residual_tolerance_m_rad": args.residual_tol,
        "dedup_joint_threshold_rad": args.dedup,
        "edge_max_distance_rad": args.edge_max_distance,
        "edge_subdivisions": args.edge_subdivisions,
        "edge_residual_tolerance_m_rad": args.edge_residual_tol,
        "continuation_subdivisions": args.continuation_subdivisions,
        "continuation_regularization": args.continuation_regularization,
        "states_with_both_arm_solutions": sum(
            all(a["valid_solutions"] > 0 for a in row["arms"]) for row in rows
        ),
        "states_with_safe_solution_pair": sum(row["safe_solution_pairs"] > 0 for row in rows),
        "rows": rows,
        "interpretation": [
            "Multi-start solutions are independent pose-closure IK candidates within joint bounds.",
            "Descriptor separation is evidence for a useful global coordinate, not a proof of disconnected components.",
            "A disconnected-branch claim still requires a certified path-connectivity or atlas analysis.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "states_with_both_arm_solutions": report["states_with_both_arm_solutions"],
        "states_with_safe_solution_pair": report["states_with_safe_solution_pair"],
        "states": args.states,
    }, indent=2))


if __name__ == "__main__":
    main()

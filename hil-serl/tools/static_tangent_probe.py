#!/usr/bin/env python3
"""Contract probe for the regular local differential-lift chart.

This script intentionally excludes collision checking and long-horizon control.
It tests only the algebraic assumptions needed before a micro-step or trajectory
experiment is allowed to run.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=100)
    p.add_argument("--seed", type=int, default=20261002)
    p.add_argument("--dt", type=float, default=1e-4)
    p.add_argument("--theta-dot", type=float, default=0.08)
    p.add_argument("--slide-dot", type=float, default=0.002)
    p.add_argument("--r-left", type=float, default=0.03)
    p.add_argument("--r-right", type=float, default=-0.025)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--sigma-threshold", type=float, default=1e-3)
    p.add_argument("--null-tol", type=float, default=1e-10)
    p.add_argument("--tangent-tol", type=float, default=1e-10)
    p.add_argument("--alignment-threshold", type=float, default=0.95)
    return p.parse_args()


def stats(values):
    a = np.asarray(values, dtype=float)
    return {
        "n": int(a.size),
        "median": float(np.median(a)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(np.max(a)),
    }


def main():
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    rng = np.random.default_rng(args.seed)
    xdot = np.array([args.theta_dot, args.slide_dot], dtype=float)
    r_nom = np.array([args.r_left, args.r_right], dtype=float)
    rows = []
    failures = []

    for sample_id in range(args.samples):
        x, _phi, lookup = sample_state(lk, rng)
        q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        e, aq, ax = analytic_constraint_matrices(fk, x, q)
        _, singular, vh = np.linalg.svd(aq, full_matrices=True)
        n = orient_basis(vh[-2:].T, None)
        horizontal = np.linalg.pinv(aq, rcond=1e-10) @ (-ax @ xdot - args.feedback * e)
        qdot = horizontal + n @ r_nom
        tangent_residual = np.max(np.abs(aq @ qdot + ax @ xdot + args.feedback * e))
        null_residual = np.max(np.abs(aq @ n))
        orth_residual = np.max(np.abs(n.T @ n - np.eye(2)))
        # A tiny unconstrained micro-step supplies the discrete Lyapunov
        # diagnostic without turning this probe into a trajectory experiment.
        x1 = x + args.dt * xdot
        q1 = q + args.dt * qdot
        e1 = fk.error(x1, q1)
        _, aq1, _ = analytic_constraint_matrices(fk, x1, q1)
        _, _, vh1 = np.linalg.svd(aq1, full_matrices=True)
        n1 = orient_basis(vh1[-2:].T, n)
        alignment = float(np.min(np.linalg.svd(n.T @ n1)[1]))
        v0 = 0.5 * float(e @ e)
        v1 = 0.5 * float(e1 @ e1)
        lyapunov_delta = v1 - v0

        row = {
            "sample": sample_id,
            "sigma_min_Aq": float(singular[-1]),
            "rank_Aq": int(np.sum(singular > args.sigma_threshold)),
            "tangent_residual_inf": float(tangent_residual),
            "null_residual_inf": float(null_residual),
            "basis_orthogonality_inf": float(orth_residual),
            "basis_alignment_min_singular": alignment,
            "closure_error_inf": float(np.max(np.abs(e))),
            "closure_error_next_inf": float(np.max(np.abs(e1))),
            "lyapunov_delta_euclidean": float(lyapunov_delta),
        }
        row_failures = []
        if row["sigma_min_Aq"] < args.sigma_threshold:
            row_failures.append("RANK_LOSS")
        if row["null_residual_inf"] > args.null_tol:
            row_failures.append("NULLSPACE_RESIDUAL")
        if row["tangent_residual_inf"] > args.tangent_tol:
            row_failures.append("TANGENT_RESIDUAL")
        if alignment < args.alignment_threshold:
            row_failures.append("BASIS_ORIENTATION_BREAK")
        row["failure_codes"] = row_failures
        if row_failures:
            failures.append({"sample": sample_id, "codes": row_failures})
        rows.append(row)

    def collect(key):
        return stats([r[key] for r in rows if r[key] is not None])

    report = {
        "kind": "static_tangent_contract_probe",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "samples": len(rows),
        "seed": args.seed,
        "dt": args.dt,
        "xdot": xdot.tolist(),
        "r_nom": r_nom.tolist(),
        "thresholds": {
            "sigma_min_Aq": args.sigma_threshold,
            "null_residual_inf": args.null_tol,
            "tangent_residual_inf": args.tangent_tol,
            "basis_alignment_min_singular": args.alignment_threshold,
        },
        "metrics": {
            "sigma_min_Aq": collect("sigma_min_Aq"),
            "tangent_residual_inf": collect("tangent_residual_inf"),
            "null_residual_inf": collect("null_residual_inf"),
            "basis_orthogonality_inf": collect("basis_orthogonality_inf"),
            "basis_alignment_min_singular": collect("basis_alignment_min_singular"),
            "lyapunov_delta_euclidean": collect("lyapunov_delta_euclidean"),
        },
        "failed_samples": failures,
        "contract_status": "pass" if not failures else "fail",
        "scope": "local algebraic and one-step diagnostic; no collision or global branch claim",
        "rows": rows,
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"contract_status": report["contract_status"], "failed": len(failures), "metrics": report["metrics"]}, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""GFR-0 contract probe for the frozen two-dimensional RL action.

This probe uses real AVIATOR/MuJoCo states to check the interface introduced by
the revised GFR plan:

    qdot = h_e + beta_g h_x + N r

The governor owns beta_g.  The policy owns r in the affine slice

    A_r r <= b - a_beta beta_g.

The probe checks the closure residual, null-space residual, pull-back of
affine inequalities, normal/degenerate/empty slices, and the three governor
outcomes.  It does not run RL, long trajectories, or claim collision safety.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--csv", required=True)
    p.add_argument("--samples", type=int, default=64)
    p.add_argument("--seed", type=int, default=20261011)
    p.add_argument("--tol", type=float, default=1e-8)
    return p.parse_args()


def feasible(A: np.ndarray, b: np.ndarray, beta_lo: float = 0.125,
             beta_hi: float = 1.0) -> tuple[bool, np.ndarray | None]:
    """Check feasibility of [r_L,r_R,beta] with bounded r and beta."""
    bounds = [(-3.0, 3.0), (-3.0, 3.0), (beta_lo, beta_hi)]
    result = linprog(
        np.zeros(3), A_ub=A, b_ub=b, bounds=bounds, method="highs"
    )
    return bool(result.success), (np.asarray(result.x) if result.success else None)


def slice_constraints(A_full: np.ndarray, b_full: np.ndarray,
                      beta_g: float) -> tuple[np.ndarray, np.ndarray]:
    return A_full[:, :2], b_full - A_full[:, 2] * float(beta_g)


def one_case(lk: ManifoldLookup, fk: AviatorClosure,
             rng: np.random.Generator, tol: float, index: int) -> dict:
    x, _phi, lookup = sample_state(lk, rng)
    q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, None)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    xdot = np.array([0.08, 0.002], dtype=np.float64)
    gain = 2.0
    he = pinv @ (-gain * e)
    hx = pinv @ (-ax @ xdot)
    beta_g = float(rng.uniform(0.2, 1.0))
    r0 = rng.uniform(-0.2, 0.2, size=2)
    qdot = he + beta_g * hx + n @ r0
    closure = aq @ qdot + beta_g * ax @ xdot + gain * e
    null_residual = aq @ n

    # Build a full affine safety set from a known feasible point.  Pulling it
    # back at beta_g must preserve the same inequalities in r.
    A_full = rng.normal(size=(32, 3))
    y0 = np.r_[r0, beta_g]
    b_full = A_full @ y0 + rng.uniform(0.05, 0.5, size=A_full.shape[0])
    A_r, b_slice = slice_constraints(A_full, b_full, beta_g)
    pullback = A_r @ r0 - b_slice
    slice_ok, slice_sol = feasible(np.c_[A_r, np.zeros(A_r.shape[0])], b_slice)

    # Normal, degenerate and empty synthetic slices.  These test only the
    # action-interface semantics, not the robot collision geometry.
    normal_A = np.array([[1., 0., 0.], [-1., 0., 0.],
                         [0., 1., 0.], [0., -1., 0.]])
    normal_b = np.array([1., 1., 1., 1.])
    normal_ok, _ = feasible(normal_A, normal_b)
    deg_A = np.vstack([normal_A, [1., 0., 0.], [-1., 0., 0.]])
    deg_b = np.r_[normal_b, 0., 0.]
    deg_ok, deg_sol = feasible(deg_A, deg_b)
    empty_A = np.array([[0., 0., 0.]])
    empty_b = np.array([-1.])
    empty_ok, _ = feasible(empty_A, empty_b)

    governor = {}
    for name, upper in (("full_speed", 1.0), ("slowdown", 0.4), ("stop", -0.1)):
        A_beta = np.array([[0., 0., 1.]])
        b_beta = np.array([upper])
        ok, sol = feasible(A_beta, b_beta)
        expected = name != "stop"
        governor[name] = {
            "upper_beta": upper,
            "feasible": ok,
            "beta": None if sol is None else float(sol[2]),
            "pass": bool(ok == expected),
        }

    row_pass = bool(
        np.max(np.abs(closure)) <= tol
        and np.max(np.abs(null_residual)) <= tol
        and np.max(pullback) <= tol
        and slice_ok
        and normal_ok
        and deg_ok
        and deg_sol is not None
        and abs(float(deg_sol[0])) <= 1e-7
        and not empty_ok
        and all(item["pass"] for item in governor.values())
    )
    return {
        "sample": index,
        "sigma_min_Aq": float(singular[-1]),
        "closure_residual_inf": float(np.max(np.abs(closure))),
        "null_residual_inf": float(np.max(np.abs(null_residual))),
        "pullback_violation_max": float(np.max(pullback)),
        "beta_g": beta_g,
        "normal_slice_feasible": bool(normal_ok),
        "degenerate_slice_feasible": bool(deg_ok),
        "degenerate_solution": None if deg_sol is None else deg_sol.tolist(),
        "empty_slice_feasible": bool(empty_ok),
        "governor": governor,
        "pass": row_pass,
    }


def main() -> None:
    a = parse_args()
    lk = ManifoldLookup(a.manifold)
    fk = AviatorClosure(a.model)
    rng = np.random.default_rng(a.seed)
    rows = [one_case(lk, fk, rng, a.tol, i) for i in range(a.samples)]
    report = {
        "kind": "gfr0_real_two_dimensional_beta_slice_contract",
        "manifold": str(Path(a.manifold).resolve()),
        "model": str(Path(a.model).resolve()),
        "samples": a.samples,
        "seed": a.seed,
        "tolerance": a.tol,
        "contract_status": "pass" if all(row["pass"] for row in rows) else "fail",
        "failed_rows": int(sum(not row["pass"] for row in rows)),
        "scope": "real MuJoCo local algebra and action-slice contract; no RL, trajectory, or collision claim",
        "rows": rows,
    }
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    fields = ["sample", "sigma_min_Aq", "closure_residual_inf", "null_residual_inf",
              "pullback_violation_max", "beta_g", "normal_slice_feasible",
              "degenerate_slice_feasible", "empty_slice_feasible", "pass"]
    csv_out = Path(a.csv)
    csv_out.parent.mkdir(parents=True, exist_ok=True)
    with csv_out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row[key] for key in fields} for row in rows)
    print(json.dumps({
        "contract_status": report["contract_status"],
        "failed_rows": report["failed_rows"],
        "output": str(out),
        "csv": str(csv_out),
    }, indent=2))


if __name__ == "__main__":
    main()

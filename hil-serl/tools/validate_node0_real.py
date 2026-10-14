#!/usr/bin/env python3
"""Node-0 validation on real AVIATOR/MuJoCo states.

The script intentionally stops at local contracts.  It does not run long
trajectories or claim collision safety.  It checks analytic Jacobians against
finite differences, the ``h_e+beta*h_x+N*r`` residual, and equality/inequality
equivalence of a full joint-space QP and its reduced action QP.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import LinearConstraint, minimize

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import (
    analytic_constraint_matrices,
    constraint_matrices,
    orient_basis,
)
from examples.experiments.aviator_manifold.safety_filter import project_reduced_action


def args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=24)
    p.add_argument("--seed", type=int, default=20261004)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--tol", type=float, default=5e-5)
    return p.parse_args()


def solve_full_reduced(aq, ax, e, n, xdot_nom, rng):
    """Compare two SLSQP formulations of the same local linear QP."""
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    he = pinv @ (-2.0 * e)
    hx = pinv @ (-ax @ xdot_nom)
    r_nom = rng.uniform(-0.2, 0.2, size=2)
    q_nom = he + hx + n @ r_nom
    beta_nom = 1.0

    # Safety inequalities are deliberately generated from a known feasible
    # point, so an infeasible solver result is an implementation failure.
    cq = rng.normal(size=(24, 14))
    cx = rng.normal(size=(24, 2))
    q0 = he + 0.35 * hx + n @ (0.5 * r_nom)
    b = cq @ q0 + cx @ (0.35 * xdot_nom) + rng.uniform(0.1, 0.5, size=24)

    def full_obj(z):
        dq = z[:14] - q_nom
        return 0.5 * float(dq @ dq) + 5.0 * (z[14] - beta_nom) ** 2

    def full_grad(z):
        out = np.zeros(15)
        out[:14] = z[:14] - q_nom
        out[14] = 10.0 * (z[14] - beta_nom)
        return out

    def full_eq(z):
        return aq @ z[:14] + ax @ (z[14] * xdot_nom) + 2.0 * e

    def red_to_full(y):
        return he + y[2] * hx + n @ y[:2]

    def red_obj(y):
        dq = red_to_full(y) - q_nom
        return 0.5 * float(dq @ dq) + 5.0 * (y[2] - beta_nom) ** 2

    def red_grad(y):
        eps = 1e-7
        g = np.zeros(3)
        for j in range(3):
            yp, ym = y.copy(), y.copy()
            yp[j] += eps
            ym[j] -= eps
            g[j] = (red_obj(yp) - red_obj(ym)) / (2.0 * eps)
        return g

    c_full = np.zeros((24, 15))
    c_full[:, :14] = cq
    c_full[:, 14] = cx @ xdot_nom
    eq_constraint = LinearConstraint(aq, -2.0 * e - ax @ np.zeros(2), -2.0 * e - ax @ np.zeros(2))
    # The equality depends on beta, so use scipy's nonlinear equality wrapper
    # while keeping all safety constraints linear.
    full = minimize(
        full_obj,
        np.r_[q0, 0.35],
        jac=full_grad,
        method="SLSQP",
        bounds=[(-5.0, 5.0)] * 14 + [(0.0, 1.0)],
        constraints=[
            {"type": "eq", "fun": full_eq},
            LinearConstraint(c_full, -np.inf, b),
        ],
        options={"ftol": 1e-10, "maxiter": 300},
    )
    c_red = cq @ n
    c_red_beta = cq @ hx + cx @ xdot_nom
    A_red = np.column_stack([c_red, c_red_beta])
    red = minimize(
        red_obj,
        np.r_[0.5 * r_nom, 0.35],
        jac=red_grad,
        method="SLSQP",
        bounds=[(-5.0, 5.0), (-5.0, 5.0), (0.0, 1.0)],
        constraints=[LinearConstraint(A_red, -np.inf, b - cq @ he)],
        options={"ftol": 1e-10, "maxiter": 300},
    )
    if not (full.success and red.success):
        return {"pass": False, "reason": "solver_failure"}
    q_red = red_to_full(red.x)
    return {
        "pass": True,
        "full_reduced_qdot_inf": float(np.max(np.abs(full.x[:14] - q_red))),
        "objective_abs_diff": float(abs(full.fun - red.fun)),
        "full_eq_residual_inf": float(np.max(np.abs(full_eq(full.x)))),
        "reduced_ineq_residual_max": float(np.max(A_red @ red.x - (b - cq @ he))),
    }


def governor_cases():
    """Exercise full-speed, slowdown, and stop semantics on a real-state QP."""
    out = {}
    for name, limit in (("full_speed", 1.1), ("slowdown", 0.4), ("stop", -0.1)):
        y, info = project_reduced_action(
            np.array([0.0, 0.0, 1.0]),
            np.array([[0.0, 0.0, 1.0]]),
            np.array([limit]),
            beta_lower=0.0,
            beta_upper=1.0,
        )
        expected = (
            info["feasible"] and abs(y[2] - 1.0) < 1e-7
            if name == "full_speed"
            else info["feasible"] and abs(y[2] - 0.4) < 1e-7
            if name == "slowdown"
            else not info["feasible"]
        )
        out[name] = {
            "limit": limit,
            "feasible": bool(info["feasible"]),
            "beta": float(y[2]) if info["feasible"] else 0.0,
            "pass": bool(expected),
        }
    return out


def main() -> None:
    a = args()
    lk = ManifoldLookup(a.manifold)
    fk = AviatorClosure(a.model)
    rng = np.random.default_rng(a.seed)
    xdot_nom = np.array([0.08, 0.002], dtype=np.float64)
    rows = []
    for idx in range(a.samples):
        x, _phi, lookup = sample_state(lk, rng)
        q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        e, aq, ax = analytic_constraint_matrices(fk, x, q)
        e_fd, aq_fd, ax_fd = constraint_matrices(fk, x, q)
        _, singular, vh = np.linalg.svd(aq, full_matrices=True)
        n = orient_basis(vh[-2:].T, None)
        pinv = np.linalg.pinv(aq, rcond=1e-10)
        he = pinv @ (-a.feedback * e)
        hx = pinv @ (-ax @ xdot_nom)
        r = rng.uniform(-0.2, 0.2, size=2)
        beta = float(rng.uniform(0.1, 1.0))
        qdot = he + beta * hx + n @ r
        tangent = aq @ qdot + ax @ (beta * xdot_nom) + a.feedback * e
        qp = solve_full_reduced(aq, ax, e, n, xdot_nom, rng)
        gov = governor_cases()
        rows.append({
            "sample": idx,
            "sigma_min_Aq": float(singular[-1]),
            "jacobian_Aq_inf": float(np.max(np.abs(aq - aq_fd))),
            "jacobian_Ax_inf": float(np.max(np.abs(ax - ax_fd))),
            "closure_matrix_inf": float(np.max(np.abs(e - e_fd))),
            "null_residual_inf": float(np.max(np.abs(aq @ n))),
            "tangent_residual_inf": float(np.max(np.abs(tangent))),
            "qp": qp,
            "governor_cases": gov,
            "pass": bool(
                np.max(np.abs(aq - aq_fd)) <= a.tol
                and np.max(np.abs(ax - ax_fd)) <= a.tol
                and np.max(np.abs(tangent)) <= a.tol
                and qp.get("pass", False)
                and qp.get("full_reduced_qdot_inf", np.inf) <= 5e-5
                and qp.get("objective_abs_diff", np.inf) <= 5e-5
                and all(v["pass"] for v in gov.values())
            ),
        })
    report = {
        "kind": "node0_real_mujoco_contract",
        "manifold": str(Path(a.manifold).resolve()),
        "model": str(Path(a.model).resolve()),
        "samples": a.samples,
        "seed": a.seed,
        "tolerance": a.tol,
        "contract_status": "pass" if all(r["pass"] for r in rows) else "fail",
        "scope": "real MuJoCo local contract; no long-horizon collision or global branch claim",
        "rows": rows,
    }
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"contract_status": report["contract_status"], "failed": sum(not r["pass"] for r in rows)}, indent=2))


if __name__ == "__main__":
    main()

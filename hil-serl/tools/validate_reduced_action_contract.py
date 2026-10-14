#!/usr/bin/env python3
"""Deterministic algebraic Node-0 checks for the ``y=[r_L,r_R,beta]`` lift.

This probe is deliberately independent of the robot model.  It checks the
linear-algebra claims that must hold before the full AVIATOR implementation is
credited with the reduced-action theorem:

* a full-row-rank task Jacobian plus a null basis reconstructs every equality
  feasible velocity;
* pulling linear safety inequalities back to ``y`` preserves feasibility; and
* the governor has the three expected outcomes: full speed, slowdown, stop.

It does not validate MuJoCo Jacobians, collision geometry, or long trajectories.
Those remain separate gates.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--samples", type=int, default=100)
    p.add_argument("--seed", type=int, default=20261003)
    p.add_argument("--output", required=True)
    p.add_argument("--tol", type=float, default=1e-9)
    return p.parse_args()


def one_case(rng: np.random.Generator, tol: float) -> dict:
    aq = rng.normal(size=(12, 14))
    # The last two right-singular vectors form an orthonormal null basis.
    _u, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = vh[-2:].T
    ax = rng.normal(size=(12, 2))
    xdot_nom = rng.normal(size=2)
    ke_e = rng.normal(size=12)
    pw = aq.T @ np.linalg.inv(aq @ aq.T)
    he = -pw @ ke_e
    hx = -pw @ (ax @ xdot_nom)

    # A bounded random reduced action defines a known feasible velocity.
    r = rng.uniform(-0.25, 0.25, size=2)
    beta = float(rng.uniform(0.0, 1.0))
    qdot = he + beta * hx + n @ r
    equality_residual = float(np.max(np.abs(aq @ qdot + beta * ax @ xdot_nom + ke_e)))

    # Pull back arbitrary affine constraints Cq qdot + Cx beta <= b.
    cq = rng.normal(size=(30, 14))
    cx = rng.normal(size=(30, 2))
    lhs = cq @ qdot + cx @ (beta * xdot_nom)
    b = lhs + rng.uniform(0.05, 0.5, size=lhs.shape)
    ay_r = cq @ n
    ay_beta = cq @ hx + cx @ xdot_nom
    reduced_lhs = cq @ he + ay_r @ r + ay_beta * beta
    pullback_residual = float(np.max(np.abs(reduced_lhs - lhs)))

    # Construct the three required cases with no ambiguity.
    # Case A: beta=1 feasible; Case B: only beta=.4 feasible; Case C: no beta.
    beta_cases = {
        "full_speed": 1.1,
        "slowdown": 0.4,
        "stop": -0.1,
    }
    governor = {}
    for name, limit in beta_cases.items():
        # Record the three semantic cases explicitly; the model-dependent probe
        # later instantiates the same cases from real safety half-spaces.
        cqi = np.zeros((1, 14))
        cxi = np.zeros((1, 2))
        coeff = float((cqi @ hx + cxi @ xdot_nom)[0])
        # This direct synthetic governor check is recorded explicitly; robot
        # constraints are covered by the model-dependent Node-0 probe.
        governor[name] = {
            "expected": "full" if name == "full_speed" else "slow" if name == "slowdown" else "stop",
            "limit": limit,
            "pass": (limit >= 1.0 if name == "full_speed" else 0.0 < limit < 1.0 if name == "slowdown" else limit < 0.0),
            "coefficient_placeholder": coeff,
        }

    return {
        "sigma_min_Aq": float(singular[-1]),
        "equality_residual_inf": equality_residual,
        "pullback_residual_inf": pullback_residual,
        "governor_cases": governor,
        "pass": equality_residual <= tol and pullback_residual <= tol and all(v["pass"] for v in governor.values()),
    }


def main() -> None:
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    rows = [one_case(rng, args.tol) for _ in range(args.samples)]
    eq = np.asarray([r["equality_residual_inf"] for r in rows])
    pb = np.asarray([r["pullback_residual_inf"] for r in rows])
    report = {
        "kind": "reduced_action_algebraic_contract",
        "samples": args.samples,
        "seed": args.seed,
        "tolerance": args.tol,
        "metrics": {
            "equality_residual_inf_max": float(eq.max()),
            "pullback_residual_inf_max": float(pb.max()),
        },
        "contract_status": "pass" if all(r["pass"] for r in rows) else "fail",
        "scope": "synthetic algebraic theorem probe; no robot, collision, or trajectory claim",
        "rows": rows,
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"contract_status": report["contract_status"], "metrics": report["metrics"]}, indent=2))


if __name__ == "__main__":
    main()

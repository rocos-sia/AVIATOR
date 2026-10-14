#!/usr/bin/env python3
"""Compare the analytic MuJoCo TCP Jacobian with closure-error differences."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state
from tools.validate_online_lift import constraint_matrices


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=20)
    p.add_argument("--seed", type=int, default=4)
    a = p.parse_args()

    lk = ManifoldLookup(a.manifold)
    fk = AviatorClosure(a.model)
    rng = np.random.default_rng(a.seed)
    errors = []
    samples = 0

    for _ in range(a.samples):
        x, _phi, lookup = sample_state(lk, rng)
        q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
        _, aq_fd, _ = constraint_matrices(fk, x, q)

        fk.data.qpos[:] = 0.0
        fk.data.qpos[:2] = x
        fk.data.qpos[2:16] = q
        mujoco.mj_forward(fk.model, fk.data)
        aq_analytic = np.zeros((12, 14), dtype=np.float64)
        for side, site in enumerate(fk.tcp):
            jacp = np.zeros((3, fk.model.nv), dtype=np.float64)
            jacr = np.zeros((3, fk.model.nv), dtype=np.float64)
            mujoco.mj_jacSite(fk.model, fk.data, jacp, jacr, site)
            tcp_rotation = fk.data.site_xmat[site].reshape(3, 3)
            cols = np.arange(2 + side * 7, 2 + (side + 1) * 7)
            aq_analytic[side * 6:side * 6 + 3, side * 7:(side + 1) * 7] = jacp[:, cols]
            # The orientation residual is log(R_tcp^T R_handle), so the
            # world-frame angular velocity is expressed in the TCP frame and
            # enters with the TCP-minus-handle sign.
            aq_analytic[side * 6 + 3:side * 6 + 6, side * 7:(side + 1) * 7] = (
                -tcp_rotation.T @ jacr[:, cols]
            )
        errors.append(float(np.max(np.abs(aq_fd - aq_analytic))))
        samples += 1

    v = np.asarray(errors)
    report = {
        "kind": "analytic_vs_finite_difference_closure_jacobian",
        "manifold": str(Path(a.manifold).resolve()),
        "model": str(Path(a.model).resolve()),
        "samples": samples,
        "median_max_abs": float(np.median(v)),
        "p95_max_abs": float(np.quantile(v, 0.95)),
        "max_max_abs": float(np.max(v)),
        "interpretation": "A_q uses MuJoCo TCP Jacobians and the finite-difference reference uses the independent pose-closure error.",
    }
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

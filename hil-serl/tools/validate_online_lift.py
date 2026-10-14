#!/usr/bin/env python3
"""Minimal online horizontal/vertical lift feasibility prototype.

The prototype constructs the constraint matrices from an independent AVIATOR
rigid-grasp closure error.  In ``analytic`` mode, MuJoCo site Jacobians are
used directly; ``finite`` mode retains the central-difference reference,

    A_q q_dot + A_x x_dot = -K_e e,

then computes the minimum-norm horizontal component and a two-dimensional
oriented null basis:

    q_dot = pinv(A_q)(-A_x x_dot - K_e e) + N r.

It is intentionally a feasibility test, not the final real-time
implementation.  Both modes currently use an unweighted minimum-norm solve.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.validate_differential_lift import AviatorClosure, sample_state


def args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=24)
    p.add_argument("--steps", type=int, default=80)
    p.add_argument("--dt", type=float, default=0.002)
    p.add_argument("--seed", type=int, default=5)
    p.add_argument("--feedback", type=float, default=2.0)
    p.add_argument("--jacobian", choices=("analytic", "finite"), default="analytic")
    return p.parse_args()


def constraint_matrices(fk: AviatorClosure, x: np.ndarray, q: np.ndarray):
    e0 = fk.error(x, q)
    aq = np.empty((12, 14), dtype=np.float64)
    ax = np.empty((12, 2), dtype=np.float64)
    for j in range(14):
        h = 1e-6
        qp, qm = q.copy(), q.copy()
        qp[j] += h
        qm[j] -= h
        aq[:, j] = (fk.error(x, qp) - fk.error(x, qm)) / (2.0 * h)
    for j, h in enumerate((1e-6, 1e-7)):
        xp, xm = x.copy(), x.copy()
        xp[j] += h
        xm[j] -= h
        ax[:, j] = (fk.error(xp, q) - fk.error(xm, q)) / (2.0 * h)
    return e0, aq, ax


def analytic_constraint_matrices(fk: AviatorClosure, x: np.ndarray, q: np.ndarray):
    """Build A_q and A_x from MuJoCo spatial site Jacobians.

    The closure residual is [p_tcp-p_handle, log(R_tcp^T R_handle)].
    MuJoCo reports both linear and angular site velocities in world coordinates.
    The angular rows are therefore rotated into the TCP frame; TCP terms enter
    with a minus sign and handle/task terms with a plus sign.
    """
    e0 = fk.error(x, q)
    fk.data.qpos[:] = 0.0
    fk.data.qpos[:2] = x
    fk.data.qpos[2:16] = q
    mujoco.mj_forward(fk.model, fk.data)
    aq = np.zeros((12, 14), dtype=np.float64)
    ax = np.zeros((12, 2), dtype=np.float64)
    for side, (handle, tcp) in enumerate(zip(fk.handle, fk.tcp)):
        jph = np.zeros((3, fk.model.nv), dtype=np.float64)
        jrh = np.zeros((3, fk.model.nv), dtype=np.float64)
        jpt = np.zeros((3, fk.model.nv), dtype=np.float64)
        jrt = np.zeros((3, fk.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(fk.model, fk.data, jph, jrh, handle)
        mujoco.mj_jacSite(fk.model, fk.data, jpt, jrt, tcp)
        rt = fk.data.site_xmat[tcp].reshape(3, 3)
        # For R=R_tcp^T R_handle, the left relative angular velocity is
        # R_tcp^T (omega_handle-omega_tcp).  d Log(R) needs J_l^{-1}.
        phi = e0[side * 6 + 3:side * 6 + 6]
        angle = float(np.linalg.norm(phi))
        skew = np.array([[0., -phi[2], phi[1]],
                         [phi[2], 0., -phi[0]],
                         [-phi[1], phi[0], 0.]])
        coeff = (1.0 / 12.0 + angle**2 / 720.0 if angle < 1e-4
                 else (1.0 - .5 * angle / np.tan(.5 * angle)) / angle**2)
        log_diff = np.eye(3) - .5 * skew + coeff * (skew @ skew)
        qcols = np.arange(2 + side * 7, 2 + (side + 1) * 7)
        row0 = side * 6
        qslice = slice(side * 7, (side + 1) * 7)
        aq[row0:row0 + 3, qslice] = jpt[:, qcols]
        aq[row0 + 3:row0 + 6, qslice] = -log_diff @ rt.T @ jrt[:, qcols]
        ax[row0:row0 + 3, :] = -jph[:, :2]
        ax[row0 + 3:row0 + 6, :] = log_diff @ rt.T @ jrh[:, :2]
    return e0, aq, ax


def orient_basis(raw: np.ndarray, previous: np.ndarray | None) -> np.ndarray:
    """Align the two-dimensional null basis to its previous orientation."""
    if previous is None:
        return raw
    # Solve min_R ||raw R - previous||_F over orthogonal R.
    u, _, vt = np.linalg.svd(raw.T @ previous)
    return raw @ (u @ vt)


def lift_step(
    fk: AviatorClosure,
    x: np.ndarray,
    q: np.ndarray,
    xdot: np.ndarray,
    r_nom: np.ndarray,
    feedback: float,
    previous_basis: np.ndarray | None,
    jacobian_mode: str,
):
    if jacobian_mode == "analytic":
        e0, aq, ax = analytic_constraint_matrices(fk, x, q)
    else:
        e0, aq, ax = constraint_matrices(fk, x, q)
    u, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous_basis)
    aq_pinv = np.linalg.pinv(aq, rcond=1e-10)
    horizontal = aq_pinv @ (-ax @ xdot - feedback * e0)
    qdot = horizontal + n @ r_nom
    residual = aq @ qdot + ax @ xdot + feedback * e0
    return qdot, n, float(np.max(np.abs(residual))), float(np.max(np.abs(e0))), float(singular[-1])


def stats(values):
    if not values:
        return {"n": 0, "median": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {
        "n": int(a.size),
        "median": float(np.median(a)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(np.max(a)),
    }


def main() -> None:
    a = args()
    lk = ManifoldLookup(a.manifold)
    fk = AviatorClosure(a.model)
    rng = np.random.default_rng(a.seed)
    tangent, sigma, initial_e, final_e = [], [], [], []
    max_e, max_qdot, basis_alignment = [], [], []
    complete = 0
    q_lower = np.concatenate([lk.joint_lower[0], lk.joint_lower[1]])
    q_upper = np.concatenate([lk.joint_upper[0], lk.joint_upper[1]])
    xdot = np.array([0.08, 0.002], dtype=np.float64)
    r_nom = np.array([0.03, -0.025], dtype=np.float64)

    for _ in range(a.samples):
        for _attempt in range(1000):
            x, _phi, lookup = sample_state(lk, rng)
            q = np.concatenate([lookup["qL"][0], lookup["qR"][0]])
            break
        initial_e.append(float(np.max(np.abs(fk.error(x, q)))))
        n_previous = None
        trajectory_max_e = 0.0
        trajectory_max_qdot = 0.0
        trajectory_min_sigma = np.inf
        ok = True
        for _step in range(a.steps):
            qdot, n0, res0, e0, sig0 = lift_step(
                fk, x, q, xdot, r_nom, a.feedback, n_previous, a.jacobian
            )
            tangent.append(res0)
            sigma.append(sig0)
            trajectory_max_e = max(trajectory_max_e, e0)
            trajectory_max_qdot = max(trajectory_max_qdot, float(np.max(np.abs(qdot))))
            trajectory_min_sigma = min(trajectory_min_sigma, sig0)
            if n_previous is not None:
                basis_alignment.append(float(np.min(np.linalg.svd(n_previous.T @ n0)[1])))

            dt = a.dt
            x_mid = x + 0.5 * dt * xdot
            q_mid = q + 0.5 * dt * qdot
            if np.any(q_mid < q_lower - 0.05) or np.any(q_mid > q_upper + 0.05):
                ok = False
                break
            qdot_mid, n_mid, res_mid, e_mid, sig_mid = lift_step(
                fk, x_mid, q_mid, xdot, r_nom, a.feedback, n0, a.jacobian
            )
            tangent.append(res_mid)
            sigma.append(sig_mid)
            trajectory_max_e = max(trajectory_max_e, e_mid)
            trajectory_max_qdot = max(trajectory_max_qdot, float(np.max(np.abs(qdot_mid))))
            trajectory_min_sigma = min(trajectory_min_sigma, sig_mid)
            q = q + dt * qdot_mid
            x = x + dt * xdot
            n_previous = n_mid
        if ok:
            complete += 1
        trajectory_max_e = max(trajectory_max_e, float(np.max(np.abs(fk.error(x, q)))))
        final_e.append(float(np.max(np.abs(fk.error(x, q)))))
        max_e.append(trajectory_max_e)
        max_qdot.append(trajectory_max_qdot)

    report = {
        "kind": "minimal_online_horizontal_vertical_lift_feasibility",
        "manifold": str(Path(a.manifold).resolve()),
        "model": str(Path(a.model).resolve()),
        "samples": a.samples,
        "complete_trajectories": complete,
        "steps": a.steps,
        "dt": a.dt,
        "feedback_gain": a.feedback,
        "jacobian_mode": a.jacobian,
        "xdot": xdot.tolist(),
        "r_nom": r_nom.tolist(),
        "tangent_residual": stats(tangent),
        "sigma_min_Aq": stats(sigma),
        "initial_closure_error_mixed_m_rad": stats(initial_e),
        "max_closure_error_mixed_m_rad": stats(max_e),
        "final_closure_error_mixed_m_rad": stats(final_e),
        "max_joint_speed_rad_s": stats(max_qdot),
        "null_basis_alignment_min_singular": stats(basis_alignment),
        "interpretation": [
            (
                "A_q uses analytic MuJoCo site Jacobians and A_x uses the "
                "analytic handle-coordinate Jacobian."
                if a.jacobian == "analytic"
                else "A_q and A_x use central differences of an independent MuJoCo closure error."
            ),
            "The solve uses W=I and is a prototype for the weighted horizontal lift.",
            "No collision-gradient phase filter or global branch guarantee is included.",
        ],
    }
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

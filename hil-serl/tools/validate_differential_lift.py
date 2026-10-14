#!/usr/bin/env python3
"""Low-cost feasibility check for the differential-lift equations.

This is a representation-level and kinematic check.  It does not claim that
the stored LUT is the proposed online lift.  It checks whether the existing
AVIATOR Q(x, phi) data and MuJoCo model are consistent with

    q_dot = Q_x x_dot + Q_phi phi_dot

and whether that velocity is tangent to the two rigid grasp constraints.
The script deliberately uses an independent MuJoCo forward-kinematics
evaluation instead of trusting the stored task-error or clearance fields.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True, help="manifold_phi directory")
    p.add_argument("--model", required=True, help="MuJoCo aviator.xml")
    p.add_argument("--output", required=True, help="JSON report path")
    p.add_argument("--samples", type=int, default=64)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--dt", type=float, default=0.002)
    p.add_argument("--steps", type=int, default=80)
    return p.parse_args()


class AviatorClosure:
    """Independent FK and rigid-grasp closure residual for the AVIATOR model."""

    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.handle = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_handle"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_handle"),
        ]
        self.tcp = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_tcp"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_tcp"),
        ]
        if min(self.handle + self.tcp) < 0:
            raise RuntimeError("AVIATOR handle/TCP sites were not found")
        if self.model.nq < 16:
            raise RuntimeError(f"expected at least 16 qpos entries, got {self.model.nq}")

    def error(self, x: np.ndarray, q14: np.ndarray) -> np.ndarray:
        """Return [position error (m), orientation log error (rad)] for both arms."""
        self.data.qpos[:] = 0.0
        self.data.qpos[0:2] = x
        self.data.qpos[2:16] = q14
        mujoco.mj_forward(self.model, self.data)
        out: list[float] = []
        for h, t in zip(self.handle, self.tcp):
            out.extend((self.data.site_xpos[t] - self.data.site_xpos[h]).tolist())
            rt = self.data.site_xmat[t].reshape(3, 3)
            rh = self.data.site_xmat[h].reshape(3, 3)
            out.extend(Rotation.from_matrix(rt.T @ rh).as_rotvec().tolist())
        return np.asarray(out, dtype=np.float64)


def finite_q(lk: ManifoldLookup, x: np.ndarray, phi: np.ndarray) -> np.ndarray:
    r = lk.query(x[None, :], phi[None, :], check_safe=False)
    return np.concatenate([r["qL"][0], r["qR"][0]])


def sample_state(lk: ManifoldLookup, rng: np.random.Generator):
    # Stay away from grid boundaries and the edge of the safe phase interval so
    # central differences and the short integration remain in one chart cell.
    for _ in range(10000):
        x = np.array([
            rng.uniform(lk.th_min + 0.04, lk.th_max - 0.04),
            rng.uniform(lk.s_min + 0.008, lk.s_max - 0.008),
        ])
        box = lk.safe_interval(x[None, :])
        lo, hi = box["phi_safe_lo"][0], box["phi_safe_hi"][0]
        # The old AVIATOR phase field has narrow but valid safe arcs (typically
        # about 0.1 rad wide).  Require a positive interval and leave a 10%
        # margin for the finite-difference and integration probes.
        if np.any(hi - lo <= 1e-3):
            continue
        phi = rng.uniform(lo + 0.1 * (hi - lo), hi - 0.1 * (hi - lo))
        r = lk.query(x[None, :], phi[None, :], check_safe=False)
        if int(r["branch"][0]) == 0:
            return x, phi, r
    raise RuntimeError("could not sample a branch-consistent interior state")


def main() -> None:
    args = parse_args()
    lk = ManifoldLookup(args.manifold)
    fk = AviatorClosure(args.model)
    rng = np.random.default_rng(args.seed)

    q_deriv_errors: list[float] = []
    qx_theta_errors: list[float] = []
    qx_slide_errors: list[float] = []
    qphi_errors: list[float] = []
    closure_errors: list[float] = []
    tangent_errors: dict[str, list[float]] = {"1e-04": [], "1e-05": [], "1e-06": []}
    integration_errors: list[float] = []
    integration_closure_errors: list[float] = []
    accepted = 0

    for _ in range(args.samples):
        x, phi, r0 = sample_state(lk, rng)
        q0 = np.concatenate([r0["qL"][0], r0["qR"][0]])
        closure_errors.append(float(np.max(np.abs(fk.error(x, q0)))))

        xdot = np.array([0.12, 0.004], dtype=np.float64)
        phidot = np.array([0.04, -0.03], dtype=np.float64)
        qdot = r0["Q_x"][0] @ xdot + r0["Q_phi"][0] @ phidot

        # Check stored Q_x/Q_phi against independent finite differences of the
        # interpolated q field.  The perturbations are small enough to remain in
        # the same local chart cell for the selected interior states.
        fd = []
        for j, h in enumerate((1e-5, 1e-6)):
            xp, xm = x.copy(), x.copy()
            xp[j] += h
            xm[j] -= h
            fd.append((finite_q(lk, xp, phi) - finite_q(lk, xm, phi)) / (2.0 * h))
        qx_fd = np.column_stack(fd)
        qx_tab = r0["Q_x"][0]
        q_deriv_errors.append(float(np.max(np.abs(qx_fd - qx_tab))))
        qx_theta_errors.append(float(np.max(np.abs(qx_fd[:, 0] - qx_tab[:, 0]))))
        qx_slide_errors.append(float(np.max(np.abs(qx_fd[:, 1] - qx_tab[:, 1]))))

        pfd = []
        for arm in range(2):
            pp, pm = phi.copy(), phi.copy()
            pp[arm] += 1e-5
            pm[arm] -= 1e-5
            pfd.append((finite_q(lk, x, pp) - finite_q(lk, x, pm)) / 2e-5)
        qphi_fd = np.column_stack(pfd)
        qphi_tab = r0["Q_phi"][0]
        qphi_errors.append(float(np.max(np.abs(qphi_fd - qphi_tab))))

        # The full constraint equation is checked by independently advancing
        # task coordinates and joints, then differentiating the FK closure error.
        for hs in (1e-4, 1e-5, 1e-6):
            e0 = fk.error(x, q0)
            e1 = fk.error(x + xdot * hs, q0 + qdot * hs)
            tangent_errors[f"{hs:.0e}"].append(float(np.max(np.abs((e1 - e0) / hs))))

        # Short midpoint integration in the stored local chart.  The direct
        # Q(x, phi) query is the independent representation reference.
        x_i, phi_i, q_i = x.copy(), phi.copy(), q0.copy()
        ok_steps = 0
        for _step in range(args.steps):
            xm = x_i + 0.5 * args.dt * xdot
            phim = phi_i + 0.5 * args.dt * phidot
            if not (lk.th_min < xm[0] < lk.th_max and lk.s_min < xm[1] < lk.s_max):
                break
            rm = lk.query(xm[None, :], phim[None, :], check_safe=False)
            if int(rm["branch"][0]) != 0:
                break
            qdot_m = rm["Q_x"][0] @ xdot + rm["Q_phi"][0] @ phidot
            q_i = q_i + args.dt * qdot_m
            x_i = x_i + args.dt * xdot
            phi_i = phi_i + args.dt * phidot
            ok_steps += 1
        if ok_steps == args.steps:
            q_ref = finite_q(lk, x_i, phi_i)
            integration_errors.append(float(np.max(np.abs(q_i - q_ref))))
            integration_closure_errors.append(float(np.max(np.abs(fk.error(x_i, q_i)))))
            accepted += 1

    def stats(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {"n": 0, "median": None, "p95": None, "max": None}
        a = np.asarray(values, dtype=np.float64)
        return {
            "n": int(a.size),
            "median": float(np.median(a)),
            "p95": float(np.quantile(a, 0.95)),
            "max": float(np.max(a)),
        }

    report = {
        "kind": "representation_and_kinematic_differential_lift_feasibility",
        "manifold": str(Path(args.manifold).resolve()),
        "model": str(Path(args.model).resolve()),
        "samples_requested": args.samples,
        "samples_with_complete_integration": accepted,
        "integration_dt": args.dt,
        "integration_steps": args.steps,
        "seed": args.seed,
        "stored_Qx_central_difference_max_abs_rad_or_rad_per_m": stats(q_deriv_errors),
        "stored_Qx_theta_central_difference_max_abs": stats(qx_theta_errors),
        "stored_Qx_slide_central_difference_max_abs": stats(qx_slide_errors),
        "stored_Qphi_central_difference_max_abs_rad_per_rad": stats(qphi_errors),
        "stored_table_pose_closure_error_mixed_m_rad": stats(closure_errors),
        "constraint_tangent_error_mixed_m_per_s_rad_per_s": {
            h: stats(v) for h, v in tangent_errors.items()
        },
        "midpoint_q_vs_direct_Q_max_abs_rad": stats(integration_errors),
        "midpoint_integrated_pose_closure_error_mixed_m_rad": stats(integration_closure_errors),
        "interpretation": [
            "This validates the local Q_x/Q_phi representation and FK tangent residual only.",
            "It does not establish global branch completeness or full collision safety.",
            "The stored manifold is a baseline representation, not the proposed online lift implementation.",
        ],
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

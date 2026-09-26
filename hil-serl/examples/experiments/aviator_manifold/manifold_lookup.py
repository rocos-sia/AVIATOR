"""Q(x, phi) manifold lookup -- vectorized batch query (Task 1.2, BLOCKER #3).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 0.2 / Task 1.2)

Loads the binary manifold produced by ``build_manifold_phi`` (Task 0.2) and
answers batch queries

    (x, phi) -> qL, qR, dL, dR, d_min, safe margins, assembled Q_x, Q_phi

Canonical storage layout (matches the C++ producer, ``clearance_trajectory.cpp``)
--------------------------------------------------------------------------------
``theta`` varies FASTEST within a grid row, ``s`` varies SLOWEST (outer)::

    grid point      gp(i_theta, i_s) = i_s * n_theta + i_theta
    sample index    k(i_theta, i_s, i_phi) = gp * n_phi + i_phi

    qL.bin     [k, joint]           (7)      qR.bin     [k, joint]
    dL.bin     [k]                  (1)      dR.bin     [k]
    QxL.bin    [k, joint, col]      (7, 2)   col 0 = dq/dtheta, col 1 = dq/ds
    QphiL.bin  [k, joint, 0]        (7, 1)
    safe.bin   [gp, 0:4] = [phi_safe_lo_L, phi_safe_hi_L, phi_safe_lo_R, phi_safe_hi_R]
    branch.bin [gp]                 uint8, 0 = cycle-consistent

Interpolation
-------------
The sampler is trilinear: bilinear over the ``(theta, s)`` grid times linear in
``phi`` (``phi.bin`` is a single uniform grid shared by every ``(theta, s)``).
The per-arm derivatives ``Q_x``/``Q_phi`` are interpolated the same way.

Assembly at query time
----------------------
``Q_x`` is a vertical stack (theta, s are shared task variables)::

    Q_x   = [Q_x,L]   in R^{14x2}
            [Q_x,R]

``Q_phi`` is block diagonal (phi_dot_L only moves the left arm)::

    Q_phi = [Q_phi,L  0      ]   in R^{14x2}
            [0        Q_phi,R]

Joint-limit margin ``m_q``
--------------------------
``m_q`` is the worst-case joint-range margin over both arms, used by the reward
and the safety filter.  Joint limits are read from the optional manifest keys
``joint_lower`` / ``joint_upper`` (2x7 radians); when absent the Aviator v0.1
defaults (mirroring ``clearance_trajectory.cpp``) are used.
"""

from __future__ import annotations

import json
import os

import numpy as np

__all__ = ["ManifoldLookup", "DEFAULT_JOINT_LOWER", "DEFAULT_JOINT_UPPER"]

# Aviator v0.1 joint limits (radians), mirroring clearance_trajectory.cpp:
#   jnt_range of the aviator model, with joint2 overridden by posture.json
#   (joint2_limits_deg = [85, 95] +/- 0.5 deg planning margin).
DEFAULT_JOINT_LOWER = np.array([
    [-3.1067, np.deg2rad(84.5), -3.1067, -1.0472, -3.1067, -1.0472, -1.0472],
    [-3.1067, np.deg2rad(84.5), -3.1067, -1.0472, -3.1067, -1.0472, -1.0472],
], dtype=np.float64)
DEFAULT_JOINT_UPPER = np.array([
    [3.1067, np.deg2rad(94.5), 3.1067, 2.5307, 3.1067, 1.0472, 1.0472],
    [3.1067, np.deg2rad(94.5), 3.1067, 2.5307, 3.1067, 1.0472, 1.0472],
], dtype=np.float64)

# grid-boundary slack so that a query written as exactly a node does not trip on
# float round-off in the manifest-derived axis.
_TOL = 1e-9


class ManifoldLookup:
    """Read-only lookup over the transported-phase Q(x, phi) manifold."""

    def __init__(self, manifold_dir: str):
        self.dir = os.fspath(manifold_dir)
        with open(os.path.join(self.dir, "manifest.json")) as f:
            self.manifest = json.load(f)
        m = self.manifest

        self.n_theta = int(m["n_theta"])
        self.n_s = int(m["n_s"])
        self.n_phi = int(m["n_phi"])
        self.L_phi = np.array([float(m["L_phi_L"]), float(m["L_phi_R"])], dtype=np.float64)
        self.anchor = np.array([float(m["anchor_theta"]), float(m["anchor_s"])], dtype=np.float64)
        self.th_min = float(m["th_min"])
        self.th_max = float(m["th_max"])
        self.s_min = float(m["s_min"])
        self.s_max = float(m["s_max"])

        self.theta_axis = np.linspace(self.th_min, self.th_max, self.n_theta)
        self.s_axis = np.linspace(self.s_min, self.s_max, self.n_s)
        self.phi_axis = self._load("phi.bin").astype(np.float64)

        n_gp = self.n_theta * self.n_s          # grid points (no phi)
        n_pt = n_gp * self.n_phi                # samples (with phi)

        # per-arm joint configs / clearance / derivatives
        self.qL = self._load("qL.bin", (n_pt, 7))
        self.qR = self._load("qR.bin", (n_pt, 7))
        self.dL = self._load("dL.bin", (n_pt,))
        self.dR = self._load("dR.bin", (n_pt,))
        self.QxL = self._load("QxL.bin", (n_pt, 7, 2))
        self.QxR = self._load("QxR.bin", (n_pt, 7, 2))
        self.QphiL = self._load("QphiL.bin", (n_pt, 7, 1))
        self.QphiR = self._load("QphiR.bin", (n_pt, 7, 1))

        # per-grid-point (theta, s) data
        self.safe = self._load("safe.bin", (n_gp, 4))
        self.branch = self._load("branch.bin", (n_gp,), dtype=np.uint8)

        self.joint_lower, self.joint_upper = self._joint_limits()

    # -- construction helpers ------------------------------------------------
    def _load(self, name: str, shape=None, dtype=np.float32) -> np.ndarray:
        path = os.path.join(self.dir, name)
        arr = np.fromfile(path, dtype=dtype)
        if shape is not None:
            if arr.size != int(np.prod(shape)):
                raise ValueError(
                    f"{name}: expected {int(np.prod(shape))} elements for shape {shape}, "
                    f"got {arr.size} ({path})"
                )
            arr = arr.reshape(shape)
        return arr

    def _joint_limits(self):
        m = self.manifest
        if "joint_lower" in m and "joint_upper" in m:
            lo = np.asarray(m["joint_lower"], dtype=np.float64).reshape(2, 7)
            hi = np.asarray(m["joint_upper"], dtype=np.float64).reshape(2, 7)
            return lo, hi
        return DEFAULT_JOINT_LOWER.copy(), DEFAULT_JOINT_UPPER.copy()

    # -- grid indexing -------------------------------------------------------
    @staticmethod
    def _frac(axis: np.ndarray, v: np.ndarray):
        """Lower index + linear weight of ``v`` in a uniformly-ish sampled axis."""
        n = axis.shape[0]
        i0 = np.clip(np.searchsorted(axis, v, side="right") - 1, 0, n - 2)
        a = axis[i0]
        b = axis[i0 + 1]
        w = (v - a) / (b - a)
        return i0, w

    def _trilinear(self, arr, i_th, i_s, i_p, w_th, w_s, w_p):
        """Trilinear blend of ``arr`` (n_pt, ...) at the 8 surrounding corners."""
        n_theta, n_phi = self.n_theta, self.n_phi
        w1_th, w1_s, w1_p = w_th, w_s, w_p
        w0_th, w0_s, w0_p = 1.0 - w_th, 1.0 - w_s, 1.0 - w_p
        out = None
        for dth in (0, 1):
            for ds in (0, 1):
                row = (i_s + ds) * n_theta + (i_th + dth)
                for dp in (0, 1):
                    idx = row * n_phi + (i_p + dp)
                    w = (w1_th if dth else w0_th) * (w1_s if ds else w0_s) * (w1_p if dp else w0_p)
                    term = arr[idx] * w.reshape((w.shape[0],) + (1,) * (arr.ndim - 1))
                    out = term if out is None else out + term
        return out

    def _bilinear(self, arr, i_th, i_s, w_th, w_s):
        """Bilinear blend of ``arr`` (n_gp, ...) -- for (theta, s)-only fields."""
        n_theta = self.n_theta
        out = None
        for dth in (0, 1):
            for ds in (0, 1):
                idx = (i_s + ds) * n_theta + (i_th + dth)
                w = (w_th if dth else 1.0 - w_th) * (w_s if ds else 1.0 - w_s)
                term = arr[idx] * w.reshape((w.shape[0],) + (1,) * (arr.ndim - 1))
                out = term if out is None else out + term
        return out

    # -- public API ----------------------------------------------------------
    def safe_interval(self, x: np.ndarray) -> dict:
        """Return the per-arm safe-phase interval at a batch of task states.

        Parameters
        ----------
        x : (B, 2) array -- ``(theta, s)``

        Returns
        -------
        dict with ``phi_safe_lo`` (B,2), ``phi_safe_hi`` (B,2), ``branch`` (B,).
        This is the bilinear (theta, s) interpolation of ``safe.bin`` and does
        *not* require a phase argument -- it is used by the safety filter to
        clip ``phi_{t+1}`` into the safe box at ``x_next``.
        """
        x = np.asarray(x, dtype=np.float64)
        if x.ndim != 2 or x.shape[1] != 2:
            raise ValueError(f"x must have shape (B, 2), got {x.shape}")
        th, s = x[:, 0], x[:, 1]
        bad_x = ((th < self.th_min - _TOL) | (th > self.th_max + _TOL)
                 | (s < self.s_min - _TOL) | (s > self.s_max + _TOL))
        if bad_x.any():
            i = int(np.flatnonzero(bad_x)[0])
            raise ValueError(
                f"x[{i}] = (theta={th[i]:.6g}, s={s[i]:.6g}) outside grid "
                f"theta in [{self.th_min}, {self.th_max}], s in [{self.s_min}, {self.s_max}]"
            )
        i_th, w_th = self._frac(self.theta_axis, th)
        i_s, w_s = self._frac(self.s_axis, s)
        safe = self._bilinear(self.safe, i_th, i_s, w_th, w_s)   # (B, 4)
        branch = np.maximum.reduce([
            self.branch[(i_s + ds) * self.n_theta + (i_th + dth)]
            for ds in (0, 1) for dth in (0, 1)
        ])
        return {
            "phi_safe_lo": safe[:, [0, 2]],
            "phi_safe_hi": safe[:, [1, 3]],
            "branch": branch,
        }

    def query(self, x: np.ndarray, phi: np.ndarray, check_safe: bool = True) -> dict:
        """Query the manifold at a batch of states.

        Parameters
        ----------
        x : (B, 2) array -- ``(theta, s)``
        phi : (B, 2) array -- ``(phi_L, phi_R)`` in raw transported-phase units

        Returns
        -------
        dict with ``qL`` (B,7), ``qR`` (B,7), ``dL`` (B,), ``dR`` (B,),
        ``d_min`` (B,), ``m_phi_minus`` (B,2), ``m_phi_plus`` (B,2), ``m_q`` (B,),
        ``phi_safe_lo`` (B,2), ``phi_safe_hi`` (B,2), ``Q_x`` (B,14,2),
        ``Q_phi`` (B,14,2), ``branch`` (B,).

        Parameters
        ----------
        check_safe : bool
            when False, the safe-interval membership check is skipped (the
            derivatives ``Q_x``/``Q_phi`` and ``dL``/``dR`` are still returned).
            Used by the safety filter to linearize at ``x_next`` while the
            current phase may be outside that state's safe box.

        Raises
        ------
        ValueError
            if any ``(x, phi)`` lies outside the grid, or (when ``check_safe``)
            outside the arm's safe interval.
        """
        x = np.asarray(x, dtype=np.float64)
        phi = np.asarray(phi, dtype=np.float64)
        if x.ndim != 2 or x.shape[1] != 2:
            raise ValueError(f"x must have shape (B, 2), got {x.shape}")
        if phi.ndim != 2 or phi.shape[1] != 2:
            raise ValueError(f"phi must have shape (B, 2), got {phi.shape}")
        if x.shape[0] != phi.shape[0]:
            raise ValueError(f"x and phi batch sizes differ: {x.shape[0]} vs {phi.shape[0]}")

        B = x.shape[0]
        th, s = x[:, 0], x[:, 1]
        pL, pR = phi[:, 0], phi[:, 1]

        # --- range checks ---------------------------------------------------
        bad_x = ((th < self.th_min - _TOL) | (th > self.th_max + _TOL)
                 | (s < self.s_min - _TOL) | (s > self.s_max + _TOL))
        if bad_x.any():
            i = int(np.flatnonzero(bad_x)[0])
            raise ValueError(
                f"x[{i}] = (theta={th[i]:.6g}, s={s[i]:.6g}) outside grid "
                f"theta in [{self.th_min}, {self.th_max}], s in [{self.s_min}, {self.s_max}]"
            )
        p_lo, p_hi = self.phi_axis[0], self.phi_axis[-1]
        bad_p = ((pL < p_lo - _TOL) | (pL > p_hi + _TOL)
                 | (pR < p_lo - _TOL) | (pR > p_hi + _TOL))
        if bad_p.any():
            i = int(np.flatnonzero(bad_p)[0])
            raise ValueError(
                f"phi[{i}] = ({pL[i]:.6g}, {pR[i]:.6g}) outside the phi grid [{p_lo}, {p_hi}]"
            )

        # --- fractional grid coordinates ------------------------------------
        i_th, w_th = self._frac(self.theta_axis, th)
        i_s, w_s = self._frac(self.s_axis, s)
        i_pL, w_pL = self._frac(self.phi_axis, pL)
        i_pR, w_pR = self._frac(self.phi_axis, pR)

        # --- per-(theta, s) fields (bilinear only) --------------------------
        safe = self._bilinear(self.safe, i_th, i_s, w_th, w_s)   # (B, 4)
        phi_safe_lo = safe[:, [0, 2]]
        phi_safe_hi = safe[:, [1, 3]]

        bad_safe = ((pL < phi_safe_lo[:, 0] - _TOL) | (pL > phi_safe_hi[:, 0] + _TOL)
                    | (pR < phi_safe_lo[:, 1] - _TOL) | (pR > phi_safe_hi[:, 1] + _TOL))
        if check_safe and bad_safe.any():
            i = int(np.flatnonzero(bad_safe)[0])
            raise ValueError(
                f"phi[{i}] = ({pL[i]:.6g}, {pR[i]:.6g}) outside the safe interval "
                f"L=[{phi_safe_lo[i, 0]:.6g}, {phi_safe_hi[i, 0]:.6g}], "
                f"R=[{phi_safe_lo[i, 1]:.6g}, {phi_safe_hi[i, 1]:.6g}] at "
                f"x=({th[i]:.6g}, {s[i]:.6g})"
            )

        # conservative: any stencil corner marked excluded flags the query
        branch = np.maximum.reduce([
            self.branch[(i_s + ds) * self.n_theta + (i_th + dth)]
            for ds in (0, 1) for dth in (0, 1)
        ])

        # --- per-arm samples (trilinear, each arm's own phi) ----------------
        qL = self._trilinear(self.qL, i_th, i_s, i_pL, w_th, w_s, w_pL)
        qR = self._trilinear(self.qR, i_th, i_s, i_pR, w_th, w_s, w_pR)
        dL = self._trilinear(self.dL, i_th, i_s, i_pL, w_th, w_s, w_pL)
        dR = self._trilinear(self.dR, i_th, i_s, i_pR, w_th, w_s, w_pR)
        Qx_L = self._trilinear(self.QxL, i_th, i_s, i_pL, w_th, w_s, w_pL)
        Qx_R = self._trilinear(self.QxR, i_th, i_s, i_pR, w_th, w_s, w_pR)
        Qphi_L = self._trilinear(self.QphiL, i_th, i_s, i_pL, w_th, w_s, w_pL)
        Qphi_R = self._trilinear(self.QphiR, i_th, i_s, i_pR, w_th, w_s, w_pR)

        # --- assembly -------------------------------------------------------
        Q_x = np.zeros((B, 14, 2))
        Q_x[:, 0:7, :] = Qx_L
        Q_x[:, 7:14, :] = Qx_R

        Q_phi = np.zeros((B, 14, 2))
        Q_phi[:, 0:7, 0:1] = Qphi_L
        Q_phi[:, 7:14, 1:2] = Qphi_R

        # --- margins --------------------------------------------------------
        m_phi_minus = np.stack([pL - phi_safe_lo[:, 0], pR - phi_safe_lo[:, 1]], axis=1)
        m_phi_plus = np.stack([phi_safe_hi[:, 0] - pL, phi_safe_hi[:, 1] - pR], axis=1)

        mL = np.minimum(qL - self.joint_lower[0], self.joint_upper[0] - qL).min(axis=1)
        mR = np.minimum(qR - self.joint_lower[1], self.joint_upper[1] - qR).min(axis=1)
        m_q = np.minimum(mL, mR)

        return {
            "qL": qL,
            "qR": qR,
            "dL": np.asarray(dL, dtype=np.float64),
            "dR": np.asarray(dR, dtype=np.float64),
            "d_min": np.minimum(dL, dR),
            "m_phi_minus": m_phi_minus,
            "m_phi_plus": m_phi_plus,
            "m_q": m_q,
            "phi_safe_lo": phi_safe_lo,
            "phi_safe_hi": phi_safe_hi,
            "Q_x": Q_x,
            "Q_phi": Q_phi,
            "branch": branch,
        }

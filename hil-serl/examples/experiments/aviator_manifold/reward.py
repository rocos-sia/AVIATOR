"""Redundancy-phase reward (Task 1.3, BLOCKER #6).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

The reward favors reserve inside the stored feasible phase region and a
clearance margin before the 5 mm failure threshold. It never measures distance
to a DP trajectory. Costs are normalized to useful per-step scales.

    R_i        = 2·min(m_phi_i^-, m_phi_i^+) / (m_phi_i^- + m_phi_i^+)   in [0,1]
    R_reserve  = min(R_L, R_R)                    (bilateral soft-min, NOT average)

    C_v        = ||phi_dot_exec / PHI_DOT_MAX||^2
    C_a        = ||(phi_dot_exec - a_prev) / DPHI_DOT_MAX||^2
    C_margin   = clip((10 mm - d_min) / 5 mm, 0, 1)^2

    reward     = w_R·R_reserve + ALIVE_BONUS - w_v·C_v - w_a·C_a - w_m·C_margin

A surviving step nets roughly ``ALIVE_BONUS + w_R·R_reserve ≈ +0.3`` (R_reserve
is ~0.05 in the DP demos: the teacher rides the safe boundary, not the centre).
The hard limits (``d_min < 5 mm``, ``max|qdot| > 1.5``) are *termination* events:
the env replaces the shaped reward of that step with ``TERMINAL_PENALTY`` (a
one-time hard penalty), rather than a shaped per-step term.

Calibration notes (2026-09-23, grounded in the 99 DP-demo CSVs):

* ``PHI_DOT_MAX = 1.5`` equals ``phi_dot_scale``; both arms moving at full
  range produce ``C_v = 2``.
* ``DPHI_DOT_MAX = 0.5`` rad/s per step.  The demo's ``phi_dot`` is quantized to
  {0, 0.5, 1.0} with 94% holds; a "reposition" move is one 0.5 rad/s jump per
  0.01 s step, i.e. **50 rad/s^2** (not the 10-20 rad/s^2 the plan guessed).
  Setting ``DPHI_DOT_MAX=0.5`` makes the teacher's *own* reposition move
  ``C_a = 1``, so a good policy pays ``w_a·1 = 0.05`` -- not the ~11x blow-up a
  15 rad/s^2 reference would impose on the demo's real moves.
* ``ALIVE_BONUS = 0.25`` favors completing the finite task trajectory.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "reward_fn",
    "PHI_DOT_MAX",
    "DPHI_DOT_MAX",
    "ALIVE_BONUS",
    "TERMINAL_PENALTY",
]

# dimensionless normalisation scales (rad/s), see module docstring
PHI_DOT_MAX = 1.5        # phase-velocity scale for C_v / C_filter
DPHI_DOT_MAX = 0.5       # per-step phase-velocity *change* scale for C_a (= 50 rad/s^2)
ALIVE_BONUS = 0.25       # constant survival bonus per surviving step
TERMINAL_PENALTY = -100.0  # one-time hard cost for feasibility violations


def reward_fn(
    x,
    phi_dot_nom,
    phi_dot_safe,
    d_min,
    m_phi_minus,
    m_phi_plus,
    m_q,
    lookup,
    *,
    w_R: float = 1.0,
    w_v: float = 0.05,
    w_a: float = 0.05,
    w_f: float = 0.0,  # deprecated; no filter cost in unshielded training
    w_m: float = 0.1,
    d_safe: float = 0.005,
    margin_width: float = 0.005,
    a_prev=None,
    a_prev2=None,   # deprecated: jerk term removed (was w_j); kept for call-site compat
    dt: float = 0.01,
) -> tuple[float, dict]:
    """Compute the per-step (shaped, non-terminal) reward.

    Parameters
    ----------
    x           : (2,) current task state ``(theta, s)``.
    phi_dot_nom : (2,) executed phase velocity (kept for call-site compatibility).
    phi_dot_safe: (2,) same executed velocity; no safety filter is used.
    d_min       : scalar, ``min(dL, dR)`` (m).
    m_phi_minus : (2,) margin to the safe lower bound per arm.
    m_phi_plus  : (2,) margin to the safe upper bound per arm.
    m_q         : scalar, worst-case joint-limit margin (rad).
    lookup      : ``ManifoldLookup`` (reserved for future reward terms).
    w_R/w_v/w_a/w_m : weights for reserve, velocity, acceleration and clearance.
    a_prev      : previous executed velocity (rad/s) for C_a.
    a_prev2     : unused (jerk penalty removed).
    dt          : step size (only kept for signature compat).

    Returns
    -------
    (reward, info).  ``info`` carries the named components for logging.
    """
    phi_dot_safe = np.asarray(phi_dot_safe, dtype=np.float64)
    phi_dot_nom = np.asarray(phi_dot_nom, dtype=np.float64)
    m_phi_minus = np.asarray(m_phi_minus, dtype=np.float64)
    m_phi_plus = np.asarray(m_phi_plus, dtype=np.float64)

    # R_i = 2·min(m^-, m^+)/(m^- + m^+)  -> 1.0 at center, 0.0 at the boundary
    width = m_phi_minus + m_phi_plus
    width_safe = np.where(width < 1e-12, 1.0, width)
    R_i = np.clip(2.0 * np.minimum(m_phi_minus, m_phi_plus) / width_safe, 0.0, 1.0)
    R_reserve = float(np.minimum(R_i[0], R_i[1]))

    # dimensionless penalties (all in [0, ~1] by construction)
    C_v = float(np.sum((phi_dot_safe / PHI_DOT_MAX) ** 2))
    # 5–10 mm is a graded warning layer; d < 5 mm is handled by the env as
    # a hard failure. The warning is defined by clearance, not a DP trajectory.
    C_margin = float(np.clip((d_safe + margin_width - d_min) / margin_width,
                             0.0, 1.0) ** 2)

    C_a = 0.0
    if a_prev is not None:
        delta = (phi_dot_safe - np.asarray(a_prev, dtype=np.float64)) / DPHI_DOT_MAX
        C_a = float(np.sum(delta ** 2))

    reward = w_R * R_reserve + ALIVE_BONUS - w_v * C_v - w_a * C_a - w_m * C_margin

    info = {
        "R_reserve": R_reserve,
        "R_L": float(R_i[0]),
        "R_R": float(R_i[1]),
        "C_v": C_v,
        "C_a": C_a,
        "C_margin": C_margin,
        "reward": reward,
    }
    return reward, info

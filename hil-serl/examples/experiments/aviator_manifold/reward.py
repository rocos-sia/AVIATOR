"""Redundancy-phase reward (Task 1.3, BLOCKER #6).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

The reward rewards the policy for keeping each arm in the *middle* of its
per-arm safe phase interval, and penalizes fast / jerky / filtered motion.

    R_i        = 2·min(m_phi_i^-, m_phi_i^+) / (m_phi_i^- + m_phi_i^+)   in [0,1]
    R_reserve  = min(R_L, R_R)                    (bilateral soft-min, NOT average)
    reward     = w_R·R_reserve - w_v·C_v - w_a·C_a - w_j·C_j - w_f·C_filter

The hard limits (``d_min < 5 mm``, ``max|qdot| > 1.5``) are *termination* events,
handled by the env -- not a shaped reward.
"""

from __future__ import annotations

import numpy as np

__all__ = ["reward_fn"]


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
    w_v: float = 1.0,
    w_a: float = 0.1,
    w_j: float = 0.05,
    w_f: float = 0.5,
    a_prev=None,
    a_prev2=None,
    dt: float = 0.01,
) -> tuple[float, dict]:
    """Compute the per-step reward.

    Parameters
    ----------
    x           : (2,) current task state ``(theta, s)``.
    phi_dot_nom : (2,) nominal phase velocity (policy output).
    phi_dot_safe: (2,) filtered phase velocity.
    d_min       : scalar, ``min(dL, dR)`` (m).
    m_phi_minus : (2,) margin to the safe lower bound per arm.
    m_phi_plus  : (2,) margin to the safe upper bound per arm.
    m_q         : scalar, worst-case joint-limit margin (rad).
    lookup      : ``ManifoldLookup`` (reserved for future reward terms).
    w_R/w_v/w_a/w_j/w_f : weights.
    a_prev/a_prev2 : optional previous filtered actions, for accel/jerk costs.

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
    R_i = 2.0 * np.minimum(m_phi_minus, m_phi_plus) / width_safe
    R_reserve = float(np.minimum(R_i[0], R_i[1]))

    C_v = float(np.sum(phi_dot_safe ** 2))
    C_filter = float(np.sum((phi_dot_safe - phi_dot_nom) ** 2))

    C_a = 0.0
    if a_prev is not None:
        C_a = float(np.sum(((phi_dot_safe - np.asarray(a_prev, dtype=np.float64)) / dt) ** 2))
    C_j = 0.0
    if a_prev2 is not None and a_prev is not None:
        da = (phi_dot_safe - np.asarray(a_prev, dtype=np.float64)) / dt
        da_prev = (np.asarray(a_prev, dtype=np.float64) - np.asarray(a_prev2, dtype=np.float64)) / dt
        C_j = float(np.sum(((da - da_prev) / dt) ** 2))

    reward = w_R * R_reserve - w_v * C_v - w_a * C_a - w_j * C_j - w_f * C_filter

    info = {
        "R_reserve": R_reserve,
        "R_L": float(R_i[0]),
        "R_R": float(R_i[1]),
        "C_v": C_v,
        "C_a": C_a,
        "C_j": C_j,
        "C_filter": C_filter,
        "reward": reward,
    }
    return reward, info

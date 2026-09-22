"""2-D polygon-clipping safety filter over the redundancy phase (Task 1.3, BLOCKER #4/#5).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

The policy emits a nominal phase velocity ``phi_dot_nom`` (rad/s per arm).  The
filter projects it onto the feasible set of a single finite step:

    |Q_x(x_next)·xdot + Q_phi(x_next)·r| <= qdot_max      (14 joint-vel half-planes)
    phi + r·dt  in  [phi_safe_lo(x_next), phi_safe_hi(x_next)]   (per-arm box)

where ``xdot = (x_next - x)/dt`` and ``r`` is the 2-D phase velocity.  The
constraints are *linear* in ``r`` (a set of 2-D half-planes), so the feasible set
is a convex polygon in the phase-velocity plane; we clip it with
Sutherland-Hodgman and project ``phi_dot_nom`` onto it.

The projected velocity is then *re-verified* on the finite-step result (the
post-Newton hard shield): because ``Q`` is trilinear (bilinear in ``(theta, s)``
times linear in ``phi``), the linearized prediction can under-deliver, so we
check the actual ``q`` and clearance at ``phi + r·dt`` and backtrack toward zero
if the true result violates the limits.  This is v0.1's finite-step guard, not a
formal CBF.
"""

from __future__ import annotations

import numpy as np

__all__ = ["project_phi_dot"]

# frozen v0.1 constants
_DSafe = 0.005          # m -- hard clearance threshold (5 mm)
_QDOT_MAX = 1.5         # rad/s -- hard joint-speed limit
_MAX_BACKTRACK = 5      # hard-shield 2-D backtracking steps
_CLIP_EPS = 1e-7        # inside-test tolerance: the manifold derivatives are float32
                        # (0.6 -> 0.60000002), so a vertex lying on a plane re-tests a
                        # few e-9 outside it; without this the repeated clipping of the
                        # 7 identical per-joint half-planes is not idempotent and the
                        # polygon accumulates spurious vertices.
_QDOT_TOL = 1e-4        # hard-shield tolerance (rad/s): float32 q (~1e-7 abs error)
                        # amplified by 1/dt=100 -> ~1e-5 rad/s; genuine trilinear
                        # curvature violations are ~1e-2..1e-1, so this does not mask.
_D_TOL = 1e-9           # hard-shield clearance tolerance (m): float32 d error scale.


# ---------------------------------------------------------------------------
# half-plane clipping (Sutherland-Hodgman)
# ---------------------------------------------------------------------------
def _clip_polygon(poly, a, b):
    """Clip a CCW polygon ``poly`` (M,2) against the half-plane ``a·r <= b``."""
    if len(poly) == 0:
        return poly
    out = []
    n = len(poly)
    for i in range(n):
        p_cur = poly[i]
        p_prev = poly[i - 1]
        cur_in = float(a @ p_cur) <= b + _CLIP_EPS
        prev_in = float(a @ p_prev) <= b + _CLIP_EPS
        if cur_in:
            if not prev_in:
                out.append(_intersect(p_prev, p_cur, a, b))
            out.append(p_cur)
        elif prev_in:
            out.append(_intersect(p_prev, p_cur, a, b))
    return out


def _intersect(p0, p1, a, b):
    """Intersection of segment p0->p1 with the line ``a·r = b`` (denominator nonzero)."""
    d = p1 - p0
    denom = float(a @ d)
    # both endpoints are on opposite sides, so |denom| > 0 up to round-off
    t = (b - float(a @ p0)) / denom
    return p0 + t * d


def _project_onto_polygon(nom, poly):
    """Closest point of ``nom`` to the convex polygon ``poly`` (M,2)."""
    poly = np.asarray(poly, dtype=np.float64)
    if len(poly) == 1:
        return poly[0].copy()
    n = len(poly)
    # inside test (convex CCW polygon): nom lies left of every directed edge
    inside = True
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if float(np.cross(b - a, nom - a)) < -_CLIP_EPS:
            inside = False
            break
    if inside:
        return nom.copy()
    # otherwise the closest point is on some edge
    best = None
    best_d = np.inf
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        seg = b - a
        seg2 = float(seg @ seg)
        if seg2 < 1e-18:
            p = a
        else:
            t = float((nom - a) @ seg) / seg2
            t = min(max(t, 0.0), 1.0)
            p = a + t * seg
        d = float(np.sum((nom - p) ** 2))
        if d < best_d:
            best_d = d
            best = p
    return best


def _assemble_q(res):
    """Concatenate per-arm (1,7) configs into a single (14,) vector."""
    return np.concatenate([res["qL"][0], res["qR"][0]])


# ---------------------------------------------------------------------------
# main entry point
# ---------------------------------------------------------------------------
def project_phi_dot(
    phi_dot_nom: np.ndarray,
    x: np.ndarray,
    x_next: np.ndarray,
    phi: np.ndarray,
    lookup,
    qdot_max: float = _QDOT_MAX,
    dt: float = 0.01,
    d_safe: float = _DSafe,
) -> tuple[np.ndarray, dict]:
    """Project a nominal phase velocity onto the one-step safe set.

    Parameters
    ----------
    phi_dot_nom : (2,) -- ``(phi_dot_L, phi_dot_R)`` rad/s.
    x, x_next   : (2,) -- current / next task state ``(theta, s)``.
    phi         : (2,) -- current unwrapped phase ``(phi_L, phi_R)``.
    lookup      : ``ManifoldLookup``.
    qdot_max    : hard joint-speed limit (rad/s).
    dt          : control period (s).
    d_safe      : hard clearance threshold (m).

    Returns
    -------
    (phi_dot_safe, info).  ``info`` carries ``{'feasible', 'intervened',
    'clipped', 'backtracked'}``.  On infeasibility ``phi_dot_safe`` is zero and
    the env terminates the episode.
    """
    phi_dot_nom = np.asarray(phi_dot_nom, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    x_next = np.asarray(x_next, dtype=np.float64)
    phi = np.asarray(phi, dtype=np.float64)

    # --- safe box at x_next (per-arm) --------------------------------------
    box = lookup.safe_interval(x_next[None, :])
    phi_safe_lo = box["phi_safe_lo"][0]
    phi_safe_hi = box["phi_safe_hi"][0]

    # --- derivatives at (x_next, current phase) -----------------------------
    res = lookup.query(x_next[None, :], phi[None, :], check_safe=False)
    Q_x = res["Q_x"][0]      # (14, 2)
    Q_phi = res["Q_phi"][0]  # (14, 2), block-diagonal

    xdot = (x_next - x) / dt  # (2,)

    # --- assemble half-planes A·r <= b --------------------------------------
    # joint-velocity: |Q_x[j]·xdot + Q_phi[j]·r| <= qdot_max  -> 2 planes/joint
    A_list, b_list = [], []
    base = Q_x @ xdot          # (14,) task-driven joint velocity
    g = Q_phi                  # (14,2) per-joint phase gradient
    for j in range(14):
        gj = g[j]              # (2,)
        #  +gj·r <= qdot_max - base[j]
        A_list.append(gj)
        b_list.append(qdot_max - base[j])
        #  -gj·r <= qdot_max + base[j]
        A_list.append(-gj)
        b_list.append(qdot_max + base[j])

    # box: phi_safe_lo - phi <= r·dt <= phi_safe_hi - phi  -> 2 planes/arm
    r_lo = (phi_safe_lo - phi) / dt
    r_hi = (phi_safe_hi - phi) / dt
    for arm in range(2):
        e = np.zeros(2)
        e[arm] = -1.0
        A_list.append(e)
        b_list.append(-r_lo[arm])        # -r_arm <= -r_lo  <=>  r_arm >= r_lo
        e = np.zeros(2)
        e[arm] = 1.0
        A_list.append(e)
        b_list.append(r_hi[arm])         # +r_arm <= r_hi

    A = np.asarray(A_list)   # (E, 2)
    b = np.asarray(b_list)   # (E,)

    # --- clip the box polygon against the joint-velocity half-planes --------
    # start from the box (CCW), then intersect all planes
    poly = [
        np.array([r_lo[0], r_lo[1]]),
        np.array([r_hi[0], r_lo[1]]),
        np.array([r_hi[0], r_hi[1]]),
        np.array([r_lo[0], r_hi[1]]),
    ]
    for k in range(A.shape[0]):
        poly = _clip_polygon(poly, A[k], b[k])
        if len(poly) == 0:
            break
    if len(poly) == 0:
        return (np.zeros(2), {"feasible": False, "intervened": True,
                              "clipped": True, "backtracked": 0})

    r = _project_onto_polygon(phi_dot_nom, poly)

    # --- post-Newton hard shield: verify the finite-step result --------------
    # q_ref = Q(x_next, phi + r·dt) (the manifold point IS the task projection;
    # the trilinear interpolation error is caught here, not by a separate Newton
    # solve).  Verify max|(q_ref - q_t)/dt| <= qdot_max and d >= d_safe.
    # clamp to joint limits (both the reference and the previous actual config),
    # matching the plan's "clamp to joint limits, then check" hard-shield rule.
    lo = np.concatenate([lookup.joint_lower[0], lookup.joint_lower[1]])
    hi = np.concatenate([lookup.joint_upper[0], lookup.joint_upper[1]])
    q_t = np.clip(_assemble_q(lookup.query(x[None, :], phi[None, :])), lo, hi)
    intervened = bool(np.linalg.norm(r - phi_dot_nom) > 1e-12)
    backtracked = 0
    for _ in range(_MAX_BACKTRACK + 1):
        phi_next = phi + r * dt
        nxt = lookup.query(x_next[None, :], phi_next[None, :], check_safe=False)
        q_ref = np.clip(_assemble_q(nxt), lo, hi)
        d_actual = float(np.minimum(nxt["dL"][0], nxt["dR"][0]))
        q_dot_actual = (q_ref - q_t) / dt
        if (float(np.max(np.abs(q_dot_actual))) <= qdot_max + _QDOT_TOL
                and d_actual >= d_safe - _D_TOL):
            break
        if _ == _MAX_BACKTRACK:
            return (np.zeros(2), {"feasible": False, "intervened": True,
                                  "clipped": True, "backtracked": _MAX_BACKTRACK})
        r = 0.5 * r
        backtracked += 1
        intervened = True

    info = {"feasible": True, "intervened": intervened,
            "clipped": bool(intervened), "backtracked": backtracked}
    return r, info

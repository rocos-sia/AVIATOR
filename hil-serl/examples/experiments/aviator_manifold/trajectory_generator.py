"""Aviator v0.1 -- C2 quintic trajectory generator (Task 1.1, BLOCKER #7 / #8).

Doc: ``docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md``

This module is the *single* trajectory source of the whole pipeline:

* it generates the unified train/val/test split
  (``data/aviator/trajectory_source/generate.py`` calls
  :func:`generate_task_trajectory`), and
* it provides the generator used by the Python env's ``reset()``.

Construction
------------
Waypoints are drawn inside the FROZEN task ranges and a piecewise quintic
Hermite interpolant is built through them.  Every knot carries a *continuous*
velocity and acceleration (central differences of the neighbouring waypoints,
one-sided at the two ends) -- motion never dwells at a waypoint.  Because
consecutive segments share the knot value, velocity and acceleration, the
resulting curve is C2 everywhere.

Each segment is then time-scaled: with the segment written as ``P(u)`` on the
unit parameter ``u in [0, 1]`` and duration ``h``, the physical derivatives are
``x^(m) = P^(m)(u) / h^m``.  With ``M_m = max_u |P^(m)(u)|`` (computed
*analytically*, from the roots of the next derivative, not by sampling) we need

    h >= M_1 / v_max,   h >= sqrt(M_2 / a_max),   h >= (M_3 / j_max) ** (1/3)

per dimension and per segment.  The *same* duration ``h`` is used for every
segment (the largest per-segment requirement): knot velocities/accelerations are
shared between the two segments meeting at a knot, so only a common time scale
keeps ``x'``/``x''`` continuous in *physical* time -- with unequal ``h`` the same
knot derivative is divided by two different durations and C1/C2 would break.
The common ``h`` is finally stretched once more so the whole trajectory spans
exactly the requested horizon ``T``; a uniform stretch divides every speed by
the same factor and therefore preserves all three bounds.

State convention (FROZEN): ``x[:, 0] = theta in [-0.87266, +0.87266] rad``,
``x[:, 1] = s in [-0.16, 0.0] m``.
"""

from __future__ import annotations

import numpy as np

# --- FROZEN v0.1 constants -------------------------------------------------
THETA_RANGE = (-0.87266, 0.87266)
S_RANGE = (-0.16, 0.0)
V_MAX = (1.486, 0.167)
A_MAX = (5.0, 2.0)
J_MAX = (20.0, 10.0)

#: Waypoint sampling interval shrink factor used when the interpolant leaves
#: the frozen state range (quintic Hermite can overshoot its waypoints).
_RANGE_SHRINK = 0.85
_MAX_RANGE_TRIES = 10
_MIN_SEGMENT_TIME = 1e-6


# ---------------------------------------------------------------------------
# polynomial helpers (coefficients are always highest-degree first)
# ---------------------------------------------------------------------------
def _quintic_hermite(p0, v0, a0, p1, v1, a1):
    """Unit-parameter quintic matching value/1st/2nd derivative at u=0 and u=1.

    All arguments are ``(D,)`` arrays; the returned coefficient array has shape
    ``(6, D)``, highest degree first (``[c5, c4, c3, c2, c1, c0]``).
    """
    c0 = np.asarray(p0, dtype=float)
    c1 = np.asarray(v0, dtype=float)
    c2 = np.asarray(a0, dtype=float) / 2.0

    # rows: p(1), p'(1), p''(1) constraints on [c3, c4, c5]
    mat = np.array([[1.0, 1.0, 1.0], [3.0, 4.0, 5.0], [6.0, 12.0, 20.0]])
    rhs = np.stack(
        [p1 - (c0 + c1 + c2), v1 - (c1 + 2.0 * c2), a1 - 2.0 * c2], axis=0
    )
    c345 = np.linalg.solve(mat, rhs)  # (3, D)
    return np.stack([c345[2], c345[1], c345[0], c2, c1, c0], axis=0)


def _poly_max_abs(coeffs):
    """``max_{u in [0,1]} |p(u)|`` for a polynomial given highest-degree first."""
    c = np.asarray(coeffs, dtype=float).ravel()
    nz = np.nonzero(np.abs(c) > 0.0)[0]
    if nz.size == 0:
        return 0.0
    c = c[nz[0]:]
    values = [abs(float(np.polyval(c, 0.0))), abs(float(np.polyval(c, 1.0)))]
    if c.size > 1:
        for root in np.roots(c):
            if abs(root.imag) <= 1e-9 and -1e-9 <= root.real <= 1.0 + 1e-9:
                u = min(max(float(root.real), 0.0), 1.0)
                values.append(abs(float(np.polyval(c, u))))
    return max(values)


def _segment_min_durations(seg_coeffs, v_max, a_max, j_max):
    """Minimum feasible duration of each segment (unit parameter -> seconds)."""
    v_max = np.asarray(v_max, dtype=float)
    a_max = np.asarray(a_max, dtype=float)
    j_max = np.asarray(j_max, dtype=float)

    durations = np.empty(seg_coeffs.shape[0], dtype=float)
    for seg in range(seg_coeffs.shape[0]):
        h = _MIN_SEGMENT_TIME
        for dim in range(seg_coeffs.shape[2]):
            c = seg_coeffs[seg][:, dim]
            m1 = _poly_max_abs(np.polyder(c, 1))
            m2 = _poly_max_abs(np.polyder(c, 2))
            m3 = _poly_max_abs(np.polyder(c, 3))
            h = max(
                h,
                m1 / v_max[dim],
                np.sqrt(m2 / a_max[dim]),
                (m3 / j_max[dim]) ** (1.0 / 3.0),
            )
        durations[seg] = h
    return durations


def _horner(coeffs, u):
    """Evaluate ``(N, K, D)`` coefficient stacks (highest first) at ``u`` ``(N,)``.

    Horner is used rather than ``np.polyval`` so that a *different* polynomial
    can be evaluated at every sample.
    """
    out = coeffs[:, 0, :]
    for k in range(1, coeffs.shape[1]):
        out = out * u[:, None] + coeffs[:, k, :]
    return out


def _knot_derivatives(waypoints, tau):
    """Continuous knot velocities/accelerations (C2 precondition)."""
    n, dim = waypoints.shape
    vel = np.zeros_like(waypoints)
    acc = np.zeros_like(waypoints)

    if n >= 3:
        vel[1:-1] = (waypoints[2:] - waypoints[:-2]) / (tau[2:] - tau[:-2])[:, None]
        slope_lo = (waypoints[1:-1] - waypoints[:-2]) / (tau[1:-1] - tau[:-2])[:, None]
        slope_hi = (waypoints[2:] - waypoints[1:-1]) / (tau[2:] - tau[1:-1])[:, None]
        acc[1:-1] = 2.0 * (slope_hi - slope_lo) / (tau[2:] - tau[:-2])[:, None]

    # one-sided first difference at the two ends; zero end acceleration
    vel[0] = (waypoints[1] - waypoints[0]) / (tau[1] - tau[0])
    vel[-1] = (waypoints[-1] - waypoints[-2]) / (tau[-1] - tau[-2])
    return vel, acc


# ---------------------------------------------------------------------------
# generator
# ---------------------------------------------------------------------------
def _build_trajectory(
    rng,
    T: float,
    dt: float = 0.01,
    theta_range=THETA_RANGE,
    s_range=S_RANGE,
    n_waypoints: int = 5,
    v_max=V_MAX,
    a_max=A_MAX,
    j_max=J_MAX,
):
    """Build one trajectory; returns ``(trajectory_dict, meta)``.

    ``meta`` (used by the analytic C1/C2 tests) holds the waypoints, the knot
    derivatives and the per-segment unit-parameter polynomial coefficients.
    """
    if n_waypoints < 2:
        raise ValueError(f"n_waypoints must be >= 2, got {n_waypoints}")

    lo = np.array([theta_range[0], s_range[0]], dtype=float)
    hi = np.array([theta_range[1], s_range[1]], dtype=float)
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    tau = np.arange(n_waypoints, dtype=float)

    n_samples = int(round(T / dt)) + 1
    t = np.arange(n_samples, dtype=float) * dt

    shrink = 1.0
    for _ in range(_MAX_RANGE_TRIES):
        w_lo = center - half * shrink
        w_hi = center + half * shrink
        waypoints = w_lo + (w_hi - w_lo) * rng.random((n_waypoints, 2))

        knot_v, knot_a = _knot_derivatives(waypoints, tau)
        seg_coeffs = np.stack(
            [
                _quintic_hermite(
                    waypoints[i], knot_v[i], knot_a[i],
                    waypoints[i + 1], knot_v[i + 1], knot_a[i + 1],
                )
                for i in range(n_waypoints - 1)
            ]
        )  # (n_seg, 6, 2)
        seg_h_min = _segment_min_durations(seg_coeffs, v_max, a_max, j_max)
        n_seg = seg_coeffs.shape[0]

        # a single duration for all segments (see module docstring)
        h_seg = float(seg_h_min.max())
        t_min = h_seg * n_seg
        if T < t_min:
            raise ValueError(
                f"requested horizon T={T} s is below the minimum feasible "
                f"trajectory duration {t_min} s (v_max/a_max/j_max bounds)"
            )

        # stretch uniformly so the whole trajectory spans T exactly
        h_seg *= T / t_min
        seg_h = np.full(n_seg, h_seg, dtype=float)
        knot_t = np.arange(n_seg + 1, dtype=float) * h_seg

        seg_idx = np.clip(
            np.searchsorted(knot_t, t, side="right") - 1, 0, n_seg - 1
        )
        local = (t - knot_t[seg_idx]) / seg_h[seg_idx]

        coeff_s = seg_coeffs[seg_idx]                                # (N, 6, 2)
        # derivative coefficient stacks (highest first, left-padded so that the
        # same Horner evaluation can be reused; degree drops by one per order)
        d1 = np.zeros_like(seg_coeffs)
        d1[:, 1:6, :] = seg_coeffs[:, 0:5, :] * np.array([5, 4, 3, 2, 1])[:, None]
        d2 = np.zeros_like(seg_coeffs)
        d2[:, 2:6, :] = seg_coeffs[:, 0:4, :] * np.array([20, 12, 6, 2])[:, None]
        d3 = np.zeros_like(seg_coeffs)
        d3[:, 3:6, :] = seg_coeffs[:, 0:3, :] * np.array([60, 24, 6])[:, None]

        h = seg_h[seg_idx][:, None]
        pos = _horner(coeff_s, local)
        vel = _horner(d1[seg_idx], local) / h
        acc = _horner(d2[seg_idx], local) / h**2
        jrk = _horner(d3[seg_idx], local) / h**3

        if (pos >= lo - 1e-12).all() and (pos <= hi + 1e-12).all():
            break
        shrink *= _RANGE_SHRINK
    else:  # pragma: no cover - practically unreachable
        raise RuntimeError(
            "could not keep the quintic interpolant inside the frozen state "
            f"ranges after {_MAX_RANGE_TRIES} attempts"
        )

    traj = {"t": t, "x": pos, "xdot": vel, "xddot": acc, "xdddot": jrk}
    meta = {
        "waypoints": waypoints,
        "knots_t": knot_t,
        "knot_v": knot_v,
        "knot_a": knot_a,
        "seg_coeffs": seg_coeffs,
        "seg_h": seg_h,
        "n_samples": n_samples,
        "T": float(T),
        "shrink": shrink,
    }
    return traj, meta


def generate_task_trajectory(
    rng,
    T: float,
    dt: float = 0.01,
    theta_range=THETA_RANGE,
    s_range=S_RANGE,
    n_waypoints: int = 5,
    v_max=V_MAX,
    a_max=A_MAX,
    j_max=J_MAX,
) -> dict:
    """C2 quintic B-spline trajectory.

    Returns ``{"t": (N,), "x": (N,2), "xdot": (N,2), "xddot": (N,2),
    "xdddot": (N,2)}``.  Waypoints carry continuous velocity/acceleration (not
    zero at knots); velocity/acceleration/jerk are bounded by
    ``v_max``/``a_max``/``j_max``.  Deterministic given ``rng``.
    """
    return _build_trajectory(
        rng,
        T,
        dt=dt,
        theta_range=theta_range,
        s_range=s_range,
        n_waypoints=n_waypoints,
        v_max=v_max,
        a_max=a_max,
        j_max=j_max,
    )[0]


def load_trajectory(path: str) -> dict:
    """Load a ``.npz`` trajectory file from ``trajectory_source/trajs/``."""
    with np.load(str(path)) as handle:
        return {key: np.asarray(handle[key], dtype=float) for key in handle.files}

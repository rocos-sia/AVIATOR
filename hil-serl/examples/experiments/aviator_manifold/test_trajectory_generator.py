"""Tests for the Aviator v0.1 C2 quintic trajectory generator (Task 1.1).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md
  - Task 1.1 (BLOCKER #7, #8)
  - Global Constraints (FROZEN for v0.1)
  - Task 0 (single trajectory source)

BLOCKER #8 tolerance: C1/C2 continuity is verified *analytically* (exact
polynomial match at knots, not a numerical gradient).  For finite-difference
checks the documented tolerance is ``atol = 1e-6 + 1e-3 * dt**2``.
"""

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from examples.experiments.aviator_manifold import trajectory_generator as tg

# --- FROZEN global constraints (v0.1) -------------------------------------
THETA_RANGE = (-0.87266, 0.87266)
S_RANGE = (-0.16, 0.0)
V_MAX = (1.486, 0.167)
A_MAX = (5.0, 2.0)
J_MAX = (20.0, 10.0)
DT = 0.01

HIL_SERL_ROOT = Path(__file__).resolve().parents[3]
GENERATE_PY = HIL_SERL_ROOT / "data" / "aviator" / "trajectory_source" / "generate.py"


def fd_atol(dt):
    """BLOCKER #8 finite-difference tolerance."""
    return 1e-6 + 1e-3 * dt**2


def _analytic(seg_coeffs, seg_h, seg, u, order):
    """Analytic d^order/dt^order of segment ``seg`` at local parameter ``u``.

    ``seg_coeffs`` has shape (n_seg, 6, 2) with coefficients highest-degree
    first, expressed in the *unit* local parameter u in [0, 1].  A physical
    time derivative of order ``order`` divides by ``h**order``.
    """
    out = np.zeros(seg_coeffs.shape[2])
    for d in range(seg_coeffs.shape[2]):
        c = seg_coeffs[seg][:, d]
        out[d] = np.polyval(np.polyder(c, order), float(u)) / seg_h[seg] ** order
    return out


def _load_generate_module():
    spec = importlib.util.spec_from_file_location("aviator_trajectory_source_generate", GENERATE_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# generator: shape / metadata contract
# --------------------------------------------------------------------------
def test_returns_documented_keys_and_shapes():
    rng = np.random.default_rng(0)
    traj = tg.generate_task_trajectory(rng, T=10.0, dt=DT)

    assert set(traj.keys()) == {"t", "x", "xdot", "xddot", "xdddot"}

    n = traj["t"].shape[0]
    assert n == int(round(10.0 / DT)) + 1
    assert traj["t"].shape == (n,)
    for key in ("x", "xdot", "xddot", "xdddot"):
        assert traj[key].shape == (n, 2)
        assert traj[key].dtype == np.float64
        assert np.all(np.isfinite(traj[key]))


def test_time_grid_is_uniform_with_dt():
    rng = np.random.default_rng(1)
    for T in (8.0, 9.53, 12.0):
        traj = tg.generate_task_trajectory(rng, T=T, dt=DT)
        t = traj["t"]
        assert t[0] == 0.0
        assert np.allclose(np.diff(t), DT, atol=1e-12)
        # last sample lands on the requested horizon within one control step
        assert abs(t[-1] - T) <= DT / 2 + 1e-9


def test_state_within_frozen_ranges():
    rng = np.random.default_rng(2)
    for _ in range(5):
        traj = tg.generate_task_trajectory(rng, T=10.0, dt=DT)
        theta = traj["x"][:, 0]
        s = traj["x"][:, 1]
        assert theta.min() >= THETA_RANGE[0] - 1e-12
        assert theta.max() <= THETA_RANGE[1] + 1e-12
        assert s.min() >= S_RANGE[0] - 1e-12
        assert s.max() <= S_RANGE[1] + 1e-12


@pytest.mark.parametrize("n_waypoints", [3, 5, 7])
@pytest.mark.parametrize("seed", [0, 3])
def test_velocity_acceleration_jerk_bounds(n_waypoints, seed):
    rng = np.random.default_rng(seed)
    traj = tg.generate_task_trajectory(rng, T=10.0, dt=DT, n_waypoints=n_waypoints)

    max_v = np.max(np.abs(traj["xdot"]), axis=0)
    max_a = np.max(np.abs(traj["xddot"]), axis=0)
    max_j = np.max(np.abs(traj["xdddot"]), axis=0)

    assert np.all(max_v <= np.asarray(V_MAX) * (1.0 + 1e-9))
    assert np.all(max_a <= np.asarray(A_MAX) * (1.0 + 1e-9))
    assert np.all(max_j <= np.asarray(J_MAX) * (1.0 + 1e-9))


def test_waypoints_carry_nonzero_velocity_and_acceleration():
    """Motion is non-stop: knots carry continuous, non-zero derivatives."""
    rng = np.random.default_rng(4)
    _, meta = tg._build_trajectory(rng, T=10.0, dt=DT)
    knot_v = meta["knot_v"]
    assert np.all(np.abs(knot_v) > 1e-9)
    assert np.all(np.isfinite(meta["knot_a"]))
    # interior knot accelerations are non-trivial (not all zero)
    assert np.max(np.abs(meta["knot_a"][1:-1])) > 1e-9


# --------------------------------------------------------------------------
# C0 / C1 / C2 at segment boundaries -- ANALYTICAL polynomial match (BLOCKER #8)
# --------------------------------------------------------------------------
def test_knots_interpolate_waypoints_analytically():
    rng = np.random.default_rng(5)
    _, meta = tg._build_trajectory(rng, T=10.0, dt=DT)
    seg_coeffs, seg_h, knots_t = meta["seg_coeffs"], meta["seg_h"], meta["knots_t"]
    wp = meta["waypoints"]

    atol = fd_atol(DT)
    assert knots_t.shape[0] == wp.shape[0]
    for k in range(wp.shape[0]):
        seg = min(k, seg_coeffs.shape[0] - 1)
        u = 1.0 if k == wp.shape[0] - 1 else 0.0
        np.testing.assert_allclose(_analytic(seg_coeffs, seg_h, seg, u, 0), wp[k], atol=atol)


@pytest.mark.parametrize("order", [0, 1, 2])
def test_c0_c1_c2_continuity_at_knots_analytical(order):
    rng = np.random.default_rng(6)
    _, meta = tg._build_trajectory(rng, T=10.0, dt=DT)
    seg_coeffs, seg_h, knots_t = meta["seg_coeffs"], meta["seg_h"], meta["knots_t"]

    atol = fd_atol(DT)
    for k in range(seg_coeffs.shape[0] - 1):
        left = _analytic(seg_coeffs, seg_h, k, 1.0, order)      # u=1 of segment k
        right = _analytic(seg_coeffs, seg_h, k + 1, 0.0, order)  # u=0 of segment k+1
        np.testing.assert_allclose(left, right, atol=atol)
        assert abs(knots_t[k + 1] - (knots_t[k] + seg_h[k])) < 1e-12


def test_returned_derivatives_match_segment_polynomials():
    """Returned xdot/xddot/xdddot are the true analytic derivatives."""
    rng = np.random.default_rng(7)
    traj, meta = tg._build_trajectory(rng, T=10.0, dt=DT)
    seg_coeffs, seg_h, knots_t = meta["seg_coeffs"], meta["seg_h"], meta["knots_t"]
    t = traj["t"]
    seg_idx = np.clip(np.searchsorted(knots_t, t, side="right") - 1, 0, seg_coeffs.shape[0] - 1)

    atol = fd_atol(DT)
    for j in range(t.shape[0]):
        u = (t[j] - knots_t[seg_idx[j]]) / seg_h[seg_idx[j]]
        for order, key in ((1, "xdot"), (2, "xddot"), (3, "xdddot")):
            np.testing.assert_allclose(
                _analytic(seg_coeffs, seg_h, seg_idx[j], u, order), traj[key][j], atol=atol
            )


def test_finite_difference_matches_returned_velocity():
    """4th-order central difference of x reproduces the returned velocity.

    The curve is C2 but *not* C3 (jerk jumps at knots), so a stencil is only
    meaningful when it stays inside one segment -- exactly why C1/C2 at knots
    is checked analytically above (BLOCKER #8).
    """
    rng = np.random.default_rng(8)
    for T in (8.0, 10.0, 12.0):
        traj, meta = tg._build_trajectory(rng, T=T, dt=DT)
        x, xdot, t = traj["x"], traj["xdot"], traj["t"]
        n = t.shape[0]
        knots_t = meta["knots_t"]

        fd = (x[:-4] - 8.0 * x[1:-3] + 8.0 * x[3:-1] - x[4:]) / (12.0 * DT)
        fd_centers = np.arange(2, n - 2)

        seg_idx = np.clip(
            np.searchsorted(knots_t, t, side="right") - 1,
            0,
            meta["seg_coeffs"].shape[0] - 1,
        )
        same_segment = seg_idx[fd_centers - 2] == seg_idx[fd_centers + 2]
        assert same_segment.sum() > n // 2  # most samples still checked

        err = np.abs(fd[same_segment] - xdot[fd_centers[same_segment]])
        np.testing.assert_allclose(err, np.zeros_like(err), atol=fd_atol(DT))


# --------------------------------------------------------------------------
# determinism
# --------------------------------------------------------------------------
def test_deterministic_given_rng():
    a = tg.generate_task_trajectory(np.random.default_rng(11), T=10.0, dt=DT)
    b = tg.generate_task_trajectory(np.random.default_rng(11), T=10.0, dt=DT)
    for key in a:
        np.testing.assert_array_equal(a[key], b[key])

    c = tg.generate_task_trajectory(np.random.default_rng(12), T=10.0, dt=DT)
    assert not np.allclose(a["x"], c["x"])


def test_infeasible_horizon_raises():
    rng = np.random.default_rng(13)
    with pytest.raises(ValueError):
        tg.generate_task_trajectory(rng, T=1e-6, dt=DT)


# --------------------------------------------------------------------------
# loader
# --------------------------------------------------------------------------
def test_load_trajectory_roundtrip(tmp_path):
    rng = np.random.default_rng(14)
    traj = tg.generate_task_trajectory(rng, T=8.0, dt=DT)
    path = tmp_path / "traj.npz"
    np.savez(path, **{k: traj[k] for k in ("t", "x", "xdot", "xddot")})

    loaded = tg.load_trajectory(str(path))
    assert set(loaded.keys()) == {"t", "x", "xdot", "xddot"}
    for key in loaded:
        np.testing.assert_array_equal(loaded[key], traj[key])


# --------------------------------------------------------------------------
# Task 0 -- unified trajectory source
# --------------------------------------------------------------------------
def test_generate_split_counts_files_manifest(tmp_path):
    gen = _load_generate_module()
    rng = np.random.default_rng(0)
    manifest = gen.generate_split(
        rng, seed=0, n_dp=3, n_rl=4, n_val=2, n_test=2,
        T_min=8.0, T_max=9.0, dt=DT, out_dir=str(tmp_path),
    )

    n_total = 3 + 4 + 2 + 2
    rows = manifest["trajectories"]
    assert len(rows) == n_total
    assert manifest["counts"] == {"dp_train": 3, "rl_train": 4, "val": 2, "test": 2}

    for row in rows:
        assert set(row) >= {"split", "index", "seed", "T", "dt", "path", "sha256"}
        assert 8.0 - 1e-9 <= row["T"] <= 9.0 + 1e-9
        assert row["dt"] == DT
        path = tmp_path / row["path"]
        assert path.is_file()
        # sha256 in the manifest matches the file on disk
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row["sha256"]

        traj = tg.load_trajectory(str(path))
        assert set(traj.keys()) == {"t", "x", "xdot", "xddot"}
        assert np.all(np.abs(traj["xdot"]) <= np.asarray(V_MAX)[None, :] * (1 + 1e-9))
        assert np.all(np.abs(traj["xddot"]) <= np.asarray(A_MAX)[None, :] * (1 + 1e-9))
        np.testing.assert_allclose(row["max_abs_xdot"], np.max(np.abs(traj["xdot"]), axis=0))
        np.testing.assert_allclose(row["max_abs_xddot"], np.max(np.abs(traj["xddot"]), axis=0))

    for split in ("dp_train", "rl_train", "val", "test"):
        assert (tmp_path / "trajs" / split).is_dir()

    written = json.loads((tmp_path / "split_manifest.json").read_text())
    assert written == manifest


def test_generate_split_is_deterministic(tmp_path):
    gen = _load_generate_module()
    kw = dict(n_dp=2, n_rl=2, n_val=1, n_test=1, T_min=8.0, T_max=9.0, dt=DT)
    m1 = gen.generate_split(np.random.default_rng(0), seed=5, out_dir=str(tmp_path / "a"), **kw)
    m2 = gen.generate_split(np.random.default_rng(0), seed=5, out_dir=str(tmp_path / "b"), **kw)
    assert m1["trajectories"] == m2["trajectories"]

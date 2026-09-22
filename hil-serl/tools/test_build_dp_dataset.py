"""Tests for DP CSV -> HIL-SERL demo pkl (Task 1.4, BLOCKER #11).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.4)

Uses a small synthetic 3-row CSV (no 100 MB manifold).  Verifies:
  - CSV parsing (47 columns, L_phi metadata comment)
  - phi_dot_scale = max|phi_dot| * 1.1
  - 3 rows -> 3 transition dicts with correct keys / shapes / masks / dones
"""

import json

import numpy as np
import pytest

from tools.build_dp_dataset import (
    COLUMNS,
    build_transitions,
    compute_phi_dot_scale,
    read_dp_csv,
)


def _write_csv(path, n=3, phi_dot_L=None, phi_dot_R=None, L_phi=(0.3, 0.3)):
    """Write a synthetic record_dp CSV with ``n`` data rows."""
    if phi_dot_L is None:
        phi_dot_L = np.linspace(0.5, 0.0, n)      # last row -> 0
    if phi_dot_R is None:
        phi_dot_R = np.linspace(-0.3, 0.0, n)
    phi_dot_L = np.asarray(phi_dot_L, dtype=float)
    phi_dot_R = np.asarray(phi_dot_R, dtype=float)

    lines = [f"# L_phi_L={L_phi[0]},L_phi_R={L_phi[1]}\n", ",".join(COLUMNS) + "\n"]
    for t in range(n):
        row = {
            "t": t * 0.01, "theta": 0.1 * t, "s": -0.1 + 0.01 * t,
            "theta_dot": 0.05, "s_dot": 0.01,
            "sin_phi_L": np.sin(0.05 * t), "cos_phi_L": np.cos(0.05 * t),
            "sin_phi_R": np.sin(-0.03 * t), "cos_phi_R": np.cos(-0.03 * t),
            "phi_dot_L": phi_dot_L[t], "phi_dot_R": phi_dot_R[t],
            "dL": 0.010, "dR": 0.012, "d_min": 0.010,
            "m_phi_minus_L": 0.10, "m_phi_minus_R": 0.12,
            "m_phi_plus_L": 0.11, "m_phi_plus_R": 0.13,
            "m_q": 0.5,
        }
        # joint configs/velocities: 14 + 14 = 28 columns, any finite values
        for i in range(1, 8):
            row[f"qL{i}"] = 0.1 * i + 0.01 * t
            row[f"qR{i}"] = -0.1 * i + 0.01 * t
            row[f"qdotL{i}"] = 0.01 * i
            row[f"qdotR{i}"] = -0.01 * i
        lines.append(",".join(str(row[c]) for c in COLUMNS) + "\n")
    path.write_text("".join(lines))
    return str(path)


def test_read_dp_csv(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=4)
    d = read_dp_csv(p)
    assert d["theta"].shape == (4,)
    assert d["L_phi_L"] == 0.3 and d["L_phi_R"] == 0.3
    assert len(d["t"]) == 4
    # phi_dot columns exist and are parsed
    assert d["phi_dot_L"][0] == pytest.approx(0.5)


def test_compute_phi_dot_scale(tmp_path):
    p1 = _write_csv(tmp_path / "traj_0000.csv", n=3,
                    phi_dot_L=[0.0, 0.0, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    p2 = _write_csv(tmp_path / "traj_0001.csv", n=3,
                    phi_dot_L=[0.0, 2.0, 0.0], phi_dot_R=[0.0, -1.5, 0.0])
    m = compute_phi_dot_scale([p1, p2], headroom=1.1)
    # max |phi_dot_L| = 2.0, |phi_dot_R| = 1.5 -> scale = 2.0 * 1.1
    assert m["max_abs_phi_dot"] == [2.0, 1.5]
    assert m["phi_dot_scale"] == pytest.approx(2.2)
    assert m["n_trajs"] == 2


def test_build_transitions_keys_shapes_masks_dones(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[0.5, 0.6, 0.0], phi_dot_R=[-0.3, -0.4, 0.0])
    d = read_dp_csv(p)
    scale = 1.0   # no normalization for this test
    tr = build_transitions(d, scale)

    assert len(tr) == 3

    expected_keys = {"observations", "next_observations", "actions",
                     "rewards", "masks", "dones"}
    for t in tr:
        assert set(t.keys()) == expected_keys
        assert t["observations"]["state"].shape == (1, 40)
        assert t["next_observations"]["state"].shape == (1, 40)
        assert t["actions"].shape == (2,)
        assert np.isscalar(t["rewards"]) or t["rewards"].shape == ()
        assert np.isscalar(t["masks"]) or t["masks"].shape == ()

    # all-but-last: masks=1.0, dones=False
    for t in tr[:2]:
        assert t["masks"] == 1.0
        assert t["dones"] is False
    # last: terminal
    assert tr[-1]["dones"] is True
    assert tr[-1]["masks"] == 0.0
    np.testing.assert_array_equal(tr[-1]["actions"], np.zeros(2))

    # non-terminal actions carry the forward-difference teacher action
    np.testing.assert_allclose(tr[0]["actions"], [0.5, -0.3])
    np.testing.assert_allclose(tr[1]["actions"], [0.6, -0.4])


def test_action_normalized_by_scale(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[1.0, 2.0, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    d = read_dp_csv(p)
    tr = build_transitions(d, phi_dot_scale=2.0)
    # action = phi_dot / scale
    np.testing.assert_allclose(tr[0]["actions"], [0.5, 0.0])
    np.testing.assert_allclose(tr[1]["actions"], [1.0, 0.0])


def test_obs40_layout_matches_env(tmp_path):
    # Obs layout: [0:2]x [2:4]xdot [4:8] sin/cos phi [8:12] margins
    # [12] d_min [13] m_q [14:16] a_prev [16:40] hist
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[0.5, 0.6, 0.0], phi_dot_R=[-0.3, -0.4, 0.0])
    d = read_dp_csv(p)
    tr = build_transitions(d, phi_dot_scale=1.0)
    s0 = tr[0]["observations"]["state"][0]
    assert s0[0] == pytest.approx(0.0)          # theta(t=0)
    assert s0[1] == pytest.approx(-0.1)         # s(t=0)
    assert s0[4] == pytest.approx(np.sin(0.0))  # sin_phi_L
    assert s0[5] == pytest.approx(np.cos(0.0))  # cos_phi_L
    assert s0[12] == pytest.approx(0.010)       # d_min
    assert s0[13] == pytest.approx(0.5)         # m_q
    np.testing.assert_allclose(s0[14:16], [0.0, 0.0])   # a_prev = 0 at t=0
    # hist_x at t=0: lags 2/5/10/20 all clamp to x(0) = (0, -0.1)
    np.testing.assert_allclose(s0[16:24], np.tile([0.0, -0.1], 4))


def test_phi_dot_scale_json_roundtrip(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[0.0, 1.5, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    m = compute_phi_dot_scale([p], headroom=1.1)
    assert m["phi_dot_scale"] == pytest.approx(1.65)
    assert m["flag_exceeds_nominal"] is False
    json.dumps(m)   # serializable

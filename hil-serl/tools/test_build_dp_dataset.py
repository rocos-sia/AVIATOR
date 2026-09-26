"""Tests for DP CSV -> HIL-SERL demo pkl (Task 1.4, BLOCKER #11).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.4)

Uses a small synthetic 3-row CSV (no 100 MB manifold).  Verifies:
  - CSV parsing (47 columns, L_phi metadata comment)
  - phi_dot_scale = clip(p95 |phi_dot|, floor=1.5, ceil=2.0)
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


def test_compute_phi_dot_scale_floors_at_1p5(tmp_path):
    # demo |phi_dot| is hold-dominated: p95 = 0.5.  The scale must NOT collapse
    # to 0.5 (that would saturate the actor at every 0.5 rad/s move); it is
    # floored at 1.5 rad/s so a full-range action stays below the ~2.97 rad/s
    # joint-speed ceiling.
    p = _write_csv(tmp_path / "traj_0000.csv", n=4,
                   phi_dot_L=[0.0, 0.5, 0.5, 0.0], phi_dot_R=[0.0, -0.5, -0.5, 0.0])
    m = compute_phi_dot_scale([p])
    assert m["p95_abs_phi_dot"] == pytest.approx(0.5)
    assert m["phi_dot_scale"] == pytest.approx(1.5)
    assert m["max_abs_phi_dot"] == [0.5, 0.5]
    assert m["flag_exceeds_nominal"] is False


def test_compute_phi_dot_scale_ignores_outlier(tmp_path):
    # a single 9.5 glitch frame must not inflate the scale (max*headroom gave
    # 10.45); p95 stays ~0, the scale is the 1.5 floor, and the manifest still
    # records the outlier so build_transitions can clip it.
    zeros = [0.0] * 20
    p = _write_csv(tmp_path / "traj_0000.csv", n=21,
                   phi_dot_L=zeros + [9.5], phi_dot_R=[0.0] * 21)
    m = compute_phi_dot_scale([p])
    assert m["max_abs_phi_dot"][0] == pytest.approx(9.5)
    assert m["p95_abs_phi_dot"] == pytest.approx(0.0)
    assert m["phi_dot_scale"] == pytest.approx(1.5)
    assert m["flag_exceeds_nominal"] is True


def test_build_transitions_keys_shapes_masks_dones(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[0.5, 0.6, 0.0], phi_dot_R=[-0.3, -0.4, 0.0])
    d = read_dp_csv(p)
    scale = 1.0   # no normalization for this test
    tr = build_transitions(d, scale)

    # N rows -> N-1 transitions (the last lands on the final state and is
    # terminal, carrying its real action -- no synthetic zero-action dummy).
    assert len(tr) == 2

    expected_keys = {"observations", "next_observations", "actions",
                     "rewards", "masks", "dones"}
    for t in tr:
        assert set(t.keys()) == expected_keys
        assert t["observations"]["state"].shape == (1, 40)
        assert t["next_observations"]["state"].shape == (1, 40)
        assert t["actions"].shape == (2,)
        assert np.isscalar(t["rewards"]) or t["rewards"].shape == ()
        assert np.isscalar(t["masks"]) or t["masks"].shape == ()

    # non-terminal first transition: mask=1.0, dones=False
    assert tr[0]["masks"] == 1.0
    assert tr[0]["dones"] is False
    # last: terminal (mask=0, done=True) but keeps its real teacher action
    assert tr[-1]["dones"] is True
    assert tr[-1]["masks"] == 0.0

    # actions carry the forward-difference teacher action (not a zero dummy)
    np.testing.assert_allclose(tr[0]["actions"], [0.5, -0.3])
    np.testing.assert_allclose(tr[-1]["actions"], [0.6, -0.4])


def test_action_normalized_by_scale(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[1.0, 2.0, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    d = read_dp_csv(p)
    tr = build_transitions(d, phi_dot_scale=2.0)
    # action = phi_dot / scale
    np.testing.assert_allclose(tr[0]["actions"], [0.5, 0.0])
    np.testing.assert_allclose(tr[1]["actions"], [1.0, 0.0])


def test_out_of_range_teacher_action_is_rejected(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[9.5, 0.0, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="not replayable"):
        build_transitions(read_dp_csv(p), phi_dot_scale=1.5)


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


def test_prior_clearance_and_reward_use_online_lookup_value(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3)
    data = read_dp_csv(p)

    class FixedLookup:
        def query(self, x, phi, check_safe=False):
            assert x.shape == phi.shape == (3, 2)
            return {"d_min": np.array([0.009, 0.008, 0.007])}

    original = build_transitions(data, 1.5)
    aligned = build_transitions(data, 1.5, lookup=FixedLookup())
    np.testing.assert_allclose(
        [aligned[0]["observations"]["state"][0, 12],
         aligned[0]["next_observations"]["state"][0, 12],
         aligned[1]["next_observations"]["state"][0, 12]],
        [0.009, 0.008, 0.007],
    )
    assert aligned[0]["rewards"] != original[0]["rewards"]


def test_phi_dot_scale_json_roundtrip(tmp_path):
    p = _write_csv(tmp_path / "traj_0000.csv", n=3,
                   phi_dot_L=[0.0, 1.5, 0.0], phi_dot_R=[0.0, 0.0, 0.0])
    m = compute_phi_dot_scale([p])
    assert m["phi_dot_scale"] == pytest.approx(1.5)
    assert m["flag_exceeds_nominal"] is False
    json.dumps(m)   # serializable

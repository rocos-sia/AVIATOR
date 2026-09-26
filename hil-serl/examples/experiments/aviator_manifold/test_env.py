"""Tests for the redundancy-policy Gym env (Task 1.3, Step 5).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

Verifies on the small synthetic manifold (no 100 MB data):
  - 40-D observation layout
  - action bounds Box(-1, 1, (2,))
  - info keys
  - termination paths: end, speed-violation
"""

import numpy as np
import pytest

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from examples.experiments.aviator_manifold.reward import reward_fn
from examples.experiments.aviator_manifold.test_manifold_lookup import make_manifold

# wide limits so the synthetic manifold's q values (max ~2.9) never clamp
_WIDE = ([[-5.0] * 7, [-5.0] * 7], [[5.0] * 7, [5.0] * 7])


def _traj(n=250):
    """Constant task state (theta, s) = (0, -0.1) with zero velocity."""
    return {"x": np.tile([0.0, -0.1], (n, 1)), "xdot": np.zeros((n, 2))}


def _env(tmp_path, *, n=250, scale=1.0, limits=_WIDE):
    d, _ = make_manifold(tmp_path, joint_limits=limits)
    return AviatorManifoldEnv(d, trajectories=[_traj(n)], phi_dot_scale=scale)


# --------------------------------------------------------------------------
# observation layout + action bounds
# --------------------------------------------------------------------------
def test_reset_obs_layout_and_action_bounds(tmp_path):
    env = _env(tmp_path)
    obs, _ = env.reset(seed=0)
    s = obs["state"]
    assert s.shape == (40,)
    assert s.dtype == np.float32

    # action space: Box(-1, 1, (2,))
    assert env.action_space.shape == (2,)
    np.testing.assert_array_equal(env.action_space.low, -1.0)
    np.testing.assert_array_equal(env.action_space.high, 1.0)

    # [0:2] x, [2:4] xdot, [4:8] sin/cos(phi), [8:12] margins
    np.testing.assert_allclose(s[0:2], [0.0, -0.1])
    np.testing.assert_allclose(s[2:4], [0.0, 0.0])
    np.testing.assert_allclose(s[4:6], [0.0, 1.0])   # sin(0), cos(0)
    np.testing.assert_allclose(s[6:8], [0.0, 1.0])
    # [12] d_min = min(dL, dR): dL = 0.010 + 0.001*(1+1+2) = 0.014 at (0,-0.1), phi=0
    np.testing.assert_allclose(s[12], 0.014, atol=1e-6)
    assert np.isfinite(s[13])
    # [14:16] a_prev = 0
    np.testing.assert_allclose(s[14:16], [0.0, 0.0])
    # history lags {2,5,10,20} all seeded from t=0 -> (0, -0.1), 0, 0
    np.testing.assert_allclose(s[16:24], np.tile([0.0, -0.1], 4))
    np.testing.assert_allclose(s[24:32], np.zeros(8))
    np.testing.assert_allclose(s[32:40], np.zeros(8))


# --------------------------------------------------------------------------
# 200-step rollout
# --------------------------------------------------------------------------
def test_rollout_200_steps(tmp_path):
    env = _env(tmp_path)
    env.reset(seed=1)
    for i in range(200):
        a = np.zeros(2)
        obs, rew, term, trunc, info = env.step(a)
        assert obs["state"].shape == (40,)
        for k in ("phi_dot_exec", "d_min", "max_qdot"):
            assert k in info
        assert not term and not trunc


def test_action_is_executed_without_projection(tmp_path):
    env = _env(tmp_path, scale=1.5)
    env.reset(seed=0)
    _, _, terminated, _, info = env.step(np.array([1.0, 0.0]))
    assert not terminated
    np.testing.assert_allclose(info["phi_dot_exec"], [1.5, 0.0])
    np.testing.assert_allclose(env._phi, [0.015, 0.0])
    assert "filter" not in info


def test_registered_initial_phase_is_used_without_projection(tmp_path):
    env = _env(tmp_path)
    obs, _ = env.reset(options={"initial_phi": [0.1, -0.1]})
    np.testing.assert_allclose(env._phi, [0.1, -0.1])
    np.testing.assert_allclose(obs["state"][4:8],
                               [np.sin(0.1), np.cos(0.1), np.sin(-0.1), np.cos(-0.1)])
    with pytest.raises(ValueError, match="initial_phi"):
        env.reset(options={"initial_phi": [2.0, 0.0]})


# --------------------------------------------------------------------------
# end termination (short trajectory)
# --------------------------------------------------------------------------
def test_end_termination(tmp_path):
    env = _env(tmp_path, n=3)
    env.reset(seed=0)
    obs, rew, term, trunc, info = env.step(np.zeros(2))
    assert not term and not trunc
    obs, rew, term, trunc, info = env.step(np.zeros(2))
    assert trunc
    assert info["termination"] == "end"


# --------------------------------------------------------------------------
# speed-violation termination
# --------------------------------------------------------------------------
def test_speed_violation_termination(tmp_path):
    env = _env(tmp_path, scale=5.0)
    env.reset(seed=0)
    obs, rew, term, trunc, info = env.step(np.array([1.0, 0.0]))
    assert term and not trunc
    assert info["termination"] == "speed"
    assert info["max_qdot"] > 1.5


def test_grid_exit_returns_terminal_transition(tmp_path):
    env = _env(tmp_path, scale=200.0)
    env.reset(seed=0)
    obs, rew, term, trunc, info = env.step(np.array([1.0, 0.0]))
    assert term and not trunc
    assert info["termination"] == "grid_exit"
    assert rew < 0
    assert obs["state"].shape == (40,)
    np.testing.assert_allclose(env._phi, [2.0, 0.0])


def test_out_of_range_action_is_rejected_not_clipped(tmp_path):
    env = _env(tmp_path)
    env.reset(seed=0)
    with pytest.raises(ValueError, match="action"):
        env.step(np.array([1.1, 0.0]))


def test_joint_limit_is_detected_before_any_clamp(tmp_path):
    env = _env(tmp_path, scale=1.5)
    env.reset(seed=0)
    q0 = env.lookup.query(env._x[None, :], env._phi[None, :], check_safe=False)["qL"][0, 0]
    env.lookup.joint_upper[0, 0] = q0 + 0.001
    _, rew, term, trunc, info = env.step(np.array([1.0, 0.0]))
    assert term and not trunc and rew < 0
    assert info["termination"] == "joint_limit"
    assert info["m_q"] < 0


def test_clearance_threshold_terminates(tmp_path):
    env = _env(tmp_path)
    env.reset(seed=0)
    env.lookup.dL[:] = 0.004
    _, rew, term, trunc, info = env.step(np.zeros(2))
    assert term and not trunc and rew < 0
    assert info["termination"] == "clearance"
    assert info["d_min"] < 0.005


def test_boundary_layer_penalizes_clearance_not_teacher_distance():
    args = (np.zeros(2), np.zeros(2), np.zeros(2))
    kwargs = dict(m_phi_minus=np.ones(2), m_phi_plus=np.ones(2),
                  m_q=1.0, lookup=None)
    safe, safe_info = reward_fn(*args, d_min=0.012, **kwargs)
    near, near_info = reward_fn(*args, d_min=0.0075, **kwargs)
    assert safe_info["C_margin"] == 0.0
    assert near_info["C_margin"] > 0.0
    assert near < safe
    assert "C_filter" not in near_info

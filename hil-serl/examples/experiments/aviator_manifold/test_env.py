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
        a = env.action_space.sample()
        obs, rew, term, trunc, info = env.step(a)
        assert obs["state"].shape == (40,)
        # info keys
        for k in ("filter", "phi_dot_nom", "phi_dot_safe", "d_min", "max_qdot"):
            assert k in info
        assert info["filter"]["feasible"]
        # constant task + safe phase (filter keeps phi in [-1,1]) -> no early stop
        assert not term and not trunc


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
def test_speed_violation_termination(tmp_path, monkeypatch):
    import examples.experiments.aviator_manifold.env as env_mod

    env = _env(tmp_path, scale=5.0)
    env.reset(seed=0)
    # bypass the shield: return the nominal velocity as if feasible, so the
    # env's own finite-step |qdot| check sees the violation (0.6 * 5.0 = 3.0 > 1.5)
    monkeypatch.setattr(
        env_mod, "project_phi_dot",
        lambda nom, x, x_next, phi, lookup, **kw:
            (nom, {"feasible": True, "intervened": False,
                   "clipped": False, "backtracked": 0}),
    )
    obs, rew, term, trunc, info = env.step(np.array([1.0, 0.0]))
    assert term and not trunc
    assert info["termination"] == "speed"
    assert info["max_qdot"] > 1.5

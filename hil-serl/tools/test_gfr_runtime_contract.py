"""Regression contracts exercised through the actual GFR environment."""
from pathlib import Path

import numpy as np
import pytest

from examples.experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.validate_online_lift import analytic_constraint_matrices

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def env():
    value = GFRManifoldEnv(
        str(ROOT / 'hil-serl/data/aviator/manifold_phi_stale_pitch-10deg'),
        str(ROOT / 'AviatorRobot/build/bin/model/aviator.xml'),
        str(ROOT / 'hil-serl/data/aviator/trajectory_source'),
        split='val', mode='raw',
    )
    value.reset(seed=0, options={'trajectory_index': 0})
    return value


@pytest.mark.parametrize('beta', [1., .75, .5, .25, .125])
def test_runtime_midpoint_obeys_tangent_equation(env, monkeypatch, beta):
    nominal_velocity = np.array([.08, .002])
    r = np.array([.1, -.1])
    original = env._lift
    residuals = []

    def capture(x, q, velocity, previous):
        result = original(x, q, velocity, previous)
        e, aq, ax = analytic_constraint_matrices(env.fk, x, q)
        _, n, he, hx, _ = result
        residuals.append(aq @ (he + beta * hx + n @ r)
                         + ax @ (beta * nominal_velocity) + 2. * e)
        return result

    monkeypatch.setattr(env, '_lift', capture)
    env._candidate(env._x, env._q, nominal_velocity, r, beta, None, False)
    assert len(residuals) == 2, 'both predictor and midpoint must be checked'
    assert max(np.max(np.abs(v)) for v in residuals) < 1e-9

"""Online trajectory coverage and transition-budget stopping regressions."""

import numpy as np
import pytest
from types import SimpleNamespace
from importlib import import_module

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from examples.experiments.aviator_manifold.test_manifold_lookup import make_manifold
from examples.experiments.aviator_manifold.vectorized_actor import VectorizedManifoldEnv
from serl_launcher.wrappers.chunking import ChunkingWrapper


@pytest.fixture
def config(tmp_path):
    manifold_dir, _ = make_manifold(
        tmp_path, joint_limits=([[-5.0] * 7] * 2, [[5.0] * 7] * 2)
    )
    trajectory_dir = tmp_path / "source"
    split_dir = trajectory_dir / "trajs" / "rl_train"
    split_dir.mkdir(parents=True)
    for i in range(4):
        x = np.tile([0.02 * i, -0.1], (2, 1))
        np.savez(split_dir / f"traj_{i:04d}.npz", x=x, xdot=np.zeros_like(x))

    class Config:
        max_online_episodes = 4

        def get_environment(self):
            env = AviatorManifoldEnv(manifold_dir, str(trajectory_dir),
                                     split="rl_train", phi_dot_scale=1.5)
            return ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)

    return Config()


def test_distinct_trajectory_coverage(config):
    vec = VectorizedManifoldEnv(config, n_envs=2, seed=0)
    vec.reset()
    observed = []
    while np.any(vec.active):
        for i, env in enumerate(vec.envs):
            if vec.active[i]:
                observed.append(round(float(env.unwrapped._traj["x"][0, 0]), 2))
        _, _, done, truncated, _ = vec.step(np.zeros((2, 2)))
        assert np.all(done | truncated | ~vec.active)
        vec.reset_done()
    assert sorted(observed) == [0.0, 0.02, 0.04, 0.06]


def test_repeated_shuffled_coverage(config):
    config.max_online_transitions = 24

    def collect(seed):
        vec = VectorizedManifoldEnv(config, n_envs=2, seed=seed)
        vec.reset()
        observed = []
        for _ in range(12):
            assert np.all(vec.active)
            observed.extend(round(float(e.unwrapped._traj["x"][0, 0]), 2)
                            for e in vec.envs)
            vec.step(np.zeros((2, 2)))
            vec.reset_done()
        return observed

    observed = collect(42)
    assert observed == collect(42)
    assert observed != collect(43)
    for start in range(0, 24, 4):
        assert sorted(observed[start:start + 4]) == [0.0, 0.02, 0.04, 0.06]


@pytest.mark.parametrize("step_cap, succeeds", [(10, True), (2, False)])
def test_actor_transition_budget(config, monkeypatch, step_cap, succeeds):
    module = import_module("examples.experiments.aviator_manifold.vectorized_actor")
    config.max_online_transitions = 9  # Includes a partial final vector batch.
    config.max_steps = step_cap
    config.random_steps = 100
    config.training_starts = 1
    config.updates_per_online_transition = 1
    config.log_period = 100
    config.actor_step_delay = 0
    stored, finished = [], []

    class Client:
        def __init__(self, *args, **kwargs):
            pass

        def recv_network_callback(self, callback):
            pass

        def update(self):
            return True

        def request(self, kind, payload):
            if kind == "learner-progress":
                return {"updates": len(stored)}
            if kind == "actor-done":
                finished.append(payload)
                return {"received": True}

        def stop(self):
            pass

    monkeypatch.setattr(module, "TrainerClient", Client)
    monkeypatch.setattr(module, "publisher_from_env", lambda: None)
    vec = VectorizedManifoldEnv(config, n_envs=2, seed=42)

    def run():
        module.vectorized_actor(None, SimpleNamespace(insert=stored.append), None,
                                vec, None, config, SimpleNamespace(ip="localhost"))

    if succeeds:
        run()
        assert len(stored) == 9
        assert finished[0]["online_transitions"] == 9
        assert vec._next_trajectory > config.max_online_episodes
    else:
        with pytest.raises(RuntimeError, match="actor step cap"):
            run()
        assert len(stored) == 4
        assert not finished

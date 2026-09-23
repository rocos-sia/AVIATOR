"""Online rollout coverage: each selected task trajectory is used once."""

import numpy as np

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from examples.experiments.aviator_manifold.test_manifold_lookup import make_manifold
from examples.experiments.aviator_manifold.vectorized_actor import VectorizedManifoldEnv
from serl_launcher.wrappers.chunking import ChunkingWrapper


def test_distinct_trajectory_coverage(tmp_path):
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

    vec = VectorizedManifoldEnv(Config(), n_envs=2, seed=0)
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

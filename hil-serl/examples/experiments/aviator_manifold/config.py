"""AviatorManifoldTrainConfig — HIL-SERL wiring for the redundancy policy (Task 2.1).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 2.1)

Proprio-only experiment, no cameras and no real robot:

  * image_keys=[]            (no vision -> proprio-only agents)
  * proprio_keys=["state"]   (the 40-D observation)
  * discount=0.999           (long-horizon gap-crossing)
  * max_traj_length=1200     (trajectories are up to 12 s @ 100 Hz)
  * replay_buffer_capacity=1_000_000

``phi_dot_scale`` is read from ``data/aviator/dp_demo/phi_dot_scale.json``
(Phase 3 calibration).  ``get_environment`` is simply
``AviatorManifoldEnv -> ChunkingWrapper(obs_horizon=1)``: no Spacemouse, no
RelativeFrame, no Quat2Euler (those are real-robot wrappers).
"""

from __future__ import annotations

import json
from pathlib import Path

from serl_launcher.wrappers.chunking import ChunkingWrapper

from experiments.config import DefaultTrainingConfig
from .env import AviatorManifoldEnv

# hil-serl/ root (config.py -> aviator_manifold -> experiments -> examples -> hil-serl)
_HIL_SERL_ROOT = Path(__file__).resolve().parents[3]
_MANIFOLD_DIR = _HIL_SERL_ROOT / "data" / "aviator" / "manifold_phi"
_TRAJECTORY_DIR = _HIL_SERL_ROOT / "data" / "aviator" / "trajectory_source"
_PHI_DOT_SCALE_PATH = _HIL_SERL_ROOT / "data" / "aviator" / "dp_demo" / "phi_dot_scale.json"


def _read_phi_dot_scale() -> float:
    with open(_PHI_DOT_SCALE_PATH) as f:
        return float(json.load(f)["phi_dot_scale"])


class TrainConfig(DefaultTrainingConfig):
    image_keys = []              # proprio-only: no cameras
    proprio_keys = ["state"]
    discount = 0.999
    max_traj_length = 1200
    replay_buffer_capacity = 1_000_000

    encoder_type = "proprio"     # placeholder; agent factories override it (Task 2.2)
    setup_mode = "single-arm-fixed-gripper"   # plain SAC branch in train_rlpd.py

    trajectory_split = "rl_train"            # actor rollout split (400 trajs)

    def __init__(self):
        self.phi_dot_scale = _read_phi_dot_scale()

    def get_environment(self, fake_env=False, save_video=False, classifier=False):
        del fake_env, save_video, classifier   # kinematic env: no robot, no camera
        env = AviatorManifoldEnv(
            manifold_dir=str(_MANIFOLD_DIR),
            trajectory_dir=str(_TRAJECTORY_DIR),
            split=self.trajectory_split,
            phi_dot_scale=self.phi_dot_scale,
        )
        env = ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)
        return env

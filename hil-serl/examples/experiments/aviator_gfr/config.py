"""Training configuration for the isolated GFR-1 online backend.

The historical ``aviator_manifold`` configuration intentionally remains
unchanged.  This module wires the same SAC/RLPD factory to ``GFRManifoldEnv``
so GFR experiments use the analytic lift, fixed-beta slice, governor and
independent endpoint/swept checker together.
"""

from __future__ import annotations

import os
from pathlib import Path

from serl_launcher.wrappers.chunking import ChunkingWrapper

from experiments.aviator_manifold.config import TrainConfig as _BaseTrainConfig
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv


_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_MANIFOLD = _ROOT / "data" / "aviator" / "manifold_phi_stale_pitch-10deg"
_DEFAULT_TRAJECTORIES = _ROOT / "data" / "aviator" / "trajectory_source"
_DEFAULT_MODEL = _ROOT.parent / "AviatorRobot" / "build" / "bin" / "model" / "aviator.xml"


class TrainConfig(_BaseTrainConfig):
    """GFR-1 configuration with a frozen two-dimensional action interface."""

    trajectory_split = os.getenv("AVIATOR_GFR_SPLIT", "rl_train")
    num_actor_envs = int(os.getenv("AVIATOR_GFR_NUM_ENVS", "1"))
    max_online_episodes = int(os.getenv("AVIATOR_GFR_MAX_EPISODES", "32"))
    checkpoint_period = int(os.getenv("AVIATOR_GFR_CHECKPOINT_PERIOD", "1000"))
    actor_step_delay = 0.0

    def __init__(self):
        super().__init__()
        self.gfr_mode = os.getenv("AVIATOR_GFR_MODE", "structured")
        self.gfr_visible_error = os.getenv("AVIATOR_GFR_VISIBLE_ERROR", "1") not in {
            "0", "false", "False"
        }
        self.gfr_model = os.getenv("AVIATOR_GFR_MODEL", str(_DEFAULT_MODEL))
        self.gfr_model_manifest = os.getenv("AVIATOR_GFR_MODEL_MANIFEST")
        self.gfr_dt = float(os.getenv("AVIATOR_GFR_DT", "0.005"))
        self.gfr_d_safe = float(os.getenv("AVIATOR_GFR_D_SAFE", "0.005"))
        self.gfr_beta_stop = float(os.getenv("AVIATOR_GFR_BETA_STOP", "0.125"))
        self.gfr_r_max = float(os.getenv("AVIATOR_GFR_R_MAX", "1.5"))
        self.gfr_qdot_max = float(os.getenv("AVIATOR_GFR_QDOT_MAX", "1.5"))
        self.gfr_correction_weight = float(os.getenv("AVIATOR_GFR_CORRECTION_WEIGHT", "0.0"))
        self.gfr_manifold = os.getenv("AVIATOR_GFR_MANIFOLD", str(_DEFAULT_MANIFOLD))
        self.gfr_trajectory_dir = os.getenv(
            "AVIATOR_GFR_TRAJECTORY_DIR", str(_DEFAULT_TRAJECTORIES)
        )

        if self.gfr_mode not in {"raw", "structured", "fixed"}:
            raise ValueError("AVIATOR_GFR_MODE must be raw, structured, or fixed")
        if not Path(self.gfr_model).exists():
            raise FileNotFoundError(self.gfr_model)
        if not Path(self.gfr_manifold).exists():
            raise FileNotFoundError(self.gfr_manifold)

    def get_environment(self, fake_env=False, save_video=False, classifier=False):
        del fake_env, save_video, classifier
        env = GFRManifoldEnv(
            manifold_dir=self.gfr_manifold,
            model_path=self.gfr_model,
            trajectory_dir=self.gfr_trajectory_dir,
            split=self.trajectory_split,
            mode=self.gfr_mode,
            visible_error=self.gfr_visible_error,
            dt=self.gfr_dt,
            d_safe=self.gfr_d_safe,
            qdot_max=self.gfr_qdot_max,
            beta_stop=self.gfr_beta_stop,
            r_max=self.gfr_r_max,
            model_manifest=self.gfr_model_manifest,
        )
        return ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)

    def make_sac_agent(self, seed, sample_obs, sample_action):
        agent = super().make_sac_agent(seed, sample_obs, sample_action)
        # ``SACAgent.config`` is a non-pytree field, so adding these optional
        # controls does not alter checkpoint layout or historical runs.
        return agent.replace(config={
            **agent.config,
            "correction_weight": self.gfr_correction_weight,
            "correction_action_scale": self.gfr_r_max,
        })

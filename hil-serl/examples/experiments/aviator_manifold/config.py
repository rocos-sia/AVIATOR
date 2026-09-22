"""AviatorManifoldTrainConfig — HIL-SERL wiring for the redundancy policy (Tasks 2.1/2.2).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Tasks 2.1, 2.2)

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

Task 2.2 agent factories: HIL-SERL's ``make_bc_agent`` / ``make_sac_pixel_agent``
build ResNet vision encoders (``image_keys`` non-empty) with mismatched MLP
widths (BC [512,512,512], SAC [256,256]).  The frozen v0.1 spec is a
proprio-only MLP(256,256,256) tanh-squashed Gaussian actor + MLP(256,256,256)
critic ensemble-2, so both agents are rebuilt here around ``StateEncoder`` with
*identical* actor structure for BC -> SAC checkpoint compatibility.
"""

from __future__ import annotations

import json
from functools import partial
from pathlib import Path

import flax.linen as nn
import jax
import optax
from serl_launcher.agents.continuous.bc import BCAgent
from serl_launcher.agents.continuous.sac import SACAgent
from serl_launcher.common.common import JaxRLTrainState, ModuleDict
from serl_launcher.networks.actor_critic_nets import Critic, Policy, ensemblize
from serl_launcher.networks.lagrange import GeqLagrangeMultiplier
from serl_launcher.networks.mlp import MLP
from serl_launcher.wrappers.chunking import ChunkingWrapper

from ..config import DefaultTrainingConfig
from .env import AviatorManifoldEnv
from .wrappers import StateEncoder

# hil-serl/ root (config.py -> aviator_manifold -> experiments -> examples -> hil-serl)
_HIL_SERL_ROOT = Path(__file__).resolve().parents[3]
_MANIFOLD_DIR = _HIL_SERL_ROOT / "data" / "aviator" / "manifold_phi"
_TRAJECTORY_DIR = _HIL_SERL_ROOT / "data" / "aviator" / "trajectory_source"
_PHI_DOT_SCALE_PATH = _HIL_SERL_ROOT / "data" / "aviator" / "dp_demo" / "phi_dot_scale.json"

# frozen v0.1 architecture: MLP(256,256,256) tanh, tanh-squashed Gaussian actor,
# critic MLP(256,256,256) ensemble-2.  BC and SAC share these so the BC actor
# checkpoint loads verbatim into the SAC actor.
_POLICY_NETWORK_KWARGS = {
    "hidden_dims": [256, 256, 256],
    "activations": nn.tanh,
    "use_layer_norm": True,
    "activate_final": True,
}
_CRITIC_NETWORK_KWARGS = {
    "hidden_dims": [256, 256, 256],
    "activations": nn.tanh,
    "use_layer_norm": True,
    "activate_final": True,
}
_POLICY_KWARGS = {
    "tanh_squash_distribution": True,
    "std_parameterization": "exp",
    "std_min": 1e-5,
    "std_max": 5.0,
}
_CRITIC_ENSEMBLE_SIZE = 2


def _read_phi_dot_scale() -> float:
    with open(_PHI_DOT_SCALE_PATH) as f:
        return float(json.load(f)["phi_dot_scale"])


class TrainConfig(DefaultTrainingConfig):
    image_keys = []              # proprio-only: no cameras
    proprio_keys = ["state"]
    discount = 0.999
    max_traj_length = 1200
    replay_buffer_capacity = 1_000_000

    encoder_type = "proprio"     # placeholder; agent factories override it
    setup_mode = "single-arm-fixed-gripper"   # plain SAC branch in train_rlpd.py

    # Drop the zero-action "hold" transitions (HIL-SERL teleop convention).  The
    # DP teacher holds phi_dot=0 88% of the time; including those holds makes
    # plain-MSE BC collapse to always-hold (28% completion, 72% infeasible), and
    # balancing to 30% moves still under-moves (38%).  Keeping only the ~12%
    # "reposition" moves gives 92% completion / e_task 0.057, at the cost of a
    # high intervention rate (79%) -- which the RLPD reward's C_filter term
    # (w_f=0.5) then drives down.  So filter out holds here.
    filter_zero_actions = True

    trajectory_split = "rl_train"            # actor rollout split (400 trajs)
    num_actor_envs = 8                       # Task 2.3: envs stepped in lockstep

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

    # -- agent factories (Task 2.2) -----------------------------------------
    def make_replay_buffer(self, observation_space, action_space, capacity):
        """Plain replay buffer (proprio-only: image_keys=[] means the
        memory-efficient buffer's pixel_key loop is a no-op and its insert()
        crashes on range(None))."""
        from serl_launcher.data.data_store import ReplayBufferDataStore
        return ReplayBufferDataStore(observation_space, action_space, capacity)

    def make_bc_agent(self, seed, sample_obs, sample_action):
        """Proprio-only BC agent, MLP(256,256,256) tanh-squashed Gaussian."""
        rng = jax.random.PRNGKey(seed)
        networks = {
            "actor": Policy(
                encoder=StateEncoder(),
                network=MLP(**_POLICY_NETWORK_KWARGS),
                action_dim=sample_action.shape[-1],
                **_POLICY_KWARGS,
                name="actor",
            )
        }
        model_def = ModuleDict(networks)
        tx = optax.adam(3e-4)
        rng, init_rng = jax.random.split(rng)
        params = model_def.init(init_rng, actor=[sample_obs])["params"]
        rng, create_rng = jax.random.split(rng)
        state = JaxRLTrainState.create(
            apply_fn=model_def.apply, params=params, txs=tx,
            target_params=params, rng=create_rng,
        )
        config = dict(
            # non-empty image_keys so update()'s image_keys[0] check is safe;
            # "state" is always present in next_observations -> no _unpack
            image_keys=["state"],
            augmentation_function=None,
            tanh_squash_distribution=_POLICY_KWARGS["tanh_squash_distribution"],
        )
        return BCAgent(state, config)

    def make_sac_agent(self, seed, sample_obs, sample_action):
        """Proprio-only SAC agent, actor identical to BC for checkpoint compat."""
        rng = jax.random.PRNGKey(seed)
        actor_def = Policy(
            encoder=StateEncoder(),
            network=MLP(**_POLICY_NETWORK_KWARGS),
            action_dim=sample_action.shape[-1],
            **_POLICY_KWARGS,
            name="actor",
        )
        critic_backbone = partial(MLP, **_CRITIC_NETWORK_KWARGS)
        critic_backbone = ensemblize(critic_backbone, _CRITIC_ENSEMBLE_SIZE)(
            name="critic_ensemble"
        )
        critic_def = partial(
            Critic, encoder=StateEncoder(), network=critic_backbone
        )(name="critic")
        temperature_def = GeqLagrangeMultiplier(
            init_value=1e-2, constraint_shape=(), constraint_type="geq",
            name="temperature",
        )
        return SACAgent.create(
            rng, sample_obs, sample_action,
            actor_def=actor_def, critic_def=critic_def, temperature_def=temperature_def,
            critic_ensemble_size=_CRITIC_ENSEMBLE_SIZE,
            critic_subsample_size=None,
            discount=self.discount,
            backup_entropy=False,
            image_keys=["state"],
            augmentation_function=None,
        )

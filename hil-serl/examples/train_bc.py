#!/usr/bin/env python3

import glob
import time
import jax
import jax.numpy as jnp
import numpy as np
import tqdm
from absl import app, flags
from flax.training import checkpoints
import os
import pickle as pkl
from gymnasium.wrappers import RecordEpisodeStatistics

from serl_launcher.agents.continuous.bc import BCAgent

from serl_launcher.utils.launcher import (
    make_bc_agent,
    make_trainer_config,
    make_wandb_logger,
)
from serl_launcher.data.data_store import (
    MemoryEfficientReplayBufferDataStore,
    ReplayBufferDataStore,
)

from experiments.mappings import CONFIG_MAPPING
from experiments.config import DefaultTrainingConfig
FLAGS = flags.FLAGS

flags.DEFINE_string("exp_name", None, "Name of experiment corresponding to folder.")
flags.DEFINE_integer("seed", 42, "Random seed.")
flags.DEFINE_string("ip", "localhost", "IP address of the learner.")
flags.DEFINE_string("bc_checkpoint_path", None, "Path to save checkpoints.")
flags.DEFINE_integer("eval_n_trajs", 0, "Number of trajectories to evaluate.")
flags.DEFINE_integer("train_steps", 20_000, "Number of pretraining steps.")
flags.DEFINE_bool("save_video", False, "Save video of the evaluation.")
flags.DEFINE_multi_string("demo_path", None, "Path to demo data pkl(s). Falls back to demo_data/*.pkl if unset.")


flags.DEFINE_boolean(
    "debug", False, "Debug mode."
)  # debug mode will disable wandb logging


devices = jax.local_devices()
num_devices = len(devices)
sharding = jax.sharding.PositionalSharding(devices)


def print_green(x):
    return print("\033[92m {}\033[00m".format(x))


def print_yellow(x):
    return print("\033[93m {}\033[00m".format(x))


def _balance_transitions(transitions, move_ratio):
    """Downsample the zero-action "hold" transitions to a target move fraction.

    ``transitions`` are BC demo dicts with an ``actions`` key.  ``move_ratio`` is
    the desired fraction of non-zero-action transitions in the returned list.
    All non-zero transitions are kept; holds are subsampled uniformly to hit the
    ratio (never upsampled -- holds are the majority, so only downsampling is
    needed in practice).
    """
    holds, moves = [], []
    for t in transitions:
        (moves if np.linalg.norm(np.asarray(t["actions"])) > 0.0 else holds).append(t)
    n_moves = len(moves)
    n_holds = len(holds)
    if n_moves == 0 or n_holds == 0:
        return transitions
    # keep k holds so n_moves / (n_moves + k) == move_ratio
    k = int(round(n_moves * (1.0 - move_ratio) / move_ratio))
    k = min(max(k, 0), n_holds)
    rng = np.random.default_rng(0)
    idx = rng.choice(n_holds, size=k, replace=False)
    balanced = moves + [holds[i] for i in idx]
    print(f"balanced demos: {n_moves} moves + {k} holds "
          f"({k + n_moves} total, move ratio {n_moves / (n_moves + k):.2f})")
    return balanced


##############################################################################

def eval(
    env,
    bc_agent: BCAgent,
    sampling_rng,
):
    """
    This is the actor loop, which runs when "--actor" is set to True.
    """
    success_counter = 0
    time_list = []
    for episode in range(FLAGS.eval_n_trajs):
        obs, _ = env.reset()
        done = False
        start_time = time.time()
        while not done:
            rng, key = jax.random.split(sampling_rng)

            actions = bc_agent.sample_actions(observations=obs, seed=key)
            actions = np.asarray(jax.device_get(actions))
            next_obs, reward, done, truncated, info = env.step(actions)
            obs = next_obs
            if done:
                if reward:
                    dt = time.time() - start_time
                    time_list.append(dt)
                    print(dt)
                success_counter += reward
                print(reward)
                print(f"{success_counter}/{episode + 1}")

    print(f"success rate: {success_counter / FLAGS.eval_n_trajs}")
    print(f"average time: {np.mean(time_list)}")


##############################################################################


def train(
    bc_agent: BCAgent,
    bc_replay_buffer,
    config: DefaultTrainingConfig,
    wandb_logger=None,
):

    # plain ReplayBuffer.sample() does not accept pack_obs_and_next_obs (only the
    # memory-efficient buffer does); pass it only for image-based experiments.
    sample_args = {"batch_size": config.batch_size}
    if config.image_keys:
        sample_args["pack_obs_and_next_obs"] = False
    bc_replay_iterator = bc_replay_buffer.get_iterator(
        sample_args=sample_args,
        device=sharding.replicate(),
    )
    
    # Pretrain BC policy to get started
    for step in tqdm.tqdm(
        range(FLAGS.train_steps),
        dynamic_ncols=True,
        desc="bc_pretraining",
    ):
        batch = next(bc_replay_iterator)
        bc_agent, bc_update_info = bc_agent.update(batch)
        if step % config.log_period == 0 and wandb_logger:
            wandb_logger.log({"bc": bc_update_info}, step=step)
        if step > FLAGS.train_steps - 100 and step % 10 == 0:
            checkpoints.save_checkpoint(
                os.path.abspath(FLAGS.bc_checkpoint_path), bc_agent.state, step=step, keep=5
            )
    print_green("bc pretraining done and saved checkpoint")


##############################################################################


def main(_):
    config: DefaultTrainingConfig = CONFIG_MAPPING[FLAGS.exp_name]()

    assert config.batch_size % num_devices == 0
    assert FLAGS.exp_name in CONFIG_MAPPING, "Experiment folder not found."
    eval_mode = FLAGS.eval_n_trajs > 0
    env = config.get_environment(
        fake_env=not eval_mode,
        save_video=FLAGS.save_video,
        classifier=True,
    )
    env = RecordEpisodeStatistics(env)

    if hasattr(config, "make_bc_agent"):
        # proprio-only factory override (config.py), no ResNet vision encoder
        bc_agent: BCAgent = config.make_bc_agent(
            seed=FLAGS.seed,
            sample_obs=env.observation_space.sample(),
            sample_action=env.action_space.sample(),
        )
    else:
        bc_agent: BCAgent = make_bc_agent(
            seed=FLAGS.seed,
            sample_obs=env.observation_space.sample(),
            sample_action=env.action_space.sample(),
            image_keys=config.image_keys,
            encoder_type=config.encoder_type,
        )

    # replicate agent across devices
    # need the jnp.array to avoid a bug where device_put doesn't recognize primitives
    bc_agent: BCAgent = jax.device_put(
        jax.tree_map(jnp.array, bc_agent), sharding.replicate()
    )

    if not eval_mode:
        assert not os.path.isdir(
            os.path.join(FLAGS.bc_checkpoint_path, f"checkpoint_{FLAGS.train_steps}")
        )

        if hasattr(config, "make_replay_buffer"):
            bc_replay_buffer = config.make_replay_buffer(
                env.observation_space,
                env.action_space,
                config.replay_buffer_capacity,
            )
        else:
            bc_replay_buffer = MemoryEfficientReplayBufferDataStore(
                env.observation_space,
                env.action_space,
                capacity=config.replay_buffer_capacity,
                image_keys=config.image_keys,
            )

        # set up wandb and logging
        wandb_logger = make_wandb_logger(
            project="hil-serl",
            description=FLAGS.exp_name,
            debug=FLAGS.debug,
        )

        if FLAGS.demo_path:
            demo_path = list(FLAGS.demo_path)
        else:
            demo_path = glob.glob(os.path.join(os.getcwd(), "demo_data", "*.pkl"))

        assert demo_path, "no demo data found (pass --demo_path or place pkls in demo_data/)"

        all_transitions = []
        for path in demo_path:
            with open(path, "rb") as f:
                transitions = pkl.load(f)
                for transition in transitions:
                    # The zero-action filter is a teleop convention (skip idle
                    # frames where the operator is not commanding).  For the DP
                    # teacher, phi_dot=0 is a *deliberate* "hold phase" action --
                    # 88% of the demos -- and dropping it leaves the BC with no
                    # hold examples, so it over-actuates and the safety filter
                    # clips ~80% of its steps.  Our config disables the filter.
                    if getattr(config, "filter_zero_actions", True) and \
                            np.linalg.norm(transition['actions']) <= 0.0:
                        continue
                    all_transitions.append(transition)

        # Class balance: the DP teacher is bimodal (88% hold phi_dot=0, 12%
        # small "reposition" moves).  Plain MSE over the raw distribution makes
        # the policy collapse to always-hold and miss the boundary moves that
        # actually cross the gap.  Downsample the holds to a target move
        # fraction (demo_balance_ratio) so the BC sees both regimes.
        balance = getattr(config, "demo_balance_ratio", None)
        if balance is not None:
            all_transitions = _balance_transitions(all_transitions, balance)

        for transition in all_transitions:
            bc_replay_buffer.insert(transition)
        print(f"bc replay buffer size: {len(bc_replay_buffer)}")

        # learner loop
        print_green("starting learner loop")
        train(
            bc_agent=bc_agent,
            bc_replay_buffer=bc_replay_buffer,
            wandb_logger=wandb_logger,
            config=config,
        )

    else:
        rng = jax.random.PRNGKey(FLAGS.seed)
        sampling_rng = jax.device_put(rng, sharding.replicate())

        bc_ckpt = checkpoints.restore_checkpoint(
            FLAGS.bc_checkpoint_path,
            bc_agent.state,
        )
        bc_agent = bc_agent.replace(state=bc_ckpt)

        print_green("starting actor loop")
        eval(
            env=env,
            bc_agent=bc_agent,
            sampling_rng=sampling_rng,
        )


if __name__ == "__main__":
    app.run(main)

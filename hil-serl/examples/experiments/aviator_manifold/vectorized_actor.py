"""8-way vectorized AviatorManifoldEnv actor (Task 2.3).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 2.3)

Runs ``n_envs`` ``AviatorManifoldEnv`` instances in lockstep and batches the
policy forward pass over their stacked observations once per control step.

Single process, not subprocesses: the kinematic env's ``step`` is pure numpy
(no MuJoCo physics to GIL-serialize), so the only per-step cost worth
parallelising is the JAX policy forward pass -- and that is *already* batched
over the ``(n_envs, 1, 40)`` observation dim on one device.  Each env keeps its
own read-only ``ManifoldLookup``, matching the plan's "each loads its own
lookup".

The actor speaks the standard HIL-SERL agentlace protocol: it registers a
``TrainerClient`` against the learner, receives network params over the wire,
and publishes the same ``{"environment": info}`` / timer stats.  Transitions
keep the demo's ``{"state": (1, 40)}`` shape (via ``ChunkingWrapper``) so the RL
replay buffer and the DP demo buffer share one format.
"""

from __future__ import annotations

import os
import time

import jax
import numpy as np
import tqdm

from agentlace.trainer import TrainerClient
from serl_launcher.utils.launcher import make_trainer_config
from serl_launcher.utils.timer_utils import Timer

from .live_stream import publisher_from_env


def aviator_trainer_config():
    config = make_trainer_config()
    config.request_types.append("actor-done")
    config.request_types.append("learner-progress")
    return config


class VectorizedManifoldEnv:
    """``n_envs`` kinematic envs stepped in lockstep with batched inference.

    Observations are stacked to ``{"state": (n_envs, 1, 40)}`` (the
    ``ChunkingWrapper(obs_horizon=1)`` shape), so one ``sample_actions`` call
    serves every env.  Envs that finished on the previous step are reset by
    ``reset_done()`` *before* the next action is sampled, so the terminal
    transition keeps ``mask=0`` and its pre-reset ``next_obs`` while the new
    episode's first action comes from its own initial observation (audit §4).
    """

    def __init__(self, config, n_envs: int = 8, seed: int = 0):
        self.n_envs = n_envs
        self._seed = seed
        self.envs = [config.get_environment() for _ in range(n_envs)]
        self.max_episodes = config.max_online_episodes
        available = len(self.envs[0].unwrapped._traj_paths)
        if self.max_episodes > available or self.max_episodes < n_envs:
            raise ValueError(f"need {self.max_episodes} distinct online trajectories; found {available}")
        self._next_trajectory = 0
        self.active = np.ones(n_envs, dtype=bool)
        self.obs = None                       # {"state": (n_envs, 1, 40)}
        self._done = np.zeros(n_envs, dtype=bool)

    @staticmethod
    def _stack(obs_list):
        return {"state": np.stack([o["state"] for o in obs_list]).astype(np.float32)}

    def reset(self):
        obss = []
        self._next_trajectory = 0
        self.active[:] = True
        for i, e in enumerate(self.envs):
            obs, _ = e.reset(seed=self._seed + i,
                             options={"trajectory_index": self._next_trajectory})
            self._next_trajectory += 1
            obss.append(obs)
        self.obs = self._stack(obss)
        self._done[:] = False
        return self.obs

    def reset_done(self):
        """Reset envs that finished last episode; call BEFORE sampling actions.

        Each reset gets the next unused trajectory from rl_train. Doing it here
        makes the new episode's first action come from its own initial
        observation, not the prior episode's terminal observation (audit §4).
        """
        for i, e in enumerate(self.envs):
            if self._done[i]:
                if self._next_trajectory < self.max_episodes:
                    obs, _ = e.reset(options={"trajectory_index": self._next_trajectory})
                    self.obs["state"][i] = obs["state"]
                    self._next_trajectory += 1
                else:
                    self.active[i] = False
                self._done[i] = False
        return self.obs

    def step(self, actions):
        actions = np.asarray(actions, dtype=np.float32)
        assert actions.shape == (self.n_envs, 2), actions.shape

        next_obss, rewards, dones, truncs, infos = [], [], [], [], []
        for i, e in enumerate(self.envs):
            if not self.active[i]:
                next_obss.append({"state": self.obs["state"][i]})
                rewards.append(0.0)
                dones.append(False)
                truncs.append(False)
                infos.append({"inactive": True})
                continue
            obs, r, d, t, info = e.step(actions[i])
            next_obss.append(obs)
            rewards.append(r)
            dones.append(d)
            truncs.append(t)
            infos.append(info)
            self._done[i] = bool(d or t)

        self.obs = self._stack(next_obss)
        return (
            self.obs,
            np.asarray(rewards, dtype=np.float32),
            np.asarray(dones, dtype=bool),
            np.asarray(truncs, dtype=bool),
            infos,
        )


def vectorized_actor(agent, data_store, intvn_data_store, vec_env, sampling_rng,
                     config, flags):
    """The ``--actor`` loop for the 8-way env.

    Mirrors ``train_rlpd.py:actor`` but steps ``n_envs`` envs per iteration and
    batches the policy forward pass. The transition stores the policy action
    that the unshielded environment executes.
    """
    datastore_dict = {
        "actor_env": data_store,
        "actor_env_intvn": intvn_data_store,
    }
    client = TrainerClient(
        "actor_env",
        flags.ip,
        aviator_trainer_config(),
        data_stores=datastore_dict,
        wait_for_server=True,
        timeout_ms=3000,
    )

    def update_params(params):
        nonlocal agent
        agent = agent.replace(state=agent.state.replace(params=params))

    client.recv_network_callback(update_params)

    obs = vec_env.reset()

    publisher = publisher_from_env()

    timer = Timer()
    episode_returns = np.zeros(vec_env.n_envs, dtype=np.float64)
    # rolling window counters for the learner stats
    window_steps = 0
    window_max_qdot = 0.0
    window_min_d = np.inf
    window_episode_returns = []
    window_episodes = 0
    window_completed = 0
    window_clearance = 0
    window_speed = 0
    window_joint_limit = 0
    window_grid_exit = 0
    window_branch = 0
    online_steps = 0

    pbar = tqdm.tqdm(range(config.max_steps), dynamic_ncols=True)
    for step in pbar:
        loop_started = time.monotonic()
        timer.tick("total")

        # reset envs that finished last episode BEFORE sampling, so the new
        # episode's first action is drawn from its own initial obs (audit §4)
        with timer.context("reset_done"):
            obs = vec_env.reset_done()
        if not np.any(vec_env.active):
            break

        with timer.context("sample_actions"):
            if step < config.random_steps:
                actions = np.stack([
                    vec_env.envs[i].action_space.sample()
                    for i in range(vec_env.n_envs)
                ])
            else:
                sampling_rng, key = jax.random.split(sampling_rng)
                actions = agent.sample_actions(
                    observations=jax.device_put(obs),
                    seed=key,
                    argmax=False,
                )
                actions = np.array(jax.device_get(actions), dtype=np.float32)

        with timer.context("step_env"):
            next_obs, reward, done, truncated, info = vec_env.step(actions)

            for i in range(vec_env.n_envs):
                if not vec_env.active[i]:
                    continue
                window_steps += 1
                qdot = float(info[i].get("max_qdot", np.nan))
                clearance = float(info[i].get("d_min", np.nan))
                if np.isfinite(qdot):
                    window_max_qdot = max(window_max_qdot, qdot)
                if np.isfinite(clearance):
                    window_min_d = min(window_min_d, clearance)
                episode_returns[i] += float(reward[i])

                # The stored action is exactly what env.step() executed. Keep
                # the resulting next observation and terminal flag together.
                episode_done = bool(done[i] or truncated[i])
                transition = dict(
                    observations={"state": obs["state"][i]},
                    actions=actions[i],
                    next_observations={"state": next_obs["state"][i]},
                    rewards=np.asarray(reward[i], dtype=np.float32),
                    masks=np.asarray(1.0 - episode_done, dtype=np.float32),
                    dones=np.asarray(episode_done, dtype=bool),
                )
                data_store.insert(transition)
                online_steps += 1

                if done[i] or truncated[i]:
                    window_episodes += 1
                    window_episode_returns.append(float(episode_returns[i]))
                    episode_returns[i] = 0.0
                    term = info[i].get("termination", "")
                    window_completed += int(term == "end")
                    window_clearance += int(term == "clearance")
                    window_speed += int(term == "speed")
                    window_joint_limit += int(term == "joint_limit")
                    window_grid_exit += int(term == "grid_exit")
                    window_branch += int(term == "branch")

        obs = next_obs

        # Give the critic a chance to consume each new batch before spending
        # another set of distinct trajectories. Otherwise a fast actor can
        # exhaust all 400 starts while the learner is still compiling.
        if online_steps >= config.training_starts:
            if not client.update():
                raise RuntimeError("could not flush online transitions to learner")
            target_updates = online_steps * config.updates_per_online_transition
            deadline = time.monotonic() + 120
            while True:
                progress = client.request("learner-progress", {})
                if progress is not None and progress.get("updates", -1) >= target_updates:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"learner stalled before update {target_updates}: {progress}")
                time.sleep(0.1)

        # Stream env 0's latest kinematic state to a live viewer, if enabled.
        if publisher is not None and vec_env.active[0]:
            i0 = info[0]
            if i0.get("q") is not None:
                publisher.publish(i0["x"], i0["q"],
                                  float(i0.get("d_min", np.nan)),
                                  float(i0.get("max_qdot", np.nan)))

        timer.tock("total")

        if step % config.log_period == 0:
            window_steps = max(window_steps, 1)
            stats = {
                "environment": {
                    "max_qdot": window_max_qdot,
                    "min_d": window_min_d,
                    "mean_episode_return": (float(np.mean(window_episode_returns))
                                            if window_episode_returns else None),
                    "completion_rate": window_completed / max(window_episodes, 1),
                    "termination_causes": {
                        "end": window_completed,
                        "clearance": window_clearance,
                        "speed": window_speed,
                        "joint_limit": window_joint_limit,
                        "grid_exit": window_grid_exit,
                        "branch": window_branch,
                    },
                },
                "timer": timer.get_average_times(),
            }
            client.request("send-stats", stats)
            client.update()

            pbar.set_description(
                f"completed={window_completed}/{window_episodes}, "
                f"mean J={np.mean(window_episode_returns) if window_episode_returns else float('nan'):.2f}"
            )
            # reset window
            window_steps = 0
            window_max_qdot = 0.0
            window_min_d = np.inf
            window_episode_returns = []
            window_episodes = 0
            window_completed = 0
            window_clearance = 0
            window_speed = 0
            window_joint_limit = 0
            window_grid_exit = 0
            window_branch = 0

        delay = getattr(config, "actor_step_delay", 0.0)
        if delay > 0:
            time.sleep(max(0.0, delay - (time.monotonic() - loop_started)))

    # Flush transitions before announcing completion to the learner. A step
    # cap hit is a failed run, not a successful 400-episode experiment.
    client.update()
    complete = not np.any(vec_env.active)
    if complete:
        response = client.request("actor-done", {"episodes": vec_env._next_trajectory})
        if response != {"received": True}:
            raise RuntimeError(f"learner did not acknowledge actor completion: {response}")
    client.stop()
    pbar.close()
    if publisher is not None:
        publisher.close()
    if not complete:
        raise RuntimeError(f"actor step cap reached after assigning "
                           f"{vec_env._next_trajectory}/{vec_env.max_episodes} episodes")
    return

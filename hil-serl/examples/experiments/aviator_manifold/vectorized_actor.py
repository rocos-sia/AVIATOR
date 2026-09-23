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
        self.obs = None                       # {"state": (n_envs, 1, 40)}
        self._done = np.zeros(n_envs, dtype=bool)

    @staticmethod
    def _stack(obs_list):
        return {"state": np.stack([o["state"] for o in obs_list]).astype(np.float32)}

    def reset(self):
        obss = []
        for i, e in enumerate(self.envs):
            # seed only on the FIRST reset: re-seeding every episode makes each
            # worker replay the same trajectory forever (audit 2026-09-23 §3).
            obs, _ = e.reset(seed=self._seed + i)
            obss.append(obs)
        self.obs = self._stack(obss)
        self._done[:] = False
        return self.obs

    def reset_done(self):
        """Reset envs that finished last episode; call BEFORE sampling actions.

        No seed is passed, so each env's RNG advances and draws a *different*
        trajectory per episode (over ~800 episodes/worker this covers the full
        400-traj set instead of 8).  Doing the reset here (not inside step)
        makes the new episode's first action come from its own initial
        observation, not the prior episode's terminal observation (audit §4).
        """
        for i, e in enumerate(self.envs):
            if self._done[i]:
                obs, _ = e.reset()
                self.obs["state"][i] = obs["state"]
                self._done[i] = False
        return self.obs

    def step(self, actions):
        actions = np.asarray(actions, dtype=np.float32)
        assert actions.shape == (self.n_envs, 2), actions.shape

        next_obss, rewards, dones, truncs, infos = [], [], [], [], []
        for i, e in enumerate(self.envs):
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
    batches the policy forward pass.  The transition stores the policy's
    *nominal* action (what the env actually received); the safety filter's
    effect is already reflected in the reward (C_filter) and in the next obs's
    action history, so the (s, a, r, s') tuple stays consistent (audit §2).
    """
    datastore_dict = {
        "actor_env": data_store,
        "actor_env_intvn": intvn_data_store,
    }
    client = TrainerClient(
        "actor_env",
        flags.ip,
        make_trainer_config(),
        data_stores=datastore_dict,
        wait_for_server=True,
        timeout_ms=3000,
    )

    def update_params(params):
        nonlocal agent
        agent = agent.replace(state=agent.state.replace(params=params))

    client.recv_network_callback(update_params)

    obs = vec_env.reset()

    timer = Timer()
    running_return = 0.0
    # rolling window counters for the learner stats
    window_steps = 0
    window_intervened = 0
    window_max_qdot = 0.0
    window_min_d = np.inf
    window_return = 0.0
    window_episodes = 0
    window_completed = 0

    pbar = tqdm.tqdm(range(config.max_steps), dynamic_ncols=True)
    for step in pbar:
        timer.tick("total")

        # reset envs that finished last episode BEFORE sampling, so the new
        # episode's first action is drawn from its own initial obs (audit §4)
        with timer.context("reset_done"):
            obs = vec_env.reset_done()

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
                fi = info[i].get("filter", {})
                window_steps += 1
                window_intervened += int(bool(fi.get("intervened")))
                window_max_qdot = max(window_max_qdot, float(info[i].get("max_qdot", 0.0)))
                window_min_d = min(window_min_d, float(info[i].get("d_min", np.inf)))
                window_return += float(reward[i])

                # Store the *nominal* action (what the policy output and what
                # env.step() received).  The env already accounts for the safety
                # filter in the reward (C_filter) and in the next obs (a_prev /
                # hist_a carry the nominal action); overwriting it with
                # phi_dot_safe made (s, a, r, s') inconsistent -- the critic
                # learned Q(s, a_safe) while the policy optimizes over a_nominal
                # (audit §2).
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

                if done[i] or truncated[i]:
                    window_episodes += 1
                    window_completed += int(info[i].get("termination", "") == "end")

        obs = next_obs

        timer.tock("total")

        if step % config.log_period == 0:
            window_steps = max(window_steps, 1)
            stats = {
                "environment": {
                    "intervention_rate": window_intervened / window_steps,
                    "max_qdot": window_max_qdot,
                    "min_d": window_min_d,
                    "mean_episode_return": window_return / max(window_episodes, 1),
                    "completion_rate": window_completed / max(window_episodes, 1),
                },
                "timer": timer.get_average_times(),
            }
            client.request("send-stats", stats)
            client.update()

            pbar.set_description(
                f"last 10-step return: {window_return / max(window_episodes, 1):.2f}"
            )
            # reset window
            window_steps = 0
            window_intervened = 0
            window_max_qdot = 0.0
            window_min_d = np.inf
            window_return = 0.0
            window_episodes = 0
            window_completed = 0

    return

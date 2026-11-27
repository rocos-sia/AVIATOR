"""Fresh SAC training, qualified only on development data (never heldout).

Requires torch and stable-baselines3 in the isolated experiment environment.
This is a new experiment, not a reproduction of the missing historical 60k.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np

from tools.audit_static_collision import CollisionChecker
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import ROOT, RULES, audit_motion, dump, rollout, sha
from tools.gfr1_shared_remount_env import SharedRemountEnv


class LearningEnv(SharedRemountEnv):
    def __init__(self, *args, episode_sink=None, projection_cost_units=8., **kwargs):
        super().__init__(*args, **kwargs)
        self.episode_sink = episode_sink
        self.training_transitions = 0
        self.projection_cost_units = float(projection_cost_units)

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        self.episode_steps = 0
        self.episode_return = 0.
        self.episode_fallbacks = 0
        self.audit = CollisionChecker(self.model_path)
        self.duration_limit = int(np.floor(2*(len(self._traj["x"])-1)*self.traj_dt/self.dt+1e-8))
        return observation, info

    def step(self, action):
        x0, q0, u0 = self._x.copy(), self._q.copy(), self._u
        previous_raw = self._r_prev/self.r_max
        (obs, _, terminated, truncated, info), direct, first = self.action_step(action, True)
        self.episode_steps += 1
        self.training_transitions += 1
        self.episode_fallbacks += int(not direct)
        if info["beta_executed"] > 0:
            audit = audit_motion(self, self.audit, x0, q0)
            if audit["violations"]:
                # Accepted unsafe states are an executor failure, not a sample
                # the learner is allowed to silently learn around.
                if self.episode_sink:
                    self.episode_sink.write(json.dumps(dict(event="safety_failure", violations=audit["violations"],
                        model=self.model_path, transition=self.training_transitions))+"\n")
                raise RuntimeError("Training independent audit failed: "+repr(audit["violations"]))
        candidate_count = int(info.get("actual_candidate_evaluations", info["candidate_count"])) + int(not direct)
        # Frozen proxy units. Actual inference/geometry milliseconds are measured
        # outside training; no wall-clock jitter is fed into rewards.
        cost_units = 1. if direct else 1. + self.projection_cost_units*max(candidate_count-1, 1)
        reward = 1000.*((self._u-u0)/max(len(self._traj["x"])-1, 1)) - .01*cost_units
        reward -= .01*float(np.sum((np.asarray(action)-previous_raw)**2))
        if truncated and info["termination"] == "end":
            reward += 100.
            terminated, truncated = True, False
        elif terminated:
            reward -= 1000.
        elif self.episode_steps >= self.duration_limit:
            terminated = True
            info["termination"] = "simulation_duration_limit"
            reward -= 1000.
        if not np.isfinite(obs).all() or not np.isfinite(reward):
            raise FloatingPointError("Nonfinite training observation/reward")
        self.episode_return += reward
        if (terminated or truncated) and self.episode_sink:
            self.episode_sink.write(json.dumps(dict(transition=self.training_transitions, model=self.model_path,
                steps=self.episode_steps, termination=info["termination"], progress=info["progress"],
                reward=self.episode_return, fallbacks=self.episode_fallbacks))+"\n")
        return obs, reward, terminated, truncated, info


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--baseline-gate", type=Path, required=True)
    p.add_argument("--profile-json", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, choices=RULES["training_seeds"], required=True)
    p.add_argument("--cpu", type=int, default=2)
    a = p.parse_args()
    for gate in (a.dataset/"gate.json", a.baseline_gate):
        if not json.loads(gate.read_text())["passed"]:
            raise RuntimeError("Unpassed prerequisite: "+str(gate))
    os.sched_setaffinity(0, {a.cpu})
    import torch
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import BaseCallback
    torch.set_num_threads(1)
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    train = json.loads((a.dataset/"train.json").read_text())
    development = json.loads((a.dataset/"development.json").read_text())
    profiles = json.loads(a.profile_json.read_text())
    direct_cost = np.mean([p["cycle_ms"]["mean"] for p in profiles if p["available"] and p["path"] == "direct_pass"])
    projected_cost = np.mean([p["cycle_ms"]["mean"] for p in profiles if p["available"] and p["path"] == "project_same_passing_action"])
    if not np.isfinite(direct_cost+projected_cost) or direct_cost <= 0:
        raise ValueError("Missing measured cost calibration")
    projection_cost_units = projected_cost/direct_cost
    data = ROOT/"hil-serl/data/aviator"
    def make_env(cls, **kw):
        return cls(str(data/"manifold_phi_stale_pitch-10deg"), train["models"][0],
            str(data/"trajectory_source"), model_manifest=str(a.dataset/"train.json"), **kw)
    configuration = dict(algorithm="SAC", seed=a.seed, steps=60000, learning_starts=2000,
        batch_size=256, buffer_size=100000, learning_rate=3e-4, gamma=.995, tau=.005,
        train_freq=1, gradient_steps=1, net_arch=[128,128], ent_coef="auto", device="cpu",
        observation="40 scaled kinematic features + 30 estimated-installation pose features + 28 current null-axis components",
        action_change_penalty=.01,
        actor_initialization="fresh random", demonstration_buffer=None,
        cost_proxy_calibration=dict(profile_sha256=sha(a.profile_json), direct_mean_ms=direct_cost,
            projected_mean_ms=projected_cost, projection_cost_units=projection_cost_units),
        training_executor="direct first, repaired fallback", checkpoints=[20000,40000,60000],
        selection="among fully completing development checkpoints, lowest median per-case P95",
        gates={str(g):sha(g) for g in (a.dataset/"gate.json", a.baseline_gate)},
        manifest_hashes={s:sha(a.dataset/(s+".json")) for s in ("train", "development", "heldout")},
        sources={str(Path(__file__)):sha(__file__),str(Path(__file__).with_name("gfr1_shared_remount_env.py")):sha(Path(__file__).with_name("gfr1_shared_remount_env.py"))})
    dump(out/"config.json", configuration)
    evaluations = []
    class Checkpoints(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 1000 == 0:
                if any(not torch.isfinite(p).all() for p in self.model.policy.parameters()):
                    raise FloatingPointError("Nonfinite learner parameters")
                dump(out/"progress.json", dict(transitions=self.num_timesteps, elapsed_s=time.monotonic()-started))
            if self.num_timesteps in (20000,40000,60000):
                checkpoint = out/f"checkpoint_{self.num_timesteps}"
                self.model.save(checkpoint)
                evaluation = make_env(SharedRemountEnv)
                rows = []
                with (out/f"development_{self.num_timesteps}.jsonl").open("x") as sink:
                    for record in development["records"]:
                        rows.append(rollout(evaluation, record, "E", sink,
                            policy=lambda core: self.model.predict(core.last_policy_observation, deterministic=True)[0]))
                evaluation.close()
                summary = dict(step=self.num_timesteps, checkpoint=str(checkpoint)+".zip",
                    checkpoint_sha256=sha(str(checkpoint)+".zip"),
                    qualified=all(r["completed"] for r in rows), episodes=rows)
                evaluations.append(summary)
                dump(out/"development_evaluations.json", evaluations)
                print(json.dumps(dict(step=self.num_timesteps, completed=sum(r["completed"] for r in rows), cases=len(rows))), flush=True)
            return True
    with (out/"training_episodes.jsonl").open("x", buffering=1) as sink, use_projector("combined"):
        env = make_env(LearningEnv, episode_sink=sink, projection_cost_units=projection_cost_units)
        model = SAC("MlpPolicy", env, learning_rate=3e-4, buffer_size=100000, learning_starts=2000,
            batch_size=256, tau=.005, gamma=.995, train_freq=1, gradient_steps=1,
            policy_kwargs=dict(net_arch=[128,128]), seed=a.seed, device="cpu", verbose=0)
        started = time.monotonic()
        try:
            model.learn(total_timesteps=60000, callback=Checkpoints())
            qualified = [e for e in evaluations if e["qualified"]]
            selected = min(qualified, key=lambda e: np.median([r["cycle_ms"]["p95"] for r in e["episodes"]])) if qualified else None
            dump(out/"gate.json", dict(passed=selected is not None, transitions=model.num_timesteps,
                selected=selected, reason="qualified" if selected else "no_checkpoint_completed_all_development_cases"))
        except Exception as error:
            model.save(out/"interrupted_debug_only")
            dump(out/"gate.json", dict(passed=False, transitions=model.num_timesteps,
                reason=type(error).__name__+": "+str(error), selected=None))
            raise
        finally:
            env.close()


if __name__ == "__main__":
    main()

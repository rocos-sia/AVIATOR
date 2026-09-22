"""evaluate_policy.py — BC gate evaluation (Task 2.2, BLOCKER #12).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 2.2)

Rolls the trained BC policy and three baselines out over held-out trajectories
and reports the BC gate metrics:

  * collision rate         (episodes ending in ``d_min < 5 mm``)
  * max |q̇|               (hard joint-speed bound, must be <= 1.5 rad/s)
  * e_task                 (completion shortfall: 1 - steps_done / total_steps)
  * intervention_rate      (fraction of steps the 2-D safety filter clipped)
  * oracle_regret          (J_BC - J_DP on the dp_train split)
  * action variance        (Var[a_DP | obs] over K-NN demo neighbours)

Baselines
---------
  * ``dp_oracle``  -- the full-horizon DP teacher's phi_dot* re-read from CSV.
  * ``static``     -- no redundancy: phi_dot = 0 (anchor phase held fixed).
  * ``greedy``     -- myopic: move phi toward the safe-interval centre at full
                      action scale every step (the safety filter clamps it).

The BC policy is deterministic (``sample_actions(argmax=True)`` = distribution
mode).

Run from ``hil-serl/``::

    python -m tools.evaluate_policy --split val --bc-checkpoint examples/experiments/aviator_manifold/debug
"""

from __future__ import annotations

import argparse
import glob
import json
import os

import jax
import numpy as np
from flax.training import checkpoints
from scipy.spatial import cKDTree

from examples.experiments.aviator_manifold.config import TrainConfig
from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

# frozen v0.1 constants (mirror env.py)
_D_SAFE = 0.005          # m
_QDOT_MAX = 1.5          # rad/s

# record_dp CSV column indices (see tools/build_dp_dataset.py COLUMNS)
_CSV_COL_PHI_DOT_L = 9
_CSV_COL_PHI_DOT_R = 10


# ---------------------------------------------------------------------------
# policies
# ---------------------------------------------------------------------------
def make_static_policy():
    """No redundancy: hold the anchor phase (phi_dot = 0)."""
    def policy(obs, env):
        return np.zeros(2)
    return policy


def make_greedy_policy():
    """Myopic: drive phi toward the safe-interval centre each step."""
    def policy(obs, env):
        x_next = env._traj["x"][env._t + 1]
        phi = env._phi
        box = env.lookup.safe_interval(x_next[None, :])
        center = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
        phi_dot = (center - phi) / env.dt
        return np.clip(phi_dot / env.phi_dot_scale, -1.0, 1.0)
    return policy


def make_dp_oracle_policy(phi_dot, phi_dot_scale):
    """Scripted: replay the DP teacher's phi_dot* (indexed by env._t)."""
    def policy(obs, env):
        return np.clip(phi_dot[env._t] / phi_dot_scale, -1.0, 1.0)
    return policy


def make_bc_policy(bc_agent):
    """Trained BC agent, deterministic mode."""
    def policy(obs, env):
        o = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
        a = bc_agent.sample_actions(observations=o, argmax=True)
        return np.asarray(a)
    return policy


# ---------------------------------------------------------------------------
# rollout
# ---------------------------------------------------------------------------
def roll_out(env, trajs, policy_fn):
    """Roll ``policy_fn`` over each trajectory; return per-episode metric dicts.

    ``env`` is constructed with ``trajectories=trajs``; here we narrow
    ``env._trajectories`` to one trajectory at a time so ``reset()`` is
    deterministic (this reuses ``reset``'s exact init rather than duplicating
    it).
    """
    episodes = []
    for traj in trajs:
        env._trajectories = [traj]
        obs, _ = env.reset()

        steps = 0
        intervened = 0
        max_qdot = 0.0
        min_d = np.inf
        reward_sum = 0.0
        done = False
        termination = "end"

        while not done:
            a = policy_fn(obs, env)
            obs, reward, terminated, truncated, info = env.step(a)
            steps += 1
            intervened += int(info["filter"]["intervened"])
            # the env's infeasible early-return omits d_min / max_qdot
            if "max_qdot" in info:
                max_qdot = max(max_qdot, float(info["max_qdot"]))
            if "d_min" in info:
                min_d = min(min_d, float(info["d_min"]))
            reward_sum += float(reward)
            done = terminated or truncated
            if terminated:
                termination = ("infeasible" if not info["filter"]["feasible"]
                               else info.get("termination", "terminated"))

        total_steps = len(env._traj["x"]) - 1
        episodes.append({
            "termination": termination,
            "steps": steps,
            "total_steps": total_steps,
            "max_qdot": max_qdot,
            "min_d": min_d,
            "intervention_rate": intervened / steps if steps else 0.0,
            "e_task": 1.0 - steps / total_steps if total_steps else 0.0,
            "J": reward_sum,
        })
    return episodes


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------
def summarize(episodes):
    n = len(episodes)
    if n == 0:
        return {}
    term = np.array([e["termination"] for e in episodes])
    return {
        "n_episodes": n,
        "completion_rate": float((term == "end").mean()),
        "collision_rate": float((term == "collision").mean()),
        "speed_violation_rate": float((term == "speed").mean()),
        "infeasible_rate": float((term == "infeasible").mean()),
        "max_qdot": float(np.max([e["max_qdot"] for e in episodes])),
        "mean_max_qdot": float(np.mean([e["max_qdot"] for e in episodes])),
        "mean_min_d_mm": float(np.mean([e["min_d"] for e in episodes]) * 1e3),
        "min_min_d_mm": float(np.min([e["min_d"] for e in episodes]) * 1e3),
        "mean_e_task": float(np.mean([e["e_task"] for e in episodes])),
        "mean_intervention_rate": float(np.mean([e["intervention_rate"] for e in episodes])),
        "mean_J": float(np.mean([e["J"] for e in episodes])),
    }


def print_table(results):
    header = (f"{'policy':<12} {'comp%':>6} {'coll%':>6} {'spd%':>6} {'inf%':>6} "
              f"{'max|qdot|':>9} {'min_d_mm':>8} {'e_task':>7} {'interv%':>7} {'J':>9}")
    print(header)
    print("-" * len(header))
    for name, s in results.items():
        print(f"{name:<12} {s['completion_rate']*100:6.1f} {s['collision_rate']*100:6.1f} "
              f"{s['speed_violation_rate']*100:6.1f} {s['infeasible_rate']*100:6.1f} "
              f"{s['max_qdot']:9.3f} {s['min_min_d_mm']:8.2f} {s['mean_e_task']:7.4f} "
              f"{s['mean_intervention_rate']*100:7.1f} {s['mean_J']:9.3f}")


# ---------------------------------------------------------------------------
# diagnostics
# ---------------------------------------------------------------------------
def action_variance_diagnostic(demo_dir, K=10, n_query=2000, seed=0):
    """Var[a_DP | obs] via K-NN in observation space (clairvoyance diagnostic).

    For ``n_query`` demo transitions, find the ``K`` nearest demo observations
    (L2 in the 40-D obs space) and report the distribution of the variance of
    their actions.  High variance => the teacher's action depends on
    information (future) the student cannot see; this bounds BC's oracle gap.
    """
    obs_list, act_list = [], []
    for path in sorted(glob.glob(os.path.join(demo_dir, "traj_*.pkl"))):
        with open(path, "rb") as f:
            import pickle as pkl
            for t in pkl.load(f):
                obs_list.append(np.asarray(t["observations"]["state"], dtype=np.float32).reshape(-1))
                act_list.append(np.asarray(t["actions"], dtype=np.float32))
    X = np.stack(obs_list)
    A = np.stack(act_list)
    rng = np.random.default_rng(seed)
    q_idx = rng.choice(len(X), size=min(n_query, len(X)), replace=False)
    tree = cKDTree(X)
    _, idx = tree.query(X[q_idx], k=K + 1)   # +1 to drop the query point itself
    var = np.array([np.var(A[idx[i, 1:]], axis=0).sum() for i in range(len(q_idx))])
    return {
        "K": K,
        "n_query": len(q_idx),
        "var_mean": float(np.mean(var)),
        "var_median": float(np.median(var)),
        "var_p90": float(np.percentile(var, 90)),
        "var_max": float(np.max(var)),
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi/")
    ap.add_argument("--trajectory-dir", default="data/aviator/trajectory_source/")
    ap.add_argument("--demo-dir", default="data/aviator/dp_demo/")
    ap.add_argument("--split", default="val", help="held-out split to evaluate")
    ap.add_argument("--bc-checkpoint", default="examples/experiments/aviator_manifold/debug")
    ap.add_argument("--n-eps", type=int, default=None, help="limit episodes per policy")
    ap.add_argument("--skip-oracle", action="store_true", help="skip oracle_regret (slow)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg = TrainConfig()
    phi_dot_scale = cfg.phi_dot_scale
    traj_dir = args.trajectory_dir

    # held-out trajectories for this split
    split_paths = sorted(glob.glob(os.path.join(traj_dir, "trajs", args.split, "*.npz")))
    assert split_paths, f"no trajectories in {os.path.join(traj_dir, 'trajs', args.split)}"
    split_trajs = []
    for p in split_paths:
        with np.load(p) as f:
            split_trajs.append({k: np.asarray(f[k], dtype=np.float64) for k in f.files})
    if args.n_eps is not None:
        split_trajs = split_trajs[: args.n_eps]
    print(f"evaluating on {len(split_trajs)} '{args.split}' trajectories")

    env = AviatorManifoldEnv(
        manifold_dir=args.manifold_dir,
        trajectory_dir=None,
        split=args.split,
        phi_dot_scale=phi_dot_scale,
        trajectories=split_trajs,
    )

    results = {}

    # -- baselines -----------------------------------------------------------
    for name, fn in [
        ("static", make_static_policy()),
        ("greedy", make_greedy_policy()),
    ]:
        results[name] = summarize(roll_out(env, split_trajs, fn))
        print(f"[{name}] done: collision={results[name]['collision_rate']:.3f} "
              f"max|qdot|={results[name]['max_qdot']:.3f}")

    # -- BC policy -----------------------------------------------------------
    rng = jax.random.PRNGKey(args.seed)
    # the agent's StateEncoder expects the chunked (1, 40) leading dim, matching
    # the demo pkls / ChunkingWrapper(obs_horizon=1); add it to the raw env sample
    sample_obs = {"state": np.asarray(env.observation_space.sample()["state"])[None, :]}
    bc_agent = cfg.make_bc_agent(
        seed=args.seed,
        sample_obs=sample_obs,
        sample_action=env.action_space.sample(),
    )
    bc_agent = bc_agent.replace(
        state=checkpoints.restore_checkpoint(os.path.abspath(args.bc_checkpoint), bc_agent.state)
    )
    results["bc"] = summarize(roll_out(env, split_trajs, make_bc_policy(bc_agent)))
    print(f"[bc] done: collision={results['bc']['collision_rate']:.3f} "
          f"max|qdot|={results['bc']['max_qdot']:.3f}")

    print()
    print_table(results)

    # -- action variance diagnostic -----------------------------------------
    print("\naction variance (Var[a_DP | obs] over K-NN demo neighbours):")
    print(json.dumps(action_variance_diagnostic(args.demo_dir, seed=args.seed), indent=2))

    # -- oracle regret (J_BC - J_DP on dp_train) -----------------------------
    if not args.skip_oracle:
        print("\noracle regret (J_BC - J_DP on dp_train):")
        compute_oracle_regret(cfg, env, bc_agent, traj_dir, args.demo_dir)


def compute_oracle_regret(cfg, env, bc_agent, traj_dir, demo_dir):
    """J_BC - J_DP on the dp_train split (the split with DP teacher solutions).

    J_DP is the DP teacher's phi_dot* replayed through the same env (so the
    safety filter and reward_fn are identical to BC's rollout); J_BC is the BC
    policy's reward sum on the same trajectories.
    """
    dp_csvs = sorted(glob.glob(os.path.join(demo_dir, "traj_*.csv")))
    dp_npzs = sorted(glob.glob(os.path.join(traj_dir, "trajs", "dp_train", "*.npz")))
    # pair them by index; a dp_train traj with no CSV is skipped
    idx_by_csv = {int(os.path.basename(c).split("_")[1].split(".")[0]): c for c in dp_csvs}

    regrets = []
    n = 0
    for p in dp_npzs:
        idx = int(os.path.basename(p).split("_")[1].split(".")[0])
        if idx not in idx_by_csv:
            continue
        csv = np.loadtxt(idx_by_csv[idx], delimiter=",", skiprows=2)
        with np.load(p) as f:
            traj = {k: np.asarray(f[k], dtype=np.float64) for k in f.files}
        if csv.shape[0] != traj["x"].shape[0]:
            continue
        phi_dot = csv[:, [_CSV_COL_PHI_DOT_L, _CSV_COL_PHI_DOT_R]]

        env._trajectories = [traj]
        j_dp = _rollout_reward(env, make_dp_oracle_policy(phi_dot, cfg.phi_dot_scale))
        j_bc = _rollout_reward(env, make_bc_policy(bc_agent))
        regrets.append(j_bc - j_dp)
        n += 1

    regrets = np.array(regrets)
    print(f"  n={n}")
    print(f"  regret mean={regrets.mean():.3f} median={np.median(regrets):.3f} "
          f"p10={np.percentile(regrets, 10):.3f} p90={np.percentile(regrets, 90):.3f}")


def _rollout_reward(env, policy_fn):
    obs, _ = env.reset()
    done = False
    J = 0.0
    while not done:
        a = policy_fn(obs, env)
        obs, reward, terminated, truncated, _ = env.step(a)
        J += float(reward)
        done = terminated or truncated
    return J


if __name__ == "__main__":
    main()

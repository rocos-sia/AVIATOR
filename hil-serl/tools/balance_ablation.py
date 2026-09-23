"""Hold-balance BC ablation (2026-09-23 reward-calibration follow-up).

Doc: the reward-calibration phase found BC fails the no-intervention gate
(90% completion but 86% intervention, 0% zero-intervention episodes).  The
root cause has two candidate contributions that must be *disentangled*:

    BC failure = demo-action-distribution bias (88% holds filtered out)
               + teacher-objective mismatch    (R_reserve ~= 0: demo rides the
                                                safe boundary, not the centre)

This script holds everything fixed -- same DP demo pkls, same MLP(256,256,256)
BC agent, same 20k steps, same val split -- and varies ONLY how hold
transitions are sampled:

    b0  filter_zero_actions=True            (current baseline; all holds dropped)
    b1  filter_zero_actions=False           (restore all 88% holds)
    b2  hold/move balanced to ~50/50        (recommended main experiment)
    b3  b2 + oversample move<->hold boundary neighbourhood

Each mode is run with 3 seeds.  For each (mode, seed) it trains BC and reports
the *rich* metric set the gate needs (not just comp% / mean intervention):

    completion rate, zero-intervention episode rate, intervention step ratio,
    max consecutive intervention length, R_reserve median/P10,
    mean |phi_dot_nom - phi_dot_safe|, P(|a_raw| < eps), and I_sign.

Run from hil-serl/::

    python -m tools.balance_ablation --modes b0,b1,b2,b3 --seeds 0,1,2
"""

from __future__ import annotations

import argparse
import glob
import os
import pickle

import jax
import jax.numpy as jnp
import numpy as np

from examples.experiments.aviator_manifold.config import TrainConfig
from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

_DEMO_DIR = "data/aviator/dp_demo/"
_TRAJ_DIR = "data/aviator/trajectory_source/"
_MANIFOLD_DIR = "data/aviator/manifold_phi/"

# I_sign / P(|a_raw|<eps) thresholds (normalized action space, a in [-1, 1])
_A_CENTER_EPS = 0.10   # |a_center| above this => "center-restore clearly non-zero"
_A_RAW_EPS = 0.05      # |a_raw| below this => "policy holds"


# ---------------------------------------------------------------------------
# demo loading + sampling
# ---------------------------------------------------------------------------
def load_demo_trajs(demo_dir: str) -> list:
    """Per-trajectory demo transitions (each pkl = one trajectory, in order)."""
    trajs = []
    for p in sorted(glob.glob(os.path.join(demo_dir, "traj_*.pkl"))):
        with open(p, "rb") as f:
            trajs.append(pickle.load(f))
    return trajs


def _transition_neighbourhood(actions: np.ndarray) -> np.ndarray:
    """Boolean mask over a trajectory marking move<->hold boundary frames.

    An "event" is any step whose action differs from the previous step (the
    quantized {0, 1/3, 2/3} demo action changes).  The neighbourhood is the
    event frame plus its immediate predecessor and successor -- the "brake" and
    "start" frames around each reposition.
    """
    n = len(actions)
    near = np.zeros(n, dtype=bool)
    for t in range(1, n):
        if not np.allclose(actions[t], actions[t - 1]):
            for d in (-1, 0, 1):
                if 0 <= t + d < n:
                    near[t + d] = True
    return near


def sample_transitions(trajs, mode: str, move_frac: float = 0.5,
                       oversample_near: int = 2, seed: int = 0) -> list:
    """Return the flattened transition list for one sampling variant."""
    rng = np.random.default_rng(seed)
    out = []
    for traj in trajs:
        if mode == "b1":
            out.extend(traj)
            continue

        holds = [t for t in traj if np.linalg.norm(np.asarray(t["actions"])) <= 0.0]
        moves = [t for t in traj if np.linalg.norm(np.asarray(t["actions"])) > 0.0]

        if mode == "b3":
            actions = np.stack([np.asarray(t["actions"]) for t in traj])
            near = _transition_neighbourhood(actions)
            extra = [t for t, n in zip(traj, near) if n]
            # duplicate the boundary neighbourhood (oversample_near - 1) extra times
            for _ in range(max(oversample_near - 1, 0)):
                for t in extra:
                    (holds if np.linalg.norm(np.asarray(t["actions"])) <= 0.0
                     else moves).append(t)

        # balance: keep all moves, subsample holds to hit `move_frac`
        n_moves, n_holds = len(moves), len(holds)
        if n_moves > 0 and n_holds > 0 and move_frac > 0:
            k = int(round(n_moves * (1.0 - move_frac) / move_frac))
            k = int(min(max(k, 0), n_holds))
            idx = rng.choice(n_holds, size=k, replace=False)
            kept = [holds[i] for i in idx]
        else:
            kept = holds
        out.extend(moves)
        out.extend(kept)
    return out


# ---------------------------------------------------------------------------
# training
# ---------------------------------------------------------------------------
def train_bc(cfg, transitions, seed: int, train_steps: int = 20000,
             batch_size: int = 256):
    """Mirror train_bc.py's learner loop (single device, no wandb)."""
    sample_obs = {"state": np.zeros((1, 40), dtype=np.float32)}
    sample_action = np.zeros((2,), dtype=np.float32)
    bc_agent = cfg.make_bc_agent(seed=seed, sample_obs=sample_obs,
                                 sample_action=sample_action)

    devices = jax.local_devices()
    sharding = jax.sharding.PositionalSharding(devices)
    bc_agent = jax.device_put(jax.tree_map(jnp.array, bc_agent),
                              sharding.replicate())

    # observation/action spaces inferred from the demo transition shape
    import gymnasium as gym
    obs_space = gym.spaces.Dict({"state": gym.spaces.Box(
        -np.inf, np.inf, transitions[0]["observations"]["state"].shape,
        dtype=np.float32)})
    act_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
    rb = cfg.make_replay_buffer(obs_space, act_space, cfg.replay_buffer_capacity)
    for t in transitions:
        rb.insert(t)
    it = rb.get_iterator(sample_args={"batch_size": batch_size},
                         device=sharding.replicate())

    for _ in range(train_steps):
        batch = next(it)
        bc_agent, _ = bc_agent.update(batch)

    # de-replicate for in-process deterministic eval
    bc_eval = cfg.make_bc_agent(seed=0, sample_obs=sample_obs,
                                sample_action=sample_action)
    bc_eval = bc_eval.replace(state=jax.device_get(bc_agent.state))
    return bc_eval


# ---------------------------------------------------------------------------
# evaluation
# ---------------------------------------------------------------------------
def _center_action(env) -> np.ndarray:
    """Greedy centre-restore action (normalized), matching evaluate_policy."""
    x_next = env._traj["x"][env._t + 1]
    box = env.lookup.safe_interval(x_next[None, :])
    center = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
    phi_dot = (center - env._phi) / env.dt
    return np.clip(phi_dot / env.phi_dot_scale, -1.0, 1.0)


def roll_out_rich(env, trajs, policy_fn):
    episodes = []
    for traj in trajs:
        env._trajectories = [traj]
        obs, _ = env.reset()

        steps = 0
        interv_steps = 0
        cur_consec = 0
        max_consec = 0
        r_res = []
        phi_diff = []
        a_raw_abs = []
        isign = []  # list of (mismatch, 1) per arm-step where |a_center|>eps
        done = False
        termination = "end"

        while not done:
            a = policy_fn(obs, env)
            a_center = _center_action(env)
            obs, _r, terminated, truncated, info = env.step(a)
            steps += 1

            iv = int(bool(info["filter"].get("intervened")))
            interv_steps += iv
            if iv:
                cur_consec += 1
                max_consec = max(max_consec, cur_consec)
            else:
                cur_consec = 0

            if "R_reserve" in info:
                r_res.append(float(info["R_reserve"]))
            if "phi_dot_nom" in info and "phi_dot_safe" in info:
                nom = np.asarray(info["phi_dot_nom"])
                safe = np.asarray(info["phi_dot_safe"])
                phi_diff.append(np.abs(nom - safe))
                a_raw_abs.append(np.abs(nom / env.phi_dot_scale))
            for k in range(2):
                if abs(a_center[k]) > _A_CENTER_EPS:
                    isign.append((int(np.sign(a[k]) != np.sign(a_center[k])), 1))

            done = terminated or truncated
            if terminated:
                termination = ("infeasible" if not info["filter"]["feasible"]
                               else info.get("termination", "terminated"))

        episodes.append({
            "termination": termination,
            "steps": steps,
            "interv_steps": interv_steps,
            "max_consec": max_consec,
            "r_res": np.asarray(r_res),
            "phi_diff": np.concatenate(phi_diff) if phi_diff else np.zeros(0),
            "a_raw_abs": np.concatenate(a_raw_abs) if a_raw_abs else np.zeros(0),
            "isign": np.asarray(isign).reshape(-1, 2) if isign else np.zeros((0, 2)),
        })
    return episodes


def summarize_rich(episodes) -> dict:
    n = len(episodes)
    term = np.array([e["termination"] for e in episodes])
    total_steps = sum(e["steps"] for e in episodes)
    interv_steps = sum(e["interv_steps"] for e in episodes)
    r = np.concatenate([e["r_res"] for e in episodes if len(e["r_res"])])
    phi_diff = np.concatenate([e["phi_diff"] for e in episodes if len(e["phi_diff"])])
    a_raw = np.concatenate([e["a_raw_abs"] for e in episodes if len(e["a_raw_abs"])])
    isign = np.concatenate([e["isign"] for e in episodes if len(e["isign"])])

    return {
        "completion_rate": float((term == "end").mean()),
        "infeasible_rate": float((term == "infeasible").mean()),
        "zero_interv_ep_rate": float(sum(1 for e in episodes if e["interv_steps"] == 0) / n),
        "interv_step_ratio": float(interv_steps / total_steps if total_steps else 0.0),
        "max_consec_interv": float(max(e["max_consec"] for e in episodes)),
        "mean_consec_interv": float(np.mean([e["max_consec"] for e in episodes])),
        "r_reserve_median": float(np.median(r)) if len(r) else np.nan,
        "r_reserve_p10": float(np.percentile(r, 10)) if len(r) else np.nan,
        "mean_phi_diff": float(np.mean(phi_diff)) if len(phi_diff) else np.nan,
        "p_a_raw_small": float((a_raw < _A_RAW_EPS).mean()) if len(a_raw) else np.nan,
        "I_sign": float(isign[:, 0].mean()) if len(isign) else np.nan,
        "n_isign": int(len(isign)),
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modes", default="b0,b1,b2,b3")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--train-steps", type=int, default=20000)
    ap.add_argument("--move-frac", type=float, default=0.5,
                    help="target move fraction for balanced modes b2/b3")
    ap.add_argument("--demo-dir", default=_DEMO_DIR)
    ap.add_argument("--traj-dir", default=_TRAJ_DIR)
    ap.add_argument("--manifold-dir", default=_MANIFOLD_DIR)
    ap.add_argument("--split", default="val")
    args = ap.parse_args()

    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    seeds = [int(s) for s in args.seeds.split(",")]

    cfg = TrainConfig()
    trajs = load_demo_trajs(args.demo_dir)

    split_paths = sorted(glob.glob(os.path.join(
        args.traj_dir, "trajs", args.split, "*.npz")))
    assert split_paths, f"no trajectories in {args.traj_dir}/trajs/{args.split}"
    val_trajs = []
    for p in split_paths:
        with np.load(p) as f:
            val_trajs.append({k: np.asarray(f[k], dtype=np.float64) for k in f.files})
    env = AviatorManifoldEnv(manifold_dir=args.manifold_dir, trajectory_dir=None,
                             split=args.split, phi_dot_scale=cfg.phi_dot_scale,
                             trajectories=val_trajs)

    all_rows = []
    for mode in modes:
        for seed in seeds:
            if mode == "b0":
                # current config: drop all holds (filter_zero_actions=True)
                trans = [t for tr in trajs for t in tr
                         if np.linalg.norm(np.asarray(t["actions"])) > 0.0]
            else:
                trans = sample_transitions(trajs, mode, move_frac=args.move_frac,
                                           seed=seed)
            n_hold = sum(1 for t in trans if np.linalg.norm(np.asarray(t["actions"])) <= 0.0)
            n_move = len(trans) - n_hold
            print(f"[{mode} seed={seed}] {len(trans)} transitions "
                  f"(move {n_move}, hold {n_hold}, move_frac {n_move/max(len(trans),1):.2f})",
                  flush=True)

            bc = train_bc(cfg, trans, seed=seed, train_steps=args.train_steps)

            def policy(obs, env, _bc=bc):
                o = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
                return np.asarray(_bc.sample_actions(observations=o, argmax=True))

            eps = roll_out_rich(env, val_trajs, policy)
            s = summarize_rich(eps)
            s["mode"] = mode
            s["seed"] = seed
            all_rows.append(s)
            print(f"    comp={s['completion_rate']*100:5.1f}%  "
                  f"zeroInterv={s['zero_interv_ep_rate']*100:5.1f}%  "
                  f"intervStep={s['interv_step_ratio']*100:5.1f}%  "
                  f"maxConsec={s['max_consec_interv']:.0f}  "
                  f"RresMed={s['r_reserve_median']:.3f}  "
                  f"RresP10={s['r_reserve_p10']:.3f}  "
                  f"|dnom-dsafe|={s['mean_phi_diff']:.3f}  "
                  f"P(|a|<eps)={s['p_a_raw_small']:.3f}  "
                  f"I_sign={s['I_sign']:.3f}", flush=True)

    # ---- aggregate across seeds ----
    print("\n" + "=" * 120)
    hdr = (f"{'mode':<4} {'comp%':>7} {'zeroInt%':>8} {'intStep%':>8} {'maxCons':>7} "
           f"{'RresMed':>8} {'RresP10':>8} {'|dn-ds|':>8} {'P|a|<eps':>9} {'I_sign':>7}")
    print(hdr)
    print("-" * 120)
    for mode in modes:
        rows = [r for r in all_rows if r["mode"] == mode]
        comp = [r["completion_rate"] for r in rows]
        zi = [r["zero_interv_ep_rate"] for r in rows]
        ist = [r["interv_step_ratio"] for r in rows]
        mc = [r["max_consec_interv"] for r in rows]
        rr = [r["r_reserve_median"] for r in rows]
        rp = [r["r_reserve_p10"] for r in rows]
        pd_ = [r["mean_phi_diff"] for r in rows]
        pa = [r["p_a_raw_small"] for r in rows]
        ig = [r["I_sign"] for r in rows]

        def f(x):
            return f"{np.mean(x)*100:6.1f}±{np.std(x)*100:.0f}"

        def g(x):
            return f"{np.mean(x):6.3f}±{np.std(x):.2f}"

        print(f"{mode:<4} {f(comp)} {f(zi)} {f(ist)} "
              f"{np.mean(mc):6.0f}±{np.std(mc):.0f} "
              f"{g(rr)} {g(rp)} {g(pd_)} {f(pa)} {g(ig)}")


if __name__ == "__main__":
    main()

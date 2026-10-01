"""Export physically checked fixed-field phase transitions for RLPD replay.

Each demonstration starts at its own task state and phase. No transition is
stitched across a rejected step. The teacher follows the frozen phase field
only where the actual Q(x, phi) rollout obeys phase, joint speed/acceleration,
joint position, branch, MuJoCo contact and wall-clearance limits.
"""
from __future__ import annotations

import argparse
import glob
import json
import pickle
import subprocess
import tempfile
from collections import Counter
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from tools.audit_static_field import interpolate_q


def valid_runs(x: np.ndarray, phi: np.ndarray, q: np.ndarray,
               state_ok: np.ndarray, dt: float, min_steps: int):
    phase_rate = np.max(np.abs(np.diff(phi, axis=0) / dt), axis=1)
    speed = np.max(np.abs(np.diff(q, axis=0) / dt), axis=1)
    accel = np.max(np.abs(np.diff(q, n=2, axis=0) / dt**2), axis=1)
    edge_ok = (state_ok[:-1] & state_ok[1:] &
               (phase_rate <= 1.5 + 1e-9) & (speed <= 1.5 + 1e-9))
    # A segment beginning at index k has no prior velocity; acceleration is
    # checked from its second action onward, as in the evaluation audit.
    start = None
    for i, good in enumerate(edge_ok):
        if not good:
            if start is not None and i - start >= min_steps:
                yield start, i
            start = None
            continue
        if start is None:
            start = i
        if i > start and accel[i - 1] > 10.0 + 1e-8:
            if i - start >= min_steps:
                yield start, i
            start = i
    if start is not None and len(edge_ok) - start >= min_steps:
        yield start, len(edge_ok)


def build(args) -> dict:
    paths = sorted(glob.glob(f"{args.trajectory_dir}/trajs/{args.split}/*.npz"))
    if args.limit is not None:
        paths = paths[:args.limit]
    if not paths:
        raise ValueError("no source trajectories")
    env = AviatorManifoldEnv(args.manifold_dir, trajectories=[{"x": np.zeros((2, 2)),
                                                                "xdot": np.zeros((2, 2))}],
                             phi_dot_scale=1.5, qddot_max=10.0)
    lookup = env.lookup
    with np.load(args.field) as field:
        if not (np.array_equal(field["theta"], lookup.theta_axis) and
                np.array_equal(field["s"], lookup.s_axis)):
            raise ValueError("field axes do not match manifold")
        phi_grid = field["phi"]
    proposed = []
    proposed_per_cell = Counter()
    candidate_segments = 0
    for path in paths:
        with np.load(path) as file:
            data = {key: file[key] for key in file.files}
        x = data["x"]
        cell = (round(float(np.max(np.abs(data["xdot"][:, 0]))), 1),
                int(np.count_nonzero(np.signbit(data["xdot"][1:, 0]) !=
                                     np.signbit(data["xdot"][:-1, 0]))))
        if args.max_per_cell is not None and proposed_per_cell[cell] >= args.max_per_cell:
            continue
        phi = interpolate_q(lookup, phi_grid, x)
        try:
            result = lookup.query(x, phi, check_safe=False)
        except ValueError:
            continue
        q = np.concatenate([result["qL"], result["qR"]], axis=1)
        lo, hi = lookup.joint_lower.reshape(14), lookup.joint_upper.reshape(14)
        state_ok = (np.all(np.isfinite(q), axis=1) & (result["branch"] < 2) &
                    (result["d_min"] >= 0.005) &
                    np.all((q >= lo) & (q <= hi), axis=1))
        for start, stop in valid_runs(x, phi, q, state_ok, env.dt, args.min_steps):
            candidate_segments += 1
            if args.max_segments is not None and len(proposed) >= args.max_segments:
                break
            if args.max_per_cell is not None and proposed_per_cell[cell] >= args.max_per_cell:
                break
            xs, qs = x[start:stop + 1], q[start:stop + 1]
            segment = {key: value[start:stop + 1] for key, value in data.items()
                       if len(value) == len(x)}
            env._trajectories = [segment]
            obs, _ = env.reset(options={"trajectory_index": 0, "initial_phi": phi[start]})
            episode = []
            for t in range(start, stop):
                action = (phi[t + 1] - phi[t]) / (env.dt * env.phi_dot_scale)
                if np.any(np.abs(action) > 1 + 1e-8):
                    break
                action = np.clip(action, -1, 1).astype(np.float32)
                next_obs, reward, terminated, truncated, info = env.step(action)
                if terminated:
                    break
                done = bool(truncated)
                episode.append({"observations": {"state": obs["state"][None]},
                                "actions": action,
                                "next_observations": {"state": next_obs["state"][None]},
                                "rewards": np.float32(reward),
                                "masks": np.float32(not done), "dones": done})
                obs = next_obs
            if len(episode) == stop - start and episode[-1]["dones"]:
                proposed.append((episode, xs, qs, {"trajectory": Path(path).stem,
                                                   "start": start, "stop": stop,
                                                   "steps": len(episode),
                                                   "speed_bin": cell[0],
                                                   "reversals": cell[1]}))
                proposed_per_cell[cell] += 1
        if args.max_segments is not None and len(proposed) >= args.max_segments:
            break
    if not proposed:
        raise ValueError("no phase-field segments passed the action and dynamics gates")
    with tempfile.TemporaryDirectory(prefix="aviator_prior_") as temp:
        directory = Path(temp)
        for index, (_, xs, qs, _) in enumerate(proposed):
            np.savez_compressed(directory / f"segment_{index:05d}.npz", x=xs, q=qs)
        accepted_file = directory / "accepted.json"
        subprocess.run([str(args.collision_python), "-m", "tools.audit_static_prior_candidates",
                        "--candidates", str(directory), "--model", args.model,
                        "--output", str(accepted_file)], check=True)
        accepted = {int(name.split("_")[-1]) for name in json.loads(accepted_file.read_text())}
    transitions = [item for index, (episode, _, _, _) in enumerate(proposed)
                   if index in accepted for item in episode]
    episodes = [meta for index, (_, _, _, meta) in enumerate(proposed) if index in accepted]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as handle:
        pickle.dump(transitions, handle, protocol=pickle.HIGHEST_PROTOCOL)
    report = {"source_paths": len(paths), "candidate_segments": candidate_segments,
              "accepted_segments": len(episodes), "transitions": len(transitions),
              "output": str(args.output), "episodes": episodes}
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    ap.add_argument("--collision-python", type=Path,
                    default=Path("/home/rocos/miniconda3/envs/mujo/bin/python"))
    ap.add_argument("--field", type=Path, required=True)
    ap.add_argument("--trajectory-dir", required=True)
    ap.add_argument("--split", default="rl_train")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--min-steps", type=int, default=25)
    ap.add_argument("--max-segments", type=int)
    ap.add_argument("--max-per-cell", type=int, default=25)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    report = build(args)
    print(json.dumps({key: value for key, value in report.items() if key != "episodes"}))


if __name__ == "__main__":
    main()

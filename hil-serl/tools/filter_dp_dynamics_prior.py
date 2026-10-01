"""Keep only whole DP demonstrations passing the new hard dynamics and MuJoCo gates."""
from __future__ import annotations

import argparse
import glob
import json
import pickle
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--demo-dir", type=Path, default=Path("data/aviator/dp_demo"))
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    ap.add_argument("--collision-python", default="/home/rocos/miniconda3/envs/mujo/bin/python")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    lookup = ManifoldLookup(args.manifold_dir)
    proposed = []
    reasons = {"grid": 0, "lookup": 0, "speed": 0, "acceleration": 0}
    paths = sorted(glob.glob(str(args.demo_dir / "traj_*.pkl")))
    with tempfile.TemporaryDirectory(prefix="aviator_dp_prior_") as temp:
        directory = Path(temp)
        for path in paths:
            with open(path, "rb") as handle:
                transitions = pickle.load(handle)
            obs = np.vstack([transitions[0]["observations"]["state"][0]] +
                            [row["next_observations"]["state"][0] for row in transitions])
            x = obs[:, :2]
            phi = np.stack([np.arctan2(obs[:, 4], obs[:, 5]),
                            np.arctan2(obs[:, 6], obs[:, 7])], axis=1)
            try:
                result = lookup.query(x, phi, check_safe=False)
            except ValueError:
                reasons["grid"] += 1
                continue
            q = np.concatenate([result["qL"], result["qR"]], axis=1)
            lo, hi = lookup.joint_lower.reshape(14), lookup.joint_upper.reshape(14)
            if (np.any(result["branch"] >= 2) or np.min(result["d_min"]) < 0.005 or
                    not np.all(np.isfinite(q)) or np.any(q < lo) or np.any(q > hi)):
                reasons["lookup"] += 1
                continue
            if np.max(np.abs(np.diff(q, axis=0) / 0.01)) > 1.5 + 1e-8:
                reasons["speed"] += 1
                continue
            if np.max(np.abs(np.diff(q, n=2, axis=0) / 0.01**2)) > 10 + 1e-8:
                reasons["acceleration"] += 1
                continue
            index = len(proposed)
            np.savez_compressed(directory / f"segment_{index:05d}.npz", x=x, q=q)
            proposed.append((Path(path).name, transitions))
        accepted_file = directory / "accepted.json"
        subprocess.run([args.collision_python, "-m", "tools.audit_static_prior_candidates",
                        "--candidates", str(directory), "--model", args.model,
                        "--output", str(accepted_file)], check=True)
        accepted = {int(name.split("_")[-1]) for name in json.loads(accepted_file.read_text())}
    kept = [item for index, (_, rows) in enumerate(proposed) if index in accepted
            for item in rows]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as handle:
        pickle.dump(kept, handle, protocol=pickle.HIGHEST_PROTOCOL)
    report = {"source_episodes": len(paths), "dynamics_candidates": len(proposed),
              "accepted_episodes": len(accepted), "transitions": len(kept),
              "reasons": reasons, "output": str(args.output)}
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()

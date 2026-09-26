"""Replay registered LUT phase trajectories with freshly derived actions.

This reads registration diagnostics only. It never writes demonstration PKLs or
starts training, and reports the first real environment termination per path.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pandas as pd

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv


def audit(args):
    with open(Path(args.demo_dir) / "phi_dot_scale.json") as file:
        scale = float(json.load(file)["phi_dot_scale"])
    paths = sorted(Path(p) for p in glob.glob(str(Path(args.registration_dir) / "traj_*.npz")))
    env = AviatorManifoldEnv(args.manifold_dir, trajectories=[], phi_dot_scale=scale)
    rows = []
    for path in paths:
        source = Path(args.trajectory_dir) / "trajs" / "dp_train" / path.name
        if not source.is_file():
            rows.append({"traj_id": path.stem, "status": "missing_source"})
            continue
        with np.load(path) as file:
            phase = file["phi" if args.mode == "feasible" else "phi_representation"]
            q_error = file["q_error" if args.mode == "feasible" else "q_error_representation"]
            times = file["t"]
        if not np.isfinite(phase).all():
            rows.append({"traj_id": path.stem, "status": "no_complete_registration"})
            continue
        with np.load(source) as file:
            env._trajectories = [{k: np.asarray(file[k], dtype=np.float64) for k in file.files}]
        env.reset(options={"initial_phi": phase[0]})
        action = np.diff(phase, axis=0) / np.diff(times)[:, None] / scale
        if np.max(np.abs(action)) > 1 + 1e-9:
            rows.append({"traj_id": path.stem, "status": "action_out_of_range",
                         "max_abs_action": float(np.max(np.abs(action)))})
            continue
        for step, a in enumerate(action, 1):
            _, _, terminated, truncated, info = env.step(a)
            if terminated or truncated:
                rows.append({"traj_id": path.stem, "status": info["termination"],
                             "stop_step": step, "fraction": step / len(action),
                             "max_abs_action": float(np.max(np.abs(action))),
                             "max_q_error_rad": float(np.max(q_error)),
                             "d_at_stop_m": float(info.get("d_min", np.nan)),
                             "max_qdot_at_stop_rad_s": float(info.get("max_qdot", np.nan))})
                break
        else:
            rows.append({"traj_id": path.stem, "status": "missing_terminal"})
    table = pd.DataFrame(rows)
    outdir = Path(args.registration_dir)
    table.to_csv(outdir / f"replay_{args.mode}.csv", index=False)
    summary = {"mode": args.mode, "trajectories": len(paths),
               "status": table.status.value_counts().to_dict(),
               "completion_rate": float((table.status == "end").mean()),
               "median_fraction_replayed": float(table.fraction.median())}
    (outdir / f"replay_{args.mode}_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registration-dir", default="data/aviator/phase_registration")
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    parser.add_argument("--demo-dir", default="data/aviator/dp_demo")
    parser.add_argument("--mode", choices=("feasible", "representation"), default="feasible")
    args = parser.parse_args()
    print(json.dumps(audit(args), indent=2))


if __name__ == "__main__":
    main()

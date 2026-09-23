"""Reject an RL rollout split whose reset states are already infeasible.

The online environment currently chooses phi=0 projected into the LUT safe
interval at reset. This audit evaluates those exact reset configurations with
the same C++ geometry and task code used by Route A DP before training starts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from tools.audit_prior_states import physical_evaluate


def audit(args):
    env = AviatorManifoldEnv(args.manifold_dir, trajectory_dir=args.trajectory_dir,
                             split=args.split, phi_dot_scale=1.5)
    paths = env._traj_paths
    if len(paths) != args.expected_count:
        raise ValueError(f"expected {args.expected_count} online paths, found {len(paths)}")
    rows = []
    geometry_input = []
    for index, path in enumerate(paths):
        env.reset(options={"trajectory_index": index})
        result = env.lookup.query(env._x[None], env._phi[None], check_safe=False)
        q = np.r_[result["qL"][0], result["qR"][0]]
        geometry_input.append(np.r_[env._x, q])
        rows.append({"index": index, "trajectory": Path(path).stem,
                     "theta": env._x[0], "s": env._x[1],
                     "phi_L": env._phi[0], "phi_R": env._phi[1],
                     "d_lookup_m": float(result["d_min"][0]),
                     "branch_exclusion": int(result["branch"][0])})
    physical = physical_evaluate(np.asarray(geometry_input), args.binary,
                                 args.config, args.output_dir)
    table = pd.concat([pd.DataFrame(rows), physical.add_prefix("physical_")], axis=1)
    table["valid_start"] = ((table.physical_d_min >= args.d_safe)
                            & (table.physical_task_error <= args.task_tolerance)
                            & (table.physical_joint_margin >= 0)
                            & (table.physical_collision == 0)
                            & (table.branch_exclusion < 2))
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    table.to_csv(outdir / "online_starts.csv", index=False)
    bad = table[~table.valid_start]
    summary = {"online_paths": len(paths), "valid_starts": int(table.valid_start.sum()),
               "invalid_starts": bad.trajectory.tolist(),
               "below_clearance": int((table.physical_d_min < args.d_safe).sum()),
               "task_error_exceeds_tolerance": int(
                   (table.physical_task_error > args.task_tolerance).sum()),
               "contacts": int((table.physical_collision != 0).sum()),
               "output": str(outdir / "online_starts.csv")}
    (outdir / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    parser.add_argument("--split", default="rl_train")
    parser.add_argument("--expected-count", type=int, default=400)
    parser.add_argument("--output-dir", default="data/aviator/online_start_audit")
    parser.add_argument("--binary", default="../AviatorRobot/build/bin/aviator_clearance_trajectory")
    parser.add_argument("--config", default="../AviatorRobot/config/aviator.yaml")
    parser.add_argument("--d-safe", type=float, default=0.005)
    parser.add_argument("--task-tolerance", type=float, default=0.002)
    args = parser.parse_args()
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    summary = audit(args)
    print(json.dumps(summary, indent=2))
    if summary["valid_starts"] != args.expected_count:
        raise SystemExit("online split contains infeasible reset states; refusing RLPD training")


if __name__ == "__main__":
    main()

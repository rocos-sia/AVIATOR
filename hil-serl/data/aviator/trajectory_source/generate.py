#!/usr/bin/env python3
"""Aviator v0.1 -- unified trajectory source (Task 0, pre-requisite, BLOCKER #7).

Doc: ``docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md``

Generates the single trajectory source consumed by *everything* downstream:

* the C++ full-horizon DP teacher (Task 0.3) reads these ``.npz`` files,
* the Python Gym env (Task 1.3) reuses the same generator for ``reset()``.

No other generator is used anywhere in the pipeline.

Output layout::

    hil-serl/data/aviator/trajectory_source/
        generate.py
        split_manifest.json
        trajs/
            dp_train/traj_0000.npz   (100)   BC / DP demonstrations
            rl_train/traj_0000.npz   (400)   RLPD rollouts
            val/traj_0000.npz        (50)
            test/traj_0000.npz       (100)

Every ``.npz`` holds ``t, x, xdot, xddot`` (see
``examples.experiments.aviator_manifold.trajectory_generator``).
``path`` entries in ``split_manifest.json`` are relative to the directory that
contains the manifest.

Usage::

    python data/aviator/trajectory_source/generate.py --seed 0
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

# the generator lives in the example package, next to the HIL-SERL examples
_HIL_SERL_ROOT = Path(__file__).resolve().parents[3]
if str(_HIL_SERL_ROOT) not in sys.path:
    sys.path.insert(0, str(_HIL_SERL_ROOT))

from examples.experiments.aviator_manifold.trajectory_generator import (  # noqa: E402
    A_MAX,
    V_MAX,
    generate_task_trajectory,
)

TRAJ_DIR_NAME = "trajs"
MANIFEST_NAME = "split_manifest.json"

#: per-split filename template and the number of trajectories
_SPLITS = ("dp_train", "rl_train", "val", "test")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate_split(
    rng,
    seed: int,
    n_dp: int = 100,
    n_rl: int = 400,
    n_val: int = 50,
    n_test: int = 100,
    T_min: float = 8.0,
    T_max: float = 12.0,
    dt: float = 0.01,
    out_dir: str = None,
) -> dict:
    """Writes trajectory files and returns ``split_manifest``.

    ``rng`` (a ``numpy.random.Generator``) fixes the assignment of trajectories
    to file indices; ``seed`` derives one independent stream per trajectory
    (``seed * 1_000_003 + global_index``), so a trajectory's content never
    depends on how many trajectories are generated around it.

    ``out_dir`` defaults to the directory holding this file.
    """
    root = Path(out_dir) if out_dir is not None else Path(__file__).resolve().parent
    traj_root = root / TRAJ_DIR_NAME
    for split in _SPLITS:
        (traj_root / split).mkdir(parents=True, exist_ok=True)

    counts = {"dp_train": n_dp, "rl_train": n_rl, "val": n_val, "test": n_test}
    rows = []
    offset = 0
    for split in _SPLITS:
        n = counts[split]
        order = rng.permutation(n)  # which stream goes to which file index
        for index in range(n):
            traj_seed = int(seed) * 1_000_003 + offset + int(order[index])
            traj_rng = np.random.default_rng(traj_seed)
            T = float(traj_rng.uniform(T_min, T_max))

            traj = generate_task_trajectory(traj_rng, T, dt=dt)

            max_v = np.max(np.abs(traj["xdot"]), axis=0)
            max_a = np.max(np.abs(traj["xddot"]), axis=0)
            assert np.all(max_v <= np.asarray(V_MAX) * (1.0 + 1e-9)), (split, index, max_v)
            assert np.all(max_a <= np.asarray(A_MAX) * (1.0 + 1e-9)), (split, index, max_a)

            rel_path = f"{TRAJ_DIR_NAME}/{split}/traj_{index:04d}.npz"
            abs_path = root / rel_path
            np.savez(
                abs_path,
                t=traj["t"],
                x=traj["x"],
                xdot=traj["xdot"],
                xddot=traj["xddot"],
            )

            rows.append(
                {
                    "split": split,
                    "index": index,
                    "seed": traj_seed,
                    "T": T,
                    "dt": float(dt),
                    "path": rel_path,
                    "max_abs_xdot": [float(v) for v in max_v],
                    "max_abs_xddot": [float(v) for v in max_a],
                    "n_samples": int(traj["t"].shape[0]),
                    "sha256": _sha256(abs_path),
                }
            )
        offset += n

    manifest = {
        "seed": int(seed),
        "dt": float(dt),
        "T_min": float(T_min),
        "T_max": float(T_max),
        "counts": counts,
        "total": int(sum(counts.values())),
        "generator": "examples.experiments.aviator_manifold.trajectory_generator"
        ".generate_task_trajectory",
        "trajectories": rows,
    }
    with open(root / MANIFEST_NAME, "w") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=False)
        handle.write("\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=str, default=None,
                        help="output root (default: this directory)")
    parser.add_argument("--n-dp", type=int, default=100)
    parser.add_argument("--n-rl", type=int, default=400)
    parser.add_argument("--n-val", type=int, default=50)
    parser.add_argument("--n-test", type=int, default=100)
    parser.add_argument("--T-min", type=float, default=8.0)
    parser.add_argument("--T-max", type=float, default=12.0)
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()

    manifest = generate_split(
        np.random.default_rng(args.seed),
        seed=args.seed,
        n_dp=args.n_dp,
        n_rl=args.n_rl,
        n_val=args.n_val,
        n_test=args.n_test,
        T_min=args.T_min,
        T_max=args.T_max,
        dt=args.dt,
        out_dir=args.out,
    )

    rows = manifest["trajectories"]
    print(f"wrote {manifest['total']} trajectories: {manifest['counts']}")
    for split in _SPLITS:
        sub = [r for r in rows if r["split"] == split]
        if not sub:
            continue
        mv = np.max([r["max_abs_xdot"] for r in sub], axis=0)
        ma = np.max([r["max_abs_xddot"] for r in sub], axis=0)
        Ts = [r["T"] for r in sub]
        print(
            f"  {split:9s} n={len(sub):4d} T=[{min(Ts):.3f}, {max(Ts):.3f}] "
            f"max|xdot|={np.round(mv, 4)} max|xddot|={np.round(ma, 4)}"
        )
    print(f"max|xdot| over all: {np.round(np.max([r['max_abs_xdot'] for r in rows], axis=0), 4)}"
          f" (v_max={V_MAX})")
    print(f"max|xddot| over all: {np.round(np.max([r['max_abs_xddot'] for r in rows], axis=0), 4)}"
          f" (a_max={A_MAX})")


if __name__ == "__main__":
    main()

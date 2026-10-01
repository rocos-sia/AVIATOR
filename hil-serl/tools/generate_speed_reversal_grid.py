"""Generate disjoint, bounded task paths for speed and reversal experiments.

Run from hil-serl, for example::

    python -m tools.generate_speed_reversal_grid --output data/aviator/speed_reversal_grid

The same theta amplitude is used in every cell. Duration changes with the
requested peak speed and number of reversals, so speed and reversal count are
independent factors. All derivatives are analytic and checked against the
task input envelope. RL start-phase randomization is configured separately.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.trajectory_generator import (
    A_MAX, J_MAX, S_RANGE, THETA_RANGE, V_MAX,
)
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup

TASK_V_MAX = (1.5, V_MAX[1])


def make_path(rng: np.random.Generator, theta_speed: float, reversals: int,
              dt: float = 0.01,
              start_cells: np.ndarray | None = None) -> dict[str, np.ndarray]:
    if theta_speed <= 0 or theta_speed > 1.5 or reversals < 1 or dt <= 0:
        raise ValueError("speed must be in (0, 1.5], reversals >= 1, dt > 0")
    theta_amp, slide_amp = 0.5, 0.03
    omega = theta_speed / theta_amp
    duration = reversals * np.pi / omega
    t = np.arange(int(np.ceil(duration / dt)) + 1, dtype=np.float64) * dt
    # Different start states, identical excursion amplitudes and derivative
    # envelopes across cells. Slide phase is independent of the wheel phase.
    if start_cells is None:
        center = np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.105, -0.055)])
        phase = rng.uniform(-np.pi, np.pi, size=2)
    else:
        start = start_cells[rng.integers(len(start_cells))]
        phase = np.array([rng.uniform(-0.15, 0.15) + rng.choice([0.0, np.pi]),
                          rng.uniform(-np.pi, np.pi)])
        center = start - np.array([theta_amp, slide_amp]) * np.sin(phase)
    amplitudes = np.array([theta_amp, slide_amp])
    angle = omega * t[:, None] + phase
    x = center + amplitudes * np.sin(angle)
    xd = amplitudes * omega * np.cos(angle)
    xdd = -amplitudes * omega**2 * np.sin(angle)
    xddd = -amplitudes * omega**3 * np.cos(angle)
    if (np.any(x < np.array([THETA_RANGE[0], S_RANGE[0]]) - 1e-12) or
            np.any(x > np.array([THETA_RANGE[1], S_RANGE[1]]) + 1e-12) or
            np.any(amplitudes * omega > TASK_V_MAX) or
            np.any(amplitudes * omega**2 > A_MAX) or
            np.any(amplitudes * omega**3 > J_MAX)):
        raise ValueError("requested path exceeds the task position, speed, acceleration or jerk envelope")
    observed_reversals = int(np.count_nonzero(np.signbit(xd[1:, 0]) != np.signbit(xd[:-1, 0])))
    if observed_reversals != reversals:
        raise ValueError(f"sampled path has {observed_reversals} rather than {reversals} reversals")
    return {"t": t, "x": x, "xdot": xd, "xddot": xdd, "xdddot": xddd}


def generate(output: Path, seed: int, counts: dict[str, int],
             speeds: tuple[float, ...], reversals: tuple[int, ...],
             manifold_dir: str) -> dict:
    if len(set(speeds)) != len(speeds) or len(set(reversals)) != len(reversals):
        raise ValueError("speed and reversal bins must be unique")
    output.mkdir(parents=True, exist_ok=True)
    lookup = ManifoldLookup(manifold_dir)
    theta_nodes = lookup.theta_axis[np.abs(lookup.theta_axis) <= 0.10]
    slide_nodes = lookup.s_axis[(lookup.s_axis >= -0.10) & (lookup.s_axis <= -0.06)]
    # Distinct start-state grid cells across train, validation and test. This
    # tests start generalization without requiring a pre-motion transition.
    start_cells = {}
    for split_index, split in enumerate(counts):
        cells = [np.array([theta, slide])
                 for i, theta in enumerate(theta_nodes)
                 for j, slide in enumerate(slide_nodes)
                 if (i + 3 * j) % len(counts) == split_index]
        if not cells:
            raise ValueError(f"no grid-aligned start cells for {split}")
        start_cells[split] = np.asarray(cells)
    rows = []
    for split_index, (split, count) in enumerate(counts.items()):
        if count < 0:
            raise ValueError("counts must be nonnegative")
        destination = output / "trajs" / split
        destination.mkdir(parents=True, exist_ok=True)
        index = 0
        for speed_index, speed in enumerate(speeds):
            for reversal_index, n_reversals in enumerate(reversals):
                for local_index in range(count):
                    path_seed = (seed * 1_000_003 + split_index * 1_000_000 +
                                 speed_index * 100_000 + reversal_index * 10_000 + local_index)
                    path = make_path(np.random.default_rng(path_seed), speed, n_reversals,
                                     start_cells=start_cells[split])
                    name = f"traj_{index:04d}.npz"
                    target = destination / name
                    np.savez_compressed(target, **path)
                    rows.append({"split": split, "index": index, "path": str(target.relative_to(output)),
                                 "seed": path_seed, "theta_speed_target": speed,
                                 "theta_speed_sampled": float(np.max(np.abs(path["xdot"][:, 0]))),
                                 "reversals": n_reversals,
                                 "initial_x": path["x"][0].tolist(),
                                 "duration_s": float(path["t"][-1]),
                                 "sha256": hashlib.sha256(target.read_bytes()).hexdigest()})
                    index += 1
    manifest = {"seed": seed, "dt": 0.01, "manifold_dir": manifold_dir,
                "theta_amplitude": 0.5,
                "slide_amplitude": 0.03, "counts_per_cell": counts,
                "speed_bins": speeds, "reversal_bins": reversals,
                "task_speed_limits": TASK_V_MAX, "task_acceleration_limits": A_MAX,
                "task_jerk_limits": J_MAX, "trajectories": rows}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260930)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    parser.add_argument("--train-per-cell", type=int, default=50)
    parser.add_argument("--val-per-cell", type=int, default=10)
    parser.add_argument("--test-per-cell", type=int, default=100)
    args = parser.parse_args()
    manifest = generate(args.output, args.seed,
                        {"rl_train": args.train_per_cell, "val": args.val_per_cell,
                         "test": args.test_per_cell},
                        (0.6, 1.0, 1.2, 1.5), (2, 4, 6), args.manifold_dir)
    print(json.dumps({"output": str(args.output),
                      "n": len(manifest["trajectories"]),
                      "counts_per_cell": manifest["counts_per_cell"]}))


if __name__ == "__main__":
    main()

"""Audit a fixed (theta, s) -> q field on stored task trajectories.

Run from hil-serl, for example::

    python -m tools.audit_static_field --split test --output /tmp/static_field_test.json

The default field is the midpoint of the stored safe phase interval at each
task grid point. It is fixed before any trajectory is read. ``--stride``
replays the same geometric paths faster while retaining the 10 ms control dt.
This is a kinematic LUT audit, not a continuous collision or actuator test.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def field_phase(lookup: ManifoldLookup, x: np.ndarray, fraction: tuple[float, float]) -> np.ndarray:
    """Interpolate a fixed phase field from safe-interval fractions on the grid."""
    box = lookup.safe_interval(x)
    return box["phi_safe_lo"] + np.asarray(fraction) * (
        box["phi_safe_hi"] - box["phi_safe_lo"]
    )


def build_q_grid(lookup: ManifoldLookup, fraction: tuple[float, float]):
    """Freeze one 14-joint posture at every (theta, s) grid node."""
    x = np.stack(np.meshgrid(lookup.theta_axis, lookup.s_axis), axis=-1).reshape(-1, 2)
    phi_raw = field_phase(lookup, x, fraction)
    phase_valid = np.all((phi_raw >= lookup.phi_axis[0]) &
                         (phi_raw <= lookup.phi_axis[-1]), axis=1)
    phi = np.clip(phi_raw, lookup.phi_axis[0], lookup.phi_axis[-1])
    result = lookup.query(x, phi, check_safe=False)
    q = np.concatenate((result["qL"], result["qR"]), axis=1)
    valid = phase_valid & (result["branch"] < 2) & (result["d_min"] >= 0.005)
    return q.reshape(lookup.n_s, lookup.n_theta, 14), phi.reshape(
        lookup.n_s, lookup.n_theta, 2), valid.reshape(lookup.n_s, lookup.n_theta)


def interpolate_q(lookup: ManifoldLookup, q_grid: np.ndarray, x: np.ndarray) -> np.ndarray:
    i, w = lookup._frac(lookup.theta_axis, x[:, 0])
    j, v = lookup._frac(lookup.s_axis, x[:, 1])
    return sum(q_grid[j + dj, i + di] * (v if dj else 1 - v)[:, None]
               * (w if di else 1 - w)[:, None]
               for dj in (0, 1) for di in (0, 1))


def audit_trajectory(lookup: ManifoldLookup, x: np.ndarray, dt: float,
                     fraction: tuple[float, float], d_safe: float,
                     qdot_max: float, phi_dot_max: float,
                     q_grid: np.ndarray | None = None,
                     phi_grid: np.ndarray | None = None) -> dict:
    phi = interpolate_q(lookup, phi_grid, x) if phi_grid is not None else field_phase(
        lookup, x, fraction)
    result = lookup.query(x, phi, check_safe=False)
    q_lut = np.concatenate((result["qL"], result["qR"]), axis=1)
    q = interpolate_q(lookup, q_grid, x) if q_grid is not None else q_lut
    lo = lookup.joint_lower.reshape(14)
    hi = lookup.joint_upper.reshape(14)
    qdot = np.diff(q, axis=0) / dt
    qddot = np.diff(qdot, axis=0) / dt
    phi_dot = np.diff(phi, axis=0) / dt
    d = result["d_min"]
    branch = result["branch"]
    failures = {
        "branch": np.flatnonzero(branch >= 2),
        "clearance": np.flatnonzero(d < d_safe),
        "joint_limit": np.flatnonzero(np.any((q < lo) | (q > hi), axis=1)),
        "joint_speed": np.flatnonzero(np.any(np.abs(qdot) > qdot_max, axis=1)) + 1,
        "invalid": np.flatnonzero(~np.all(np.isfinite(q), axis=1) | ~np.isfinite(d)),
    }
    return {
        "n_samples": len(x),
        "complete": all(len(v) == 0 for v in failures.values()),
        "first_failure": min((int(v[0]), key) for key, v in failures.items() if len(v))
                         if any(len(v) for v in failures.values()) else None,
        "failure_counts": {key: len(v) for key, v in failures.items()},
        "phase_speed_exceedances": int(np.sum(np.any(np.abs(phi_dot) > phi_dot_max, axis=1))),
        "min_clearance_m": float(np.min(d)),
        "min_joint_margin_rad": float(np.min(np.minimum(q - lo, hi - q))),
        "max_joint_speed_rad_s": float(np.max(np.abs(qdot))),
        "max_joint_acceleration_rad_s2": float(np.max(np.abs(qddot))),
        "max_phase_speed_s1": float(np.max(np.abs(phi_dot))),
        "max_q_proxy_error_rad": float(np.max(np.abs(q - q_lut))),
        "initial_phase": phi[0].tolist(),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    ap.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    ap.add_argument("--split", default="test")
    ap.add_argument("--stride", type=int, default=1,
                    help="sample every nth point, keeping dt=0.01 s")
    ap.add_argument("--fraction", type=float, nargs=2, default=(0.5, 0.5),
                    metavar=("LEFT", "RIGHT"))
    ap.add_argument("--limit", type=int)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--field-output", type=Path,
                    help="save the frozen 2-D q table as NPZ")
    ap.add_argument("--field-input", type=Path,
                    help="read a previously frozen 2-D q table")
    args = ap.parse_args()
    if args.stride < 1 or any(not 0 <= v <= 1 for v in args.fraction):
        ap.error("stride must be positive and fractions must be within [0, 1]")

    lookup = ManifoldLookup(args.manifold_dir)
    if args.field_input:
        with np.load(args.field_input) as field:
            if not (np.array_equal(field["theta"], lookup.theta_axis) and
                    np.array_equal(field["s"], lookup.s_axis)):
                ap.error("field axes do not match the manifold")
            q_grid, phi_grid, valid_grid = (field[key] for key in ("q", "phi", "valid"))
            if "fraction" in field:
                args.fraction = field["fraction"].tolist()
    else:
        q_grid, phi_grid, valid_grid = build_q_grid(lookup, tuple(args.fraction))
    if args.field_output:
        args.field_output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.field_output, theta=lookup.theta_axis, s=lookup.s_axis,
                            q=q_grid, phi=phi_grid, valid=valid_grid,
                            fraction=np.asarray(args.fraction))
    paths = sorted(glob.glob(f"{args.trajectory_dir}/trajs/{args.split}/*.npz"))
    if args.limit is not None:
        paths = paths[:args.limit]
    if not paths:
        ap.error("no trajectories found")
    episodes = {}
    for path in paths:
        with np.load(path) as f:
            x = np.asarray(f["x"], dtype=np.float64)[::args.stride]
        episodes[Path(path).stem] = audit_trajectory(
            lookup, x, 0.01, tuple(args.fraction), 0.005, 1.5, 1.5, q_grid,
            phi_grid if args.field_input else None
        )
    report = {
        "method": "frozen 2-D q table with bilinear interpolation",
        "fraction": args.fraction,
        "split": args.split,
        "stride": args.stride,
        "valid_grid_nodes": int(np.sum(valid_grid)),
        "total_grid_nodes": int(valid_grid.size),
        "n_episodes": len(episodes),
        "completion_rate": sum(e["complete"] for e in episodes.values()) / len(episodes),
        "failure_episode_counts": {
            key: sum(e["failure_counts"][key] > 0 for e in episodes.values())
            for key in next(iter(episodes.values()))["failure_counts"]
        },
        "phase_speed_diagnostic_episodes": sum(
            e["phase_speed_exceedances"] > 0 for e in episodes.values()
        ),
        "worst_clearance_m": min(e["min_clearance_m"] for e in episodes.values()),
        "worst_joint_speed_rad_s": max(e["max_joint_speed_rad_s"] for e in episodes.values()),
        "worst_joint_acceleration_rad_s2": max(
            e["max_joint_acceleration_rad_s2"] for e in episodes.values()
        ),
        "max_q_proxy_error_rad": max(e["max_q_proxy_error_rad"] for e in episodes.values()),
        "episodes": episodes,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "episodes"}, indent=2))


if __name__ == "__main__":
    main()

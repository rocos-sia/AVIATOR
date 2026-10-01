"""Time-rescale stored task paths without changing their geometry.

Each source sample supplies position, velocity and acceleration. A quintic
Hermite segment joins neighbouring samples, preserving C2 continuity. The
result is sampled at the original controller dt after changing duration.
"""
from __future__ import annotations

import argparse
import glob
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly, PPoly


def continuous_theta_peak(data: dict) -> float:
    """Exact peak |theta_dot| of the source piecewise quintic reconstruction."""
    t = np.asarray(data["t"])
    curve = BPoly.from_derivatives(t, list(zip(data["x"][:, 0],
                                               data["xdot"][:, 0],
                                               data["xddot"][:, 0])))
    poly = PPoly.from_bernstein_basis(curve)
    critical = poly.derivative(2).roots(extrapolate=False)
    # SciPy encodes identically-zero polynomial intervals with NaN roots.
    # Stationary holds are valid; evaluate knots and finite interior extrema.
    critical = critical[np.isfinite(critical)]
    times = np.r_[t, critical]
    return float(np.max(np.abs(poly.derivative(1)(times))))


def rescale(data: dict, rate: float, dt: float = 0.01) -> dict:
    if rate <= 0:
        raise ValueError("rate must be positive")
    t = np.asarray(data["t"], dtype=np.float64)
    x = np.asarray(data["x"], dtype=np.float64)
    xd = np.asarray(data["xdot"], dtype=np.float64)
    xdd = np.asarray(data["xddot"], dtype=np.float64)
    if len(t) < 3 or not np.all(np.diff(t) > 0):
        raise ValueError("trajectory needs at least 3 increasing time samples")
    # BPoly.from_derivatives handles scalar values; make one C2 curve per task axis.
    curves = [BPoly.from_derivatives(t, list(zip(x[:, j], xd[:, j], xdd[:, j])))
              for j in range(x.shape[1])]
    new_t = np.arange(int(np.floor((t[-1] - t[0]) / (rate * dt))) + 1) * dt
    src_t = np.minimum(t[0] + rate * new_t, t[-1])
    return {
        "t": new_t,
        "x": np.stack([curve(src_t) for curve in curves], axis=1),
        "xdot": rate * np.stack([curve.derivative(1)(src_t) for curve in curves], axis=1),
        "xddot": rate**2 * np.stack([curve.derivative(2)(src_t) for curve in curves], axis=1),
        "xdddot": rate**3 * np.stack([curve.derivative(3)(src_t) for curve in curves], axis=1),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="data/aviator/trajectory_source")
    ap.add_argument("--split", default="val")
    speed = ap.add_mutually_exclusive_group(required=True)
    speed.add_argument("--rate", type=float)
    speed.add_argument("--target-theta-speed", type=float,
                       help="set each trajectory's peak |theta_dot| to this rad/s")
    ap.add_argument("--max-theta-speed", type=float,
                    help="reject trajectories that exceed this rad/s cap after scaling")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    paths = sorted(glob.glob(f"{args.source}/trajs/{args.split}/*.npz"))
    if not paths:
        ap.error("no source trajectories found")
    dest = args.output / "trajs" / args.split
    dest.mkdir(parents=True, exist_ok=True)
    kept = 0
    peak_speeds = []
    for path in paths:
        with np.load(path) as f:
            data = {k: f[k] for k in f.files}
        peak = continuous_theta_peak(data) if args.target_theta_speed else float(
            np.max(np.abs(data["xdot"][:, 0])))
        rate = args.rate if args.rate is not None else args.target_theta_speed / peak
        scaled = rescale(data, rate)
        if args.target_theta_speed is not None:
            # Interpolation between the original 10 ms samples can reveal a
            # slightly higher peak than the source samples. Enforce the cap on
            # the actual emitted samples, including roundoff.
            for _ in range(3):
                emitted_peak = float(np.max(np.abs(scaled["xdot"][:, 0])))
                if emitted_peak <= args.target_theta_speed:
                    break
                rate *= args.target_theta_speed / emitted_peak * (1 - 1e-5)
                scaled = rescale(data, rate)
        if (args.max_theta_speed is not None and
                np.max(np.abs(scaled["xdot"][:, 0])) > args.max_theta_speed + 1e-9):
            continue
        np.savez_compressed(dest / Path(path).name, **scaled)
        kept += 1
        peak_speeds.append(float(np.max(np.abs(scaled["xdot"][:, 0]))))
    if not kept:
        ap.error("all trajectories exceeded the requested speed cap")
    print(f"saved {kept}/{len(paths)} trajectories to {dest}; "
          f"peak theta speed range {min(peak_speeds):.4f}..{max(peak_speeds):.4f} rad/s")


if __name__ == "__main__":
    main()

"""Register DP configurations to a continuous phase path in the RL LUT.

Uses a lexicographic Viterbi search over the tabulated phase grid: minimize
the maximum joint reconstruction error, then total squared error and phase
motion, subject to the actual per-step 1.5 rad/s action bound. Runs both an
unrestricted representation diagnostic and the environment-feasible version.
No prior file is overwritten. Registration output is diagnostic until every
state and action passes the launch gate.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pandas as pd

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def spatial_phase_grid(lookup: ManifoldLookup, x: np.ndarray, side: int,
                       phase_axis: np.ndarray | None = None):
    """Bilinear spatial interpolation for every discrete phase at each x."""
    n = len(x)
    ip = np.arange(lookup.n_phi)[None, :]
    it, wt = lookup._frac(lookup.theta_axis, x[:, 0])
    is_, ws = lookup._frac(lookup.s_axis, x[:, 1])
    qfield = lookup.qL if side == 0 else lookup.qR
    dfield = lookup.dL if side == 0 else lookup.dR
    q = np.zeros((n, lookup.n_phi, 7), dtype=np.float64)
    d = np.zeros((n, lookup.n_phi), dtype=np.float64)
    for dth in (0, 1):
        for ds in (0, 1):
            gp = (is_ + ds) * lookup.n_theta + it + dth
            idx = gp[:, None] * lookup.n_phi + ip
            w = ((wt if dth else 1 - wt) * (ws if ds else 1 - ws))[:, None]
            q += qfield[idx] * w[:, :, None]
            d += dfield[idx] * w
    safe = lookup._bilinear(lookup.safe, it, is_, wt, ws)
    lo, hi = safe[:, 2 * side], safe[:, 2 * side + 1]
    branch = np.maximum.reduce([
        lookup.branch[(is_ + ds) * lookup.n_theta + (it + dth)]
        for ds in (0, 1) for dth in (0, 1)
    ])
    if phase_axis is not None and len(phase_axis) != lookup.n_phi:
        base = lookup.phi_axis
        left = np.clip(np.searchsorted(base, phase_axis, side="right") - 1,
                       0, lookup.n_phi - 2)
        w = (phase_axis - base[left]) / (base[left + 1] - base[left])
        q = q[:, left] * (1 - w)[None, :, None] + q[:, left + 1] * w[None, :, None]
        d = d[:, left] * (1 - w)[None, :] + d[:, left + 1] * w[None, :]
    return q, d, lo, hi, branch


def register_sequence(error: np.ndarray, axis: np.ndarray, dt: np.ndarray,
                      allowed: np.ndarray, max_rate: float):
    """Find the best complete grid path, with hard action and state constraints.

    The primary cost is max_t ||q_LUT-q_DP||_inf, making the reported max error
    the best attainable on this grid. The secondary cost resolves equivalent
    phases toward small sum squared error and small phase movement.
    """
    n, p = error.shape
    if not np.any(allowed[0]):
        return None, "no_allowed_start", 0
    delta = axis[:, None] - axis[None, :]
    max_shift = int(np.ceil(max_rate * float(np.max(dt)) / float(np.min(np.diff(axis))))) + 1
    max_cost = np.where(allowed[0], error[0], np.inf)
    # First solve the exact minimax reconstruction problem.
    for t in range(1, n):
        if not np.any(allowed[t]):
            return None, "no_allowed_state", t
        next_max = np.full(p, np.inf)
        for shift in range(-max_shift, max_shift + 1):
            j = np.arange(max(0, shift), min(p, p + shift))
            i = j - shift
            if not len(j):
                continue
            step = delta[j, i]
            reachable = np.abs(step) <= max_rate * dt[t - 1] + 1e-12
            candidate_max = np.maximum(max_cost[i], error[t, j])
            next_max[j] = np.minimum(next_max[j], np.where(reachable, candidate_max, np.inf))
        next_max[~allowed[t]] = np.inf
        if not np.isfinite(next_max).any():
            return None, "no_speed_feasible_path", t
        max_cost = next_max
    optimum = float(np.min(max_cost))

    # Then find the minimum-sum continuous path within the optimal minimax
    # error level. A one-pass lexicographic predecessor can discard a path
    # that becomes preferable after a larger future error, so use two passes.
    allowed = allowed & (error <= optimum + 1e-12)
    sum_cost = np.where(allowed[0], error[0] ** 2, np.inf)
    back = np.full((n, p), -1, dtype=np.int16)
    for t in range(1, n):
        next_sum = np.full(p, np.inf)
        next_prev = np.full(p, -1, dtype=np.int16)
        for shift in range(-max_shift, max_shift + 1):
            j = np.arange(max(0, shift), min(p, p + shift))
            i = j - shift
            if not len(j):
                continue
            step = delta[j, i]
            reachable = np.abs(step) <= max_rate * dt[t - 1] + 1e-12
            candidate = sum_cost[i] + error[t, j] ** 2 + 1e-4 * step ** 2
            better = reachable & (candidate < next_sum[j])
            jj = j[better]
            next_sum[jj] = candidate[better]
            next_prev[jj] = i[better]
        next_sum[~allowed[t]] = np.inf
        next_prev[~allowed[t]] = -1
        sum_cost = next_sum
        back[t] = next_prev
    end = int(np.argmin(sum_cost))
    path = np.empty(n, dtype=np.int16)
    path[-1] = end
    for t in range(n - 1, 0, -1):
        path[t - 1] = back[t, path[t]]
    return path, "complete", n - 1


def register_trajectory(lookup: ManifoldLookup, csv_path: Path, max_rate: float,
                        phase_subdiv: int = 1):
    frame = pd.read_csv(csv_path, comment="#")
    x = frame[["theta", "s"]].to_numpy(np.float64)
    t = frame["t"].to_numpy(np.float64)
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError(f"{csv_path}: non-increasing timestamps")
    phase_axis = np.linspace(lookup.phi_axis[0], lookup.phi_axis[-1],
                             (lookup.n_phi - 1) * phase_subdiv + 1)
    canonical = np.full((len(frame), 2), np.nan)
    fit_error = np.full((len(frame), 2), np.nan)
    safe_margin = np.full((len(frame), 2), np.nan)
    clearance = np.full((len(frame), 2), np.nan)
    representation_phase = np.full((len(frame), 2), np.nan)
    representation_error = np.full((len(frame), 2), np.nan)
    representation_d = np.full((len(frame), 2), np.nan)
    representation_safe_margin = np.full((len(frame), 2), np.nan)
    result = {"traj_id": csv_path.stem, "states": len(frame)}
    for side, name in enumerate(("L", "R")):
        target = frame[[f"q{name}{j}" for j in range(1, 8)]].to_numpy(np.float64)
        q, d, lo, hi, branch = spatial_phase_grid(lookup, x, side, phase_axis)
        err = np.max(np.abs(q - target[:, None, :]), axis=2)
        all_allowed = np.ones(err.shape, dtype=bool)
        feasible = ((phase_axis[None, :] >= lo[:, None] - 1e-9)
                    & (phase_axis[None, :] <= hi[:, None] + 1e-9)
                    & (d >= 0.005) & (branch[:, None] == 0))
        result[f"{name}_excluded_grid_states"] = int(np.sum(branch != 0))
        result[f"{name}_no_feasible_phase_states"] = int(np.sum(~feasible.any(axis=1)))
        for mode, allowed in (("representation", all_allowed), ("feasible", feasible)):
            path, status, stop = register_sequence(err, phase_axis, dt, allowed, max_rate)
            result[f"{name}_{mode}_status"] = status
            result[f"{name}_{mode}_stop"] = int(stop)
            if path is None:
                continue
            selected = np.arange(len(frame)), path
            selected_err = err[selected]
            result[f"{name}_{mode}_max_error_rad"] = float(selected_err.max())
            result[f"{name}_{mode}_p95_error_rad"] = float(np.quantile(selected_err, .95))
            result[f"{name}_{mode}_max_rate_rad_s"] = float(
                np.max(np.abs(np.diff(phase_axis[path]) / dt)))
            if mode == "representation":
                representation_phase[:, side] = phase_axis[path]
                representation_error[:, side] = selected_err
                representation_d[:, side] = d[selected]
                representation_safe_margin[:, side] = np.minimum(
                    representation_phase[:, side] - lo, hi - representation_phase[:, side])
            if mode == "feasible":
                canonical[:, side] = phase_axis[path]
                fit_error[:, side] = selected_err
                clearance[:, side] = d[selected]
                safe_margin[:, side] = np.minimum(
                    canonical[:, side] - lo, hi - canonical[:, side])
    result["both_feasible"] = bool(np.isfinite(canonical).all())
    return result, {"phi": canonical, "q_error": fit_error,
                    "d_lookup": clearance, "safe_margin": safe_margin,
                    "phi_representation": representation_phase,
                    "q_error_representation": representation_error,
                    "d_lookup_representation": representation_d,
                    "safe_margin_representation": representation_safe_margin,
                    "t": t}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--demo-dir", default="data/aviator/dp_demo")
    parser.add_argument("--output-dir", default="data/aviator/phase_registration")
    parser.add_argument("--max-rate", type=float, default=1.5)
    parser.add_argument("--phase-subdiv", type=int, default=1,
                        help="interpolated subcells per native LUT phase interval")
    args = parser.parse_args()
    if args.phase_subdiv < 1:
        parser.error("--phase-subdiv must be positive")
    paths = sorted(Path(p) for p in glob.glob(str(Path(args.demo_dir) / "traj_*.pkl")))
    if not paths:
        raise ValueError("no paired demonstration PKLs")
    lookup = ManifoldLookup(args.manifold_dir)
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    results = []
    for index, pkl in enumerate(paths, 1):
        row, series = register_trajectory(lookup, pkl.with_suffix(".csv"), args.max_rate,
                                          args.phase_subdiv)
        np.savez_compressed(outdir / f"{pkl.stem}.npz", **series)
        results.append(row)
        if index % 10 == 0 or index == len(paths):
            print(f"registered {index}/{len(paths)}", flush=True)
    table = pd.DataFrame(results)
    table.to_csv(outdir / "trajectory_results.csv", index=False)
    summary = {
        "trajectories": len(results),
        "representation_complete_both": int(((table.L_representation_status == "complete") &
                                             (table.R_representation_status == "complete")).sum()),
        "feasible_complete_both": int(table.both_feasible.sum()),
        "left_feasible_status": table.L_feasible_status.value_counts().to_dict(),
        "right_feasible_status": table.R_feasible_status.value_counts().to_dict(),
        "max_rate_rad_s": args.max_rate,
        "phase_subdiv": args.phase_subdiv,
        "output": str(outdir),
    }
    (outdir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

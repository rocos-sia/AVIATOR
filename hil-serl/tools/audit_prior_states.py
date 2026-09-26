"""Audit every replayable DP state against the LUT and the DP geometry engine.

The output Parquet keeps raw DP/LUT joint vectors. Missing branch identities and
an arbitrary-q Python-environment clearance evaluator are recorded as nulls;
the current LUT only supplies a grid exclusion flag and interpolated distance.
Run with the MuJoCo 3.x Python environment and the compiled C++ evaluator.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pickle
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def physical_evaluate(rows: np.ndarray, binary: str, config: str, workdir: str) -> pd.DataFrame:
    """Evaluate (theta,s,q[14]) with record_dp's exact wall cloud and FK code."""
    with tempfile.TemporaryDirectory(dir=workdir) as tmp:
        input_path = os.path.join(tmp, "states.csv")
        np.savetxt(input_path, rows, delimiter=",", fmt="%.17g")
        proc = subprocess.run(
            [binary, config, tmp, "0.005", "0.03", "10", "0.1", "audit_states", input_path],
            check=True, capture_output=True, text=True,
        )
        result = pd.read_csv(os.path.join(tmp, "audit_states.csv"))
    if len(result) != len(rows):
        raise RuntimeError(f"geometry evaluator returned {len(result)} of {len(rows)} rows: {proc.stdout}")
    return result


def replay_failures(paths: list[str], trajectory_dir: str, manifold_dir: str, scale: float) -> dict:
    env = AviatorManifoldEnv(manifold_dir, trajectories=[], phi_dot_scale=scale)
    result = {}
    for path in paths:
        stem = Path(path).stem
        source = Path(trajectory_dir) / "trajs" / "dp_train" / f"{stem}.npz"
        if not source.is_file():
            result[stem] = (-1, "missing_source", np.nan, np.nan, np.nan)
            continue
        with open(path, "rb") as file:
            transitions = pickle.load(file)
        with np.load(source) as file:
            env._trajectories = [{k: np.asarray(file[k], dtype=np.float64) for k in file.files}]
        # Route A emits the canonical LUT phase directly, so the DP's phase is
        # already on the replay chart. Start the replay from the DP's recorded
        # φ₀ (recovered from the closed-loop sin/cos in the first observation)
        # instead of the default φ=0: the DP solves for its own φ₀, and a shifted
        # start integrates the same actions into a different phase path that can
        # drift out of the safe box and trip the clearance check.
        obs0 = transitions[0]["observations"]["state"][0]
        phi_0 = np.array([np.arctan2(obs0[4], obs0[5]), np.arctan2(obs0[6], obs0[7])])
        env.reset(options={"initial_phi": phi_0})
        for step, transition in enumerate(transitions, start=1):
            _, _, terminated, truncated, info = env.step(transition["actions"])
            if terminated or truncated:
                result[stem] = (step, info["termination"], float(env._phi[0]),
                                float(env._phi[1]), float(info.get("d_min", np.nan)))
                break
        else:
            result[stem] = (len(transitions), "missing_terminal", float(env._phi[0]),
                            float(env._phi[1]), np.nan)
    return result


def inverse_phase_scan(lookup: ManifoldLookup, x: np.ndarray, q_dp: np.ndarray,
                       batch_size: int = 128) -> tuple[np.ndarray, np.ndarray]:
    """Diagnostic nearest phase in the existing LUT, independently for each arm.

    This is not a trajectory remapping: nearest phases may jump between
    equivalent periodic representations and do not define valid policy actions.
    """
    phase = np.full((len(x), 2), np.nan)
    residual = np.full((len(x), 2), np.nan)
    axis = lookup.phi_axis
    for start in range(0, len(x), batch_size):
        stop = min(len(x), start + batch_size)
        xi = x[start:stop]
        i_th, w_th = lookup._frac(lookup.theta_axis, xi[:, 0])
        i_s, w_s = lookup._frac(lookup.s_axis, xi[:, 1])
        for side, field in enumerate((lookup.qL, lookup.qR)):
            # Bilinear interpolation at every tabulated phase. A same-q match
            # tests the coordinate chart without assuming DP's phi origin.
            grid = np.zeros((len(xi), lookup.n_phi, 7), dtype=np.float64)
            for dth in (0, 1):
                for ds in (0, 1):
                    gp = (i_s + ds) * lookup.n_theta + (i_th + dth)
                    idx = gp[:, None] * lookup.n_phi + np.arange(lookup.n_phi)[None, :]
                    weight = ((w_th if dth else 1 - w_th) *
                              (w_s if ds else 1 - w_s))[:, None, None]
                    grid += field[idx] * weight
            target = q_dp[start:stop, side * 7:(side + 1) * 7]
            error = np.max(np.abs(grid - target[:, None, :]), axis=2)
            best = np.argmin(error, axis=1)
            phase[start:stop, side] = axis[best]
            residual[start:stop, side] = error[np.arange(len(xi)), best]
    return phase, residual


def audit(args: argparse.Namespace) -> dict:
    paths = sorted(glob.glob(os.path.join(args.demo_dir, "traj_*.pkl")))
    if not paths:
        raise ValueError("no paired DP demonstrations found")
    lookup = ManifoldLookup(args.manifold_dir)
    frames = []
    for path in paths:
        stem = Path(path).stem
        csv_path = Path(args.demo_dir) / f"{stem}.csv"
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
        frame = pd.read_csv(csv_path, comment="#")
        frame.insert(0, "traj_id", stem)
        frame.insert(1, "sample_index", np.arange(len(frame)))
        frames.append(frame)
    source = pd.concat(frames, ignore_index=True)
    x = source[["theta", "s"]].to_numpy(np.float64)
    phi = np.stack([
        np.arctan2(source[f"sin_phi_{side}"], source[f"cos_phi_{side}"])
        for side in ("L", "R")
    ], axis=1)
    q_dp = source[[f"q{side}{j}" for side in ("L", "R") for j in range(1, 8)]].to_numpy(np.float64)
    n = len(source)
    q_lut = np.full_like(q_dp, np.nan)
    d_lookup = np.full(n, np.nan)
    branch_flag = np.full(n, -1, dtype=np.int16)
    safe_margin = np.full(n, np.nan)
    lookup_valid = np.zeros(n, dtype=bool)
    for start in range(0, n, args.batch_size):
        stop = min(n, start + args.batch_size)
        xi, pi = x[start:stop], phi[start:stop]
        valid = ((xi[:, 0] >= lookup.th_min - 1e-9) & (xi[:, 0] <= lookup.th_max + 1e-9)
                 & (xi[:, 1] >= lookup.s_min - 1e-9) & (xi[:, 1] <= lookup.s_max + 1e-9)
                 & np.all((pi >= lookup.phi_axis[0] - 1e-9) &
                          (pi <= lookup.phi_axis[-1] + 1e-9), axis=1))
        idx = start + np.flatnonzero(valid)
        if len(idx) == 0:
            continue
        result = lookup.query(x[idx], phi[idx], check_safe=False)
        q_lut[idx] = np.concatenate([result["qL"], result["qR"]], axis=1)
        d_lookup[idx] = result["d_min"]
        branch_flag[idx] = result["branch"]
        safe_margin[idx] = np.minimum(result["m_phi_minus"], result["m_phi_plus"]).min(axis=1)
        lookup_valid[idx] = True

    rows_dp = np.column_stack([x, q_dp])
    physical_dp = physical_evaluate(rows_dp, args.binary, args.config, args.output_dir)
    physical_lut = pd.DataFrame(np.nan, index=np.arange(n), columns=physical_dp.columns)
    if lookup_valid.any():
        rows_lut = np.column_stack([x[lookup_valid], q_lut[lookup_valid]])
        evaluated = physical_evaluate(rows_lut, args.binary, args.config, args.output_dir)
        physical_lut.loc[lookup_valid, :] = evaluated.to_numpy()

    error = np.arctan2(np.sin(q_lut - q_dp), np.cos(q_lut - q_dp))
    nearest_phase, nearest_residual = inverse_phase_scan(lookup, x, q_dp)
    out = pd.DataFrame({
        "traj_id": source["traj_id"], "sample_index": source["sample_index"],
        "t": source["t"], "theta": source["theta"], "s": source["s"],
        "phi_L_wrapped": phi[:, 0], "phi_R_wrapped": phi[:, 1],
        "lookup_valid": lookup_valid,
        "safe_interval_margin": safe_margin,
        "branch_LUT_exclusion_flag": branch_flag,
        "branch_DP": pd.Series([pd.NA] * n, dtype="Int64"),
        "branch_reconstructed": pd.Series([pd.NA] * n, dtype="Int64"),
        "q_error_inf_rad": np.nanmax(np.abs(error), axis=1),
        "q_error_l2_rad": np.linalg.norm(error, axis=1),
        "nearest_LUT_phi_L": nearest_phase[:, 0],
        "nearest_LUT_phi_R": nearest_phase[:, 1],
        "nearest_LUT_q_error_L_rad": nearest_residual[:, 0],
        "nearest_LUT_q_error_R_rad": nearest_residual[:, 1],
        "d_DP_recorded_m": source["d_min"],
        "d_DP_recomputed_same_q_m": physical_dp["d_min"],
        "d_env_arbitrary_q_DP_m": np.full(n, np.nan),
        "d_LUT_interpolated_m": d_lookup,
        "d_physical_q_LUT_m": physical_lut["d_min"],
        "FK_error_q_DP_m": physical_dp["task_error"],
        "FK_error_q_LUT_m": physical_lut["task_error"],
        "joint_margin_q_DP_rad": physical_dp["joint_margin"],
        "joint_margin_q_LUT_rad": physical_lut["joint_margin"],
        "contact_q_DP": physical_dp["collision"].astype(bool),
        "contact_q_LUT": physical_lut["collision"].astype("boolean"),
    })
    for j in range(14):
        out[f"q_DP_{j + 1}"] = q_dp[:, j]
        out[f"q_LUT_{j + 1}"] = q_lut[:, j]
    out["d_recompute_error_m"] = out["d_DP_recomputed_same_q_m"] - out["d_DP_recorded_m"]
    out["d_lookup_physical_error_m"] = out["d_LUT_interpolated_m"] - out["d_physical_q_LUT_m"]
    out["DP_meets_5mm"] = out["d_DP_recomputed_same_q_m"] >= 0.005
    out["LUT_physical_meets_5mm"] = out["d_physical_q_LUT_m"] >= 0.005
    out["LUT_lookup_meets_5mm"] = out["d_LUT_interpolated_m"] >= 0.005

    with open(os.path.join(args.demo_dir, "phi_dot_scale.json")) as file:
        scale = float(json.load(file)["phi_dot_scale"])
    failures = replay_failures(paths, args.trajectory_dir, args.manifold_dir, scale)
    out["replay_first_failure_step"] = out["traj_id"].map(lambda name: failures[name][0])
    out["replay_first_failure_cause"] = out["traj_id"].map(lambda name: failures[name][1])
    out["replay_phi_L_at_stop"] = out["traj_id"].map(lambda name: failures[name][2])
    out["replay_phi_R_at_stop"] = out["traj_id"].map(lambda name: failures[name][3])
    out["replay_d_lookup_at_stop_m"] = out["traj_id"].map(lambda name: failures[name][4])
    # This selects the DP row at the same time index. Its q and d may differ
    # from the replay state because reset starts from a different phase.
    out["is_DP_row_at_replay_stop"] = out["sample_index"] == out["replay_first_failure_step"]

    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    table_path = output_path / "prior_state_truth.parquet"
    out.to_parquet(table_path, index=False)
    first = out[out["is_DP_row_at_replay_stop"]].copy()
    first.to_csv(output_path / "first_failures.csv", index=False)
    quantiles = lambda values: {str(k): float(v) for k, v in values.quantile([0.5, 0.95, 0.99, 1]).items()}
    summary = {
        "samples": n, "trajectories": len(paths), "table": str(table_path),
        "gate_A_q_error_inf_rad": quantiles(out["q_error_inf_rad"]),
        "gate_A_nearest_phase_q_error_inf_rad": quantiles(pd.Series(nearest_residual.max(axis=1))),
        "phase_coordinate_offset_abs_rad": quantiles(pd.Series(
            np.abs(nearest_phase - phi).max(axis=1))),
        "gate_A_lookup_invalid": int((~lookup_valid).sum()),
        "gate_B_recorded_vs_recomputed_m": quantiles(out["d_recompute_error_m"].abs()),
        "gate_B_env_arbitrary_q": "unavailable: current env only interpolates d(x,phi)",
        "LUT_d_vs_physical_same_q_m": quantiles(out["d_lookup_physical_error_m"].abs()),
        "DP_below_5mm_states": int((~out["DP_meets_5mm"]).sum()),
        "LUT_physical_below_5mm_states": int((~out["LUT_physical_meets_5mm"]).sum()),
        "LUT_lookup_below_5mm_states": int((~out["LUT_lookup_meets_5mm"]).sum()),
        "DP_contact_states": int(out["contact_q_DP"].sum()),
        "replay_causes": out.groupby("traj_id")["replay_first_failure_cause"].first().value_counts().to_dict(),
        "branch_DP": "unavailable: DP CSV has no branch identity",
        "branch_LUT": "grid exclusion code, not IK branch identity",
    }
    with open(output_path / "summary.json", "w") as file:
        json.dump(summary, file, indent=2)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    parser.add_argument("--demo-dir", default="data/aviator/dp_demo")
    parser.add_argument("--output-dir", default="data/aviator/prior_state_audit")
    parser.add_argument("--binary", default="../AviatorRobot/build/bin/aviator_clearance_trajectory")
    parser.add_argument("--config", default="../AviatorRobot/config/aviator.yaml")
    parser.add_argument("--batch-size", type=int, default=2048)
    args = parser.parse_args()
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    print(json.dumps(audit(args), indent=2))


if __name__ == "__main__":
    main()

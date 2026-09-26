"""Plot the LUT safe-phase domain with registered DP and RLPD trajectories.

The DP path uses representation-mode phase registration because the original
DP phase is not the LUT coordinate. The plot is descriptive, not a safety
certificate for the DP trajectory in the LUT environment.
"""

from __future__ import annotations

import csv
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

ROOT = Path(__file__).resolve().parents[2]
HIL = ROOT / "hil-serl"
sys.path.insert(0, str(HIL))

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from flax.training import checkpoints
from matplotlib.lines import Line2D

sys.path.insert(0, "/home/rocos/.codex/skills/nature-figure/scripts")
from audit_panel_alignment import require_matplotlib_panel_alignment

from examples.experiments.aviator_manifold.config import TrainConfig
from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from tools.eval_rlpd_policy import make_sac_policy

OUT = Path(__file__).resolve().parent
DATA = HIL / "data/aviator"
MANIFOLD = DATA / "manifold_phi"
DP = DATA / "dp_demo"
REGISTRATION = DATA / "phase_registration"
TASKS = DATA / "trajectory_source/trajs/dp_train"
CHECKPOINT_ROOT = HIL / "examples/experiments/aviator_manifold/route_a_lut_native_rlpd_800x64"
CHECKPOINT_STEP = 60000  # best completion on the existing fixed 50-trajectory validation table


def load_task(index: int) -> dict[str, np.ndarray]:
    with np.load(TASKS / f"traj_{index:04d}.npz") as source:
        return {key: np.asarray(source[key], dtype=np.float64) for key in source.files}


def make_agent(env: AviatorManifoldEnv):
    cfg = TrainConfig()
    sample_obs = {"state": np.asarray(env.observation_space.sample()["state"])[None, :]}
    agent = cfg.make_sac_agent(seed=0, sample_obs=sample_obs,
                               sample_action=env.action_space.sample())
    state = checkpoints.restore_checkpoint(str(CHECKPOINT_ROOT), agent.state,
                                            step=CHECKPOINT_STEP)
    return agent.replace(state=state)


def run_episode(env: AviatorManifoldEnv, policy, task: dict[str, np.ndarray]):
    env._trajectories = [task]
    obs, _ = env.reset()
    rows = [(float(task["t"][0]), *env._x, *env._phi, np.nan, np.nan)]
    termination = "end"
    while True:
        action = policy(obs, env)
        obs, _, terminated, truncated, info = env.step(action)
        rows.append((float(task["t"][env._t]), *env._x, *env._phi,
                     float(info.get("d_min", np.nan)), float(info.get("max_qdot", np.nan))))
        if terminated or truncated:
            if terminated:
                termination = str(info.get("termination", "terminated"))
            break
    return np.asarray(rows, dtype=float), termination


def choose_example(env: AviatorManifoldEnv, policy):
    ranking = pd.read_csv(REGISTRATION / "trajectory_results.csv")
    ranking = ranking[(ranking.L_representation_status == "complete") &
                      (ranking.R_representation_status == "complete")].copy()
    ranking["max_error"] = ranking[["L_representation_max_error_rad",
                                    "R_representation_max_error_rad"]].max(axis=1)
    ranking = ranking.sort_values("max_error")
    attempts = []
    for item in ranking.itertuples():
        index = int(item.traj_id.split("_")[1])
        task = load_task(index)
        trace, termination = run_episode(env, policy, task)
        attempts.append({"index": index, "termination": termination,
                         "steps": len(trace) - 1,
                         "max_dp_registration_error_rad": float(item.max_error)})
        if termination == "end" and item.max_error <= 0.005:
            return index, task, trace, attempts
    raise RuntimeError("No complete RLPD rollout with DP registration error <= 0.005 rad")


def save_trace(trace: np.ndarray):
    with (OUT / "rlpd_trace.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_s", "theta_rad", "s_m", "phi_L", "phi_R",
                         "d_min_m", "max_qdot_rad_s"])
        writer.writerows(trace)


def plot(index: int, task: dict[str, np.ndarray], trace: np.ndarray,
         dp_phi: np.ndarray, registration_error: float, zoom: bool = False):
    manifest = json.loads((MANIFOLD / "manifest.json").read_text())
    ns, nt = manifest["n_s"], manifest["n_theta"]
    safe = np.fromfile(MANIFOLD / "safe.bin", dtype="<f4").reshape(ns, nt, 4)
    branch = np.fromfile(MANIFOLD / "branch.bin", dtype="u1").reshape(ns, nt)
    assert safe.shape == (ns, nt, 4) and branch.shape == (ns, nt)
    theta = np.linspace(manifest["th_min"], manifest["th_max"], nt) * 180 / np.pi
    slide = np.linspace(manifest["s_min"], manifest["s_max"], ns) * 1000
    TH, SL = np.meshgrid(theta, slide)
    if zoom:
        th_values = task["x"][:, 0] * 180 / np.pi
        s_values = task["x"][:, 1] * 1000
        x_bounds = (max(-50, float(th_values.min()) - 7),
                    min(50, float(th_values.max()) + 7))
        y_bounds = (max(-160, float(s_values.min()) - 12),
                    min(0, float(s_values.max()) + 12))
        th_mask = (theta >= x_bounds[0]) & (theta <= x_bounds[1])
        s_mask = (slide >= y_bounds[0]) & (slide <= y_bounds[1])
        TH, SL = TH[np.ix_(s_mask, th_mask)], SL[np.ix_(s_mask, th_mask)]

    mpl.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans", "Arial"],
        "font.size": 8, "axes.titlesize": 10, "axes.labelsize": 8,
        "pdf.fonttype": 42, "svg.fonttype": "none", "figure.facecolor": "white",
    })
    fig = plt.figure(figsize=(11.6, 5.8), dpi=300)
    axes = [fig.add_subplot(121, projection="3d"),
            fig.add_subplot(122, projection="3d")]
    colors = {"lower": "#a9bbc9", "upper": "#6786a2",
              "dp": "#e17857", "rl": "#087e88"}
    for arm, ax in enumerate(axes):
        lo = safe[:, :, 2 * arm].astype(float)
        hi = safe[:, :, 2 * arm + 1].astype(float)
        valid = (branch == 0) & np.isfinite(lo) & np.isfinite(hi) & (lo <= hi)
        lo = np.where(valid, lo, np.nan)
        hi = np.where(valid, hi, np.nan)
        if zoom:
            lo, hi = lo[np.ix_(s_mask, th_mask)], hi[np.ix_(s_mask, th_mask)]
        ax.plot_surface(TH, SL, lo, color=colors["lower"], alpha=0.24,
                        linewidth=0, antialiased=False, shade=False)
        ax.plot_surface(TH, SL, hi, color=colors["upper"], alpha=0.24,
                        linewidth=0, antialiased=False, shade=False)
        ax.plot(task["x"][:, 0] * 180 / np.pi, task["x"][:, 1] * 1000,
                dp_phi[:, arm], color=colors["dp"], lw=2.2, alpha=0.96)
        ax.plot(trace[:, 1] * 180 / np.pi, trace[:, 2] * 1000,
                trace[:, 3 + arm], color=colors["rl"], lw=2.2, alpha=0.98)
        ax.scatter([trace[0, 1] * 180 / np.pi], [trace[0, 2] * 1000],
                   [trace[0, 3 + arm]], color=colors["rl"], s=22, depthshade=False)
        ax.set(xlabel="", ylabel="",
               zlabel=f"φ{'L' if arm == 0 else 'R'} (rad)",
               xlim=x_bounds if zoom else (-50, 50),
               ylim=y_bounds if zoom else (-160, 0),
               zlim=tuple(manifest["phi_grid"]))
        ax.set_xticks([-10, 10, 30] if zoom else [-40, 0, 40])
        ax.set_yticks([-130, -100, -70] if zoom else [-160, -80, 0])
        ax.set_zticks([-0.3, 0, 0.3])
        ax.set_title(f"{'a' if arm == 0 else 'b'}   {'Left' if arm == 0 else 'Right'} arm",
                     loc="left", weight="bold", pad=0)
        ax.view_init(elev=23, azim=-62)
        ax.set_box_aspect((1.3, 1.2, 0.88), zoom=0.9)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.grid(False)
    handles = [Line2D([0], [0], color=colors["dp"], lw=2.5,
                      label="DP (registered to LUT phase)"),
               Line2D([0], [0], color=colors["rl"], lw=2.5,
                      label=f"RLPD (checkpoint {CHECKPOINT_STEP:,})"),
               Line2D([0], [0], color=colors["upper"], lw=7, alpha=0.35,
                      label="LUT safe-phase bounds")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
               ncol=3, frameon=False, fontsize=8)
    fig.suptitle("Bimanual redundancy manifold: DP and RLPD phase paths" +
                 (" (detail)" if zoom else ""),
                 y=0.995, fontsize=12, weight="bold")
    fig.text(0.5, 0.065, "Axes: θ = wheel angle (°) · s = pull (mm) · φ = redundancy phase (rad)",
             ha="center", fontsize=8, color="#253443")
    fig.text(0.5, 0.026,
             f"DP task {index:04d} · DP registration max joint error {registration_error:.4f} rad · "
             "shaded domain excludes flagged grid cells",
             ha="center", fontsize=7, color="#4b5563")
    fig.subplots_adjust(left=0.025, right=0.98, bottom=0.10, top=0.89, wspace=0.02)
    stem = "manifold_dp_rl_detail" if zoom else "manifold_dp_rl"
    require_matplotlib_panel_alignment(fig, json_out=str(OUT / f"{stem}.alignment.json"),
                                       overlay_svg=str(OUT / f"{stem}.alignment.svg"),
                                       tolerance_pt=1.5, gutter_tolerance_pt=1.5,
                                       strict=True)
    fig.savefig(OUT / f"{stem}.png", dpi=300)
    fig.savefig(OUT / f"{stem}.tiff", dpi=300)
    fig.savefig(OUT / f"{stem}.svg")
    fig.savefig(OUT / f"{stem}.pdf")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    first = load_task(0)
    env = AviatorManifoldEnv(manifold_dir=str(MANIFOLD), trajectory_dir=None,
                             split="dp_train", phi_dot_scale=TrainConfig().phi_dot_scale,
                             trajectories=[first])
    policy = make_sac_policy(make_agent(env))
    index, task, trace, attempts = choose_example(env, policy)
    with np.load(REGISTRATION / f"traj_{index:04d}.npz") as registered:
        dp_phi = np.asarray(registered["phi_representation"], dtype=float)
        registration_error = float(np.nanmax(registered["q_error_representation"]))
    assert len(task["x"]) == len(dp_phi) == len(trace), "Selected paths must complete"
    save_trace(trace)
    plot(index, task, trace, dp_phi, registration_error)
    plot(index, task, trace, dp_phi, registration_error, zoom=True)
    (OUT / "figure_metadata.json").write_text(json.dumps({
        "task_index": index, "checkpoint_step": CHECKPOINT_STEP,
        "checkpoint_run": CHECKPOINT_ROOT.name,
        "dp_phase": "representation-mode registration to LUT coordinate",
        "max_dp_joint_reconstruction_error_rad": registration_error,
        "rl_termination": "end", "rl_steps": len(trace) - 1,
        "attempts": attempts,
        "interpretation": "DP curve is a registered approximation; RL curve is an actual deterministic environment rollout. LUT safe bounds do not certify physical clearance.",
    }, indent=2) + "\n")
    print(f"Saved manifold figure for task {index:04d}; RL steps={len(trace)-1}; "
          f"DP registration error={registration_error:.5f} rad")


if __name__ == "__main__":
    main()

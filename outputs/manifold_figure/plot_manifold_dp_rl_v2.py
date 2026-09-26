"""Plot the LUT safe-phase domain with registered DP and RLPD trajectories.

The DP path uses representation-mode phase registration because the original
DP phase is not the LUT coordinate. The plot is descriptive, not a safety
certificate for the DP trajectory in the LUT environment.

v2 — scientific-visualization pass (plot_manifold_dp_rl.py is unchanged):
  * Okabe-Ito colourblind-safe palette (blue = safe corridor, orange = DP,
    green = RLPD)
  * on-axis labels theta(deg) / s(mm) / phi(rad) instead of a shared caption
  * start (o) / end (square) markers on BOTH paths
  * nature publication style (scientific-visualization skill presets)
  * --reuse-trace: replot from the saved rlpd_trace.csv without re-running the
    (deterministic) rollout — same data, faster iteration
"""

from __future__ import annotations

import argparse
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

sys.path.insert(0, "/home/rocos/.claude/skills/scientific-visualization/scripts")
from style_presets import apply_publication_style, set_color_palette

apply_publication_style("nature")
set_color_palette("okabe_ito")
# 3D axes + explicit subplots_adjust need a fixed layout engine, not
# constrained_layout (which the nature preset enables by default).
plt.rcParams["figure.constrained_layout.use"] = False

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

# Okabe-Ito colourblind-safe palette: the corridor's lower/upper sheets share
# the blue family; orange (DP) and bluish green (RLPD) are maximally distinct
# from the blue and from each other.
COLORS = {"lower": "#56B4E9", "upper": "#0072B2",
          "dp": "#D55E00", "rl": "#009E73",
          "loop": "#CC79A7"}  # reddish purple: loop-open φ arc (near wrist singularity)


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
         dp_phi: np.ndarray, registration_error: float, view: str = "overview"):
    """Standalone 2-panel (Left/Right) figure for a single view.

    ``view``: "overview" (full range, 3/4) · "detail" (zoomed to the task, 3/4) ·
    "top-down" (full range, elev=90 — collapses φ to the θ–s footprint).
    """
    zoom = (view == "detail")
    top_down = (view == "top-down")
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

    mpl.rcParams.update({"pdf.fonttype": 42, "svg.fonttype": "none"})
    fig = plt.figure(figsize=(11.6, 5.8), dpi=300)
    axes = [fig.add_subplot(121, projection="3d"),
            fig.add_subplot(122, projection="3d")]
    for arm, ax in enumerate(axes):
        lo = safe[:, :, 2 * arm].astype(float)
        hi = safe[:, :, 2 * arm + 1].astype(float)
        finite = np.isfinite(lo) & np.isfinite(hi) & (lo <= hi)
        closed = (branch == 0) & finite      # cycle-consistent self-motion loop
        open_loop = (branch == 1) & finite   # loop-open arc (near wrist singularity)
        if zoom:
            sl = np.ix_(s_mask, th_mask)
            lo, hi = lo[sl], hi[sl]
            closed, open_loop = closed[sl], open_loop[sl]
        # safe corridor (closed loop): translucent lower/upper sheets, blue family
        ax.plot_surface(TH, SL, np.where(closed, lo, np.nan), color=COLORS["lower"],
                        alpha=0.26, linewidth=0, antialiased=False, shade=False)
        ax.plot_surface(TH, SL, np.where(closed, hi, np.nan), color=COLORS["upper"],
                        alpha=0.26, linewidth=0, antialiased=False, shade=False)
        # loop-open arc (valid, near wrist singularity): distinct translucent sheets
        ax.plot_surface(TH, SL, np.where(open_loop, lo, np.nan), color=COLORS["loop"],
                        alpha=0.30, linewidth=0, antialiased=False, shade=False)
        ax.plot_surface(TH, SL, np.where(open_loop, hi, np.nan), color=COLORS["loop"],
                        alpha=0.30, linewidth=0, antialiased=False, shade=False)
        # DP path (registered phase) and RLPD path (actual rollout)
        ax.plot(task["x"][:, 0] * 180 / np.pi, task["x"][:, 1] * 1000,
                dp_phi[:, arm], color=COLORS["dp"], lw=2.4, alpha=0.97)
        ax.plot(trace[:, 1] * 180 / np.pi, trace[:, 2] * 1000,
                trace[:, 3 + arm], color=COLORS["rl"], lw=2.4, alpha=0.98)
        # start (o) / end (square) markers on both paths
        ax.scatter([task["x"][0, 0] * 180 / np.pi], [task["x"][0, 1] * 1000],
                   [dp_phi[0, arm]], color=COLORS["dp"], s=28, marker="o",
                   depthshade=False, edgecolor="k", linewidths=0.3)
        ax.scatter([task["x"][-1, 0] * 180 / np.pi], [task["x"][-1, 1] * 1000],
                   [dp_phi[-1, arm]], color=COLORS["dp"], s=20, marker="s",
                   depthshade=False, edgecolor="k", linewidths=0.3)
        ax.scatter([trace[0, 1] * 180 / np.pi], [trace[0, 2] * 1000],
                   [trace[0, 3 + arm]], color=COLORS["rl"], s=28, marker="o",
                   depthshade=False, edgecolor="k", linewidths=0.3)
        ax.scatter([trace[-1, 1] * 180 / np.pi], [trace[-1, 2] * 1000],
                   [trace[-1, 3 + arm]], color=COLORS["rl"], s=20, marker="s",
                   depthshade=False, edgecolor="k", linewidths=0.3)
        ax.set(xlabel="θ (°)", ylabel="s (mm)",
               zlabel=f"φ{'L' if arm == 0 else 'R'} (rad)",
               xlim=x_bounds if zoom else (-50, 50),
               ylim=y_bounds if zoom else (-160, 0),
               zlim=tuple(manifest["phi_grid"]))
        ax.set_xticks([-10, 10, 30] if zoom else [-40, 0, 40])
        ax.set_yticks([-130, -100, -70] if zoom else [-160, -80, 0])
        ax.set_zticks([-0.3, 0, 0.3])
        ax.set_title(f"{'a' if arm == 0 else 'b'}   {'Left' if arm == 0 else 'Right'} arm",
                     loc="left", weight="bold", pad=0, fontsize=9)
        ax.view_init(elev=(90 if top_down else 23), azim=-62)
        ax.set_box_aspect((1.3, 1.2, 0.88), zoom=0.9)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.grid(False)
    handles = [Line2D([0], [0], color=COLORS["dp"], lw=2.5,
                      label="DP (registered to LUT phase)"),
               Line2D([0], [0], color=COLORS["rl"], lw=2.5,
                      label=f"RLPD (checkpoint {CHECKPOINT_STEP:,})"),
               Line2D([0], [0], color=COLORS["upper"], lw=7, alpha=0.40,
                      label="safe φ corridor [φ$_{min}$, φ$_{max}$]"),
               Line2D([0], [0], color=COLORS["loop"], lw=7, alpha=0.40,
                      label="loop-open φ arc (near wrist singularity)")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
               ncol=2, frameon=False, fontsize=8)
    fig.suptitle("Bimanual redundancy manifold: DP and RLPD phase paths" +
                 {"detail": " (detail)", "top-down": " (top-down)"}.get(view, ""),
                 y=0.995, fontsize=12, weight="bold")
    fig.text(0.5, 0.026,
             f"DP task {index:04d} · DP registration max joint error {registration_error:.4f} rad · "
             "○ start / ■ end · blue = cycle-consistent loop, purple = loop-open arc (both valid)" +
             (" · top-down collapses φ (shows θ–s footprint)" if top_down else ""),
             ha="center", fontsize=7, color="#4b5563")
    fig.subplots_adjust(left=0.025, right=0.98, bottom=0.10, top=0.89, wspace=0.02)
    stem = {"overview": "manifold_dp_rl_v2",
            "detail": "manifold_dp_rl_v2_detail",
            "top-down": "manifold_dp_rl_v2_top_down"}[view]
    require_matplotlib_panel_alignment(fig, json_out=str(OUT / f"{stem}.alignment.json"),
                                       overlay_svg=str(OUT / f"{stem}.alignment.svg"),
                                       tolerance_pt=1.5, gutter_tolerance_pt=1.5,
                                       strict=True)
    fig.savefig(OUT / f"{stem}.png", dpi=300)
    fig.savefig(OUT / f"{stem}.tiff", dpi=300)
    fig.savefig(OUT / f"{stem}.svg")
    fig.savefig(OUT / f"{stem}.pdf")
    plt.close(fig)


def plot_combined(index: int, task: dict[str, np.ndarray], trace: np.ndarray,
                  dp_phi: np.ndarray, registration_error: float):
    """One 3x2 figure: overview / detail / top-down, each Left + Right.

    Rows: overview (3/4, full range) · detail (3/4, zoomed to the task) ·
    top-down (elev=90, full range — collapses φ to show the θ–s footprint).
    """
    manifest = json.loads((MANIFOLD / "manifest.json").read_text())
    ns, nt = manifest["n_s"], manifest["n_theta"]
    safe = np.fromfile(MANIFOLD / "safe.bin", dtype="<f4").reshape(ns, nt, 4)
    branch = np.fromfile(MANIFOLD / "branch.bin", dtype="u1").reshape(ns, nt)
    theta = np.linspace(manifest["th_min"], manifest["th_max"], nt) * 180 / np.pi
    slide = np.linspace(manifest["s_min"], manifest["s_max"], ns) * 1000
    TH_full, SL_full = np.meshgrid(theta, slide)

    th_values = task["x"][:, 0] * 180 / np.pi
    s_values = task["x"][:, 1] * 1000
    x_bounds = (max(-50, float(th_values.min()) - 7), min(50, float(th_values.max()) + 7))
    y_bounds = (max(-160, float(s_values.min()) - 12), min(0, float(s_values.max()) + 12))
    th_mask = (theta >= x_bounds[0]) & (theta <= x_bounds[1])
    s_mask = (slide >= y_bounds[0]) & (slide <= y_bounds[1])

    mpl.rcParams.update({"pdf.fonttype": 42, "svg.fonttype": "none"})
    fig = plt.figure(figsize=(11.6, 12.6), dpi=300)
    views = [("overview", False, 23, -62), ("detail", True, 23, -62),
             ("top-down", False, 90, -62)]
    labels = "abcdef"
    for r, (vname, zoom, elev, azim) in enumerate(views):
        if zoom:
            TH, SL = TH_full[np.ix_(s_mask, th_mask)], SL_full[np.ix_(s_mask, th_mask)]
        else:
            TH, SL = TH_full, SL_full
        for arm in range(2):
            k = 2 * r + arm
            ax = fig.add_subplot(3, 2, k + 1, projection="3d")
            lo = safe[:, :, 2 * arm].astype(float)
            hi = safe[:, :, 2 * arm + 1].astype(float)
            finite = np.isfinite(lo) & np.isfinite(hi) & (lo <= hi)
            closed = (branch == 0) & finite      # cycle-consistent self-motion loop
            open_loop = (branch == 1) & finite   # loop-open arc (near wrist singularity)
            if zoom:
                sl = np.ix_(s_mask, th_mask)
                lo, hi = lo[sl], hi[sl]
                closed, open_loop = closed[sl], open_loop[sl]
            ax.plot_surface(TH, SL, np.where(closed, lo, np.nan), color=COLORS["lower"],
                            alpha=0.26, linewidth=0, antialiased=False, shade=False)
            ax.plot_surface(TH, SL, np.where(closed, hi, np.nan), color=COLORS["upper"],
                            alpha=0.26, linewidth=0, antialiased=False, shade=False)
            ax.plot_surface(TH, SL, np.where(open_loop, lo, np.nan), color=COLORS["loop"],
                            alpha=0.30, linewidth=0, antialiased=False, shade=False)
            ax.plot_surface(TH, SL, np.where(open_loop, hi, np.nan), color=COLORS["loop"],
                            alpha=0.30, linewidth=0, antialiased=False, shade=False)
            ax.plot(task["x"][:, 0] * 180 / np.pi, task["x"][:, 1] * 1000,
                    dp_phi[:, arm], color=COLORS["dp"], lw=1.9, alpha=0.97)
            ax.plot(trace[:, 1] * 180 / np.pi, trace[:, 2] * 1000,
                    trace[:, 3 + arm], color=COLORS["rl"], lw=1.9, alpha=0.98)
            ax.scatter([task["x"][0, 0] * 180 / np.pi], [task["x"][0, 1] * 1000],
                       [dp_phi[0, arm]], color=COLORS["dp"], s=24, marker="o",
                       depthshade=False, edgecolor="k", linewidths=0.3)
            ax.scatter([task["x"][-1, 0] * 180 / np.pi], [task["x"][-1, 1] * 1000],
                       [dp_phi[-1, arm]], color=COLORS["dp"], s=16, marker="s",
                       depthshade=False, edgecolor="k", linewidths=0.3)
            ax.scatter([trace[0, 1] * 180 / np.pi], [trace[0, 2] * 1000],
                       [trace[0, 3 + arm]], color=COLORS["rl"], s=24, marker="o",
                       depthshade=False, edgecolor="k", linewidths=0.3)
            ax.scatter([trace[-1, 1] * 180 / np.pi], [trace[-1, 2] * 1000],
                       [trace[-1, 3 + arm]], color=COLORS["rl"], s=16, marker="s",
                       depthshade=False, edgecolor="k", linewidths=0.3)
            ax.set(xlabel="θ (°)", ylabel="s (mm)",
                   zlabel=f"φ{'L' if arm == 0 else 'R'} (rad)",
                   xlim=x_bounds if zoom else (-50, 50),
                   ylim=y_bounds if zoom else (-160, 0),
                   zlim=tuple(manifest["phi_grid"]))
            ax.set_xticks([-10, 10, 30] if zoom else [-40, 0, 40])
            ax.set_yticks([-130, -100, -70] if zoom else [-160, -80, 0])
            ax.set_zticks([-0.3, 0, 0.3])
            ax.set_title(f"{labels[k]}  {'Left' if arm == 0 else 'Right'} arm — {vname}",
                         loc="left", weight="bold", pad=0, fontsize=8.5)
            ax.view_init(elev=elev, azim=azim)
            ax.set_box_aspect((1.3, 1.2, 0.88), zoom=0.9)
            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.grid(False)
    handles = [Line2D([0], [0], color=COLORS["dp"], lw=2.5,
                      label="DP (registered to LUT phase)"),
               Line2D([0], [0], color=COLORS["rl"], lw=2.5,
                      label=f"RLPD (checkpoint {CHECKPOINT_STEP:,})"),
               Line2D([0], [0], color=COLORS["upper"], lw=7, alpha=0.40,
                      label="safe φ corridor [φ$_{min}$, φ$_{max}$]"),
               Line2D([0], [0], color=COLORS["loop"], lw=7, alpha=0.40,
                      label="loop-open φ arc (near wrist singularity)")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.975),
               ncol=2, frameon=False, fontsize=8)
    fig.suptitle("Bimanual redundancy manifold: overview · detail · top-down",
                 y=0.998, fontsize=12, weight="bold")
    fig.text(0.5, 0.012,
             f"DP task {index:04d} · registration error {registration_error:.4f} rad · ○ start / ■ end · "
             "blue = cycle-consistent loop, purple = loop-open arc (both valid) · top-down collapses φ (shows θ–s footprint)",
             ha="center", fontsize=7, color="#4b5563")
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.035, top=0.93,
                        wspace=0.02, hspace=0.24)
    stem = "manifold_dp_rl_v2_combined"
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--reuse-trace", action="store_true",
                    help="replot from the saved rlpd_trace.csv (skips the rollout)")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    if args.reuse_trace:
        # Reconstruct the exact same arrays the original rollout produced,
        # without re-loading the checkpoint / re-running the episode.
        index = 30  # from figure_metadata.json (lowest-registration-error complete rollout)
        task = load_task(index)
        trace = pd.read_csv(OUT / "rlpd_trace.csv").to_numpy(dtype=float)
        with np.load(REGISTRATION / f"traj_{index:04d}.npz") as registered:
            dp_phi = np.asarray(registered["phi_representation"], dtype=float)
            registration_error = float(np.nanmax(registered["q_error_representation"]))
        attempts = []  # not re-run; see figure_metadata.json
    else:
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

    plot(index, task, trace, dp_phi, registration_error, view="overview")
    plot(index, task, trace, dp_phi, registration_error, view="detail")
    plot(index, task, trace, dp_phi, registration_error, view="top-down")
    plot_combined(index, task, trace, dp_phi, registration_error)
    (OUT / "figure_metadata_v2.json").write_text(json.dumps({
        "task_index": index, "checkpoint_step": CHECKPOINT_STEP,
        "checkpoint_run": CHECKPOINT_ROOT.name,
        "dp_phase": "representation-mode registration to LUT coordinate",
        "max_dp_joint_reconstruction_error_rad": registration_error,
        "rl_termination": "end", "rl_steps": len(trace) - 1,
        "attempts": attempts,
        "interpretation": "DP curve is a registered approximation; RL curve is an actual deterministic environment rollout. LUT safe bounds do not certify physical clearance.",
        "v2_changes": "Okabe-Ito colours, on-axis labels, start/end markers, nature style; combined 3x2 figure (overview/detail/top-down)",
    }, indent=2) + "\n")
    print(f"Saved manifold figure (v2) for task {index:04d}; RL steps={len(trace)-1}; "
          f"DP registration error={registration_error:.5f} rad")


if __name__ == "__main__":
    main()

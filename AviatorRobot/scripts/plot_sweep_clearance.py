#!/usr/bin/env python3
"""Summary plots for the inner-gap sweep + joint-margin ablation.

Reads sweep_summary.csv (produced by sweep_clearance.py) and renders:

  sweep_static_envelope.png   d*_static and d_base minima vs inner gap, with the
                              d_safe line marking the geometric-infeasibility
                              crossing (where d*_static < d_safe).
  sweep_executability.png     P_succ vs gap per task (B3 continuous executability).
  sweep_margin_ablation.png   roll_neg P_succ / d_b3_min vs q_margin_target,
                              separating "conservative margin" from "true
                              redundancy exhaustion".
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TASKS = [("roll_pos", r"roll $+\theta$", "tab:red", "o"),
         ("roll_neg", r"roll $-\theta$", "tab:blue", "s"),
         ("pull",     r"pull $s$",        "tab:green", "^"),
         ("combined", r"combined",        "tab:purple", "d")]
D_SAFE = 0.005
G_SAFE = 0.5022   # d*_static^rho == d_safe  (redundancy-safe boundary)
G_GEOM = 0.4922   # d*_static^rho == 0       (rigid-grasp geometric boundary)


def load(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    # genfromtxt returns a 0-d/1-d structured array depending on row count; normalize.
    return np.atleast_1d(rows)


def main(sweepdir):
    sweepdir = Path(sweepdir)
    d = load(sweepdir / "sweep_summary.csv")

    # ---- figure 1: static envelope + baseline penetration --------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    ax = axes[0]
    for tag, label, color, mk in TASKS:
        m = d["task"] == tag
        g = d["gap_inner"][m]
        ax.plot(g, d["d_static_min"][m] * 1000, color=color, marker=mk, label=label)
    ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.2)
    ax.text(0.482, D_SAFE * 1000 + 0.2, r"$d_{\rm safe}=5$ mm", fontsize=8, va="bottom")
    ax.axvline(G_SAFE, color="gray", ls="--", lw=0.8)
    ax.axvline(G_GEOM, color="red", ls="-.", lw=0.8, alpha=0.6)
    ax.set_xlabel("inner wall gap [m]")
    ax.set_ylabel(r"$d^*_{\rm static,min}$ [mm]")
    ax.set_title(r"Static clearance envelope (B1) vs gap")
    ax.invert_xaxis()
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3)

    ax = axes[1]
    for tag, label, color, mk in TASKS:
        m = d["task"] == tag
        ax.plot(d["gap_inner"][m], d["d_base_min"][m] * 1000, color=color, marker=mk, label=label)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlabel("inner wall gap [m]")
    ax.set_ylabel(r"$d_{\rm base,min}$ [mm]")
    ax.set_title("Baseline (B0 continuation IK) minimum clearance")
    ax.invert_xaxis()
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3)
    fig.suptitle("Manipulation authority collapse as cockpit clearance tightens")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(sweepdir / "sweep_static_envelope.png", dpi=150)
    plt.close(fig)

    # ---- figure 2: executability --------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 4.4))
    for tag, label, color, mk in TASKS:
        m = d["task"] == tag
        ax.plot(d["gap_inner"][m], d["P_succ"][m], color=color, marker=mk, label=label)
    ax.axvline(G_SAFE, color="gray", ls="--", lw=0.8)
    ax.axvline(G_GEOM, color="red", ls="-.", lw=0.8, alpha=0.6)
    ax.set_xlabel("inner wall gap [m]")
    ax.set_ylabel(r"$P_{\rm succ}$ (B3 least-intervention)")
    ax.set_ylim(-0.05, 1.08)
    ax.set_title("Continuous executable recovery rate vs gap")
    ax.invert_xaxis()
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(sweepdir / "sweep_executability.png", dpi=150)
    plt.close(fig)

    # ---- figure 3: joint-margin ablation ------------------------------------
    m = d["task"] == "roll_neg"
    fig, ax = plt.subplots(figsize=(7, 4.4))
    ax2 = ax.twinx()
    ax.plot(d["q_margin"][m], d["P_succ"][m], "o-", color="tab:blue", label=r"$P_{\rm succ}$")
    ax.set_xlabel(r"joint-limit margin $m_q$ [rad]")
    ax.set_ylabel(r"$P_{\rm succ}$ (roll $-\theta$)", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2.plot(d["q_margin"][m], d["d_b3_min"][m] * 1000, "s--", color="tab:red",
             label=r"$d_{\rm B3,min}$ [mm]")
    ax2.set_ylabel(r"$d_{\rm B3,min}$ [mm]", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")
    ax.set_xticks(sorted(set(d["q_margin"][m])))
    ax.set_title("Joint-margin ablation on roll $-$\\theta @ gap=0.54")
    ax.set_ylim(0.95, 1.02)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(sweepdir / "sweep_margin_ablation.png", dpi=150)
    plt.close(fig)

    print("wrote", sweepdir / "sweep_static_envelope.png")
    print("wrote", sweepdir / "sweep_executability.png")
    print("wrote", sweepdir / "sweep_margin_ablation.png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/aviator-sweep")

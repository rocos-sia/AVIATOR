#!/usr/bin/env python3
"""Plot the B0 / B1 (d*_static) / B3 (least-intervention) clearance profiles.

Reads trajectory_*.csv written by aviator_clearance_trajectory and produces, in the
same directory:

  clearance_trajectory.png    d_base vs d_static vs d_b3 (d_safe line) per task
  clearance_intervention.png  per-knot intervention ||qdot - qdot_nom|| + joint margin

The three-regime story: d_base <= d_executable (B3) <= d*_static, with B3's goal
d_executable >= d_safe (5 mm) achieved with the smallest self-motion (least intervention).
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TASKS = [("roll_pos", r"Task 1: roll $+\theta$ ($s=0$)"),
         ("roll_neg", r"Task 2: roll $-\theta$ ($s=0$)"),
         ("pull",     r"Task 3: pull $s$ ($\theta=0$)"),
         ("combined", r"Task 4: combined roll+pull")]

D_SAFE = 0.005


def xaxis(tag, rows):
    return rows["theta"] if "pull" not in tag else rows["s"]


def xlabel(tag):
    return r"$\theta$ [rad]" if "pull" not in tag else "$s$ [m]"


def main(outdir):
    outdir = Path(outdir)

    # ---- clearance profiles --------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(15, 8))
    for ax, (tag, label) in zip(axes.flat, TASKS):
        rows = np.genfromtxt(outdir / f"trajectory_{tag}.csv", delimiter=",", names=True)
        t = xaxis(tag, rows)
        ax.axhline(0, color="gray", lw=0.7, ls="--")
        ax.axhline(D_SAFE, color="#2ca02c", lw=0.9, ls=":", alpha=0.7)
        ax.text(t[-1], D_SAFE, r"$d_{\rm safe}$", color="#2ca02c", fontsize=7, va="bottom")
        ax.plot(t, rows["d_base"], label=r"$d_{\rm base}$ (B0)", color="#d62728", lw=1.2)
        ax.plot(t, rows["d_static"], label=r"$d^*_{\rm static}$ (B1)", color="#1f77b4", lw=0.9, ls=":")
        ax.plot(t, rows["d_b3"], label=r"$d_{\rm B3}$ (least-interv.)", color="#ff7f0e", lw=1.4)
        ax.set_xlabel(xlabel(tag)); ax.set_ylabel("wall clearance [m]")
        ax.set_title(label)
        ax.legend(fontsize=8, loc="lower left")
        ax.set_ylim(-0.02, 0.012)
    fig.suptitle("Clearance along task trajectory @ gap=0.54 (B0 vs d*_static vs least-intervention B3)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(outdir / "clearance_trajectory.png", dpi=150)
    plt.close(fig)

    # ---- intervention + joint margin ----------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(15, 8))
    for ax, (tag, label) in zip(axes.flat, TASKS):
        rows = np.genfromtxt(outdir / f"trajectory_{tag}.csv", delimiter=",", names=True)
        t = xaxis(tag, rows)
        ax.set_title(label)
        ax.set_xlabel(xlabel(tag))
        ax.plot(t, rows["interv"], label=r"$\|\dot q-\dot q_{\rm nom}\|$", color="#9467bd", lw=1.0)
        ax.set_ylabel(r"intervention [rad/s]", color="#9467bd")
        ax.tick_params(axis="y", labelcolor="#9467bd")
        ax2 = ax.twinx()
        ax2.plot(t, rows["q_margin"], label=r"$q$ margin", color="#8c564b", lw=0.8, ls="--")
        ax2.axhline(0.03, color="#8c564b", lw=0.7, ls=":", alpha=0.6)
        ax2.set_ylabel(r"joint margin [rad]", color="#8c564b")
        ax2.tick_params(axis="y", labelcolor="#8c564b")
    fig.suptitle("Intervention effort and joint-limit margin (B3 least-intervention)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(outdir / "clearance_intervention.png", dpi=150)
    plt.close(fig)

    # ---- constraint-activation strip ----------------------------------------
    # active: 0 none, 1 clearance-bound, 2 dynamic-bound, 3 joint-limit-bound.
    cmap = matplotlib.colors.ListedColormap(
        ["#ffffff", "#2ca02c", "#ff7f0e", "#d62728"])
    fig, axes = plt.subplots(2, 2, figsize=(15, 6))
    for ax, (tag, label) in zip(axes.flat, TASKS):
        rows = np.genfromtxt(outdir / f"trajectory_{tag}.csv", delimiter=",", names=True)
        t = xaxis(tag, rows)
        a = np.atleast_1d(rows["active"])
        ax.set_title(label)
        ax.set_xlabel(xlabel(tag))
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(["none", "clearance", "dynamic", "joint-limit"], fontsize=7)
        ax.set_ylim(-0.5, 3.5)
        for i in range(len(t) - 1):
            ax.axvspan(t[i], t[i + 1], color=cmap(int(a[i])), lw=0)
    fig.suptitle("Constraint activation along trajectory (B3 least-intervention)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(outdir / "clearance_activation.png", dpi=150)
    plt.close(fig)

    print("wrote", outdir / "clearance_trajectory.png")
    print("wrote", outdir / "clearance_intervention.png")
    print("wrote", outdir / "clearance_activation.png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")

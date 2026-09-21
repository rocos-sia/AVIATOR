#!/usr/bin/env python3
"""Constraint-release ladder (R0→R1→R2a/R2b→R3) plot.

Reads one release_ablation.csv per gap (produced by
`aviator_clearance_trajectory ... release_ablation`) and renders the
"DOF-value curve": does releasing wrist orientation (φ → +β → full orientation)
move the geometric boundary g_geom below the rigid-grasp ρ-only value 0.4921?

Answer (measured): NO. d*_{R1} = d*_{R2a} = d*_{R2b} = d*_{R3} = 0.5·g − 0.2461,
so every release level shares g_geom = 0.4921. Orientation freedom is not the
binding leverage; the clearance-limiting geometry (elbow/forearm) is fixed by
the wheel pose and the 1-D self-motion ρ alone.

Two panels:
  left  — d* vs inner gap for each level (binding arm roll_neg side 0);
          all released levels collapse onto the continuation-ρ line.
  right — incremental d* gain per release level at the tightest gap.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_SAFE = 0.005
G_GEOM = 0.49211
LEVELS = [("R0", r"$3P{+}3R$ (rigid, $\rho$)"),
          ("R1", r"$3P{+}2R$ ($+\varphi$)"),
          ("R2a", r"$3P{+}2R$ ($+\beta_r$)"),
          ("R2b", r"$3P{+}2R$ ($+\beta_n$)"),
          ("R3", r"$3P$ (orientation free)")]
COLORS = {"R0": "tab:blue", "R1": "tab:red", "R2a": "tab:orange",
          "R2b": "tab:green", "R3": "tab:purple"}
MARKERS = {"R0": "o", "R1": "s", "R2a": "^", "R2b": "v", "R3": "D"}


def load(path, task="roll_neg", side=0):
    rows = {}
    with open(path) as f:
        next(f)  # header
        for line in f:
            p = line.rstrip("\n").split(",")
            if p[0] != task or int(p[1]) != side:
                continue
            rows[p[2]] = float(p[3])
    return rows


def main(abdir):
    abdir = Path(abdir)
    gaps = sorted({p.parent.name.split("_")[-1] for p in abdir.glob("fine_*/release_ablation.csv")},
                  key=float)
    g = np.array([float(x) for x in gaps])
    d = {lvl: np.array([load(abdir / f"fine_{x}" / "release_ablation.csv")[lvl]
                        for x in gaps]) for lvl, _ in LEVELS}

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))

    # left: d* vs gap ---------------------------------------------------------
    ax = axes[0]
    gg = np.linspace(float(gaps[0]), float(gaps[-1]), 50)
    ax.plot(gg, (0.5 * gg - 0.246053) * 1000, "k--", lw=1.0,
            label=r"continuation $\rho$ ($0.5g{-}0.2461$)")
    for lvl, label in LEVELS:
        ax.plot(g, d[lvl] * 1000, MARKERS[lvl] + "-", color=COLORS[lvl], lw=1.4, label=label)
    ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.1)
    ax.axvline(G_GEOM, color="red", ls="-.", lw=0.9, alpha=0.6)
    ax.text(G_GEOM - 0.0003, 6.0, r"$g_{\rm geom}{=}0.4921$", color="red",
            fontsize=8, rotation=90, va="bottom")
    ax.set_xlabel("inner wall gap [m]")
    ax.set_ylabel(r"$d^*$ [mm]  (roll $-\theta$, side 0 — binding arm)")
    ax.set_title("Constraint-release ladder: all release levels share one boundary")
    ax.invert_xaxis()
    ax.legend(fontsize=7, loc="lower left")
    ax.grid(alpha=0.3)

    # right: incremental value at the tightest gap ----------------------------
    ax = axes[1]
    gt = float(gaps[0])
    vals = [d[lvl][0] * 1000 for lvl, _ in LEVELS]
    labs = [lbl for _, lbl in LEVELS]
    colors = [COLORS[lvl] for lvl, _ in LEVELS]
    ax.bar(range(len(vals)), vals, color=colors, alpha=0.85)
    for i, v in enumerate(vals):
        ax.text(i, v + (0.12 if v < 0 else -0.22), f"{v:.2f}", ha="center",
                va="bottom" if v < 0 else "top", fontsize=8)
    ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.1)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xticks(range(len(vals)))
    ax.set_xticklabels(labs, fontsize=7.5)
    ax.set_ylabel(r"$d^*$ [mm]  @ gap=" + f"{gt}")
    ax.set_title("DOF-value curve: wrist orientation adds <0.02 mm")
    ax.grid(alpha=0.3, axis="y")

    fig.suptitle("Orientation relaxation is not the main leverage at the geometric boundary")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out = abdir / "release_ablation.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/aviator-sweep/release-abl")

#!/usr/bin/env python3
"""Render the D_g feasibility-atlas heatmap for topology sanity check.

Three layers on one main panel (as requested):
  - background: D_g(θ,s) in mm, diverging colormap centred at 0 (red = penetration,
    blue = clearance); the IK-fail sentinel cells are shown as grey + X markers
  - two contours: D_g = 0 (collision boundary) and D_g = d_safe (safety margin)
  - the no-IK points overlaid as distinct symbols
A side panel shows the feasible fraction vs s-band (exposes the mid-pull bottleneck).

Run with the serl_clean env (numpy/matplotlib).
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SENTINEL = -0.5  # D_g <= this means baseline IK failed (stored as -1.0)


def load(atlas_dir):
    d = Path(atlas_dir)
    meta = json.loads((d / "atlas.json").read_text())
    g = meta["grid"]
    n_theta, n_s = g["theta"]["n"], g["s"]["n"]
    th = np.linspace(g["theta"]["min"], g["theta"]["max"], n_theta) / np.pi * 180.0
    s = np.linspace(g["s"]["min"], g["s"]["max"], n_s) * 1e3
    Dg = np.fromfile(d / "atlas_Dg.bin", dtype=np.float32).reshape(n_s, n_theta)
    Feasible = np.fromfile(d / "atlas_feasible.bin", dtype=np.uint8).reshape(n_s, n_theta)
    return meta, th, s, Dg, Feasible


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--atlas", required=True, help="dir holding atlas_Dg.bin/atlas_feasible.bin/atlas.json")
    ap.add_argument("--out", required=True, help="output PNG path")
    args = ap.parse_args()

    meta, th, s, Dg, Feasible = load(args.atlas)
    d_safe = meta.get("d_safe", 0.005) * 1e3  # mm
    n_s, n_theta = Dg.shape

    valid = Dg > SENTINEL          # IK succeeded
    Dm = np.where(valid, Dg * 1e3, np.nan)  # mm; NaN = no-IK sentinel

    fig = plt.figure(figsize=(11.5, 5.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[3.1, 1.0], wspace=0.14)

    # ---- main panel ----
    ax = fig.add_subplot(gs[0])
    vmax = max(float(np.nanmax(np.abs(Dm))), d_safe + 1.0)
    cmap = plt.cm.RdBu.copy()          # low (penetration) -> red, high -> blue
    cmap.set_bad((0.80, 0.80, 0.80))   # grey = no-IK cells
    im = ax.pcolormesh(th, s, Dm, cmap=cmap, vmin=-vmax, vmax=vmax, shading="nearest")

    c0 = ax.contour(th, s, Dm, levels=[0.0], colors="black", linewidths=1.4, linestyles="-")
    c5 = ax.contour(th, s, Dm, levels=[d_safe], colors="lime", linewidths=1.4, linestyles="--")
    ax.clabel(c0, fmt="D_g = 0 mm", inline=True, fontsize=8)
    ax.clabel(c5, fmt=f"D_g = {d_safe:g} mm", inline=True, fontsize=8)

    iy, ix = np.where(~valid)
    ax.plot(th[ix], s[iy], "x", color="purple", ms=3.0, mew=1.0, lw=0.7,
            label=f"no IK ({len(ix)} pts)")

    ax.set_xlabel("roll θ (deg)")
    ax.set_ylabel("pull s (mm)")
    ax.set_title(f"D_g(θ,s) static max clearance [mm]   (gap = {meta['gap_inner']:g} m)")
    ax.grid(alpha=0.2, lw=0.4)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    cb = fig.colorbar(im, ax=ax, pad=0.01, label="D_g [mm]")

    # ---- side panel: feasible fraction per s-band ----
    ax2 = fig.add_subplot(gs[1])
    frac = Feasible.mean(axis=1) * 100.0
    ax2.plot(frac, s, "o-", ms=3, lw=1.0, color="#1f77b4")
    ax2.axvline(Feasible.mean() * 100.0, color="gray", ls=":", lw=1.0)
    ax2.set_xlabel("feasible fraction (%)")
    ax2.set_ylabel("pull s (mm)")
    ax2.set_title("feasibility vs s-band")
    ax2.set_xlim(0, 100)
    ax2.grid(alpha=0.3, lw=0.4)

    fig.tight_layout()
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

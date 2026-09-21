#!/usr/bin/env python3
"""Render the AVIATOR authority failure-mode map as PNG heatmaps.

Reads authority_grid.csv + metadata.json written by aviator_authority_map and
produces, in the same directory:

  authority_a_max.png    capability envelope A_kin (4 task directions)
  authority_a_exec.png   executed authority A_exec along the baseline
  authority_delta.png    DeltaA = A_kin - A_exec (unused capability)
  authority_ratio.png    R_A = A_exec / A_kin (authority utilization ratio)
  diagnostics.png        d_base / d_best / R_d / C / q_margin / sigma_min(J)

The failure of interest is *clearance*, not authority: A_kin is kinematic (wall
agnostic), so the real limit is how close the baseline elbow comes to the wall
(d_base) versus what redundancy can recover (d_best, R_d = d_best - d_base) and
a 3-class feasibility C = {0 baseline-feasible, 1 redundancy-recoverable,
2 geometrically-infeasible}.
"""
import csv
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DIRS = [("a_max_theta_p", "turn +$\\theta$"),
        ("a_max_theta_m", "turn $-\\theta$"),
        ("a_max_s_p", "push $+s$"),
        ("a_max_s_m", "pull $-s$")]
EXEC = [("a_exec_theta_p", "turn +$\\theta$"),
        ("a_exec_theta_m", "turn $-\\theta$"),
        ("a_exec_s_p", "push $+s$"),
        ("a_exec_s_m", "pull $-s$")]
TINY = 1e-9


def load(rows_dir):
    rows_dir = Path(rows_dir)
    rows = list(csv.DictReader((rows_dir / "authority_grid.csv").open()))
    meta = json.loads((rows_dir / "metadata.json").read_text())
    n_s, n_theta = meta["grid"]["n_s"], meta["grid"]["n_theta"]
    tr = meta["grid"]["theta_range"]
    sr = meta["grid"]["s_range"]

    def field(key):
        return np.array([float(r[key]) for r in rows]).reshape(n_s, n_theta)

    data = {key: field(key) for key in
            ["ik_ok", "n_valid", "d_base", "d_best", "R_d", "C", "q_margin", "sigma_min"] +
            [k for k, _ in DIRS + EXEC] +
            ["a_min_theta_p", "a_min_theta_m", "a_min_s_p", "a_min_s_m"]}
    return data, tr, sr, meta, rows_dir


def pcol(ax, z, tr, sr, cmap, label, vmin=None, vmax=None):
    m = ax.imshow(z, origin="lower", aspect="auto", cmap=cmap,
                  extent=[tr[0], tr[1], sr[0], sr[1]], vmin=vmin, vmax=vmax)
    ax.set_xlabel("$\\theta$ [rad]")
    ax.set_ylabel("$s$ [m]")
    ax.set_title(label)
    plt.colorbar(m, ax=ax, fraction=0.046, pad=0.04)
    return m


def main(rows_dir):
    data, tr, sr, meta, out = load(rows_dir)

    # ---- capability envelope A_max -------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    vals = np.array([data[k] for k, _ in DIRS])
    vmax = np.nanpercentile(vals, 98)
    for ax, (k, label) in zip(axes.flat, DIRS):
        pcol(ax, data[k], tr, sr, "magma", f"$A_{{kin}}$: {label}", 0, vmax)
    fig.suptitle("Kinematic authority $A_{kin}(x;v)$ — best over redundancy")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out / "authority_a_max.png", dpi=150)
    plt.close(fig)

    # ---- executed authority A_exec -------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    vals = np.array([data[k] for k, _ in EXEC])
    vmax = np.nanpercentile(vals, 98)
    for ax, (k, label) in zip(axes.flat, EXEC):
        pcol(ax, data[k], tr, sr, "magma", f"A_exec: {label}", 0, vmax)
    fig.suptitle("Executed authority $A_{exec}(x;v)$ — baseline IK continuation")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out / "authority_a_exec.png", dpi=150)
    plt.close(fig)

    # ---- DeltaA = A_max - A_exec ---------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, (k, label) in zip(axes.flat, EXEC):
        dk = k.replace("a_exec", "a_max")
        pcol(ax, data[dk] - data[k], tr, sr, "viridis", f"$\\Delta A$: {label}")
    fig.suptitle("Unused capability $\\Delta A = A_{kin} - A_{exec}$")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out / "authority_delta.png", dpi=150)
    plt.close(fig)

    # ---- R_A = A_exec / A_max ------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, (k, label) in zip(axes.flat, EXEC):
        dk = k.replace("a_exec", "a_max")
        ratio = np.where(data[dk] > TINY, data[k] / np.maximum(data[dk], TINY), np.nan)
        pcol(ax, ratio, tr, sr, "RdYlGn", f"$R_A$: {label}", 0, 1)
    fig.suptitle("Authority utilization ratio $R_A = A_{exec}/A_{kin}$ (red = collapse)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out / "authority_ratio.png", dpi=150)
    plt.close(fig)

    # ---- clearance diagnostics (centerpiece) --------------------------------------
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    # Row 1: the clearance story — baseline gap, best redundancy gap, recovery margin,
    # and the 3-class feasibility map.
    pcol(axes[0, 0], data["d_base"], tr, sr, "coolwarm",
         "$d_{base}$ [m, <0 = baseline collision]", -0.05, 0.05)
    pcol(axes[0, 1], data["d_best"], tr, sr, "coolwarm",
         "$d^{*}$ [m, best over self-motion]", -0.05, 0.05)
    pcol(axes[0, 2], data["R_d"], tr, sr, "viridis",
         "$R_d = d^{*} - d_{base}$ [m, recovery margin]")
    cmap = matplotlib.colors.ListedColormap(["#2ca02c", "#ff7f0e", "#d62728"])
    zc = data["C"].copy()
    zc[data["ik_ok"] != 1] = np.nan
    m = axes[1, 0].imshow(zc, origin="lower", aspect="auto", cmap=cmap,
                          extent=[tr[0], tr[1], sr[0], sr[1]], vmin=-0.5, vmax=2.5)
    axes[1, 0].set_xlabel("$\\theta$ [rad]")
    axes[1, 0].set_ylabel("$s$ [m]")
    axes[1, 0].set_title("feasibility class $C$")
    cb = plt.colorbar(m, ax=axes[1, 0], fraction=0.046, pad=0.04, ticks=[0, 1, 2])
    cb.ax.set_yticklabels(["0 baseline", "1 redundancy", "2 infeasible"])
    # Row 2: supporting metrics.
    pcol(axes[1, 1], data["q_margin"], tr, sr, "viridis", "joint-limit margin [rad]")
    pcol(axes[1, 2], np.log10(np.maximum(data["sigma_min"], 1e-12)), tr, sr, "magma",
         "$\\log_{10}\\sigma_{min}(J_{arm})$")
    fig.suptitle("Clearance diagnostics: baseline vs self-motion gap, recovery margin, feasibility")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out / "diagnostics.png", dpi=150)
    plt.close(fig)

    # ---- clearance summary ---------------------------------------------------------
    reachable = data["ik_ok"] == 1
    c0 = reachable & (data["C"] == 0)
    c1 = reachable & (data["C"] == 1)
    c2 = reachable & (data["C"] == 2)
    print(f"grid {data['ik_ok'].shape[1]}x{data['ik_ok'].shape[0]}: "
          f"reachable={reachable.sum():d} | "
          f"C0 baseline-feasible={c0.sum():d}, "
          f"C1 redundancy-recoverable={c1.sum():d}, "
          f"C2 geometrically-infeasible={c2.sum():d}")
    print("PNGs written to", out)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: plot_authority_map.py OUTPUT_DIRECTORY", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1])

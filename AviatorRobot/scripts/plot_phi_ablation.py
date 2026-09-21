#!/usr/bin/env python3
"""Step-5 φ-ablation plot: d*_ρ (φ=0) vs d*_{ρ+φ} for the binding arm (roll_neg, side 0).

Reads one phi_ablation.csv per gap (produced by `aviator_clearance_trajectory ...
phi_ablation`), and overlays the continuation-seeded d*_ρ envelope from the sweep
(0.5·g − 0.246053) to show that releasing φ recovers the continuation optimum but
does NOT extend the geometric boundary g_geom = 0.4921.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_SAFE = 0.005
G_GEOM = 0.49211
D_RHO_CONT = lambda g: 0.5 * g - 0.246053  # continuation d*_ρ (sweep fit)


def load_one(path, task="roll_neg", side=0):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(",")
            if p[0] != task or int(p[1]) != side:
                continue
            rows.append((float(p[2]), float(p[3]), float(p[4]), float(p[5])))
    return rows[0]


def main(abdir):
    abdir = Path(abdir)
    gaps = sorted({p.parent.name.split("-")[-1] for p in abdir.glob("aviator-phi-ablation-*/phi_ablation.csv")},
                  key=float)
    g = np.array([float(x) for x in gaps])
    d_rho = np.array([load_one(abdir / f"aviator-phi-ablation-{x}" / "phi_ablation.csv")[0] for x in gaps])
    d_rp = np.array([load_one(abdir / f"aviator-phi-ablation-{x}" / "phi_ablation.csv")[1] for x in gaps])

    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    gg = np.linspace(g.min(), g.max(), 50)
    ax.plot(gg, D_RHO_CONT(gg) * 1000, "k--", lw=1.0,
            label=r"$d^*_{\rho}$ (continuation, $0.5g{-}0.2461$)")
    ax.plot(g, d_rho * 1000, "o-", color="tab:blue", lw=1.4,
            label=r"$d^*_{\rho}$ (home seed, $\varphi{=}0$)")
    ax.plot(g, d_rp * 1000, "s-", color="tab:red", lw=1.4,
            label=r"$d^*_{\rho+\varphi}$ (home seed, 2-D null space)")
    ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.1)
    ax.axvline(G_GEOM, color="red", ls="-.", lw=0.9, alpha=0.6)
    ax.text(G_GEOM - 0.001, 6.5, r"$g_{\rm geom}$", color="red", fontsize=8,
            rotation=90, va="bottom")
    ax.set_xlabel("inner wall gap [m]")
    ax.set_ylabel("clearance [mm]  (roll $-$θ, side 0 — binding arm)")
    ax.set_title(r"Step-5 φ ablation: $\varphi$ recovers, but does not extend, the static envelope")
    ax.invert_xaxis()
    ax.legend(fontsize=7.5, loc="lower left")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = abdir / "phi_ablation.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp")

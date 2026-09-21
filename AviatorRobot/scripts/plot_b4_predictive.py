#!/usr/bin/env python3
"""Milestone 2: B4 (predictive) vs B3 (reactive) vs B0 (baseline).

Reads online_summary_b0.csv / online_summary_b3.csv / online_summary_b4.csv and
plots the dynamic f-sweep: minimum clearance d_min vs command frequency f per
profile, with the d_safe = 5 mm line. The headline is the maximum safe tracking
frequency f_max (largest f with d_min >= d_safe and no rate saturation): B4's
short-horizon lookahead should push f_max above B3's.

Three panels:
  left   — d_min [mm] vs f for sine
  middle — d_min [mm] vs f for roll_pull
  right  — f_max [Hz] bar per method (max of sine/roll_pull), annotated
"""
import sys
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_SAFE = 0.005
COLORS = {"b0": "tab:blue", "b3": "tab:red", "b4": "tab:green"}
LINESTYLE = {"b0": "--", "b3": "-", "b4": "-."}
MARKERS = {"sine": "o", "roll_pull": "s"}


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["profile"] != "step":
                rows.append((r["profile"], float(r["f"]), float(r["d_min"]),
                             float(r["safe_frac"]), float(r["rate_sat"])))
    return rows


def fmax(store, method):
    """Largest f with d_min >= d_safe (across both profiles)."""
    fs = sorted({f for (p, f) in store if store[(p, f)][0] >= D_SAFE})
    return fs[-1] if fs else 0.0


def main(d0, d3, d4, out):
    stores = {}
    for m, path in (("b0", d0), ("b3", d3), ("b4", d4)):
        if not Path(path).exists():
            print(f"skip missing {path}")
            continue
        stores[m] = {(p, f): (d, sf, rs) for p, f, d, sf, rs in load(path)}

    methods = [m for m in ("b0", "b3", "b4") if m in stores]
    profiles = ["sine", "roll_pull"]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    for ax, p in zip(axes[:2], profiles):
        for m in methods:
            store = stores[m]
            fs = sorted({f for (pp, f) in store if pp == p})
            dd = [store[(p, f)][0] * 1000 for f in fs]
            ax.plot(fs, dd, MARKERS[p] + LINESTYLE[m], color=COLORS[m], lw=1.5,
                    label=f"{m.upper()}")
        ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.1, label=r"$d_{\rm safe}=5$ mm")
        ax.axhline(0, color="k", lw=0.8)
        ax.fill_between([0, 1.05], -1, 0, color="tab:red", alpha=0.06, label="penetration")
        ax.set_xlabel("command frequency $f$ [Hz]")
        ax.set_ylabel(r"$d_{\min}$ [mm]")
        ax.set_title(p.replace("_", " / "))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="upper right")

    # right: f_max bar
    ax = axes[2]
    xs = range(len(methods))
    vals = [fmax(stores[m], m) for m in methods]
    bars = ax.bar(xs, vals, color=[COLORS[m] for m in methods], alpha=0.85)
    for x, v in zip(xs, vals):
        ax.text(x, v + 0.015, f"{v:.2f}", ha="center", va="bottom", fontsize=10)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([m.upper() for m in methods])
    ax.set_ylabel(r"$f_{\max}$ [Hz]  ($d_{\min}\geq d_{\rm safe}$)")
    ax.set_title("Max safe tracking frequency")
    ax.set_ylim(0, max(vals) * 1.25 + 0.1)
    ax.grid(alpha=0.3, axis="y")

    fig.suptitle("B4 predictive vs B3 reactive @ gap 0.54 m: does lookahead raise $f_{\\max}$?")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out = Path(out)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("wrote", out)
    for m in methods:
        print(f"  {m}: f_max = {fmax(stores[m], m):.2f} Hz")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3],
         sys.argv[4] if len(sys.argv) > 4 else "b4_predictive.png")

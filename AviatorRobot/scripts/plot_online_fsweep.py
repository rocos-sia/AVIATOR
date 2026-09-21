#!/usr/bin/env python3
"""Online dynamic phase: B0 (continuation IK) vs B3 (reactive nullspace reshaping).

Reads online_summary_b0.csv / online_summary_b3.csv (produced by
`aviator_clearance_trajectory ... online_b0|online_b3`) and plots the dynamic
f-sweep: minimum clearance d_min vs command frequency f for each profile.

Headline (Milestone 1): at the default gap 0.54 m, B0 penetrates the wall at
EVERY frequency (d_min < 0), while B3's nullspace reshaping holds d_min ~ d_safe
at low f and degrades monotonically as f rises ("reactive too late"). This
motivates B4 (short-horizon predictive reshaping).

Two panels:
  left  — d_min [mm] vs f, B0 vs B3, per profile (sine / roll_pull), with the
          d_safe = 5 mm line and a zero-penetration line.
  right — safe fraction (share of knots with d >= d_safe) vs f, B0 vs B3.
"""
import sys
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_SAFE = 0.005
COLORS = {"b0": "tab:blue", "b3": "tab:red"}
MARKERS = {"sine": "o", "roll_pull": "s"}
LINESTYLE = {"b0": "--", "b3": "-"}


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["profile"] != "step":
                rows.append((r["profile"], float(r["f"]), float(r["d_min"]),
                             float(r["safe_frac"])))
    return rows


def main(d0, d3, out):
    b0 = {(p, f): (d, sf) for p, f, d, sf in load(d0)}
    b3 = {(p, f): (d, sf) for p, f, d, sf in load(d3)}
    profiles = ["sine", "roll_pull"]
    fs = {p: sorted({f for (pp, f) in b0 if pp == p}) for p in profiles}

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))

    # left: d_min vs f -------------------------------------------------------
    ax = axes[0]
    for p in profiles:
        for m in ("b0", "b3"):
            store = b0 if m == "b0" else b3
            dd = [store[(p, f)][0] * 1000 for f in fs[p]]
            ax.plot(fs[p], dd, MARKERS[p] + LINESTYLE[m], color=COLORS[m], lw=1.4,
                    label=f"{m.upper()} {p}")
    ax.axhline(D_SAFE * 1000, color="k", ls=":", lw=1.1, label=r"$d_{\rm safe}=5$ mm")
    ax.axhline(0, color="k", lw=0.8)
    ax.fill_between([min(fs["sine"]), max(fs["sine"])], -0.5, 0,
                    color="tab:red", alpha=0.06, label="penetration")
    ax.set_xlabel("command frequency $f$ [Hz]")
    ax.set_ylabel(r"$d_{\min}$ [mm]")
    ax.set_title("B0 penetrates at every $f$; B3 rescues at low $f$, then degrades")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(alpha=0.3)

    # right: safe fraction vs f ---------------------------------------------
    ax = axes[1]
    for p in profiles:
        for m in ("b0", "b3"):
            store = b0 if m == "b0" else b3
            sf = [store[(p, f)][1] for f in fs[p]]
            ax.plot(fs[p], sf, MARKERS[p] + LINESTYLE[m], color=COLORS[m], lw=1.4,
                    label=f"{m.upper()} {p}")
    ax.set_xlabel("command frequency $f$ [Hz]")
    ax.set_ylabel("safe fraction  ($d\\geq d_{\\rm safe}$)")
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("Safe-knot share drops with $f$ for B3, ~constant for B0")
    ax.legend(fontsize=7, loc="lower left")
    ax.grid(alpha=0.3)

    fig.suptitle("Online dynamic envelope @ gap 0.54 m: redundancy reshapes clearance, "
                 "but reactive B3 is too slow at high $f$")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out = Path(out)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "online_fsweep.png")

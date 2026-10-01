"""Plot the first generated mixed steering path in each speed group."""
import argparse
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_field import interpolate_q


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--directory", type=Path, required=True)
    ap.add_argument("--field", type=Path, default=Path("../outputs/static_field_audit/old_fixed_q_field_00_10.npz"))
    args = ap.parse_args()
    manifest = json.loads((args.directory / "manifest.json").read_text())
    lookup = ManifoldLookup("data/aviator/manifold_phi_stale_pitch-10deg")
    with np.load(args.field) as f:
        grid = f["q"]
    fig, axes = plt.subplots(5, 3, figsize=(12, 10), sharex="col", constrained_layout=True)
    for col, speed in enumerate((.6, 1., 1.5)):
        name, row = next((name, row) for name, row in manifest["trajectories"].items()
                         if row["speed_cap"] == speed and row["mode"] == "steering_with_slide")
        with np.load(args.directory / "trajs" / (name + ".npz")) as f:
            t, x, xd = f["t"], f["x"], f["xdot"]
        q = interpolate_q(lookup, grid, x)
        traces = [(t, x[:, 0]), (t, xd[:, 0]), (t, x[:, 1] * 1000),
                  (t[1:], np.max(abs(np.diff(q, axis=0) / .01), axis=1)),
                  (t[2:], np.max(abs(np.diff(q, n=2, axis=0) / .01**2), axis=1))]
        for j, (time, y) in enumerate(traces):
            ax = axes[j, col]
            ax.plot(time, y, color="#0072B2", linewidth=1.2)
            for event in row["events"]:
                if event["kind"] == "hold":
                    ax.axvspan(event["start"], event["end"], color="gray", alpha=.12)
            ax.grid(alpha=.2)
            ax.spines[["top", "right"]].set_visible(False)
        for y in (-speed, speed):
            axes[1, col].axhline(y, color="#D55E00", linestyle="--", linewidth=1)
        for row_index, limit in ((3, 1.5), (4, 10)):
            axes[row_index, col].axhline(limit, color="#D55E00", linestyle="--", linewidth=1)
            axes[row_index, col].set_ylim(bottom=0)
        axes[0, col].set_title(f"Speed cap {speed:g} rad/s | {name}", fontsize=11)
        axes[-1, col].set_xlabel("Time (s)")
    for j, label in enumerate(("Wheel angle (rad)", "Wheel speed (rad/s)", "Slide (mm)",
                                "Max joint speed (rad/s)", "Max joint accel. (rad/s²)")):
        axes[j, 0].set_ylabel(label)
    fig.suptitle("Synthetic random steering — frozen 2-D joint table\n"
                 "Gray: pauses; dashed orange: limits; examples selected before audit results", fontsize=13)
    for extension in ("png", "pdf"):
        fig.savefig(args.directory / ("steering_examples." + extension), dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()

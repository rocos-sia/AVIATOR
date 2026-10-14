#!/usr/bin/env python3
"""Create auditable N15 latency decomposition and stop-reason tables."""

from __future__ import annotations

import csv
import json
from pathlib import Path


ROOT = Path("/home/rocos/sia/AVIATOR")
OUT = ROOT / "outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency"


def main() -> int:
    summary = json.loads((OUT / "n15_summary.json").read_text())
    components = {
        "policy_inference": "policy_inference_ns",
        "action_decode": "action_decode_ns",
        "manifold_lookup_and_kinematic_env": "environment_step_ns",
        "endpoint_checker": None,
        "governor": None,
        "measured_unshielded_cycle": "measured_unshielded_cycle_ns",
    }
    decomp = []
    for condition in summary["conditions"]:
        for component, field in components.items():
            stats = condition["timing"].get(field) if field else None
            decomp.append({
                "method": condition["method"], "seed": condition["seed"],
                "component": component,
                "status": "measured" if stats is not None else "not_invoked_in_AviatorManifoldEnv",
                "n": stats["n"] if stats else "",
                "mean_ms": stats["mean_ms"] if stats else "",
                "median_ms": stats["median_ms"] if stats else "",
                "p95_ms": stats["p95_ms"] if stats else "",
                "p99_ms": stats["p99_ms"] if stats else "",
                "max_ms": stats["max_ms"] if stats else "",
                "std_ms": stats["std_ms"] if stats else "",
            })
    with (OUT / "n15_latency_decomposition.csv").open("w", newline="") as stream:
        fields = list(decomp[0])
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(decomp)

    stops = []
    for condition in summary["conditions"]:
        episodes = condition["episodes"]
        counts = episodes["termination_counts"]
        n = episodes["n_episodes"]
        for reason in sorted(set(counts) | {"end", "clearance", "speed", "grid_exit", "branch", "joint_limit", "acceleration", "invalid_lookup"}):
            count = int(counts.get(reason, 0))
            stops.append({
                "method": condition["method"], "seed": condition["seed"],
                "reason": reason, "count": count,
                "rate": count / n if n else "",
                "endpoint_checker_status": condition["endpoint_checker_status"],
                "governor_status": condition["governor_status"],
            })
    with (OUT / "n15_safe_stop_summary.csv").open("w", newline="") as stream:
        fields = list(stops[0])
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(stops)
    print(json.dumps({
        "latency_decomposition_rows": len(decomp),
        "safe_stop_rows": len(stops),
        "endpoint_checker_rows_marked_unavailable": sum(row["status"] != "measured" for row in decomp),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

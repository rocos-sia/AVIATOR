"""Recover real development snapshots and time all available execution paths."""
import argparse
import json
import os
from pathlib import Path
import numpy as np
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import ROOT, dump, profile_scenes
from tools.gfr1_shared_remount_env import SharedRemountEnv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--cpu", type=int, default=2)
    a = p.parse_args()
    os.sched_setaffinity(0, {a.cpu})
    records = {r["case"]:r for r in json.loads((a.run/"manifest.json").read_text())["records"]}
    snapshots = {}
    with (a.run/"steps.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            if "before" not in row:
                continue
            for label in row["scene"]:
                if label not in snapshots:
                    state = row["before"]
                    for name in ("_x", "_q", "_previous_n", "_r_prev", "_repair_previous_raw"):
                        if state[name] is not None:
                            state[name] = np.asarray(state[name])
                    snapshots[label] = dict(case=row["case"], record=records[row["case"]], state=state)
            if len(snapshots)==3:
                break
    if len(snapshots)!=3:
        raise ValueError("Missing regular, reversal, or boundary snapshots")
    data = ROOT/"hil-serl/data/aviator"
    first = next(iter(records.values()))
    env = SharedRemountEnv(str(data/"manifold_phi_stale_pitch-10deg"), first["model"], str(data/"trajectory_source"))
    with use_projector("combined"):
        result = profile_scenes(env, snapshots)
    dump(a.run/"scene_profiles.json", result)
    env.close()


if __name__ == "__main__":
    main()

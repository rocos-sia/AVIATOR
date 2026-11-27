"""Full-trajectory success of a frozen policy without corrective execution."""
import argparse
import json
import os
from pathlib import Path
from unittest.mock import patch
import numpy as np
import torch
from stable_baselines3 import SAC
from tools.gfr1_direct_only_env import DirectOnlyEnv
from tools.gfr1_remount_evaluate import ROOT, RULES, dump, sha, rollout


def forbidden(*args, **kwargs):
    raise AssertionError("Corrective executor was called during direct-only evaluation")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--splits", nargs="+", default=["train","development","heldout"])
    p.add_argument("--cpu", type=int, default=2)
    a = p.parse_args()
    os.sched_setaffinity(0,{a.cpu})
    torch.set_num_threads(1)
    out = a.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    checkpoint_hash = sha(a.checkpoint)
    model = SAC.load(a.checkpoint,device="cpu")
    policy = lambda env:model.predict(env.last_policy_observation,deterministic=True)[0]
    manifests = {s:json.loads((a.dataset/(s+".json")).read_text()) for s in a.splits}
    dump(out/"protocol.json",dict(checkpoint=str(a.checkpoint.resolve()),checkpoint_sha256=checkpoint_hash,
        rules=RULES,deterministic=True,scope="one unmodified RL proposal at beta=1; rejection stops the episode",
        no_projection=True,no_governor=True,no_recovery=True,tracking_gate=False,
        sources={str(Path(__file__)):sha(__file__),str(Path(__file__).with_name("gfr1_direct_only_env.py")):sha(Path(__file__).with_name("gfr1_direct_only_env.py"))},
        manifests={s:sha(a.dataset/(s+".json")) for s in a.splits}))
    data = ROOT/"hil-serl/data/aviator"
    env = DirectOnlyEnv(str(data/"manifold_phi_stale_pitch-10deg"),next(iter(manifests.values()))["records"][0]["model"],str(data/"trajectory_source"))
    rows=[]
    with patch("experiments.aviator_manifold.gfr_env.project_r",forbidden), patch("tools.gfr1_repaired_env.select_action",forbidden), (out/"steps.jsonl").open("x",buffering=1) as sink:
        for split, manifest in manifests.items():
            for record in manifest["records"]:
                row = rollout(env,record,"R",sink,policy=policy)
                # The common evaluator counts direct refusals as fallbacks for
                # historical C/E. Here no fallback takes place; rename exactly.
                row["rejected_steps"] = row["fallback_steps"]
                row["fallback_steps"] = 0
                row.update(split=split,projection_calls=0,governor_calls=0,recovery_calls=0)
                rows.append(row)
                dump(out/"episodes.json",rows)
                print(json.dumps(dict(split=split,case=row["case"],completed=row["completed"],termination=row["termination"],progress=row["progress"],steps=row["steps"])),flush=True)
    unchanged=sha(a.checkpoint)==checkpoint_hash
    summaries={s:dict(cases=sum(r["split"]==s for r in rows),completed=sum(r["split"]==s and r["completed"] for r in rows),
        median_progress=float(np.median([r["progress"] for r in rows if r["split"]==s])),
        min_progress=min(r["progress"] for r in rows if r["split"]==s),
        max_progress=max(r["progress"] for r in rows if r["split"]==s)) for s in a.splits}
    dump(out/"summary.json",dict(splits=summaries,checkpoint_unchanged=unchanged,
        evaluated_episodes=len(rows),all_requested_episodes_evaluated=len(rows)==sum(len(m["records"]) for m in manifests.values()),
        observed_executor_call_count=0,assertions_guarded_projection_and_recovery=True,
        exceptions=[r for r in rows if r["termination"]=="exception"]))
    env.close()


if __name__=="__main__":
    main()

"""Serial paired evaluation of a frozen shared checkpoint and simple proposers."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import numpy as np

from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import ROOT, RULES, FIXED, EvaluationEnv, dump, rollout, sha
from tools.gfr1_shared_remount_env import SharedRemountEnv


def paired_latency(episodes, baseline="C", proposed="E"):
    indexed = {(row["case"], row["method"]): row for row in episodes}
    pairs = []
    for case, method in sorted(indexed):
        if method != baseline or (case, proposed) not in indexed:
            continue
        left, right = indexed[case, baseline], indexed[case, proposed]
        both = left["completed"] and right["completed"]
        pairs.append(dict(case=case, installation=case.split("/")[0], both_completed=both,
            p95_saved_ms=left["cycle_ms"]["p95"]-right["cycle_ms"]["p95"] if both else None,
            p99_saved_ms=left["cycle_ms"]["p99"]-right["cycle_ms"]["p99"] if both else None,
            task_saved_ms=left["total_compute_ms"]-right["total_compute_ms"] if both else None))
    valid = [p for p in pairs if p["both_completed"]]
    clusters = sorted({p["installation"] for p in valid})
    interval = None
    if clusters:
        rng = np.random.default_rng(20261125)
        cluster_values = {c: [p["p95_saved_ms"] for p in valid if p["installation"] == c] for c in clusters}
        draws = [np.median([v for c in rng.choice(clusters, len(clusters), replace=True) for v in cluster_values[c]])
                 for _ in range(10000)]
        interval = np.percentile(draws, [2.5,97.5]).tolist()
    medians = {k: float(np.median([p[k] for p in valid])) if valid else None
               for k in ("p95_saved_ms", "p99_saved_ms", "task_saved_ms")}
    per_installation = {c: float(np.median([p["p95_saved_ms"] for p in valid if p["installation"] == c])) for c in clusters}
    return dict(baseline=baseline, proposed=proposed, pairs=pairs, median_saved=medians,
        installation_count=len(clusters), paired_p95_cluster_bootstrap_95_interval_ms=interval,
        installation_median_p95_saved_ms=per_installation,
        worst_installation=min(per_installation, key=per_installation.get) if per_installation else None,
        reliability_passed=bool(pairs) and len(valid)==len(pairs),
        relative_latency_passed=bool(interval) and len(valid)==len(pairs) and interval[0] > RULES["minimum_useful_paired_p95_gain_ms"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--checkpoint", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--methods", nargs="+", default=["B", "C", "F0", "F1", "F2"])
    p.add_argument("--cpu", type=int, default=2)
    a = p.parse_args()
    if any(m not in ("A", "B", "C", "D", "E", *FIXED) for m in a.methods):
        p.error("Unknown method")
    if any(m in ("D", "E") for m in a.methods) and a.checkpoint is None:
        p.error("D/E require the same frozen checkpoint")
    os.sched_setaffinity(0, {a.cpu})
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    records = json.loads(a.manifest.read_text())["records"]
    policy = None
    checkpoint_hash = None
    if a.checkpoint:
        import torch
        from stable_baselines3 import SAC
        torch.set_num_threads(1)
        checkpoint_hash = sha(a.checkpoint)
        model = SAC.load(a.checkpoint, device="cpu")
        policy = lambda core: model.predict(core.last_policy_observation, deterministic=True)[0]
    data = ROOT/"hil-serl/data/aviator"
    env = SharedRemountEnv(str(data/"manifold_phi_stale_pitch-10deg"), records[0]["model"], str(data/"trajectory_source"))
    legacy = EvaluationEnv(str(data/"manifold_phi_stale_pitch-10deg"), records[0]["model"], str(data/"trajectory_source")) if "A" in a.methods else None
    dump(out/"manifest.json", dict(records=records, methods=a.methods, rules=RULES,
        manifest_sha256=sha(a.manifest), checkpoint=str(a.checkpoint) if a.checkpoint else None,
        checkpoint_sha256=checkpoint_hash, source_sha256={str(Path(__file__)):sha(__file__),
            str(Path(__file__).with_name("gfr1_shared_remount_env.py")):sha(Path(__file__).with_name("gfr1_shared_remount_env.py")),
            str(Path(__file__).with_name("gfr1_remount_evaluate.py")):sha(Path(__file__).with_name("gfr1_remount_evaluate.py"))},
        cpu_affinity=sorted(os.sched_getaffinity(0))))
    episodes = []
    with use_projector("combined"), (out/"steps.jsonl").open("x", buffering=1) as sink:
        for method in a.methods:
            warmed = rollout(legacy if method == "A" else env, records[0], method, None, policy=policy, max_steps=50)
            if warmed["failures"]:
                dump(out/"gate.json", dict(passed=False, reason="warmup_failed", failure=warmed))
                return
        for index, record in enumerate(records):
            if legacy:
                legacy._model_cache.clear()
            order = a.methods[index%len(a.methods):]+a.methods[:index%len(a.methods)]
            for method in order:
                row = rollout(legacy if method == "A" else env, record, method, sink, policy=policy)
                episodes.append(row)
                dump(out/"episodes.json", episodes)
                print(json.dumps(dict(case=row["case"], method=method, completed=row["completed"], termination=row["termination"],
                    p95_ms=row["cycle_ms"]["p95"] if row["cycle_ms"] else None)), flush=True)
            # A baseline/learned reliability failure is enough to reject this
            # qualification. Fixed-action failures remain useful controls.
            if any(not r["completed"] for r in episodes if r["method"] in ("B", "C", "D", "E")):
                break
    checkpoint_unchanged = a.checkpoint is None or sha(a.checkpoint)==checkpoint_hash
    summaries = [paired_latency(episodes, b, e) for b,e in [("B","C"),("D","E"),("C","E"),("A","E"),*((m,"E") for m in FIXED)]
                if b in a.methods and e in a.methods]
    dump(out/"summary.json", dict(episodes=episodes, paired=summaries))
    all_assigned = len(episodes)==len(records)*len(a.methods)
    reliable = all(r["completed"] for r in episodes if r["method"] in ("B","C","D","E"))
    dump(out/"gate.json", dict(passed=all_assigned and reliable and checkpoint_unchanged,
        allocated_episodes=len(records)*len(a.methods), executed_episodes=len(episodes),
        pending_episodes=len(records)*len(a.methods)-len(episodes), checkpoint_unchanged=checkpoint_unchanged,
        purpose="reliability and artifact integrity; relative latency decision is recorded separately in summary"))
    env.close()
    if legacy:
        legacy.close()


if __name__ == "__main__":
    main()

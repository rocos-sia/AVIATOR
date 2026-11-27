"""Matched-budget BC-initialized vs random-initialized SAC with persistent demos."""
import argparse
from functools import partial
import hashlib
import json
from pathlib import Path
import os
import time
import numpy as np
import torch
from stable_baselines3.common.vec_env import SubprocVecEnv

from tools.gfr1_remount_evaluate import dump,sha
from tools.gfr2_train_shared_bc import evaluate
from tools.gfr3_curriculum_env import make_worker
from tools.gfr3_sac import SAC,DemoReplay,OnlineReplay,GAMMA


def terminal_transition(next_obs,dones,infos):
    """VecEnv auto-reset observations must never become terminal successors."""
    successor=next_obs.copy()
    terminal=np.zeros(len(dones),dtype=np.float32)
    for i,done in enumerate(dones):
        if done:
            successor[i]=infos[i]["terminal_observation"]
            terminal[i]=float(not infos[i].get("TimeLimit.truncated",False))
    return successor,terminal


def fingerprint(module):
    h=hashlib.sha256()
    for name,value in module.state_dict().items():
        h.update(name.encode());h.update(value.detach().cpu().numpy().tobytes())
    return h.hexdigest()


def run(a):
    root=a.root.resolve();source=a.source_root.resolve()
    out=root/a.initialization
    out.mkdir(exist_ok=False)
    if not json.loads((root/"demonstrations/gate.json").read_text())["passed"]:
        raise RuntimeError("SAC demonstration transition audit failed")
    demo_rows=json.loads((root/"demonstrations/episodes.json").read_text())
    demo_files=[row["file"] for row in sorted(demo_rows,key=lambda r:r["case"])]
    checkpoint=source/"shared_bc/actor_round_2.pt"
    learner=SAC(checkpoint,a.initialization,a.seed)
    demo=DemoReplay(demo_files,learner.device)
    replay=OnlineReplay(a.steps+8,learner.device,a.seed)
    evaluation_records=[]
    for split in ("train","development"):
        evaluation_records+=json.loads((source/f"dataset/{split}.json").read_text())["records"]
    if a.smoke:evaluation_records=evaluation_records[:1]
    schedule=[0,a.steps] if a.smoke else [0,20000,60000,120000]
    if not a.smoke and a.steps!=120000:
        raise ValueError("Formal budget is frozen at 120k environment transitions")
    workers=2
    if a.steps%workers:raise ValueError("Steps must be divisible by two")
    dependencies=[Path(__file__).with_name(name) for name in (
        "gfr2_shared_env.py","gfr2_train_shared_bc.py","gfr1_direct_only_env.py",
        "gfr1_shared_remount_env.py","gfr1_remount_evaluate.py","validate_online_lift.py")]
    dependencies.append(Path(__file__).parents[1]/"examples/experiments/aviator_manifold/gfr_env.py")
    config=dict(initialization=a.initialization,seed=a.seed,device=str(learner.device),
        gpu=torch.cuda.get_device_name(),torch=torch.__version__,environment_transitions_budget=a.steps,
        actor_updates_per_environment_transition=1,critic_warmup_updates=16 if a.smoke else 1000,
        batch_size=256,demo_batch_size=128,online_batch_size=128,
        demo_sampling="equal trajectory probability; half uniform frames, half critical/approach frames",
        reward="DirectOnlyEnv reward divided by 100; identical demo and online semantics",
        gamma=GAMMA,tau=.005,actor_lr=1e-4,critic_lr=3e-4,
        entropy_target=-4.,initial_alpha=.01,alpha_bounds=[1e-5,.1],
        initial_log_std=-3.,log_std_bounds=[-5.,1.],
        bc_weight="0.2 for first 20k; linearly decreases to 0.05 at 120k",
        curriculum=[dict(until=20000,segment_steps=64,original_start_probability=.25),
                    dict(until=60000,segment_steps=256,original_start_probability=.25),
                    dict(until=90000,segment_steps=1024,original_start_probability=.5),
                    dict(until=120000,segment_steps=None,original_start_probability=1.)],
        initialization_checkpoint_sha256=sha(checkpoint),bc_transfer_max_error=learner.transfer_max_error,
        initial_critic_sha256=[fingerprint(learner.q1),fingerprint(learner.q2)],
        initial_actor_sha256=fingerprint(learner.actor),evaluation_steps=schedule,
        evaluation="full original-start train/development trajectories; deterministic mean; no correction",
        no_zero_success_early_stop=True,stop_on="nonfinite learner, accepted-motion audit failure, instrumentation error",
        reset_workers=workers,evaluation_workers=1,installation_rotation="256 worker transitions per installation block, applied at reset",
        smoke=a.smoke,heldout_used=False,
        source_sha256={str(p.resolve()):sha(p) for p in [*Path(__file__).parent.glob("gfr3_*.py"),*dependencies]},
        training_manifest_sha256=sha(source/"dataset/train.json"),
        development_manifest_sha256=sha(source/"dataset/development.json"),
        demo_hashes={str(p):sha(p) for p in demo_files})
    dump(out/"config.json",config)
    env=None
    summaries=[];steps=0;accepted=0;failures=0;full_ends=0;segment_ends=0
    started=time.perf_counter()
    try:
        dump(out/"status.json",dict(phase="critic_warmup",pid=os.getpid(),environment_steps=0))
        for _ in range(config["critic_warmup_updates"]):
            batch=demo.sample(256)
            learner.update(batch,(batch[0],batch[1]),0.,critic_only=True)
        factories=[partial(make_worker,str(source),str(root/"demonstrations"),str(out),i,workers) for i in range(workers)]
        env=SubprocVecEnv(factories,start_method="spawn")
        env.seed(a.seed);obs=env.reset()
        def checkpoint_and_evaluate():
            actor_file=out/f"actor_{steps:06d}.pt"
            learner.export(actor_file);learner.save(out/f"sac_{steps:06d}.pt",steps)
            dump(out/"status.json",dict(phase="full_trajectory_evaluation",pid=os.getpid(),
                environment_steps=steps,actor_gradient_updates=learner.updates,actor_checkpoint=str(actor_file)))
            gate,rows=evaluate(evaluation_records,actor_file,out/f"evaluation_{steps:06d}",1)
            if any(r["termination"]=="exception" or r.get("audit_violations") for r in rows):
                raise RuntimeError("Evaluation instrumentation/audit failure")
            summary=dict(environment_steps=steps,actor_gradient_updates=learner.updates,
                critic_gradient_updates=learner.updates+learner.critic_warmup_updates,
                checkpoint=str(actor_file),checkpoint_sha256=sha(actor_file),splits={})
            for split in ("train","development"):
                part=[r for r in rows if r["case"].startswith(split+"_")]
                if part:summary["splits"][split]=dict(cases=len(part),completed=sum(r["completed"] for r in part),
                    median_progress=float(np.median([r["progress"] for r in part])),max_progress=max(r["progress"] for r in part))
            summaries.append(summary);dump(out/"evaluations.json",summaries)
            print(json.dumps(dict(event="evaluation",**summary)),flush=True)
        checkpoint_and_evaluate()
        with (out/"learning.jsonl").open("x",buffering=1) as log:
            while steps<a.steps:
                actions=learner.predict(obs)
                next_obs,rewards,dones,infos=env.step(actions)
                successor,terminal=terminal_transition(next_obs,dones,infos)
                replay.add(obs,actions,rewards,successor,terminal)
                obs=next_obs;steps+=workers
                accepted+=sum(info["accepted"] for info in infos)
                failures+=sum(not info["accepted"] for info in infos)
                full_ends+=sum(info["start_frame"]==0 and info["termination"]=="end" for info in infos)
                segment_ends+=sum(info.get("curriculum_truncated",False) for info in infos)
                weight=.2-.15*max(0.,min(1.,(steps-20000)/100000.))
                for _ in range(workers):
                    db=demo.sample(128);ob=replay.sample(128)
                    batch=tuple(torch.cat((d,o),dim=0) for d,o in zip(db,ob))
                    metrics=learner.update(batch,(db[0],db[1]),weight)
                if steps%1000==0 or steps==a.steps:
                    row=dict(environment_steps=steps,actor_gradient_updates=learner.updates,
                        critic_gradient_updates=learner.updates+learner.critic_warmup_updates,
                        accepted=accepted,rejected=failures,training_original_start_completions=full_ends,
                        curriculum_segments_ended=segment_ends,elapsed_s=time.perf_counter()-started,
                        gpu_allocated_bytes=torch.cuda.memory_allocated(),**metrics)
                    log.write(json.dumps(row)+"\n")
                    dump(out/"status.json",dict(phase="sac_training",pid=os.getpid(),**row))
                    print(json.dumps(row),flush=True)
                if steps in schedule:checkpoint_and_evaluate()
        changed=[p for p,h in config["demo_hashes"].items() if sha(p)!=h]
        changed += [p for p,h in config["source_sha256"].items() if sha(p)!=h]
        if changed:raise AssertionError("Frozen artifacts changed: "+str(changed))
        # Save actual online transitions, including terminal successors rather
        # than auto-reset states, for audit/reuse. No claim of exact resume.
        np.savez_compressed(out/"online_replay.npz",**{k:v[:replay.size] for k,v in zip(
            ("observations","actions","rewards","next_observations","terminals"),replay.arrays)})
        qualified=not a.smoke and any(all(s["completed"]==s["cases"] for s in row["splits"].values())
                                      for row in summaries if row["environment_steps"]>0)
        result=dict(execution_completed=True,passed=qualified,smoke=a.smoke,
            environment_steps=steps,actor_gradient_updates=learner.updates,
            critic_gradient_updates=learner.updates+learner.critic_warmup_updates,
            heldout_evaluated=False,reason="smoke_only" if a.smoke else ("qualified_for_freezing" if qualified else "budget_finished_without_full_reliability"))
        dump(out/"gate.json",result);dump(out/"status.json",dict(phase="finished",**result))
        return result
    except BaseException as error:
        result=dict(execution_completed=False,passed=False,environment_steps=steps,error=repr(error))
        dump(out/"gate.json",result);dump(out/"status.json",dict(phase="stopped_on_error",**result))
        raise
    finally:
        if env is not None:env.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",type=Path,required=True);p.add_argument("--source-root",type=Path,required=True)
    p.add_argument("--initialization",choices=["bc","random"],required=True)
    p.add_argument("--steps",type=int,default=120000);p.add_argument("--seed",type=int,default=11)
    p.add_argument("--smoke",action="store_true")
    run(p.parse_args())

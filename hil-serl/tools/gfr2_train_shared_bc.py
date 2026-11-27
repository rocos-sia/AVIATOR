"""CUDA-only shared BC, bounded DAgger, and no-correction qualification.

This is the first feasibility stage, not an RL or real-world success claim.
Development labels are never queried; heldout runs only after qualification.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
import os
from pathlib import Path
import time
import numpy as np
import torch
from torch import nn

from tools.gfr1_remount_evaluate import dump, sha, audit_motion
from tools.gfr1_clearance_variants import use_projector
from tools.audit_static_collision import CollisionChecker
from tools.gfr2_teacher import make_env, choose_teacher


def cuda_device():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required; CPU fallback is forbidden")
    torch.set_num_threads(1)
    device = torch.device("cuda:0")
    # Execute an actual forward/backward kernel, not merely an availability test.
    x = torch.randn(128, 128, device=device, requires_grad=True)
    (x@x.T).square().mean().backward()
    torch.cuda.synchronize()
    return device


class Actor(nn.Module):
    def __init__(self, mean, scale):
        super().__init__()
        self.register_buffer("mean", torch.as_tensor(mean, dtype=torch.float32))
        self.register_buffer("scale", torch.as_tensor(scale, dtype=torch.float32))
        self.net = nn.Sequential(nn.Linear(128,256), nn.ReLU(), nn.Linear(256,256), nn.ReLU(),
                                 nn.Linear(256,2), nn.Tanh())

    def forward(self, obs):
        return self.net((obs-self.mean)/self.scale)


def load_actor(checkpoint, device):
    saved = torch.load(checkpoint, map_location=device, weights_only=True)
    actor = Actor(np.zeros(128), np.ones(128)).to(device)
    actor.load_state_dict(saved["state_dict"])
    return actor.eval()


def train(files, output, updates, seed, initialize=None):
    device = cuda_device()
    torch.manual_seed(seed)
    observations, actions, weights = [], [], []
    for filename in files:
        with np.load(filename) as data:
            if len(data["actions"]) == 0:
                continue
            observations.append(data["observations"])
            actions.append(data["actions"])
            weights.extend([1./len(data["actions"])]*len(data["actions"]))
    obs = np.concatenate(observations).astype(np.float32)
    target = np.concatenate(actions).astype(np.float32)
    if not np.isfinite(obs).all() or not np.isfinite(target).all() or np.max(np.abs(target))>1:
        raise ValueError("Invalid teacher data")
    mean = obs.mean(0)
    scale = np.maximum(obs.std(0), .05)
    actor = load_actor(initialize, device) if initialize else Actor(mean, scale).to(device)
    actor.train()
    x = torch.as_tensor(obs, device=device)
    y = torch.as_tensor(target, device=device)
    sampling = torch.as_tensor(weights, dtype=torch.float32, device=device)
    optimizer = torch.optim.Adam(actor.parameters(), lr=3e-4)
    started = time.perf_counter()
    output = Path(output)
    with output.with_suffix(".training.jsonl").open("x",buffering=1) as log:
        for step in range(1,updates+1):
            index = torch.multinomial(sampling,1024,replacement=True)
            prediction = actor(x[index])
            loss = (prediction-y[index]).square().mean()
            if not torch.isfinite(loss):
                raise FloatingPointError("Nonfinite BC loss")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(actor.parameters(),1.)
            optimizer.step()
            if step % 500 == 0 or step == updates:
                torch.cuda.synchronize()
                row = dict(update=step,mse=float(loss.detach()),elapsed_s=time.perf_counter()-started,
                           device=str(next(actor.parameters()).device),gpu_allocated_bytes=torch.cuda.memory_allocated())
                log.write(json.dumps(row)+"\n")
                print(json.dumps(row),flush=True)
    actor.eval()
    with torch.no_grad():
        mse = float((actor(x)-y).square().mean())
    torch.save(dict(state_dict=actor.state_dict()),output)
    result = dict(checkpoint=str(output),checkpoint_sha256=sha(output),device=str(device),
        gpu_name=torch.cuda.get_device_name(),torch=torch.__version__,cuda=torch.version.cuda,
        updates=updates,samples=len(obs),in_sample_mse=mse,elapsed_s=time.perf_counter()-started,
        sampling="equal dataset-file probability; uniform frames within each file",
        gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated())
    dump(output.with_suffix(".training.json"),result)
    return result


def run_case(record, checkpoint, output, dagger=False, seed=11):
    device = cuda_device()
    actor = load_actor(checkpoint,device)
    out = Path(output)
    name = record["case"].replace("/","__")
    env = make_env(record)
    audit = CollisionChecker(record["true_model"])
    observations, labels = [], []
    rows, violations = [], []
    termination = "initialization_invalid"
    started = time.perf_counter()
    try:
        if record["valid"]:
            budget = int(np.ceil((len(env._traj["x"])-1)*env.traj_dt/env.dt))+1
            with use_projector("combined"), (out/f"{name}.jsonl").open("x",buffering=1) as log:
                for step in range(budget):
                    before_obs = env.last_policy_observation.copy()
                    x,q = env._x.copy(),env._q.copy()
                    cycle = time.perf_counter()
                    with torch.no_grad():
                        action = actor(torch.as_tensor(before_obs,device=device)).cpu().numpy()
                    # Student evaluation performs exactly one direct step. In
                    # DAgger only, query a teacher before commit, with interventions
                    # reported separately and never counted as student success.
                    intervention = False
                    if dagger:
                        trial,_ = env._candidate(x,q,env._path_state(env._u)[1],action*env.r_max,
                                                 1.,env._previous_n,structured=False)
                        if step % 20 == 0 or not trial["feasible"]:
                            target,selection = choose_teacher(env,step,force_search=True)
                            if target is None:
                                termination = "dagger_teacher_unavailable"
                                break
                            observations.append(before_obs)
                            labels.append(target)
                            if not trial["feasible"]:
                                action = target
                                intervention = True
                    _,_,done,truncated,info = env.step(action)
                    elapsed_ms = (time.perf_counter()-cycle)*1000.
                    verdict = audit_motion(env,audit,x,q) if env._last_direct_accepted else None
                    violations = verdict["violations"] if verdict else []
                    row = dict(step=step,progress=info["progress"],accepted=env._last_direct_accepted,
                        action=action.tolist(),elapsed_ms=elapsed_ms,teacher_intervention=intervention,
                        termination=info["termination"],audit_violations=violations,
                        direct_trial=env._json_safe_candidate(env._last_direct_candidate))
                    rows.append(row)
                    log.write(json.dumps(row)+"\n")
                    termination = info["termination"]
                    if violations:
                        termination = "independent_audit_failed"
                        break
                    if done or truncated:
                        break
                else:
                    termination = "step_budget"
        result = dict(case=record["case"],completed=termination=="end" and not violations and not dagger,
            rollout_reached_end=termination=="end" and not violations,dagger=dagger,
            termination=termination,steps=len(rows),progress=float(env._u/(len(env._traj["x"])-1)),
            teacher_interventions=sum(r["teacher_intervention"] for r in rows),
            elapsed_s=time.perf_counter()-started,audit_violations=violations,
            p95_ms=float(np.percentile([r["elapsed_ms"] for r in rows],95)) if rows else None,
            timing_scope="parallel diagnostic; not exclusive paired latency evidence",
            projection_calls=0,recovery_calls=0,policy_device=str(device),labels=len(labels))
        if dagger:
            file = out/f"{name}.npz"
            np.savez_compressed(file,observations=np.asarray(observations,np.float32).reshape(-1,128),
                                actions=np.asarray(labels,np.float32).reshape(-1,2))
            result["dataset_file"] = str(file)
        dump(out/f"{name}.summary.json",result)
        return result
    finally:
        env.close()


def evaluate(records,checkpoint,output,workers,dagger=False):
    out = Path(output)
    out.mkdir(exist_ok=False)
    rows = []
    checkpoint_hash = sha(checkpoint)
    with ProcessPoolExecutor(max_workers=workers,mp_context=mp.get_context("spawn")) as pool:
        futures = {pool.submit(run_case,r,str(checkpoint),str(out),dagger):r for r in records}
        for future in as_completed(futures):
            try:
                row = future.result()
            except Exception as exc:
                row = dict(case=futures[future]["case"],completed=False,termination="exception",error=repr(exc))
            rows.append(row)
            dump(out/"episodes.json",rows)
            print(json.dumps(row),flush=True)
    result = dict(passed=not dagger and len(rows)==len(records) and all(r["completed"] for r in rows),
        cases=len(records),completed=sum(r["completed"] for r in rows),dagger=dagger,
        checkpoint_unchanged=sha(checkpoint)==checkpoint_hash,
        checkpoint_sha256=checkpoint_hash,failures=[r for r in rows if not r["completed"]])
    if not result["checkpoint_unchanged"]:
        raise AssertionError("Checkpoint changed during evaluation")
    dump(out/"gate.json",result)
    return result,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root",type=Path,required=True)
    parser.add_argument("--updates",type=int,default=8000)
    parser.add_argument("--dagger-rounds",type=int,default=2)
    parser.add_argument("--workers",type=int,default=4)
    a=parser.parse_args()
    root=a.root.resolve()
    out=root/"shared_bc"
    out.mkdir(exist_ok=False)
    if not json.loads((root/"teacher/gate.json").read_text())["passed"]:
        raise RuntimeError("Teacher full-trajectory replay gate failed")
    cases={split:json.loads((root/f"dataset/{split}.json").read_text())["records"]
           for split in ("train","development","heldout")}
    files=[r["dataset_file"] for r in json.loads((root/"teacher/episodes.json").read_text())]
    teacher_hashes={str(file):sha(file) for file in files}
    dump(out/"protocol.json",dict(seed=11,updates_per_round=a.updates,dagger_rounds=a.dagger_rounds,
        device="cuda:0",qualification="all 16 train and 8 development complete without correction or audit violation",
        heldout="once, after development qualification; never provides teacher labels",
        stage="BC feasibility before RL",rules="same closure/clearance/velocity constraints as original validation",
        source_sha256=sha(__file__),teacher_data_sha256=teacher_hashes,
        dataset_hashes={s:sha(root/f"dataset/{s}.json") for s in cases}))
    checkpoint=None
    for round_index in range(a.dagger_rounds+1):
        dump(root/"status.json",dict(phase="gpu_bc_training",round=round_index,pid=os.getpid()))
        target=out/f"actor_round_{round_index}.pt"
        train(files,target,a.updates,11,checkpoint)
        checkpoint=target
        dump(root/"status.json",dict(phase="direct_closed_loop_evaluation",round=round_index,checkpoint=str(checkpoint),pid=os.getpid()))
        gate,_=evaluate(cases["train"]+cases["development"],checkpoint,out/f"evaluation_{round_index}",a.workers)
        if any(sha(file)!=expected for file,expected in teacher_hashes.items()):
            raise AssertionError("Frozen teacher data changed")
        if gate["passed"]:
            dump(root/"status.json",dict(phase="frozen_heldout_evaluation",checkpoint=str(checkpoint),pid=os.getpid()))
            heldout,_=evaluate(cases["heldout"],checkpoint,out/"heldout",a.workers)
            result=dict(passed=heldout["passed"],reason="shared_bc_qualified" if heldout["passed"] else "heldout_completion_failed",
                        checkpoint=str(checkpoint),checkpoint_sha256=sha(checkpoint),rl_started=False)
            dump(out/"gate.json",result)
            dump(root/"status.json",dict(phase="bc_stage_finished",**result))
            return
        if any(r["termination"]=="exception" or r.get("audit_violations") for r in gate["failures"]):
            reason="instrumentation_or_audit_failure"
            break
        if round_index==a.dagger_rounds:
            reason="standalone_shared_bc_gate_failed_after_bounded_dagger"
            break
        dump(root/"status.json",dict(phase="training_only_dagger_collection",round=round_index,pid=os.getpid()))
        _,rows=evaluate(cases["train"],checkpoint,out/f"dagger_{round_index}",a.workers,dagger=True)
        if any(r["termination"]=="exception" or r.get("audit_violations") or not r.get("rollout_reached_end") for r in rows):
            reason="dagger_teacher_coverage_failed"
            break
        files.extend(r["dataset_file"] for r in rows if r.get("labels",0)>0)
    result=dict(passed=False,reason=reason,checkpoint=str(checkpoint),rl_started=False,heldout_evaluated=False)
    dump(out/"gate.json",result)
    dump(root/"status.json",dict(phase="stopped_at_failed_condition",**result))


if __name__=="__main__":
    main()

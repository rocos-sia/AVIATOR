"""Read-only checkpoint/replay audit; no optimizer updates or new training."""
import argparse
from collections import Counter,defaultdict
import json
import math
from pathlib import Path
import numpy as np
import torch
from tools.gfr1_remount_evaluate import dump,sha
from tools.gfr3_sac import GaussianActor,Critic,GAMMA
from tools.gfr2_train_shared_bc import cuda_device


def stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return dict(n=0)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),
                p05=float(np.quantile(a,.05)),p95=float(np.quantile(a,.95)),min=float(a.min()),max=float(a.max()))


def load_nets(path,device):
    state=torch.load(path,map_location=device,weights_only=True)
    mean=state["actor"]["mean"].cpu().numpy();scale=state["actor"]["scale"].cpu().numpy()
    actor=GaussianActor(mean,scale).to(device);actor.load_state_dict(state["actor"]);actor.eval()
    q1=Critic(mean,scale).to(device);q1.load_state_dict(state["q1"]);q1.eval()
    q2=Critic(mean,scale).to(device);q2.load_state_dict(state["q2"]);q2.eval()
    return actor,q1,q2,float(state["log_alpha"].exp()),state


def gradients(actor,q1,q2,obs,targets,alpha,weight):
    for q in (q1,q2):q.requires_grad_(False)
    torch.manual_seed(20261126)
    action,logprob=actor.sample(obs)
    sac=(alpha*logprob-torch.minimum(q1(obs,action),q2(obs,action))).mean()
    bc=(actor.sample(obs,True)[0]-targets).square().mean()
    params=list(actor.parameters())
    def flat(loss):
        parts=torch.autograd.grad(loss,params,allow_unused=True)
        return torch.cat([(torch.zeros_like(p) if g is None else g).ravel() for p,g in zip(params,parts)])
    gs=flat(sac);gb=flat(bc)*weight
    return dict(sac_gradient_norm=float(gs.norm()),weighted_bc_gradient_norm=float(gb.norm()),
        weighted_bc_to_sac_norm_ratio=float(gb.norm()/gs.norm().clamp_min(1e-12)),
        cosine=float((gs@gb)/(gs.norm()*gb.norm()).clamp_min(1e-12)),bc_weight=weight,
        scope="fixed uniform teacher probe, pre-clipping gradients; not causal attribution")


def inspect(root,out):
    device=cuda_device();rng=np.random.default_rng(20261126)
    out.mkdir(parents=True,exist_ok=False)
    demo_parts=[];keys=("observations","actions","rewards","terminals","next_observations","critical")
    installations={}
    for file in sorted((root/"demonstrations").glob("*.npz")):
        with np.load(file) as data:part={k:data[k].copy() for k in keys}
        installations[file.stem.split("__")[0]]=part["observations"][0,40:64]
        demo_parts.append(part)
    demo={k:np.concatenate([p[k] for p in demo_parts]) for k in keys}
    probe=rng.choice(len(demo["actions"]),4096,replace=False)
    result=dict(scope="read-only, fixed checkpoints; no retraining; same data probes for both checkpoints",arms={})
    all_hashes={}
    for arm in ("bc","random"):
        directory=root/arm
        with np.load(directory/"online_replay.npz") as archive:data={k:archive[k].copy() for k in keys if k!="critical"}
        data["rewards"]=data["rewards"].ravel();data["terminals"]=data["terminals"].ravel()
        before={r["case"]:r for r in json.loads((directory/"evaluation_060000/episodes.json").read_text())}
        after={r["case"]:r for r in json.loads((directory/"evaluation_120000/episodes.json").read_text())}
        lost=[k for k in before if before[k]["completed"] and not after[k]["completed"]]
        gained=[k for k in before if not before[k]["completed"] and after[k]["completed"]]
        comparisons=[dict(case=k,progress_60k=before[k]["progress"],progress_120k=after[k]["progress"],
            completed_60k=before[k]["completed"],completed_120k=after[k]["completed"]) for k in before]
        report=dict(lost_successes=lost,gained_successes=gained,paired_cases=comparisons,checkpoints={},stages=[])
        rows=[json.loads(line) for line in (directory/"learning.jsonl").open()]
        for low,high in ((0,20000),(20000,60000),(60000,90000),(90000,120000)):
            selected=[row for row in rows if low<row["environment_steps"]<=high]
            window=data["observations"][low:high]
            geometry=window[:,40:64]
            counts={name:int(np.count_nonzero(np.max(np.abs(geometry-value),axis=1)<1e-6)) for name,value in installations.items()}
            report["stages"].append(dict(start=low,end=high,
                learning={key:stats([r[key] for r in selected]) for key in ("critic_loss","q_mean","bc_loss","alpha","bc_weight")},
                online_installation_frames=counts,rejected_frames=int(np.sum(data["rewards"][low:high]<-9)),
                terminal_successes=int(np.sum((data["terminals"][low:high]>0)&(data["rewards"][low:high]>0))),
                online_progress=stats(window[:,39])))
        resets=[];episodes=[]
        for file in directory.glob("worker_*.jsonl"):
            current=None
            for line in file.open():
                row=json.loads(line)
                if row["event"]=="reset":current=row;resets.append(row)
                elif row["event"]=="episode_end":
                    row["estimated_global_end"]=current["global_steps"]+2*row["steps"]
                    row["reset_global_step"]=current["global_steps"]
                    row["original_start_reset"]=current["from_original_start"]
                    episodes.append(row)
        report["reset_stages"]=[]
        for low,high in ((0,20000),(20000,60000),(60000,90000),(90000,120001)):
            rs=[r for r in resets if low<=r["global_steps"]<high]
            es=[e for e in episodes if low<=e["reset_global_step"]<high]
            report["reset_stages"].append(dict(start=low,end=high,resets=len(rs),
                original_start_resets=sum(r["from_original_start"] for r in rs),completed_episodes=len(es),
                full_task_successes=sum(e["full_task_success"] for e in es),
                ended_segments=sum(e["curriculum_truncated"] for e in es),
                ending_progress=stats([e["progress"] for e in es])))
        masks=dict(seen_terminal_failures=(data["rewards"]<-9)&(np.arange(len(data["rewards"]))<60000),
                   later_terminal_failures=(data["rewards"]<-9)&(np.arange(len(data["rewards"]))>=60000),
                   seen_terminal_successes=(data["terminals"]>0)&(data["rewards"]>0)&(np.arange(len(data["rewards"]))<60000))
        for step in (60000,120000):
            path=directory/f"sac_{step:06d}.pt";all_hashes[str(path)]=sha(path)
            actor,q1,q2,alpha,state=load_nets(path,device)
            with torch.no_grad():
                observations=torch.as_tensor(demo["observations"][probe],device=device)
                targets=torch.as_tensor(demo["actions"][probe],device=device)
                actions=actor.sample(observations,True)[0]
                q_teacher=torch.minimum(q1(observations,targets),q2(observations,targets)).ravel()
                q_policy=torch.minimum(q1(observations,actions),q2(observations,actions)).ravel()
                # Upper bound on expected soft-Q for normalized actions in [-1,1]^2:
                # remaining positive progress reward <=10*(1-progress), completion <=1;
                # entropy <= log(area)=log(4). Infinite horizon entropy is conservative.
                bound=10.*(1.-observations[:,39])+1.+alpha*GAMMA/(1.-GAMMA)*math.log(4.)
                errs=((actions-targets)**2).mean(-1).cpu().numpy()
                changing=np.linalg.norm(demo["actions"][probe]-demo["observations"][probe,33:35],axis=1)>.1
                checkpoint=dict(alpha=alpha,teacher_probe_mse=float(errs.mean()),
                    teacher_change_mse=float(errs[changing].mean()),teacher_hold_mse=float(errs[~changing].mean()),
                    mean_action_saturation_fraction=float((actions.abs()>.95).float().mean()),
                    teacher_action_q=stats(q_teacher.cpu().numpy()),policy_action_q=stats(q_policy.cpu().numpy()),
                    expected_soft_q_upper_bound=stats(bound.cpu().numpy()),
                    teacher_q_above_bound_fraction=float((q_teacher>bound).float().mean()),
                    policy_q_above_bound_fraction=float((q_policy>bound).float().mean()),terminal_calibration={})
                for label,mask in masks.items():
                    obs=torch.as_tensor(data["observations"][mask],device=device)
                    act=torch.as_tensor(data["actions"][mask],device=device)
                    pred=torch.minimum(q1(obs,act),q2(obs,act)).ravel().cpu().numpy()
                    truth=data["rewards"][mask]
                    checkpoint["terminal_calibration"][label]=dict(q=stats(pred),true_reward=stats(truth),
                        error=stats(pred-truth),positive_prediction_fraction=float(np.mean(pred>0)) if len(pred) else None)
                # Check failures under actual actor-at-collection online data.
                # The first 60k prefix is shared between checkpoint probes.
            checkpoint["gradient_balance"]=gradients(actor,q1,q2,observations[:512],targets[:512],alpha,
                                                        .14 if step==60000 else .05)
            report["checkpoints"][str(step)]=checkpoint
            del actor,q1,q2,state
        result["arms"][arm]=report
        dump(out/"checkpoint_audit.json",result)
        print(json.dumps(dict(arm=arm,lost=lost,gained=gained,checkpoints=report["checkpoints"])),flush=True)
    result["checkpoint_sha256"]=all_hashes
    result["checkpoints_unchanged"]=all(sha(p)==h for p,h in all_hashes.items())
    result["source_sha256"]=sha(__file__)
    dump(out/"checkpoint_audit.json",result)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();inspect(a.root.resolve(),a.output.resolve())

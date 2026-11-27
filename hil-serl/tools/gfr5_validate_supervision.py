"""Fixed-budget boundary classification and terminal-anchored critic checks.

No Actor updates. Ordinary feasible transitions retain their Bellman targets;
only genuine terminal transitions receive known-return supervision.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import torch
from torch.nn import functional as F
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr2_train_shared_bc import cuda_device
from tools.gfr3_sac import Critic, DemoReplay, GAMMA
from tools.gfr3_train_sac_demo import fingerprint
from tools.gfr4_common import require_memory_limit, restore


class PairedData:
    def __init__(self, directory, device):
        self.groups=json.loads((directory/'groups.json').read_text())
        with np.load(directory/'transitions.npz') as data:
            self.arrays={k:data[k].copy() for k in data.files}
        self.tensors={k:torch.as_tensor(v,device=device) for k,v in self.arrays.items()}
        self.indices={s:np.flatnonzero(np.isin(self.arrays['group'],[g['id'] for g in self.groups if g['split']==s]))
                      for s in ('fit','trajectory_validation','installation_test','historical_test')}
        self.fit_groups=[g for g in self.groups if g['split']=='fit']
        self.mixed=[g for g in self.fit_groups if g['mixed']]
        self.by_group={g['id']:np.arange(g['start'],g['end']) for g in self.groups}
        self.failures={};self.successes={}
        for g in self.fit_groups:
            idx=self.by_group[g['id']]
            fail=idx[self.arrays['feasible'][idx]==0]
            success=idx[(self.arrays['terminals'][idx]==1)&(self.arrays['feasible'][idx]==1)]
            if len(fail):self.failures[g['id']]=fail
            if len(success):self.successes[g['id']]=success
        assert self.failures and self.successes and self.mixed
        fit_obs={self.arrays['observations'][g['start']].tobytes() for g in self.fit_groups}
        other_obs={self.arrays['observations'][g['start']].tobytes() for g in self.groups if g['split']!='fit'}
        if fit_obs & other_obs:
            raise AssertionError('Identical observations appear in fitting and held-out groups')
        for g in self.groups:
            idx=self.by_group[g['id']]
            assert np.all(self.arrays['observations'][idx]==self.arrays['observations'][idx[0]])
        fit=self.indices['fit']
        assert np.all(self.arrays['terminals'][fit[self.arrays['feasible'][fit]==0]]==1)

    def uniform_states(self, rng, count):
        chosen=rng.integers(len(self.fit_groups),size=count)
        return np.array([rng.integers(self.fit_groups[i]['start'],self.fit_groups[i]['end']) for i in chosen])

    def anchors(self, rng, count=32):
        result=[]
        for bank in (self.failures,self.successes):
            keys=sorted(bank)
            ids=rng.integers(len(keys),size=count)
            result.append(np.array([rng.choice(bank[keys[i]]) for i in ids]))
        return result

    def classification_batch(self, rng):
        ordinary=self.uniform_states(rng,192);pairs=[]
        for gi in rng.integers(len(self.mixed),size=32):
            indices=self.by_group[self.mixed[gi]['id']]
            flags=self.arrays['feasible'][indices]
            pairs.extend((rng.choice(indices[flags==0]),rng.choice(indices[flags==1])))
        return np.r_[ordinary,pairs].astype(np.int64)

    def batch(self, index):
        return tuple(self.tensors[k][index].reshape(len(index),-1) for k in
                     ('observations','actions','rewards','next_observations','terminals'))


def terminal_stats(prediction, target):
    if not len(prediction):
        return dict(count=0)
    error=prediction-target
    return dict(count=len(prediction),q_mean=float(prediction.mean()),target_mean=float(target.mean()),
        mae=float(np.abs(error).mean()),rmse=float(np.sqrt(np.mean(error**2))),
        max_absolute_error=float(np.abs(error).max()),positive_fraction=float(np.mean(prediction>0)))


def value_metrics(learner, data, splits):
    result={}
    with torch.no_grad():
        for split in splits:
            ids=data.indices[split];obs=data.tensors['observations'][ids];act=data.tensors['actions'][ids]
            q1=learner.q1(obs,act).ravel().cpu().numpy();q2=learner.q2(obs,act).ravel().cpu().numpy()
            masks=dict(failure=data.arrays['feasible'][ids]==0,
                       success=(data.arrays['feasible'][ids]==1)&(data.arrays['terminals'][ids]==1))
            row={}
            for name,mask in masks.items():
                truth=data.arrays['rewards'][ids][mask]
                row[name]=terminal_stats(np.minimum(q1,q2)[mask],truth)
                row[name]['individual_critics']=[terminal_stats(q[mask],truth) for q in (q1,q2)]
            result[split]=row
    return result


def value_update(learner,batch,anchors):
    obs,act,reward,nxt,terminal=batch
    with torch.no_grad():
        action,logprob=learner.actor.sample(nxt)
        target=reward+GAMMA*(1-terminal)*(torch.minimum(learner.target1(nxt,action),learner.target2(nxt,action))
                                       -learner.log_alpha.exp()*logprob)
    loss=F.mse_loss(learner.q1(obs,act),target)+F.mse_loss(learner.q2(obs,act),target)
    parts=dict(td_loss=float(loss.detach()))
    for name,anchor in zip(('failure_anchor','success_anchor'),anchors):
        aobs,aact,areward,_,aterminal=anchor
        assert bool(torch.all(aterminal==1))
        term=F.mse_loss(learner.q1(aobs,aact),areward)+F.mse_loss(learner.q2(aobs,aact),areward)
        loss=loss+term;parts[name]=float(term.detach())
    if not torch.isfinite(loss):raise FloatingPointError('Nonfinite critic loss')
    learner.q_opt.zero_grad(set_to_none=True);loss.backward()
    torch.nn.utils.clip_grad_norm_(list(learner.q1.parameters())+list(learner.q2.parameters()),10.)
    learner.q_opt.step()
    with torch.no_grad():
        for live,slow in ((learner.q1,learner.target1),(learner.q2,learner.target2)):
            for param,old in zip(live.parameters(),slow.parameters()):old.lerp_(param,.005)
    learner.critic_warmup_updates+=1
    return parts


def classification_metrics(model,data,splits):
    result={}
    with torch.no_grad():
        scores=model(data.tensors['observations'],data.tensors['actions']).sigmoid().ravel().cpu().numpy()
    for split in splits:
        indices=data.indices[split];truth=data.arrays['feasible'][indices].astype(bool)
        prediction=scores[indices]>=.5;pair_scores=[]
        for g in data.groups:
            if g['split']==split and g['mixed']:
                idx=data.by_group[g['id']];yes=data.arrays['feasible'][idx]==1
                delta=scores[idx][yes,None]-scores[idx][~yes][None,:]
                pair_scores.append(float(np.mean((delta>0)+.5*(delta==0))))
        result[split]=dict(actions=len(indices),rejected_actions=int((~truth).sum()),
            false_accept_rate=float(prediction[~truth].mean()) if (~truth).any() else None,
            feasible_recall=float(prediction[truth].mean()) if truth.any() else None,
            mixed_states=len(pair_scores),mean_within_state_pair_ranking=float(np.mean(pair_scores)) if pair_scores else None,
            per_state_pair_ranking=pair_scores)
    return result


def main(a):
    require_memory_limit()
    root=a.root;out=root/'validation';out.mkdir(exist_ok=False)
    gate=json.loads((root/'data/gate.json').read_text());assert gate['passed']
    device=cuda_device();data=PairedData(root/'data',device)
    inputs={str(root/'data'/name):sha(root/'data'/name) for name in ('transitions.npz','groups.json','gate.json')}
    inputs[str(a.artifacts/'bc/sac_060000.pt')]=sha(a.artifacts/'bc/sac_060000.pt')
    fit_cases=sorted({g['case'] for g in data.fit_groups})
    demo_files=[a.artifacts/'demonstrations'/f"{case.replace('/','__')}.npz" for case in fit_cases]
    inputs.update({str(p):sha(p) for p in demo_files})
    demo=DemoReplay(demo_files,device)
    sources={str(p.resolve()):sha(p) for p in Path(__file__).parent.glob('gfr5_*.py')}
    protocol=dict(updates_per_model=a.updates,device=str(device),gpu=torch.cuda.get_device_name(),seed=11,
        actor_frozen=True,alpha_frozen=True,critic_modes=['paired_td','paired_td_terminal_anchors'],
        critic_start='original BC 60k with original optimizers/normalization/target networks',
        critic_batch='128 fitting-trajectory teacher transitions plus128 paired actions uniformly by fitting state; identical RNG across modes',
        fitting_teacher_cases=fit_cases,
        anchor_loss='additional per-stratum mean MSE for 32 true failures and32 true terminal successes, coefficient1 each',
        feasible_nonterminal_target='ordinary SAC Bellman target; NEVER artificially set to+1',
        classifier='independent fresh 130-256-256-1 network, same size as one critic; BCE; fit-only normalization',
        classifier_batch='192 uniform state/action examples plus32 matched good/bad pairs',
        classifier_interpretation='score at fixed threshold0.5, not calibrated safety probability',
        tests='fixed final weights only; no test-driven selection; original 60k critic pretraining saw test installations',
        diagnostic_thresholds=dict(false_accept_rate_max=.01,feasible_recall_min=.95,pair_ranking_min=.95,
                                    terminal_failure_mae_max=1.,terminal_success_mae_max=.25),
        thresholds_scope='exploratory diagnostic gates, not robot safety certification or paper acceptance criteria',
        full_rollouts=False,actor_training=False,online_latency_claim=False,inputs=inputs,sources=sources)
    dump(out/'protocol.json',protocol)
    summaries={};fit_val=('fit','trajectory_validation');all_splits=tuple(data.indices)
    for mode in ('paired_td','paired_td_terminal_anchors'):
        learner,identity=restore(a.artifacts,a.source,'bc');learner.actor.requires_grad_(False)
        learner.log_alpha.requires_grad_(False);alpha=learner.log_alpha.detach().clone()
        initial=value_metrics(learner,data,all_splits)
        rng=np.random.default_rng(11);anchor_rng=np.random.default_rng(12);history=[];start=time.perf_counter()
        for step in range(1,a.updates+1):
            paired=data.batch(data.uniform_states(rng,128));teacher=demo.sample(128)
            batch=tuple(torch.cat((old,new)) for old,new in zip(teacher,paired))
            anchor_batches=[data.batch(idx) for idx in data.anchors(anchor_rng)] if mode.endswith('anchors') else []
            losses=value_update(learner,batch,anchor_batches)
            if step%1000==0 or step==a.updates:
                row=dict(update=step,elapsed_s=time.perf_counter()-start,metrics=value_metrics(learner,data,fit_val),**losses)
                history.append(row);dump(out/f'{mode}_curve.json',history)
                dump(root/'status.json',dict(phase='critic_supervision_test',mode=mode,update=step,budget=a.updates))
                print(json.dumps(dict(mode=mode,**row)),flush=True)
        assert fingerprint(learner.actor)==identity['actor'] and torch.equal(learner.log_alpha,alpha)
        learner.save(out/f'{mode}.pt',60000)
        final=value_metrics(learner,data,all_splits)
        ok=all(row['failure']['mae']<=1 and row['failure']['positive_fraction']<=.01 and
               (row['success']['count']==0 or row['success']['mae']<=.25) for row in final.values())
        summaries[mode]=dict(initial=initial,final=final,actor_unchanged=True,alpha_unchanged=True,diagnostic_gate_passed=ok)
        dump(out/'results.json',summaries);del learner;torch.cuda.empty_cache()
    fit_groups=data.fit_groups
    observations=np.stack([data.arrays['observations'][g['start']] for g in fit_groups])
    torch.manual_seed(11)
    model=Critic(observations.mean(0),np.maximum(observations.std(0),.05)).to(device)
    optimizer=torch.optim.Adam(model.parameters(),lr=3e-4);rng=np.random.default_rng(11);history=[]
    for step in range(1,a.updates+1):
        ids=data.classification_batch(rng)
        logits=model(data.tensors['observations'][ids],data.tensors['actions'][ids]).ravel()
        loss=F.binary_cross_entropy_with_logits(logits,data.tensors['feasible'][ids])
        if not torch.isfinite(loss):raise FloatingPointError('Nonfinite classifier loss')
        optimizer.zero_grad(set_to_none=True);loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),10.);optimizer.step()
        if step%1000==0 or step==a.updates:
            row=dict(update=step,loss=float(loss.detach()),metrics=classification_metrics(model,data,fit_val))
            history.append(row);dump(out/'feasibility_curve.json',history)
            dump(root/'status.json',dict(phase='feasibility_supervision_test',update=step,budget=a.updates))
            print(json.dumps(dict(mode='feasibility',**row)),flush=True)
    final=classification_metrics(model,data,all_splits)
    ok=all(row['false_accept_rate']<=.01 and row['feasible_recall']>=.95 and row['mean_within_state_pair_ranking']>=.95
           for row in final.values())
    torch.save(dict(state_dict=model.state_dict(),role='diagnostic feasibility score only'),out/'feasibility.pt')
    summaries['feasibility']=dict(final=final,diagnostic_gate_passed=ok)
    assert all(sha(p)==h for p,h in {**inputs,**sources}.items())
    dump(out/'results.json',summaries)
    dump(root/'status.json',dict(phase='completed',execution_completed=True,
        diagnostic_gates={mode:row['diagnostic_gate_passed'] for mode,row in summaries.items()},
        actor_trained=False,source_and_input_artifacts_unchanged=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts','source','root'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--updates',type=int,default=10000)
    a=p.parse_args()
    for name in ('artifacts','source','root'):setattr(a,name,getattr(a,name).resolve())
    try:main(a)
    except BaseException as error:
        dump(a.root/'status.json',dict(phase='stopped_on_error',error=repr(error),execution_completed=False))
        raise

"""Coverage x local-information diagnostic, equal capacity and update budgets.

This fits immediate-feasibility and genuine-terminal-return heads. It is NOT
a trained SAC long-horizon critic or an updated Actor.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from tools.gfr1_remount_evaluate import dump,sha
from tools.gfr2_train_shared_bc import cuda_device
from tools.gfr4_common import require_memory_limit
from tools.gfr5_validate_supervision import terminal_stats


class DiagnosticModel(nn.Module):
    def __init__(self,mean,scale):
        super().__init__()
        self.register_buffer('mean',torch.as_tensor(mean,dtype=torch.float32))
        self.register_buffer('scale',torch.as_tensor(scale,dtype=torch.float32))
        self.net=nn.Sequential(nn.Linear(186,256),nn.LayerNorm(256),nn.ReLU(),
            nn.Linear(256,256),nn.LayerNorm(256),nn.ReLU(),nn.Linear(256,2))

    def forward(self,obs,action,local):
        return self.net(torch.cat(((obs-self.mean)/self.scale,action,local),dim=-1))


class Sampler:
    def __init__(self,arrays,groups,split,seed):
        self.arrays=arrays;self.groups=[g for g in groups if g['split']==split]
        self.mixed=[g for g in self.groups if g['mixed']]
        self.negative=[];self.success=[]
        for g in self.groups:
            ids=np.arange(g['start'],g['end']);flags=arrays['feasible'][ids]
            bad=ids[flags==0];good_end=ids[(flags==1)&(arrays['terminals'][ids]==1)]
            if len(bad):self.negative.append(bad)
            if len(good_end):self.success.append(good_end)
        assert self.mixed and self.negative and self.success
        self.rng=np.random.default_rng(seed)

    def classification(self):
        chosen=self.rng.integers(len(self.groups),size=192)
        ids=[self.rng.integers(self.groups[i]['start'],self.groups[i]['end']) for i in chosen]
        for j in self.rng.integers(len(self.mixed),size=32):
            group=self.mixed[j];pool=np.arange(group['start'],group['end'])
            flags=self.arrays['feasible'][pool]
            ids.extend((self.rng.choice(pool[flags==0]),self.rng.choice(pool[flags==1])))
        return np.asarray(ids)

    def terminals(self):
        result=[]
        for bank in (self.negative,self.success):
            selected=self.rng.integers(len(bank),size=32)
            result.extend(self.rng.choice(bank[i]) for i in selected)
        return np.asarray(result)


def metrics(model,tensors,arrays,groups,feature_mode,splits):
    local=tensors['local'] if feature_mode=='local' else torch.zeros_like(tensors['local'])
    with torch.no_grad():
        output=model(tensors['observations'],tensors['actions'],local)
        probability=output[:,0].sigmoid().cpu().numpy();value=(output[:,1]*10).cpu().numpy()
    result={}
    for split in splits:
        selected=[g for g in groups if g['split']==split]
        ids=np.concatenate([np.arange(g['start'],g['end']) for g in selected])
        good=arrays['feasible'][ids].astype(bool);predicted=probability[ids]>=.5
        pairs=[]
        for g in selected:
            if not g['mixed']:continue
            ii=np.arange(g['start'],g['end']);passed=arrays['feasible'][ii]==1
            delta=probability[ii][passed,None]-probability[ii][~passed][None,:]
            pairs.append(float(np.mean((delta>0)+.5*(delta==0))))
        reasons={}
        for kind in ('failure_joint','failure_clearance','failure_speed'):
            mask=(arrays[kind][ids]>0)&(~good)
            reasons[kind]=dict(count=int(mask.sum()),false_accept_rate=float(predicted[mask].mean()) if mask.any() else None)
        end=good&(arrays['terminals'][ids]==1)
        per_case={}
        for case in sorted({g['case'] for g in selected}):
            case_ids=np.concatenate([np.arange(g['start'],g['end']) for g in selected if g['case']==case])
            truth=arrays['feasible'][case_ids].astype(bool);prediction=probability[case_ids]>=.5
            ending=truth&(arrays['terminals'][case_ids]==1)
            per_case[case]=dict(actions=len(case_ids),rejected_actions=int((~truth).sum()),
                false_accept_rate=float(prediction[~truth].mean()) if (~truth).any() else None,
                feasible_recall=float(prediction[truth].mean()) if truth.any() else None,
                failure_value=terminal_stats(value[case_ids][~truth],arrays['rewards'][case_ids][~truth]),
                success_value=terminal_stats(value[case_ids][ending],arrays['rewards'][case_ids][ending]))
        result[split]=dict(states=len(selected),actions=len(ids),per_case=per_case,
            false_accept_rate=float(predicted[~good].mean()),feasible_recall=float(predicted[good].mean()),
            within_state_ranking=float(np.mean(pairs)),failure_types=reasons,
            failure_value=terminal_stats(value[ids][~good],arrays['rewards'][ids][~good]),
            success_value=terminal_stats(value[ids][end],arrays['rewards'][ids][end]))
    return result


def train(a):
    require_memory_limit();root=a.root;out=root/'models';out.mkdir(exist_ok=False)
    plan=json.loads((root/'training_plan.json').read_text())
    assert a.updates==plan['updates_per_model']
    assert all(sha(p)==h for p,h in plan['sources'].items())
    gate=json.loads((root/'data/gate.json').read_text());assert gate['passed']
    device=cuda_device()
    with np.load(root/'data/transitions.npz') as data:arrays={k:data[k].copy() for k in data.files}
    groups=json.loads((root/'data/groups.json').read_text())
    tensors={k:torch.as_tensor(v,device=device) for k,v in arrays.items()}
    # All four groups share the SAME normalization fitted only on narrow-fit states.
    narrow=np.stack([arrays['observations'][g['start']] for g in groups if g['split']=='narrow_fit'])
    mean=narrow.mean(0);scale=np.maximum(narrow.std(0),.05)
    held=('trajectory_validation','installation_test','fresh_test')
    fit_obs={arrays['observations'][g['start']].tobytes() for g in groups if g['split'].endswith('_fit')}
    test_obs={arrays['observations'][g['start']].tobytes() for g in groups if g['split'] in held}
    assert not (fit_obs & test_obs)
    for split in ('narrow_fit','broad_fit'):
        assert sum(g['split']==split for g in groups)==168
        assert all(g['end']-g['start']==26 for g in groups if g['split']==split)
    inputs={str(root/'data'/name):sha(root/'data'/name) for name in ('transitions.npz','groups.json','gate.json')}
    inputs[str(root/'training_plan.json')]=sha(root/'training_plan.json')
    sources={str(p.resolve()):sha(p) for p in Path(__file__).parent.glob('gfr6_*.py')}
    protocol=dict(seeds=[11,22,33],updates=a.updates,coverage=['narrow','broad'],features=['original','local'],
        device=str(device),gpu=torch.cuda.get_device_name(),parameter_count=sum(p.numel() for p in DiagnosticModel(mean,scale).parameters()),
        inputs_per_model=186,hidden_layers=[256,256],baseline_local_slots='56 zeros; identical parameter count',
        normalization='same narrow-fit state mean/std floor0.05 for128 inputs in every arm; fixed physical scales for56 local inputs',
        labels='head0 BCE immediate feasibility; head1 genuine terminal reward/10 MSE only; no targets on ordinary nonterminal Q',
        sampling='192 uniform state/actions +32 matched valid/invalid pairs; separate32 failure +32 terminal-success anchors',
        loss='BCE + MSE of terminal return divided by10; coefficient1',
        optimizer='Adam3e-4, clip10',seed_control='identical initialization and batch RNG within each coverage/feature pair',
        fit_states_per_coverage=168,fit_actions_per_coverage=4368,actor_frozen=True,long_horizon_critic_trained=False,
        test_protocol='fixed final budget, no test selection; entire trajectory IDs excluded from fitting',
        diagnostic_thresholds=dict(false_accept_rate_max=.01,feasible_recall_min=.95,within_state_ranking_min=.95,
                                  failure_value_mae_max=1.,success_value_mae_max=.25),
        gate_scope='diagnostic only, not deployment qualification',inputs=inputs,sources=sources)
    dump(out/'protocol.json',protocol)
    results=[];initials={}
    order=[('narrow','original'),('narrow','local'),('broad','original'),('broad','local')]
    for seed in (11,22,33):
        for coverage,feature in order:
            name=f'{coverage}_{feature}_seed{seed}';torch.manual_seed(seed)
            model=DiagnosticModel(mean,scale).to(device)
            fingerprint=__import__('hashlib').sha256(b''.join(p.detach().cpu().numpy().tobytes() for p in model.parameters())).hexdigest()
            if seed in initials:assert initials[seed]==fingerprint
            initials[seed]=fingerprint
            optimizer=torch.optim.Adam(model.parameters(),lr=3e-4)
            sampler=Sampler(arrays,groups,coverage+'_fit',seed)
            local=tensors['local'] if feature=='local' else torch.zeros_like(tensors['local'])
            started=time.perf_counter();history=[]
            for update in range(1,a.updates+1):
                ids=sampler.classification();anchor=sampler.terminals()
                logits=model(tensors['observations'][ids],tensors['actions'][ids],local[ids])[:,0]
                predicted=model(tensors['observations'][anchor],tensors['actions'][anchor],local[anchor])[:,1]
                assert bool(torch.all(tensors['terminals'][anchor]==1))
                classification=F.binary_cross_entropy_with_logits(logits,tensors['feasible'][ids])
                value=F.mse_loss(predicted,tensors['rewards'][anchor]/10)
                loss=classification+value
                if not torch.isfinite(loss):raise FloatingPointError('Nonfinite supervised loss')
                optimizer.zero_grad(set_to_none=True);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),10.);optimizer.step()
                if update%1000==0 or update==a.updates:
                    row=dict(update=update,classification_loss=float(classification.detach()),terminal_loss=float(value.detach()),elapsed_s=time.perf_counter()-started)
                    history.append(row);dump(out/f'{name}_curve.json',history)
                    dump(root/'status.json',dict(phase='fitting_controls',model=name,**row))
            torch.save(dict(state_dict=model.state_dict(),features=feature,coverage=coverage,
                            role='immediate-feasibility and terminal-calibration diagnostic, not SAC'),out/f'{name}.pt')
            measured=metrics(model,tensors,arrays,groups,feature,(coverage+'_fit',*held))
            passed=all(r['false_accept_rate']<=.01 and r['feasible_recall']>=.95 and r['within_state_ranking']>=.95 and
                       r['failure_value']['mae']<=1 and r['success_value']['count']>0 and r['success_value']['mae']<=.25
                       for r in measured.values())
            row=dict(seed=seed,coverage=coverage,features=feature,updates=a.updates,metrics=measured,diagnostic_gate_passed=passed)
            results.append(row);dump(root/'results.json',results)
            print(json.dumps(row),flush=True)
            del model,optimizer;torch.cuda.empty_cache()
    assert all(sha(p)==h for p,h in {**inputs,**sources}.items())
    summaries={}
    for coverage,feature in order:
        entries=[r for r in results if r['coverage']==coverage and r['features']==feature]
        summaries[f'{coverage}_{feature}']={}
        for split in held:
            summaries[f'{coverage}_{feature}'][split]={key:dict(mean=float(np.mean([r['metrics'][split][key] for r in entries])),
                min=float(np.min([r['metrics'][split][key] for r in entries])),max=float(np.max([r['metrics'][split][key] for r in entries])))
                for key in ('false_accept_rate','feasible_recall','within_state_ranking')}
    dump(root/'summary.json',dict(arms=summaries,all_models=12,inputs_unchanged=True,
        any_diagnostic_gate_passed=any(r['diagnostic_gate_passed'] for r in results),
        actor_trained=False,long_horizon_critic_trained=False))
    dump(root/'status.json',dict(phase='completed',execution_completed=True,
        qualified_models=[dict(seed=r['seed'],coverage=r['coverage'],features=r['features']) for r in results if r['diagnostic_gate_passed']],
        actor_trained=False,long_horizon_critic_trained=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--updates',type=int,default=10000);a=p.parse_args();a.root=a.root.resolve()
    try:train(a)
    except BaseException as error:
        dump(a.root/'status.json',dict(phase='stopped_on_error',error=repr(error),execution_completed=False));raise

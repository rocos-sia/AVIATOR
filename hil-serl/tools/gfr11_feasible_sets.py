"""J4 fixed-state supervision ablation: tie labels vs finite feasible sets.

No state aggregation, geometry thresholds, normalization or architecture changes.
Finite-set loss is NOT distance to the true continuous feasible region.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr10_student_aggregation import weighted_groups,new_label_audit
from tools.gfr9_local_bc import Policy,evaluate,fitting_audit
from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr7_n0i_feasibility import assess
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT,dump,sha
from tools.gfr4_common import require_memory_limit

SEEDS=(11,23,37)
GROUPS=('A_tie_continuity','B_top_set','B_all_set')
UPDATES=3000
TIE_TOL=1e-12


def get_candidates(old_root,j3,arrays,meta,old_count,seed):
    selected=set(np.flatnonzero(arrays['repair']).tolist());by_index={}
    with (old_root/'candidate_labels.jsonl').open() as stream:
        for line in stream:
            d=json.loads(line);i=d['sample_index']
            if i in selected:by_index[i]=d
    new={}
    with (j3/f'seed_{seed}'/'candidate_labels.jsonl').open() as stream:
        for line in stream:
            d=json.loads(line);new[(d['state_id'],d['step'])]=d
    for i in selected:
        if i>=old_count:by_index[i]=new[(meta[i]['state_id'],meta[i]['step'])]
    assert set(by_index)==selected
    return by_index


def prepare_sets(out,arrays,meta,candidates,records,actor,cache,status,seed):
    ids=np.flatnonzero(arrays['repair']);all_sets=[];top_sets=[];new_targets=[];stats=[]
    env=None;active_case=None
    for i in ids:
        d=candidates[int(i)]
        np.testing.assert_array_equal(np.asarray(d['state']),arrays['states'][i])
        np.testing.assert_array_equal(np.asarray(d['observation'],np.float32),arrays['observations'][i])
        good=[r for r in d['candidates'] if r['feasible']]
        assert good
        # Keep first appearance; remove aliases without averaging modes.
        unique={}
        for r in good:unique.setdefault(np.asarray(r['action'],np.float32).tobytes(),r)
        good=list(unique.values());best=max(r['score'] for r in good)
        top=[r for r in good if abs(r['score']-best)<=TIE_TOL]
        pool=np.asarray([r['action'] for r in good],np.float32)
        tied=np.asarray([r['action'] for r in top],np.float32)
        assert np.any(np.all(pool==arrays['actions'][i],axis=1))
        previous=np.asarray(arrays['states'][i,45:47]/1.5,np.float32)
        nearest=int(np.argmin(np.square(tied-previous).sum(1)))
        label=tied[nearest]
        key=(meta[i]['case'],arrays['states'][i].tobytes())
        if active_case!=meta[i]['case']:
            if env is not None:env.close()
            env=make_env(records[meta[i]['case']]);active_case=meta[i]['case']
        obs=unpack_state(env,arrays['states'][i]);raw=actor(obs)
        np.testing.assert_allclose(obs,arrays['observations'][i],atol=1e-6,rtol=0)
        if key not in cache:
            for candidate in d['candidates']:
                status(phase='candidate_pool_audit',seed=seed,sample=int(i))
                check=assess(env,candidate['action'])
                assert check['feasible']==candidate['feasible']
                assert abs(check['score']-candidate['score'])<1e-9
            cache.add(key)
        assert assess(env,label)['feasible']
        proposal=assess(env,raw)
        all_sets.append(pool);top_sets.append(tied);new_targets.append(label)
        closest=pool[np.argmin(np.square(pool-raw).sum(1))]
        stats.append(dict(index=int(i),feasible_count=len(pool),top_count=len(tied),
            old_label=arrays['actions'][i].tolist(),continuity_label=label.tolist(),previous_action=previous.tolist(),
            actor_action=raw.tolist(),actor_feasible=proposal['feasible'],
            finite_set_attraction=(closest-raw).tolist(),
            old_label_change_from_previous=float(np.linalg.norm(arrays['actions'][i]-previous)),
            relabeled_change_from_previous=float(np.linalg.norm(label-previous)),
            finite_set_distance=float(np.min(np.linalg.norm(pool-raw,axis=1))),
            changed_label=not np.array_equal(label,arrays['actions'][i])))
    if env is not None:env.close()
    def pad(sets):
        result=np.zeros((len(sets),max(map(len,sets)),2),np.float32)
        mask=np.zeros(result.shape[:2],bool)
        for j,s in enumerate(sets):result[j,:len(s)]=s;mask[j,:len(s)]=True
        return result,mask
    pools={}
    for name,sets in [('all',all_sets),('top',top_sets)]:
        pools[name],pools[name+'_mask']=pad(sets)
    target=arrays['actions'].copy();target[ids]=np.asarray(new_targets)
    np.savez_compressed(out/f'sets_seed{seed}.npz',repair_indices=ids,continuity_targets=target,**pools)
    dump(out/f'pool_audit_seed{seed}.json',dict(rows=stats,
        changed_labels=sum(s['changed_label'] for s in stats),
        already_feasible_but_nonzero_finite_distance=sum(s['actor_feasible'] and s['finite_set_distance']>1e-7 for s in stats),
        note='Finite sampling induces nonzero loss even inside the true feasible region; hard-min gradients may switch between equally close candidates'))
    return ids,target,pools


def train(out,initial,arrays,meta,ids,continuous,pools,group,status):
    reference=FrozenActor(initial);net=copy.deepcopy(reference.actor).requires_grad_(True)
    net.log_std.requires_grad_(False)
    optimizer=torch.optim.Adam([p for p in net.parameters() if p.requires_grad],lr=1e-4)
    saved=torch.load(initial,map_location='cuda:0',weights_only=True)
    optimizer.load_state_dict(saved['optimizer'])
    x=torch.as_tensor(arrays['observations'],device='cuda:0')
    y=torch.as_tensor(continuous if group=='A_tie_continuity' else arrays['actions'],device='cuda:0')
    old=torch.as_tensor(arrays['reference_actions'],device='cuda:0')
    batches=weighted_groups(arrays,meta,x.device)
    if group.startswith('B_'):
        kind='top' if group=='B_top_set' else 'all'
        points=torch.as_tensor(pools[kind],device='cuda:0')
        mask=torch.as_tensor(pools[kind+'_mask'],device='cuda:0')
        mapping=torch.full((len(x),),-1,device='cuda:0',dtype=torch.long)
        mapping[torch.as_tensor(ids,device='cuda:0')]=torch.arange(len(ids),device='cuda:0')
    torch.set_rng_state(saved['torch_rng'].cpu());torch.cuda.set_rng_state(saved['cuda_rng'].cpu())
    with (out/'training.jsonl').open('x',buffering=1) as log:
        for step in range(1,UPDATES+1):
            net.train();losses={}
            for flag,(indices,w) in batches.items():
                chosen=indices[torch.multinomial(w,128,replacement=True)]
                prediction=net.sample(x[chosen],True)[0]
                if flag and group.startswith('B_'):
                    positions=mapping[chosen];assert torch.all(positions>=0)
                    distances=(prediction[:,None,:]-points[positions]).square().mean(-1)
                    losses[flag]=distances.masked_fill(~mask[positions],torch.inf).min(-1).values.mean()
                else:losses[flag]=(prediction-(y if flag else old)[chosen]).square().mean()
            loss=losses[True]+losses[False];assert torch.isfinite(loss)
            optimizer.zero_grad(set_to_none=True);loss.backward()
            torch.nn.utils.clip_grad_norm_(net.parameters(),1.);optimizer.step()
            if step%100==0:
                row=dict(group=group,update=step,repair_loss=float(losses[True].detach()),
                    preserve_mse=float(losses[False].detach()),device='cuda:0')
                log.write(json.dumps(row)+'\n');status(phase='cuda_fixed_data_training',run=str(out.name),**row)
    net.eval();assert reference.unchanged()
    assert torch.equal(net.mean,reference.actor.mean) and torch.equal(net.scale,reference.actor.scale)
    for k,v in net.log_std.state_dict().items():assert torch.equal(v,reference.actor.log_std.state_dict()[k])
    checkpoint=out/'actor_006000.pt'
    torch.save(dict(actor=net.state_dict(),optimizer=optimizer.state_dict(),updates=6000,group=group,
        initial_sha256=sha(initial),torch_rng=torch.get_rng_state(),cuda_rng=torch.cuda.get_rng_state()),checkpoint)
    return net,checkpoint


def repair_audit(out,policy,arrays,meta,ids,records,label):
    rows=[]
    for case in sorted({meta[int(i)]['case'] for i in ids}):
        env=make_env(records[case])
        try:
            for i in ids:
                if meta[int(i)]['case']!=case:continue
                obs=unpack_state(env,arrays['states'][i]);c=assess(env,policy(obs))
                rows.append(dict(index=int(i),feasible=c['feasible'],reasons=c['reasons']))
        finally:env.close()
    report=dict(samples=len(rows),feasible=sum(r['feasible'] for r in rows),rows=rows)
    dump(out/'evaluations'/label/'all_repair_audit.json',report)
    return {k:v for k,v in report.items() if k!='rows'}


def motion_summary(directory):
    rows=[]
    for path in sorted(directory.glob('s0*.json')):
        episode=json.loads(path.read_text());changes=[]
        for row in episode['rows']:
            if not row['passed']:continue
            changes.append(float(np.linalg.norm(np.asarray(row['action'])-np.asarray(row['state_before'])[45:47]/1.5)))
        rows.append(dict(state=path.stem,completed=episode['completed'],accepted_steps=episode['accepted_steps'],
            action_change_mean=float(np.mean(changes)) if changes else None,
            action_change_p95=float(np.percentile(changes,95)) if changes else None,
            caution='Descriptive only: failed paths have different duration and state coverage'))
    return rows


def run(a):
    require_memory_limit();a.output.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    def status(**kw):dump(a.output/'status.json',dict(execution_completed=False,elapsed_s=time.monotonic()-started,**kw))
    j1=ROOT/'outputs/gfr_local_feasible_bc_gpu_20261127';j3=ROOT/'outputs/gfr_student_aggregation_gpu_20261127'
    j1results=json.loads((j1/'results.json').read_text());j3results=json.loads((j3/'results.json').read_text())
    initials={m['seed']:m for m in j1results['models'] if m['update']==3000}
    baselines={m['seed']:m for m in j3results['models'] if m['group']=='C'}
    source=ROOT/'outputs/gfr_foresight_n0_20261126/states.json'
    all_jobs=json.loads(source.read_text());jobs=[s for s in all_jobs if s['split']=='development']
    forbidden={s['record']['case'] for s in all_jobs if s['split']!='development'}
    records={r['case']:r for r in json.loads((ROOT/'outputs/gfr_shared_distill_gpu_20261126/dataset/train.json').read_text())['records']}
    with np.load(j1/'dataset.npz') as data:old={k:data[k].copy() for k in data.files}
    old_meta=json.loads((j1/'dataset_metadata.json').read_text())
    files=[Path(__file__),j1/'candidate_labels.jsonl',j3/'results.json',ROOT/'hil-serl/tools/gfr10_student_aggregation.py']
    for seed in SEEDS:
        files.extend([Path(initials[seed]['checkpoint']),Path(baselines[seed]['checkpoint']),
            j3/f'seed_{seed}/C/dataset.npz',j3/f'seed_{seed}/C/dataset_metadata.json',
            j3/f'seed_{seed}/candidate_labels.jsonl',j3/f'seed_{seed}/aggregated.npz',j3/f'seed_{seed}/aggregated_metadata.json'])
    hashes={str(p):sha(p) for p in files}
    deps=json.loads((j3/'protocol.json').read_text())['sources'];assert all(sha(p)==h for p,h in deps.items());hashes.update(deps)
    dump(a.output/'protocol.json',dict(groups=['0_MSE_reuse_J3C',*GROUPS],seeds=SEEDS,additional_updates=UPDATES,
        data='exact per-seed J3-C arrays, state distribution and preserve labels; no additions or adaptive resampling',
        repair_states='61 old plus 2 own student rejection states per seed; preserve stratum unchanged',
        A='among original best-score ties choose minimum distance to actual previous accepted action; remaining ties keep original order',
        B_top='minimum mean squared coordinate distance to original best-score tied feasible candidates',
        B_all='minimum mean squared coordinate distance to ALL recorded feasible candidates',
        loss_scale='mean over two action coordinates, matching original MSE scale; preserve loss unchanged',
        candidate_labels='reuse recorded 9x9 adaptive search; revalidate all candidates; no geometry threshold or quality score change',
        tie_tolerance=TIE_TOL,optimizer='restore same J1 seed-specific Actor, Adam, CPU/CUDA RNG;128 repair+128 preserve;equal MSE weights;case-balanced',
        baseline_verification='seed11 exact tensor reproduction required, then reuse frozen J3-C baseline results for all seeds',
        checker='beta1; full endpoint+swept unchanged; rejection terminates; no online candidate search',
        limitations='hard-min finite-set loss can penalize already feasible non-grid actions and has nonsmooth nearest-point switches; no continuous field or robust radius claim',
        diagnostics='same-state finite attraction vectors, tie-label displacement from previous action, accepted-action variation, full DirectOnly segments',
        meaningful_gate='at least one hard segment and all original8 completed in at least2/3 seeds; compare matched baseline',
        full_gate='10/10 in at least2/3 seeds',stop='one fixed-budget ablation; no residual, classifier, Critic, state aggregation, margin tuning or heldout evaluation',sources=hashes))
    install_guards();prepared={};cache=set()
    for seed in SEEDS:
        with np.load(j3/f'seed_{seed}/C/dataset.npz') as data:arrays={k:data[k].copy() for k in data.files}
        meta=json.loads((j3/f'seed_{seed}/C/dataset_metadata.json').read_text())
        assert not forbidden&{m['case'] for m in meta}
        model=FrozenActor(initials[seed]['checkpoint'])
        candidates=get_candidates(j1,j3,arrays,meta,len(old['repair']),seed)
        ids,target,pools=prepare_sets(a.output,arrays,meta,candidates,records,model,cache,status,seed)
        assert model.unchanged();prepared[seed]=(arrays,meta,ids,target,pools)
    baseline_dir=a.output/'baseline_reproduction_seed11';baseline_dir.mkdir()
    net,path=train(baseline_dir,initials[11]['checkpoint'],*prepared[11],group='0_MSE',status=status)
    expected=FrozenActor(baselines[11]['checkpoint'])
    identical=all(torch.equal(v,expected.actor.state_dict()[k]) for k,v in net.state_dict().items())
    dump(a.output/'baseline_reproduction.json',dict(seed=11,bitwise_identical=identical,
        reproduced_checkpoint=str(path),existing_checkpoint=baselines[11]['checkpoint']))
    assert identical,'Baseline numerical reproduction failed; stop before comparing methods'
    del net,expected
    results=[]
    for seed in SEEDS:
        baseline=copy.deepcopy(baselines[seed]);baseline.update(group='0_MSE',reused=True)
        baseline['action_variation']=motion_summary(j3/'evaluations'/baseline['model'])
        results.append(baseline)
        arrays,meta,ids,target,pools=prepared[seed]
        with np.load(j3/f'seed_{seed}/aggregated.npz') as data:new={k:data[k].copy() for k in data.files}
        new_meta=json.loads((j3/f'seed_{seed}/aggregated_metadata.json').read_text())
        for group in GROUPS:
            label=f'J4_{group}_seed{seed}';directory=a.output/label;directory.mkdir()
            net,checkpoint=train(directory,initials[seed]['checkpoint'],arrays,meta,ids,target,pools,group,status)
            report=evaluate(a.output,Policy(net),jobs,status,label)
            report['old_fitting']=fitting_audit(a.output,Policy(net),old,old_meta,records,status,label)
            report['new_fitting']=new_label_audit(a.output,Policy(net),new,new_meta,records,label)
            report['all_repair_fitting']=repair_audit(a.output,Policy(net),arrays,meta,ids,records,label)
            report.update(seed=seed,group=group,checkpoint=str(checkpoint),checkpoint_sha256=sha(checkpoint),
                fixed_data_sha256=sha(j3/f'seed_{seed}/C/dataset.npz'),reused=False,
                action_variation=motion_summary(a.output/'evaluations'/label))
            results.append(report);dump(a.output/'results.partial.json',results)
            del net
    summary={}
    for group in ('0_MSE',*GROUPS):
        models=[r for r in results if r['group']==group]
        summary[group]=dict(hard_completed=sum(m['difficult_completed'] for m in models),hard_attempts=6,
            old_retained=sum(m['retained_original_successes'] for m in models),old_attempts=24,
            meaningful_seeds=sum(m['difficult_completed']>=1 and m['retained_original_successes']==8 for m in models),
            full_gate_seeds=sum(m['passed'] for m in models))
    assert all(sha(p)==h for p,h in hashes.items())
    dump(a.output/'results.json',dict(models=results,summary=summary,inputs_unchanged=True,
        heldout_queried=False,residual_training_started=False,continuous_set_claim=False,
        gpu=torch.cuda.get_device_name(),elapsed_s=time.monotonic()-started))
    dump(a.output/'status.json',dict(phase='completed_fixed_data_ablation',execution_completed=True,summary=summary,
        residual_training_started=False,heldout_queried=False,elapsed_s=time.monotonic()-started))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output=a.output.resolve()
    if a.output.exists():p.error('Refuse existing output')
    try:run(a)
    except Exception as exc:
        if a.output.exists():
            dump(a.output/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            dump(a.output/'status.json',dict(phase='stopped_error',execution_completed=False,error=repr(exc)))
        raise

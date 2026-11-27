"""One controlled round of pure BC student-state aggregation (N0-J3).

A: old data; B: old + own terminal rejections; C: B + own pre-failure,
boundary and stable states. No new margins, auxiliary networks or RL updates.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr9_local_bc import Policy,evaluate,fitting_audit
from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr7_n0i_feasibility import assess,search,select_witness,execute
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT,dump,sha
from tools.audit_static_collision import CollisionChecker
from tools.gfr4_common import require_memory_limit

SEEDS=(11,23,37)
UPDATES=3000


def selection(rows,completed):
    selected={}
    def add(i,kind):
        if 0<=i<len(rows):selected.setdefault(i,[]).append(kind)
    if completed:
        for i in (0,125,249):add(i,'stable_retention')
    else:
        last=len(rows)-1;assert not rows[last]['passed']
        add(last,'first_rejection')
        for offset in (1,2,3,5,10):add(last-offset,'pre_failure')
        critical=[]
        for i,row in enumerate(rows[:-1]):
            c=row['info']
            if c['d_actual_m']-.005<=.00025 or c['joint_limit_margin_rad']<=.005 or 1.5-c['qdot_max_rad_s']<=.05:
                critical.append(i)
        if critical:
            for j in np.linspace(0,len(critical)-1,min(6,len(critical)),dtype=int):add(critical[j],'accepted_near_boundary')
        add(0,'initial_retention')
    return selected


def weighted_groups(arrays,meta,device):
    result={}
    for flag in (True,False):
        ids=np.flatnonzero(arrays['repair']==flag)
        cases=[meta[int(i)]['case'] for i in ids]
        counts={c:cases.count(c) for c in set(cases)}
        weights=np.asarray([1/counts[c] for c in cases],np.float32)
        result[flag]=(torch.as_tensor(ids,device=device),torch.as_tensor(weights,device=device))
    return result


def gradient_audit(net,arrays,meta):
    net.requires_grad_(True);net.log_std.requires_grad_(False)
    params=[p for p in net.parameters() if p.requires_grad]
    x=torch.as_tensor(arrays['observations'],device='cuda:0')
    targets={True:torch.as_tensor(arrays['actions'],device='cuda:0'),
             False:torch.as_tensor(arrays['reference_actions'],device='cuda:0')}
    gradients={};losses={}
    for flag,(ids,w) in weighted_groups(arrays,meta,x.device).items():
        per=(net.sample(x[ids],True)[0]-targets[flag][ids]).square().mean(-1)
        loss=(per*w).sum()/w.sum()
        gradients[flag]=torch.cat([g.ravel() for g in torch.autograd.grad(loss,params)])
        losses[str(flag)]=float(loss.detach())
    a,b=gradients[True],gradients[False];na,nb=a.norm(),b.norm()
    result=dict(repair_mse=losses['True'],preserve_mse=losses['False'],
        repair_gradient_norm=float(na),preserve_gradient_norm=float(nb),
        cosine=float((a@b)/(na*nb)) if float(na*nb)>1e-15 else None,
        interpretation='Exact case-balanced stratum mean gradients, before global norm clipping; negative cosine alone is not proof of a faulty loss')
    net.requires_grad_(False)
    return result


def old_audit(out,old,meta,models,records,status):
    repairs=np.flatnonzero(old['repair']);preserve=np.flatnonzero(~old['repair'])
    audits=[];pair_rows=[]
    for model in models:
        actor=FrozenActor(model['checkpoint']);scale=actor.actor.scale.detach().cpu().numpy()
        stats=dict(seed=model['seed'],gradient=gradient_audit(actor.actor,old,meta),
            valid_endpoint_pairs=0,interpolation_trials=0,infeasible_interior=0,old_teacher_rejected=0)
        for case in sorted({meta[int(i)]['case'] for i in repairs}):
            env=make_env(records[case])
            try:
                for i in repairs:
                    if meta[int(i)]['case']!=case:continue
                    status(phase='old_label_audit',seed=model['seed'],sample=int(i))
                    obs=unpack_state(env,old['states'][i]);student=actor(obs);teacher=old['actions'][i]
                    tc=assess(env,teacher);sc=assess(env,student)
                    stats['old_teacher_rejected']+=not tc['feasible'];assert tc['feasible']
                    if tc['feasible'] and sc['feasible']:
                        stats['valid_endpoint_pairs']+=1
                        for alpha in np.linspace(.1,.9,9):
                            check=assess(env,(1-alpha)*student+alpha*teacher)
                            stats['interpolation_trials']+=1
                            stats['infeasible_interior']+=not check['feasible']
                    # Same-case nearest retention label, with cross-state action
                    # validity checked at THIS state before any convexity claim.
                    if model['seed']==SEEDS[0]:
                        ids=np.asarray([j for j in preserve if meta[int(j)]['case']==case])
                        distances=np.linalg.norm((old['observations'][ids]-obs)/scale,axis=1)
                        j=int(ids[np.argmin(distances)])
                        other=old['reference_actions'][j];cross=assess(env,other)
                        interpolation=[]
                        if cross['feasible']:
                            interpolation=[assess(env,(1-alpha)*teacher+alpha*other)['feasible'] for alpha in np.linspace(.1,.9,9)]
                        pair_rows.append(dict(repair_index=int(i),preserve_index=j,
                            normalized_observation_l2=float(distances.min()),
                            action_l2=float(np.linalg.norm(teacher-other)),
                            preserve_label_feasible_at_repair_state=cross['feasible'],
                            feasible_endpoint_interpolation=interpolation))
            finally:env.close()
        assert actor.unchanged();audits.append(stats)
    temporal=[]
    for i in range(1,len(meta)):
        if meta[i]['segment']==meta[i-1]['segment'] and meta[i]['step']==meta[i-1]['step']+1:
            temporal.append(dict(index=i,repair_transition=bool(old['repair'][i]!=old['repair'][i-1]),
                action_l2=float(np.linalg.norm(old['actions'][i]-old['actions'][i-1])),
                observation_l2=float(np.linalg.norm((old['observations'][i]-old['observations'][i-1])/scale))))
    dump(out/'old_label_audit.json',dict(model_audits=audits,nearest_repair_preserve_pairs=pair_rows,
        temporal_pairs=temporal,batch_repair=128,batch_preserve=128,loss_coefficients=[1,1],
        sampling='equal cases inside each stratum; NOT sampling according to 61:3463 counts',
        close_large_jump_diagnostic=sum(r['normalized_observation_l2']<=1 and r['action_l2']>=.5 for r in pair_rows),
        warning='Near unequal labels and opposing gradients are diagnostics, not evidence that all feasible sets are disconnected'))


def gather(out,source,model,jobs,status):
    actor=FrozenActor(model['checkpoint']);samples=[];meta=[]
    directory=out/f"seed_{model['seed']}";directory.mkdir()
    with (directory/'candidate_labels.jsonl').open('x',buffering=1) as log:
        for job in jobs:
            sid=job['id'];path=source/'evaluations'/model['model']/f'{sid}.json'
            episode=json.loads(path.read_text());chosen=selection(episode['rows'],episode['completed'])
            env=make_env(job['record']);audit=CollisionChecker(job['record']['true_model'])
            try:
                for i,kinds in sorted(chosen.items()):
                    status(phase='student_state_labeling',seed=model['seed'],state=sid,step=i)
                    row=episode['rows'][i];state=row['state_before'];obs=unpack_state(env,state)
                    raw=actor(obs);np.testing.assert_array_equal(raw,np.asarray(row['action'],np.float32))
                    trial=assess(env,raw);assert trial['feasible']==row['passed']
                    scan=search(env,raw,dense=False);teacher=select_witness(scan,raw,'margin')
                    log.write(json.dumps(dict(source_checkpoint=model['checkpoint'],source_checkpoint_sha256=model['checkpoint_sha256'],
                        state_id=sid,step=i,categories=kinds,case=job['record']['case'],
                        state=state,observation=obs.tolist(),actor_action=raw.tolist(),candidates=scan['rows']))+'\n')
                    if teacher is None:raise RuntimeError('No finite teacher witness at selected state; stop before training')
                    target=raw if trial['feasible'] else np.asarray(teacher['action'],np.float32)
                    replay=execute(env,audit,target);assert replay['passed']
                    if trial['feasible']:
                        np.testing.assert_allclose(replay['state_after'],row['state_after'],atol=1e-10,rtol=0)
                    samples.append(dict(observations=obs,actions=target,reference_actions=raw,
                                        states=np.asarray(state),repair=not trial['feasible']))
                    meta.append(dict(case=job['record']['case'],segment=f"seed{model['seed']}_{sid}",step=i,
                        state_id=sid,source_seed=model['seed'],categories=kinds,repair=not trial['feasible'],
                        source_checkpoint_sha256=model['checkpoint_sha256'],source_episode_sha256=sha(path),
                        model_sha256=sha(job['record']['model']),trajectory_sha256=sha(job['record']['trajectory_path']),
                        teacher_action=teacher['action'],teacher_score=teacher['score'],
                        actor_check=trial['check'],label_rule='keep accepted student action; otherwise N0-I margin teacher'))
            finally:env.close()
    assert actor.unchanged()
    arrays={k:np.asarray([s[k] for s in samples]) for k in samples[0]}
    for k in ('observations','actions','reference_actions'):arrays[k]=arrays[k].astype(np.float32)
    np.savez_compressed(directory/'aggregated.npz',**arrays)
    dump(directory/'aggregated_metadata.json',meta)
    return arrays,meta


def combine(old,old_meta,new,new_meta,group):
    ids=np.asarray([],int) if group=='A' else np.flatnonzero(new['repair']) if group=='B' else np.arange(len(new['repair']))
    result={k:np.concatenate([old[k],new[k][ids]]) for k in old}
    return result,old_meta+[new_meta[int(i)] for i in ids]


def new_label_audit(out,policy,new,meta,records,label):
    rows=[]
    for case in sorted({m['case'] for m in meta}):
        env=make_env(records[case])
        try:
            for i,m in enumerate(meta):
                if m['case']!=case:continue
                obs=unpack_state(env,new['states'][i]);check=assess(env,policy(obs))
                rows.append(dict(index=i,repair=bool(new['repair'][i]),categories=m['categories'],
                    feasible=check['feasible'],reasons=check['reasons']))
        finally:env.close()
    result={name:dict(total=sum(r['repair']==flag for r in rows),
        feasible=sum(r['repair']==flag and r['feasible'] for r in rows)) for name,flag in [('new_failures',True),('context',False)]}
    dump(out/'evaluations'/label/'new_state_audit.json',dict(summary=result,rows=rows))
    return result


def run(a):
    require_memory_limit();a.output.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    def status(**kw):dump(a.output/'status.json',dict(execution_completed=False,elapsed_s=time.monotonic()-started,**kw))
    source=ROOT/'outputs/gfr_local_feasible_bc_gpu_20261127'
    previous=json.loads((source/'results.json').read_text())
    models=[m for m in previous['models'] if m['update']==3000]
    assert [m['seed'] for m in models]==list(SEEDS)
    state_path=ROOT/'outputs/gfr_foresight_n0_20261126/states.json'
    all_jobs=json.loads(state_path.read_text());jobs=[s for s in all_jobs if s['split']=='development']
    records={r['case']:r for r in json.loads((ROOT/'outputs/gfr_shared_distill_gpu_20261126/dataset/train.json').read_text())['records']}
    with np.load(source/'dataset.npz') as data:old={k:data[k].copy() for k in data.files}
    old_meta=json.loads((source/'dataset_metadata.json').read_text())
    heldout_cases={s['record']['case'] for s in all_jobs if s['split']!='development'}
    assert not heldout_cases&{m['case'] for m in old_meta}
    files=[Path(__file__),state_path,source/'results.json',source/'dataset.npz',source/'dataset_metadata.json',
           ROOT/'hil-serl/tools/gfr9_local_bc.py',ROOT/'hil-serl/tools/gfr7_n0i_feasibility.py']
    for model in models:
        files.append(Path(model['checkpoint']))
        files.extend(source/'evaluations'/model['model']/f"{j['id']}.json" for j in jobs)
    hashes={str(p):sha(p) for p in files}
    deps=json.loads((source/'protocol.json').read_text())['sources']
    assert all(sha(p)==h for p,h in deps.items());hashes.update(deps)
    dump(a.output/'protocol.json',dict(rounds=1,seeds=SEEDS,additional_updates=UPDATES,
        groups={'A':'old data only','B':'old plus own first-rejection states','C':'B plus own accepted pre-failure, critical and stable states'},
        paired_initialization='same seed-specific J1 update3000 Actor AND Adam state AND saved RNG for A/B/C',
        data_pooling='each seed aggregates its own student states; do not mix conflicting same-observation preservation labels across students',
        replay='reuse frozen actual DirectOnly trajectories; selected states restore exact Actor output and accepted successor',
        selected_context='failure offsets1/2/3/5/10; up to6 evenly spread critical accepted states; step0; successful segments steps0/125/249',
        critical_sampling_only=dict(clearance_margin_m=.00025,joint_margin_rad=.005,speed_margin_rad_s=.05),
        teacher='N0-I 9x9 adaptive margin rule; preserve any accepted student action; critical thresholds select data only, never alter acceptance or labels',
        optimization='Adam1e-4, repair128/preserve128 per batch, equal MSE coefficients, equal cases per stratum, gradient clip1',
        evaluation='one forward per step, beta1, original DirectOnly checker, first rejection terminates; ten development segments',
        meaningful_gain='at least one difficult segment completed and old8 retained in at least2/3 seeds; compare paired completions against A',
        full_gate='both difficult segments completed and old8 retained in at least2/3 seeds',
        stop='end of one round regardless of outcome; no automatic second round, classifier, Critic, foresight or heldout evaluation',
        audit='nearest repair/preserve labels; temporal target jumps; interpolation only between verified feasible endpoints; exact expected-stratum gradients',
        sources=hashes))
    install_guards();old_audit(a.output,old,old_meta,models,records,status)
    new_sets={}
    for model in models:new_sets[model['seed']]=gather(a.output,source,model,jobs,status)
    results=[]
    for model in models:
        new,new_meta=new_sets[model['seed']]
        for group in ('A','B','C'):
            label=f"J3{group}_seed{model['seed']}"
            arrays,meta=combine(old,old_meta,new,new_meta,group)
            directory=a.output/f"seed_{model['seed']}"/group;directory.mkdir()
            np.savez_compressed(directory/'dataset.npz',**arrays)
            dump(directory/'dataset_metadata.json',meta)
            reference=FrozenActor(model['checkpoint']);net=copy.deepcopy(reference.actor).requires_grad_(True)
            net.log_std.requires_grad_(False)
            optimizer=torch.optim.Adam([p for p in net.parameters() if p.requires_grad],lr=1e-4)
            saved=torch.load(model['checkpoint'],map_location='cuda:0',weights_only=True)
            optimizer.load_state_dict(saved['optimizer'])
            x=torch.as_tensor(arrays['observations'],device='cuda:0')
            targets={True:torch.as_tensor(arrays['actions'],device='cuda:0'),False:torch.as_tensor(arrays['reference_actions'],device='cuda:0')}
            groups=weighted_groups(arrays,meta,x.device)
            torch.set_rng_state(saved['torch_rng'].cpu());torch.cuda.set_rng_state(saved['cuda_rng'].cpu())
            with (directory/'training.jsonl').open('x',buffering=1) as log:
                for step in range(1,UPDATES+1):
                    net.train();losses={}
                    for flag,(ids,w) in groups.items():
                        picked=ids[torch.multinomial(w,128,replacement=True)]
                        losses[flag]=(net.sample(x[picked],True)[0]-targets[flag][picked]).square().mean()
                    loss=losses[True]+losses[False];assert torch.isfinite(loss)
                    optimizer.zero_grad(set_to_none=True);loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(),1.);optimizer.step()
                    if step%100==0:
                        row=dict(model=label,additional_update=step,repair_mse=float(losses[True].detach()),
                            preserve_mse=float(losses[False].detach()),repair_batch=128,preserve_batch=128,device='cuda:0')
                        log.write(json.dumps(row)+'\n');status(phase='cuda_training',**row)
            net.eval()
            assert torch.equal(net.mean,reference.actor.mean) and torch.equal(net.scale,reference.actor.scale)
            for key,value in net.log_std.state_dict().items():assert torch.equal(value,reference.actor.log_std.state_dict()[key])
            checkpoint=directory/'actor_006000.pt'
            torch.save(dict(actor=net.state_dict(),optimizer=optimizer.state_dict(),updates=6000,
                source_checkpoint_sha256=model['checkpoint_sha256'],data_sha256=sha(directory/'dataset.npz'),
                torch_rng=torch.get_rng_state(),cuda_rng=torch.cuda.get_rng_state()),checkpoint)
            report=evaluate(a.output,Policy(net),jobs,status,label)
            report['old_fitting']=fitting_audit(a.output,Policy(net),old,old_meta,records,status,label)
            report['new_fitting']=new_label_audit(a.output,Policy(net),new,new_meta,records,label)
            report.update(seed=model['seed'],group=group,checkpoint=str(checkpoint),checkpoint_sha256=sha(checkpoint),
                old_samples=len(old['repair']),added_samples=len(arrays['repair'])-len(old['repair']),
                added_repairs=int(arrays['repair'].sum()-old['repair'].sum()))
            assert reference.unchanged()
            results.append(report);dump(a.output/'results.partial.json',results)
            del optimizer,net,reference,saved,x,targets
    summary={}
    for group in ('A','B','C'):
        rows=[r for r in results if r['group']==group]
        summary[group]=dict(difficult_completed=sum(r['difficult_completed'] for r in rows),difficult_attempts=6,
            old_retained=sum(r['retained_original_successes'] for r in rows),old_attempts=24,
            meaningful_seeds=sum(r['difficult_completed']>=1 and r['retained_original_successes']==8 for r in rows),
            full_gate_seeds=sum(r['passed'] for r in rows))
    assert all(sha(p)==h for p,h in hashes.items())
    dump(a.output/'results.json',dict(models=results,summary=summary,rounds=1,
        sources_unchanged=True,heldout_queried=False,critic_updates=0,auxiliary_training=False,
        gpu=torch.cuda.get_device_name(),elapsed_s=time.monotonic()-started))
    dump(a.output/'status.json',dict(phase='completed_one_round',execution_completed=True,summary=summary,
        second_round_started=False,heldout_queried=False,elapsed_s=time.monotonic()-started))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();a.output=a.output.resolve()
    if a.output.exists():parser.error('Refuse existing output')
    try:run(a)
    except Exception as exc:
        if a.output.exists():
            dump(a.output/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            dump(a.output/'status.json',dict(phase='stopped_error',execution_completed=False,error=repr(exc)))
        raise

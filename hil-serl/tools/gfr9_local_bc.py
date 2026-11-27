"""N0-J1: bounded CUDA BC repair with behavior retention, DirectOnly evaluation.

Original experiments/checkpoints are read-only. No SAC/Critic updates, iterative
aggregation, auxiliary classifier, protected evaluation, or holdout queries.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr7_n0i_feasibility import assess, search, select_witness, execute, trajectory
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import pack_state, unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT, dump, sha
from tools.audit_static_collision import CollisionChecker
from tools.gfr4_common import require_memory_limit

SEEDS=(11,23,37)
CHECKPOINTS=(500,1500,3000)


class Policy:
    def __init__(self,net):self.net=net
    @torch.inference_mode()
    def __call__(self,obs):
        return self.net.sample(torch.as_tensor(obs,dtype=torch.float32,device='cuda:0'),True)[0].cpu().numpy()


def collect(out,jobs,records,reference,status):
    xs=[];ys=[];old=[];states=[];repair=[];metadata=[];segments=[]
    candidate_log=(out/'candidate_labels.jsonl').open('x',buffering=1)
    demo_root=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/demonstrations'
    def segment(record,state,target,name,expected=None):
        env=make_env(record);audit=CollisionChecker(record['true_model'])
        unpack_state(env,state);count=0;changed=0;failure=None
        try:
            while env._u<target-1e-9:
                status(phase='collecting_teacher_data',segment=name,step=count,samples=len(xs))
                before=pack_state(env);obs=env.last_policy_observation.copy();raw=reference(obs)
                trial=assess(env,raw);rows=[trial];chosen=trial
                if not trial['feasible']:
                    scan=search(env,raw,dense=False);rows=scan['rows']
                    chosen=select_witness(scan,raw,'margin')
                index=len(xs)
                candidate_log.write(json.dumps(dict(sample_index=index if chosen else None,
                    segment=name,case=record['case'],state=before.tolist(),observation=obs.tolist(),
                    candidates=rows,teacher_available=chosen is not None))+'\n')
                if chosen is None:
                    failure='finite_teacher_search_found_no_witness';break
                action=np.asarray(chosen['action'],np.float32)
                if expected is not None:
                    np.testing.assert_array_equal(action,np.asarray(expected[count]['action'],np.float32))
                    np.testing.assert_allclose(before,expected[count]['state_before'],atol=1e-10,rtol=0)
                row=execute(env,audit,action)
                assert row['passed']
                xs.append(obs);ys.append(action);old.append(raw);states.append(before)
                repair.append(not trial['feasible'])
                metadata.append(dict(segment=name,step=count,case=record['case'],
                    repair=not trial['feasible'],teacher_score=chosen['score'],
                    clearance_margin_m=chosen['check']['d_actual_m']-env.d_safe,
                    joint_margin_rad=chosen['check']['joint_limit_margin_rad'],
                    speed_margin_rad_s=env.qdot_max-chosen['check']['qdot_max_rad_s']))
                changed+=not trial['feasible'];count+=1
            row=dict(segment=name,case=record['case'],steps=count,repairs=changed,
                completed=env._u>=target-1e-9,failure=failure)
            segments.append(row);dump(out/'collection_progress.json',segments)
        finally:env.close()
    try:
        for job in jobs:
            sid=job['id'];expected=None
            if sid in ('s008','s009'):
                path=ROOT/f'outputs/gfr_foresight_n0i_20261127/{sid}_rescue_margin.json'
                expected=json.loads(path.read_text())['rows']
            segment(job['record'],job['state'],job['state'][0]+125,sid,expected)
        for record in records:
            path=demo_root/f"{record['case'].replace('/','__')}.npz"
            with np.load(path) as data: demo=data['states'].copy()
            for fraction in (.35,.65):
                frame=min(int(fraction*len(demo)),len(demo)-65)
                segment(record,demo[frame],demo[frame,0]+32,
                        record['case'].replace('/','__')+f'_frame{frame}')
    finally:candidate_log.close()
    arrays=dict(observations=np.asarray(xs,np.float32),actions=np.asarray(ys,np.float32),
                reference_actions=np.asarray(old,np.float32),states=np.asarray(states),
                repair=np.asarray(repair,bool))
    assert arrays['repair'].any() and (~arrays['repair']).any()
    np.savez_compressed(out/'dataset.npz',**arrays)
    dump(out/'dataset_metadata.json',metadata)
    dump(out/'collection.json',dict(samples=len(xs),repair_samples=sum(repair),
        preserve_samples=len(xs)-sum(repair),segments=segments,
        candidate_labels_sha256=sha(out/'candidate_labels.jsonl'),dataset_sha256=sha(out/'dataset.npz'),
        retained_partial_teacher_segments=True,all_recorded_transitions_direct_audited=True))
    return arrays,metadata


def evaluate(out,policy,jobs,status,label):
    directory=out/'evaluations'/label;directory.mkdir(parents=True,exist_ok=False)
    results=[]
    for job in jobs:
        env=make_env(job['record']);audit=CollisionChecker(job['record']['true_model'])
        try:
            status(phase='independent_development_evaluation',model=label,state=job['id'])
            result=trajectory(env,audit,policy,job['state'],job['state'][0]+125)
            dump(directory/f"{job['id']}.json",result)
            failed=result['rows'][-1]['info'] if not result['completed'] else None
            results.append(dict(state=job['id'],completed=result['completed'],
                accepted_steps=result['accepted_steps'],progress_s=result['accepted_progress_s'],
                attempts=len(result['rows']),first_rejection=failed))
        finally:env.close()
    good={r['state'] for r in results if r['completed']}
    report=dict(model=label,completed=len(good),states=10,
        difficult_completed=sum(s in good for s in ('s008','s009')),
        retained_original_successes=sum(f's{i:03d}' in good for i in range(8)),
        passed=len(good)==10,rows=results,teacher_calls=0,projection_calls=0,
        governor_calls=0,recovery_calls=0,full_task_claim=False,latency_claim=False)
    dump(directory/'summary.json',report)
    return report


def fitting_audit(out,policy,arrays,meta,all_records,status,label):
    # Fixed, stratified in-sample diagnostics. Not a heldout accuracy claim.
    chosen=[]
    for flag in (True,False):
        indices=np.flatnonzero(arrays['repair']==flag)
        chosen.extend(indices[np.linspace(0,len(indices)-1,min(128,len(indices)),dtype=int)].tolist())
    rows=[]
    for case in sorted({meta[i]['case'] for i in chosen}):
        env=make_env(all_records[case])
        try:
            for i in chosen:
                if meta[i]['case']!=case:continue
                status(phase='in_sample_direct_action_audit',model=label,sample=i)
                obs=unpack_state(env,arrays['states'][i])
                np.testing.assert_allclose(obs,arrays['observations'][i],atol=1e-6,rtol=0)
                check=assess(env,policy(obs))
                rows.append(dict(index=i,repair=bool(arrays['repair'][i]),
                    feasible=check['feasible'],reasons=check['reasons'],action=check['action']))
        finally:env.close()
    report={name:dict(samples=sum(r['repair']==flag for r in rows),
        feasible=sum(r['repair']==flag and r['feasible'] for r in rows))
        for name,flag in [('repair',True),('preserve',False)]}
    dump(out/'evaluations'/label/'fitting_audit.json',dict(summary=report,rows=rows))
    return report


def run(a):
    require_memory_limit();a.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic()
    def status(**kw):dump(a.output/'status.json',dict(execution_completed=False,elapsed_s=time.monotonic()-start,**kw))
    source=ROOT/'outputs/gfr_foresight_n0_20261126/states.json'
    all_jobs=json.loads(source.read_text());jobs=[j for j in all_jobs if j['split']=='development']
    excluded={j['record']['case'] for j in all_jobs if j['split']!='development'}
    manifest=ROOT/'outputs/gfr_shared_distill_gpu_20261126/dataset/train.json'
    all_records={r['case']:r for r in json.loads(manifest.read_text())['records']}
    extra=[r for c,r in all_records.items() if c.split('/')[0] in ('train_02','train_03','train_04','train_05')]
    training_cases={j['record']['case'] for j in jobs}|{r['case'] for r in extra}
    assert not training_cases&excluded and len(jobs)==10 and len(extra)==8
    checkpoint=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/bc/sac_060000.pt'
    inputs=[Path(__file__),source,manifest,checkpoint,
        ROOT/'hil-serl/tools/gfr7_n0i_feasibility.py',ROOT/'hil-serl/tools/gfr7_n0_qualify.py',
        ROOT/'hil-serl/tools/gfr1_direct_only_env.py',ROOT/'hil-serl/tools/gfr3_sac.py',
        ROOT/'hil-serl/tools/gfr2_shared_env.py',ROOT/'hil-serl/tools/gfr3_demo_transitions.py',
        ROOT/'hil-serl/tools/gfr1_shared_remount_env.py',ROOT/'hil-serl/tools/gfr1_remount_evaluate.py',
        ROOT/'hil-serl/examples/experiments/aviator_manifold/gfr_env.py']
    for case in training_cases:
        for key in ('model','true_model','trajectory_path'):inputs.append(Path(all_records[case][key]))
    for record in extra:
        inputs.append(ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/demonstrations'/f"{record['case'].replace('/','__')}.npz")
    for sid in ('s008','s009'):inputs.append(ROOT/f'outputs/gfr_foresight_n0i_20261127/{sid}_rescue_margin.json')
    hashes={str(p):sha(p) for p in inputs}
    protocol=dict(stage='N0-J1 initial supervised repair only',seeds=SEEDS,updates=CHECKPOINTS,
        lr=1e-4,batch_repair=128,batch_preserve=128,loss='repair MSE + preserve MSE; equal coefficients',
        normalization='unchanged original 60k buffers',initialization='same original 60k Actor for each seed',
        trainable='mean action network only; log_std frozen; no Critic/optimizer transfer',
        sampling='equal case weight within repair and preserve strata',
        teacher='exact N0-I rescue_margin: retain any directly feasible original Actor action; otherwise max fixed margin score in 9x9 adaptive search',
        additional_data='train_02..05, traj_0000/0001; 35% and 65% demo anchors, 64 steps each',
        training_cases=sorted(training_cases),excluded_case_combinations=sorted(excluded),
        labels='repair: local accepted teacher; preserve: original Actor accepted output',
        eval='10 original development segments, 250 steps each; beta1; reject terminates; no search',
        gate='same scheduled update must yield 10/10 for at least 2 of 3 seeds',
        model_selection='earliest scheduled update satisfying gate; descriptive best also reported',
        if_gate_fails='stop; diagnose fitting vs rollout. No automatic extra updates, new labels or auxiliary training',
        next_stage='separate decision; this run never reads holdout outcomes or starts A2/A3',
        timing='concurrent diagnostic only; no microsecond or end-to-end speed claim',sources=hashes)
    dump(a.output/'protocol.json',protocol)
    install_guards();reference=FrozenActor(checkpoint)
    status(phase='baseline_evaluation',device='cuda:0')
    baseline=evaluate(a.output,reference,jobs,status,'A0_original60k')
    assert baseline['completed']==8 and baseline['retained_original_successes']==8
    arrays,meta=collect(a.output,jobs,extra,reference,status)
    device=reference.device
    x=torch.as_tensor(arrays['observations'],device=device)
    y=torch.as_tensor(arrays['actions'],device=device)
    old=torch.as_tensor(arrays['reference_actions'],device=device)
    groups={}
    for flag in (True,False):
        indices=np.flatnonzero(arrays['repair']==flag)
        counts={c:sum(meta[int(i)]['case']==c for i in indices) for c in training_cases}
        groups[flag]=(torch.as_tensor(indices,device=device),
            torch.as_tensor([1/counts[meta[int(i)]['case']] for i in indices],dtype=torch.float32,device=device))
    results=[]
    for seed in SEEDS:
        torch.manual_seed(seed);torch.cuda.manual_seed_all(seed)
        net=copy.deepcopy(reference.actor).requires_grad_(True)
        net.log_std.requires_grad_(False)
        optimizer=torch.optim.Adam([p for p in net.parameters() if p.requires_grad],lr=1e-4)
        directory=a.output/f'seed_{seed}';directory.mkdir()
        with (directory/'training.jsonl').open('x',buffering=1) as log:
            for step in range(1,max(CHECKPOINTS)+1):
                net.train();sample={}
                for flag,(ids,weights) in groups.items():sample[flag]=ids[torch.multinomial(weights,128,replacement=True)]
                repair_loss=(net.sample(x[sample[True]],True)[0]-y[sample[True]]).square().mean()
                preserve_loss=(net.sample(x[sample[False]],True)[0]-old[sample[False]]).square().mean()
                loss=repair_loss+preserve_loss
                assert torch.isfinite(loss)
                optimizer.zero_grad(set_to_none=True);loss.backward()
                torch.nn.utils.clip_grad_norm_(net.parameters(),1.);optimizer.step()
                if step%100==0:
                    row=dict(seed=seed,update=step,repair_mse=float(repair_loss.detach()),
                        preserve_mse=float(preserve_loss.detach()),device=str(next(net.parameters()).device))
                    log.write(json.dumps(row)+'\n');status(phase='cuda_bc_repair_training',**row)
                if step in CHECKPOINTS:
                    net.eval();label=f'A1_seed{seed}_update{step}'
                    assert torch.equal(net.mean,reference.actor.mean) and torch.equal(net.scale,reference.actor.scale)
                    for key,v in net.log_std.state_dict().items():assert torch.equal(v,reference.actor.log_std.state_dict()[key])
                    path=directory/f'actor_{step:06d}.pt'
                    torch.save(dict(actor=net.state_dict(),seed=seed,updates=step,
                        optimizer=optimizer.state_dict(),source_sha256=sha(checkpoint),
                        data_sha256=sha(a.output/'dataset.npz'),torch_rng=torch.get_rng_state(),
                        cuda_rng=torch.cuda.get_rng_state()),path)
                    report=evaluate(a.output,Policy(net),jobs,status,label)
                    report['fitting']=fitting_audit(a.output,Policy(net),arrays,meta,all_records,status,label)
                    report.update(seed=seed,update=step,checkpoint=str(path),checkpoint_sha256=sha(path))
                    results.append(report);dump(a.output/'results.partial.json',results)
        del optimizer,net
    passing=[step for step in CHECKPOINTS if sum(r['update']==step and r['passed'] for r in results)>=2]
    assert reference.unchanged() and all(sha(p)==h for p,h in hashes.items())
    dump(a.output/'results.json',dict(baseline=baseline,models=results,passed=bool(passing),
        selected_update=passing[0] if passing else None,gpu=torch.cuda.get_device_name(),
        source_unchanged=True,heldout_queried=False,sac_updates=0,critic_updates=0,
        elapsed_s=time.monotonic()-start))
    dump(a.output/'status.json',dict(phase='completed_gate_passed' if passing else 'stopped_development_gate',
        execution_completed=True,scientific_gate_passed=bool(passing),selected_update=passing[0] if passing else None,
        trained_seeds=len(SEEDS),models_evaluated=len(results),heldout_queried=False,
        auxiliary_training_started=False,state_aggregation_started=False,elapsed_s=time.monotonic()-start))


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

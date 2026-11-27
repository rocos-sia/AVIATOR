"""Matched-size motion-coverage data with label-independent local features."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from tools.gfr1_remount_evaluate import ROOT,dump,sha,audit_motion
from tools.gfr1_remount_pilot import initialize
from tools.gfr1_clearance_variants import use_projector
from tools.audit_static_collision import CollisionChecker
from tools.gfr2_teacher import make_env,choose_teacher
from tools.gfr3_demo_transitions import pack_state,unpack_state
from tools.gfr4_common import require_memory_limit
from tools.gfr5_pair_data import anchors


def local_features(h,basis,lower_margin,upper_margin,action,dt,rmax,qmax):
    """First-order prediction from the CURRENT lift, before any candidate check."""
    velocity=h+basis@(np.asarray(action)*rmax)
    movement=dt*velocity
    return np.r_[h/qmax,velocity/qmax,
        np.clip((lower_margin+movement)/(dt*qmax),-10,10),
        np.clip((upper_margin-movement)/(dt*qmax),-10,10)].astype(np.float32)


def select_seven(obs,actions):
    if len(obs)<7:raise ValueError('Need at least seven audited states per allocated task')
    # Seven distinct anchors: endpoint, both slide directions, peak turn speed,
    # closest joint/clearance boundary, and a middle point.
    choices=[len(obs)-1,int(np.argmin(obs[:,3])),int(np.argmax(obs[:,3])),
             int(np.argmax(np.abs(obs[:,2]))),int(np.argmin(obs[:,31])),
             int(np.argmin(obs[:,30])),len(obs)//2]
    selected=[]
    for wanted in choices:
        candidates=np.argsort(np.abs(np.arange(len(obs))-wanted),kind='stable')
        selected.append(next(int(i) for i in candidates if i not in selected))
    return sorted(selected)


def collect(record,root):
    name=record['case'].replace('/','__');path=root/'teacher'/f'{name}.npz'
    env=make_env(record);audit=CollisionChecker(record['true_model'])
    obs=[];states=[];actions=[];termination='initialization_invalid';started=time.perf_counter()
    try:
        budget=int(np.ceil((len(env._traj['x'])-1)*env.traj_dt/env.dt))+1
        with use_projector('combined'):
            for step in range(budget):
                before=env.last_policy_observation.copy();state=pack_state(env)
                action,selection=choose_teacher(env,step)
                if action is None:
                    termination='teacher_no_full_speed_candidate';break
                x,q=env._x.copy(),env._q.copy()
                _,_,done,truncated,info=env.step(action)
                if not env._last_direct_accepted:raise AssertionError('Teacher label failed direct replay')
                audit_result=audit_motion(env,audit,x,q)
                if audit_result['violations']:raise AssertionError(audit_result['violations'])
                obs.append(before);states.append(state);actions.append(action);termination=info['termination']
                if step%200==0:
                    dump(root/'status.json',dict(phase='collecting_training_paths',case=record['case'],step=step,progress=info['progress']))
                if done or truncated:break
    finally:
        env.close()
    np.savez_compressed(path,observations=np.asarray(obs,np.float32).reshape(-1,128),
                        states=np.asarray(states).reshape(-1,53),actions=np.asarray(actions,np.float32).reshape(-1,2))
    row=dict(case=record['case'],steps=len(actions),completed=termination=='end',termination=termination,
             file=str(path),elapsed_s=time.perf_counter()-started,accepted_audit_violations=0)
    dump(path.with_suffix('.json'),row);print(json.dumps(row),flush=True)
    return row


def prepare(a):
    require_memory_limit();root=a.root;root.mkdir(parents=True,exist_ok=False)
    (root/'teacher').mkdir();(root/'data').mkdir()
    records=json.loads((a.source/'dataset/train.json').read_text())['records']
    original={r['case']:r for r in records};models={r['case'].split('/')[0]:r['model'] for r in records}
    trajectories=ROOT/'hil-serl/data/aviator/trajectory_source/trajs/rl_train'
    selected=[3,6,7];fresh=[40,41]
    inputs={str(p):sha(p) for p in (a.artifacts/'demonstrations').glob('*.npz')}
    inputs.update({str(a.source/'dataset/train.json'):sha(a.source/'dataset/train.json')})
    inputs.update({str(trajectories/f'traj_{i:04d}.npz'):sha(trajectories/f'traj_{i:04d}.npz') for i in [0,1,*selected,*fresh]})
    inputs.update({str(p):sha(p) for p in models.values()})
    protocol=dict(training_installations=list(models)[:6],narrow_trajectories=[0],
        broad_trajectories=[0,*selected],validation_trajectory=1,fresh_test_trajectories=fresh,
        fresh_test_installations=['train_00','train_06'],states_per_installation_per_coverage=28,
        actions_per_state=26,selection='new fitting paths chosen by task sign/speed bins among IDs2..39, then reversal count, then ID; test paths excluded',
        full_path_split=True,labels='direct-only acceptance plus independent audit; ordinary feasible is never labeled terminal success',
        local_features='56 fixed-scale features: h/qmax, (h+Nr)/qmax, predicted lower/upper joint margins/(dt*qmax), margins clipped to[-10,10]',
        feature_constraints='current-state analytic lift only; no collision, QP, endpoint, midpoint or acceptance query used as network input',
        capacity_control='all models receive186 inputs; baseline pads56 zeros; same256x256 network and initialization',
        calibration='true=estimated; no deployment or latency qualification',
        teacher_failures='retain failed audited prefixes, never claim task completion; require>=7 states and broad motion coverage before fitting',
        inputs=inputs,source_sha256=sha(__file__))
    dump(root/'protocol.json',protocol)
    env=make_env(original['train_00/traj_0000']);new=[]
    try:
        for installation in [f'train_{i:02d}' for i in range(6)]:
            new.extend(initialize(env,[trajectories/f'traj_{i:04d}.npz' for i in selected],[(installation,Path(models[installation]),0.)]))
        for installation in ('train_00','train_06'):
            new.extend(initialize(env,[trajectories/f'traj_{i:04d}.npz' for i in fresh],[(installation,Path(models[installation]),0.)]))
    finally:env.close()
    for r in new:r.update(true_model=r['model'],estimated_model=r['model'])
    dump(root/'new_records.json',new)
    if not all(r['valid'] for r in new):
        dump(root/'status.json',dict(phase='stopped_initialization_gate',invalid=[r for r in new if not r['valid']]))
        raise RuntimeError('Allocated starts invalid; no resampling')
    rows=[]
    for record in new:
        rows.append(collect(record,root));dump(root/'teacher/episodes.json',rows)
    if any(row['steps']<7 for row in rows):raise RuntimeError('An allocated task supplied fewer than seven valid states')
    all_records={**original,**{r['case']:r for r in new}}
    sources={case:a.artifacts/'demonstrations'/f"{case.replace('/','__')}.npz" for case in original}
    sources.update({r['case']:Path(r['file']) for r in rows})
    fields=('observations','actions','rewards','terminals','feasible','local','group','failure_joint','failure_clearance','failure_speed')
    arrays={k:[] for k in fields};groups=[]

    def sweep(env,audit,state,expected,teacher,meta):
        gid=len(groups);start=len(arrays['actions'])
        obs=unpack_state(env,state)
        np.testing.assert_allclose(obs,expected,atol=1e-5,rtol=0)
        tick=time.perf_counter_ns()
        _,basis,he,hx,_=env._lift(env._x,env._q,env._path_state(env._u)[1],env._previous_n)
        lift_ms=(time.perf_counter_ns()-tick)/1e6;h=he+hx
        lo=env._q-env.lookup.joint_lower.ravel();hi=env.lookup.joint_upper.ravel()-env._q
        grid=[np.array([x,y],np.float32) for x in np.linspace(-1,1,5) for y in np.linspace(-1,1,5)]+[teacher]
        feature_us=[]
        for action in grid:
            obs=unpack_state(env,state);tick=time.perf_counter_ns()
            local=local_features(h,basis,lo,hi,action,env.dt,env.r_max,env.qdot_max)
            feature_us.append((time.perf_counter_ns()-tick)/1000)
            x,q,u=env._x.copy(),env._q.copy(),env._u
            _,reward,done,truncated,info=env.step(action);candidate=env._last_direct_candidate
            passed=env._last_direct_accepted
            assert not truncated and not info['projection_calls'] and not info['recovery_calls']
            if passed:
                verdict=audit_motion(env,audit,x,q)
                if verdict['violations']:raise AssertionError(verdict['violations'])
            else:
                assert done and env._u==u
                np.testing.assert_array_equal(env._q,q)
            values=(obs,action,reward/100.,float(done),float(passed),local,gid,
                float(candidate['joint_limit_margin_rad']<0),
                float(candidate['d_actual_m']<env.d_safe or candidate['endpoint_contact'] or candidate['swept_contact']),
                float(candidate['qdot_max_rad_s']>env.qdot_max+2e-5))
            for key,value in zip(fields,values):arrays[key].append(np.array(value,copy=True))
        flags=np.asarray(arrays['feasible'][start:])
        groups.append(dict(id=gid,start=start,end=len(arrays['actions']),mixed=bool(flags.any() and not flags.all()),
            lift_ms=lift_ms,feature_assembly_us=feature_us,timing_scope='concurrent diagnostic; not exclusive latency evidence',**meta))

    allocations=[]
    for case,record in sorted(all_records.items()):
        installation,trajectory=case.split('/');tid=int(trajectory.split('_')[1]);iid=int(installation.split('_')[1])
        with np.load(sources[case]) as d:obs=d['observations'].copy();actions=d['actions'].copy();states=d['states'].copy()
        if iid<6 and tid==0:
            narrow=anchors(obs,actions)
            assert len(narrow)==28
            for frame in narrow:allocations.append((case,frame,'narrow_fit',obs[frame],actions[frame],states[frame]))
        if iid<6 and tid in [0,*selected]:
            for frame in select_seven(obs,actions):allocations.append((case,frame,'broad_fit',obs[frame],actions[frame],states[frame]))
        if tid==1 or (iid>=6 and tid==0) or tid in fresh:
            split='fresh_test' if tid in fresh else ('installation_test' if iid>=6 else 'trajectory_validation')
            for frame in anchors(obs,actions):allocations.append((case,frame,split,obs[frame],actions[frame],states[frame]))
    for case in sorted({r[0] for r in allocations}):
        env=make_env(all_records[case]);audit=CollisionChecker(all_records[case]['true_model'])
        try:
            for _,frame,split,obs,action,state in [r for r in allocations if r[0]==case]:
                sweep(env,audit,state,obs,action,dict(case=case,frame=frame,split=split))
        finally:env.close()
        dump(root/'status.json',dict(phase='collecting_action_pairs',case=case,states=len(groups),actions=len(arrays['actions'])))
    packed={k:np.asarray(v,np.int64 if k=='group' else np.float32) for k,v in arrays.items()}
    np.savez_compressed(root/'data/transitions.npz',**packed);dump(root/'data/groups.json',groups)
    summary={}
    for split in sorted({g['split'] for g in groups}):
        chosen=[g for g in groups if g['split']==split];ids=[g['id'] for g in chosen]
        mask=np.isin(packed['group'],ids);state_obs=np.stack([packed['observations'][g['start']] for g in chosen])
        summary[split]=dict(states=len(chosen),actions=int(mask.sum()),mixed_states=sum(g['mixed'] for g in chosen),
            failures=int((mask&(packed['feasible']==0)).sum()),success_terminals=int((mask&(packed['feasible']==1)&(packed['terminals']==1)).sum()),
            negative_slide_states=int((state_obs[:,3]<-1e-4).sum()),positive_slide_states=int((state_obs[:,3]>1e-4).sum()),
            slow_slide_states=int((abs(state_obs[:,3])<.15).sum()),fast_slide_states=int((abs(state_obs[:,3])>.35).sum()),
            negative_turn_states=int((state_obs[:,2]<-1e-4).sum()),positive_turn_states=int((state_obs[:,2]>1e-4).sum()),
            boundary_states=int(((state_obs[:,31]<.05)|(state_obs[:,30]<.55)).sum()))
    broad=summary['broad_fit'];narrow=summary['narrow_fit']
    passed=broad['states']==narrow['states']==168 and broad['actions']==narrow['actions'] and all(broad[k]>=6 for k in
        ('negative_slide_states','positive_slide_states','slow_slide_states','fast_slide_states','negative_turn_states','positive_turn_states','boundary_states'))
    passed &= all(row['mixed_states']>=3 and row['success_terminals']>0 for row in summary.values())
    assert all(sha(p)==h for p,h in inputs.items())
    gate=dict(passed=bool(passed),splits=summary,teacher_completed=sum(r['completed'] for r in rows),teacher_cases=len(rows),
        teacher_failures=[r for r in rows if not r['completed']],accepted_audit_violations=0,
        data_sha256=sha(root/'data/transitions.npz'),source_inputs_unchanged=True)
    dump(root/'data/gate.json',gate)
    dump(root/'status.json',dict(phase='data_ready' if passed else 'stopped_coverage_gate',**gate))
    if not passed:raise RuntimeError('Coverage or paired-label availability gate failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts','source','root'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    for name in ('artifacts','source','root'):setattr(a,name,getattr(a,name).resolve())
    try:prepare(a)
    except BaseException as error:
        if a.root.exists():dump(a.root/'error.json',dict(error=repr(error)))
        raise

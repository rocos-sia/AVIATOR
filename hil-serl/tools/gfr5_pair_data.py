"""Audited same-state action sweeps with group-disjoint supervision splits."""
import argparse
import json
from pathlib import Path
import numpy as np
from tools.gfr1_remount_evaluate import dump, sha, audit_motion
from tools.gfr1_train_direct_only import install_guards
from tools.audit_static_collision import CollisionChecker
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr4_common import require_memory_limit


def split_case(case):
    installation, trajectory = case.split('/')
    if installation in ('train_06','train_07'):
        return 'installation_test'
    return 'fit' if trajectory == 'traj_0000' else 'trajectory_validation'


def anchors(obs, actions):
    n = len(obs)
    selected = set(np.linspace(0,n-1,8,dtype=int).tolist())
    scores = [obs[:,31],obs[:,30],-np.linalg.norm(actions-obs[:,33:35],axis=1)]
    for score, count in zip(scores,(8,8,4)):
        added = []
        for index in np.argsort(score,kind='stable'):
            if index not in selected and all(abs(int(index)-j)>=12 for j in added):
                selected.add(int(index)); added.append(int(index))
                if len(added)==count:
                    break
    selected.add(n-1)
    return sorted(selected)


def action_grid(extra):
    grid = [np.array([a,b],np.float32) for a in np.linspace(-1,1,5) for b in np.linspace(-1,1,5)]
    for action in extra:
        action=np.asarray(action,np.float32)
        assert action.shape==(2,) and np.isfinite(action).all() and np.max(np.abs(action))<=1
        if not any(np.array_equal(action,old) for old in grid):
            grid.append(action)
    return grid


def prepare(a):
    require_memory_limit(); install_guards()
    out=a.output; out.mkdir(parents=True,exist_ok=False)
    records=json.loads((a.source/'dataset/train.json').read_text())['records']
    inputs={str(a.source/'dataset/train.json'):sha(a.source/'dataset/train.json')}
    inputs.update({str(p):sha(p) for p in (a.artifacts/'demonstrations').glob('*.npz')})
    inputs.update({str(a.retention/name):sha(a.retention/name) for name in ('manifest.json','states.npz')})
    for record in records:
        for key in ('model','true_model'):
            inputs[record[key]]=sha(record[key])
    dump(out/'protocol.json',dict(
        fit='train_00..05 / traj_0000 only',validation='train_00..05 / traj_0001',
        installation_test='train_06..07 / both trajectories',historical_test='nine frozen historical failure states',
        actions='5x5 grid plus original teacher/rejected action and previous action',
        sampling='8 uniform anchors +8 joint boundary +8 clearance boundary +4 action-change anchors; terminal state included',
        labels='exact direct-only step, no projection; every accepted motion independently audited',
        feasible_is_not_success=True,terminal_rewards='actual scaled rewards; no invented +1 for ordinary feasible actions',
        original_60k_critic_caveat='pretraining saw all eight installations; only NEW supervision is group-disjoint',
        fresh_classifier_caveat='new classifier will fit only fit split, including normalization',
        heldout_paper_dataset_used=False,inputs=inputs,source_sha256=sha(__file__)))
    fields=('observations','actions','rewards','next_observations','terminals','feasible','group')
    arrays={k:[] for k in fields}; groups=[]

    def sweep(env,audit,state,expected,extra,meta):
        gid=len(groups); start=len(arrays['actions'])
        for action in action_grid(extra):
            obs=unpack_state(env,state)
            np.testing.assert_allclose(obs,expected,atol=1e-5,rtol=0)
            x,q,u=env._x.copy(),env._q.copy(),env._u
            successor,reward,done,truncated,info=env.step(action)
            passed=env._last_direct_accepted
            assert not truncated and not info['projection_calls'] and not info['recovery_calls']
            if passed:
                verdict=audit_motion(env,audit,x,q)
                if verdict['violations']:
                    raise AssertionError(('accepted motion audit',meta,verdict['violations']))
            else:
                assert done and env._u==u
                np.testing.assert_array_equal(env._q,q);np.testing.assert_array_equal(env._x,x)
            assert (not done) or ((not passed) or info['termination']=='end')
            for key,value in zip(fields,(obs,action,reward/100.,successor,float(done),float(passed),gid)):
                arrays[key].append(np.array(value,copy=True))
        flags=np.asarray(arrays['feasible'][start:])
        groups.append(dict(id=gid,start=start,end=len(arrays['actions']),
            passed_actions=int(flags.sum()),rejected_actions=int(len(flags)-flags.sum()),
            mixed=bool(flags.any() and not flags.all()),**meta))

    for record in sorted(records,key=lambda r:r['case']):
        case=record['case']; file=a.artifacts/'demonstrations'/f"{case.replace('/','__')}.npz"
        with np.load(file) as data:
            obs=data['observations'].copy(); states=data['states'].copy(); actions=data['actions'].copy()
        env=make_env(record);audit=CollisionChecker(record['true_model'])
        try:
            for frame in anchors(obs,actions):
                sweep(env,audit,states[frame],obs[frame],[actions[frame],obs[frame,33:35]],
                      dict(case=case,frame=frame,split=split_case(case),origin='teacher'))
        finally:
            env.close()
        dump(out/'status.json',dict(phase='collecting_pairs',last_case=case,states=len(groups),transitions=len(arrays['actions'])))
        print(json.dumps(dict(case=case,states=len(groups),transitions=len(arrays['actions']))),flush=True)
    history=json.loads((a.retention/'manifest.json').read_text())
    with np.load(a.retention/'states.npz') as data:
        obs=data['observations'].copy();states=data['states'].copy();actions=data['rejected_actions'].copy()
    for i,entry in enumerate(history['cases']):
        record=entry['record'];env=make_env(record);audit=CollisionChecker(record['true_model'])
        try:
            sweep(env,audit,states[i],obs[i],[actions[i],obs[i,33:35]],
                  dict(case=entry['case'],frame=entry['failed_step'],split='historical_test',origin=entry['origin_arm']))
        finally:
            env.close()
    packed={k:np.asarray(v,np.int64 if k=='group' else np.float32) for k,v in arrays.items()}
    assert np.all(packed['terminals'][packed['feasible']==0]==1)
    assert np.all(packed['rewards'][packed['feasible']==0]<-9)
    assert np.all(packed['rewards'][(packed['feasible']==1)&(packed['terminals']==1)]>0)
    np.savez_compressed(out/'transitions.npz',**packed)
    dump(out/'groups.json',groups)
    summary={}
    for split in ('fit','trajectory_validation','installation_test','historical_test'):
        ids=[g['id'] for g in groups if g['split']==split];mask=np.isin(packed['group'],ids)
        summary[split]=dict(states=len(ids),mixed_states=sum(g['mixed'] for g in groups if g['split']==split),
            actions=int(mask.sum()),rejected=int((mask&(packed['feasible']==0)).sum()),
            success_terminals=int((mask&(packed['feasible']==1)&(packed['terminals']==1)).sum()))
    # Availability gate, not a safety or learning qualification threshold.
    passed=all(summary[s]['mixed_states']>=3 for s in summary) and all(summary[s]['success_terminals']>0 for s in summary if s!='historical_test')
    assert all(sha(p)==h for p,h in inputs.items())
    dump(out/'gate.json',dict(passed=passed,splits=summary,accepted_audit_violations=0,
         group_disjoint=True,inputs_unchanged=True,data_sha256=sha(out/'transitions.npz')))
    dump(out/'status.json',dict(phase='completed' if passed else 'insufficient_boundary_coverage',splits=summary,passed=passed))
    if not passed:
        raise RuntimeError('Insufficient paired boundary coverage; do not train')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts','source','retention','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    for name in ('artifacts','source','retention','output'):
        setattr(a,name,getattr(a,name).resolve())
    prepare(a)

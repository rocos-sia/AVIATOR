"""Small frozen-Actor feedback diagnostic, separate from N0/E and all training.

Only the next Actor observation is intervened on. Real geometry, null basis,
executor history, safety checks and all subsequent observations remain intact.
Mixed observations are diagnostic interventions, not realizable sensor history.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr7_n0_qualify import FrozenActor, make_env, freeze, thaw, rollout
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr1_remount_evaluate import ROOT, dump, sha
from tools.gfr1_same_state_diagnosis import safe
from tools.gfr1_clearance_variants import use_projector, pair_distances
from tools.audit_static_collision import CollisionChecker
from tools.gfr4_common import require_memory_limit

GROUPS = {'normal': [], 'previous_action': [33,34], 'beta': [35],
          'intervention': [36], 'rejection': [37], 'recovery': [38],
          'all_flags': [36,37,38], 'all_history': [33,34,35,36,37,38]}


def mixed_observation(real, donor, indices):
    mixed = np.asarray(real).copy()
    mixed[indices] = np.asarray(donor)[indices]
    mask = np.ones(len(real), dtype=bool); mask[indices] = False
    np.testing.assert_array_equal(mixed[mask], np.asarray(real)[mask])
    return mixed


def row_equal(row, original):
    # Local continuation step indexing and timings differ by definition.
    clean = lambda r: safe({k:v for k,v in r.items() if k not in ('step','elapsed_ms')})
    if clean(row) != clean(original):
        raise AssertionError('Existing-run deterministic replay mismatch')


def summary(result):
    rows=result['rows']
    return dict(completed=result['completed'],termination=result['termination'],
        calls=result['candidate_calls'],steps=result['steps'],fallback_cycles=result['fallback_steps'],
        projection_calls=result['projection_calls'],boundary_retries=sum(r['boundary_iterations'] for r in rows),
        alternative_recoveries=sum(r['recovery'] for r in rows),
        slowdown_deficit_s=result['slowdown_deficit_s'],
        first_raw=rows[0]['raw'],first_accepted=rows[0]['executed'],first_direct=rows[0]['direct'],
        final_q=rows[-1]['q'],min_clearance_m=min(r['clearance_m'] for r in rows if r['clearance_m'] is not None))


def audit_training(a, actor):
    names={35:'beta',36:'intervention',37:'rejection',38:'recovery'}
    rows={}
    with np.load(a.artifacts/'bc/online_replay.npz') as data:
        obs=data['observations'][:60000];nxt=data['next_observations'][:60000]
        terminal=data['terminals'][:60000].reshape(-1)
        for index,name in names.items():
            rows[name]=dict(index=index,values=np.unique(obs[:,index]).tolist(),
                next_values=np.unique(nxt[:,index]).tolist(),
                nonterminal_next_values=np.unique(nxt[terminal==0,index]).tolist())
    counts=np.zeros(4,dtype=int);n=0
    for file in (a.artifacts/'demonstrations').glob('*.npz'):
        with np.load(file) as d:
            counts+=np.count_nonzero(d['observations'][:,35:39]-np.array([1,0,0,0]),axis=0)
            n+=len(d['observations'])
    bc=torch.load(a.bc,map_location='cpu',weights_only=True)['state_dict']
    current=actor.actor.state_dict()
    for i,name in names.items():
        mean=float(current['mean'][i]);scale=float(current['scale'][i])
        rows[name].update(mean=mean,scale=scale,normalized_zero=(0-mean)/scale,normalized_one=(1-mean)/scale,
                         max_first_layer_weight_change_since_bc=float((current['net.0.weight'][:,i].cpu()-bc['net.0.weight'][:,i]).abs().max()))
    return dict(online_rows=60000,demonstration_rows=n,demo_deviation_from_beta1_flags0=counts.tolist(),features=rows)


def local_features(env, saved, previous, actor):
    thaw(env,saved)
    q,x=env._q.copy(),env._x.copy();xd=env._path_state(env._u)[1]
    obs=env.last_policy_observation.copy();raw=actor(obs)
    _,basis,he,hx,sigma=env._lift(x,q,xd,env._previous_n)
    null=basis@(raw*env.r_max);velocity=he+hx+null
    lower=env.lookup.joint_lower.ravel();upper=env.lookup.joint_upper.ravel()
    margins=np.r_[q-lower,upper-q];rates=np.r_[velocity,-velocity]
    pair_margin=pair_distances(env.checker,x,q)-env.d_safe
    derivatives={}
    for eps in (1e-4,1e-5):
        plus=pair_distances(env.checker,x+eps*xd,q+eps*velocity)
        minus=pair_distances(env.checker,x-eps*xd,q-eps*velocity)
        derivatives[str(eps)]=((plus-minus)/(2*eps)).tolist()
    before_q=np.asarray(previous['fields']['_q'])
    result=dict(q=q.tolist(),x=x.tolist(),observation=obs.tolist(),
        joint_lower_margins=(q-lower).tolist(),joint_upper_margins=(upper-q).tolist(),
        limiting_signed_joint_index=int(np.argmin(margins)),
        previous_executed_r=env._r_prev.tolist(), previous_beta=env._beta_prev,
        explicit_history=obs[33:39].tolist(),
        actual_first_step_joint_velocity=((q-before_q)/env.dt).tolist(),
        next_actor_raw=raw.tolist(),next_nominal_qdot=velocity.tolist(),next_null_velocity=null.tolist(),
        mandatory_velocity=(he+hx).tolist(),sigma_min=sigma,
        signed_joint_margin_rates=rates.tolist(),collision_pair_margins=pair_margin.tolist(),
        collision_pair_rate_finite_difference=derivatives,
        derivative_scope='instantaneous nominal next Actor motion, includes task and joint motion; not a feasible-step assertion')
    thaw(env,saved)
    return result


def run(a):
    require_memory_limit();a.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    original_protocol=json.loads((a.source/'protocol.json').read_text())
    # Verify implementation dependencies against N0's frozen manifest.
    dependency_names=['gfr7_n0_qualify.py','gfr1_remount_evaluate.py','gfr1_repaired_env.py',
        'gfr1_recovery_boundary_ablation.py','gfr1_clearance_variants.py','gfr2_shared_env.py',
        'gfr1_shared_remount_env.py','gfr1_direct_only_env.py','gfr_env.py','gfr3_sac.py']
    dependencies={p:h for p,h in original_protocol['sources'].items() if Path(p).name in dependency_names}
    assert all(sha(p)==h for p,h in dependencies.items()), 'Executor/Actor code differs from original N0'
    checkpoint=a.artifacts/'bc/sac_060000.pt'
    inputs={str(checkpoint):sha(checkpoint),str(a.source/'states.json'):sha(a.source/'states.json'),str(a.bc):sha(a.bc)}
    inputs[str(a.source/'results.json')]=sha(a.source/'results.json')
    inputs.update({str(p):sha(p) for p in (a.artifacts/'demonstrations').glob('*.npz')})
    inputs[str(a.artifacts/'bc/online_replay.npz')]=sha(a.artifacts/'bc/online_replay.npz')
    own_hash=sha(__file__)
    dump(a.output/'protocol.json',dict(scope='N0-F1 s004/s007; N0-F2 s009 candidates 2/5 only',
        groups=GROUPS,intervention='one next Actor observation only; real executor state never changed; all later inputs genuine',
        horizon='same original common 1.25 s nominal-progress target; first original action already executed',
        actor='frozen BC-initialized SAC 60k on CUDA; no weight updates',
        groups_with_identical_inputs_or_actions='reuse identical rollout and report alias, not independent evidence',
        sources=dependencies,inputs=inputs,source_sha256=own_hash,
        no_training=True,no_remaining_30_states=True,no_N0G=True,
        interpretation='diagnostic observation intervention, not physically consistent substitute sensor history; not deployment fix',
        timings='not a formal latency benchmark',stop='audit/replay/hash failure stops all work'))
    actor=FrozenActor(checkpoint)
    dump(a.output/'training_distribution.json',audit_training(a,actor))
    jobs={j['id']:j for j in json.loads((a.source/'states.json').read_text())}
    records=[];static=[];f2=[];baseline_checks=[]
    with use_projector('combined'):
        for sid,indices in [('s004',[0,1]),('s007',[0,1]),('s009',[2,5])]:
            job=jobs[sid];env=make_env(job['record']);audit=CollisionChecker(job['record']['true_model'])
            try:
                unpack_state(env,job['state']);initial=freeze(env)
                saved=[];old=[];first=[]
                for index in indices:
                    file=a.source/'rollouts'/f'{sid}_{index:02d}.json';inputs[str(file)]=sha(file)
                    document=json.loads(file.read_text());old.append(document['result'])
                    partial=rollout(env,initial,actor,document['proposal']['action'],document['result']['target_u'],audit,budget=1)
                    row_equal(partial['rows'][0],document['result']['rows'][0])
                    first.append(partial['rows'][0]);saved.append(freeze(env))
                for origin in (0,1):
                    real=saved[origin]['fields']['last_policy_observation']
                    donor=saved[1-origin]['fields']['last_policy_observation']
                    base_action=actor(real);cached={}
                    groups=GROUPS if sid!='s009' else {'normal':[], 'all_history':[33,34,35,36,37,38]}
                    for name,columns in groups.items():
                        observation=mixed_observation(real,donor,columns);action=actor(observation)
                        key=action.tobytes()
                        entry=dict(state=sid,original_candidate=indices[origin],group=name,
                            donor_candidate=indices[1-origin],columns=columns,
                            real_history=real[33:39].tolist(),visible_history=observation[33:39].tolist(),
                            proposed=action.tolist(),delta_proposal_inf=float(np.max(np.abs(action-base_action))),
                            changed_input_indices=np.flatnonzero(observation!=real).tolist())
                        static.append(entry)
                        # F2 cross-history is a static network diagnostic only.
                        if sid=='s009' and name!='normal':
                            continue
                        dump(a.output/'status.json',dict(phase='controlled_replay',state=sid,origin=indices[origin],group=name,
                            completed_replays=len(records),elapsed_s=time.monotonic()-started,actor_trained=False,device='cuda:0'))
                        if key in cached:
                            original,result=cached[key]
                            records.append(dict(**entry,alias_of=original,summary=summary(result)))
                            continue
                        result=rollout(env,saved[origin],actor,action,old[origin]['target_u'],audit,
                                       budget=old[origin]['budget']-1)
                        filename=f'{sid}_{indices[origin]}_{name}.json'
                        cached[key]=(filename,result)
                        if name=='normal':
                            assert len(result['rows'])==len(old[origin]['rows'])-1
                            for now,prior in zip(result['rows'],old[origin]['rows'][1:]):row_equal(now,prior)
                            baseline_checks.append(dict(state=sid,origin=indices[origin],passed=True,
                                full_calls=result['candidate_calls']+first[origin]['candidate_calls']))
                        report=summary(result);report['first_original_action_calls']=first[origin]['candidate_calls']
                        report['including_original_first_calls']=result['candidate_calls']+first[origin]['candidate_calls']
                        dump(a.output/filename,dict(diagnostic=entry,result=result))
                        records.append(dict(**entry,file=filename,summary=report))
                        dump(a.output/'results.json',dict(replays=records,static=static,baseline_checks=baseline_checks))
                        assert actor.unchanged()
                if sid=='s009':
                    for origin in (0,1):
                        f2.append(dict(state=sid,original_candidate=indices[origin],
                                       features=local_features(env,saved[origin],initial,actor)))
                    dump(a.output/'f2_local_features.json',f2)
            finally:
                env.close()
    dump(a.output/'results.json',dict(replays=records,static=static,baseline_checks=baseline_checks))
    # All original sources, checkpoint and recorded trajectories remain intact.
    assert actor.unchanged() and sha(__file__)==own_hash
    assert all(sha(p)==h for p,h in inputs.items()) and all(sha(p)==h for p,h in dependencies.items())
    dump(a.output/'input_hashes.json',inputs)
    compact=[dict(state=r['state'],origin=r['original_candidate'],group=r['group'],
        delta_proposal_inf=r['delta_proposal_inf'],alias=r.get('alias_of'),
        **{k:r['summary'][k] for k in ('completed','calls','fallback_cycles','boundary_retries','first_direct')}) for r in records]
    dump(a.output/'summary.json',dict(rows=compact,baseline_checks=baseline_checks,
         static_diagnostics=len(static),unique_replays=sum('file'in r for r in records),
         inputs_unchanged=True,actor_updates=0,elapsed_s=time.monotonic()-started,
         gpu=torch.cuda.get_device_name(),peak_cuda_bytes=torch.cuda.max_memory_allocated()))
    dump(a.output/'status.json',dict(phase='completed',execution_completed=True,actor_trained=False,
         unique_replays=sum('file'in r for r in records),baseline_checks=len(baseline_checks),inputs_unchanged=True,
         scope='N0-F1 and F2 diagnostic only; N0-G not run'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'outputs/gfr_foresight_n0_20261126')
    p.add_argument('--artifacts',type=Path,default=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126')
    p.add_argument('--bc',type=Path,default=ROOT/'outputs/gfr_shared_distill_gpu_20261126/shared_bc/actor_round_2.pt')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    for name in ('source','artifacts','bc','output'):setattr(a,name,getattr(a,name).resolve())
    if a.output.exists():p.error('Refuse to overwrite an existing experiment')
    try:run(a)
    except Exception as exc:
        if a.output.exists():
            dump(a.output/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            dump(a.output/'status.json',dict(phase='stopped_error',execution_completed=False,error=repr(exc),actor_trained=False))
        raise


if __name__=='__main__':main()

"""Bounded, no-training DirectOnly feasibility and continuation diagnostics.

All search is external diagnostic action selection, never Actor-only success.
Only beta=1, structured=False checks; rejected states are never committed.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import pack_state, unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT, dump, sha, audit_motion
from tools.audit_static_collision import CollisionChecker
from tools.gfr4_common import require_memory_limit


def assess(env, action):
    raw = np.asarray(action, np.float32)
    # Match DirectOnlyEnv's float64 conversion BEFORE applying r_max.
    c, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                          raw.astype(float)*env.r_max, 1., env._previous_n, structured=False)
    reasons = []
    if c['joint_limit_margin_rad'] < -1e-9: reasons.append('joint_limit')
    if c['qdot_max_rad_s'] > env.qdot_max+2e-5: reasons.append('joint_speed')
    if c['d_actual_m'] < env.d_safe-1e-9: reasons.append('clearance')
    if c['endpoint_contact'] or c['swept_contact']: reasons.append('contact')
    assert c['feasible'] == (not reasons)
    score = min((c['joint_limit_margin_rad']+1e-9)/.005,
                (c['d_actual_m']-env.d_safe+1e-9)/.001,
                (env.qdot_max+2e-5-c['qdot_max_rad_s'])/.1,
                -100. if 'contact' in reasons else 100.)
    return dict(action=raw.tolist(), feasible=bool(c['feasible']), score=float(score),
                reasons=reasons, check=env._json_safe_candidate(c))


def search(env, actor_action, dense=True):
    """Grid then best-margin local refinement; a finite witness search only."""
    state = pack_state(env); obs = env.last_policy_observation.copy()
    found = {}
    def add(action):
        raw = np.clip(np.asarray(action, np.float32), -1, 1)
        key = raw.tobytes()
        if key not in found: found[key] = assess(env, raw)
    add(actor_action); add(env._r_prev/env.r_max)
    size = 17 if dense else 9
    for x in np.linspace(-1, 1, size):
        for y in np.linspace(-1, 1, size): add([x,y])
    # For fast continuation searches refine only if no witness was found.
    if dense or not any(v['feasible'] for v in found.values()):
        radius = 1/(size-1)
        for _ in range(3):
            centers = sorted(found.values(), key=lambda v:-v['score'])[:4]
            for center in centers:
                for dx in (-radius, 0., radius):
                    for dy in (-radius, 0., radius):
                        add(np.asarray(center['action'])+[dx,dy])
            radius /= 2
    np.testing.assert_array_equal(pack_state(env), state)
    np.testing.assert_allclose(unpack_state(env,state), obs, atol=1e-6, rtol=0)
    rows = list(found.values())
    return dict(evaluations=len(rows), feasible_count=sum(r['feasible'] for r in rows),
                failure_reasons=dict(Counter(reason for r in rows for reason in r['reasons'])),
                rows=rows, best=max(rows,key=lambda r:r['score']),
                conclusion='witness_found' if any(r['feasible'] for r in rows) else 'no_witness_in_finite_search')


def select_witness(scan, actor_action, mode):
    valid = [r for r in scan['rows'] if r['feasible']]
    if not valid: return None
    if mode == 'closest':
        return min(valid,key=lambda r:(np.sum((np.asarray(r['action'])-actor_action)**2),-r['score']))
    return max(valid,key=lambda r:r['score'])


def execute(env, audit, action):
    before = pack_state(env); x,q = env._x.copy(),env._q.copy()
    _,_,done,truncated,info = env.step(np.asarray(action,np.float32))
    assert info['projection_calls']==info['governor_calls']==info['recovery_calls']==0
    passed = env._last_direct_accepted
    if passed:
        assert info['beta_executed']==1
        verdict = audit_motion(env,audit,x,q)
        assert not verdict['violations'], verdict
        assert np.array_equal(env.last_policy_observation[35:39],[1,0,0,0])
        np.testing.assert_allclose(env._r_prev,np.asarray(action,np.float32).astype(float)*env.r_max,atol=0,rtol=0)
    else:
        assert done and not truncated and info['termination']=='direct_rejected'
        np.testing.assert_array_equal(env._q,q);np.testing.assert_array_equal(env._x,x)
        assert env._u==before[0]
    return dict(passed=bool(passed),action=np.asarray(action).tolist(),
                info=env._json_safe_candidate(env._last_direct_candidate),
                state_before=before.tolist(), state_after=pack_state(env).tolist())


def trajectory(env,audit,actor,state,target,first=None,mode='actor',script=None,heartbeat=None):
    unpack_state(env,state); start=env._u; rows=[]; searches=[]; termination='target'
    budget=int(np.ceil((target-start)*env.traj_dt/env.dt))+1
    for step in range(budget):
        if env._u>=target-1e-9: break
        if heartbeat: heartbeat(step)
        before=pack_state(env)
        raw=actor(env.last_policy_observation)
        action = np.asarray(first,np.float32) if step==0 and first is not None else raw
        if script is not None: action=np.asarray(script[step],np.float32)
        selected=False
        if mode != 'actor' and script is None and not (step==0 and first is not None):
            check=assess(env,action)
            if mode=='proactive' or not check['feasible']:
                scan=search(env,raw,dense=False)
                chosen=select_witness(scan,raw,'closest' if mode=='rescue_closest' else 'margin')
                searches.append(dict(step=step,actor_passed=check['feasible'],
                                     evaluations=scan['evaluations'],feasible_count=scan['feasible_count'],
                                     best=scan['best']))
                if chosen is None:
                    termination='no_witness_in_finite_search';break
                action=np.asarray(chosen['action'],np.float32);selected=True
        row=execute(env,audit,action)
        row.update(step=step,external_selection=selected,actor_action=raw.tolist())
        rows.append(row)
        if not row['passed']:
            termination='direct_rejected'
            unpack_state(env,before) # Return the actual state BEFORE rejection.
            break
    completed=env._u>=target-1e-9
    return dict(completed=completed,termination=termination,rows=rows,searches=searches,
                accepted_steps=sum(r['passed'] for r in rows),
                accepted_progress_s=(env._u-start)*env.traj_dt,
                final_state=pack_state(env).tolist(),target_u=target,
                external_selected_steps=sum(r['external_selection'] for r in rows),
                actor_only=first is None and mode=='actor' and script is None)


def run(args):
    require_memory_limit(); args.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic()
    source=ROOT/'outputs/gfr_foresight_n0_20261126'
    jobs={x['id']:x for x in json.loads((source/'states.json').read_text())}
    checkpoint=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/bc/sac_060000.pt'
    demo_path=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/demonstrations/train_01__traj_0000.npz'
    files=[Path(__file__),checkpoint,source/'states.json',demo_path,
           ROOT/'hil-serl/tools/gfr1_direct_only_env.py',ROOT/'hil-serl/tools/gfr3_demo_transitions.py',
           ROOT/'hil-serl/tools/gfr2_shared_env.py',ROOT/'hil-serl/tools/gfr1_shared_remount_env.py',
           ROOT/'hil-serl/examples/experiments/aviator_manifold/gfr_env.py']
    for key in ('model','true_model','trajectory_path'): files.append(Path(jobs['s008']['record'][key]))
    hashes={str(p):sha(p) for p in files}
    dump(args.output/'protocol.json',dict(beta=1.,dt=.005,normalized_action_bounds=[-1,1],
        scope='s008/s009 only; original common 1.25 nominal-second segments',
        training=False,neural_device='cuda:0',no_projection_governor_recovery=True,
        dense_search='17x17 grid + Actor/previous + 3 refinements around best 4 margin scores',
        continuation_search='9x9 grid, refine on no witness; max-margin or closest-to-Actor',
        score='min((joint margin+1e-9)/.005,(clearance-.005+1e-9)/.001,(qdotmax+2e-5-speed)/.1); contact penalized',
        single_replacement='up to 4 distinct feasible witnesses then frozen Actor',
        continuation_modes=['rescue_margin','rescue_closest','proactive'],
        backtracking='if needed, 1/3/5/10 accepted steps before a no-witness state; use independent alternative continuations',
        negative_result='finite search cannot prove empty feasible set or unreachable target',
        demo_replay='same-state successful recorded sequence is an existence witness, never RL performance',
        latency_claim=False,sources=hashes))
    install_guards();actor=FrozenActor(checkpoint)
    with np.load(demo_path) as data: demo={k:data[k].copy() for k in ('states','observations','actions')}
    results={};counts={'accepted_audits':0}
    def status(**kw):
        dump(args.output/'status.json',dict(execution_completed=False,actor_trained=False,
             elapsed_s=time.monotonic()-start,device='cuda:0',**kw))
    for sid in ('s008','s009'):
        job=jobs[sid];env=make_env(job['record']);audit=CollisionChecker(job['record']['true_model'])
        state=np.asarray(job['state']);target=state[0]+1.25/env.traj_dt
        result={'origin_frame':job['frame'],'target_u':target,'rejection_audits':[],'controllers':{}}
        results[sid]=result
        try:
            np.testing.assert_array_equal(state,demo['states'][job['frame']])
            status(phase='demo_existence_replay',state=sid)
            witness=trajectory(env,audit,actor,state,target,script=demo['actions'][job['frame']:])
            assert witness['completed']
            for i,row in enumerate(witness['rows']):
                np.testing.assert_allclose(row['state_before'],demo['states'][job['frame']+i],atol=1e-10,rtol=0)
            dump(args.output/f'{sid}_demo.json',witness)
            result['demo_witness']={k:v for k,v in witness.items() if k not in ('rows','searches','final_state')}
            origins=[1,2] if sid=='s008' else [1,6,2,5]
            baseline=None
            for ci in origins:
                status(phase='rejection_state_replay',state=sid,candidate=ci)
                document=json.loads((source/'rollouts'/f'{sid}_{ci:02d}.json').read_text())
                first=document['proposal']['action']
                path=trajectory(env,audit,actor,state,target,first=first)
                assert not path['completed']
                if ci==1: baseline=path
                dump(args.output/f'{sid}_origin_{ci}.json',path)
                rejection=path['final_state'];unpack_state(env,rejection)
                raw=actor(env.last_policy_observation)
                assert not assess(env,raw)['feasible']
                status(phase='dense_feasibility_search',state=sid,candidate=ci)
                scan=search(env,raw)
                dump(args.output/f'{sid}_scan_{ci}.json',dict(state=rejection,**scan))
                entry=dict(candidate=ci,accepted_before_rejection=path['accepted_steps'],
                    evaluations=scan['evaluations'],feasible_count=scan['feasible_count'],
                    failure_reasons=scan['failure_reasons'],best=scan['best'],single_replacements=[])
                result['rejection_audits'].append(entry)
                valid=sorted([r for r in scan['rows'] if r['feasible']],key=lambda r:-r['score'])
                chosen=[]
                for proposal in ([select_witness(scan,raw,'closest')] if valid else [])+valid:
                    if proposal is None: continue
                    if chosen and min(np.linalg.norm(np.asarray(proposal['action'])-r['action']) for r in chosen)<.25: continue
                    chosen.append(proposal)
                    if len(chosen)==4: break
                for j,proposal in enumerate(chosen):
                    status(phase='single_replacement',state=sid,candidate=ci,witness=j)
                    continued=trajectory(env,audit,actor,rejection,target,first=proposal['action'])
                    dump(args.output/f'{sid}_replace_{ci}_{j}.json',continued)
                    entry['single_replacements'].append(dict(action=proposal['action'],
                        completed=continued['completed'],accepted_steps=continued['accepted_steps'],
                        accepted_progress_s=continued['accepted_progress_s']))
                dump(args.output/'results.partial.json',results)
            for mode in ('rescue_margin','rescue_closest','proactive'):
                path=trajectory(env,audit,actor,state,target,mode=mode,
                    heartbeat=lambda step:status(phase='continuous_external_selection',state=sid,mode=mode,step=step))
                dump(args.output/f'{sid}_{mode}.json',path)
                result['controllers'][mode]={k:v for k,v in path.items() if k not in ('rows','searches','final_state')}
                result['controllers'][mode]['search_evaluations']=sum(s['evaluations'] for s in path['searches'])
                # Record any new dead end with a stronger scan, not an emptiness assertion.
                if not path['completed']:
                    unpack_state(env,path['final_state'])
                    scan=search(env,actor(env.last_policy_observation))
                    dump(args.output/f'{sid}_{mode}_final_scan.json',dict(state=path['final_state'],**scan))
                    result['controllers'][mode]['final_dense_witnesses']=scan['feasible_count']
                dump(args.output/'results.partial.json',results)
            # Backtrack on the original Actor prefix only when its first rejection
            # has no found witness. The original prefix is physically unchanged.
            if result['rejection_audits'][0]['feasible_count']==0:
                result['backtracks']=[]
                for offset in (1,3,5,10):
                    index=baseline['accepted_steps']-offset
                    if index<0: continue
                    earlier=baseline['rows'][index]['state_before']
                    path=trajectory(env,audit,actor,earlier,target,mode='proactive',
                         heartbeat=lambda step:status(phase='backtrack_continuation',state=sid,offset=offset,step=step))
                    dump(args.output/f'{sid}_backtrack_{offset}.json',path)
                    result['backtracks'].append(dict(offset_steps=offset,completed=path['completed'],
                        accepted_steps=path['accepted_steps'],accepted_progress_s=path['accepted_progress_s'],
                        termination=path['termination']))
                if not result['backtracks']:
                    result['backtracking_note']='Actor rejects at original anchor; same-anchor recorded demo already proves a successful sequence exists'
        finally: env.close()
    assert actor.unchanged() and all(sha(p)==h for p,h in hashes.items())
    dump(args.output/'results.json',dict(states=results,actor_updates=0,inputs_unchanged=True,
        gpu=torch.cuda.get_device_name(),elapsed_s=time.monotonic()-start,
        caution='External sequence/search success is a feasibility witness, not autonomous RL success or deployment timing'))
    dump(args.output/'status.json',dict(phase='completed',execution_completed=True,actor_trained=False,
         elapsed_s=time.monotonic()-start,inputs_unchanged=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output=args.output.resolve()
    if args.output.exists():p.error('Refuse overwrite')
    try:run(args)
    except Exception as exc:
        if args.output.exists():
            dump(args.output/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            dump(args.output/'status.json',dict(phase='stopped_error',execution_completed=False,error=repr(exc)))
        raise

"""Requalify existing N0 candidates under reject-and-terminate dynamics.

No protected suffix is reused. Every independent prefix is confirmed through
the original DirectOnlyEnv with projection/recovery guarded against execution.
No new candidates, training, or remaining 30-state evaluation are introduced.
"""
import argparse
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT, dump, sha, audit_motion
from tools.audit_static_collision import CollisionChecker
from tools.gfr4_common import require_memory_limit


HORIZONS = {'one_step': .005, 'h005': .05, 'h010': .1,
            'h025': .25, 'h050': .5, 'h100': 1., 'h125': 1.25}


def extract(document):
    result=document['result'];rows=result['rows']
    failed=next((i for i,r in enumerate(rows) if not r['direct']),None)
    good=rows if failed is None else rows[:failed]
    for row in good:
        assert row['beta']==1 and row['candidate_calls']==1 and row['projection_calls']==0
        assert not row['boundary_iterations'] and not row['recovery'] and not row['violations']
        np.testing.assert_allclose(row['executed'],np.asarray(row['raw'])*1.5,atol=1e-12,rtol=0)
    u=good[-1]['u'] if good else result['start_u']
    completed=failed is None and result['completed']
    progress=(u-result['start_u'])*.01
    return dict(completed=completed,first_rejected_zero_based_step=failed,
         directly_accepted_steps=len(good),accepted_nominal_progress_s=progress,
         first_reject_attempt_elapsed_s=None if failed is None else (failed+1)*.005,
         rejected_state_u=None if failed is None else u,
         censored_at_common_target=completed,
         label='completed_segment' if completed else 'direct_rejected')


def select_at_horizon(candidates,horizon):
    # No protected-call score. Complete the inspected progress interval first;
    # otherwise maximize accepted progress before first rejection. Original
    # candidate order breaks unresolved ties; there is no future oracle tiebreak.
    return min(range(len(candidates)),key=lambda i:(
        0 if candidates[i]['accepted_nominal_progress_s']>=horizon-1e-9 else 1,
        0 if candidates[i]['accepted_nominal_progress_s']>=horizon-1e-9 else -candidates[i]['accepted_nominal_progress_s']))


def run(a):
    require_memory_limit();a.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    states={s['id']:s for s in json.loads((a.source/'states.json').read_text())}
    cases=json.loads((a.source/'results.json').read_text())
    files=[a.source/'states.json',a.source/'results.json',a.source/'protocol.json',a.checkpoint,
           *sorted((a.source/'rollouts').glob('*.json'))]
    hashes={str(p):sha(p) for p in files}
    old_protocol=json.loads((a.source/'protocol.json').read_text())
    deps={p:h for p,h in old_protocol['sources'].items() if Path(p).name in (
        'gfr7_n0_qualify.py','gfr2_teacher.py','gfr2_shared_env.py','gfr1_direct_only_env.py',
        'gfr1_shared_remount_env.py','gfr1_remount_evaluate.py','gfr1_remount_pilot.py',
        'gfr_env.py','gfr3_sac.py','gfr3_demo_transitions.py')}
    assert all(sha(p)==h for p,h in deps.items())
    hashes.update(deps)
    for case in cases:
        record=states[case['id']]['record']
        for key in ('model','true_model','trajectory_path'):hashes[record[key]]=sha(record[key])
    dump(a.output/'protocol.json',dict(
        objective='Independent segment completion and first rejection; no protected correction-cost labels',
        scope='Only original 10 development states, original 11 candidates/state, original 60k continuation Actor',
        semantics='exact original DirectOnlyEnv.step: raw action -> beta1 check -> accept or terminate without state commit',
        counterfactual='only the first proposal differs; continuation is frozen Actor with genuine direct-only observations',
        independent_actor='candidate index 1, not external search',
        horizon_selection='finish inspected interval first; otherwise greatest accepted progress before rejection; original ordering for ties',
        horizons_nominal_s=HORIZONS,
        completion='common 1.25 nominal seconds, NOT full steering-wheel task completion',
        failures='remain failures even if a candidate postpones first rejection; all-candidate failure is never a successful teacher label',
        statistics='deterministic simulated outcomes, not calibrated probabilities or generalization estimates',
        no_new_training=True,no_remaining_30_states=True,
        candidate_search_cost_included=False,formal_latency_claim=False,
        sources=hashes,source_sha256=sha(__file__)))
    summaries=[];documents={}
    for case in cases:
        sid=case['id'];docs=[json.loads((a.source/'rollouts'/f'{sid}_{i:02d}.json').read_text()) for i in range(11)]
        documents[sid]=docs
        candidates=[extract(d) for d in docs]
        selected={name:select_at_horizon(candidates,h) for name,h in HORIZONS.items()}
        selected['next_event']=select_at_horizon(candidates,case['horizon_endpoints_nominal_s']['next_event'])
        selected['actor']=1;selected['previous']=0
        selected['finite_set_oracle']=max(range(11),key=lambda i:(candidates[i]['completed'],candidates[i]['accepted_nominal_progress_s']))
        summaries.append(dict(state=sid,case=case['case'],candidates=candidates,selections=selected,
            outcomes={name:candidates[i] for name,i in selected.items()},
            all_candidates_fail=not any(c['completed'] for c in candidates)))
    dump(a.output/'prefix_audit.json',summaries)
    install_guards();actor=FrozenActor(a.checkpoint)
    confirmations=[]
    for summary in summaries:
        sid=summary['state'];job=states[sid];env=make_env(job['record']);audit=CollisionChecker(job['record']['true_model'])
        try:
            for i,(document,label) in enumerate(zip(documents[sid],summary['candidates'])):
                dump(a.output/'status.json',dict(phase='direct_only_confirmation',state=sid,candidate=i,
                    confirmed_candidates=len(confirmations),total_candidates=110,elapsed_s=time.monotonic()-started,
                    actor_trained=False,device='cuda:0'))
                initial=unpack_state(env,job['state'])
                np.testing.assert_allclose(initial,job['observation'],atol=1e-5,rtol=0)
                expected=document['result'];steps=label['directly_accepted_steps']+(not label['completed'])
                for step in range(steps):
                    raw=np.asarray(document['proposal']['action'],np.float32) if step==0 else actor(env.last_policy_observation)
                    np.testing.assert_array_equal(raw, np.asarray(expected['rows'][step]['raw'],np.float32))
                    x0,q0,u0=env._x.copy(),env._q.copy(),env._u
                    obs,_,done,truncated,info=env.step(raw)
                    assert not info['projection_calls'] and not info['recovery_calls'] and not info['governor_calls']
                    passed=env._last_direct_accepted
                    assert passed == (step<label['directly_accepted_steps'])
                    if passed:
                        row=expected['rows'][step]
                        np.testing.assert_allclose(env._q,row['q'],atol=1e-12,rtol=0)
                        np.testing.assert_allclose(env._x,row['x'],atol=1e-12,rtol=0)
                        np.testing.assert_allclose(env._previous_n,row['n'],atol=1e-12,rtol=0)
                        assert abs(env._u-row['u'])<1e-12
                        assert info['beta_executed']==1
                        assert not audit_motion(env,audit,x0,q0)['violations']
                        assert not done and not truncated
                        assert np.array_equal(obs[35:39],np.array([1,0,0,0],np.float32))
                    else:
                        assert done and not truncated and info['termination']=='direct_rejected'
                        np.testing.assert_array_equal(env._q,q0);np.testing.assert_array_equal(env._x,x0)
                        assert env._u==u0
                        assert step==steps-1
                if label['completed']:
                    assert env._u>=expected['target_u']-1e-9
                else:
                    assert abs(env._u-label['rejected_state_u'])<1e-12
                confirmations.append(dict(state=sid,candidate=i,passed=True,attempts=steps,
                    completed_segment=label['completed'],termination='segment_target' if label['completed'] else 'direct_rejected',
                    accepted_progress_s=(env._u-expected['start_u'])*env.traj_dt,
                    first_reject_attempt_elapsed_s=label['first_reject_attempt_elapsed_s']))
                dump(a.output/'confirmations.json',confirmations)
        finally:env.close()
    methods={}
    for name in summaries[0]['selections']:
        chosen=[s['outcomes'][name] for s in summaries]
        methods[name]=dict(completed_segments=sum(c['completed'] for c in chosen),states=len(chosen),
            failures=[dict(state=s['state'],**s['outcomes'][name]) for s in summaries if not s['outcomes'][name]['completed']])
    assert actor.unchanged() and all(sha(p)==h for p,h in hashes.items())
    report=dict(methods=methods,states=summaries,confirmed_candidates=len(confirmations),
        all_candidates_fail_states=[s['state'] for s in summaries if s['all_candidates_fail']],
        completion_gain_over_actor=methods['finite_set_oracle']['completed_segments']-methods['actor']['completed_segments'],
        inputs_unchanged=True,actor_updates=0,elapsed_s=time.monotonic()-started,gpu=torch.cuda.get_device_name(),
        interpretation='Independent direct-only prefix qualification; no full-trajectory claim and no successful labels in all-fail states')
    dump(a.output/'results.json',report)
    dump(a.output/'status.json',dict(phase='completed',execution_completed=True,confirmed_candidates=len(confirmations),
        independent_actor_completed_segments=methods['actor']['completed_segments'],
        finite_set_oracle_completed_segments=methods['finite_set_oracle']['completed_segments'],
        completion_gain_over_actor=report['completion_gain_over_actor'],states=len(summaries),actor_trained=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'outputs/gfr_foresight_n0_20261126')
    p.add_argument('--checkpoint',type=Path,default=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/bc/sac_060000.pt')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    for name in ('source','checkpoint','output'):setattr(a,name,getattr(a,name).resolve())
    if a.output.exists():p.error('Refuse to overwrite existing experiment')
    try:run(a)
    except Exception as exc:
        if a.output.exists():
            dump(a.output/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            dump(a.output/'status.json',dict(phase='stopped_error',execution_completed=False,error=repr(exc)))
        raise


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Frozen-policy scheduler comparison on the SAME combined clearance executor.

All monkeypatches are scoped and require serial execution. Inclusive function
measurements overlap; exclusive timings form a partition, never sum inclusives.
Teacher uses historical previous/zero-first, expand-on-failure search (25 raw
candidates, highest beta then clearance), with the same repaired candidate path.
Its validated result is consumed once by the executor, without duplicate checks.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager, ExitStack
import functools
import json
import math
import os
from pathlib import Path
import time
import numpy as np

METHODS = ('fixed', 'greedy', 'teacher', 'rl')

class Meter:
    def __init__(self):
        self.stack=[]
        self.reset()
    def reset(self):
        assert not self.stack
        self.counts=Counter(); self.inclusive=defaultdict(float); self.exclusive=defaultdict(float)
    @contextmanager
    def span(self, name):
        frame=[name,time.perf_counter(),0.]; self.stack.append(frame)
        try: yield
        finally:
            elapsed=time.perf_counter()-frame[1]; self.stack.pop()
            self.counts[name]+=1; self.inclusive[name]+=elapsed
            self.exclusive[name]+=elapsed-frame[2]
            if self.stack:self.stack[-1][2]+=elapsed
    def wrap(self, name, fn):
        @functools.wraps(fn)
        def wrapped(*a,**kw):
            with self.span(name):return fn(*a,**kw)
        return wrapped

@contextmanager
def patch(obj,name,value):
    old=getattr(obj,name); setattr(obj,name,value)
    try:yield
    finally:setattr(obj,name,old)

@contextmanager
def instrumentation(core,meter):
    from experiments.aviator_manifold import gfr_env
    from tools import gfr1_clearance_variants as cv
    from tools import validate_branch_connector as base
    targets=[(core,'_lift','lift'),(core,'_candidate','candidate'),(core,'_get_obs','observation'),
             (core.lookup,'query','lookup'),(type(core.checker),'check','distance_check'),
             (gfr_env,'swept_clearance','swept'),(base,'swept_clearance','swept'),
             (cv,'pair_distances','distance_pairs'),(cv,'distance_planes','distance_planes'),
             (gfr_env,'project_r','projection'),(base,'_clip_polygon','polygon_clip'),
             (base,'_project_onto_polygon','polygon_project')]
    with ExitStack() as stack:
        for obj,name,label in targets:stack.enter_context(patch(obj,name,meter.wrap(label,getattr(obj,name))))
        yield

def greedy_action(core):
    # Explicit online heuristic, NOT the old LUT phi-center implementation.
    _,n,_,_,_=core._lift(core._x,core._q,core._path_state(core._u)[1],core._previous_n)
    center=.5*(np.concatenate(core.lookup.joint_lower)+np.concatenate(core.lookup.joint_upper))
    return np.clip(n.T@(center-core._q)/core.r_max,-1.,1.).astype(np.float32)

def teacher_selection(core):
    from tools.gfr1_recovery_boundary_ablation import evaluate_candidate
    from tools.gfr1_teacher_replay_consistency import candidates
    primary=core._repair_previous_raw.copy()
    history=[]
    def attempt(raw, source):
        local=[]
        for beta in core.governor_schedule:
            info,trials=evaluate_candidate(core,raw,beta,boundary=True)
            entry=dict(source=source,raw=np.asarray(raw).copy(),beta=beta,trials=trials)
            history.append(entry);local.append(entry)
            if info['feasible']:return (np.asarray(raw,dtype=np.float32),info),local
        return None,local
    chosen,selected_history=attempt(primary,'teacher_primary')
    expanded=chosen is None
    if expanded:
        rng=np.random.default_rng(core._repair_seed+core._repair_cycle)
        feasible=[]
        for raw in candidates(rng,core._repair_previous_raw,25):
            pick,path=attempt(raw,'teacher_search')
            if pick is not None:feasible.append((pick,path))
        if feasible:chosen,selected_history=max(feasible,key=lambda p:(p[0][1]['beta'],p[0][1]['d_actual_m']))
    diagnostics=dict(teacher_expanded=int(expanded),teacher_search_candidates=len(history),
        teacher_search_candidate_evaluations=sum(len(h['trials']) for h in history))
    # The selected raw is the teacher's nominal decision. Other search paths
    # count as scheduler work, not intervention against that final decision.
    if chosen is not None:return chosen[0],(chosen,selected_history,False),diagnostics
    return primary,(None,history,True),diagnostics

@contextmanager
def cached_selection(core,raw,selection):
    from tools import gfr1_repaired_env as re
    q=core._q.copy();x=core._x.copy();u=core._u;uses=[]
    def consume(c,action,*args,**kwargs):
        assert c is core and c._u==u
        np.testing.assert_array_equal(c._q,q);np.testing.assert_array_equal(c._x,x)
        np.testing.assert_array_equal(action,raw)
        assert not uses, 'teacher candidate cache consumed twice'
        uses.append(True);return selection
    with patch(re,'select_action',consume):yield
    assert len(uses)==1

def stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),p95=float(np.percentile(a,95)),
                p99=float(np.percentile(a,99)),maximum=float(a.max()),std=float(a.std()))

def validate_accepted(core,info):
    if info.get('beta_executed',0)>0:
        assert info['endpoint_validation'] and info['swept_validation']
        assert info['d_actual_m'] >= core.d_safe-1e-9
        assert info['joint_limit_margin_rad']>=-1e-9
        assert info['qdot_max_rad_s']<=core.qdot_max+2e-5

def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--method',choices=METHODS,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--warmup',type=int,default=100)
    p.add_argument('--max-steps',type=int,default=0,help='smoke only; never counted as completion')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from tools.gfr1_same_state_diagnosis import digest,write_json,safe
    from tools.gfr1_gate_support import atomic_json
    checkpoint_hash=digest(a.checkpoint);manifest_hash=digest(a.manifest)
    records=json.loads(a.manifest.read_text())['records']
    os.environ.update(AVIATOR_GFR_MODEL_MANIFEST=str(a.manifest),AVIATOR_GFR_MODE='structured',AVIATOR_GFR_VISIBLE_ERROR='1')
    os.environ.pop('AVIATOR_REPAIRED_TRACE_DIR',None)
    import jax
    from flax.training import checkpoints
    from experiments.aviator_gfr import config as cfg
    from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
    from tools.gfr1_clearance_variants import use_projector
    cfg.GFRManifoldEnv=RepairedGFRManifoldEnv
    conf=cfg.TrainConfig();env=conf.get_environment();core=env.unwrapped
    # Construct/load for every arm of the comparison so memory/device setup is shared.
    agent=conf.make_sac_agent(11,env.observation_space.sample(),env.action_space.sample())
    agent=agent.replace(state=checkpoints.restore_checkpoint(str(a.checkpoint),agent.state))
    def policy(obs,step):
        action=agent.sample_actions({'state':np.asarray([obs['state']])},seed=jax.random.PRNGKey(step),argmax=True)
        return np.asarray(jax.device_get(action))[0].astype(np.float32)
    obs,_=env.reset(seed=20262000,options={'trajectory_index':0})
    cold=time.perf_counter();policy(obs,500000);cold=time.perf_counter()-cold
    meter=Meter();calls=[]
    def cycle(obs,step):
        meter.reset();calls.clear()
        with meter.span('total_cycle'):
            with meter.span('scheduler'):
                selection=None;search={}
                if a.method=='rl':
                    with meter.span('policy_inference'):raw=policy(obs,500000+step)
                elif a.method=='fixed':raw=np.zeros(2,dtype=np.float32)
                elif a.method=='greedy':raw=greedy_action(core)
                else:raw,selection,search=teacher_selection(core)
            with meter.span('environment_step'):
                if selection is None:result=env.step(raw)
                else:
                    with cached_selection(core,raw,selection):result=env.step(raw)
        validate_accepted(core,result[-1])
        assert abs(sum(meter.exclusive.values())-meter.inclusive['total_cycle'])<1e-8
        return result,dict(counts=dict(meter.counts),inclusive_s=dict(meter.inclusive),exclusive_s=dict(meter.exclusive),search=search)
    rows=[];timings=[];all_counts=Counter();warmup_count=0
    with use_projector('combined',calls.append),instrumentation(core,meter):
        valid_indexes=[i for i,r in enumerate(records) if r.get('initialization_valid',True)]
        warmup_index=valid_indexes[0] if valid_indexes else None
        if warmup_index is not None:obs,_=env.reset(seed=20262000+warmup_index,options={'trajectory_index':warmup_index})
        for k in range(a.warmup if warmup_index is not None else 0):
            (obs,_,done,truncated,_),_=cycle(obs,k)
            warmup_count+=1
            if done or truncated:obs,_=env.reset(seed=20262000+warmup_index,options={'trajectory_index':warmup_index})
        with (a.output/'episodes.jsonl').open('x',buffering=1) as elog,(a.output/'steps.jsonl').open('x',buffering=1) as slog:
            for i,record in enumerate(records):
                obs,_=env.reset(seed=20262000+i,options={'trajectory_index':i})
                q0=core._q.copy();x0=core._x.copy()
                initial_valid=record.get('initialization_valid',True)
                budget=math.ceil((len(core._traj['x'])-1)*core.traj_dt/(core.dt*core.beta_stop))+2
                counts=Counter();exclusive=Counter();inclusive=Counter();times=[]
                totals=Counter();minimum=float(core.checker.check(core._x,core._q)[0]);closure=0.;last={};steps=0;accepted_steps=0;endpoint_minimum=minimum;swept_minimum=minimum
                cpu0=time.process_time();wall0=time.perf_counter()
                if initial_valid:
                    for step in range(min(budget,a.max_steps) if a.max_steps else budget):
                        (obs,_,done,truncated,info),timing=cycle(obs,i*10000+step)
                        steps+=1;last=info
                        accepted_steps+=int(info.get('beta_executed',0)>0)
                        totals.update(timing['search'])
                        counts.update(timing['counts']);exclusive.update(timing['exclusive_s']);inclusive.update(timing['inclusive_s'])
                        times.append(timing['inclusive_s']['total_cycle'])
                        totals.update(alternative_recoveries=int(info.get('alternative_recovered',False)),
                            interventions=int(info['intervened']),projection_or_retry=int(info['rejected']),
                            boundary_corrections=int(info['boundary_correction_iterations']),
                            candidate_evaluations=int(info['actual_candidate_evaluations']),
                            euler_endpoint_rejections=sum(not t['endpoint_valid'] for c in calls for t in c['attempts']),
                            euler_swept_rejections=sum(not t['swept_valid'] for c in calls for t in c['attempts']))
                        endpoint_minimum=min(endpoint_minimum,float(core.checker.check(core._x,core._q)[0]))
                        if info.get('beta_executed',0)>0:
                            swept_minimum=min(swept_minimum,float(info['d_swept_m']))
                            minimum=min(minimum,float(info['d_actual_m']))
                        closure=max(closure,float(np.max(np.abs(core.fk.error(core._x,core._q)))))
                        slog.write(json.dumps(safe(dict(episode=i,case_id=record.get('case_id',i),step=step,method=a.method,
                            progress=info['progress'],beta=info['beta_executed'],clearance=info.get('d_actual_m'),
                            accepted=bool(info['beta_executed']>0),termination=info.get('termination'),
                            policy_raw=info['policy_action_raw'],selected_raw=info.get('selected_action_raw'),
                            timing=timing)),allow_nan=False)+'\n')
                        if step%100==0:atomic_json(a.output/'progress.json',dict(episode=i,step=step,progress=info['progress'],method=a.method,updated_unix=time.time()))
                        if done or truncated:break
                assert counts.get('total_cycle',0)==steps
                if accepted_steps:
                    assert counts['distance_check']>0 and counts['swept']>0 and counts['projection']>0
                termination=last.get('termination')
                if not initial_valid:termination='initialization_invalid'
                elif termination not in ('end','governor_exhausted'):termination='diagnostic_step_budget' if a.max_steps else 'task_time_budget'
                progress=float(last.get('progress',0.))
                row=dict(case_id=record.get('case_id',i),episode=i,method=a.method,trajectory_path=record['trajectory_path'],model=record['model'],
                    initial_q=q0.tolist(),initial_x=x0.tolist(),completed=termination=='end',termination=termination,progress=progress,
                    steps=steps,accepted_steps=accepted_steps,simulated_time_s=accepted_steps*core.dt,total_compute_s=sum(times),cycle_s=stats(times),
                    process_cpu_s=time.process_time()-cpu0,wall_including_audit_io_s=time.perf_counter()-wall0,
                    min_executed_clearance_m=minimum,min_endpoint_clearance_m=endpoint_minimum,min_swept_clearance_m=swept_minimum,max_closure_residual_inf=closure,accepted_validation_violations=0,
                    counts=dict(counts),inclusive_s=dict(inclusive),exclusive_s=dict(exclusive),**totals)
                rows.append(row);timings.extend(times);all_counts.update(counts)
                elog.write(json.dumps(safe(row),allow_nan=False)+'\n');print(json.dumps(safe(row),allow_nan=False),flush=True)
    env.close()
    assert digest(a.checkpoint)==checkpoint_hash and digest(a.manifest)==manifest_hash
    write_json(a.output/'summary.json',dict(method=a.method,episodes=rows,completed=sum(r['completed'] for r in rows),
        count=len(rows),cycle_s=stats(timings),counts=all_counts,cold_policy_first_call_s=cold,warmup_steps=warmup_count,
        checkpoint_sha256=checkpoint_hash,manifest_sha256=manifest_hash,manifest_path=str(a.manifest.resolve()),devices=str(jax.devices()),
        simulation_side_latency=True,instrumentation_unknown=0,training_started=False,
        timing_note='Full-cycle includes scheduler, execution and next observation. File IO and independent audit outside cycle excluded. Inclusive categories overlap; exclusive categories partition total. Cold call recorded for all methods, excluded from steady state.',
        teacher_note='Historical previous/zero-first search, expand 25 only on exhaustion, same repaired candidate evaluator. Cache consumed once.',
        greedy_note='r=clip(N.T @ (joint_midpoint-q), -r_max,r_max), gain 1/s; online joint-centering, not old LUT phi-center.'))

if __name__=='__main__':main()

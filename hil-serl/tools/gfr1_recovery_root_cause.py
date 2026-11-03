#!/usr/bin/env python3
"""Replay accepted controls, then instrument frozen recovery candidates (no training)."""
import argparse,json,sys,time
from pathlib import Path
from collections import Counter
import numpy as np
from experiments.aviator_manifold import gfr_env
from tools.gfr1_same_state_diagnosis import ProjectTrace,safe,write_json,digest
from tools.gfr1_recovery_boundary_ablation import evaluate_candidate
from tools.validate_branch_connector import swept_clearance

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--pilot',action='store_true');a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=False)
 root=Path('outputs/differential_lift_feasibility_20261103/gfr_final60k_holdout')
 protocol=json.loads((root/'evaluation/protocol.json').read_text())
 protected={str(root/'evaluation'):digest(root/'evaluation'),str(root/'selection'):digest(root/'selection')}
 for path in protocol['protected_input_hashes']:
  if 'tools/' in path or 'gfr_env.py' in path: protected[path]=digest(path)
 write_json(a.output/'protocol.json',dict(method='Exact accepted-control midpoint replay; original project_r Python local tracing; no policy, solver, or threshold changes',protected_hashes=protected,pilot=a.pilot))
 events=[];start=time.monotonic()
 with (a.output/'events.jsonl').open('x',buffering=1) as log:
  for batch in range(2):
   manifest=root/f'selection/batch_{batch}_manifest.json';records=json.loads(manifest.read_text())['records']
   tracepath=next((root/f'evaluation/batch_{batch}/executor_traces').glob('*.jsonl'))
   rows=[json.loads(s) for s in tracepath.open()]
   for local in range(10):
    glob=batch*10+local
    if a.pilot and glob!=6:continue
    seq=[r for r in rows if r['episode']==local+1];recs=[r for r in seq if r['alternative_recovered']]
    if not recs:continue
    last=recs[0]['step'] if a.pilot else recs[-1]['step']
    core=gfr_env.GFRManifoldEnv(manifold_dir='hil-serl/data/aviator/manifold_phi_stale_pitch-10deg',model_path=records[local]['model'],trajectory_dir='hil-serl/data/aviator/trajectory_source',model_manifest=str(manifest),split='test')
    core.reset(seed=20262000+local,options={'trajectory_index':local})
    for row in seq:
     step=row['step'];q=core._q.copy();x=core._x.copy();xd=core._path_state(core._u)[1]
     r=np.asarray(row['action_executed_r']);beta=row['beta_executed']
     _,n,he,hx,_=core._lift(x,q,xd,core._previous_n)
     qm=q+.5*core.dt*(he+beta*hx+n@r);xm=x+.5*core.dt*beta*xd
     _,nm,hem,hxm,_=core._lift(xm,qm,xd,n)
     qn=q+core.dt*(hem+beta*hxm+nm@r);xn=x+core.dt*beta*xd
     u=min(core._u+beta*core.dt/core.traj_dt,len(core._traj['x'])-1.)
     assert abs(u/(len(core._traj['x'])-1)-row['progress'])<1e-12
     if row['alternative_recovered']:
      dend,_=core.checker.check(xn,qn);ds,_=swept_clearance(core.checker,x,q,xn,qn,subdivisions=5)
      err=abs(min(dend,ds)-row['d_actual_m']);assert err<1e-10,(glob,step,err)
      original=gfr_env.project_r;observer=ProjectTrace(original);calls=[]
      def traced(*args,**kwargs):
       result,rec=observer.call(*args,**kwargs);calls.append(rec);return result
      gfr_env.project_r=traced
      candidate_rows=[]
      try:
       for b in core.governor_schedule:
        calls.clear();info,hist=evaluate_candidate(core,row['policy_action_raw'],b,boundary=True)
        assert not info['feasible'],(glob,step,b,'policy unexpectedly accepted')
        candidate_rows.append(dict(beta=b,projections=list(calls),midpoint_trials=hist))
       calls.clear();accepted,hist=evaluate_candidate(core,row['selected_action_raw'],beta,boundary=True)
       assert accepted['feasible'],(glob,step,'recovery not reproduced')
       qerr=float(np.max(np.abs(accepted['q_next']-qn)));rerr=float(np.max(np.abs(np.asarray(accepted['r_exec'])-r)))
       assert qerr<1e-10 and rerr<1e-9,(glob,step,qerr,rerr)
       recovered=dict(projections=list(calls),midpoint_trials=hist)
      finally:gfr_env.project_r=original
      event=dict(episode=glob,step=step,progress=row['progress'],model=records[local]['model'],policy_raw=row['policy_action_raw'],recovery_raw=row['selected_action_raw'],beta=beta,reconstruction_clearance_error=err,recovery_q_error=qerr,recovery_r_error=rerr,q=q,x=x,previous_n=core._previous_n,u=core._u,policy_candidates=candidate_rows,recovered=recovered)
      log.write(json.dumps(safe(event),allow_nan=False)+'\n');events.append(event)
      print(json.dumps(dict(episode=glob,step=step,events=len(events),elapsed_s=time.monotonic()-start)),flush=True)
     core._q=qn;core._x=xn;core._previous_n=nm;core._u=u
     if step>=last:break
 for path,value in protected.items():assert digest(path)==value,path
 stages=Counter();first=Counter();attempts=Counter()
 for e in events:
  for c in e['policy_candidates']:
   for call in c['projections']:
    stages[call['stage']]+=1
    if call['trials']:first.update(call['trials'][0]['violated'])
    for trial in call['trials']:attempts.update(trial['violated'])
 write_json(a.output/'summary.json',dict(events=len(events),episodes=len(set(e['episode'] for e in events)),projection_stages=stages,first_attempt_violation_counts=first,all_attempt_violation_counts=attempts,input_hashes_unchanged=True,elapsed_s=time.monotonic()-start,max_clearance_reconstruction_error=max(e['reconstruction_clearance_error'] for e in events)))
if __name__=='__main__':main()

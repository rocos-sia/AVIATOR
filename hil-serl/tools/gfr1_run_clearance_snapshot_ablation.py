#!/usr/bin/env python3
"""Frozen 2x2 same-state policy-schedule ablation, before whole-path regression."""
import json,time,csv
from pathlib import Path
from collections import Counter
import numpy as np
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.gfr1_clearance_variants import VARIANTS,use_projector
from tools.gfr1_recovery_boundary_ablation import evaluate_candidate,independent_check
from tools.gfr1_same_state_diagnosis import safe,write_json,digest

ROOT=Path('outputs/differential_lift_feasibility_20261103')
OUT=ROOT/'gfr_clearance_ablation/snapshots_v2'
SOURCE=ROOT/'gfr_recovery_root_cause/full/events.jsonl'

def main():
 OUT.mkdir(parents=True,exist_ok=False)
 frozen=json.loads((ROOT/'gfr_final60k_holdout/evaluation/protocol.json').read_text())
 protected=dict(frozen['protected_input_hashes'])
 for p in [SOURCE,Path(__file__),Path('hil-serl/tools/gfr1_clearance_variants.py')]:protected[str(p)]=digest(p)
 for p,h in protected.items():assert digest(p)==h,p
 write_json(OUT/'protocol.json',dict(variants=VARIANTS,checkpoint=frozen['checkpoint'],checkpoint_sha256=frozen['checkpoint_sha256'],protected_input_hashes=protected,
  snapshots=198,policy='Original logged frozen60k action at each identical pre-recovery state',
  multi='All fixed mesh-cloud/wall minima separately constrained; no clearance margin, no new near-distance threshold; finite difference remains 1e-5 rad',
  affine='At most 6 trials, interpolate r halfway toward mean of clipped convex polygon vertices; same beta per candidate',
  frozen='Original dt, beta schedule, bounds, original midpoint compensation, endpoint and 6-point swept checker; no alternative search in this policy-only snapshot test',
  selection='Among non-original groups with at least one accepted original policy schedule and zero accepted check violations, maximize schedule acceptance; ties maximize summed accepted beta, then minimize Euler checks, then prefer multi, affine, combined in that order. Whole-path regression only after selection; snapshot success is not whole-task completion.',
  next='Run selected group on all original20 trajectories from initial state; reuse frozen original20 result for outcome comparison, no claim of paired runtime speedup'))
 events=[json.loads(l) for l in SOURCE.open()];rows=[];cache={};start=time.monotonic()
 with (OUT/'raw.jsonl').open('x',buffering=1) as stream:
  for index,e in enumerate(events):
   episode=e['episode'];batch,local=divmod(episode,10)
   if episode not in cache:
    core=GFRManifoldEnv(manifold_dir='hil-serl/data/aviator/manifold_phi_stale_pitch-10deg',model_path=e['model'],trajectory_dir='hil-serl/data/aviator/trajectory_source',model_manifest=str(ROOT/f'gfr_final60k_holdout/selection/batch_{batch}_manifest.json'),split='test')
    core.reset(seed=20262000+local,options={'trajectory_index':local});cache[episode]=core
   core=cache[episode];core._q=np.array(e['q']);core._x=np.array(e['x']);core._previous_n=np.array(e['previous_n']);core._u=e['u']
   for name in VARIANTS:
    calls=[];history=[];accepted=None;beg=time.perf_counter()
    with use_projector(name,calls.append):
     for beta in core.governor_schedule:
      info,trials=evaluate_candidate(core,e['policy_raw'],beta,boundary=True)
      history.append(dict(beta=beta,trials=[dict(iteration=t['iteration'],violated=t['violated'],result=core._json_safe_candidate(t['result'])) for t in trials]))
      if info['feasible']:accepted=info;break
    wall=time.perf_counter()-beg
    audit=independent_check(core,core._x,core._q,accepted) if accepted else None
    if name=='original':assert accepted is None,(episode,e['step'])
    row=dict(episode=episode,step=e['step'],variant=name,policy_schedule_accepted=accepted is not None,beta=accepted['beta'] if accepted else 0.,candidate_calls=len(calls),euler_checks=sum(len(c['attempts']) for c in calls),linear_empty=sum(c['linear_empty'] for c in calls),affine_violation_calls=sum(c['affine_violation'] for c in calls),wall_s=wall,independent_check=audit,history=history)
    stream.write(json.dumps(safe(row),allow_nan=False)+'\n');rows.append(row)
   if (index+1)%10==0 or index+1==len(events):print(json.dumps(dict(events=index+1,elapsed_s=time.monotonic()-start)),flush=True)
 summaries={}
 for name in VARIANTS:
  rr=[r for r in rows if r['variant']==name]
  summaries[name]=dict(events=len(rr),accepted=sum(r['policy_schedule_accepted'] for r in rr),sum_accepted_beta=sum(r['beta'] for r in rr),euler_checks=sum(r['euler_checks'] for r in rr),candidate_calls=sum(r['candidate_calls'] for r in rr),linear_empty=sum(r['linear_empty'] for r in rr),affine_violation_calls=sum(r['affine_violation_calls'] for r in rr),mean_wall_s=float(np.mean([r['wall_s'] for r in rr])),accepted_check_violations=0)
 rank={'multi':0,'affine':1,'combined':2}
 eligible=[k for k in rank if summaries[k]['accepted']>0]
 selected=min(eligible,key=lambda k:(-summaries[k]['accepted'],-summaries[k]['sum_accepted_beta'],summaries[k]['euler_checks'],rank[k])) if eligible else None
 for p,h in protected.items():assert digest(p)==h,p
 write_json(OUT/'summary.json',dict(groups=summaries,selected_for_whole_path=selected,source_hashes_unchanged=True,instrumentation_unknown=0))
 keys=['episode','step','variant','policy_schedule_accepted','beta','candidate_calls','euler_checks','linear_empty','affine_violation_calls','wall_s']
 with (OUT/'comparison.csv').open('x',newline='') as f:
  w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r[k] for k in keys} for r in rows)
 print(json.dumps(summaries,indent=2),flush=True);print('selected',selected,flush=True)
if __name__=='__main__':main()

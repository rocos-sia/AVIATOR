#!/usr/bin/env python3
"""Read-only active clearance-source audit using original checker point clouds."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
from tools.audit_static_collision import CollisionChecker
from tools.gfr1_same_state_diagnosis import write_json
root=Path('outputs/differential_lift_feasibility_20261103/gfr_recovery_root_cause/full')
events=[json.loads(l) for l in (root/'events.jsonl').open()];cache={};rows=[]
def sources(checker,x,q):
 reference,contact=checker.check(x,q);best={}
 for owner,bid,points in checker.clouds:
  xyz=points @ checker.data.xmat[bid].reshape(3,3).T + checker.data.xpos[bid]
  for wi,(center,half,rot) in enumerate(checker.walls):
   local=(xyz-center)@rot;excess=np.abs(local)-half
   outside=np.linalg.norm(np.maximum(excess,0),axis=1)
   dist=np.where(outside>0,outside,checker.wall_sign[wi]*local[:,2]-half[2])
   i=int(np.argmin(dist));val=float(dist[i]);key=(int(owner),wi)
   if key not in best or val<best[key]['distance']:best[key]=dict(owner=int(owner),wall=wi,body=int(bid),point=i,distance=val)
 vals=sorted(best.values(),key=lambda r:r['distance']);assert abs(vals[0]['distance']-reference)<1e-12,(vals[0],reference)
 return vals
for e in events:
 model=e['model']
 if model not in cache:cache[model]=CollisionChecker(model)
 c=cache[model];before=sources(c,e['x'],e['q'])
 for cand in e['policy_candidates']:
  p=cand['projections'][0]
  if not p['trials']:continue
  t=p['trials'][0]
  # No geometry checks depend on x for these robot clouds; use current x and
  # verify against original recorded candidate distance rather than assume.
  after=sources(c,e['x'],t['q_next']);assert abs(after[0]['distance']-t['d_endpoint_m'])<1e-12
  oldkey=(before[0]['owner'],before[0]['wall']);same=next(z for z in after if (z['owner'],z['wall'])==oldkey)
  rows.append(dict(episode=e['episode'],step=e['step'],beta=cand['beta'],before=before,after=after,old_source_after=same,changed_arm=before[0]['owner']!=after[0]['owner'],changed_wall=before[0]['wall']!=after[0]['wall'],old_arm_wall_still_safe_new_source_unsafe=same['distance']>=.005-1e-9 and after[0]['distance']<.005-1e-9))
write_json(root/'clearance_sources.json',rows)
summary=dict(candidates=len(rows),changed_arm=sum(r['changed_arm'] for r in rows),changed_wall=sum(r['changed_wall'] for r in rows),old_arm_wall_still_safe_new_source_unsafe=sum(r['old_arm_wall_still_safe_new_source_unsafe'] for r in rows),events_old_arm_wall_still_safe_new_source_unsafe=len({(r['episode'],r['step']) for r in rows if r['old_arm_wall_still_safe_new_source_unsafe']}),events_changed_arm=len({(r['episode'],r['step']) for r in rows if r['changed_arm']}))
write_json(root/'clearance_source_summary.json',summary);print(json.dumps(summary,indent=2))

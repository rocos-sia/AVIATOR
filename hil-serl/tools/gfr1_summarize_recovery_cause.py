#!/usr/bin/env python3
"""Summarize traced original constraints, keeping events distinct from candidates."""
import argparse,csv,json
from collections import Counter
from pathlib import Path
import numpy as np
from tools.gfr1_same_state_diagnosis import write_json

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args()
 events=[json.loads(l) for l in (a.root/'events.jsonl').open()]
 event_types=Counter();stage=Counter();first=Counter();last=Counter();midpoint=Counter();per_event=[];project_rows=[]
 for e in events:
  reasons=set();event_half_bad=0;event_worse=0;event_first_errors=[]
  for c in e['policy_candidates']:
   for call in c['projections']:
    stage[call['stage']]+=1
    if not call['trials']:
     reasons.add('linear_slice_empty');continue
    t0=call['trials'][0];tf=call['trials'][-1]
    first.update(t0['violated']);last.update(tf['violated']);reasons.update(t0['violated'])
    planes=call['planes']
    def violation(t):return max(float(np.dot(z['a'],t['r'])-z['b']) for z in planes)
    half_bad=len(call['trials'])>1 and violation(call['trials'][1])>1e-9
    worse=len(call['trials'])>1 and call['trials'][1]['d_endpoint_m']<t0['d_endpoint_m']-1e-9
    event_half_bad+=int(half_bad);event_worse+=int(worse)
    linear=0.005-(float(np.dot(planes[-1]['a'],t0['r']))-planes[-1]['b'])
    event_first_errors.append(t0['d_endpoint_m']-linear)
    project_rows.append(dict(episode=e['episode'],step=e['step'],beta=c['beta'],stage=call['stage'],first_reason=';'.join(t0['violated']),d0=call['d0_m'],first_endpoint=t0['d_endpoint_m'],first_swept=t0['d_swept_m'],linear_endpoint=linear,endpoint_minus_linear=t0['d_endpoint_m']-linear,first_plane_violation=violation(t0),halving_leaves_linear_set=half_bad,halving_worsens_endpoint=worse,final_endpoint=tf['d_endpoint_m'],first_joint_margin=t0['joint_limit_margin_rad'],first_speed=t0['qdot_max_rad_s']))
   for trial in c['midpoint_trials']:
    # Only a real midpoint result counts here; early rejection is traced above.
    if 'q_next' in trial['result']:
     midpoint.update(trial['violated']);reasons.update(trial['violated'])
  typ=';'.join(sorted(reasons));event_types[typ]+=1
  per_event.append(dict(episode=e['episode'],step=e['step'],progress=e['progress'],constraint_union=typ,halving_leaves_linear_set_count=event_half_bad,halving_worsens_endpoint_count=event_worse,recovery_beta=e['beta']))
 for name,rows in [('events.csv',per_event),('projections.csv',project_rows)]:
  with (a.root/name).open('x',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 def stats(v):return dict(min=float(np.min(v)),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(np.max(v)))
 summary=dict(events=len(events),event_constraint_unions=event_types,projection_stages=stage,first_attempt_violations=first,last_attempt_violations=last,midpoint_violations=midpoint,events_with_halving_leaving_linear_set=sum(r['halving_leaves_linear_set_count']>0 for r in per_event),events_with_halving_worsening_clearance=sum(r['halving_worsens_endpoint_count']>0 for r in per_event),projection_count=len(project_rows),first_linear_clearance_error_m=stats([r['endpoint_minus_linear'] for r in project_rows]),first_clearance_deficit_m=stats([.005-r['first_endpoint'] for r in project_rows if 'endpoint_clearance' in r['first_reason']]),halving_leaves_linear_set_count=sum(r['halving_leaves_linear_set'] for r in project_rows),halving_worsens_endpoint_count=sum(r['halving_worsens_endpoint'] for r in project_rows),reconstruction_max_clearance_error=max(e['reconstruction_clearance_error'] for e in events),recovery_max_q_error=max(e['recovery_q_error'] for e in events),recovery_max_r_error=max(e['recovery_r_error'] for e in events))
 write_json(a.root/'cause_summary.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':main()

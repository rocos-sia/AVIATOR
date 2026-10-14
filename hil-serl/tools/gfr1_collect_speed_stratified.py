#!/usr/bin/env python3
"""Collect 100 nominal endpoint-governor demonstrations in speed strata.

The collector is intentionally separate from historical N16/N17 runners. It
reuses their frozen online governor and writes only a new raw JSON artifact.
"""
from __future__ import annotations
import argparse, json, sys, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n8_dt_heldout import run_endpoint_governor
from tools.validate_differential_lift import AviatorClosure

BINS={
 "v015_050":(0.020,0.0005), "v050_150":(0.050,0.00125),
 "v150_300":(0.150,0.00375), "v300_450":(0.300,0.00750),
 "v450_700":(0.450,0.01125)}

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--n16-raw',required=True); ap.add_argument('--manifold',required=True); ap.add_argument('--output',required=True); ap.add_argument('--target-per-bin',type=int,default=20); ap.add_argument('--max-attempts-per-bin',type=int,default=40); args=ap.parse_args()
 out=Path(args.output).resolve(); out.mkdir(parents=True,exist_ok=True)
 if any(out.iterdir()): raise RuntimeError(f'non-empty output: {out}')
 raw=json.loads(Path(args.n16_raw).read_text()); variants={v['model_sha256']:v['model'] for v in raw['variants']}
 # Deduplicate states and retain only source trials that succeeded at nominal speed.
 states={}
 for r in raw['trial_rows']:
  if r.get('method')=='online_endpoint_governor' and r.get('complete'):
   key=(r.get('family'),float(r.get('error_level',0)),int(r.get('trial')))
   states.setdefault(key,r)
 states=list(states.values())
 rng=np.random.default_rng(20261013); rng.shuffle(states)
 lk=ManifoldLookup(str(Path(args.manifold).resolve())); lower=lk.joint_lower.reshape(14); upper=lk.joint_upper.reshape(14)
 rows=[]; counts={k:0 for k in BINS}; attempts={k:0 for k in BINS}
 for b,(theta,slide) in BINS.items():
  for r in states:
   if counts[b]>=args.target_per_bin or attempts[b]>=args.max_attempts_per_bin: break
   attempts[b]+=1; model=variants[r['model_sha256']]
   fk,checker=AviatorClosure(model),CollisionChecker(model)
   result=run_endpoint_governor(fk,checker,np.asarray(r['initial_x']),np.asarray(r['initial_q']),lower,upper,0.005,100,theta,slide)
   row={"method":"online_endpoint_governor","speed":b,"theta_dot":theta,"slide_dot":slide,"family":r.get('family'),"error_level":r.get('error_level'),"trial":r.get('trial'),"model":model,"model_sha256":r['model_sha256'],"initial_x":r['initial_x'],"initial_q":r['initial_q'],"initial_phi":r['initial_phi'],"initial_clearance_m":r.get('initial_clearance_m'),"dt":0.005,"steps_target":100,"d_safe_m":0.005,"beta_stop":0.125,"source_speed":"nominal","source_n16_trial":r.get('trial'),**result}
   rows.append(row)
   if row.get('complete'): counts[b]+=1
   print(f'{b} attempt={attempts[b]} accepted={counts[b]}/{args.target_per_bin} complete={row.get("complete")} progress={row.get("task_progress_fraction",0):.3f} reason={row.get("stop_reason")}',flush=True)
 report={"kind":"gfr1_speed_stratified_demo_raw","source_n16_raw":str(Path(args.n16_raw).resolve()),"source_n16_sha256":sha(args.n16_raw),"manifold":str(Path(args.manifold).resolve()),"speed_bins":BINS,"target_per_bin":args.target_per_bin,"counts":counts,"attempts":attempts,"summary":{"episodes":len(rows),"complete":sum(bool(r.get('complete')) for r in rows),"completion_rate":float(np.mean([bool(r.get('complete')) for r in rows])) if rows else 0.0},"trial_rows":rows}
 (out/'speed_stratified_demo_raw.json').write_text(json.dumps(report,indent=2)+"\n"); (out/'speed_stratified_demo_counts.json').write_text(json.dumps({"counts":counts,"attempts":attempts,"target_per_bin":args.target_per_bin},indent=2)+"\n")
 print(json.dumps(report['summary']|{'counts':counts,'attempts':attempts},indent=2))
if __name__=='__main__': main()

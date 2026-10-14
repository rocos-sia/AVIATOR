#!/usr/bin/env python3
"""Build a model/trajectory manifest with model-consistent initial q states.

Each record binds one trajectory file to exactly one MuJoCo model and stores a
Newton-corrected initial joint state.  The manifest is consumed by
``GFRManifoldEnv(model_manifest=...)``.
"""
from __future__ import annotations
import argparse, glob, json
from pathlib import Path
import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure
from tools.validate_online_lift import analytic_constraint_matrices

def repair_q(fk, x, q, lower, upper, iters=12):
    history=[]
    for _ in range(iters):
        e,A,_=analytic_constraint_matrices(fk,x,q)
        err=float(np.max(np.abs(e))); history.append(err)
        if err < 1e-7: break
        dq=np.linalg.pinv(A,rcond=1e-10) @ (-e)
        q=np.clip(q+0.8*dq,lower+1e-5,upper-1e-5)
    return q,history

def main():
    p=argparse.ArgumentParser(); p.add_argument('--manifold',required=True); p.add_argument('--trajectory-dir',required=True); p.add_argument('--split',default='val'); p.add_argument('--models',nargs='+',required=True); p.add_argument('--output',required=True); p.add_argument('--max-trajectories',type=int,default=4)
    a=p.parse_args(); lk=ManifoldLookup(a.manifold); lower=np.concatenate([lk.joint_lower[0],lk.joint_lower[1]]); upper=np.concatenate([lk.joint_upper[0],lk.joint_upper[1]])
    trajs=sorted(glob.glob(str(Path(a.trajectory_dir)/'trajs'/a.split/'*.npz')))[:a.max_trajectories]
    if not trajs: raise RuntimeError('no trajectories found')
    records=[]; audit=[]
    for model in a.models:
        fk=AviatorClosure(model); checker=CollisionChecker(model)
        for tp in trajs:
            with np.load(tp) as z: x=np.asarray(z['x'][0],dtype=float)
            box=lk.safe_interval(x[None,:]); phi=0.5*(box['phi_safe_lo'][0]+box['phi_safe_hi'][0]); q0=lk.query(x[None,:],phi[None,:],check_safe=False); q=np.concatenate([q0['qL'][0],q0['qR'][0]]).astype(float)
            q,hist=repair_q(fk,x,q,lower,upper); e=fk.error(x,q); d,contact=checker.check(x,q)
            rec={'trajectory_path':str(Path(tp).resolve()),'model':str(Path(model).resolve()),'initial_q':q.tolist(),'initial_x':x.tolist(),'initial_closure_inf':float(np.max(np.abs(e))),'initial_clearance_m':float(d),'initial_contact':bool(contact),'repair_history':hist}
            records.append(rec); audit.append({k:rec[k] for k in ('trajectory_path','model','initial_closure_inf','initial_clearance_m','initial_contact')})
    payload={'kind':'gfr1_consistent_model_manifest','split':a.split,'models':[str(Path(x).resolve()) for x in a.models],'trajectory_count':len(trajs),'records':records,'audit':audit,'frozen':{'dt':0.005,'d_safe':0.005,'beta_stop':0.125}}
    out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(payload,indent=2)+'\n'); print(json.dumps({'records':len(records),'closure_p95':float(np.percentile([x['initial_closure_inf'] for x in records],95)),'clearance_min':float(min(x['initial_clearance_m'] for x in records)),'contacts':sum(x['initial_contact'] for x in records)},indent=2))
if __name__=='__main__': main()

#!/usr/bin/env python3
"""Audit converted successful demos under the nominal GFR checker."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_branch_connector import swept_clearance
from tools.validate_differential_lift import AviatorClosure

def main():
    p=argparse.ArgumentParser(); p.add_argument('--raw',nargs='+',required=True); p.add_argument('--model',required=True); p.add_argument('--output',required=True); a=p.parse_args()
    rows=[]
    for path in a.raw:
        d=json.loads(Path(path).read_text()); rows.extend([r for r in d['trial_rows'] if r.get('method')=='online_endpoint_governor' and r.get('complete')])
    rows.sort(key=lambda r:(r.get('family',''),float(r.get('error_level',0)),int(r.get('trial',0)),r.get('speed','')))
    rows=rows[:100]; fk=AviatorClosure(a.model); checker=CollisionChecker(a.model)
    trial_out=[]; endpoint_pass=swept_pass=0; closure=[]
    for i,row in enumerate(rows):
        ep=sweep=0; ce=[]
        for tr in row['step_trace']:
            x0=np.asarray(tr['x_before'],float); q0=np.asarray(tr['q_before'],float); x1=np.asarray(tr['x_after'],float); q1=np.asarray(tr['q_after'],float)
            d,contact=checker.check(x1,q1); sm,sc=swept_clearance(checker,x0,q0,x1,q1,subdivisions=5)
            ep += int(not contact and d>=0.005-1e-9); sweep += int(not sc and sm>=0.005-1e-9); ce.append(float(np.max(np.abs(fk.error(x1,q1))))); closure.extend(ce[-1:])
        endpoint_pass += ep; swept_pass += sweep
        trial_out.append({'episode':i,'steps':len(row['step_trace']),'endpoint_pass_steps':ep,'swept_pass_steps':sweep,'max_closure_nominal':max(ce) if ce else None,'source_model':row.get('model')})
    payload={'kind':'gfr1_success_demo_nominal_audit','trials':trial_out,'total_steps':sum(x['steps'] for x in trial_out),'endpoint_pass_rate':endpoint_pass/max(sum(x['steps'] for x in trial_out),1),'swept_pass_rate':swept_pass/max(sum(x['steps'] for x in trial_out),1),'max_closure_nominal':max(closure) if closure else None,'interpretation':'Source trials were complete under their source models; this audit checks compatibility with the nominal GFR checker.'}
    Path(a.output).write_text(json.dumps(payload,indent=2)+'\n'); print(json.dumps({k:payload[k] for k in ['total_steps','endpoint_pass_rate','swept_pass_rate','max_closure_nominal']},indent=2))
if __name__=='__main__': main()

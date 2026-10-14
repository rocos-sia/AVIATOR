#!/usr/bin/env python3
"""Actual shared-constraint Node 0; development evidence until all gates pass."""
import argparse,json
from pathlib import Path
import numpy as np
from tools.reduced_lift_problem import build_problem,solve_problem
from tools.validate_online_lift import analytic_constraint_matrices,constraint_matrices,orient_basis
from tools.validate_differential_lift import AviatorClosure,sample_state
from tools.audit_static_collision import CollisionChecker
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def integration_probe(fk,x,q):
    xd=np.array([.08,.002]);rn=np.array([.04,-.04])
    _,a,_=analytic_constraint_matrices(fk,x,q);_,_,vh=np.linalg.svd(a,full_matrices=True);gauge=vh[-2:].T
    def field(y):
        e,aq,ax=analytic_constraint_matrices(fk,y[:2],y[2:]);_,_,vh=np.linalg.svd(aq,full_matrices=True)
        n=orient_basis(vh[-2:].T,gauge)
        return np.r_[xd,np.linalg.pinv(aq,rcond=1e-10)@(-ax@xd-2*e)+n@rn]
    def integrate(dt,midpoint):
        y=np.r_[x,q].copy()
        for _ in range(round(.08/dt)):
            f=field(y)
            y+=dt*field(y+.5*dt*f) if midpoint else dt*f
        return y
    ref=integrate(.000125,True)
    out={}
    for name,mid in [('euler',False),('midpoint',True)]:
        errs=[float(np.linalg.norm(integrate(dt,mid)-ref)) for dt in [.004,.002,.001]]
        orders=[float(np.log2(errs[i]/errs[i+1])) for i in range(2)]
        out[name]=dict(errors=errs,orders=orders)
    out['pass']=bool(min(out['euler']['orders'])>.8 and min(out['midpoint']['orders'])>1.8)
    return out


def main():
    a=argparse.ArgumentParser();a.add_argument('--samples',type=int,default=32);a.add_argument('--output',required=True)
    a.add_argument('--model',required=True);a.add_argument('--manifold',required=True);a.add_argument('--seed',type=int,default=20261004)
    args=a.parse_args();rng=np.random.default_rng(args.seed)
    fk=AviatorClosure(args.model);ck=CollisionChecker(args.model);lk=ManifoldLookup(args.manifold)
    lo=np.r_[lk.joint_lower[0],lk.joint_lower[1]];hi=np.r_[lk.joint_upper[0],lk.joint_upper[1]]
    rows=[];cases=[]
    for i in range(args.samples):
        for attempt in range(1000):
            x,_,look=sample_state(lk,rng);q=np.r_[look['qL'][0],look['qR'][0]]
            d,hit=ck.check(x,q)
            if d>=.007 and not hit:break
        else: raise RuntimeError('initial state sampling exhausted')
        derivatives=[]
        for qtest in (q,q+rng.uniform(-.01,.01,14)):
            e,aq,ax=analytic_constraint_matrices(fk,x,qtest);_,af,xf=constraint_matrices(fk,x,qtest)
            derivatives.append(dict(Aq_inf=float(np.max(abs(aq-af))),Ax_inf=float(np.max(abs(ax-xf)))))
        p=build_problem(fk,ck,x,q,np.array([.08,.002]),np.array([.4,-.4]),None,lo,hi,.005)
        r=solve_problem(p,'reduced');f=solve_problem(p,'full')
        agree=r['feasible']==f['feasible']
        diff=float(np.max(abs(r['z']-f['z']))) if r['feasible'] and f['feasible'] else None
        obj=abs(r['objective']-f['objective']) if diff is not None else None
        row=dict(sample=i,jacobians=derivatives,sigma=p['sigma'],feasible_agree=agree,
                 velocity_beta_diff=diff,objective_diff=obj,
                 reduced_status=r['status'],full_status=f['status'])
        # Controlled directional joint-rate half-space using an actual Aq row:
        # this row annihilates N, so it isolates the governor's beta semantics.
        if i<3:
            k=int(np.argmax(abs(p['aq']@p['hx'])));normal=p['aq'][k].copy()
            if normal@p['hx']<0:normal=-normal
            for name,limit in [('full_speed',1.),('slowdown',.4),('stop',-.1)]:
                pc=dict(p);pc['C']=np.vstack([p['C'],np.r_[normal,0.]])
                pc['b']=np.r_[p['b'],float(normal@(p['he']+limit*p['hx']))]
                sol=[solve_problem(pc,form) for form in ['reduced','full']]
                valid=all((s['feasible'] and abs(s['beta']-limit)<1e-5) if limit>0 else
                          (not s['feasible'] and s['status']=='linear_infeasible') for s in sol)
                cases.append(dict(sample=i,name=name,beta=[s['beta'] if s['feasible'] else None for s in sol],
                                  status=[s['status'] for s in sol],pass_gate=valid))
        row['pass_gate']=bool(max(v[k] for v in derivatives for k in ('Aq_inf','Ax_inf'))<=2e-5
                              and agree and (diff is None or diff<=2e-5) and (obj is None or obj<=1e-7)
                              and r['status']!='qp_solver_failure' and f['status']!='qp_solver_failure')
        rows.append(row)
        if i==0:integration=integration_probe(fk,x,q)
    report=dict(kind='shared_problem_node0',seed=args.seed,samples=args.samples,rows=rows,
                governor_cases=cases,integration=integration,
                status='pass' if all(r['pass_gate'] for r in rows) and all(c['pass_gate'] for c in cases)
                                 and integration['pass'] else 'fail',
                scope='Actual MuJoCo Jacobians and identical local constraints; controlled governor rate constraints; smooth-field integration order. No continuous collision certificate.')
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['status','samples','governor_cases','integration']},indent=2))
if __name__=='__main__':main()

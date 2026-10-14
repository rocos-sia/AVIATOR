"""One frozen linear problem shared by full and reduced online connectors.

Safety is assessed by sampled collocation, not certified continuous collision
checking. The clearance derivative belongs to the point-cloud/wall checker.
"""
from __future__ import annotations
import time
import numpy as np
from scipy.optimize import LinearConstraint, minimize, linprog
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis
from tools.validate_branch_connector import clearance_gradient, swept_clearance


def build_problem(fk, checker, x, q, xdot, r_nom, previous, lower, upper, dt,
                  feedback=2., qdot_max=1.5, d_safe=.005, r_max=3., beta_min=.125):
    e, aq, ax = analytic_constraint_matrices(fk, x, q)
    _, singular, vh = np.linalg.svd(aq, full_matrices=True)
    n = orient_basis(vh[-2:].T, previous)
    pinv = np.linalg.pinv(aq, rcond=1e-10)
    he, hx = -pinv @ (feedback*e), -pinv @ (ax @ xdot)
    d, contact = checker.check(x, q)
    grad_q = clearance_gradient(checker, x, q)
    grad_x = np.zeros(2)
    for j, eps in enumerate((1e-5, 1e-6)):
        xp, xm = x.copy(), x.copy()
        xp[j] += eps; xm[j] -= eps
        grad_x[j] = (checker.check(xp, q)[0]-checker.check(xm, q)[0])/(2*eps)
    # z=[qdot,beta], E z = -Ke e, C z <= b.
    C = np.vstack([np.c_[np.eye(14), np.zeros(14)],
                   np.c_[-np.eye(14), np.zeros(14)],
                   np.c_[dt*np.eye(14), np.zeros(14)],
                   np.c_[-dt*np.eye(14), np.zeros(14)],
                   np.r_[-dt*grad_q, -dt*float(grad_x@xdot)][None,:]])
    b = np.r_[np.full(28,qdot_max), upper-q, q-lower, d-d_safe]
    # z=c+B y; pull back all bounds, including phase-rate bounds, identically.
    B = np.zeros((15,3)); B[:14,:2]=n; B[:14,2]=hx; B[14,2]=1.
    c = np.r_[he, 0.]
    D = np.c_[n.T, -(n.T@hx)]
    C = np.vstack([C,D,-D]); b=np.r_[b, np.full(2,r_max)+n.T@he,
                                                   np.full(2,r_max)-n.T@he]
    return dict(e=e,aq=aq,ax=ax,n=n,he=he,hx=hx,E=np.c_[aq,ax@xdot],
                rhs=-feedback*e,C=C,b=b,B=B,c=c,nom=np.r_[he+hx+n@r_nom,1.],
                beta_min=beta_min,sigma=float(singular[-1]),d=float(d),contact=bool(contact),
                qdot_max=qdot_max,feedback=feedback)


def solve_problem(p, form='reduced'):
    start=time.perf_counter()
    Q=np.diag(np.r_[np.ones(14),20.])
    C,b,E,rhs=p['C'],p['b'],p['E'],p['rhs']
    if form=='reduced':
        B,c=p['B'],p['c']; H=B.T@Q@B; g=B.T@Q@(c-p['nom'])
        A=C@B; bound=b-C@c
        bounds=[(-3.,3.),(-3.,3.),(p['beta_min'],1.)]
        constraints=[LinearConstraint(A,-np.inf,bound)]
        x0=np.r_[np.zeros(2),1.]
    elif form=='full':
        B=np.eye(15); c=np.zeros(15); H=Q; g=-Q@p['nom']; A=C;bound=b
        bounds=[(None,None)]*14+[(p['beta_min'],1.)]
        constraints=[LinearConstraint(E,rhs,rhs),LinearConstraint(A,-np.inf,bound)]
        x0=p['nom']
    else: raise ValueError(form)
    fun=lambda u: .5*float(u@H@u)+float(g@u)
    jac=lambda u: H@u+g
    res=minimize(fun,x0,jac=jac,method='SLSQP',bounds=bounds,constraints=constraints,
                 options={'ftol':1e-11,'maxiter':150})
    z=c+B@res.x
    eq=float(np.max(np.abs(E@z-rhs))); violation=float(np.max(C@z-b))
    feasible=bool(res.success and eq<=1e-7 and violation<=1e-8 and
                  p['beta_min']-1e-9<=z[14]<=1+1e-9)
    status='ok'
    if not feasible:
        lp=linprog(np.zeros(len(x0)),A_ub=A,b_ub=bound,
                   A_eq=E if form=='full' else None,b_eq=rhs if form=='full' else None,
                   bounds=bounds,method='highs')
        status='linear_infeasible' if lp.status==2 else 'qp_solver_failure'
    r=p['n'].T@(z[:14]-p['he']-z[14]*p['hx'])
    return dict(feasible=feasible,status=status,z=z,r=r,beta=float(z[14]),
                objective=.5*float((z-p['nom'])@Q@(z-p['nom'])),
                eq_residual=eq,ineq_violation=violation,
                elapsed_ms=1000*(time.perf_counter()-start),iterations=int(res.nit))


def safe_midpoint(fk,checker,x,q,xdot,r_nom,previous,lower,upper,dt,form,
                  feedback=2.,beta_min=.125,min_dt=.005/32.,subdivisions=5):
    """Re-solve at midpoint, retry with smaller dt, validate before accepting.

    Both formulations execute their own filtered midpoint command. Rejections
    are returned separately from accepted-state metrics.
    """
    rejected=0; solves=0; residuals=[]; qp_ms=[]; last_reason='unknown'
    while dt>=min_dt-1e-12:
        p=build_problem(fk,checker,x,q,xdot,r_nom,previous,lower,upper,dt,
                        feedback=feedback,beta_min=beta_min)
        if p['sigma']<1e-3: last_reason='rank_loss'; break
        if p['d']<.005-1e-9 or p['contact']: last_reason='unsafe_initial_state'; break
        a=solve_problem(p,form);solves+=1;qp_ms.append(a['elapsed_ms'])
        if not a['feasible']: last_reason=a['status'];break
        xm=x+.5*dt*a['beta']*xdot;qm=q+.5*dt*a['z'][:14]
        pm=build_problem(fk,checker,xm,qm,xdot,r_nom,p['n'],lower,upper,dt,
                         feedback=feedback,beta_min=beta_min)
        am=solve_problem(pm,form);solves+=1;qp_ms.append(am['elapsed_ms'])
        if not am['feasible'] or pm['sigma']<1e-3:
            rejected+=1;last_reason='midpoint_'+am['status'];dt*=.5;continue
        xn=x+dt*am['beta']*xdot;qn=q+dt*am['z'][:14]
        d,contact=checker.check(xn,qn)
        swept,hit=swept_clearance(checker,x,q,xn,qn,subdivisions=subdivisions)
        error=fk.error(xn,qn)
        valid=(d>=.005-1e-9 and swept>=.005-1e-9 and not contact and not hit
               and np.all(qn>=lower-1e-9) and np.all(qn<=upper+1e-9)
               and np.max(np.abs(am['z'][:14]))<=1.5+1e-7
               and np.max(np.abs(error))<=2e-4)
        if valid:
            return dict(accepted=True,x=xn,q=qn,n=pm['n'],beta=am['beta'],dt=dt,
                        sigma=min(p['sigma'],pm['sigma']),residual=max(a['eq_residual'],am['eq_residual']),
                        qdot=am['z'][:14],rejected=rejected,solves=solves,qp_ms=qp_ms,
                        clearance=float(d),swept=float(swept),closure=error,reason='accepted')
        rejected+=1;last_reason='nonlinear_validation';dt*=.5
    return dict(accepted=False,reason=last_reason,rejected=rejected,solves=solves,qp_ms=qp_ms)

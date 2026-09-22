#!/usr/bin/env python3
"""Offline whole-trajectory sequential LP; exact TCP position, bounded rotation.
Uses the study's C++/MuJoCo model through pose_slack_bridge.cpp, no hardware IO.
"""
import argparse, ctypes, json, time
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.optimize import linprog

ROOT=Path(__file__).resolve().parents[2]
PTR=ctypes.POINTER(ctypes.c_double)
def ptr(a): return a.ctypes.data_as(PTR)
class Model:
 def __init__(self,threads=8):
  self.lib=ctypes.CDLL(str(ROOT/'AviatorRobot/build/lib/libpose_slack_bridge.so'))
  self.lib.ps_init.argtypes=[ctypes.c_char_p,ctypes.c_int,PTR]
  self.lib.ps_stream.argtypes=[ctypes.c_int,ctypes.c_double,PTR]
  self.lib.ps_eval.argtypes=[ctypes.c_int,ctypes.c_int,PTR,PTR,PTR,PTR]
  self.lib.ps_validate.argtypes=[ctypes.c_int,PTR,PTR,PTR]
  self.bounds=np.empty((2,2,7))
  assert self.lib.ps_init(str(ROOT/'AviatorRobot/config/aviator.yaml').encode(),threads,ptr(self.bounds))==0
 def stream(self,n,dt):
  x=np.empty((n,2));self.lib.ps_stream(n,dt,ptr(x));return x
 def eval(self,q,x,side,jac=True):
  q=np.ascontiguousarray(q);x=np.ascontiguousarray(x);e=np.empty((len(q),7));J=np.empty((len(q),7,7)) if jac else None
  self.lib.ps_eval(len(q),side,ptr(q),ptr(x),ptr(e),ptr(J) if jac else None)
  return e,J
 def validate(self,q,x):
  q=np.ascontiguousarray(q);x=np.ascontiguousarray(x);out=np.empty((len(q),8));self.lib.ps_validate(len(q),ptr(q),ptr(x),ptr(out));return out

def blocks(a):
 n,r,c=a.shape
 rows=np.broadcast_to(np.arange(n*r).reshape(n,r,1),(n,r,c)).ravel()
 cols=np.broadcast_to(np.arange(n*c).reshape(n,1,c),(n,r,c)).ravel()
 return sparse.coo_matrix((a.ravel(),(rows,cols)),shape=(n*r,n*c)).tocsr()
def metrics(q,e,dt):
 return dict(vmax=float(np.abs(np.diff(q,axis=0)).max()/dt),pos=float(np.linalg.norm(e[:,:3],axis=1).max()),rot=float(np.linalg.norm(e[:,3:6],axis=1).max()),clear=float(e[:,6].min()))

def solve_lp_with_cuts(c,A,b,bounds,full=False):
 """Constraint generation: every final LP solution is checked against ALL rows.
 Subsampling initializes the working set only; violated rows are added until the
 complete 200 Hz linearized problem is satisfied to the LP feasibility tolerance.

 full=True skips the working-set refinement and solves the complete LP directly:
 the subsample relaxation is too loose for the multi-dimensional nullspace used by
 pose slack (zdim=4), so the cut loop degenerates into many expensive re-solves.
 """
 if full:
  result=linprog(c,A_ub=A,b_ub=b,bounds=bounds,method='highs',options={'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
  if not result.success:
   result=linprog(c,A_ub=A,b_ub=b,bounds=bounds,method='highs',options={'presolve':False,'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
  return result
 active=np.arange(0,len(b),31)
 for cut in range(40):
  result=linprog(c,A_ub=A[active],b_ub=b[active],bounds=bounds,method='highs',options={'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
  if not result.success:
   # Degenerate zero-slack joint bounds can upset presolve; retry the identical LP.
   result=linprog(c,A_ub=A[active],b_ub=b[active],bounds=bounds,method='highs',options={'presolve':False,'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
   if not result.success:break
  violation=A@result.x-b;bad=np.flatnonzero(violation>2e-7)
  if len(bad)==0:return result
  if len(bad)>5000:bad=bad[np.argpartition(violation[bad],-5000)[-5000:]]
  expanded=np.union1d(active,bad)
  if len(expanded)==len(active):break
  active=expanded
 # Exact full-row fallback if working-set refinement did not finish or a cut LP failed.
 result=linprog(c,A_ub=A,b_ub=b,bounds=bounds,method='highs',options={'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
 if not result.success:
  result=linprog(c,A_ub=A,b_ub=b,bounds=bounds,method='highs',options={'presolve':False,'dual_feasibility_tolerance':1e-7,'primal_feasibility_tolerance':1e-7})
 return result

def optimize(model,q,x,side,alpha,dt,out,iterations=30,trust=.03,control_period=0.):
 """Joint positions at ALL times participate in each LP; no horizon truncation.
 Linearized equality constraints eliminated by pointwise SVD; exact nonlinear
 quantities determine convergence. LP result is not a global nonlinear certificate.
 """
 n=len(q);lo,hi=model.bounds[side];history=[]
 D=sparse.kron(sparse.diags([-np.ones(n-1),np.ones(n-1)],[0,1],shape=(n-1,n)),sparse.eye(7),format='csr')
 # Per-edge local peak controls otherwise arbitrary nonbinding minimax segments.
 T=sparse.kron(sparse.eye(n-1),np.ones((7,1)),format='csr')
 best=None
 for it in range(iterations):
  t=time.monotonic();e,J=model.eval(q,x,side);m=metrics(q,e,dt)
  valid=m['pos']<1e-7 and m['rot']<=alpha+1e-4 and m['clear']>=.005-1e-8 and np.min(q-lo)>-1e-8 and np.min(hi-q)>-1e-8
  if valid and (best is None or m['vmax']<best[0]):best=(m['vmax'],q.copy())
  if alpha==0: E=J[:,:6];res=e[:,:6]
  else:E=J[:,:3];res=e[:,:3]
  U,S,Vh=np.linalg.svd(E,full_matrices=True);rank=E.shape[1];zdim=7-rank
  dq0=-np.einsum('nki,ni->nk',Vh[:,:rank,:].transpose(0,2,1),np.einsum('nji,nj->ni',U,res)/S)
  Z=Vh[:,rank:,:].transpose(0,2,1).copy()
  if control_period>dt:
   # Parallel-transport the nullspace basis before using continuous phase controls.
   for k in range(1,n):
    u,_,vt=np.linalg.svd(Z[k].T@Z[k-1]);Z[k]=Z[k]@(u@vt)
   nc=int(np.ceil((n-1)*dt/control_period))+1
   coord=np.arange(n)*(nc-1)/(n-1);ix=np.minimum(coord.astype(int),nc-2);w=coord-ix
   W=sparse.coo_matrix((np.c_[1-w,w].ravel(),(np.repeat(np.arange(n),2),np.c_[ix,ix+1].ravel())),shape=(n,nc)).tocsr()
   H=sparse.kron(W,sparse.eye(zdim),format='csr')
  else:nc=n;H=sparse.eye(n*zdim,format='csr')
  B=blocks(Z)@H;base=q+dq0
  nvar=nc*zdim+1
  def pad(A):return sparse.hstack([A,sparse.csr_matrix((A.shape[0],1))],format='csr')
  mats=[];rhs=[]
  # Bounds on actual dq and hard joint ranges, expressed in reduced coordinates.
  dl=np.maximum(lo-q,-trust)-dq0;du=np.minimum(hi-q,trust)-dq0
  mats.extend([pad(B),pad(-B)]);rhs.extend([du.ravel(),-dl.ravel()])
  cJ=np.einsum('ni,nij->nj',J[:,6,:],Z)[:,None,:]
  cd=e[:,6]+np.einsum('ni,ni->n',J[:,6,:],dq0)
  mats.append(pad(-blocks(cJ)@H));rhs.append(cd-.005002)
  if alpha>0:
   # Inscribed octahedron (6 axes at alpha/sqrt(3)) bounds ||r||<=alpha exactly -- the
   # naive box |r_i|<=alpha reaches sqrt(3)*alpha at its corners. The adaptive tangent
   # plane adds a tight bound along the current rotation direction.
   rv=e[:,3:6]+np.einsum('nij,nj->ni',J[:,3:6],dq0)
   RJ=np.einsum('nij,njk->nik',J[:,3:6],Z)
   ax=np.r_[np.eye(3),-np.eye(3)]
   tang=rv/np.maximum(np.linalg.norm(rv,axis=1)[:,None],1e-9)
   dirs=np.concatenate([np.broadcast_to(ax,(n,6,3)),tang[:,None,:]],axis=1)
   beta=np.r_[np.full(6,alpha/np.sqrt(3)),alpha]
   A=np.einsum('nri,nij->nrj',dirs,RJ)
   mats.append(pad(blocks(A)@H));rhs.append((beta[None,:]*(1-1e-5)-np.einsum('nri,ni->nr',dirs,rv)).ravel())
  db=np.diff(base,axis=0).ravel();DB=D@B
  # Exact minimax objective: every joint step is bounded by the same global peak.
  peak_col=sparse.csr_matrix(-dt*np.ones((7*(n-1),1)))
  mats.extend([sparse.hstack([DB,peak_col]),sparse.hstack([-DB,peak_col])]);rhs.extend([-db,db])
  A=sparse.vstack(mats,format='csr');b=np.concatenate(rhs)
  c=np.zeros(nvar);c[-1]=1
  result=solve_lp_with_cuts(c,A,b,[(None,None)]*(nc*zdim)+[(0,None)],full=(zdim>1))
  if not result.success:
   print('LP_FAILED',side,np.rad2deg(alpha),it,result.message,flush=True);history.append(dict(it=it,**m,lp_status=result.message));break
  dq=dq0+(B@result.x[:nc*zdim]).reshape(n,7)
  # Merit line search balances minimax speed with hard nonlinear constraint residuals.
  def merit(qq,ee):
   mm=metrics(qq,ee,dt)
   pos_mm=1000*np.linalg.norm(ee[:,:3],axis=1).max()
   clear_viol_m=max(0,5e-3-mm['clear'])
   rot_viol=max(0,mm['rot']-alpha)
   violation=1e2*pos_mm+1e10*clear_viol_m+1e4*rot_viol
   return mm['vmax']+violation
  old=merit(q,e);accepted=False
  for scale in [1.,.5,.25,.125,.0625,.03125,.015625,.0078125]:
   qn=q+scale*dq
   # Retract to the nonlinear position (or rigid pose) manifold before acceptance.
   for projection in range(3):
    en,jn=model.eval(qn,x,side)
    er=en[:,:rank];je=jn[:,:rank]
    free=np.ones_like(qn);fixed=np.zeros_like(qn)
    for active_set in range(7):
     residual=er+np.einsum('nij,nj->ni',je,fixed)
     correction=fixed-np.einsum('nij,nj->ni',np.linalg.pinv(je*free[:,None,:]),residual)
     proposed=qn+correction
     hit=((proposed<lo-1e-12)|(proposed>hi+1e-12)) & (free>0)
     if not np.any(hit):break
     fixed[hit]=(np.clip(proposed,lo,hi)-qn)[hit];free[hit]=0
    qn=np.clip(qn+correction,lo,hi)
    if np.max(np.abs(er))<1e-10:break
   en,_=model.eval(qn,x,side,False)
   if merit(qn,en)<=old+1e-7:accepted=True;break
  if not accepted:trust*=.5
  else:
   q=qn
   if scale<.25:trust=max(.001,trust*.5)
  mn=metrics(q,en if accepted else e,dt)
  history.append(dict(it=it,**mn,lp_v=float(result.x[-1]),step=float(np.max(np.abs(dq))),scale=scale if accepted else 0,trust=trust,seconds=time.monotonic()-t))
  print('ITER',side,np.rad2deg(alpha),history[-1],flush=True)
  np.savez_compressed(out,q=q,x=x,dt=dt,alpha=alpha,side=side)
  out.with_suffix('.json').write_text(json.dumps(history,indent=2))
  if accepted and scale==1 and np.max(np.abs(dq))<1e-6:break
  if len(history)>=4 and mn['pos']<1e-7 and mn['rot']<=alpha+1e-4 and mn['clear']>=.005-1e-8:
   recent=history[-3:]
   if all(abs(h['vmax']-h['lp_v'])<1e-3 for h in recent):break
  if trust<1e-6:break
 e,_=model.eval(q,x,side,False);m=metrics(q,e,dt)
 if best is not None and (m['pos']>1e-7 or m['rot']>alpha+1e-4 or m['clear']<.005-1e-8 or m['vmax']>best[0]+1e-6):q=best[1]
 np.savez_compressed(out,q=q,x=x,dt=dt,alpha=alpha,side=side)
 return q

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=Path,default=Path('/tmp/aviator-exists/exists_path.csv'));ap.add_argument('--hz',type=int,default=50);ap.add_argument('--degrees',type=float,nargs='+',default=[0,1,2,3]);ap.add_argument('--iterations',type=int,default=30);ap.add_argument('--side',type=int,nargs='+',default=[0,1]);ap.add_argument('--max-time',type=float,default=199.98);ap.add_argument('--check',action='store_true');ap.add_argument('--warm-dir',type=Path);ap.add_argument('--control-period',type=float,default=0.);args=ap.parse_args()
 args.out.mkdir(parents=True,exist_ok=True);model=Model();dt=1/args.hz;n=round(args.max_time/dt)+1;x=model.stream(n,dt);seed=np.genfromtxt(args.seed,delimiter=',',names=True);times=np.arange(n)*dt
 qs=np.array([np.interp(times,seed['t'],seed[f'{s}{j}']) for s in ['L','R'] for j in range(1,8)]).T
 if args.check:
  for side in [0,1]:
   q=qs[:20,side*7:side*7+7].copy();e,J=model.eval(q,x[:20],side);err=[]
   for j in range(7):
    qp=q.copy();qm=q.copy();qp[:,j]+=1e-6;qm[:,j]-=1e-6
    ep,_=model.eval(qp,x[:20],side,False);em,_=model.eval(qm,x[:20],side,False);err.append(np.max(np.abs((ep-em)/2e-6-J[:,:,j])))
   print('FD_JAC',side,err,flush=True);assert max(err)<1e-4
  v=model.validate(qs[:20],x[:20]);print('VALIDATE',v.min(axis=0),v.max(axis=0));print('STREAM_50_MATCH',np.max(np.abs(model.stream(len(seed),.02)-np.c_[seed['theta'],seed['s']])));return
 for side in args.side:
  q=qs[:,side*7:side*7+7].copy()
  for deg in args.degrees:
   out=args.out/f'pose_{args.hz}hz_{deg:g}deg_side{side}.npz'
   if args.warm_dir:
    warm=np.load(args.warm_dir/f'pose_50hz_{deg:g}deg_side{side}.npz')
    q=np.array([np.interp(times,np.arange(len(warm['q']))*float(warm['dt']),warm['q'][:,j]) for j in range(7)]).T
   if out.exists():q=np.load(out)['q'];print('RESUME',out,flush=True)
   q=optimize(model,q,x,side,np.deg2rad(deg),dt,out,args.iterations,control_period=args.control_period)
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Replay a safe teacher on complete trajectories through the exact GFR env."""
from __future__ import annotations
import argparse, json
import copy
import pickle
from pathlib import Path
import numpy as np
from examples.experiments.aviator_manifold.gfr_env import GFRManifoldEnv

def candidates(rng, previous=None, samples=16):
    base=[np.zeros(2),np.array([1.,0.]),np.array([-1.,0.]),np.array([0.,1.]),np.array([0.,-1.]),np.array([1.,1.]),np.array([1.,-1.]),np.array([-1.,1.]),np.array([-1.,-1.])]
    if previous is not None: base.insert(0,np.asarray(previous,dtype=float))
    if samples <= len(base):
        return base[:samples]
    base.extend(rng.uniform(-1.,1.,size=(samples-len(base),2)))
    return base

def choose(env, action_list):
    x,q,xd=env._x,env._q,env._path_state(env._u)[1]
    feasible=[]
    for action in action_list:
        r=np.asarray(action,dtype=float)*env.r_max
        for beta in env.governor_schedule:
            info,_=env._candidate(x,q,xd,r,beta,env._previous_n,structured=True)
            if info.get('feasible',False):
                feasible.append((float(beta),float(info.get('d_actual_m',-np.inf)),np.asarray(action,dtype=np.float32),info)); break
    if not feasible: return None
    return max(feasible,key=lambda z:(z[0],z[1]))

def main():
 p=argparse.ArgumentParser(); p.add_argument('--manifest',required=True); p.add_argument('--manifold',required=True); p.add_argument('--model',required=True); p.add_argument('--trajectory-dir',required=True); p.add_argument('--output',required=True); p.add_argument('--demo-output'); p.add_argument('--max-episodes',type=int,default=4); p.add_argument('--max-steps',type=int,default=2000); p.add_argument('--samples',type=int,default=16); p.add_argument('--seed',type=int,default=20261013)
 a=p.parse_args(); env=GFRManifoldEnv(manifold_dir=a.manifold,model_path=a.model,trajectory_dir=a.trajectory_dir,split='val',mode='structured',visible_error=True,dt=.005,d_safe=.005,beta_stop=.125,model_manifest=a.manifest)
 episodes=[]; traces=[]; transitions=[]
 for ep in range(min(a.max_episodes,len(env._model_records))):
  obs,_=env.reset(seed=a.seed+ep,options={'trajectory_index':ep}); prev=None; accepted=0; rejects=0; minclr=float('inf'); maxclosure=0.; reason=''; start=len(traces)
  for step in range(a.max_steps):
   rng=np.random.default_rng(a.seed+ep*100000+step)
   primary=[np.zeros(2,dtype=float)] if prev is None else [np.asarray(prev,dtype=float)]
   pick=choose(env,np.asarray(primary))
   if pick is None:
    # Expand only at a local failure; the common path stays cheap while the
    # fallback distinguishes a sparse candidate set from true local failure.
    pick=choose(env,np.asarray(candidates(rng,prev,max(25,a.samples))))
   if pick is None:
    reason='teacher_no_feasible_candidate'; break
   action=pick[2]; obs_before=copy.deepcopy(obs); next_obs,reward,done,truncated,info=env.step(action); accepted+=1; rejects+=int(info.get('rejected',False)); minclr=min(minclr,float(info.get('d_actual_m',np.inf))); maxclosure=max(maxclosure,float(info.get('closure_residual_inf',0.))); prev=action.copy(); traces.append({'episode':ep,'step':step,'action':action.tolist(),'beta':float(info.get('beta_g',0.)),'progress':float(info.get('progress',0.)),'d_actual_m':float(info.get('d_actual_m',np.nan)),'intervened':bool(info.get('intervened',False)),'rejected':bool(info.get('rejected',False)),'termination':info.get('termination')}); transitions.append({'observations':obs_before,'actions':action.copy(),'next_observations':copy.deepcopy(next_obs),'rewards':np.asarray(reward,dtype=np.float32),'masks':np.asarray(1.0-float(done or truncated),dtype=np.float32),'dones':np.asarray(bool(done or truncated),dtype=bool)})
   if done or truncated:
    reason=info.get('termination','unknown'); break
   obs=next_obs
  episodes.append({'episode':ep,'model':env._traj.get('_model'),'trajectory_index':ep,'accepted_steps':accepted,'rejection_steps':rejects,'termination':reason,'complete':reason=='end','progress':float(info.get('progress',env._u/max(len(env._traj['x'])-1,1))) if 'info' in locals() else 0.,'min_clearance_m':minclr,'max_closure_residual_inf':maxclosure,'trace_start':start,'trace_count':len(traces)-start})
 payload={'kind':'gfr1_full_trajectory_teacher_replay','manifest':str(Path(a.manifest).resolve()),'episodes':episodes,'traces':traces,'summary':{'episodes':len(episodes),'complete':sum(x['complete'] for x in episodes),'completion_rate':float(np.mean([x['complete'] for x in episodes])) if episodes else 0.,'instrumentation_unknown':0},'frozen':{'dt':.005,'d_safe':.005,'beta_stop':.125,'teacher':'structured feasible candidate search'}}
 out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(payload,indent=2)+'\n'); print(json.dumps(payload['summary']|{'episodes':episodes},indent=2))
 if a.demo_output:
  dp=Path(a.demo_output); dp.parent.mkdir(parents=True,exist_ok=True)
  with dp.open('wb') as f: pickle.dump(transitions,f)
if __name__=='__main__': main()

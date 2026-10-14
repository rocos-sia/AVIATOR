#!/usr/bin/env python3
"""Replay an already validated full-trajectory action sequence on each model."""
import argparse,copy,json,pickle
from pathlib import Path
import numpy as np
from examples.experiments.aviator_manifold.gfr_env import GFRManifoldEnv

def main():
 p=argparse.ArgumentParser(); p.add_argument('--manifest',required=True); p.add_argument('--action-source',required=True); p.add_argument('--manifold',required=True); p.add_argument('--model',required=True); p.add_argument('--trajectory-dir',required=True); p.add_argument('--output',required=True); p.add_argument('--demo-output',required=True); a=p.parse_args()
 src=json.loads(Path(a.action_source).read_text()); actions=[x['action'] for x in src['traces'] if x['episode']==0]
 env=GFRManifoldEnv(a.manifold,a.model,a.trajectory_dir,split='rl_train',mode='structured',visible_error=True,model_manifest=a.manifest)
 eps=[]; trans=[]
 for ep in range(len(env._model_records)):
  obs,_=env.reset(seed=20261013+ep,options={'trajectory_index':ep}); start=len(trans); minc=1e9; maxe=0.; rej=0; term=''; n=0
  for action in actions:
   before=copy.deepcopy(obs); obs,r,d,t,info=env.step(np.asarray(action,dtype=np.float32)); n+=1; minc=min(minc,float(info.get('d_actual_m',1e9))); maxe=max(maxe,float(info.get('closure_residual_inf',0))); rej+=int(info.get('rejected',False)); term=info.get('termination','none'); trans.append({'observations':before,'actions':np.asarray(action,dtype=np.float32),'next_observations':copy.deepcopy(obs),'rewards':np.asarray(r,dtype=np.float32),'masks':np.asarray(1.-float(d or t),dtype=np.float32),'dones':np.asarray(bool(d or t),dtype=bool)})
   if d or t: break
  eps.append({'episode':ep,'model':env.model_path,'steps':n,'complete':term=='end','termination':term,'progress':float(info.get('progress',0)),'min_clearance_m':minc,'max_closure_residual_inf':maxe,'rejection_steps':rej,'transition_start':start,'transition_count':len(trans)-start})
 payload={'kind':'gfr1_replayed_validated_action_sequence','source':str(Path(a.action_source).resolve()),'episodes':eps,'summary':{'episodes':len(eps),'complete':sum(x['complete'] for x in eps),'completion_rate':float(np.mean([x['complete'] for x in eps])),'instrumentation_unknown':0}}
 Path(a.output).write_text(json.dumps(payload,indent=2)+'\n'); dp=Path(a.demo_output); dp.parent.mkdir(parents=True,exist_ok=True); pickle.dump(trans,dp.open('wb')); print(json.dumps(payload['summary']|{'episodes':eps},indent=2))
if __name__=='__main__': main()

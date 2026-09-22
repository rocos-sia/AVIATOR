#!/usr/bin/env python3
"""Independent mj_forward validation, 1 kHz interpolant audit, and plots."""
import argparse,json,hashlib,platform
from pathlib import Path
import numpy as np
import scipy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from test_pose_slack_200hz import Model,ROOT

def assess(m,q,x,dt,deg):
 v=m.validate(q,x)
 # Stable SO(3) log from the bridge (KDL's near-zero log has a deadband).
 for s in [0,1]:v[:,4+s]=np.linalg.norm(m.eval(q[:,7*s:7*s+7],x,s,False)[0][:,3:6],axis=1)
 speed=np.abs(np.diff(q,axis=0))/dt
 peak_idx=np.unravel_index(np.argmax(speed),speed.shape)
 d=dict(hz=1/dt,n=len(q),duration_s=(len(q)-1)*dt,alpha_deg=deg,
  vmax_rad_s=float(speed.max()),left_vmax_rad_s=float(speed[:,:7].max()),right_vmax_rad_s=float(speed[:,7:].max()),
  over_speed_intervals=int((speed.max(axis=1)>1.5+1e-7).sum()),peak_time_s=float((peak_idx[0]+1)*dt),peak_joint=('L' if peak_idx[1]<7 else 'R')+str(peak_idx[1]%7+1),
  position_max_mm=float(v[:,0].max()*1000),orientation_max_deg=float(np.rad2deg(v[:,4:6].max())),clearance_min_mm=float(v[:,1:3].min()*1000),
  joint_margin_min_rad=float(v[:,6:8].min()),collision_cycles_existing_detector=int(v[:,3].sum()),
  qddot_max_rad_s2=float(np.abs(np.diff(q,n=2,axis=0)).max()/dt**2))
 d['position_ok_0p1um']=d['position_max_mm']<=.0001
 d['orientation_ok']=d['orientation_max_deg']<=deg+0.02
 d['clearance_ok']=d['clearance_min_mm']>=5-1e-5
 d['joint_limits_ok']=d['joint_margin_min_rad']>=-1e-8
 d['speed_ok']=d['vmax_rad_s']<=1.5+1e-7
 d['sampled_kinematic_pass']=all(d[k] for k in ['position_ok_0p1um','orientation_ok','clearance_ok','joint_limits_ok','speed_ok']) and d['collision_cycles_existing_detector']==0
 return {k:(value.item() if isinstance(value,np.generic) else value) for k,value in d.items()},v,speed

def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);ap.add_argument('--degrees',type=float,nargs='+',default=[0,1,2,3]);ap.add_argument('--substeps',type=int,default=5);args=ap.parse_args();m=Model();summaries=[];curves=[]
 for deg in args.degrees:
  L=np.load(args.directory/f'pose_200hz_{deg:g}deg_side0.npz');R=np.load(args.directory/f'pose_200hz_{deg:g}deg_side1.npz')
  q=np.c_[L['q'],R['q']];x=L['x'];dt=float(L['dt'])
  assert q.shape==(39997,14) and x.shape==(39997,2)
  assert np.isfinite(q).all() and np.isfinite(x).all()
  assert np.array_equal(x,R['x']) and dt==float(R['dt'])==.005
  assert int(L['side'])==0 and int(R['side'])==1
  assert abs(float(L['alpha'])-np.deg2rad(deg))<1e-12 and abs(float(R['alpha'])-np.deg2rad(deg))<1e-12
  assert np.max(np.abs(x-m.stream(39997,.005)))<1e-12
  d,v,speed=assess(m,q,x,dt,deg);np.savez_compressed(args.directory/f'validated_{deg:g}deg.npz',q=q,x=x,metrics=v,speed=speed,dt=dt)
  np.savetxt(args.directory/f'trajectory_{deg:g}deg.csv',np.c_[np.arange(len(q))*dt,x,q,v],delimiter=',',header='t,theta,s,'+','.join(f'{s}{j}' for s in ['L','R'] for j in range(1,8))+',position_error_m,clearance_L_m,clearance_R_m,collision,rotation_L_rad,rotation_R_rad,margin_L_rad,margin_R_rad',comments='',fmt='%.12g')
  if args.substeps>1:
   nd=(len(q)-1)*args.substeps+1;times=np.arange(nd)*(dt/args.substeps)
   qi=np.array([np.interp(times,np.arange(len(q))*dt,q[:,j]) for j in range(14)]).T
   xi=m.stream(nd,dt/args.substeps);di,vi,_=assess(m,qi,xi,dt/args.substeps,deg)
   d['linear_interpolation_audit']=di
  summaries.append(d);curves.append((deg,np.arange(len(q))*dt,v,speed));print(json.dumps(d),flush=True)
 report={'scope':'Offline kinematic experiment; local sequential LP, no global optimality or infeasibility certificate. Clearance uses the original decimated mesh vertex-to-wall metric. Collision detector ignores penetration smaller than 1 mm. Sampled pass does not imply interpolant or dynamic pass.', 'environment':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__},'results':summaries,'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'AviatorRobot/tools/clearance_trajectory.cpp',ROOT/'AviatorRobot/tools/pose_slack_bridge.cpp',ROOT/'AviatorRobot/scripts/test_pose_slack_200hz.py',ROOT/'AviatorRobot/scripts/report_pose_slack_200hz.py',ROOT/'AviatorRobot/config/aviator.yaml',ROOT/'reference/rocos-mujoco/model/aviator.xml']}}
 (args.directory/'summary.json').write_text(json.dumps(report,indent=2))
 fig,ax=plt.subplots(2,2,figsize=(12,7),layout='constrained')
 ax[0,0].plot([d['alpha_deg'] for d in summaries],[d['vmax_rad_s'] for d in summaries],'o-');ax[0,0].axhline(1.5,color='red',ls='--',label='1.5 rad/s limit');ax[0,0].set(xlabel='Orientation tolerance (deg)',ylabel='Peak joint speed (rad/s)');ax[0,0].legend()
 for deg,t,v,speed in curves:
  ax[0,1].plot(t[1:],speed.max(axis=1),lw=.6,label=f'{deg:g} deg')
  ax[1,0].plot(t,v[:,1:3].min(axis=1)*1000,lw=.6,label=f'{deg:g} deg')
  ax[1,1].plot(t,np.rad2deg(v[:,4:6].max(axis=1)),lw=.6,label=f'{deg:g} deg')
 ax[0,1].axhline(1.5,color='red',ls='--');ax[0,1].set(xlabel='Time (s)',ylabel='Joint speed envelope (rad/s)');ax[0,1].legend()
 ax[1,0].axhline(5,color='red',ls='--');ax[1,0].set(xlabel='Time (s)',ylabel='Wall clearance (mm)')
 ax[1,1].set(xlabel='Time (s)',ylabel='Orientation error (deg)')
 for a in ax.flat:a.grid(alpha=.2)
 fig.savefig(args.directory/'summary.png',dpi=180);fig.savefig(args.directory/'summary.svg');plt.close(fig)
if __name__=='__main__':main()

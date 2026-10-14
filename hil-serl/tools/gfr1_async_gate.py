#!/usr/bin/env python3
"""Train B4 continuously while evaluating checkpoints at 20k/40k.

The gate is deliberately independent of the learner replay buffer.  It starts
evaluation subprocesses after checkpoints appear and only applies the hard
stop after both 20k and 40k reports are available and both have zero
completion.  Historical checkpoints are never touched.
"""
from __future__ import annotations
import argparse, hashlib, json, os, signal, subprocess, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]; HIL=ROOT/'hil-serl'
PY=Path(os.environ.get('GFR_PYTHON','/home/rocos/miniconda3/envs/gfr_clean/bin/python'))

def sha_tree(p:Path):
 h=hashlib.sha256();
 for f in sorted(x for x in p.rglob('*') if x.is_file()):
  h.update(str(f.relative_to(p)).encode()); h.update(f.read_bytes())
 return h.hexdigest()

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--demo-raw',required=True); ap.add_argument('--demo-dir',required=True); ap.add_argument('--output',required=True); ap.add_argument('--manifold',required=True); ap.add_argument('--model',required=True); ap.add_argument('--train-manifest'); ap.add_argument('--eval-manifest'); ap.add_argument('--steps',type=int,default=60000); ap.add_argument('--num-envs',type=int,default=16); ap.add_argument('--seed',type=int,default=11); ap.add_argument('--poll-s',type=float,default=10.0); args=ap.parse_args()
 out=Path(args.output).resolve(); out.mkdir(parents=True,exist_ok=True); demo_dir=Path(args.demo_dir).resolve(); demo_dir.mkdir(parents=True,exist_ok=True)
 # Build a fresh, speed-stratified demo buffer; no historical buffer is changed.
 demo_pkl=demo_dir/'structured_visible.pkl'; manifest=demo_dir/'demo_manifest.json'
 if not demo_pkl.exists() or not manifest.exists():
  cmd=[str(PY),str(HIL/'tools/gfr1_make_success_demo.py'),'--raw',str(Path(args.demo_raw).resolve()),'--manifold',str(Path(args.manifold).resolve()),'--output',str(demo_pkl),'--manifest',str(manifest),'--max-episodes','100','--visible-error']
  subprocess.run(cmd,cwd=HIL,check=True)
 # run_gfr1_formal owns creation of the fresh training directory; precreating
 # it would trigger its intentional non-overwrite guard.
 train_out=out/'training'
 env=os.environ.copy(); env.update({'GFR_PYTHON':str(PY),'PYTHONPATH':f'{HIL}/examples:{HIL}:{env.get("PYTHONPATH","")}'.rstrip(':'),'AVIATOR_GFR_MODEL':str(Path(args.model).resolve()),'AVIATOR_GFR_MODE':'structured','AVIATOR_GFR_VISIBLE_ERROR':'1','XLA_PYTHON_CLIENT_PREALLOCATE':'false','XLA_PYTHON_CLIENT_MEM_FRACTION':'0.55','JAX_PLATFORMS':'cuda'})
 if args.train_manifest: env['AVIATOR_GFR_MODEL_MANIFEST']=str(Path(args.train_manifest).resolve())
 train_cmd=[str(PY),str(HIL/'tools/run_gfr1_formal.py'),'--output',str(train_out),'--demo-dir',str(demo_dir),'--seed',str(args.seed),'--num-envs',str(args.num_envs),'--steps',str(args.steps),'--episodes',str(args.num_envs),'--checkpoint-period','20000','--conditions','B4']
 log=(out/'training_wrapper.log').open('w')
 proc=subprocess.Popen(train_cmd,cwd=HIL,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 run_dir=train_out/('b4_seed'+str(args.seed)); eval_dir=out/'evaluations'; eval_dir.mkdir()
 evals={}; eval_procs={}; both_zero=False
 def start_eval(step):
  ck=run_dir/f'checkpoint_{step}'
  if not ck.exists() or step in eval_procs or step in evals: return
  e=env.copy(); e.update({'AVIATOR_GFR_MODE':'structured','AVIATOR_GFR_VISIBLE_ERROR':'1','XLA_PYTHON_CLIENT_MEM_FRACTION':'0.20'})
  if args.eval_manifest: e['AVIATOR_GFR_MODEL_MANIFEST']=str(Path(args.eval_manifest).resolve())
  outj=eval_dir/f'checkpoint_{step}.json'; elog=(eval_dir/f'checkpoint_{step}.log').open('w')
  c=[str(PY),str(HIL/'tools/gfr1_eval.py'),'--checkpoint',str(ck),'--mode','structured','--visible-error','--episodes','10','--output',str(outj)]
  ep=subprocess.Popen(c,cwd=HIL,env=e,stdout=elog,stderr=subprocess.STDOUT); eval_procs[step]=(ep,elog,ck)
  print(f'launched evaluation {step} hash={sha_tree(ck)}',flush=True)
 def harvest():
  for step,(ep,elog,ck) in list(eval_procs.items()):
   rc=ep.poll()
   if rc is None: continue
   elog.close(); p=eval_dir/f'checkpoint_{step}.json'
   if p.exists():
    d=json.loads(p.read_text()); d['checkpoint_hash']=sha_tree(ck); d['returncode']=rc; evals[step]=d
   else: evals[step]={'checkpoint_hash':sha_tree(ck),'returncode':rc,'summary':{'completion_rate':0.0},'instrumentation_unknown':True}
   del eval_procs[step]; print(f'evaluation {step} done rc={rc} completion={evals[step].get("summary",{}).get("completion_rate")}',flush=True)
 while proc.poll() is None:
  for s in (20000,40000): start_eval(s)
  harvest()
  if 20000 in evals and 40000 in evals:
   valid20=not evals[20000].get('instrumentation_unknown',False) and evals[20000].get('returncode')==0; valid40=not evals[40000].get('instrumentation_unknown',False) and evals[40000].get('returncode')==0
   z20=valid20 and float(evals[20000].get('summary',{}).get('completion_rate',0.0))==0.0; z40=valid40 and float(evals[40000].get('summary',{}).get('completion_rate',0.0))==0.0
   if valid20 and valid40 and z20 and z40:
    both_zero=True; print('hard stop: 20k and 40k completion are both zero',flush=True)
    os.killpg(proc.pid,signal.SIGTERM); break
  time.sleep(args.poll_s)
 harvest()
 if proc.poll() is None: proc.wait()
 log.close()
 summary={'kind':'gfr1_async_gate','demo_raw':str(Path(args.demo_raw).resolve()),'demo_manifest':str(manifest),'demo_manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),'training_returncode':proc.returncode,'training_run_dir':str(run_dir),'checkpoint_evaluations':evals,'hard_stopped_both_zero':both_zero,'gate_rule':'continue after 20k zero; stop after 20k=0 and 40k=0; otherwise 60k continuation allowed without convergence claim','frozen':{'num_envs':args.num_envs,'checkpoint_steps':[20000,40000,60000],'eval_episodes':10,'model':str(Path(args.model).resolve())}}
 (out/'gfr1_async_gate_summary.json').write_text(json.dumps(summary,indent=2)+"\n"); print(json.dumps(summary,indent=2))
if __name__=='__main__': main()

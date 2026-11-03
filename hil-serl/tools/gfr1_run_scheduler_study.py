#!/usr/bin/env python3
"""Serial development comparison -> frozen G1--G3 -> formal evaluation.

Never trains; fails closed on instrumentation or modified inputs. Each episode
is flushed by the child. No performance-based replacement or parameter tuning.
"""
from __future__ import annotations
import argparse,csv,json,os,platform,subprocess,sys,time,traceback
from collections import Counter
from pathlib import Path
import numpy as np
from tools.gfr1_same_state_diagnosis import digest,write_json
from tools.gfr1_gate_support import atomic_json
from tools.gfr1_scheduler_benchmark import METHODS

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'outputs/differential_lift_feasibility_20261103/gfr_scheduler_generalization'
CHECKPOINT=ROOT/'outputs/differential_lift_feasibility_20261102/gfr_repaired_scratch/formal_seed11/training/b4_seed11/checkpoint_60000'

def aggregate(directory,group):
    records={m:json.loads((directory/m/'summary.json').read_text()) for m in METHODS}
    n=records['rl']['count']
    assert all(p['count']==n and p['instrumentation_unknown']==0 for p in records.values())
    for i in range(n):
        first=records['rl']['episodes'][i]
        for m in METHODS:
            r=records[m]['episodes'][i]
            for k in ('case_id','trajectory_path','model','initial_q','initial_x'):assert r[k]==first[k],(m,i,k)
            assert r['accepted_validation_violations']==0
    table=[]
    for m,p in records.items():
        rr=p['episodes'];success=[r for r in rr if r['completed']]
        total_progress=sum(r['progress'] for r in rr)
        row=dict(group=group,method=m,cases=n,completed=len(success),completion_rate=len(success)/n,
            mean_progress=float(np.mean([r['progress'] for r in rr])),
            intervention_steps=sum(r.get('interventions',0) for r in rr),
            alternative_recoveries=sum(r.get('alternative_recoveries',0) for r in rr),
            candidate_calls=sum(r['counts'].get('candidate',0) for r in rr),
            distance_calls=sum(r['counts'].get('distance_check',0)+r['counts'].get('distance_pairs',0) for r in rr),
            swept_calls=sum(r['counts'].get('swept',0) for r in rr),
            mean_success_compute_s=float(np.mean([r['total_compute_s'] for r in success])) if success else None,
            mean_all_compute_s=float(np.mean([r['total_compute_s'] for r in rr])),
            compute_s_per_progress=sum(r['total_compute_s'] for r in rr)/total_progress if total_progress else None,
            min_executed_clearance_m=min(r['min_executed_clearance_m'] for r in rr),
            cycle_p95_s=p['cycle_s']['p95'] if p['cycle_s'] else None,cycle_p99_s=p['cycle_s']['p99'] if p['cycle_s'] else None,
            accepted_validation_violations=0,terminations=dict(Counter(r['termination'] for r in rr)))
        row['distance_calls_per_progress']=row['distance_calls']/total_progress if total_progress else None
        row['swept_calls_per_progress']=row['swept_calls']/total_progress if total_progress else None
        row['candidate_calls_per_progress']=row['candidate_calls']/total_progress if total_progress else None
        table.append(row)
    comparisons=[]
    rng=np.random.default_rng(20261103)
    for m in ('fixed','greedy','teacher'):
        rr=records['rl']['episodes'];bb=records[m]['episodes']
        diff=np.array([int(r['completed'])-int(b['completed']) for r,b in zip(rr,bb)])
        boot=diff[rng.integers(0,n,size=(10000,n))].mean(axis=1)
        paired=[(r,b) for r,b in zip(rr,bb) if r['completed'] and b['completed']]
        comparisons.append(dict(baseline=m,completion_difference=float(diff.mean()),paired_bootstrap95=np.percentile(boot,[2.5,97.5]).tolist(),
            both_completed=len(paired),paired_success_task_compute_speedup=float(np.median([b['total_compute_s']/r['total_compute_s'] for r,b in paired])) if paired else None,
            cycle_p95_speedup=records[m]['cycle_s']['p95']/records['rl']['cycle_s']['p95'] if records[m]['cycle_s'] and records['rl']['cycle_s'] else None,
            cycle_p99_speedup=records[m]['cycle_s']['p99']/records['rl']['cycle_s']['p99'] if records[m]['cycle_s'] and records['rl']['cycle_s'] else None,
            note='Task speedup conditional on paired success; failures reported separately. Single training seed. Empirical bootstrap can collapse at all-zero/all-one outcomes; not a population guarantee.'))
    strata=[]
    manifest=json.loads(Path(records['rl']['manifest_path']).read_text()) if 'manifest_path' in records['rl'] else None
    if manifest:
        for stratum in sorted({r.get('stratum','development') for r in manifest['records']}):
            indexes=[i for i,r in enumerate(manifest['records']) if r.get('stratum','development')==stratum]
            for m,p in records.items():
                rr=[p['episodes'][i] for i in indexes]
                strata.append(dict(stratum=stratum,method=m,cases=len(rr),completed=sum(r['completed'] for r in rr),mean_progress=float(np.mean([r['progress'] for r in rr]))))
    per_model=[]
    if manifest:
        for model in sorted({r['model'] for r in manifest['records']}):
            indexes=[i for i,r in enumerate(manifest['records']) if r['model']==model]
            for m,p in records.items():
                rr=[p['episodes'][i] for i in indexes];progress=sum(r['progress'] for r in rr)
                dc=sum(r['counts'].get('distance_check',0)+r['counts'].get('distance_pairs',0) for r in rr)
                sc=sum(r['counts'].get('swept',0) for r in rr)
                per_model.append(dict(model=model,method=m,cases=len(rr),completed=sum(r['completed'] for r in rr),
                    mean_progress=float(np.mean([r['progress'] for r in rr])),
                    total_compute_s=sum(r['total_compute_s'] for r in rr),
                    alternative_recoveries=sum(r.get('alternative_recoveries',0) for r in rr),
                    distance_calls=dc,swept_calls=sc,distance_calls_per_progress=dc/progress if progress else None,
                    swept_calls_per_progress=sc/progress if progress else None))
    payload=dict(group=group,table=table,paired_comparisons=comparisons,strata=strata,per_model=per_model,instrumentation_unknown=0,
        all_initial_states_paired=True,training_started=False,episodes={m:p['episodes'] for m,p in records.items()})
    write_json(directory/'comparison.json',payload)
    with (directory/'comparison.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(table[0]));w.writeheader();w.writerows(table)
    return payload

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=BASE/'study');a=p.parse_args()
    os.chdir(ROOT);a.output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('AVIATOR_','JAX_','XLA_')):del env[key]
    env.update(PYTHONPATH=f'{ROOT}/hil-serl/examples:{ROOT}/hil-serl:{ROOT}',JAX_PLATFORMS='cpu',CUDA_VISIBLE_DEVICES='',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1',
        AVIATOR_GFR_SPLIT='test',AVIATOR_GFR_MODE='structured',AVIATOR_GFR_VISIBLE_ERROR='1')
    for key,val in dict(DT=.005,D_SAFE=.005,BETA_STOP=.125,R_MAX=1.5,QDOT_MAX=1.5).items():env['AVIATOR_GFR_'+key]=str(val)
    sources=[ROOT/'hil-serl/tools'/f for f in ('gfr1_scheduler_benchmark.py','gfr1_prepare_generalization.py','gfr1_run_scheduler_study.py',
        'gfr1_repaired_env.py','gfr1_clearance_variants.py','gfr1_recovery_boundary_ablation.py','gfr1_teacher_replay_consistency.py','validate_branch_connector.py')]
    sources += [ROOT/'hil-serl/examples/experiments/aviator_manifold/gfr_env.py',ROOT/'hil-serl/examples/experiments/aviator_gfr/config.py',BASE/'dev_manifest.json',CHECKPOINT]
    hashes={str(x):digest(x) for x in sources}
    write_json(a.output/'protocol.json',dict(checkpoint=str(CHECKPOINT),input_hashes=hashes,
        execution='Serial CPU-only simulator evaluation; one BLAS thread; no training;100 discarded warmup steps per method per group',
        method_definitions=dict(fixed='raw r=0, same safety projection may change executed r',
            greedy='online null-space joint-midpoint gradient, unit gain, clipped by existing r_max; not old LUT phase-center',
            teacher='previous/zero first; only after all beta candidates fail, expand25 and maximize feasible beta then clearance; cache validated result',
            rl='frozen final60k argmax deterministic'),
        development_cases=4,formal_cases_per_group=30,methods=METHODS,hardware=dict(platform=platform.platform(),processor=platform.processor(),cpu_count=os.cpu_count()),
        gate='Mechanical correctness, same starts, unchanged hashes, no accepted-check violation; no positive-effect selection gate. Negative findings retained.',
        timer='Inclusive overlaps explicitly separated from exclusive partition; full cycle includes next observation; all search work counted once. Audit IO outside cycle.',
        information='All methods supplied same calibrated model; new error magnitude/family reported separately. Single training seed.',
        no_retraining=True))
    child=None;stage='development';completed_jobs=0;start=time.time();summaries={}
    def verify():
        for path,h in hashes.items():assert digest(path)==h,path
    def execute(cmd,logpath,group,method=None,outdir=None):
        nonlocal child,completed_jobs
        verify()
        with logpath.open('x') as log:
            child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
            while child.poll() is None:
                partial=[];progress=None
                if outdir:
                    ep=outdir/'episodes.jsonl'
                    if ep.exists():
                        text=ep.read_text();lines=text.split('\n')[:-1]
                        partial=[json.loads(l) for l in lines if l.strip()]
                    pp=outdir/'progress.json'
                    if pp.exists():progress=json.loads(pp.read_text())
                atomic_json(a.output/'run_status.json',dict(status='running',stage=group,method=method,completed_jobs=completed_jobs,
                    completed_episodes_current_job=len(partial),completed_tasks_current_job=sum(r['completed'] for r in partial),
                    current_progress=progress,supervisor_pid=os.getpid(),child_pid=child.pid,elapsed_s=time.time()-start,updated_unix=time.time()))
                time.sleep(5)
        write_json(logpath.with_suffix('.process.json'),dict(returncode=child.returncode,command=cmd))
        if child.returncode:raise RuntimeError(f'{group}/{method} rc={child.returncode}; see {logpath}')
        verify();completed_jobs+=1
    def run_group(group,manifest,order):
        directory=a.output/group;directory.mkdir()
        for method in order:
            destination=directory/method
            execute([sys.executable,str(ROOT/'hil-serl/tools/gfr1_scheduler_benchmark.py'),'--manifest',str(manifest),
                '--checkpoint',str(CHECKPOINT),'--method',method,'--warmup','100','--output',str(destination)],
                directory/f'{method}.console.log',group,method,destination)
        result=aggregate(directory,group);summaries[group]=result
        report='# 冻结60k策略：调度器配对对照与泛化验收\n\nCPU串行仿真侧计时；未重训。所有方法共用 combined 修复执行器。\n\n'
        report+='| 组 | 方法 | 完成 | 平均进度 | 替代恢复 | 候选调用 | 成功任务平均计算秒 | 周期P95秒 |\n|---|---|---:|---:|---:|---:|---:|---:|\n'
        for g,s in summaries.items():
            for r in s['table']:report+=f"| {g} | {r['method']} | {r['completed']}/{r['cases']} | {r['mean_progress']:.4f} | {r['alternative_recoveries']} | {r['candidate_calls']} | {r['mean_success_compute_s']} | {r['cycle_p95_s']} |\n"
        report+='\n总计算成本只在双方均完成的配对案例上计算任务加速比；各组失败和总成本同时保留。当前单训练seed，已知模型输入，不能解释为未知误差识别或多seed稳定性。开发集结果不算泛化证据。正式分组及误差类型明细见 comparison.json。\n'
        atomic_json(a.output/'partial_summary.json',summaries)
        (a.output/'report.md').write_text(report)
        (ROOT/'docs/research/gfr_scheduler_generalization_2026-11-03.md').write_text(report)
    try:
        run_group('development',BASE/'dev_manifest.json',('rl','fixed','teacher','greedy'))
        selection=a.output/'selection'
        execute([sys.executable,str(ROOT/'hil-serl/tools/gfr1_prepare_generalization.py'),'--output',str(selection)],a.output/'selection.console.log','freeze_G1_G3')
        protocol=json.loads((selection/'selection_protocol.json').read_text());hashes.update(protocol['protected_input_hashes'])
        for group in ('G1','G2','G3'):hashes[str(selection/f'{group}_manifest.json')]=digest(selection/f'{group}_manifest.json')
        write_json(a.output/'formal_input_hashes.json',hashes)
        for group,order in [('G1',('fixed','greedy','teacher','rl')),('G2',('teacher','rl','fixed','greedy')),('G3',('greedy','teacher','rl','fixed'))]:
            stage=group;run_group(group,selection/f'{group}_manifest.json',order)
        verify();write_json(a.output/'summary.json',dict(groups=summaries,source_hashes_unchanged=True,training_started=False))
        atomic_json(a.output/'run_status.json',dict(status='completed',completed_jobs=completed_jobs,summary=str(a.output/'summary.json'),updated_unix=time.time()))
    except BaseException as exc:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        atomic_json(a.output/'run_status.json',dict(status='error',stage=stage,error=str(exc),traceback=traceback.format_exc(),completed_jobs=completed_jobs,updated_unix=time.time()))
        raise

if __name__=='__main__':main()

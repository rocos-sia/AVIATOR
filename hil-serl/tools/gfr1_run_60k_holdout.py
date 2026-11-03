#!/usr/bin/env python3
"""Detached, serial two-batch final-checkpoint test with immutable source audit."""
import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from tools.gfr1_same_state_diagnosis import digest, write_json
from tools.gfr1_gate_support import atomic_json


def summarize(rows):
    if len(rows)!=20 or len({r['trajectory_path'] for r in rows})!=20:
        raise ValueError('Expected all 20 distinct frozen trajectories')
    for r in rows:
        if bool(r['completed']) != (r['termination']=='end' and r['progress']>=1-1e-9):
            raise ValueError('Completion/progress/termination mismatch')
    result=dict(episodes=20,completed=sum(r['completed'] for r in rows),
        completion_rate=sum(r['completed'] for r in rows)/20,
        termination_counts=dict(Counter(r['termination'] for r in rows)),
        accepted_validation_violations=sum(r['accepted_validation_violations'] for r in rows))
    for key in ('progress','alternative_recoveries','boundary_correction_iterations',
                'actual_candidate_evaluations','intervention_steps','endpoint_rejection_steps',
                'endpoint_failed_candidates','swept_failed_candidates','elapsed_s',
                'minimum_executed_clearance_m','max_closure_residual_inf'):
        result['mean_'+key]=float(np.mean([r[key] for r in rows]))
    result['minimum_executed_clearance_m']=min(r['minimum_executed_clearance_m'] for r in rows)
    result['by_model']={m:dict(episodes=len(rr),completed=sum(r['completed'] for r in rr),
        mean_progress=float(np.mean([r['progress'] for r in rr]))) for m in sorted({r['model'] for r in rows})
        for rr in [[r for r in rows if r['model']==m]]}
    return result


def partial_episodes(path):
    if not path.exists():return 0
    count=0
    for line in path.read_text().splitlines():
        try:json.loads(line);count+=1
        except json.JSONDecodeError:pass
    return count


def main():
    p=argparse.ArgumentParser();p.add_argument('--selection',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[2];hil=root/'hil-serl'
    selection=a.selection.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((selection/'selection_protocol.json').read_text())
    manifest=json.loads((selection/'heldout_manifest.json').read_text())
    init=json.loads((selection/'initialization_audit.json').read_text())
    if init['valid']!=20:
        write_json(out/'initialization_failures.json',init)
        raise RuntimeError('Frozen invalid starts retained; no policy run or outcome-based sample replacement')
    frozen=json.loads(Path(protocol['training_frozen_config']).read_text())
    protected=dict(protocol['protected_input_hashes'])
    for path in [selection,Path(__file__),hil/'tools/gfr1_holdout60k_eval.py',hil/'tools/gfr1_prepare_60k_holdout.py']:
        protected[str(path)]=digest(path)
    for path,h in protected.items():assert digest(path)==h,path
    write_json(out/'protocol.json',dict(selection_protocol=str(selection/'selection_protocol.json'),
        checkpoint=protocol['checkpoint'],checkpoint_sha256=protocol['checkpoint_sha256'],
        protected_input_hashes=protected,policy='Frozen final 60k; deterministic; no updates',
        batches=2,episodes_per_batch=10,device='cpu',executor='combined_repair_v1',
        safety='Unchanged endpoint/discrete swept checks, no continuous-time safety claim',
        interpretation=protocol['interpretation'],training_started=False))
    env=os.environ.copy()
    for k in list(env):
        if k.startswith(('AVIATOR_','JAX_','XLA_')):del env[k]
    for key in ('AVIATOR_GFR_MANIFOLD','AVIATOR_GFR_TRAJECTORY_DIR','AVIATOR_GFR_MODEL'):
        env[key]=frozen['environment'][key]
    for key in ('dt','d_safe','beta_stop','r_max','qdot_max'):
        env['AVIATOR_GFR_'+key.upper()]=str(frozen[key])
    env.update(PYTHONPATH=f'{hil}/examples:{hil}:{root}',JAX_PLATFORMS='cpu',CUDA_VISIBLE_DEVICES='',
               AVIATOR_GFR_SPLIT='test',AVIATOR_GFR_MODE='structured',AVIATOR_GFR_VISIBLE_ERROR='1',
               OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
    rows=[];batch_records=[];child=None;started=time.time()
    try:
        for batch in range(2):
            directory=out/f'batch_{batch}';directory.mkdir()
            env['AVIATOR_GFR_MODEL_MANIFEST']=str(selection/f'batch_{batch}_manifest.json')
            env['AVIATOR_REPAIRED_TRACE_DIR']=str(directory/'executor_traces')
            result=directory/'evaluation.json'
            cmd=[sys.executable,str(hil/'tools/gfr1_holdout60k_eval.py'),'--checkpoint',protocol['checkpoint'],
                 '--mode','structured','--visible-error','--full-horizon','--episodes','10','--output',str(result)]
            with (directory/'console.log').open('x') as log:
                child=subprocess.Popen(cmd,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
                print(f'BATCH {batch} started pid={child.pid}',flush=True)
                while child.poll() is None:
                    atomic_json(out/'run_status.json',dict(status='running',supervisor_pid=os.getpid(),
                        evaluator_pid=child.pid,current_batch=batch,completed_batches=len(batch_records),
                        completed_episodes=len(rows)+partial_episodes(result.with_suffix('.episodes.jsonl')),
                        selected_episodes=20,updated_unix=time.time(),elapsed_s=time.time()-started))
                    time.sleep(5)
            write_json(directory/'process.json',dict(returncode=child.returncode))
            if child.returncode!=0:raise RuntimeError(f'Batch {batch} evaluation failed: rc={child.returncode}')
            report=json.loads(result.read_text())
            assert report['instrumentation_unknown']==0
            assert report['checkpoint_sha256']==protocol['checkpoint_sha256']
            assert report['split']=='test' and report['executor']=='combined_repair_v1'
            assert len(report['episodes'])==10
            for local,row in enumerate(report['episodes']):
                index=batch*10+local;rec=manifest['records'][index]
                assert row['trajectory_path']==rec['trajectory_path'] and row['model']==rec['model']
                row=dict(row,global_episode=index,batch=batch)
                rows.append(row)
            batch_records.append(dict(batch=batch,raw_json=str(result),sha256=digest(result),summary=report['summary']))
            print(f'BATCH {batch} finished: {report["summary"]}',flush=True)
        summary=summarize(rows)
        for path,h in protected.items():assert digest(path)==h,f'Frozen input changed: {path}'
        with (out/'episodes.csv').open('x') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        result=dict(kind='gfr_final60k_new_trajectory_evaluation',summary=summary,episodes=rows,batches=batch_records,
            checkpoint=protocol['checkpoint'],checkpoint_sha256=protocol['checkpoint_sha256'],
            source_hashes_unchanged=True,instrumentation_unknown=0,interpretation=protocol['interpretation'],training_started=False)
        write_json(out/'summary.json',result)
        text='# 冻结 60k 策略的新轨迹评估（2026-11-03）\n\n'
        text+='固定最终 checkpoint_60000；20 条新 test 轨迹，四个既有扰动模型各 5 条，未重训或选择最佳 checkpoint。\n\n'
        text+=f'完整完成：**{summary["completed"]}/20**；平均 progress={summary["mean_progress"]:.6f}。\n\n'
        text+='| 指标 | 结果 |\n|---|---:|\n'
        for key in ('mean_alternative_recoveries','mean_boundary_correction_iterations','mean_intervention_steps',
                    'mean_actual_candidate_evaluations','minimum_executed_clearance_m','accepted_validation_violations'):
            text+=f'| {key} | {summary[key]} |\n'
        text+='\n停止原因：`'+json.dumps(summary['termination_counts'],ensure_ascii=False)+'`。\n\n'
        text+='本次检验最终策略与修复执行器组合在新轨迹上的表现；四种扰动模型已在开发阶段使用，不构成未见扰动模型泛化。没有额外基线，不能仅凭这次成绩归因于 RL。所有阈值、checkpoint 和冻结代码哈希保持一致；安全结论限于既定端点/离散 swept 检查。\n\n'
        text+='原始结果：'+str(out/'summary.json')+'\n'
        (out/'report.md').write_text(text)
        (root/'docs/research/gfr_final60k_new_trajectory_validation_2026-11-03.md').write_text(text)
        status_path=root/'docs/research/long_horizon_status.md';s=status_path.read_text()
        marker='- Node: GFR final60k prospective new-trajectory evaluation (2026-11-03).'
        if marker in s:
            s=s.replace('- Status: running; 20 frozen new test trajectories, two serial batches of 10.',
                f'- Status: completed; {summary["completed"]}/20 complete, mean progress={summary["mean_progress"]:.6f}; accepted-validation violations={summary["accepted_validation_violations"]}; protected input hashes unchanged.')
            status_path.write_text(s)
        atomic_json(out/'run_status.json',dict(status='completed',completed_batches=2,completed_episodes=20,
            selected_episodes=20,summary=str(out/'summary.json'),updated_unix=time.time()))
        write_json(out/'job_complete.json',dict(status='completed_and_verified',training_started=False,summary=summary))
    except BaseException as e:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        atomic_json(out/'run_status.json',dict(status='error',error=str(e),completed_batches=len(batch_records),
            completed_episodes=len(rows),instrumentation_unknown=1,updated_unix=time.time()))
        raise


if __name__=='__main__':main()

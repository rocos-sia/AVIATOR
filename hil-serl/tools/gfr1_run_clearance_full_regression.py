#!/usr/bin/env python3
"""Serial full-path regression selected by a predeclared same-state gate."""
import csv,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
from tools.gfr1_same_state_diagnosis import digest,write_json
from tools.gfr1_gate_support import atomic_json

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'outputs/differential_lift_feasibility_20261103'
OUT=BASE/'gfr_clearance_ablation/full_regression'


def main():
    os.chdir(ROOT)
    snapshots=BASE/'gfr_clearance_ablation/snapshots_v2'
    gate=json.loads((snapshots/'summary.json').read_text())
    variant=gate['selected_for_whole_path']
    if not variant or gate['instrumentation_unknown'] or not gate['source_hashes_unchanged']:
        raise RuntimeError('Snapshot gate not satisfied')
    oldgate=json.loads((BASE/'gfr_clearance_ablation/snapshots/summary.json').read_text())
    for k in gate['groups']:
        for metric in ('accepted','sum_accepted_beta','euler_checks','candidate_calls','linear_empty','affine_violation_calls'):
            assert gate['groups'][k][metric]==oldgate['groups'][k][metric],(k,metric)
    frozen=json.loads((BASE/'gfr_final60k_holdout/evaluation/protocol.json').read_text())
    protected=dict(frozen['protected_input_hashes'])
    for path in [snapshots,Path(__file__),ROOT/'hil-serl/tools/gfr1_clearance_variants.py',ROOT/'hil-serl/tools/gfr1_clearance_regression_env.py',ROOT/'hil-serl/tools/gfr1_clearance_regression_eval.py']:
        protected[str(path)]=digest(path)
    for path,h in protected.items():assert digest(path)==h,path
    OUT.mkdir(parents=True,exist_ok=False)
    write_json(OUT/'protocol.json',dict(variant=variant,checkpoint=frozen['checkpoint'],checkpoint_sha256=frozen['checkpoint_sha256'],protected_input_hashes=protected,
        snapshot_gate=str(snapshots/'summary.json'),snapshots_v1_v2_decisions_identical=True,
        execution='CPU serial, first two episodes measured individually; original 20 starts/trajectories/models, same checkpoint and original final checker; no training',
        comparison='Historical frozen original20 outcomes reused; no claim of matched-runtime speedup; these cases are now development regression, not independent new holdout',
        old_counter_warning='Historical endpoint_failed_candidates/swept_failed_candidates have missing-value sentinel ambiguity; raw counters are retained but not used as physical cause comparisons. New Euler attempt rejection counters are separately logged.'))
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('AVIATOR_','JAX_','XLA_')):del env[key]
    env.update(PYTHONPATH=f'{ROOT}/hil-serl/examples:{ROOT}/hil-serl:{ROOT}',JAX_PLATFORMS='cpu',CUDA_VISIBLE_DEVICES='',
        AVIATOR_GFR_SPLIT='test',AVIATOR_GFR_MODE='structured',AVIATOR_GFR_VISIBLE_ERROR='1',AVIATOR_CLEARANCE_VARIANT=variant,
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
    # Environment defaults match the frozen numeric fields; set them explicitly.
    for key,value in dict(DT=.005,D_SAFE=.005,BETA_STOP=.125,R_MAX=1.5,QDOT_MAX=1.5).items():env['AVIATOR_GFR_'+key]=str(value)
    jobs=[(0,0,1),(0,1,1),(0,2,8),(1,0,10)]
    rows=[];child=None;started=time.time();pilot_estimate=None
    try:
        for job,(batch,start,count) in enumerate(jobs):
            directory=OUT/f'job_{job}';directory.mkdir()
            result=directory/'evaluation.json';epfile=directory/'evaluation.episodes.jsonl'
            env['AVIATOR_GFR_MODEL_MANIFEST']=str(BASE/f'gfr_final60k_holdout/selection/batch_{batch}_manifest.json')
            env['AVIATOR_CLEARANCE_TRACE_DIR']=str(directory/'candidate_traces')
            cmd=[sys.executable,str(ROOT/'hil-serl/tools/gfr1_clearance_regression_eval.py'),
                 '--checkpoint',frozen['checkpoint'],'--mode','structured','--visible-error','--full-horizon',
                 '--episode-start',str(start),'--episodes',str(count),'--output',str(result)]
            with (directory/'console.log').open('x') as log:
                child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
                while child.poll() is None:
                    partial=[json.loads(l) for l in epfile.read_text().splitlines() if l] if epfile.exists() else []
                    atomic_json(OUT/'run_status.json',dict(status='running',variant=variant,supervisor_pid=os.getpid(),evaluator_pid=child.pid,current_job=job,
                        completed_episodes=len(rows)+len(partial),total_episodes=20,elapsed_s=time.time()-started,
                        pilot_estimated_total_s=pilot_estimate,updated_unix=time.time()))
                    time.sleep(5)
            write_json(directory/'process.json',dict(returncode=child.returncode))
            if child.returncode:raise RuntimeError(f'job{job} failed, rc={child.returncode}')
            payload=json.loads(result.read_text());assert payload['instrumentation_unknown']==0
            assert payload['checkpoint_sha256']==frozen['checkpoint_sha256'] and len(payload['episodes'])==count
            assert payload['executor']=='clearance_ablation_'+variant
            for row in payload['episodes']:
                rows.append(dict(row,global_episode=batch*10+row['episode']))
            if len(rows)<=2:
                pilot_estimate=float(np.mean([r['elapsed_s'] for r in rows]))*20
                write_json(OUT/f'pilot_{len(rows)}_timing.json',dict(episodes=len(rows),mean_episode_s=pilot_estimate/20,estimated_20_episode_s=pilot_estimate))
            print(json.dumps(dict(job=job,episodes=len(rows),completed=sum(r['completed'] for r in rows),elapsed_s=time.time()-started,pilot_estimate_s=pilot_estimate)),flush=True)
        original=json.loads((BASE/'gfr_final60k_holdout/evaluation/summary.json').read_text())
        assert len(rows)==20 and len({r['global_episode'] for r in rows})==20
        baseline={r['global_episode']:r for r in original['episodes']}
        for row in rows:
            ref=baseline[row['global_episode']]
            assert row['trajectory_path']==ref['trajectory_path'] and row['model']==ref['model']
            assert row['accepted_validation_violations']==0
        from collections import Counter
        def metrics(rr):
            return dict(episodes=len(rr),completed=sum(r['completed'] for r in rr),mean_progress=float(np.mean([r['progress'] for r in rr])),
                alternative_recoveries=sum(r['alternative_recoveries'] for r in rr),boundary_corrections=sum(r['boundary_correction_iterations'] for r in rr),
                candidate_evaluations=sum(r['actual_candidate_evaluations'] for r in rr),interventions=sum(r['intervention_steps'] for r in rr),
                total_steps=sum(r['steps'] for r in rr),mean_elapsed_s=float(np.mean([r['elapsed_s'] for r in rr])),
                accepted_validation_violations=sum(r['accepted_validation_violations'] for r in rr),terminations=dict(Counter(r['termination'] for r in rr)))
        summary=dict(original=metrics(original['episodes']),selected=metrics(rows))
        trace_counts=Counter()
        for path in OUT.glob('job_*/candidate_traces/*.jsonl'):
            for line in path.open():
                row=json.loads(line)
                for key in ('projection_calls','euler_checks','euler_endpoint_rejections','euler_swept_rejections','linear_empty_count','affine_violation_calls'):
                    trace_counts[key]+=row[key]
        for path,h in protected.items():assert digest(path)==h,path
        write_json(OUT/'summary.json',dict(variant=variant,summary=summary,episodes=rows,selected_projection_counters=trace_counts,
            source_hashes_unchanged=True,instrumentation_unknown=0,training_started=False))
        with (OUT/'episodes.csv').open('x',newline='') as stream:
            w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        report='# Clearance 约束与回退消融：完整轨迹回归（2026-11-03）\n\n'
        report+=f'冻结 final60k，按预声明同状态门槛选择 `{variant}`，在原20条轨迹上从头运行。属于开发回归，不是新的泛化测试。\n\n'
        report+='| 指标 | 原流程 | 本轮 |\n|---|---:|---:|\n'
        for key in ('completed','mean_progress','alternative_recoveries','boundary_corrections','candidate_evaluations','interventions','total_steps','accepted_validation_violations'):
            report+=f'| {key} | {summary["original"][key]} | {summary["selected"][key]} |\n'
        report+='\n原流程复用冻结结果；本轮时延保留在JSON中，但没有新跑同时计时对照，不声称实时速度优势。原 endpoint/swept 失败汇总字段存在已记录的日志口径问题，不能与本轮细分 Euler 拒绝计数直接比较。\n\n'
        if summary['selected']['completed']==20 and summary['selected']['alternative_recoveries']<summary['original']['alternative_recoveries']:
            report+='结论：固定策略下，全程完成保持且替代恢复减少，支持执行机制修复。暂不以此要求重训；下一步应冻结修复版，在新的未参与诊断轨迹上验收。\n'
        else:report+='结论：同状态改善未充分转化为全程改善。先定位本轮失败或代价变化，不能把局部接受率写成任务完成率，也不据此直接重训。\n'
        (OUT/'report.md').write_text(report)
        (ROOT/'docs/research/gfr_clearance_ablation_results_2026-11-03.md').write_text(report)
        status=ROOT/'docs/research/long_horizon_status.md';text=status.read_text()
        text=text.replace('- Status: clearance ablation full regression running; no training.',f'- Status: clearance ablation completed; selected {variant}, completed {summary["selected"]["completed"]}/20, alternative recoveries {summary["selected"]["alternative_recoveries"]}; no training.')
        status.write_text(text)
        atomic_json(OUT/'run_status.json',dict(status='completed',completed_episodes=20,total_episodes=20,summary=str(OUT/'summary.json'),updated_unix=time.time()))
        write_json(OUT/'job_complete.json',dict(status='completed_and_verified',summary=summary))
    except BaseException as exc:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        atomic_json(OUT/'run_status.json',dict(status='error',error=str(exc),completed_episodes=len(rows),updated_unix=time.time()))
        raise
if __name__=='__main__':main()

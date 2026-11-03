#!/usr/bin/env python3
"""Audit and summarize completed immutable full-start regression output."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np

from tools.gfr1_same_state_diagnosis import digest, write_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    args = p.parse_args()
    out = args.output
    summary = json.loads((out/'summary.json').read_text())
    protocol = json.loads((out/'protocol.json').read_text())
    rows = summary['episodes']
    assert len(rows) == 30 and summary['original_reproduced'] == 10
    assert protocol['input_hashes'] == {path:digest(path) for path in protocol['input_hashes']}
    aggregate, failure_rows = [], []
    accepted_total = 0
    for method in ('original','alternative_only','combined'):
        rr = [r for r in rows if r['method'] == method]
        timing, betas = [], []
        for row in rr:
            case = out/f'episode_{row["episode"]:02d}_{method}'
            traces = [json.loads(s) for s in (case/'trace.jsonl').read_text().splitlines()]
            assert len(traces) == row['cycles']
            accepted = [s for s in traces if s['accepted']]
            assert len(accepted) == row['accepted_steps']
            assert all(not s['independent_check']['violations'] for s in accepted)
            accepted_total += len(accepted)
            assert all(s['beta'] >= protocol['frozen']['beta_stop'] for s in accepted)
            progress = [s['progress'] for s in accepted]
            assert all(b > a for a,b in zip(progress,progress[1:]))
            timing += [s['diagnostic_cycle_s'] for s in traces]
            betas += [s['beta'] for s in accepted]
            if row['stop_reason'] == 'governor_exhausted':
                stop = json.loads((case/'stop_candidates.json').read_text())
                reasons = Counter(reason for h in stop['history'] for t in h['trials'] for reason in t['violated'])
                failure_rows.append(dict(episode=row['episode'],method=method,progress=row['progress'],
                    reason_counts=json.dumps(dict(reasons),sort_keys=True),
                    note='All searched candidates rejected; not a proof of local infeasibility',
                    snapshot=str(case/'pre_stop_state.pkl')))
        total_progress = sum(r['progress'] for r in rr)
        aggregate.append(dict(method=method,cases=len(rr),completed=sum(r['completed'] for r in rr),
            mean_progress=float(np.mean([r['progress'] for r in rr])),
            total_interventions=sum(r.get('intervention_steps',0) for r in rr),
            total_projection_or_retry_steps=sum(r.get('projection_or_retry_steps',0) for r in rr),
            total_endpoint_failed_candidates=sum(r.get('endpoint_failed_candidates',0) for r in rr),
            total_swept_failed_candidates=sum(r.get('swept_failed_candidates',0) for r in rr),
            total_alternative_steps=sum(r.get('alternative_steps',0) for r in rr),
            total_boundary_correction_steps=sum(r.get('boundary_correction_steps',0) for r in rr),
            candidate_evaluations_per_unit_progress=sum(r['candidate_evaluations'] for r in rr)/max(total_progress,1e-12),
            diagnostic_wall_seconds_per_unit_progress=sum(r['elapsed_s'] for r in rr)/max(total_progress,1e-12),
            diagnostic_cycle_p50_s=float(np.percentile(timing,50)),
            diagnostic_cycle_p95_s=float(np.percentile(timing,95)),
            min_clearance_m=min(r['minimum_executed_clearance_m'] for r in rr),
            max_closure_mixed_inf=max(r['max_closure_residual_inf'] for r in rr),
            mean_accepted_beta=float(np.mean(betas)),
            stops=json.dumps(dict(Counter(r['stop_reason'] for r in rr)),sort_keys=True)))
    for filename, items in [('aggregate.csv',aggregate),('remaining_failure_constraints.csv',failure_rows)]:
        with (out/filename).open('x') as f:
            if items:
                w=csv.DictWriter(f,fieldnames=list(items[0])); w.writeheader(); w.writerows(items)
            else:
                f.write('episode,method,progress,reason_counts,note,snapshot\n')
    lines=['40k 旧策略全程回归：原执行流程 vs 修复执行流程',
           '不重训。10 个已用于诊断的任务从起点重置，三组共 30 次。',
           '原流程全部复现历史停止步数和进度。任务完成要求 termination=end 且 progress 到达 1。',
           '', '方法 | 完成 | 平均进度 | 替代恢复步数 | 边界修正步数 | 最小实际 clearance (m)']
    for r in aggregate:
        lines.append(f'{r["method"]} | {r["completed"]}/{r["cases"]} | {r["mean_progress"]:.6f} | {r["total_alternative_steps"]} | {r["total_boundary_correction_steps"]} | {r["min_clearance_m"]:.10f}')
    lines += ['', f'独立复核的接受动作：{accepted_total}；检查违规：0。',
        '解释边界：',
        '- 恢复动作来自额外的确定性候选搜索；若完成率提高，应归因于旧策略与修复执行机制的组合，不能称旧 RL 已独立收敛。',
        '- 增加候选搜索会增加计算。aggregate.csv 同时报告单位进度候选次数和诊断耗时；该耗时包含独立重复验证，不是部署实时性能。',
        '- 本轮修复仅补偿 joint-limit-only midpoint 误差并搜索替代动作；安全阈值和最终 nonlinear checker 没有放宽。',
        '- 所有结论限于这 10 个开发回归案例，不能作为新 held-out 泛化证据；离散 swept 检查不是连续时间安全证明。',
        '- 剩余停止只说明有限候选搜索耗尽，不能推断真实局部不可行。',
        '- 纠偏数据优先来自经验证能够持续推进的替代动作；本轮诊断集不能不加区分混入训练再作为测试集。',
        '- 本轮没有启动训练。先根据剩余失败快照决定纠偏采样范围，再以独立任务集验证。']
    (out/'report.txt').write_text('\n'.join(lines)+'\n')
    write_json(out/'verification.json',dict(episodes=30,original_reproduced=10,
        accepted_steps_independently_checked=accepted_total,accepted_validation_violations=0,
        protected_hashes_unchanged=True,complete_trace_counts=True,strict_positive_progress=True,
        training_started=False))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()

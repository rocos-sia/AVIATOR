#!/usr/bin/env python3
"""Read-only event-level recovery attribution from already saved counters.

Counters establish check rejection patterns, not exclusive physical causes.
No new rollout, parameter change, or overwrite of held-out evidence.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np
from tools.gfr1_same_state_diagnosis import digest, write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    summary=json.loads((a.source/'summary.json').read_text());episodes={r['global_episode']:r for r in summary['episodes']}
    hashes={str(a.source/'summary.json'):digest(a.source/'summary.json')};rows=[]
    for batch in (0,1):
        for file in sorted((a.source/f'batch_{batch}/executor_traces').glob('*.jsonl')):
            hashes[str(file)]=digest(file)
            for line in file.open():
                r=json.loads(line)
                if not r['alternative_recovered']:continue
                index=batch*10+r['episode']-1;ep=episodes[index]
                assert r['model']==ep['model']
                rejected=r['actual_candidate_evaluations']-1
                end=r['actual_endpoint_failed_candidates'];swept=r['actual_swept_failed_candidates'];corrections=r['boundary_correction_iterations']
                if end==0 and swept==rejected and corrections==0:
                    label='swept_failure_on_every_rejected_candidate_no_endpoint_failure_flag'
                elif corrections>0:
                    label='swept_failure_with_boundary_correction'
                else:
                    label='endpoint_failure_flag_also_present'
                rows.append(dict(global_episode=index,trajectory=Path(ep['trajectory_path']).name,model=Path(r['model']).name,
                    step=r['step'],progress=r['progress'],pattern=label,candidate_evaluations=r['actual_candidate_evaluations'],
                    endpoint_failed_candidates=end,swept_failed_candidates=swept,boundary_corrections=corrections,
                    selected_nominal_zero=bool(np.all(np.asarray(r['selected_action_raw'])==0)),
                    accepted_beta=r['beta_executed'],accepted_clearance_m=r['d_actual_m'],
                    policy_raw=r['policy_action_raw'],selected_raw=r['selected_action_raw'],executed_r=r['action_executed_r']))
    assert len(rows)==sum(r['alternative_recoveries'] for r in episodes.values())==198
    for index,ep in episodes.items():assert sum(r['global_episode']==index for r in rows)==ep['alternative_recoveries']
    result=dict(events=len(rows),episodes_with_recovery=len({r['global_episode'] for r in rows}),
        patterns=dict(Counter(r['pattern'] for r in rows)),by_model=dict(Counter(r['model'] for r in rows)),
        by_trajectory=dict(Counter(r['trajectory'] for r in rows)),events_progress_ge_0_8=sum(r['progress']>=.8 for r in rows),
        selected_nominal_zero=sum(r['selected_nominal_zero'] for r in rows),accepted_beta_counts=dict(Counter(str(r['accepted_beta']) for r in rows)),
        source_hashes=hashes,interpretation='Primary observed trigger is swept-check rejection after exhausting the policy beta schedule. Aggregate counters do not exclude simultaneous joint/speed violations, and cannot separate true geometry from threshold/linearization errors. Zero selected nominal r is subsequently projected and is not zero robot velocity.',
        no_retraining=True,no_new_rollout=True)
    with (a.output/'recovery_events.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    write_json(a.output/'summary.json',result)
    for path,h in hashes.items():assert digest(path)==h
    report='''# 60k 新轨迹替代恢复：日志归因（2026-11-03）

读取既有 20 条完整测试日志，不重训、不回放，不改动正式 JSON。

## 结果

共 198 次替代恢复，发生在 12/20 条轨迹上：

| 日志模式 | 恢复次数 |
|---|---:|
| 全部拒绝候选均标记 swept 失败，无端点失败标记、无边界重算 | 191（96.5%） |
| swept 失败伴随积分边界修正 | 6（3.0%） |
| 同时出现端点失败标记 | 1（0.5%） |

触发条件是策略原动作在 beta=1、0.75、0.5、0.25、0.125 的整组尝试后仍没有可接受候选。恢复搜索更换名义 phase-rate，再由同一投影器和 nonlinear checker 验收。

132/198 次发生在 progress>=0.8。基座旋转 0.25 度模型占 146 次；其中 traj_0013 为 83 次、traj_0025 为 59 次。该集中现象是当前样本中的观察，模型/轨迹没有做全交叉，不能据此单独认定基座旋转是因果因素。

137/198 次选中的替代名义动作是 r=[0,0]。它仍会经过投影，实际执行 r 可以非零；任务推进和闭合反馈继续工作。因此不是机械臂停止，也不是将零速记为成功。

## 解释边界

主要可确认的触发机制是 swept 检查拒绝，而非策略完全无法执行整条轨迹。端点通过并不代表连接中间段通过。安全拒绝的是候选动作，不是已经发生的执行碰撞。

现有 trace 保存的是汇总计数及最终接受动作，未保存每个拒绝候选的距离、contact、速度及关节 margin。因此不能排除同时存在关节/速度约束，不能把 191 次称作唯一由碰撞引起，也不能确认拒绝距离究竟低于阈值多少、是否存在数值边界效应。最终接受动作靠近 5 mm 边界，但不能用它替代被拒绝候选的距离。

当前证据支持：策略在部分新轨迹的局部几何边界上仍需换方向恢复，且后段较集中。它不支持仅凭这些计数认定奖励、训练步数或示范数量是根因。本次不改算法或启动重训。
'''
    (a.output/'report.md').write_text(report)
    print(json.dumps({k:v for k,v in result.items() if k!='source_hashes'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()

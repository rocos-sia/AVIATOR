# Clearance 约束与回退消融：完整轨迹回归（2026-11-03）

冻结 final60k，按预声明同状态门槛选择 `combined`，在原20条轨迹上从头运行。属于开发回归，不是新的泛化测试。

| 指标 | 原流程 | 本轮 |
|---|---:|---:|
| completed | 20 | 20 |
| mean_progress | 1.0 | 1.0 |
| alternative_recoveries | 198 | 0 |
| boundary_corrections | 16204 | 15843 |
| candidate_evaluations | 61126 | 56370 |
| interventions | 27283 | 26940 |
| total_steps | 40647 | 40111 |
| accepted_validation_violations | 0 | 0 |

原流程复用冻结结果；本轮时延保留在JSON中，但没有新跑同时计时对照，不声称实时速度优势。原 endpoint/swept 失败汇总字段存在已记录的日志口径问题，不能与本轮细分 Euler 拒绝计数直接比较。

结论：固定策略下，全程完成保持且替代恢复减少，支持执行机制修复。暂不以此要求重训；下一步应冻结修复版，在新的未参与诊断轨迹上验收。

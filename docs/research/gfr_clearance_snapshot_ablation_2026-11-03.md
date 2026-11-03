# 冻结 60k：多距离约束与仿射可行回退的同状态消融

日期：2026-11-03。完整轨迹回归另见 `gfr_clearance_ablation_results_2026-11-03.md`（运行完成后生成）。

## 范围与冻结项

使用原20条评估轨迹中12条上的198个替代恢复前状态。冻结最终 checkpoint_60000；此阶段直接使用原日志中的确定性策略动作，不更新网络。恢复 q、x、任务进度和零空间基底，对同一状态测试原五档 beta。保留 dt=0.005、d_safe=0.005 m、速度及关节限位、midpoint 边界补偿、最终 endpoint 和离散 swept 检查。

这198个状态是有意选出的历史失败点，结果只代表局部诊断，不是无偏成功率、完整任务完成率或新的泛化结论。

## 两个独立变量

1. 多距离约束：对原 checker 的每个 mesh-cloud/wall 距离单独建立半空间，约束全部固定组合，不引入新的警戒距离或 curvature margin。保持原点云、墙面、距离公式和1e-5 rad有限差分步长。最小分量与原全局 clearance 一致。
2. 可行域内回退：保持 beta 不变，将 r 朝裁剪后凸多边形顶点的均值进行二分插值。最多6次，与原尝试预算一致。没有朝零点缩小的要求；最终非线性 checker 必须通过。

保留 original、multi、affine、combined 四组；本阶段只检查原策略的 beta schedule，不启动替代动作搜索。

## 结果

| 组别 | 原策略 schedule 通过 | 通过时 beta 总和 | Euler 检查次数 | 候选投影调用 |
|---|---:|---:|---:|---:|
| 原流程 | 0/198 | 0 | 5930 | 990 |
| 仅多距离约束 | 183/198 | 119.875 | 2141 | 511 |
| 仅仿射可行回退 | 181/198 | 181 | 917 | 266 |
| 两项同时启用 | 198/198 | 198 | 297 | 198 |

每个接受动作都经过原最终检查与额外独立复查，未发现接受状态违反冻结检查条件。combined 的198个状态均在 beta=1 时接受，但这些动作经过局部投影/回退，不能称为原始网络动作直接安全。

结果支持两个已定位机制均有作用，组合在这些状态上消除了对替代动作搜索的需求。不能推断整条轨迹不再需要替代恢复，也不能把该结果写成 RL 权重改进。

## 审查与数值契约

独立代码审查发现初版 diagnostic assertion 的1e-9限值比原多边形投影容差更严，可能在尚未回退时把原投影数值尾差判为错误。已改为不超过初始投影点与参考点的最大半空间违反量加1e-12；动作公式、原 nonlinear checker 和安全阈值均未改。

- 增加了带偏移仿射集、单点/线段退化、原投影一致性和审查最小复现测试。
- 16项相关测试通过。
- v1与v2各运行完整198状态×4组；792条记录逐候选决策一致，不仅汇总相同。
- v1原源码保存在 snapshots/projector_source_v1.py 和 runner_source_v1.py。正式后续使用 snapshots_v2 冻结代码。
- 所有原实验文件、权重和原执行源文件保持不变。

## 完整回归规则

依据实验前写入 protocol 的规则，按 schedule 通过数、通过 beta 总和、Euler 检查次数选择变体；本轮选择 combined。

按原20个模型、轨迹、初值和种子从头执行。先单独运行前两条估算耗时，再串行运行余下18条。逐步和逐episode保存结果。保留原替代动作恢复作为兜底，统计其是否仍被调用。对照复用已冻结的原20条结果，不声称严格匹配计时的加速证据。

完整回归完成前，不认定修复获得全程改善，不启动重训。由于这些轨迹已参与诊断，后续独立验收应使用新的保留轨迹。

## 可审计文件

- `outputs/differential_lift_feasibility_20261103/gfr_clearance_ablation/snapshots_v2/protocol.json`
- 同目录 `raw.jsonl`、`summary.json`、`comparison.csv`
- `outputs/differential_lift_feasibility_20261103/gfr_clearance_ablation/verification.json`
- 全程进度：`full_regression/run_status.json`
- 全程原始记录：`full_regression/job_*/evaluation.episodes.jsonl` 和 `candidate_traces/*.jsonl`

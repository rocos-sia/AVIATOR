# 冻结 60k 策略的新轨迹评估（2026-11-03）

固定最终 checkpoint_60000；20 条新 test 轨迹，四个既有扰动模型各 5 条，未重训或选择最佳 checkpoint。

完整完成：**20/20**；平均 progress=1.000000。

| 指标 | 结果 |
|---|---:|
| mean_alternative_recoveries | 9.9 |
| mean_boundary_correction_iterations | 810.2 |
| mean_intervention_steps | 1364.15 |
| mean_actual_candidate_evaluations | 3056.3 |
| minimum_executed_clearance_m | 0.004999998956918716 |
| accepted_validation_violations | 0 |

停止原因：`{"end": 20}`。

本次检验最终策略与修复执行器组合在新轨迹上的表现；四种扰动模型已在开发阶段使用，不构成未见扰动模型泛化。没有额外基线，不能仅凭这次成绩归因于 RL。所有阈值、checkpoint 和冻结代码哈希保持一致；安全结论限于既定端点/离散 swept 检查。

原始结果：/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261103/gfr_final60k_holdout/evaluation/summary.json

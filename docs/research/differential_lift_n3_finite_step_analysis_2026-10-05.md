# N3：有限步 clearance 预测误差与线性过滤器边界分析

**节点**：N3  
**日期**：2026-10-05  
**状态**：完成

## 1. 冻结协议

- Python：`/home/rocos/miniconda3/envs/mujo/bin/python`
- 正式场景来源：`outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json`
- N2 只读输入：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json`
- 决策变量：`[r_L,r_R,beta]`
- 正式步长：`dt=0.005`；诊断步长：`0.0025, 0.005, 0.01`
- 每个诊断步长保持总任务时间约 `0.5 s`
- `beta_stop=0.125`，`d_safe=0.005 m`
- QP 权重、closure tolerance、swept checker、v3 矩阵均未修改

N3 只增加逐步 diagnostic trace logging，动作选择逻辑保持不变。v3、N1、N2 原始 JSON 未覆盖。

## 2. 诊断定义

对每个 filtered 候选动作记录

\[
d_{\mathrm{linear}}=d_0+\Delta t\left(\nabla_qd^T\dot q+
\nabla_xd^T(\beta\dot x_{\mathrm{nom}})\right),
\]

并以独立端点与 swept checker 定义

\[
d_{\mathrm{actual}}=\min(d_{\mathrm{endpoint}},d_{\mathrm{swept}}),
\qquad \Delta d=d_{\mathrm{actual}}-d_{\mathrm{linear}}.
\]

`linearization_optimism` 定义为 `d_linear >= d_safe` 且 `d_actual < d_safe`；`conservative_rejection` 定义为候选被拒绝且 `d_actual >= d_safe`。其余带完整字段的记录归入 `regular_finite_step_sample`。这三个分类只描述 clearance/finite-step 行为，不描述 IK branch。

## 3. 数据完整性

- 27 个 cell：3 个 `dt` × 3 个场景 × 3 个速度设置。
- 每个方法共 26,600 个逐步记录；两种方法合计 53,200 条。
- `instrumentation_unknown=0`。
- 每条记录都有 `d0`、`d_linear`、`d_endpoint`、`d_swept`、`d_actual`、`Delta d`、`beta`、任务进度、`sigma_min(A_q)`、closure residual、joint-limit margin、QP 可行性以及 endpoint/swept 结果。

## 4. 主要结果

| 方法 | `dt` | 记录数 | `median |Delta d|` | `Delta d` 最小值 | linearization optimism | conservative rejection |
|---|---:|---:|---:|---:|---:|---:|
| lift_filtered | 0.0025 | 15,250 | 0.00374 µm | −0.181 mm | 45 | 1 |
| lift_filtered | 0.005 | 7,611 | 0.0160 µm | −0.378 mm | 46 | 1 |
| lift_filtered | 0.01 | 3,739 | 0.0676 µm | −0.768 mm | 49 | 1 |

`full_joint_qp` 的三行统计与上表相同。每种方法总计 140/26,600（0.526%）linearization optimism，3/26,600（0.0113%）conservative rejection。

边界敏感性符合预期：在 `dt=0.005` 时，endpoint failure 为 45/7,611，swept failure 为 46/7,611；在 `dt=0.01` 时分别为 47/3,739 和 49/3,739。失败主要集中在预测余量 `[0,0.05) mm` 的边界箱，余量至少 `0.05 mm` 的箱没有 clearance optimism。少量余量为负的记录单独保留在 `<0 mm` 诊断箱，不被误报为过滤器成功。

三条步长的 `median |Delta d|` 为

\[
3.74\times10^{-9},\;1.60\times10^{-8},\;6.76\times10^{-8}\;\mathrm{m},
\]

描述性 log--log slope 为 `2.087`。这支持当前实现范围内的近似二阶有限步缩放观察，但不是对任意曲率、任意步长或接触转换的定理。`Delta d` 的负尾部随步长变大，说明不能只用中位误差替代独立非线性验证。

3 个 conservative rejection（每个 `dt` 一个，两个 QP 完全相同）都来自 `post_integrator_check` 的极小 joint-limit margin 负值，实际 endpoint/swept clearance 约 `8.35 mm`、预测余量约 `3.36 mm`，不是 clearance 过滤器过于保守。

## 5. reduced/full QP 对照

- 每个 `dt` 的逐步记录数相同：15,250、7,611、3,739。
- 全部记录的 `median Delta d` 差异为 0；`dt=0.01` 的 P95 绝对误差差异约 `1.05e-14 m`，其余为 0。
- 三个全局分类计数完全一致：每种方法 140 optimism、3 conservative rejection，其余为 regular finite-step sample。
- 在 `<0 mm` 这个数值边界箱内，个别样本落箱数略有差异；这些差异来自 `d_linear-d_safe` 接近机器精度的分箱边界，不形成未解释的安全分类差异。

因此，N3 支持当前实现下 reduced action-space QP 与 full joint-space QP 在 clearance 预测误差和拒绝分类上的数值一致性；不支持“reduced QP 当前更快”的结论。

## 6. 对论文理论的影响

1. 保留 differential lift、三维线性 QP、条件性 governor 和 full/reduced QP 等价命题。
2. 将安全保证明确写成两层协议：线性 QP 只生成候选，独立的非线性 endpoint/swept checker 决定接受；边界附近的 linearization optimism 必须触发拒绝或停止。
3. N3 的约 `2.09` 斜率只作为有限步敏感性结果，不能升级为普适的二阶误差界，也不能由此推出连续时间不变性。
4. 当前数据不要求修改核心 lift 公式，也不要求立即加入未经校准的 curvature margin。后续若需要提高边界通过率，应把 curvature margin 作为单独、冻结阈值之外的改进实验，并保持独立 checker。
5. conservative rejection 极少且由 joint-limit post-check 主导，因此没有证据表明当前 clearance filter 普遍过于保守。
6. N3 不提供 branch jump 或全局 IK branch preservation 证据。

## 7. 输出文件

- 原始逐步 trace：`outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_step_trace_raw.json`
- 汇总 JSON：`outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_dt_sensitivity_summary.json`
- 每步 CSV：`outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_clearance_error_steps.csv`
- 分箱统计：`outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_clearance_error_statistics.csv`
- 分类统计：`outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_classification_summary.csv` 和 `.json`
- reduced/full 总体对照：`n3_reduced_full_comparison.csv`
- reduced/full 分箱对照：`n3_reduced_full_comparison_by_margin_bin.csv`
- 图：`n3_predicted_actual_delta_figures.png`
- 运行清单：`outputs/differential_lift_feasibility_20261005/n3_dt_sensitivity/n3_dt_sensitivity_manifest.json`

## 8. 下一节点建议

进入 N4 前不再重跑 N3。下一节点应在不改变 N3 结论的前提下，选择一个明确问题：

- 若继续做安全性能：冻结 N3 的 trace 和分类，设计独立 curvature-margin 消融，并报告它对 optimism、保守拒绝和任务进度的影响；或
- 若转向论文闭环：用 N1 的负结果、N2 的停止分类、N3 的有限步误差统一重写 Results/Discussion，明确论文主张是“安全监管的局部在线延拓”，而不是全局 branch preservation。


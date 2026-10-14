# N2 endpoint-validation 停止原因与安全过滤保守性分析

## 结论

N2 通过了日志完整性和 reduced/full 对照门槛，但没有证明存在真实的局部不可行集合。v3 中发生停止的 92 个行级案例（reduced 与 full-QP 各 46 个）均有完整停止 trace；两种 QP 的停止分类逐案例一致。

| 方法 | nonlinear endpoint mismatch | swept-clearance failure | joint-limit failure | true local infeasibility | instrumentation unknown |
|---|---:|---:|---:|---:|---:|
| `lift_filtered` | 44 | 1 | 1 | 0 | 0 |
| `full_joint_qp` | 44 | 1 | 1 | 0 | 0 |

分类比例以该方法的 46 个停止为分母。`nonlinear_endpoint_mismatch` 占 44/46=95.65%；另外两类各占 1/46=2.17%。

## 冻结条件与数据来源

- Python：`/home/rocos/miniconda3/envs/mujo/bin/python`
- manifold：`hil-serl/data/aviator/manifold_phi_stale_pitch-10deg`
- v3 矩阵：`outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json`
- `dt=0.005`，`beta_stop=0.125`，`d_safe=0.005 m`
- N1 闭合残差阈值：`2e-4`
- 未修改冻结阈值；只重放原矩阵中 reduced/full 发生停止的试验索引。

## 逐停止证据

每个停止点记录了：停止前任务进度、当前和历史 beta、QP 可行性、线性预测 clearance、独立 endpoint clearance、swept clearance、关节限位余量、闭合残差和 `sigma_min(A_q)`。全部停止行的必需字段均存在，`instrumentation_unknown=0`。

`nonlinear_endpoint_mismatch` 的典型统计为：

- 线性预测 clearance 中位数：`0.005000 m`；
- 独立 endpoint clearance 中位数：约 `0.004898 m`；
- swept clearance 中位数：约 `0.004898 m`；
- 停止时 beta 中位数：`1.0`；
- 停止前任务进度中位数：约 `0.715`；
- 关节限位余量中位数：约 `0.070 rad`；
- 闭合残差中位数：约 `1.47e-5`，没有超过 `2e-4`；
- `sigma_min(A_q)` 保持正值，未出现秩退化证据。

因此这些停止不是 QP 无解，而是线性 clearance 半空间刚好贴在安全阈值上，独立非线性端点评估发现真实 clearance 已经低于阈值。唯一的 swept-only 案例端点仍为 `0.005000004 m`，但扫掠最小值为 `0.004999998 m`；唯一的 joint-limit 案例越界约 `6.6e-8 rad`，属于数值边界附近的硬停止。

## 对“过滤器是否过于保守”的判断

本轮数据不支持“过滤器过于保守”。相反，主要失败模式是**线性化在 clearance 边界处略偏乐观**，随后由独立非线性检查拒绝候选。过滤器的安全价值仍然成立：已执行并接受的 filtered 轨迹没有碰撞或关节限位样本；但当前线性 QP 单独不足以预测有限步非线性 clearance。

这一区分不能解释为“已经证明真实局部不可行”。本轮 `true_local_infeasibility=0` 只表示没有观测到 QP 无解的停止；它不证明整个任务段全局可行。

## 对论文理论的修改建议

保留以下数学结论：

1. 正则局部图内的微分 lift 分解 `dot q = h_e + beta h_x + N r`；
2. 线性瞬时约束在 `y=[r_L,r_R,beta]` 中形成凸 QP；
3. full/reduced QP 在冻结状态上的等价性；
4. governor 的条件性降速和安全停止逻辑。

将安全命题写成两层：

- **线性层**：在当前线性化模型中，QP 可行性给出候选动作；
- **离散非线性层**：候选必须通过独立 endpoint/swept checker；失败时只能声明“线性候选被非线性验证拒绝”，并触发停止或后续 recovery。

不要把 `endpoint_validation` 写成 branch-preservation 证据，也不要把本轮停止写成真实局部不可行的证明。Results 中应报告 44/46 的 nonlinear mismatch，以及 reduced/full 分类完全一致这一事实。

## 输出文件

- trace 原始补充：`outputs/differential_lift_feasibility_20261005/n2_endpoint_trace_v2/`
- 重放清单：`outputs/differential_lift_feasibility_20261005/n2_endpoint_trace_v2/n2_trace_manifest.json`
- N2 原始合并 JSON：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_endpoint_stop_raw.json`
- 分类汇总：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json`
- predicted-vs-actual CSV：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_predicted_vs_actual_clearance.csv`
- predicted-vs-actual 图：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_predicted_vs_actual_clearance.png`
- 统计表：`outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_statistics.csv`

## 下一节点建议：N3

冻结 N2 结果后，不调节本轮阈值。N3 应验证有限步 clearance 余量的可预测性：在相同任务和模型上记录线性预测与非线性误差，检查随 `dt`、曲率和初始 clearance 的变化，并比较“线性 QP + 独立拒绝”与带显式余量的候选策略。N3 的目标是量化保守性/漏检边界，而不是制造 branch jump。

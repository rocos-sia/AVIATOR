# N6：Online-vs-LUT 适应性与成本资格实验协议

**节点**：N6-A（冻结 LUT 与在线延拓资格实验）  
**日期**：2026-10-06  
**状态**：协议冻结，N6-A/N6-B 已完成；结论为条件性/负结果

## 目标

验证当前在线局部延拓相对于成熟 frozen LUT 管线的独特价值。N6 不再寻找 branch jump，也不修改 N1–N4 的冻结结果。

## 方法组

- `frozen_lut_safe_scaled`：使用同一初始状态的 `(x_0, phi_0)`，查询已有 `Q(x,phi)`；每步沿原始相位动作方向做一维 safe-interval scaling，复现 LUT 管线的相位动作缩放，不使用任务 governor、Jacobian 或 QP。
- 旧的 `n6_smoke2` 和 `n6_online_lut_qualification_v1` 使用固定相位速度，已标记为协议诊断，不得作为最终证据。
- `online_lift_filtered`：复用 v3 的三维 QP、governor、midpoint 积分和独立 nonlinear endpoint/swept checker。
- `full_joint_qp`：作为可选表示对照，使用与 reduced QP 相同的约束和 checker。

N6-A 暂不宣称 rebuilt LUT 已完成；重建成本和新表结果列为 N6-B 的后续节点。

## 场景与速度

复用 v3 的 3 个场景（nominal、tight、shifted）和 3 个任务速度（gentle、nominal、stress），每个 cell 默认 10 个 paired trials。每一对方法共享：

- 初始 `(x_0, q_0, phi_0)`；
- task velocity；
- phase-rate command；
- dt=0.005、100 steps；
- MuJoCo model 和独立 clearance checker；
- joint limits、closure tolerance 和安全阈值。

## Frozen LUT 定义

每一步令

\[
 x_{k+1}=x_k+dt\,\dot x_{nom},\qquad
 \phi_{k+1}=\phi_k+dt\,r_{nom},
\]

并查询 `Q(x_{k+1},phi_{k+1})`。LUT 不因场景扰动重建；每一步先沿相位动作方向缩放到插值 safe interval 内，再查询 Q。任务/相位网格越界、非线性 endpoint/swept checker 失败或关节越限均记录独立原因。LUT 结果不得使用线性 QP 或任务 governor 补救。

## 主要指标

- 完成率、任务进度、停止步和停止原因；
- endpoint/swept 最小 clearance、碰撞样本、关节限位样本；
- closure residual；
- LUT domain/safe-interval exit；
- online governor 的 beta、停止分类；
- 每步延迟和 p95/p99；
- LUT 查询时间、在线 Jacobian/QP/checker 时间（可获得时分开记录）。

## 资格判据

N6-A 不要求在线方法全面优于 LUT。通过需要：

1. 标称场景中在线方法满足已冻结的安全和进度边界；
2. 至少一个扰动场景中，frozen LUT 出现可重复的 domain/safe/clearance 退化，而在线方法在相同 checker 和数值预算下保持可解释的安全推进或安全停止；
3. 若两者结果近似等价，记录为负结果，不增加适应性主张；
4. 所有失败原因可审计，不能把 LUT 查询失败或 solver failure 混写为碰撞。

## 边界

N6-A 只能支持“frozen LUT 在当前扰动下的部署脆弱性与在线方法的局部适应性证据”。只有 N6-B 完成 rebuilt LUT 的构建时间、存储量和恢复性能后，才能声称在线方法减少 LUT 重建成本。

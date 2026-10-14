# Branch-Preserving Differential Lift：冻结矩阵与论文主旨判据

日期：2026-10-02  
代码分支：`grasp-conditioned`  
对应计划：[branch_preserving_differential_lift_experiment_plan_2026-10-01.md](./branch_preserving_differential_lift_experiment_plan_2026-10-01.md)

## 1. 本轮要回答的问题

本轮不是重新调参，而是用冻结参数回答三个问题：

1. 在任务速度和障碍场景变化时，解析 differential lift 是否仍能保持局部 IK branch 的连续性？
2. 三维 `y=[r_L,r_R,beta]` safety QP 是否在独立 swept checker 下保持安全，并且给出真实的任务进度？
3. 相同初值、任务命令和计算预算下，安全性和 branch 连续性的变化究竟来自 filter/governor，还是只来自 (H/N) 的代数表示？

## 2. 冻结协议

### 方法

- `lift_filtered`：解析 (H/N) + 三维 `y=[r_L,r_R,beta]` phase/clearance QP；
- `lift_unfiltered`：相同解析 (H/N)，不做 filter；
- `dls`：相同任务与 null-rate 的 DLS differential IK；
- `pointwise_projection`：上一时刻关节角 warm-start 的逐点投影。

所有方法使用同一初态、同一任务速度、同一 nominal (r)、同一模型、同一时间步长和独立 swept collision checker。正式矩阵冻结后不为某个方法单独调参。

### 速度层

| 层级 | (dot\theta) | (dot s) | 作用 |
|---|---:|---:|---|
| gentle | 0.03 | 0.0005 | 检查正常可行段 |
| nominal | 0.08 | 0.0020 | 复现当前 pilot |
| stress | 0.12 | 0.0030 | 检查边界与 governor |

### 障碍层

以同一 MuJoCo 模型为基础生成三个只改变墙体场景几何的模型副本：

- `nominal`：原始双墙；
- `tight`：两墙向中间各移动 1 mm。当前冻结流形的名义净空约为 9 mm；更大的内移会使 (d\ge 7) mm 的合法初态不存在，因此不把不可初始化的场景混入主矩阵；
- `shifted`：双墙整体沿 (x) 方向平移 15 mm。

机器人、抓取、流形查表和任务坐标定义保持不变。每个场景重新用独立 checker 筛选初态，初始净空要求为 (d\ge 7) mm。

### 样本量与配对

- 3 个速度 × 3 个场景 × 5 个 paired trials = **45 个 trial**；
- 每个 trial 100 步，(Delta t=0.005) s；
- 每个速度/场景组合使用固定但不同的 seed；
- 四种方法重放同一组初态，因此统计以 trial 为实验单位，而不是把时间步当作独立样本。

## 3. 必须记录的指标

### 安全性

- 完成率与安全停止率；
- 端点和 swept 最小净空；
- 碰撞样本、关节限位样本；
- `stop_reason`：`complete`、`no_feasible_candidate`、`projection_residual`、`collision_detected`、`joint_limit`；
- governor 干预步数和最小速度缩放。

### Branch 连续性

- 独立 swivel descriptor 的最大单步变化；
- descriptor jump 次数（阈值 0.5 rad）；
- 已完成轨迹的 descriptor jump rate；
- 后续正式阶段必须再加入由多初值 IK/atlas 验证的 branch label。仅有局部 descriptor 连续，不能称为全局 branch preservation。

### 任务进度

- 最终 ((\Delta\theta,\Delta s))；
- 相对于计划任务位移的每个分量进度；
- 最小分量任务进度率；
- 安全停止前的完成步数。

### 计算代价

- 每步 wall-clock latency 的 P50/P95/P99；
- 总运行时间；
- Jacobian、checker、projection/rejection 调用次数；
- 仅比较同一硬件和同一 Python 环境中的相对值。

## 4. 预先声明的论文主旨判据

### 可以支持论文主旨的最低证据

需要同时满足：

1. 已有解析 (A_q/A_x) 与闭链残差实验通过；
2. 在至少 45 个矩阵 trial 中，filtered 方法在所有已完成或安全停止的轨迹上没有碰撞/限位样本；
3. 在 nominal 场景中，filtered 方法有可重复的真实任务进度，而不是通过零速保持获得完成；
4. branch descriptor 在完成轨迹上无异常跳变，并且至少一个独立 atlas/多初值 branch label 与 descriptor 一致；
5. 相同 (H/N) 的未过滤版本不能同时达到同等安全性，说明增益来自可验证的 filter/governor 机制；
6. 结果在至少两个场景或两个速度层重复出现。

### 只能支持较弱结论的情况

- filtered 安全但大多数 trial 立即停止：只能声称“安全拒绝/局部可行性过滤”，不能声称可靠长轨迹执行；
- descriptor 无跳变，但所有基线也无跳变且没有独立 branch label：只能声称局部连续积分，不能声称 branch preservation；
- filtered 仅在 nominal 场景有效：只能声称单场景原型，不足以支持适应性主张；
- 只改善碰撞率但没有任务进度或延迟统计：不能说明安全性是否以任务停滞为代价。

## 5. 本轮结束后的决策节点

### Gate A：验证协议

脚本可复现、45 个 trial 全部有完整记录、无初态筛选不足、测试和 JSON 校验通过。

### Gate B：安全过滤

filtered 的碰撞和限位样本为零；若不是，先修正 checker、步长或 filter，再讨论论文结论。

### Gate C：论文主张

只有在安全、进度和独立 branch label 三者同时成立时，才把题目写成 branch-preserving differential lift。否则收紧为“constraint-aware differential lift with safe phase governor”，并把 branch preservation 作为局部证据或后续工作。

## 6. 后续实验

若本轮通过 Gate A/B 但未通过 Gate C，下一轮优先做：

1. 多初值 IK atlas 的 branch label 与在线 descriptor 对齐；
2. 设计会诱发普通逐点 IK 分支切换的跨奇异/窄通道段；
3. 对 filtered DLS/QP 增加同等安全约束，排除“只比较了未过滤基线”的质疑；
4. 将 45 个 pilot 扩展到每组合 20 个 trial，并报告 paired bootstrap 或 Wilson 区间。

## 7. 重定位后的论文主线与正式矩阵

现有 atlas 和 branch-stress 结果没有找到“Pointwise/DLS 合法跳变而 H+Nr 不跳”
的稳定任务段。因此论文主线冻结为：

> 提出一种不依赖全局查表的局部在线延拓方法，通过 branch-preserving differential
> lift、解析 Jacobian、闭合误差控制和 phase-space safety governor，在约束任务执行
> 中维持局部连续性，并在接近不可行区域时主动降速或安全停止。

`branch jump` 不再是论文必须出现的现象，而是条件性指标：只有独立 finite atlas、
descriptor margin 和闭合残差同时支持时才计数。已有 45 个 paired pilot 结果保留，
正式矩阵按以下指标重新统计：

1. 完整任务成功率和最小任务进度；
2. 最大/均值闭合误差；
3. 碰撞、关节限位和安全停止次数；
4. `stop_reason` 分类；
5. governor 降速次数、持续时间和最小速度缩放；
6. P50/P95/P99 计算延迟及 Jacobian、checker、projection 调用次数；
7. descriptor 漂移、atlas 覆盖率和条件性 branch label 结果。

正式比较至少包含：

- 完整方法：解析 Jacobian + H+Nr + safety filter + governor；
- Pointwise IK；
- DLS 或普通局部伪逆延拓；
- 去掉 safety filter 的消融；
- 去掉 governor 的消融；
- 去掉 Nr 项或改用数值 Jacobian 的消融。

每个 paired trial 固定初始状态、任务轨迹、场景、随机种子和数值预算。若完整方法
只提高安全停止率而任务进度接近零，结论收紧为“安全拒绝机制”；若同时保持真实任务
进度、较低闭合误差和可接受延迟，才支持“安全监管的局部在线延拓框架”。

## 8. 理论冻结后的 Regular-Component Validation Gate

本节是进入下一轮正式 30--50 paired trials 的前置门槛。通过前，不能把已有 45-trial pilot 写成新理论下的最终验证。

### 8.1 已冻结的数学对象

- 核心决策变量固定为 `y=[r_L, r_R, beta]`，其中 `beta in [0,1]` 是任务速度缩放；
- 核心速度分解固定为 `dot q = h_e + beta h_x + N r`、`dot x = beta dot x_nom`；
- 误差证据使用 ISS 残差 `delta = A_q dot q + A_x dot x + K_e e`，不再把理想指数衰减当作有饱和时的保证；
- branch 结论只写为 regular connected component 内的 local continuity lemma；
- 中点安全由起点预测、半步重新求解和独立非线性 endpoint/collocation validator 共同判定。

### 8.2 Node 0：确定性验证项目

在增加 trial 数量以前，对固定状态集逐项运行并保存 JSON。新增的抽象代数探针为 `hil-serl/tools/validate_reduced_action_contract.py`，它只验证线性代数契约，不替代 MuJoCo 模型验证：

1. **降维代数契约**：运行 `validate_reduced_action_contract.py`，检查 full/reduced 速度重构、线性约束 pullback 和三类 governor 语义；
2. **解析 Jacobian**：`A_q`、`A_x` 与独立中心有限差分的中位数、P95 和最大误差；
3. **分解残差**：随机可行 `y` 上验证 `||A_q(h_e+beta h_x+N r)+A_x beta dot x_nom+K_e e||`；
4. **全空间/降维 QP 等价**：同一线性约束下，比较 full joint-space QP 与 reduced `y`-QP 的可行性、目标值和恢复出的 `dot q`；
5. **积分阶次**：Euler 与 midpoint 的误差随步长收敛斜率；
6. **governor 行为**：构造 beta=1 可行、仅 beta<1 可行、beta=0 也不可行三类状态，验证分别为满速、降速和安全停止；
7. **独立 endpoint/collocation 检查**：过滤器自身不得作为唯一安全判定器。

建议冻结门槛：解析 Jacobian 相对误差、分解残差和 full/reduced QP 恢复误差均按代码配置的数值容差报告；任何失败都必须分类为实现错误、正则图外状态或模型不可行，不能合并成“solver failure”。

### 8.3 通过门后的正式矩阵

Node 0 通过后，重新运行每个场景类至少 30--50 个 paired trials。正式方法必须使用统一的三维 `y`-QP；旧 pilot 的“二维 phase filter + 外部 governor”只作为兼容性基线。正式矩阵至少包含：

- 完整方法：解析 Jacobian + `h_e+beta h_x+N r` + 三维安全 QP + midpoint validator；
- Pointwise IK；
- DLS/QP differential IK；
- full joint-space QP + 同一 governor；
- 去掉 safety filter、去掉 governor、去掉 `N r`、数值 Jacobian 四个消融。

### 8.4 正式判据

只有同时满足以下条件，才保留“安全监管的局部在线延拓框架”这一主张：

- full/reduced QP 等价在冻结测试集通过；
- 完整方法在已完成或安全停止的轨迹上无碰撞和关节限位样本；
- 安全停止前仍有可量化任务进度，且不是长期 `beta=0`；
- 闭合误差满足 ISS 预测的残差上界趋势；
- 至少一个独立 atlas/多初值 descriptor 验证 local component label；
- 对比 full joint-space QP 后，优势可归因于统一的低维表示或实现成本，而不是不公平的安全约束。

若只通过安全停止而没有真实进度，论文主张收紧为“局部可行性过滤与安全拒绝机制”。若 branch label 仍不能独立验证，则正文只保留 local continuity lemma，不写 global branch preservation。

## 9. 实现冻结记录（2026-10-04）

Node 0 已通过后，正式矩阵固定使用：

- `project_reduced_action` 的三变量 QP；
- `connector_step_y` 的 `h_e+beta h_x+N r`；
- `full_joint_qp_step` 作为相同线性约束下的 15 变量强表示基线；
- MuJoCo 3.3.6 环境 `/home/rocos/miniconda3/envs/mujo/bin/python`；
- `dt=0.005` s、100 steps/trial、`beta_stop=0.125`；
- 3 个速度层、3 个墙体场景、每速度/场景 10 个 trial。

三维矩阵报告输出为：

`outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json`

若 full joint-space QP 在独立 endpoint validator 处停止，记录为 `full_qp_endpoint_validation`，不能将其解释为全空间表示不可行；这表示当前局部线性模型与有限步非线性检查之间存在差距，需要在结果中单独报告。

## 10. v3 冻结矩阵判定（2026-10-05）

v3 矩阵已完成 90 个 paired trials。`lift_filtered` 和 `full_joint_qp` 均为 44/90 完整
完成，碰撞与关节越界样本均为 0；未过滤 lift/DLS 分别产生 2679 个碰撞样本和 248 个
关节限位样本，pointwise projection 产生 2610 个碰撞样本。两种 QP 的最小 swept
clearance、任务进度、闭合误差和停止分布一致，说明 reduced/full 比较没有因中点积分
使用不同 null-space action 而失配。

因此当前证据可以冻结为：

- **已验证**：解析 Jacobian 契约、分解残差、full/reduced QP 的局部行为等价、三类
  governor 状态、二阶 midpoint 收敛，以及安全过滤器的独立 endpoint/swept 检查；
- **已支持**：安全监管的局部在线延拓能够把部分局部不可行状态变成可解释停止，并在
  测试矩阵中消除碰撞和关节越界样本；
- **仍未证明**：全局 IK branch preservation、Pointwise/DLS 必然 branch jump、所有
  场景下的高任务完成率，以及相对于 full joint-space QP 的速度优势。

后续若继续扩展实验，应优先分析 `endpoint_validation` 的非线性原因和安全停止前的
任务进度，而不是把当前 44/90 提升到更大样本后再扩大理论主张。

## 11. N1 分支区分门槛结果（2026-10-05）

N1 使用独立 128-start atlas 和 `(theta,s,phi)` continuation，新增真实三维
`lift_filtered` 对照。严格 jump 规则固定为：component label 改变、两端 label margin
不少于 `0.05`、closure residual 不超过 `2e-4`、独立 swivel descriptor 变化不少于
`0.04 rad`，并且 filtered lift 必须走到基线跳变所在步数。

24 个 stress cases 中没有 candidate。20 个完成了三方法比较，4 个因静态 component
不足而退出，程序失败为 0。右臂局部 atlas 保留 2 个 component，但双臂同时稳定多个
component 的门槛仍未通过。因而 N1 不支持 Pointwise/DLS branch jump 与 lift_filtered
不跳的比较性结论，也不改变论文的局部安全延拓定位。

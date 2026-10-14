# GFR-0/GFR-1 修正版实施方案（2026-10-11）

## 1. 方案定位

本方案在固定的双臂闭链流形和现有安全执行器上加入 RL，而不是让 RL 重建或修改流形。

当前核心命题为：

> RL 在固定的低维闭链可行表示上学习冗余 phase-rate 调度；确定性闭合反馈维持局部闭链一致性；governor 决定任务推进速率并执行降速、恢复和停止；独立 nonlinear endpoint/swept checker 决定候选动作是否接受。

当前版本暂不声称：

- RL 自动获得 ISS 鲁棒性；
- 全局 IK branch preservation；
- 线性动作可行性提供整段连续时间安全保证；
- RL 必然提高任务成功率或总控制周期；
- reduced QP 必然比 full QP 更快。

## 2. 冻结的几何和动作接口

闭链微分提升保持为：

```text
qdot = h_e + beta_g h_x + N r
```

其中：

- `h_e` 是确定性的闭合误差反馈；
- `h_x` 是任务推进方向；
- `N r` 是零空间冗余运动；
- `r = [r_L, r_R]` 是 GFR-1 中 RL 唯一直接控制的量；
- `beta_g` 由冻结的 governor 规则决定，不由第一版 RL 直接输出。

三维线性安全集写成：

```text
Y_lin(z) = {(r, beta): A_r(z) r + a_beta(z) beta <= b(z)}.
```

给定 governor 当前候选速率 `beta_g` 后，RL 使用二维截面：

```text
R_lin(z; beta_g) = {r: A_r(z) r <= b(z) - a_beta(z) beta_g}.
```

因此，第一版动作语义严格为：

```text
governor 选择 beta_g；RL 在对应二维截面中选择 r。
```

若截面为空、退化或顶点排序不稳定，进入既有恢复逻辑；不得强行输出二维动作。

## 3. 观测接口

GFR-1 的 Actor 观测至少包含：

- 当前关节状态和任务状态；
- 任务进度及未来局部任务描述；
- 归一化闭环误差 `e_t`；
- clearance、关节限位裕量和 `sigma_min(A_q)`；
- 上一周期名义动作和实际执行动作；
- 上一周期 governor 降速、动作缩短、endpoint/swept 拒绝和恢复标志。

必须在日志中区分：

```text
raw policy output -> structured nominal action -> executed action
```

安装误差只有在观测来源明确时才能作为泛化证据。不能把仿真隐藏状态直接输入 Actor 后声称实现了本体感知抗扰。

## 4. GFR-0：接口和数学契约验收

GFR-0 不进行大规模训练，先完成静态和单步验证。

### 4.1 闭链微分契约

在正则状态上检查：

```text
||A_q N||
||A_q h_e + K_e e||
||A_q h_x + A_x xdot_nom||
||A_q(qdot) + A_x(xdot) + K_e e||
```

同时记录数值残差、`sigma_min(A_q)` 和退化状态。

### 4.2 二维截面动作契约

对随机状态和边界状态检查：

```text
max_i (A_r r - (b - a_beta beta_g))_i
```

要求所有被标记为结构化可行的动作满足冻结数值容差。

必须覆盖：

- 正常二维多边形；
- 线段或单点退化截面；
- 空截面；
- 顶点顺序变化；
- governor 改变 `beta_g` 后重新构造截面；
- 所有候选失败时的恢复路径。

### 4.3 误差信息契约

分别测试：

1. 数值闭合残差；
2. 执行跟踪误差；
3. 可观测任务位姿偏差。

报告每种误差是否真正改变 Actor 观测，以及是否与隐藏仿真状态混淆。

### 4.4 动作日志契约

每个 transition 必须保存：

- `policy_action_raw`；
- `action_nominal_r`；
- `beta_g`；
- `action_executed_r`；
- `beta_executed`；
- `tau_requested`（GFR-1 固定）；
- `tau_executed`；
- endpoint/swept checker 结果；
- stop/recovery reason。

## 5. GFR-1：固定持续时间的学习增量验证

GFR-1 暂不学习动作持续时间，也不改变 `dt`、安全阈值、governor 或 checker。

### 5.1 消融组

- `B0`: 固定 phase 或现有确定性名义策略；
- `B1`: 原始 RL，相同几何执行后端；
- `B2`: `B1 +` 闭环误差和安全裕量观测；
- `B3`: `B2 +` 二维截面结构化动作映射；
- `B4`: `B3 +` governor 执行修正辅助监督。

正式报告另保留两个删项：

- `B4 - error observation`；
- `B4 - correction supervision`。

不同时修改宏动作、curvature margin、积分器或安全阈值。

### 5.2 Governor 监督

第一版辅助损失只作用于 RL 实际控制的 `r`：

```text
l_corr = w_valid ||D_r^{-1}(r_policy - stopgrad(r_executed))||^2
```

其中 `w_valid` 只对有有效接受动作的样本启用。所有候选都失败时不能把零动作自动当作监督标签。

该项作为 RL loss 的辅助项：

```text
L_actor = L_RL + lambda_corr l_corr
```

必须保留 `lambda_corr=0` 对照。若 governor 只改变 `beta_g` 而未修改 `r`，相位修正损失可以为零；此时只通过任务回报、恢复成本和下一状态学习。

### 5.3 奖励和完成定义

GFR-1 使用：

```text
R = w_progress * delta_progress
    - w_intervention * intervention_cost
    - w_recovery * recovery_cost
    - w_compute * compute_cost
    + terminal_reward
```

任务完成必须同时满足目标进度、任务误差和冻结时间预算，不能以接受步数或长期零速作为完成。

拒绝候选仍计入计算成本，并设置最大连续拒绝和无进度预算。

### 5.4 主要指标

- completion rate；
- task progress 和 progress shortfall；
- clearance、endpoint/swept rejection；
- governor 降速和恢复次数；
- closure residual；
- joint-limit 和 speed failure；
- 名义动作到执行动作的修正量；
- 每单位任务进度的策略、QP、distance、swept 调用次数；
- 总控制周期 P50/P95/P99；
- 多训练 seed 的方向一致性。

GFR-1 的通过条件是：在相同执行后端和安全标准下，任务性能不下降，并且拒绝、恢复或单位进度干预成本出现可复现改善。仅 reward 上升不算通过。

## 6. GFR-2（后续，不在本轮实现）

GFR-2 才加入：

```text
(r_t, tau_req_t) = pi(o_t)
```

验证器决定实际执行：

```text
(r_exec_t, tau_exec_t) = V(z_t, r_t, tau_req_t)
```

必须区分：

- 任务速率 `beta`；
- 决策持续时间 `tau`；
- 数值积分步长 `h`；
- 前瞻范围 `H`。

宏动作允许提前终止。训练回报和 bootstrap 使用实际 `tau_exec`，而非请求持续时间。GFR-2 必须分别报告策略调用、QP、距离查询和 swept checker 调用次数，不能把减少 Actor 调用误报为几何计算加速。

## 7. 当前证据边界

N14 的多 seed screening 未支持稳定的随机 phase 泛化收益；N15 未证明完整 endpoint-aware 系统的总延迟优势；N16/N17 支持在线闭链 governor 在测试扰动范围内的条件性执行优势，但不支持全局安全或 reduced-QP 速度优势。

因此 GFR-1 的新增结论必须独立产生，不得覆盖或改写 N14/N15/N16/N17 原始 JSON。

## 8. 输出和不可变性

GFR-0/GFR-1 计划输出到新的版本化目录，例如：

```text
outputs/differential_lift_feasibility_20261011/gfr_gfr01/
```

建议输出：

- `gfr0_contract_report.json`；
- `gfr0_state_probe.csv`；
- `gfr1_transition_raw.jsonl`；
- `gfr1_ablation_summary.json`；
- `gfr1_execution_summary.csv`；
- `docs/research/gfr_gfr01_results_2026-10-11.md`；
- 更新后的 `long_horizon_status.md`。

禁止覆盖 N1--N17、N14/N15 原始结果、历史 checkpoint 和正式安全阈值。

## 9. 冻结后的论文表述

> We use reinforcement learning to schedule redundant phase rates on a fixed local closed-chain representation. Deterministic differential feedback maintains the local closure relation, while a governor selects task-rate scaling and an independent nonlinear checker accepts, shortens, recovers, or rejects candidate actions. The GFR-1 study tests whether error-conditioned observations, structured feasible actions, and execution-feedback supervision reduce avoidable intervention under the same safety backend.

该表述不把误差观测写成 ISS 证明，不把线性可行性写成连续时间安全保证，也不把 RL 结果写成全局 branch preservation。

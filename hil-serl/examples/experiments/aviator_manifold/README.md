# AVIATOR 冗余相位策略的强化学习训练说明（RLPD / SAC）

> 本文档解释 `hil-serl/examples/experiments/aviator_manifold/` 这套 **proprio-only（纯本体感知）**
> 强化学习实验：在 `tools/README.md` 建好的相位安全流形 `Q(x, φ)` 之上，学一个把 40 维状态映射到
> 双臂相位速度 `(φ̇_L, φ̇_R)` 的冗余自由度策略。教师是 Route-A LUT-native DP，部署侧是 T6 + ONNX actor。
>
> 流形本身怎么建见 `AviatorRobot/tools/README.md`；在线消费/部署见 `methods/t6/README.md`。

---

## 0. 一句话概括

任务变量 `x=(θ,s)`（方向盘转角/推拉）沿**固定参考轨迹**推进，策略只学**冗余自由度**：
每臂的肘部自运动相位 `φ` 以多快走。每步用流形查表把 `(x, φ)` 变成 14 维关节角
`q = Q(x, φ)`（运动学近似，不做在线 IK），碰撞、关节限位、速度超限都是**终止事件**。
奖励只奖励"待在安全相位区间中心"和"留出间隙余量"，从不度量到 DP 轨迹的距离。

```
策略 π : obs(40) → a ∈ [−1,1]²       (归一化相位速度)
执行   : φ̇_exec = a · φ̇_scale,   φ_{t+1} = φ_t + φ̇_exec·dt
查表   : q_{t+1} = Q(x_{t+1}, φ_{t+1})     (三线性插值)
```

---

## 1. 学什么：策略与任务分解

| 维度 | 谁控制 | 说明 |
|---|---|---|
| 任务变量 `x=(θ,s)` | **固定参考轨迹**（教师 DP 生成的 `traj`） | 策略不碰，`x_{t+1}=traj["x"][t+1]` |
| 冗余相位 `φ=(φ_L,φ_R)` | **策略** | 每臂 1 个相位速度，是唯一的学习输出 |

机械臂是 7 自由度，抓取焊缝是 6 自由度刚体约束，剩 1 冗余 → 肘部自运动环（S¹），
参数化成 φ。策略的本质是：给定当前任务状态与相位，决定往自运动环哪个方向、多快走，
**始终把构型停在安全相位区间的中心附近**（间隙 ≥ 5 mm、不触墙、不超关节/速度限）。

---

## 2. 环境 `AviatorManifoldEnv`（[env.py](env.py)）

### 2.1 观察（40 维，闭环相位编码）

观察规约（[env.py:20-25](env.py#L20-L25)，实现 [env.py:236-274](env.py#L236-L274)）：
**展开后的 φ 永远不进 MLP**，只给 `[sin φ, cos φ]`（闭环编码，避免相位 2π 跳变）。

| 切片 | 维度 | 内容 |
|---|---|---|
| `[0:2]` | 2 | 任务状态 `x = (θ, s)` |
| `[2:4]` | 2 | 任务速度 `xdot` |
| `[4:6]` | 2 | `sin(φ_L), cos(φ_L)` |
| `[6:8]` | 2 | `sin(φ_R), cos(φ_R)` |
| `[8:10]` | 2 | 距安全区间下界的余量 `m_φ⁻ = φ − φ_safe_lo` |
| `[10:12]` | 2 | 距安全区间上界的余量 `m_φ⁺ = φ_safe_hi − φ` |
| `[12]` | 1 | 最小间隙 `d_min = min(dL, dR)` |
| `[13]` | 1 | 最差关节限位余量 `m_q` |
| `[14:16]` | 2 | 上一步动作 `a_prev` |
| `[16:24]` | 8 | 历史 `x`（lag ∈ {2,5,10,20}） |
| `[24:32]` | 8 | 历史 `xdot`（同 lag） |
| `[32:40]` | 8 | 历史 `a`（同 lag） |

`m_φ⁻`、`m_φ⁺`、`d_min`、`m_q` 都由流形查表 `ManifoldLookup.query()` 直接给出，
所以策略能"看到"自己离危险边界有多近。

### 2.2 动作

```
action_space = Box(−1, 1, shape=(2,))           # (φ̇_L_norm, φ̇_R_norm)
φ̇_exec = action · φ̇_scale                        # φ̇_scale 读自 dp_demo/phi_dot_scale.json (=1.5)
```

动作是**归一化相位速度**，`φ̇_scale = 1.5 rad/s`（p95 下界，见 `reward.py` 标定）。
动作必须有限且在 `[−1,1]`，**从不 clip**（越界直接 `ValueError`）。

### 2.3 状态推进（运动学，完美跟踪）

```
x_{t+1} = traj["x"][t+1]                        # 任务变量来自固定轨迹
φ_{t+1} = φ_t + φ̇_exec · dt                     # dt = 0.01
q_{t+1} = Q(x_{t+1}, φ_{t+1})                   # 查表，非新碰撞检测
```

v0.1 是**运动学近似**：`q` 精确等于流形存储的构型，间隙就是流形里烘焙好的 `d_min`
（MuJoCo 碰撞结果），物理 `mj_step` 位置伺服是部署细节，不在学习环境里。

### 2.4 终止条件（[env.py:175-212](env.py#L175-L212)）

| 终止 | 条件 | 类型 |
|---|---|---|
| `grid_exit` | `φ_{t+1}` 超出 LUT 的 φ 网格 | terminated |
| `invalid_lookup` | `q` 或 `d_min` 非有限 | terminated |
| `branch` | 查表 `branch ≥ 2`（2=IK 不可达，3=相位不一致） | terminated |
| `clearance` | `d_min < d_safe`（5 mm） | terminated |
| `joint_limit` | `q` 越 `[lo, hi]` | terminated |
| `speed` | `max\|qdot\| > qdot_max`（1.5 rad/s） | terminated |
| `end` | `t ≥ len(traj) − 1` | truncated |

> **branch=1（开环）不算终止**：它只是手腕奇异点附近的"短但合法"的自运动弧，
> 仍然通过 clearance/joint/speed 检查（[env.py:195-200](env.py#L195-L200)）。

任何终止事件 → 该步奖励被覆盖为 `TERMINAL_PENALTY = −100`（一次性硬惩罚，[env.py:228-230](env.py#L228-L230)）。

---

## 3. 奖励函数（[reward.py](reward.py)）

### 3.1 公式

```
R_i        = 2·min(m_φ_i⁻, m_φ_i⁺) / (m_φ_i⁻ + m_φ_i⁺)        ∈ [0,1]     (单臂相位中心度)
R_reserve  = min(R_L, R_R)                                     (双臂 soft-min，非平均)

C_v        = ‖φ̇_exec / PHI_DOT_MAX‖²                           (速度代价)
C_a        = ‖(φ̇_exec − a_prev) / DPHI_DOT_MAX‖²               (加速度代价)
C_margin   = clip((d_safe + margin_width − d_min) / margin_width, 0, 1)²   (间隙余量警告)

reward     = w_R·R_reserve + ALIVE_BONUS − w_v·C_v − w_a·C_a − w_m·C_margin
```

### 3.2 系数与标定

| 常量/权重 | 值 | 含义 |
|---|---|---|
| `PHI_DOT_MAX` | 1.5 rad/s | = `φ̇_scale`，双臂满速 → `C_v = 2` |
| `DPHI_DOT_MAX` | 0.5 rad/s | 每步相位速度变化尺度（= 50 rad/s²） |
| `ALIVE_BONUS` | +0.25 | 每步存活奖励，鼓励走完有限轨迹 |
| `TERMINAL_PENALTY` | −100.0 | 终止硬惩罚 |
| `w_R / w_v / w_a / w_m` | 1.0 / 0.05 / 0.05 / 0.1 | reserve / 速度 / 加速度 / 间隙权重 |
| `w_f` | 0.0（废弃） | v0.1 无屏蔽训练，没有滤波代价 |

存活步净奖励 ≈ `ALIVE_BONUS + w_R·R_reserve ≈ +0.3`（DP 教师的 `R_reserve ≈ 0.05`：
**教师骑在安全边界上，不是区间中心**）。

### 3.3 关键设计点

- **`R_reserve` 用双臂 soft-min**：只要一只臂贴边就压奖励，逼策略把两只臂都留在中心。
- **`C_margin` 是"分级警告层"**：`d ∈ [5mm, 10mm]` 时按平方惩罚，`d < 5mm` 交给 env 作硬失败。
- **`C_a` 用上一步的"已执行速度" `prev_phi_dot`**（不是原始 action，[env.py:217](env.py#L217)），
  这是 C_a 单位 bug 的修复：加速度应作用在过滤后的真实速度上。
- **无 `w_j` 加加速度项**（已移除），奖励全部无量纲化（`∈[0,1]` 量级）。

---

## 4. 安全滤波 `project_phi_dot`（[safety_filter.py](safety_filter.py)）

> 注意：**v0.1 学习环境不接这个滤波**（env 里 action 直接执行，无安全投影）。
> 它是**在线/部署侧**的硬防护（T6），这里一并说明其数学。

策略给名义相位速度 `φ̇_nom`，滤波把它投影到单步可行集上（[safety_filter.py:5-21](safety_filter.py#L5-L21)）：

```
|Q_x(x_next)·xdot + Q_phi(x_next)·r| ≤ qdot_max          (14 个关节速度半平面)
φ + r·dt  ∈  [φ_safe_lo(x_next), φ_safe_hi(x_next)]      (每臂 box)
```

约束对 `r` **线性**（一组 2 维半平面）→ 可行集是相位速度平面上的凸多边形。
用 **Sutherland–Hodgman** 逐面裁剪，再把 `φ̇_nom` 投影到多边形上（[safety_filter.py:122-213](safety_filter.py#L122-L213)）。

投影后还有 **post-Newton 硬屏蔽**（[safety_filter.py:215-240](safety_filter.py#L215-L240)）：
因为 `Q` 是三线性的（(θ,s) 双线性 × φ 线性），线性化预测会**欠交付**，
所以对 `φ + r·dt` 处真实的 `q` 和 `d` 复查，违规则把 `r` 折半回退（最多 5 次），
退回 0 表示不可行。这是 v0.1 的有限步护栏，不是形式化 CBF。

---

## 5. 网络结构与训练配置（[config.py](config.py)）

### 5.1 网络（冻结的 v0.1 规格）

| 组件 | 结构 | 说明 |
|---|---|---|
| Actor | `MLP(256,256,256)` tanh + LayerNorm，**tanh-squashed Gaussian** | `std` 用 exp 参数化，`std∈[1e-5, 5.0]` |
| Critic | `MLP(256,256,256)` tanh + LayerNorm，**ensemble-2** | 双 Q |
| Temperature | `GeqLagrangeMultiplier`，初值 1e-2，`geq` 约束 | 自动温度 |
| Encoder | `StateEncoder`（[wrappers.py](wrappers.py)） | 直接展平 40 维 state，**无卷积、无 64 维 proprio 投影** |

BC 与 SAC 的 actor 结构**完全一致**，这样 BC 预训练的 actor checkpoint 能原样加载进 SAC actor
（BC→SAC warmstart）。

### 5.2 `TrainConfig` 关键项

| 项 | 值 | 含义 |
|---|---|---|
| `image_keys` | `[]` | 无视觉 → proprio-only |
| `proprio_keys` | `["state"]` | 40 维观察 |
| `discount` | 0.999 | 长时域（跨间隙） |
| `max_traj_length` | 1200 | 轨迹最长 12 s @ 100 Hz |
| `replay_buffer_capacity` | 1e6 | |
| `filter_zero_actions` | `True` | **丢掉 88% 的"保持"动作**（见 §6） |
| `trajectory_split` | `"rl_train"` | 400 条 rollout 轨迹 |
| `num_actor_envs` | 8 | 8 个 env 锁步 |
| `max_online_episodes` | 400 | 每条 rl_train 轨迹覆盖一次 |
| `actor_step_delay` | 0.5 | rollout 步间留 learner 更新 |
| `updates_per_online_transition` | 1 | 学习紧跟新在线数据 |
| `checkpoint_period` | 20000 | 每 2 万迭代存 SAC 状态 |

BC agent 用 Adam `lr=3e-4`（[config.py:161](config.py#L161)）。

---

## 6. 教师与 BC 预训练（DP 先验）

- **教师**：Route-A LUT-native DP（离线，查流形表做动态规划），产出 `traj_*.pkl` 作为
  `--demo_path`，其转换由 `tools/build_dp_dataset.py` 生成，只带 `{"state": ...}`。
- **`filter_zero_actions=True` 的原因**（[config.py:96-103](config.py#L96-L103)）：
  DP 教师 88% 时间 `φ̇=0`（保持）。若保留这些保持动作，纯 MSE BC 会坍缩成"永远保持"
  （28% 完成、72% 不可行），平衡到 30% 移动仍然欠动（38%）。只保留约 12% 的"重定位"移动后
  → 92% 完成 / `e_task 0.057`，代价是干预率高（79%），再由 RLPD 奖励里的滤波项压下去。
- **`φ̇_scale`** 从 `data/aviator/dp_demo/phi_dot_scale.json` 读（Phase-3 标定），与
  `PHI_DOT_MAX` 一致（=1.5）。

---

## 7. 训练流程（launch）

`launch_rlpd.sh` 的启动顺序（详见脚本本体）：

1. **Preflight 审计**：`audit_prior_states.py`、`audit_unshielded_prior.py`、
   `audit_online_starts.py` —— 训练前校验 demo 先验、无屏蔽先验、在线起点一致。
2. **Learner**（`run_learner.sh`）：`MEM_FRACTION=.3`、`XLA_PYTHON_CLIENT_PREALLOCATE=false`。
3. **Actor**（`run_actor.sh`）：`MEM_FRACTION=.1`、同样关预分配。
4. `--demo_path` 指向 DP demo pkl。

> 运行运维要点（见 memory `aviator-rlpd-run-ops`）：
> 重启必须设 `XLA_PYTHON_CLIENT_PREALLOCATE=false` + `MEM_FRACTION(.3/.1)`，
> 否则 learner 预分配 36GB OOM；fresh restart 需整删 `debug_rlpd/`。

---

## 8. 部署（T6 + ONNX）

训练出的 actor 导出 ONNX，在线 T6（C++）用同一 40 维观察、同一 `action∈[−1,1]²` 接口：

```
φ_command = φ_estimate + action · φ̇_scale · dt        (φ̇_scale=1.5, dt=0.01)
```

在线流程（见 `methods/t6/README.md`）：实测 14 关节角反查当前 φ → actor 给 φ 速度 →
推下一 φ → 查流形表取 14 关节参考 → damped-Newton 投影回安全区间（`Q_x`/`Q_phi` 作雅可比），
最后一层硬防护就是 §4 的 `project_phi_dot`。

---

## 9. 关键参数速查

| 参数 | 值 | 出处 |
|---|---|---|
| 观察维度 | 40 | [env.py:45](env.py#L45) |
| 动作空间 | `[−1,1]²` | [env.py:82](env.py#L82) |
| dt / φ̇_scale / qdot_max / d_safe | 0.01 s / 1.5 rad/s / 1.5 rad/s / 5 mm | [env.py:41-43](env.py#L41-L43) |
| 历史 lag | {2,5,10,20} | [env.py:44](env.py#L44) |
| PHI_DOT_MAX / DPHI_DOT_MAX | 1.5 / 0.5 rad/s | [reward.py:50-51](reward.py#L50-L51) |
| ALIVE_BONUS / TERMINAL_PENALTY | +0.25 / −100 | [reward.py:52-53](reward.py#L52-L53) |
| 奖励权重 w_R/v/a/m | 1.0 / 0.05 / 0.05 / 0.1 | [reward.py:66-70](reward.py#L66-L70) |
| 网络 | MLP(256,256,256) tanh, ensemble-2 | [config.py:60-78](config.py#L60-L78) |
| discount / traj 长度 / buffer | 0.999 / 1200 / 1e6 | [config.py:89-91](config.py#L89-L91) |
| 硬屏蔽回退 | ≤5 次折半 | [safety_filter.py:33](safety_filter.py#L33) |

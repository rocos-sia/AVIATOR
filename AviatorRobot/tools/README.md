# AVIATOR 安全流形 Q(x, φ) 构建说明

> 本文档解释 `clearance_trajectory.cpp` 里 `build_manifold_phi()`（`TASKS=manifold_phi`）如何
> 离线建立"相位一致的安全流形"。流形是**双臂 14 关节构型关于任务状态与冗余自由度的确定性查表**，
> 供 T6（RLPD 在线）与 Route-A LUT-native DP（离线教师）直接查表，不再在线做 IK。
>
> 消费侧见 `methods/t6/README.md`；查表器为 `manifold_lookup.py` / `PhiLUT`。

---

## 0. 一句话概括

每个机械臂是 **7 自由度**，抓取任务是 **6 自由度刚体焊缝**（TCP 相对手柄的位姿固定），
因此每个臂还剩 **1 个冗余自由度**，即 elbow 绕"肩—腕"轴线的自运动环（拓扑上是 S¹）。
流形把这个冗余自由度参数化成相位 φ，并对每个任务状态 x=(θ,s) 离线追踪整条自运动环，
把环上每个 φ 处的关节构型与安全间隙存成表：

```
Q : (θ, s, φ) → (q_L, q_R) ∈ R^14
d : (θ, s, φ) → R           # 该构型对两堵墙的最小间隙
```

"安全流形"就是每个 (θ,s) 上满足 `d ≥ d_safe` 的那段 φ 区间。

---

## 1. 任务空间与运动学

### 1.1 任务状态 x = (θ, s)

| 符号 | 含义 | 取值范围 | 关节 |
|---|---|---|---|
| θ | 方向盘 roll（转向角，绕世界 z） | [−50°, +50°] = [−0.87266, +0.87266] rad | `roll_input_joint` |
| s | 方向盘 pitch（推拉，沿世界 z） | [−0.16, 0] m | `pitch_input_joint` |

### 1.2 抓取约束 = 6 自由度刚体焊缝

TCP 位姿在手柄坐标系下固定（`weld` 约束 `left_grasp` / `right_grasp`）。IK 目标帧为
（[clearance_trajectory.cpp:698-708](clearance_trajectory.cpp#L698-L708)）：

```
T_world→tcp(θ, s) = T_origin · T_roll(θ) · T_pitch(s) · T_grasp^rigid · T_tool⁻¹
```

其中

```
T_roll(θ)   = [ Rz(θ)  | 0     ]
T_pitch(s)  = [ I      | (0,0,s) ]
T_grasp^rigid = [ R_handle⁰ | p_handle ]   # 刚体焊缝：β=0，手柄轴自旋 φ=0
T_tool⁻¹    = 工具（夹爪）安装位姿的逆
```

> **注意**：`target()` 里还有一个 `φ` 参数（绕手柄轴 `handle_axis` 的抓取自旋，即 R0 松弛），
> 那是 grip-roll 研究用的，**不是本流形的 φ**。流形里焊缝是刚性的（β=0、手柄轴自旋固定），
> 因此任务维度 = 6，冗余维度 = 1。

### 1.3 冗余自由度 → 自运动环

7 关节 − 6 任务约束 = 1 冗余。该冗余对应"肘部绕肩—腕连线画圆"的自运动（self-motion）。
在关节空间里它是一个**闭环**（S¹），这是流形 φ 的几何来源。

---

## 2. 相位 φ 的定义：零空间方向 n

对当前构型 q，取 TCP 位点的 6×7 雅可比（3 位置 + 3 姿态，
[clearance_trajectory.cpp:781-795](clearance_trajectory.cpp#L781-L795)）：

```
J(q) = ∂[ p_tcp ; ω_tcp ] / ∂q  ∈ R^{6×7}
J = U Σ Vᵀ
n = V[:, 6]          # 右奇异向量，对应零奇异值，是零空间基
n ← n / ‖n‖∞        # ∞-范数归一化，使一步 ≈ 一个关节量程分数
```

于是 φ 是**零空间弧长**：

```
∂q/∂φ = n(q),   J(q) n(q) = 0        # 沿 n 走不改变 TCP 位姿
```

即 φ 每增加 1 单位，某个关节最多动 1 个"量程分数"。φ 是无量纲的（量纲类似 rad）。

---

## 3. 自运动环追踪：`self_motion_arc`

给定锚点构型 q₀ 与任务 (θ,s)，从 q₀ 出发沿 ±n 双向走，每一步都重新投影回焊缝约束：

```
前向：  q_{k+1} = IK( q_k + Δ · n(q_k) )      Δ = arc_step = 0.005
后向：  q_{k+1} = IK( q_k − Δ · n(q_k) )
```

- 每步重算 `n(q_k)`，若 `n_k · n_{k-1} < 0` 则翻转符号，保证沿同一分支继续（[1698-1834](clearance_trajectory.cpp#L1698-L1834)）。
- `IK(·)` = TRAC-IK `CartToJnt`，把 `q_k ± Δ·n` 作为种子，重新满足 6 自由度焊缝，
  所以 q 始终落在自运动曲线上，φ 度量的是零空间弧长。
- 越界（`clamp`）或 IK 失败即停止。

### 3.1 闭环判定与 branch 标志

```
闭环条件 = 前/后向走回 q₀ (‖q_k − q₀‖∞ < eps_q=0.12 rad，
           且途中曾离开 q₀ 超过 delta_away=0.25 rad，且步数 ≥ min_steps=300)
        或 双向都在关节限位处终止
branch = 0 (闭环)  /  1 (开环，一侧 IK 失败或单侧限位)
```

`L` = 环的总弧长 = `φ_fwd_total − φ_bwd_total`（[1840](clearance_trajectory.cpp#L1840)）。

---

## 4. 安全约束：wall clearance 与 safe 区间

### 4.1 间隙 d

对构型 q，取该臂网格采样点云（`cloud`），算每个点到两堵墙盒（`aviator_wall_left/right`）
的最近距离，取最小（[753-780](clearance_trajectory.cpp#L753-L780)）：

```
d(x, q) = min_{采样点 p, 墙 w}  dist_box(p_world, wall_w)
```

> **环境不对称**：右墙 z=−0.042 m 比左墙 z=−0.018 m **低约 24 mm**（由手柄连线倾斜
> `axis_z=0.044` 经 `generate_aviator.py` 对称放置放大而来）。所以 d、进而 safe 区间、
> 进而整张表都不是 θ ↔ −θ 镜像对称的——+θ 端比 −θ 端更早触墙，这是**几何事实不是 bug**。

### 4.2 safe 区间

沿环把每个采样点的 d 与阈值比（`d_safe = 5 mm`）：

```
I_safe(x) = [ min φ,  max φ ]  其中 φ 遍历 { φ_k : d(q(x, φ_k)) ≥ d_safe }
```

注意：这是**全部安全 φ 的外接盒**，不要求包含 φ=0（连续化种子本身可能在 +θ 深角处穿透，
见 [1861-1866](clearance_trajectory.cpp#L1861-L1866) 注释）。策略与安全滤波作用在这个完整的
safe box 上，最后的硬防护由查表后的 damped-Newton 投影保证。

---

## 5. 全局 φ 网格与重采样

在锚点 (θ=0, s=0) 取左右两臂 safe 区间的并集，向外扩 `phi_grid_margin=0.3`：

```
glo = min(lo_L, lo_R) − 0.3
ghi = max(hi_L, hi_R) + 0.3
φ_grid[k] = glo + (ghi − glo) · k / (n_phi − 1),   k = 0..127
```

每个网格点的自运动环轨迹 `(φ_k, q_k, d_k)` 再**分段线性重采样**到这个全局 φ 网格上
（[1932-1959](clearance_trajectory.cpp#L1932-L1959)）。这样整张表共享同一个 φ 轴，查表才可做
三线性插值。

---

## 6. 连续化（continuation）与 wavefront

"相位一致"靠**相邻网格点之间用 φ=0 构型作种子**传递实现——φ 在所有点上保持同一物理含义。

```
q0(x) = ik_multi_seed( side, x ; seed = q0(邻居) )
```

即每个点的 φ=0 构型由邻居的 φ=0 构型 warm-start 得到（TRAC-IK 局部连续化，而非全局
Distance 搜索，避免跳到另一个不相连的自运动分支，见 [1171-1179](clearance_trajectory.cpp#L1171-L1179)）。

Wavefront 三层（确定性，[1986-2083](clearance_trajectory.cpp#L1986-L2083)）：

```
Layer 0  锚点 (θ=0, s=0)，种子 = home 构型 seeds[side]
Layer 1  s=0 的 ±θ 行，从锚点向两侧顺序链式（每列种子上一个 θ）
Layer 2  每条 θ 的 s 列，跨 θ 并行、列内顺序（每个 s 种子 s+1）
```

### 6.1 相位一致性校验

对每个网格点，比较其 φ=0 构型与 4 邻居的 φ=0 构型最大单关节差；若 > 0.5 rad 则
`branch = 3`（排除出安全流形，[2086-2116](clearance_trajectory.cpp#L2086-L2116)）。

### 6.2 branch 编码

| 值 | 含义 |
|---|---|
| 0 | 相位一致（闭环，可用） |
| 1 | 开环（自运动弧未闭合）——**Route A 不可硬过滤** |
| 2 | IK 不可达（无法播种） |
| 3 | 相位一致性违反（φ=0 构型跨邻居跳变） |

---

## 7. 输出文件与布局

输出目录 `outdir/manifold_phi/`（[2151-2203](clearance_trajectory.cpp#L2151-L2203)），
布局 `s-major`：`index = i_s · n_theta + i_theta`；每点每臂 `n_phi` 个 φ 采样，关节内层。

| 文件 | 形状 | 内容 |
|---|---|---|
| `phi.bin` | (n_phi,) | 全局 φ 轴 |
| `qL.bin` / `qR.bin` | (N·n_phi·7,) | 构型 Q，`N = n_theta·n_s` |
| `dL.bin` / `dR.bin` | (N·n_phi,) | 每 φ 采样的间隙 d |
| `safe.bin` | (N·4,) | 每点 `[lo_L, hi_L, lo_R, hi_R]` |
| `branch.bin` | (N,) | branch 标志（uint8） |
| `QxL/QxR.bin` | (N·n_phi·7·2,) | ∂q/∂θ、∂q/∂s（中心差分） |
| `QphiL/QphiR.bin` | (N·n_phi·7,) | ∂q/∂φ（中心差分） |
| `manifest.json` | — | 网格尺寸、L_phi、φ 编码、gap 等 |

导数用中心差分（[2118-2149](clearance_trajectory.cpp#L2118-L2149)）：

```
∂q/∂θ ≈ (q(θ+dθ) − q(θ−dθ)) / (2·dθ),   dθ = 1°
∂q/∂s ≈ (q(s+ds) − q(s−ds)) / (2·ds),   ds = 2.5 mm
∂q/∂φ ≈ (q(φ+dφ) − q(φ−dφ)) / (2·dφ)
```

---

## 8. 在线查询：三线性插值

`PhiLUT`（[341-473](clearance_trajectory.cpp#L341-L473)）与 `manifold_lookup.py` 位一致：

```
Q(x, φ)  = Σ_{dθ,ds,dφ ∈ {0,1}} w_θ·w_s·w_φ · Q(i_θ+dθ, i_s+ds, i_φ+dφ)   # 8 角加权
d(x, φ)  = 同上对 d
branch(x) = max_{4 个 (θ,s) 角} branch            # 保守排除
```

在线（T6）流程：实测 14 关节角反查当前 φ → 用 actor 给的 φ 速度推下一 φ → 查表取 14 关节
参考 → damped-Newton 投影回 safe 区间（`Qx`/`Qphi` 作雅可比），细节见 `methods/t6/README.md`。

---

## 9. 关键参数速查

| 参数 | 值 | 出处 |
|---|---|---|
| n_theta / n_s / n_phi | 101 / 65 / 128 | [1910-1912](clearance_trajectory.cpp#L1910-L1912) |
| θ 范围 | ±50°（1°/步） | |
| s 范围 | [−0.16, 0] m（2.5 mm/步） | |
| arc_step | 0.005（零空间单位/步） | [1685](clearance_trajectory.cpp#L1685) |
| eps_q（闭环容差） | 0.12 rad（‖·‖∞） | [1686](clearance_trajectory.cpp#L1686) |
| delta_away | 0.25 rad | [1687](clearance_trajectory.cpp#L1687) |
| min_steps / max_steps | 300 / 2517 | [1688-1689](clearance_trajectory.cpp#L1688-L1689) |
| d_safe | 5 mm | [491](clearance_trajectory.cpp#L491) |
| phi_grid_margin | 0.3 | [2001](clearance_trajectory.cpp#L2001) |
| 相位一致性阈值 | 0.5 rad | [2112](clearance_trajectory.cpp#L2112) |

---

## 10. 构建命令

```bash
# 生成相位流形（写入 OUTPUT_DIR/manifold_phi/）。argv[7] 是逗号分隔的任务标签，
# argv[8] 对 manifold_phi 而言是线程数。其余参数按默认值即可。
./aviator_clearance_trajectory <CONFIG> <OUTPUT_DIR> 0.005 0.03 10 0.1 manifold_phi 16
#        argv:                  [1]       [2]       [3]=d_safe [4]=q_margin [5]=trace_half [6]=trace_step [7]=task [8]=threads
```

（`argv[7]` 传 `manifold` / `manifold_safe` 是 2 维截面 Q_g(θ,s)，不含 φ，供 T5 用；
本流形传 `manifold_phi`，含完整 φ 维度，供 T6 / Route-A DP 用。代码里报错文案写
`TASKS=manifold_phi`，但实际读的是 argv[7] 的逗号分隔列表，不是环境变量。）

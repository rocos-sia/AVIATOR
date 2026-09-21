# AVIATOR 狭小空间间隙–冗余研究整理

整理日期：2026-09-20。本文汇总 AVIATOR 平台上「狭小空间 clearance 如何限制双臂操纵、冗余能否持续救援」这一组离线测量的**全部结论与过程**，包括方法、修复、数值边界、姿态自由度（φ / β / 全姿态）释放实验，以及产物与复现方式。

本文所有数值均来自 `AviatorRobot/tools/clearance_trajectory.cpp` 在 MuJoCo 中的离线测量，并已逐项核对数据文件。结论**只适用于**下述限定的平台、边界与任务；不自动推广到其他抓取、其他障碍布局、其他机构任务。

---

## 0. 一句话结论

$$
\boxed{\text{在当前 AVIATOR + 当前侧向狭窄边界 + 当前操纵盘任务下，释放末端姿态自由度几乎没有额外几何价值。}}
$$

更精确地：**position-only（R3，姿态全自由）都没有把几何边界 $g_{\rm geom}$ 从 0.4921 m 往下推**，因此没有理由继续在 ranged orientation（可松弛姿态）方向上投入。姿态自由度在当前任务下不是主要 leverage；真正起作用的是 1-D 自运动冗余 $\rho$。

---

## 1. 研究问题与「逐层收紧」链条

场景：左右各 7 自由度的双臂，在 MuJoCo 中经两个 **weld**（刚性锁死）被动带动一个 **2 自由度操纵盘**（yoke，非可无限转动的汽车方向盘）：转角 $\theta$（±50°）与推拉 $s$（−165～0 mm）。两侧 ±X 放置**可调间距的垂直墙**，把工作空间夹窄，使「狭小空间」真实成立。

核心问题：随着墙间隙收窄，操纵权限（clearance / 可执行性）如何坍塌？机器人冗余能否在**引入姿态自由度之前**持续救援轨迹？

研究组织为一条「逐层收紧」链条：

| 层级 | 含义 | 关系 |
|---|---|---|
| **B0** | baseline continuation IK（`q_k = TRAC_IK(seed=q_{k-1})`，不重塑） | 最弱 |
| **B1** | $d^*_{\rm static}(x_k)$：对整个自运动流形取最大 clearance 的**逐点几何上界** | 静态最优 |
| **B3** | least-intervention QP：`q_k = q_base + α_k n`，在 clearance ≥ $d_{\rm safe}$ 前提下取最小 |α| 重塑 | 连续可执行 |
| — | 冗余耗尽 | $g_{\rm geom}$ |
| — | 释放姿态自由度（φ / β / 全姿态） | 本研究的终点问题 |

三分类可执行性：$\Omega_{\rm base} \subseteq \Omega_{\rm red} \subseteq \Omega_{\rm IK}$；clearance class $C\in\{0=\text{baseline}, 1=\text{redundancy-recoverable}, 2=\text{geometrically-infeasible}\}$。

贯穿始终的不等式：$d_{\rm base} \le d_{\rm executable} \le d^*_{\rm static}$，安全阈值 $d_{\rm safe}=5$ mm，关节限位裕量 $m_q=0.03$ rad。

---

## 2. 平台与实验设置

- **模型** `aviator.xml`：`roll_input_joint` hinge 轴 `"0 0 1"`，范围 ±0.87266 rad；`pitch_input_joint` slide 轴 `"0 0 1"`，范围 −0.165～0 m。两无驱动关节由双臂经 weld 被动带动。
- **墙**：左右 ±X 两块 box，half-extent (0.3, 0.3, 0.015) m，厚度沿局部 Z；`contype=1, conaffinity=7`（与臂碰撞）。间隙 `gap` 可调（本文扫 0.47～0.54 m，关键区 0.480～0.518 m 用 fine 模型）。
- **抓取**：`grasp.json`，rigid weld（`mjEQ_WELD`）；`tool` offset (0,0,0.1) m + 绕轴 180°。
- **任务**：`roll_pos`（$\theta=+0.87266$）、`roll_neg`（$\theta=-0.87266$，**最难**，即绑定任务）、`pull`（$s=-0.16$）。
- **绑定臂**：`roll_neg` 的 side 0（左臂）。其余 arm/task 在边界附近 clearance 高 1～32 mm，不绑定。
- **关节**：各臂 7 轴（joint_1 Z, joint_2 Y, joint_3 Z, joint_4 Y, joint_5 Z, joint_6 Y, joint_7 X）；J2 任务范围 85°～95°（保留 0.5° 余量，不放宽）。
- **姿态参数化**（用于释放实验）：$t$=把手切线（=handle axis）、$r$=指向轮心的径向、$n$=操纵盘法线；$R_i(\alpha,\beta)=R_{i,0}\cdot\mathrm{Rot}_t(\alpha)\cdot\mathrm{Rot}_r(\beta)$。实测 $\text{handle\_radial}\approx \pm X$，$\text{handle\_axis}\approx +Y$ 向 +Z 倾斜约 21°，$n=(0,0,1)$。

---

## 3. 方法要点

- **1-D 自运动**：对 6×7 雅可比（`mj_jacSite`）做 JacobiSVD，零空间方向 $n=V_{.6}$；沿单位零空间 continuation + re-projection 走完整闭环，取最大 clearance。
- **IK**：TRAC-IK Distance 模式（取最近 seed）；`CartToJnt` 成功返回正迭代数、失败返回负。
- **continuation seeding**：姿态/间隙扫时以上一个可达解作 seed，回退到 home seed，保证只覆盖与 baseline 相连的 IK 分量。
- **clearance**：decimated mesh 顶点**点云**（每 link ~400 点，体坐标系）+ 解析 point-to-box 有符号距离（见 §4）。
- **穿透**：深穿透时 box 最近面距离会饱和（half-thickness = 15 mm）；改求**内面平面**有符号距离，单调无界。

---

## 4. 关键发现一：`mj_geomDistance` 假零点（根因修复）

- **现象**：$d^*_{\rm static}(g)$ 非单调，在 0.504 / 0.506 m 出现假零点（fake zero）。
- **根因**：MuJoCo 3.4.0 的 `mj_geomDistance(mesh, box)` 对（非凸）臂 mesh geom 返回**精确 0.0**，且随墙位置非单调。早先怀疑「自运动扫覆盖不足」是**错误**的——全环扫不改结果，几何查询才是 bug。
- **修复**：mesh 顶点降采样点云（体坐标，~400 pts/link）+ 解析 point-to-box 有符号距离。修复后 $d^*_{\rm static}(g)$ 单调且**精确线性**（斜率 0.5 m/m）。

---

## 5. 关键发现二：四个边界（$d_{\rm safe}=5$ mm，任务 `roll_neg`）

$d^*_{\rm static}$ 与 $d_{\rm base}$ 都精确线性于 gap，斜率 0.5 m/m（臂只受一面墙限制，墙以半速移动）：

$$
d^*_{\rm static}(g) = 0.5\,g - 0.246053,\qquad
d_{\rm base}(g) = 0.5\,g - 0.267751
$$

| 边界 | 定义 | 数值 |
|---|---|---|
| Baseline 边界 $g_{\rm B0}$ | $d_{\rm base}=0$ | **0.5355** |
| 冗余安全边界 $g_{\rm safe}^{\rho}$ | $d^*_{\rm static}=d_{\rm safe}$ | **0.5021** |
| 刚性抓取几何边界 $g_{\rm geom}^{\rho}$ | $d^*_{\rm static}=0$（真实物理极限） | **0.4921** |

**自运动救援**：冗余在整条边界上提供**常数 21.70 mm** clearance 提升（$0.267751-0.246053$），折算 **43.4 mm** 的间隙回收（$g_{\rm B0}-g_{\rm geom}$）。

**$d_{\rm safe}$ 灵敏度**（$g_{\rm safe}$ 随 $d_{\rm safe}$ 以 2× 平移）：$d_{\rm safe}=2$ mm→0.4961，5 mm→0.5021，10 mm→0.5121。线性拟合在实测 [0.480, 0.518] 全区间到 6 位小数精确。

> 注：旧（corrupted）数据的 $g^*\approx 0.505$ 不是几何极限，而是坏 `mj_geomDistance` 下 $d_{\rm safe}=5$ mm 的安全可执行边界。

**B3 连续可执行边界**：从 `sweep_summary_fine.csv`，$d_{\rm B3,min}$ 在 $g=0.504$ 处首次 $\ge d_{\rm safe}$（$P_{\rm succ}=1$），即连续执行边界 $\approx 0.504$，几乎实现 B1 的静态上界（差 ~0.5 mm 是动态/连续代价）。

---

## 6. 关键发现三：joint-margin ablation

`roll_neg` 末端（$\theta\approx -0.87$）在 $g=0.54$ 出现 joint-limit 绑定的 knot。$m_q\in\{0,0.01,0.02,0.03\}$ 消融显示 $m_q=0$ 时 $P_{\rm succ}=1.0$——那是**保守裕量**，不是真实冗余耗尽。

---

## 7. 关键发现四：φ 的物理合法性（Step 4）

- φ = 绕把手（握持圆柱）轴的旋转。因抓取圆柱与把手段**同轴且包含于**把手内（`handle_axis` ≈ handle.M.UnitZ），φ 是**无滑移的纯自旋**，任意 φ 都物理合法。
- 合法性另以 φ sweep（`phi_sweep.csv`）验证：φ 从 baseline 分支**连续可达**的范围有限。在绑定位姿 $\theta=-0.87266$ 处，side 0 可达约 $\varphi\in[0, +0.98]$ rad、side 1 约 $\varphi\in[0, +1.18]$ rad，超出后 IK 因关节限位失败（`ik_ok<0`）。
- 在可达范围内 φ 会显著改变 clearance：绑定位姿处 home seed（$\varphi=0$）的 clearance 仅 0.0022 m，转到 $\varphi\approx 0.39$ 提升到 0.0239 m（回收约 21.7 mm，与自运动救援同量级）。这正是 φ 把 home 次优分支拉回 continuation-ρ 最优分支的机制（定量见 §8）。

---

## 8. 关键发现五：静态 φ ablation（Step 5，**负结果**）

`self_motion_trace_phi()` 扫 φ ∈ [−1,1]（9 点，continuation 从 φ=0），每个 φ 走完整 ρ 环，得 $d^*_{\rho+\varphi}=\max_{\rho,\varphi}$。绑定臂 `roll_neg` side 0 的结果：

- **φ gain 在 gap 上精确常数**：+0.004024856 m（到 9 位小数）——这是臂几何的固有属性，与墙位置无关。
- home-seed 的 $d^*_{\rho}=0.5g-0.250130$ → $g=0.5003$（home 落在**次优分支**上，自运动环被关节限位分段）。
- home-seed 的 $d^*_{\rho+\varphi}=0.5g-0.246105$ → $g=0.4922$，与 continuation 的 $d^*_{\rho}=0.5g-0.246053$（$g_{\rm geom}=0.49211$）**相差仅 0.05 mm clearance / 0.1 mm gap**，远低于 ~1–2 mm 的点云噪声底。

**结论**：$g_{\rm geom}^{\rho+\varphi}=g_{\rm geom}^{\rho}=0.4921$。释放 φ 只是从一个次优 seed 追平了 continuation-ρ 的最优，**并未延伸几何边界**。φ 与 ρ 在边界处冗余——限制几何是腕/前臂靠近把手处，其到墙距离由轮位姿固定，与 φ 无关。故 **B3 不升级 φ**。

---

## 9. 关键发现六：约束释放阶梯 R0→R3（Step 6，**负结果**）

为回答「姿态自由度（不只一维 φ）到底有没有价值」，做分层释放实验（`release_ablation` 模式），逐级测 $d^*$：

| 层级 | 约束 | 零空间维数 |
|---|---|---|
| **R0** | 3P+3R 刚性（仅 ρ） | 1-D |
| **R1** | 3P+2R（+φ 绕把手轴） | 2-D |
| **R2a** | 3P+2R（+β 绕径向 $r$） | 2-D |
| **R2b** | 3P+2R（+β 绕法线 $n$） | 2-D |
| **R3** | 3P（姿态全自由，3 参数 Rot_t·Rot_r·Rot_n） | 4-D |

绑定臂 `roll_neg` side 0 的 $d^*(g)$ 拟合（斜率恒 0.5 m/m；g ∈ {0.490, 0.492, 0.494, 0.500}）：

| 层级 | $d^*(g)$ 拟合 | $g_{\rm geom}$ |
|---|---|---|
| R0（home seed） | $0.5g - 0.250130$ | 0.5003 |
| R1 | $0.5g - 0.246105$ | **0.4922** |
| R2a | $0.5g - 0.246092$ | **0.4922** |
| R2b | $0.5g - 0.246092$ | **0.4922** |
| R3 | $0.5g - 0.246092$ | **0.4922** |

三个关键点：

1. **R2a / R2b / R3 逐位相同**（g=0.490 处分别为 −0.0010923862 / −0.0010923863 / −0.0010923863 m），R1→R2/R3 的增益只有 **+0.0123 mm**——比 ~1–2 mm 的点云噪声底低约 100×。**R3（position-only）在 R1 之上没有任何增量。**
2. **这 0.0123 mm 不是 β 的功劳，而是 φ 网格分辨率的伪差**：每个 R2a/R2b/R3 最优点都是 $\varphi=0.333,\ \beta_r=\beta_n=0$；R1 用 9 点 φ 网格取到 $\varphi=0.5$，R2/R3 用 7 点取到 $\varphi=1/3$，只是 φ 采得更细。（t,r）与（t,n）两组 β 都**不影响** elbow/forearm 包络。
3. **绑定臂未被翻转**：`roll_pos` side 0 的 φ 增益为 0（R0=R1），β 只给 ~0.03 mm；`pull` 任何姿态释放增益为 0。`roll_neg` side 0 在所有层级下仍是绑定臂。

**物理解释**：限制几何是 **elbow/forearm 的包络（位置层级）**，由轮位姿 $(\theta,s)$ + 一维自运动 $\rho$ 决定。手腕姿态（φ / β）只能重定向手边最后 2–3 个连杆，无法移动这个包络。所以 1-D 冗余 $\rho$ 已是该机构在此边界下**完整**的救援机制。

---

## 10. 最终结论（限定范围）

$$
\boxed{\text{在当前 AVIATOR + 当前侧向狭窄边界 + 当前操纵盘任务下，释放末端姿态自由度几乎没有额外几何价值。}}
$$

- **数值支撑**：$g_{\rm geom}^{\rm R0}=g_{\rm geom}^{\rm R1}=g_{\rm geom}^{\rm R2a}=g_{\rm geom}^{\rm R2b}=g_{\rm geom}^{\rm R3}=0.4921$（R0 取 continuation-ρ 意义；home-seed 的 0.5003 是次优分支的 seed 伪影）。
- **R3 是强运动学上界**：position-only 都没把 $g_{\rm geom}$ 往下推 ⇒ 无需再在 ranged orientation（可松弛姿态）上消耗时间，也无需为动态 $\alpha(t),\beta(t)$ 把 weld 改成 2-DoF 万向节 / 柔顺抓取 / 滚动接触。
- **范围限定**：这不代表其他抓取、其他障碍布局、其他机构任务中姿态自由度没有价值。只有在本平台、本侧向狭窄边界、本操纵盘任务下才成立。

---

## 11. 后续建议

- **不必**：继续 ranged-orientation（R2/R3 已是 kinematic upper bound，静态价值为零）。
- **可选**（若要进一步压 $g_{\rm geom}$，与姿态无关）：双臂关节联合优化（当前单臂扫、另一臂冻结为近似）；用**真实 mesh 碰撞**替代点云（点云 ~400 pts/link 高估 clearance ~1–2 mm，故 $g_{\rm geom}$ 是下界，真实极限可再宽 ~2–4 mm）。
- **论文叙事**：冗余救援（B0→B1 = 常数 21.70 mm）是正确且完整的主机制；姿态自由度可作为一个**明确测过且排除**的候选 leverage，作为消融写进论文。

---

## 12. 产物与复现

### 代码（`AviatorRobot/`）

| 文件 | 作用 |
|---|---|
| `tools/clearance_trajectory.cpp` | 离线测量主工具（B0/B1/B3、φ sweep、φ ablation、release_ablation、点云 clearance） |
| `tools/authority_map.cpp` | 方向任务 authority heatmap（诊断用，非 headline） |
| `scripts/sweep_clearance.py` / `plot_sweep_clearance.py` | 间隙扫描汇总 + 三图（静态包络 / 可执行性 / 裕量消融） |
| `scripts/plot_phi_ablation.py` | Step 5 φ ablation 图 |
| `scripts/plot_release_ablation.py` | Step 6 阶梯（DOF-value curve）图 |
| `reference/rocos-mujoco/scripts/generate_aviator.py` | 加墙 MJCF 生成（`WALLS` 可调 gap） |

### 数据（`/tmp/aviator-sweep/`）

- `sweep_summary.csv` / `sweep_summary_fine.csv`：B0/B1/B3 全指标。
- `/tmp/aviator-phi-ablation-*/phi_ablation.csv`：Step 5。
- `release-abl/fine_*/release_ablation.csv` + `release_ablation.png`：Step 6。
- `configs/fine_*.yaml` + `models/fine_*/aviator.xml`：各 gap 模型。

### 复现命令

```bash
# 1) 生成带墙模型（改 gap 后重跑）
python3 reference/rocos-mujoco/scripts/generate_aviator.py

# 2) 构建
cmake --build AviatorRobot/build --target aviator_clearance_trajectory -j4

# 3) 单 gap 三 regime 扫描 / φ ablation / release 阶梯
./AviatorRobot/build/bin/aviator_clearance_trajectory \
  /tmp/aviator-sweep/configs/fine_0.490.yaml /tmp/out 0.005 0.03 10 0.1 release_ablation
# TASKS 参数可换：默认任务 / phi_sweep / phi_ablation / release_ablation

# 4) 画图（务必用 serl_clean 环境的 python）
/home/rocos/miniconda3/envs/serl_clean/bin/python AviatorRobot/scripts/plot_release_ablation.py /tmp/aviator-sweep/release-abl
```

CLI：`aviator_clearance_trajectory CONFIG OUTDIR [D_SAFE=0.005] [Q_MARGIN=0.03] [TRACE_HALF=10] [TRACE_STEP=0.1] [TASKS] [DT=0.02]`。

---

## 13. 已知近似与局限

1. **点云高估 clearance**：~400 pts/link 的降采样点云比真实 mesh 高估 ~1–2 mm，故 $g_{\rm geom}$ 是下界（真实极限最多再宽 ~2–4 mm）。
2. **自运动扫覆盖**：从 baseline 分量出发的 nullspace continuation 只覆盖与 baseline **相连**的 IK 分量，非全 branch（home-seed 0.5003 次优分支即此来源）。
3. **扫单臂时另一臂冻结**在 baseline 构型（臂–臂碰撞是近似）。
4. **R2/R3 是运动学上界**：β 破坏 coaxial weld（建模 palm-tilt），非刚性抓取；若真要动态执行需改 weld。本研究的结论正是**无需**这样做。
5. **φ/β 网格分辨率**：R2 7×7、R3 7×5×5、R1 φ 9 点；最优 φ 由网格采样决定（0.5 vs 0.333），产生的 0.0123 mm 差是伪差，不影响结论。
6. B3 的 authority/动态约束是 rate-constrained 近似；近奇异处用保守 min-norm 回退。

---

## 14. 在线动态阶段（B0/B3 实时冗余重塑，Milestone 1）

静态阶段（§10）结论：锁回 rigid grasp，转入**实时、逐控制周期、因果**的双臂冗余重塑。研究问题从
「How narrow can it fit?」（静态 $g_{\min}$）升级为「How fast can it operate safely in that narrow
space?」（动态 $f_{\max}$ + 二维包络 $\Omega_{\rm executable}(g,f)$）。

### 控制器结构

- 每控制周期只读当前状态 $(x_t,\dot x_t,q_t,\dot q_t,d_t)$ + 纯时间命令 profile，**不读未来轨迹**。
- **B0**（对照）：一步 continuation IK（`CartToJnt(seed=q_prev)`），无重塑。
- **B3**（reactive）：continuation IK 给 on-manifold baseline $q_{\rm base}$，再在 1-D 自运动零空间
  （`solve_b3`，rate/accel/joint-limit 约束）做**最小干预**重塑：最小 $|\alpha|$ 使
  $d(q_{\rm base}+\alpha n)\ge d_{\rm safe}$，否则取 max-$d$。
- 三种命令 profile：step reversal（三角 0→+0.87）、正弦扫频 $\theta=A\sin 2\pi ft$（$f\in\{0.1..1.0\}$ Hz）、
  roll+pull 混合（$\theta,s$ 独立正弦）。
- 命令 profile 推进轮位姿（`set_config`），双臂通过 6-D weld 约束实时跟踪。

### 关键发现：紧位姿在 $\theta\approx +0.5$（右臂 side 1），而非极端角

离线只扫了端点 $\theta=\pm0.87$，**漏掉了中间紧位姿**。在 gap=0.54 m（默认配置）下：

- `debug_qp` @ $\theta=0.5$：**side 1（右臂）是绑定臂**，baseline clearance 仅 2.99 mm $<d_{\rm safe}$；
  其零空间 `+0.05n` 可达 53.7 mm（**可达但需足够快**）。
- B0 的 continuation IK 在 $\theta\approx0.5$ 因方向反转/分支不稳走坏分支，clearance 掉到 **−0.43 mm（穿透）**，
  且**与频率无关**（f=0.1..1.0 全如此）。

### 动态结果（gap=0.54 m，$d_{\rm safe}=5$ mm）

| 方法 | 结果 | $f_{\max}$ |
|---|---|---|
| **B0**（continuation IK） | $d_{\min}\approx-0.43$ mm（穿透）@ 所有 $f$ | **0**（任何频率都不安全） |
| **B3**（reactive 重塑） | 低速 $d_{\min}\approx d_{\rm safe}$；随 $f$ 单调恶化 | 0.10 Hz（roll_pull 0.1 Hz 仅 5.00004 mm，薄裕量） |

B3 的 $d_{\min}$ 随频率退化（正弦）：f=0.1 → 4.80 mm，f=0.2 → 3.11 mm，f=0.3 → −0.40 mm，f=0.5 → −0.43 mm；
roll_pull 同构（0.1 Hz 5.00 mm → 0.7 Hz −0.12 mm）。这就是计划里预判的 **“reactive 反应太晚”**：
紧位姿 $\theta\approx0.5$ 被轮子以速度扫过，而 B3 的重塑受加速度约束（$\ddot q_{\max}=10$ rad/s²）限速，
来不及在穿透前把肘收起。→ **动机 B4（短时域预测，提前收肘）**。

### 在线阶段的架构决策与已知问题

- **B3 用 position-level（continuation IK + 零空间网格），不用 velocity-level**。velocity-level
  $q̇=J^+v_T+N z$ 在重塑后的近奇异构型处 $J^+$ 病态（首步 20 rad/s 发散），且 `safe_start` 曾返回
  off-manifold home seed（TCP 偏 76 mm）+ 解析速度前馈双计了被精确设定的轮位姿。position-level 的
  `solve_b3` 已证明稳健（tracking 误差 $e_{\rm task}\approx10^{-6}$ m）。
- **`rate_sat` 指标被 continuation-IK 分支跳变污染**：max $\ddot q$（f=0.1 时 17.6 rad/s²）来自离散
  $q_{\rm base}$ 在分支间跳变，而非真实臂加速度（轮子 $\ddot\theta$ @0.1 Hz 仅 ~0.34 rad/s²）。
  max $\dot q$（0.29→4.81 rad/s）才是真实速度信号。做 $f_{\max}$ headline 前需修（只测零空间干预
  $\alpha$ 的 rate，或把 $q_{\rm base}$ 每步 clamp 到 $\dot q_{\max}\Delta t$）。

### 复现命令

```bash
./AviatorRobot/build/bin/aviator_clearance_trajectory config/aviator.yaml /tmp/out 0.005 0.03 10 0.1 online_b0 0.02
./AviatorRobot/build/bin/aviator_clearance_trajectory config/aviator.yaml /tmp/out 0.005 0.03 10 0.1 online_b3 0.02
# 输出 online_summary_{b0,b3}.csv + trajectory_online_{b0,b3}_{profile}_f*.csv
```

---

## 15. B4 短时域预测（Milestone 2）：$f_{\max}$ 提升 3×

§14 确认了 B3 的 “reactive 反应太晚” —— 紧位姿 $\theta\approx0.5$ 被轮子扫过时，B3 受加速度约束限速，
来不及提前收肘。B4 的做法：**在 B3 的同一套 rate/accel/joint-limit 约束盒内，把“重塑触发”从
“当前 $d<d_{\rm safe}$”换成“未来 baseline $d<d_{\rm safe}$”** —— 用 constant-velocity 预测轮位姿
$\theta_{k+h}=\theta+h\Delta t\,\dot\theta$（$H=10$，0.2 s），continuation IK（不重塑）算出未来 baseline
clearance $d_{\rm future}=\min_h$；若 $d_{\rm future}<d_{\rm safe}$，**现在**就朝**当前最大 clearance 的
自运动分支**重塑（max-$d$ 目标，平局取最小 $|\alpha|$），为后续 continuation 腾出“高分支”余量。
实现为 `solve_b4`（`clearance_trajectory.cpp:909`），与 `solve_b3` 共用零空间与约束盒，仅触发与目标不同。

### 结果（gap=0.54 m，$d_{\rm safe}=5$ mm）

$d_{\min}$ [mm] vs 频率 $f$ [Hz]（正弦 / roll_pull）：

| $f$ [Hz] | 0.1 | 0.2 | 0.3 | 0.5 | 0.7 | 1.0 |
|---|---|---|---|---|---|---|
| B0 sine | −0.46 | −0.46 | −0.46 | −0.43 | −0.43 | −0.43 |
| B3 sine | 4.80 | 3.11 | −0.40 | −0.43 | −0.43 | −0.43 |
| **B4 sine** | 6.09 | 9.24 | **13.52** | 2.55 | −0.43 | −0.43 |
| B0 roll_pull | −0.14 | −0.14 | −0.14 | −0.12 | −0.14 | −0.00 |
| B3 roll_pull | 5.00 | 4.20 | 1.81 | −0.00 | −0.12 | −0.00 |
| **B4 roll_pull** | 5.78 | 8.12 | **10.48** | 2.12 | 2.14 | −0.00 |

**Headline**：$f_{\max}$（$d_{\min}\ge d_{\rm safe}$ 的最大频率，任取 profile）从 **B0=0 → B3=0.10 Hz → B4=0.30 Hz**，
**B4 提升 3×**。且 B4 的裕量是实的（0.3 Hz 时 10.5–13.5 mm），而 B3 的 0.10 Hz 是 razor-thin（roll_pull 0.1 Hz
仅 5.00004 mm）。B4 还把 roll_pull 的**穿透**推迟到 f=1.0（0.5/0.7 Hz 仍维持 2.1 mm 不穿透），而 B3 在
0.5 Hz 就穿透了。

### 解释

- B4 的 $d_{\min}$ 在 0.1→0.2→0.3 Hz **非单调上升**（6.09→9.24→13.52 mm）：轮子越快，0.2 s 的 lookahead 看到的
  $\theta$ 偏移越大，触发越早/越激进，max-$d$ 重塑越多。这是 constant-velocity 时域的自然结果，而非噪声。
- 到 f=0.5 Hz 仍跌破（2.55 mm）：预测正确但**执行受加速度限速**——$H\Delta t=0.2$ s 的提前量已不足以把肘在
  紧位姿到达前收到高分支。这是 rate/accel 约束下的物理上限，属于下一层（更高 $\ddot q_{\max}$ 或更长时域）。
- **成本**：B4 干预更多（$J_{\rm interv}=0.34$–$0.61$ vs B3 的 $0.16$–$0.33$，因为 max-$d$ 而非最小 $|\alpha|$）；
  max $\dot q$ 也更高（0.3 Hz 时 2.5 rad/s vs 1.6），但仍在限内。这是“提前、主动”重塑的合理代价。

### 复现命令

```bash
./AviatorRobot/build/bin/aviator_clearance_trajectory config/aviator.yaml /tmp/out 0.005 0.03 10 0.1 online_b4 0.02
python AviatorRobot/scripts/plot_b4_predictive.py \
  /tmp/aviator-online-b0-fixed/online_summary_b0.csv \
  /tmp/aviator-online-full/online_summary_b3.csv \
  /tmp/out/online_summary_b4.csv
```

### 遗留（Milestone 3 之前）

- **`rate_sat` 仍恒为 1**（§14 的 continuation-IK 分支跳变污染未修）。做 $f_{\max}$ 为正式 headline 前需修。
- B4 目标是**当前**最大 clearance（不是“未来最小 clearance 最大”）；更原则性的 B4 应把候选 $\alpha$ 沿时域
  前推后取 $\min_h d$。当前启发式已足够证明“提前重塑”的价值，可在正式版升级。

---

## 16. T5：Safe-Manifold Tracking（§19 三问的静态验证）

### 方法定位

T0/T1（每帧 TRAC-IK continuation）快但穿透（82–89% collision）；T4（在线 reshape）是唯一安全杠杆但
34–40 ms ≈ 2× deadline。T5 把「安全」从在线挪到离线：**离线构造连续安全构型流形
$q^\star_g(\theta,s):\mathbb{R}^2\to\mathbb{R}^{14}$**，在线只做 `lookup → 双线性插值 → 1–2 步固定 Newton
投影 → q_cmd`，全程**无每帧 TRAC-IK**。

对象是一张 101 θ × 65 s = 6565 点的网格（θ∈±50°，s∈[−0.16, 0] m，gap=0.51 m），每格存一个 14-D 构型。
关键不是「每格独立 argmax clearance」（会在格间跳分支，见下），而是**受约束的平滑剖面选择**：
`safe = d≥d_safe` 且 `margin = min_j min(q_j−q_min, q_max−q_j) ≥ m_q`（m_q=0.03 rad），
在满足两约束的构型里取 margin 最大者。这直接回应 §9-10 的「不要存 per-grid argmax」警告。

### 三问的答案（margin-respecting safe 剖面，`/tmp/aviator-manifold-safe/`）

| 问题 | 结果 |
|---|---|
| Q1 连续性 max‖qⁱ⁺¹ʲ−qⁱʲ‖ | θ 邻 0.194 rad，s 邻 0.185 rad（argmax 剖面 0.086 rad 更光滑但违 margin） |
| Q2 双线性插值任务流形误差 e₀ | 中位 11 µm，max 0.37 mm（argmax 剖面同为 11 µm） |
| Q3 固定 1/2 步 Newton 投影 | e₁ 中位 2.2e-10 m，P(e₁<1e-4)=97.9%；e₂ 中位 2.6e-16，P(e₂<1e-5)=97.88% |
| 安全保持 | d₀ 中位 8.05 mm，P(d₂≥d_safe)=100% |
| 剖面可达性 | 6418/6565 = 97.76% 点存在 safe+margin 构型 |

**三种剖面逐格对比（关键实验）**：

1. **argmax 剖面**（每格 max clearance）：连续（max Δq 0.086 rad）、可插值（11 µm），但**最优构型贴着关节限位
   边界**——Newton 步本身是精确的（1 步闭链 3.8e-11），却因关节限位 clamp（margin 0.03）把解**弹回 0.03 rad**，
   |q_out−(q+dq)|=0.03 精确等于 margin ⇒ 违反 §9 的 joint-margin 约束。
2. **continuation 剖面**：有分支跳变（max Δq 0.459 rad）且**不安全**（d₀ 中位 −11.1 mm）。
3. **safe 剖面**（本文采用）：三问全部干净通过，Q3 的 ~2% 失败精确对应「该点无 safe+margin 构型」的
   静态不可行点（= §8/`dg-static-feasibility-certificate` 的 D_g<d_safe 区域），**不是投影失败**。

### 结论

T5 的三问在 margin-respecting safe 剖面上全部成立：连续（Δq≈0.19 rad）、可插值（11 µm）、
1–2 步 Newton 即可恢复任务约束（亚 nm 级）且全程保持 d≥d_safe。argmax 剖面的「贴关节限位」是
把「max clearance」换成「sufficiently safe + margin + smooth」的直接动机，与 §9-10 的警告完全一致。

### 本次修掉的 harness 三类 bug

1. **`write_f32` 截断**：按 `N` 而非 `v.size()` 写字节，qLmax/qRmax（N×7 float）被截成 N float。
2. **`std::cout` sticky 标志污染**：进度打印的 `std::fixed<<setprecision(0)` 让后续所有浮点输出
   四舍五入到 0 位小数，「max 0 rad」实为 0.086 rad。
3. **帧/符号错**：闭链误差须用「TCP site vs handle site」（grasp closure，匹配 `task_error()`），
   不能用 `target()`（返回 flange 帧，含 ~100 mm tool 偏移）；Newton 用 `solve(+e)`（e=handle−tcp），
   `solve(−e)` 会把 TCP 推离目标。

### 复现命令

```bash
./bin/aviator_clearance_trajectory manifold_safe /tmp/aviator-manifold-safe 16
# 输出 manifold_qL.bin / manifold_qR.bin / dcont.bin + reachable/has-safety 分数 + Q1 连续性摘要
```

### 下一步

- 在线固定预算 manifold tracking（lookup + 1–2 Newton）对 T0/T1/T4 的 timing+safety 对比（§15 复现 harness 复用）。
- 若要把 97.76% 可行性往上推：§11 的全局平滑剖面选择
  $\min\sum[\|q_{i+1,j}-q_{ij}\|^2+\|q_{i,j+1}-q_{ij}\|^2]$ s.t. FK、d≥d_safe、joint limits（offline 一次性）。

---

## 17. T5 在线对比：safe-manifold tracking vs T0/T1/T4（§19 动态验证）

### 方法

在 `compare` 模式加入 **method 3 = T5**：每周期 `lookup(θ,s) → 双线性插值 → 2 步固定 damped-Newton`，
**无每帧 TRAC-IK**。四个方法重放同一条预生成轨迹流（`sine` 0.2 Hz / `roll_pull` 0.2 Hz / `random`
5 分量扫频），50 Hz、N=10000 周期，gap=0.51 m、d_safe=5 mm、q_margin=0.03。T5 的 Newton 步用
**damped least-squares**（λ=1e-2）+ **真实关节限位 clamp**（见下「关键 bug」）。

### 结果（N=10000）

| profile | 方法 | P50 ms | P99 ms | max ms | miss% | d_min mm | coll | e_task mm |
|---|---|---|---|---|---|---|---|---|
| sine | T0 TRAC-IK | 0.775 | 1.59 | 1.75 | 0 | −15.5 | 8875 | 0.00 |
| | T4 reshape（active）| 34.6 | 38.2 | 67.6 | 100 | 3.24 | 0 | 0.07 |
| | **T5 manifold+Newton** | **0.069** | **0.085** | **1.23** | **0** | **5.05** | **0** | **0.00** |
| roll_pull | T0 | 0.751 | 1.57 | 2.50 | 0 | −15.3 | 8593 | 0.00 |
| | T4（active）| 34.3 | 37.2 | 58.5 | 100 | 4.62 | 0 | 0.02 |
| | **T5** | **0.072** | **0.087** | **0.121** | **0** | **5.13** | **0** | **0.00** |
| random | T0 | 0.792 | 2.34 | 17.3 | 0 | −17.7 | 8228 | 0.00 |
| | T4（active）| 39.1 | 76.8 | 84.4 | 100 | −17.7 | 7668 | 0.14 |
| | **T5** | **0.070** | **0.096** | **0.150** | **0** | **5.04** | **0** | **0.00** |

（T1 = T0 + collision detect，时序 0.90–0.94 ms，安全指标与 T0 相同。）

**T5 单周期成本分解**：bilinear lookup ≈ 0.07–0.21 µs（可忽略）；2 步 Newton ≈ 71 µs（含
`mj_forward`+`mj_jacSite`+6×6 LDLT）。合计 ~0.07 ms = 20 ms deadline 的 **1/280**。

### 结论

T5 是唯一在**三个 profile 上全部**做到 `d_min ≥ d_safe`、`coll=0`、`e_task=0` 的方法，且 P99
≈ 0.09 ms（miss 0%）。对照：

- T0/T1 快（0.75–0.94 ms）但穿透 −15…−18 mm（coll 82–89%）。
- T4 是唯一能消碰撞的旧方法，但 34–40 ms ≈ 2× deadline，且 **random 上仍穿透**（−17.7 mm，
  coll 7668）——rate/accel-limited 的在线 reshape 追不上快速不规则运动。

这正是 §16 命题的在线版：**昂贵的 IK branch 选择与冗余搜索搬到离线，在线只留 ~71 µs 的局部
manifold 投影**。T5 把「安全」从每帧在线搜索变成一次离线证书，把 T4 的 34–40 ms 砍到 0.07 ms。

### 已知限制

- **关节速度/加速度尖峰**：T5 的 `max|q̇|` 由（Q1 连续性 0.19 rad）/（dt=0.02）≈ 9.5 rad/s 封顶
  （random 实测 11.19），`max|q̈|` 775 rad/s²。这是**网格格间跳变**伪运动（T0 的 468 rad/s² 是 IK
  分支跳变伪运动，同类），非物理运动。sine/roll_pull 上 T5 的 max|q̇|=3.47/2.74 < 6.28 rad/s。
  正式版用 §11 全局平滑剖面选择或速度前馈 `q̇_ff = J_Q(x)ẋ` 压低。
- **margin 软化**：97.76% 域保证 margin≥0.03；剩余 2.24% margin-infeasible 点（无构型同时满足
  d≥d_safe 与 margin≥m_q）回退到 max-clearance 构型，margin<0.03 但仍在真实限位内。

### 关键 bug（T5 第一版 → 本版）

第一版在线 T5 把 Newton 步 clamp 到 `[lower+m_q, upper−m_q]`，在 2.24% margin-infeasible 回退点把
插值构型**向内 snap ~0.02 rad**，导致 (a) 闭链误差爆到 13 mm，(b) 被 snap 的位姿撞到机身
（coll 33/200）。先试的 damped LS 只压住了步长、**没动 max**（证明发散源是 clamp 而非奇异性）；
真正的修法是 **clamp 到真实限位 [lower, upper]**——margin 是离线剖面选择保证的软约束，不该在线
用硬 clamp 重复强制。修后 e₁ max 13 mm→1.03 µm、coll→0、e_task→0.00 mm。

### 复现命令

```bash
# 1) 离线：build_manifold_safe 到同一目录（含 atlas_Dg/feasible）
# 2) 在线对比（4 方法重放同一轨迹流）：
./build/bin/aviator_clearance_trajectory config/aviator.yaml /tmp/aviator-t5-compare \
    0.005 0.03 10 0.1 compare 10000
# 输出 compare_timing.csv / compare_safety.csv / compare_substeps.csv
```

---

## 附录：数值速查

- $d_{\rm base}(g) = 0.5g - 0.267751$；$d^*_{\rm static}(g) = 0.5g - 0.246053$（continuation-ρ，绑定臂）。
- $g_{\rm B0}=0.5355$，$g_{\rm safe}^{\rho}=0.5021$，$g_{\rm geom}^{\rho}=0.4921$，$g_{\rm exec}^{\rm B3}\approx 0.504$。
- 自运动救援 = 21.70 mm（常数）⇒ 43.4 mm 间隙回收。
- φ gain（roll_neg side 0）= 0.004024856 m（常数）；roll_pos side 1 = 0.0152633 m（非绑定）。
- home $d^*_{\rho}=0.5g-0.250130$；home $d^*_{\rho+\varphi}=0.5g-0.246105$；R1/R2/R3 $=0.5g-0.246092$。
- 释放层级 $g_{\rm geom}$ 全部 = 0.4921；R1→R3 增益 0.0123 mm（伪差）；所有最优点 β=0。

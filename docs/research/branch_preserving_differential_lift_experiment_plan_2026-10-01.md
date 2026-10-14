# Branch-Preserving Differential Lift：AVIATOR 实验流程与论文大纲

日期：2026-10-01  
代码分支：`grasp-conditioned`  
对应论文草稿：`/home/rocos/sia/tsr/aviator_branch_preserving_differential_lift_draft.tex`

本文把论文草稿中的主张与 AVIATOR 分支里已经完成的仿真、静态间隙实验、B0/B3/B4 在线实验、T5 safe-manifold 实验和固定二维表/RLPD 实验对齐。它是实验执行协议，不把尚未完成的数值写成结果。

---

## 1. 先固定论文究竟要证明什么

### 1.1 论文主旨

论文的核心不是“再做一张更好的查表”，也不是“RL 比查表好”。更准确的主张是：

> 在双臂刚性抓取、低维任务运动和每臂一维自运动的场景中，可以把当前 IK 分支上的任务运动与自运动写成一个在线的 branch-preserving differential lift；再把关节限位、碰撞净空和相位动作约束投影到二维自运动速度空间，并在冗余不足时降低任务速率。

因此，中心比较是：

\[
\text{固定几何上的全局 LUT + 在线查表}
\quad\text{vs.}\quad
\text{当前分支上的局部在线 lifting + safety filter}.
\]

两者使用相同的任务目标、抓取、机器人模型、障碍物和高层二维相位动作接口。查表在固定场景中的延迟很可能更低，这不是需要回避的负结果；论文要回答的是在线方法能否在分支连续性、安全和任务/环境变化下减少全局重建依赖。

### 1.2 四条可检验的主要贡献

| 主张 | 必须看到的证据 | 不能用什么替代 |
|---|---|---|
| **C1 几何正确性**：lift 给出正确的任务切向速度，并能在有限步长下保持闭链 | 雅可比/零空间与独立有限差分、高精度参考的误差；步长收敛图；任务残差 | 只报告求解器返回 success |
| **C2 分支连续性**：连续积分比逐点 IK/普通投影更少发生分支跳变 | 同一初态、同一任务段、同一预算下的 branch-jump、完整连接率、恢复次数 | 只比较平均运行时间 |
| **C3 安全与恢复**：二维 phase filter 能处理关节/净空约束，必要时由 governor 降低任务速率 | 独立碰撞检查、最小净空、任务速率缩放、失败原因和停止/恢复统计 | 只检查采样点合法 |
| **C4 适应性**：抓取、障碍物或任务轨迹改变时无需重建全局表即可工作 | 未见场景的成功率、重建时间/内存/失败网格数、在线更新开销 | 把固定旧表直接复用的失败归因于 RL |

物理实验还可支持“能否在实际跟踪和接触负载下执行”，但不能把阻抗/导纳控制或力控制包装成本文的算法贡献。

### 1.3 明确不声称的内容

- 不声称在所有任务上全局完备，也不声称穿越奇异点或连接所有 IK 分支。
- 不声称在线完全没有投影；有限步验证失败时允许单独计数的恢复投影或步长回退。
- 不声称 T5、B4 的结果就是本文方法。T5 是“离线安全流形 + 在线查表/Newton”，B4 是 position-level 预测重塑；它们是重要的工程基线和动机证据。
- 不声称“释放更多末端姿态自由度一定有价值”。当前 AVIATOR 研究已经显示，在现有侧向狭窄边界和操纵盘任务上，R1/R2/R3 没有把几何边界继续推窄；这个负结果应作为限定范围的消融。

---

## 2. 当前分支证据盘点

### 2.1 可以直接放进论文的已有证据

| 现有结果 | 已经支持的结论 | 仍缺什么 |
|---|---|---|
| 静态 B0/B1/B3 间隙扫描 | 基线 continuation 到静态最优自运动存在约 21.70 mm 的净空救援；`roll_neg` 是绑定任务；$g_{B0}\approx0.5355$、$g_{safe}^{\rho}\approx0.5021$、$g_{geom}^{\rho}\approx0.4921$（$d_{safe}=5$ mm） | 这是静态冗余价值，不是在线 differential lift 的分支存活证据 |
| B0/B3/B4 在线频率实验 | 在 gap=0.54 m、$d_{safe}=5$ mm 的已有设置中，B0 安全频率为 0，B3 约 0.10 Hz，B4 约 0.30 Hz；短时域提前重塑比纯 reactive 重塑更有余量 | 需要统一动态约束，修正 continuation 分支跳变污染的速度/加速度统计，并加入本文的 velocity-level lift |
| T5 safe-manifold tracking | 固定 profile 上可做到闭链误差近零、碰撞为零和约 0.07 ms 的 command-generation 时间；说明“把分支/冗余选择离线编译”是一个强工程路线 | T5 依赖离线安全剖面，不等于在线 branch-preserving lift；random profile 仍有陡峭 section、速度/加速度问题，且 5.04 mm 的净空缓冲太薄 |
| 固定二维表 vs RLPD | 固定表是强基线；高速随机任务下必须独立检查速度、加速度、净空和接触，不能只看 RL reward | 不能由此证明 RL 或本文 lift 的分支连续性；还需共同初态和接入段 |
| grasp-conditioned 速度/换向 protocol | 已有 1200 条训练、每格 100 条 val/test 的数据生成方案，速度和换向维度可复用 | `steps1m` 训练结果尚未成为最终测试结果；当前协议的 `qdot_0=0` 接入边界还没有被证明 |

已有静态间隙、B4、T5 的详细记录见：

- [AVIATOR clearance study](./AVIATOR_clearance_study_2026-09-20.md)
- [固定二维表与 RLPD 高速对照](./static_q_vs_rl_speed_sweep_2026-09-30.md)
- [速度/换向 1200×64 protocol](./speed_reversal_1200_64_protocol.md)
- [固定二维关节角表与 RLPD 初步审计](./static_q_field_vs_rl_2026-09-29.md)

### 2.2 必须标成“尚未证明”的地方

1. **当前 `branch` 字段不等于物理 IK 分支身份。** 现有工具中的部分 branch 值是网格排除/有效性标记；论文要报告 branch preservation，必须增加独立的连续分支描述，例如预测关节增量误差、肘部 swivel 角或一个经过验证的局部 branch label。
2. **T5 的 `d_min=5.04` mm 只有约 0.04 mm 缓冲。** 仿真中要增加离线鲁棒净空 `d_atlas=d_safe+d_robustness`，真机的 5 mm 才有意义。
3. **表/RL 的旧 checkpoint 只能配旧流形。** `manifold_phi_stale_pitch-10deg` 与后来重建的 `manifold_phi` 不能混用；所有最终对照必须记录流形目录、哈希、模型版本和 checkpoint。
4. **现有速度统计有初始速度边界问题。** 从静止 `qdot_0=0` 到工作段的接入轨迹必须单独规划并计入成功率。
5. **点和中点检查不是连续时间碰撞证明。** 论文应明确检查分辨率；如要声称更强保证，需要速度/加速度界或连续碰撞验证。

---

## 3. 总体实验矩阵

### 3.1 方法集合

正式比较建议分成三层，避免让每个方法回答所有问题。

**几何/连接层（主实验）**

1. `LUT-native`：现有 phase-consistent LUT、插值、phase polygon、有限步回退和 damped-Newton。
2. `Pointwise IK`：每个任务点以上一时刻关节角 warm-start 的非线性 IK。
3. `DLS/QP differential IK`：标准任务雅可比伪逆/阻尼最小二乘和传统 null-space 目标。
4. `Implicit projection`：先积分或插值，再用 Newton/伪逆投影。
5. `Proposed lift`：本文 horizontal lift + oriented null basis + task-error feedback + midpoint + adaptive rejection + phase filter/governor。

**AVIATOR 工程层（诊断基线）**

- `B0`：continuation IK，不做冗余重塑。
- `B3`：当前净空不足才进行 position-level 最小干预重塑。
- `B4`：使用短时域预测提前重塑。
- `T5`：离线 safe-manifold section + 在线 lookup/Newton。

这些方法的目标不同：B3/B4 用于解释实时冗余重塑和预测的价值，T5 用于比较“离线编译安全分支”的代价与收益；不能把它们的优势直接写成本文 lift 的优势。

**策略接口层（单独报告）**

- 固定同一个 actor 输出的 $r_{nom}$，分别经过 LUT 和本文 lift；或把相同 action trace replay 到两个 backend。
- RLPD 结果只作为高层控制器上下文。若重新训练，必须把 observation、reward、seed 和在线转移预算独立列出，不能与确定性 connector 结果混在一起。

### 3.2 所有方法必须统一的条件

- 同一个 AVIATOR 模型、抓取几何、墙体/障碍物和碰撞检查器。
- 同一个任务轨迹 $x(t)=(\theta(t),s(t))$、初始 $(q_0,x_0,\dot q_0)$ 和接入轨迹。
- 同一个控制周期、关节速度/加速度/限位、$d_{safe}=5$ mm 和任务误差阈值。
- 同一个随机任务集合；train/development、validation、held-out test 分开，最终测试前冻结参数。
- 所有轨迹同时用采样点和相邻关节空间中点做独立检查；报告检查分辨率。
- 记录每种方法的 IK、Jacobian、`mj_forward`、距离查询、QP、projection、step rejection 和 wall-clock 次数。

---

## 4. 可执行实验流程

### Phase 0：证据和环境冻结（先做）

**目的：** 保留旧结果，避免流形、模型和检查器混用。

1. 对以下目录生成只读 manifest 和 SHA256：
   - `hil-serl/data/aviator/manifold_phi_stale_pitch-10deg`
   - `outputs/static_field_audit`
   - `outputs/speed_reversal_grid_v3_1200`
   - `outputs/t5_pose_slack_200hz_20260922`
   - B0/B3/B4 在线 CSV 和静态 clearance 汇总。
2. 记录 Git commit、分支、MuJoCo 版本、URDF/MJCF、抓取文件、Python 环境和编译器。
3. 固定成功判据：完整任务、闭链误差、净空、关节限位、速度/加速度、数值有效性分别判定，失败原因不合并成 timeout。
4. 补做共同初态接入：从同一个实测或合成 $q_0,\dot q_0=0$ 到每个 backend 的工作点，接入段也检查碰撞、净空、速度和加速度。

**产物：** `experiment_manifest.json`、`frozen_parameters.yaml`、`failure_taxonomy.md`、共同初态数据集。

### Phase 1：解析与数值正确性

**目的：** 证明“lift 算对了”，不把实现正确性和性能混为一谈。

**场景：**

- 7DOF 单臂固定抓取的局部 AVIATOR 状态；
- 一个有解析解的平面冗余臂单元测试，用 elbow sign 作为真实 branch label；
- AVIATOR 的 regular、near-limit、near-clearance、near-singular 四个分层。

**比较：** 中心差分/高精度 reference、Euler、midpoint、本文 lift；记录 `A_q`、`A_x`、$H$、$N$、null-basis orientation。

**指标：**

- Jacobian 和 lift 误差的 median/IQR/P95/worst；
- 任务误差对 $\Delta t$ 的 log-log 收敛；
- null vector 与表中局部差分的夹角；
- rank、$\sigma_{min}$、反馈项大小和拒步率。

**通过标准：** 在预先声明的 regular 区域误差随步长按预期下降；near-singular/branch-inconsistent 状态单独计数，不静默删掉。

### Phase 2：局部连接和 branch preservation（论文的第一主实验）

每个 trial 从同一 $(q_0,x_0)$ 出发，执行相同的任务段和相同的 $r_{nom}$。先不放障碍，再加入窄通道。

**方法：** LUT-native、pointwise IK、DLS/QP、implicit projection、本文 lift；另做本文组件消融。

**分层：** 距关节限位、距障碍物净空、$\sigma_{min}(A_q)$、任务段长度、phase 变化速度、开弧/闭环 self-motion。

**必须实现的 branch 指标：**

- 平面单元：elbow sign；
- AVIATOR：连续 elbow-swivel descriptor 或经过独立验证的局部 branch label；
- 预测增量误差：$\|q_{k+1}-q_k-\hat{\dot q}_k\Delta t\|$；
- branch jump 前后任务误差和关节增量。

**主要指标：** 完整连接率、branch-jump rate、到失败点的进度、最大平移/转动误差、最大关节增量、最小关节 margin、最小净空、IK/projection/recovery 次数、P50/P95/P99 延迟。

**关键判断：** 本文方法只有在相同输入、初始化和数值预算下 branch survival 或完整连接率有稳定增益，才能声称分支保持贡献；若只有接口更整洁或运行更快，应收紧贡献表述。

### Phase 3：AVIATOR 名义轨迹复现

**目的：** 检查本文 lift 能否复现成熟 LUT 的行为，而不是要求它在固定场景中击败查表速度。

**轨迹：**

- `roll_pos`、最难的 `roll_neg`、`pull`；
- roll、pull、roll+pull 三种任务 profile；
- gap 取宽、中、接近 $d_{safe}$ 的三个等级；
- 同一双臂刚性抓取、同一 phase trace 和接入姿态。

**报告：** 左右臂关节轨迹、任务误差、phase action、净空、branch descriptor、任务完成率、failure cause、offline construction time、online latency。

**已有结果的使用方式：** 静态研究中的 21.70 mm 冗余救援和 $g_{B0}/g_{safe}^{\rho}/g_{geom}^{\rho}$ 作为场景选择和 motivation；不要把它们当成本文 online lift 的结果。

### Phase 4：安全 filter、governor 和动态包线（论文的第二主实验）

**目的：** 证明 phase filter 不只是输出一个形式上合法的二维动作，而能在真实约束下维持任务或明确降速/停止。

**第一轮：复现并清理已有 B0/B3/B4。**

1. 修正 continuation IK 分支跳变对离散 $\dot q,\ddot q$ 统计的污染。
2. 对 B3/B4 分别报告 `q_base` 变化、零空间干预变化和真实执行变化。
3. 把任务速度、任务加速度、任务 jerk、关节速度、关节加速度统一列为硬约束。
4. 使用 `f\times` reversal grid，但在同一速度格内分析换向次数，不能把两者解释为独立因素。

**第二轮：加入本文 lift。**

- 相同 task-rate profile 和相同 $r_{nom}$，比较 LUT-native、B3、B4、本文 filter/governor；
- governor 逐步缩小 $\beta$，记录可行 $\beta$、停止阈值、拒步、恢复和最终完成率；
- 同时做“只过滤 phase、不 governor”和“只 governor、不碰撞梯度”的消融。

**主要图：** 速度×换向次数成功率热图、$\beta(t)$、最小净空、任务误差、branch descriptor 和 latency 的时间序列。

**已有 B4 结果的定位：** B0=0、B3≈0.10 Hz、B4≈0.30 Hz 的结果是“提前重塑有用”的强动机；本文要证明的是 differential lift 的统一 tangent/safety 表达，不应直接把 B4 的 3× 写成本文增益。

### Phase 5：变化任务和环境下的适应性（论文的第三主实验）

从原 LUT 构建条件逐项施加变化，每一项使用 held-out 场景：

1. 左右墙平移和 gap 改变；
2. 抓取位姿小幅平移/转动校准误差；
3. 任务轨迹不落在原 $(\theta,s)$ 网格；
4. 任务速度、换向和 roll/pull 比例改变；
5. 初始自运动 phase 改变；
6. 表构建后加入局部障碍物。

**比较：**

- 旧 LUT 直接复用；
- 重新构建 LUT；
- 本文在线 lift；
- 若可实现，再加 pointwise IK 作为不依赖 LUT 的参考。

**指标：** success、失败进度、branch jump、净空和任务误差；LUT 重建时间、存储、失败网格数、人工干预；本文在线初始化和每周期开销。

**注意：** 若变化超过局部 chart 的有效范围，本文应输出 governor stop/retract，而不是把“安全拒绝”记成算法错误。应报告可工作范围和停止原因。

### Phase 6：高层策略接口隔离

使用同一个 actor 或一组固定 action trace，使高层策略只输出

\[
\pi(o_t)\mapsto r_{nom,t}\in\mathbb{R}^2.
\]

把相同 action trace 分别送入 LUT backend 和本文 lift backend。这样能回答“收益来自 motion-generation layer 还是 RL policy”。

RLPD 的 1200×64 protocol 可复用，但须做到：

- 旧 checkpoint 只配 `manifold_phi_stale_pitch-10deg`；
- `dp_extra`、`dp_static` 的训练预算和 seed 一致；
- 最终 test 只在 val 选 checkpoint 后运行；
- 固定表、RL、本文 lift 的共同初始姿态和接入段一致；
- 独立 MuJoCo 检查加上同一速度/加速度硬限制。

这部分作为接口实验或附录，不能替代 Phase 2 的 branch connector 主实验。

### Phase 7：物理验证（有硬件数据后再写成结果）

建议四类 trial：

1. free-space transport；
2. nominal guided insertion；
3. 校准或墙体小扰动；
4. 带柔顺夹具的受控接触。

记录双臂 $q,\dot q$、工具位姿、闭链误差、净空估计、phase action、$\beta$、六维 wrist wrench、motor current、internal-force proxy 和 controller latency。

成功必须同时满足：任务完成、姿态/闭链误差、净空/限位、速度/加速度、wrench/current process limit。几何成功但力或电流超限，单独记为 contact-force failure。

---

## 5. 消融实验

每次只去掉一项，其他参数冻结：

1. 去掉 task-error feedback；
2. midpoint 换成 Euler；
3. 禁用 null-basis sign/orientation continuation；
4. 用普通 joint-centering 替换 phase-space filter；
5. 去掉 collision-gradient half-spaces，只保留 finite-step rejection；
6. 去掉 governor；
7. 改变 null-basis 的归一化和速度权重；
8. 不使用 branch-conditioned metric $G_H$，只用 task-space metric；
9. 离线 safe-manifold section 与本文 online lift 的对照。

每个消融至少报告：任务残差、branch jump、完整连接率、最小净空、$\beta$、拒步/恢复、延迟和调用次数。消融的目标是定位机制，不是制造一个所有指标都变差的图。

---

## 6. 统一数据结构与统计规则

每个 trial 保存：

```text
metadata: git_commit, model_hash, grasp_hash, manifold_hash, seed, method
task: x, xdot, xddot, profile, gap, obstacle_id
state: q, qdot, qddot_est, task_error, branch_descriptor
safety: clearance, joint_margin, collision, beta, phase_polygon, stop_reason
compute: wall_time, jacobian_calls, fk_calls, distance_calls, ik_calls,
         qp_calls, projection_calls, rejected_steps
outcome: completed, progress, failure_cause
```

统计规则：

- 报告 trial-level 成功率及 Wilson 或 bootstrap 置信区间；
- 配对轨迹报告双方独有成功、共同成功、共同失败；
- 速度×换向×gap 分层，不把不同难度混成一个平均数；
- 同时报告 median、IQR、P95、worst-case；
- 失败原因至少分为 task drift、branch jump、joint limit、clearance/collision、velocity、acceleration、numerical rejection、governor stop、contact force；
- 参数和阈值在最终 test 前冻结，开发集调参过程写进日志。

---

## 7. 论文正文大纲

### 1. Introduction

1. 双臂刚性抓取任务中的低维任务运动与高维冗余；
2. 全局 LUT 的三个限制：构建成本、环境绑定、分支与插值问题；
3. 本文问题定义：保留二维 phase-action interface，改为当前分支上的在线 differential lifting；
4. 四条贡献和明确范围。

### 2. Related Work

- task-space regions 与约束流形规划；
- differential IK、null-space control 和 branch continuation；
- safety filter/CBF/低维 action projection；
- 双臂装配、刚性抓取和力/柔顺执行。

### 3. AVIATOR Task and Existing LUT Pipeline

1. 双 7DOF、刚性 grasp、$(\theta,s)$ 任务；
2. 原有 phase-consistent LUT 和在线 phase polygon；
3. B0/B3/B4/T5 作为工程背景和基线；
4. 静态 clearance 研究、绑定任务和现有工作包线；
5. branch label、碰撞检查器和数据版本定义。

### 4. Branch-Preserving Differential Lift

1. augmented constraint manifold；
2. horizontal minimum-norm lift；
3. oriented null-basis continuation；
4. $q\dot{}=h+Nr$ 的自运动坐标；
5. task-error feedback、midpoint integration 和 adaptive rejection；
6. 局部 branch 保证的假设和边界。

### 5. Intrinsic Safety Filter and Task-Rate Governor

1. joint-rate、joint-limit、clearance half-spaces；
2. 二维 phase feasible polygon/QP；
3. empty polygon 时的 $\beta$ governor；
4. finite-step independent validation、stop/retract；
5. 与 B3/B4/T5 的关系和差异。

### 6. Experimental Protocol

1. 研究问题 RQ1–RQ5；
2. 方法和公平比较；
3. Phase 0–7 流程；
4. 指标、失败分类、统计和可复现性；
5. 仿真与物理装置的不同成功判据。

### 7. Results

1. derivative/integration correctness；
2. local branch survival；
3. nominal AVIATOR reproduction；
4. filter/governor dynamic envelope；
5. adaptation without LUT reconstruction；
6. policy-interface replay；
7. physical traces and contact-force audit（若数据已完成）；
8. ablation。

### 8. Discussion

- LUT 在固定场景的速度优势；
- 本文方法的优势是局部连续性、在线适应和统一安全接口；
- B4/T5 的结果说明什么，不能说明什么；
- 当前任务中姿态自由度释放的负结果；
- 失败模式、停止策略和工程部署条件。

### 9. Limitations

- local chart、奇异性和全局 branch 不完备；
- SDF/点云和离散验证的误差；
- 速度/加速度和真实控制器跟踪；
- 物理实验的抓取校准与内力测量；
- 未声称 force control 或全局最优规划。

### 10. Conclusion

只总结被实验真正支持的结果：在定义的 regular branch 和 AVIATOR 工作包线内，在线 lift 是否保持任务/分支/安全，以及为此付出的计算和建模代价。

---

## 8. 建议的图表清单

### Figures

1. **系统架构图**：高层二维 action → LUT 或 differential lift → compliant controller；
2. **局部流形图**：horizontal lift、两个 null coordinates 和 branch gauge；
3. **Phase 2 branch benchmark**：不同方法的分支轨迹、branch jump 和任务误差；
4. **Nominal AVIATOR**：左右臂关节、任务误差、净空和 phase action；
5. **Dynamic envelope**：速度×换向×gap 成功率热图与 $\beta(t)$；
6. **Adaptation**：墙体/抓取/轨迹扰动下的成功率和 LUT 重建代价；
7. **Ablation**：去反馈、去 gauge、Euler、去 governor 的机制分解；
8. **Physical validation**：位姿、净空、wrench/current 和失败 trial 时间序列。

### Tables

1. 平台、任务、抓取和安全参数；
2. 基线、输入信息和公平性；
3. Jacobian/lift 误差与步长收敛；
4. branch survival 和完整连接率；
5. 动态安全、延迟和调用次数；
6. 适应性和 LUT reconstruction cost；
7. failure taxonomy；
8. 物理 trial 汇总。

---

## 9. 实际执行顺序

### 第一轮：先判断方法是否有可区分增量

1. 完成 Phase 0 版本冻结和共同初态接入；
2. 实现 Phase 1 的 lift/reference 单元测试；
3. 完成 Phase 2 的 branch descriptor 和五方法 replay harness；
4. 只在一个 AVIATOR 窄通道场景做小规模 pilot。

若 Phase 2 在公平输入下没有 branch 或完整连接增益，就不要立刻扩大硬件实验；先修改方法或收紧贡献表述。

### 第二轮：验证 AVIATOR 的主要机制

1. 复现并修正 B0/B3/B4 的动态指标；
2. 将本文 lift 接入同一 task/reversal grid；
3. 完成 Phase 3 名义轨迹和 Phase 4 governor；
4. 先冻结一个规划器、一个 gap、一个正式 test split，再扩大到多场景。

### 第三轮：证明论文的实际价值

1. 做 Phase 5 的墙体、抓取和轨迹扰动；
2. 做 Phase 6 的相同 action trace replay；
3. 对所有结果做独立 MuJoCo 审计和 paired statistics；
4. 只有有真实记录后才写 Phase 7 物理结论。

---

## 10. 最终论文判断标准

论文可以成立的最小证据链是：

1. lift 的几何和有限步误差经过独立 reference 验证；
2. 在相同任务和初始化下，本文方法相对 pointwise IK/普通 differential IK 的 branch continuity 或完整连接率有可重复改善；
3. phase filter/governor 的安全结果经过独立碰撞和动态约束审计；
4. 至少一个任务/障碍/抓取变化显示旧 LUT 需要重建，而本文方法可以在局部有效范围内继续运行或明确安全停止；
5. 运行代价、失败模式和适用边界都如实报告。

如果最终只有 T5 的固定环境低延迟结果，而 Phase 2 的 branch experiment 没有增益，那么论文应改写成“safe-manifold compilation/tracking”方向；如果只有 B4 的频率提升，则应写成“predictive redundancy reshaping”。这两种结果都可能有价值，但它们不是当前草稿所定义的 branch-preserving differential lift 论文。

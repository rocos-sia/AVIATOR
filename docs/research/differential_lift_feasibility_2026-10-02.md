# Differential Lift 低成本可行性验证

日期：2026-10-02  
分支：`grasp-conditioned`  
验证脚本：[validate_differential_lift.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_differential_lift.py)  
解析 Jacobian 检查：[validate_analytic_jacobian.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_analytic_jacobian.py)  
数值报告：[report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/report.json)
  
online lift 原型报告：[online_lift_analytic_report_v2.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/online_lift_analytic_report_v2.json)；解析 Jacobian 报告：[analytic_jacobian_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/analytic_jacobian_report.json)
长轨迹 connector 脚本：[validate_branch_connector.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_branch_connector.py)；swept-check 100 步报告：[branch_connector_swept_100_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_connector_swept_100_report.json)；swept-check 200 步报告：[branch_connector_swept_200_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_connector_swept_200_report.json)；最小进度 governor 报告：[branch_connector_governed_minprogress_200_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_connector_governed_minprogress_200_report.json)；阈值敏感性报告：[branch_connector_governed_minprogress025_100_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_connector_governed_minprogress025_100_report.json)；governor 200 步报告：[branch_connector_governed_200_v2_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_connector_governed_200_v2_report.json)
paired baseline 小批次报告：[paired_baseline_stress_4x100.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/paired_baseline_stress_4x100.json)；governed paired baseline 报告：[paired_baseline_governed_4x100.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/paired_baseline_governed_4x100.json)；8-trial 扩展报告：[paired_baseline_governed_8x100.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/paired_baseline_governed_8x100.json)
多初值 IK continuation atlas 报告：[global_branch_descriptor_cont_report.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/global_branch_descriptor_cont_report.json)；密集 \(\theta,s,\phi\) 延拓报告：[branch_atlas_extension_state6_final.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_atlas_extension_state6_final.json)；大范围 BFS atlas：[branch_atlas_extension_state6_bfs_wide.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/branch_atlas_extension_state6_bfs_wide.json)；atlas 敏感性报告：[global_branch_descriptor_sensitivity_default.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/global_branch_descriptor_sensitivity_default.json)、[global_branch_descriptor_sensitivity_strict.json](/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261002/global_branch_descriptor_sensitivity_strict.json)

## 目的

这轮测试只验证草稿中最基础的公式和接口是否可行：

\[
\dot q = Q_x(x,\phi)\dot x+Q_\phi(x,\phi)\dot\phi.
\]

检查内容包括：

1. 旧 grasp-conditioned 流形中的 (Q_x,Q_\phi) 是否能由局部有限差分近似；
2. 用上述 (dot q) 和任务速度同时推进任务坐标与关节时，双臂 TCP/handle 闭链误差的一阶变化是否接近零；
3. midpoint 积分是否能跟踪直接的 (Q(x,\phi)) 查询；
4. 现有二维 phase safety filter 在真实流形数据上是否能返回满足有限步速度和净空条件的动作。

这不是完整的 branch-preserving differential-lift 实现，也不是全局安全证明。

## 使用的数据和模型

- 流形：`hil-serl/data/aviator/manifold_phi_stale_pitch-10deg`；
- MuJoCo 模型：`AviatorRobot/build/bin/model/aviator.xml`；
- 采样：随机种子 7，48 个 regular、branch-consistent 内部状态；
- midpoint：(Delta t=0.002) s，每条 80 步；
- phase filter：另用随机种子 11/12 做各 200 个单步检查，(Delta t=0.01) s。

## 结果

### 1. 流形查询和有限差分

48 个采样点均能生成有效状态。存储导数与独立中心差分的差异为：

| 检查 | median | P95 | max |
|---|---:|---:|---:|
| (Q_x) 总体 | 0.0145 | 0.1089 | 0.1651 |
| (Q_x) 的 (	heta) 列 | 0.0038 | 0.1089 | 0.1651 |
| (Q_x) 的 (s) 列 | 0.0133 | 0.0591 | 0.0780 |
| (Q_\phi) | 0.00155 | 0.00479 | 0.0403 |

单位分别为 rad/rad、rad/m 和 rad/rad。这个结果说明当前表中的导数可以用于局部 lifting，但它们不是严格解析导数；正式实验需要把导数误差作为数值误差来源报告，并区分网格插值误差、float32 存储误差和局部 branch/网格边界效应。

### 2. 独立 MuJoCo 闭链切向残差

对每个采样状态，用 MuJoCo 独立计算 TCP 与 handle 的位置误差和旋转 log 误差。然后同时推进

\[
x^+=x+\dot x h,\qquad q^+=q+\dot q h.
\]

不同有限差分步长下，闭链误差导数的最大绝对值为：

| (h) | median | P95 | max |
|---:|---:|---:|---:|
| (10^{-4}) | (1.52\times10^{-5}) | (2.08\times10^{-4}) | (2.92\times10^{-4}) |
| (10^{-5}) | (1.53\times10^{-5}) | (2.08\times10^{-4}) | (2.91\times10^{-4}) |
| (10^{-6}) | (1.52\times10^{-5}) | (2.08\times10^{-4}) | (2.91\times10^{-4}) |

位置量和旋转量在这里放在同一个最大范数中，因此单位是 m/s 与 rad/s 的混合量。结果随有限差分步长稳定，说明主要误差不是差分步长造成的。直接从当前旧表查询得到的 TCP/handle 闭链误差中位数为 (3.78\times10^{-5})，最大为 (6.47\times10^{-5})（同样是 m/rad 混合范数）。

### 3. midpoint 短轨迹

48 条轨迹中 47 条完成了 80 步积分；1 条在局部 branch/phase 有效区域边界前停止。对完成的 47 条：

| 指标 | median | P95 | max |
|---|---:|---:|---:|
| 积分 (q) 与直接 (Q(x,\phi)) 的最大差异 | (1.15\times10^{-5}) rad | (6.57\times10^{-4}) rad | (9.93\times10^{-4}) rad |
| 积分后的独立闭链误差 | (3.99\times10^{-5}) | (5.77\times10^{-5}) | (6.79\times10^{-5}) |

这支持“在当前局部 chart 内，midpoint differential integration 可以作为在线 motion-generation 的数值核心”。它还没有证明长轨迹 branch preservation。

### 4. phase safety filter

在真实旧流形上做 200 个随机一步检查，任务速度范围约为

\[
|\dot\theta|\le 0.15\ \mathrm{rad/s},\qquad |\dot s|\le 0.006\ \mathrm{m/s}.
\]

- nominal phase 速度范围约 ([-0.4,0.4]) rad/s：200/200 可行，0/200 被裁剪，最大实际关节速度 0.470 rad/s，最小净空 7.75 mm；
- nominal phase 速度范围约 ([-2,2]) rad/s：200/200 可行，88/200 被裁剪，最大实际关节速度 1.50003 rad/s，最小净空 7.50 mm。

这表明当前 phase-space polygon、有限步回查和 backtracking 在旧流形上可以运行，并且确实会对过大的 phase action 进行约束。这里的 1.50003 rad/s 是 float32 和容差造成的轻微超出，应在正式报告中使用独立审计器和统一容差。

## 当前结论

当前方案在“公式和局部数值接口”层面是可行的：

- (Q_x\dot x+Q_\phi\dot\phi) 能产生接近闭链切空间的关节速度；
- midpoint 积分在短局部轨迹上能跟踪直接流形查询；
- 二维 phase filter 能在真实流形数据上裁剪动作并维持有限步净空/速度条件。

因此可以继续实现正式的 branch-preserving differential lift。

## 最小 online horizontal/vertical lift 原型

随后加入了一个独立的原型脚本 [validate_online_lift.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_online_lift.py)。它没有复用流形中的 (Q_x,Q_\phi) 来构造约束矩阵，而是对 MuJoCo 的双臂 TCP/handle 闭链误差做有限差分，得到 (A_q,A_x)，再求

\[
\dot q=A_q^+(-A_x\dot x-K_e e)+Nr.
\]

这里使用 (W=I)、可切换的解析/有限差分 Jacobian（本轮正式结果使用解析模式）、(K_e=2)，并对两个零空间基向量做相邻时刻的 Procrustes orientation continuation。

在 24 个随机 regular 状态上，每条轨迹执行 80 步，

\[
\Delta t=0.002\,\mathrm{s},\quad
\dot x=(0.08,0.002),\quad r=(0.03,-0.025),
\]

结果为：

| 指标 | 结果 |
|---|---:|
| 完成 80 步的轨迹 | 24/24 |
| (A_q\dot q+A_x\dot x+K_e e) 中位数 | (5.65\times10^{-17}) |
| 上述残差最大值 | (5.46\times10^{-16}) |
| (\sigma_{min}(A_q)) 中位数 | 0.142 |
| 最大闭链误差（混合 m/rad 范数） | (6.84\times10^{-5}) |
| 末端闭链误差中位数 | (2.50\times10^{-5}) |
| 最大关节速度 | 0.0642 rad/s |
| 零空间基相邻对齐最小奇异值中位数 | 0.9999999995 |

把步长增大到 (0.005\) s 后，12/12 条轨迹仍完成，末端闭链误差最大为 (3.07\times10^{-5})。这说明论文中的 (H/N) 分解、误差反馈和局部基向量延续在当前模型上具有可运行的数值原型。

这一步仍然只是结构验证。虽然本轮使用了解析任务 Jacobian，但任务/自运动速度仍较小，因此没有验证实时计算成本、碰撞梯度约束、长距离 branch survival 或速度受限下的 phase governor。

## 解析 (A_q) 检查

为了检查论文中的“exact pose-error Jacobian”能否落到当前模型，使用 `mj_jacSite` 计算 TCP 的线速度/角速度 Jacobian，并将角速度转换到 (R_{TCP}^{T}R_{handle}) 的局部误差坐标系，再与闭链误差的中心差分 (A_q) 比较。20 个 regular 状态的逐点最大绝对差异为：

| 指标 | 数值 |
|---|---:|
| median | (2.11\times10^{-5}) |
| P95 | (2.51\times10^{-5}) |
| max | (2.67\times10^{-5}) |

这给出了从有限差分原型迁移到解析 MuJoCo Jacobian 的可行依据。在线 lift 的 analytic 模式同时使用了任务坐标对 handle 位姿的解析 Jacobian；有限差分模式保留为交叉检查参考。

## 还没有证明的内容

1. 这次使用的是现有 LUT 的 (Q_x,Q_\phi)，不是草稿中的在线 horizontal lift、weighted minimum-norm (H) 和 oriented null basis (N)。
2. 没有与 pointwise IK、DLS/QP differential IK 和 generic projection 做配对比较。
3. 没有独立的物理 branch label；当前 `branch.bin` 不能直接当作 IK 分支身份。
4. 已有局部 descriptor 的 100--200 步跳变统计，但没有跨分支、奇异点和全局拓扑验证。
5. 这轮 branch connector 使用独立离散端点检查，没有连续时间碰撞证明；安全停止也尚未通过 phase governor 转化为稳定的全局可行连接。

## 下一步

下一步应把这个原型整理成一个最小的 `online_lift` backend，输入当前 (q,x,\dot x,\dot\phi_{nom})，输出

\[
\dot q=H\dot x+Nr^*.
\]

先在同一个 48 状态集合上复用本报告的 FK tangent residual 和 midpoint 检查，再加入：

1. phase governor：在预测到边界时降低 \(\dot x\) 或 \(r\)，而不是直接停止；
2. pointwise IK/DLS/QP 对照；
3. 用多初值 IK/碰撞分解验证 descriptor 的全局分支含义；
4. 更高分辨率 swept-volume 或连续时间 clearance 检查；
5. 多场景 100--200 步局部连接与失败原因统计。

只有这一步完成后，才能开始用 B0/B3/B4/T5 和 LUT-native 做正式性能比较。

## 5. 100--200 步 branch connector 对比

新增脚本 `validate_branch_connector.py` 使用解析约束矩阵、连续定向的二维零空间基、独立的局部 elbow-swivel descriptor，以及独立 point-cloud/box clearance checker。每一步先在 \(N r\) 空间内施加关节速度、关节限位和净空线性约束，再用实际 midpoint 端点和 5 个等间隔 swept 插值点回查；若回查失败，对同一个 \(r\) 做有限次 backtracking。这样过滤器认证的候选与实际积分的候选一致。

### 100 步常规速度（3 个相同种子的配对 trial）

参数为 \(\Delta t=0.005\) s、\(\dot x=(0.08,0.002)\)、\(r=(0.4,-0.4)\)。

| 方法 | 完成 | 平均步数 | 最小净空 | 碰撞样本 | 关节限位样本 | descriptor 跳变 |
|---|---:|---:|---:|---:|---:|---:|
| filtered \(N r\) | 1/3 | 68.7 | 5.000 mm | 0 | 0 | 0 |
| unfiltered \(N r\) | 3/3 | 100 | -20.7 mm（中位数） | 345 | 30 | 0 |

过滤器在剩余 trial 中选择安全停止，而不是接受违规步；因此“完成率”不能单独解释为规划成功率，必须同时报告拒绝和安全性。

### 200 步温和速度（3 个相同种子的配对 trial）

参数为 \(\Delta t=0.005\) s、\(\dot x=(0.03,0.0005)\)、\(r=(0.1,-0.1)\)。

| 方法 | 完成 | 平均步数 | 最小净空 | 碰撞样本 | 关节限位样本 | descriptor 跳变 |
|---|---:|---:|---:|---:|---:|---:|
| filtered \(N r\) | 2/3 | 164.0 | 5.000 mm | 0 | 0 | 0 |
| unfiltered \(N r\) | 3/3 | 200 | -0.122 mm（中位数） | 284 | 0 | 0 |

这轮结果支持三个有限范围内的判断：

1. 解析 \(A_q/A_x\) 与 \(N r\) connector 可以执行 100--200 步局部连接；
2. 独立有限步检查确实能阻止当前场景中的净空和关节限位违规；
3. 未过滤的同一 \(N r\) 指令会进入碰撞区域，而过滤版本会裁剪或拒绝该步。

加入 swept 检查后，filtered \(N r\) 的 100 步和 200 步报告中的 `min_swept_clearance_m` 均保持在 5.000 mm 以上，collision samples 均为 0；unfiltered \(N r\) 的 swept 最小净空中位数分别为 -20.7 mm 和 -0.122 mm。descriptor 的最大单步变化约为 0.00094 rad（100 步）和 0.00026 rad（200 步），两组均没有超过 0.5 rad 的跳变。

这仍然不是连续时间碰撞证明：swept check 只检查每个步内的有限个插值点。它也不能证明全局 branch preservation 或全局可行性。当前 descriptor 是局部连续性指标，不是经过独立 IK 分解得到的全局 branch 身份；filtered connector 在边界处安全停止，说明还需要 phase governor 或可回退的全局连接策略，才能把安全性转化为稳定的长轨迹完成率。

## 6. Phase governor 与多初值 IK 分支诊断

### Phase governor

`validate_branch_connector.py --governor --min-progress-scale 0.125` 在过滤器拒绝当前候选时，依次尝试降低任务速度和零空间速率，但不允许零速保持作为正常进度步，并对实际 midpoint/swept 端点重新检查。200 步温和速度的 3 个 trial 中：

- filtered \(N r\)：2/3 完成，0 个碰撞样本，0 个关节限位样本；第 3 条在 165 步安全停止；
- 触发 5 次 governor 干预，最小允许缩放为 0.25；没有零相位速度保持；
- 未过滤 \(N r\)：3/3 完成，但有 284 个碰撞样本。

这排除了通过长期零速保持制造“完成”的情况：无法保持最低任务进度时，connector 明确拒绝该步并停止。高速度场景仍需要更强的可回退任务路径，才能同时满足进度和安全性。

阈值敏感性检查将最小进度缩放提高到 0.25，并在 100 步、3 个 trial 上复跑：filtered \(N r\) 完成 2/3，平均 99 步，碰撞样本 0；未过滤版本完成 3/3，但有 40 个碰撞样本。说明提高最低进度要求会牺牲少量完成率，但不会牺牲过滤安全性。

### 多初值 IK

`validate_global_branch_descriptor.py` 对 8 个状态、每臂 24 个初值执行独立 7-DOF 闭链 IK，并用同一个 point-cloud/box checker 检查组合解。随后对解对建立 predictor-corrector continuation edge：每条边使用 12 个 waypoint，每个 waypoint 都重新投影到闭链约束并检查关节界限。8/8 个状态都找到两臂闭链解和无碰撞组合解。

图结果显示：左臂 8/8 个状态的采样解形成单一 continuation component；右臂 7/8 个状态形成单一 component，另一个状态出现 1 个包含 4 个节点的主 component 和 2 个孤立节点。多解 descriptor 的范围约为：左臂最大 0.175 rad，右臂最大 0.253 rad。孤立节点尚不能直接称为独立全局分支，因为它们可能来自初值覆盖不足或 continuation 分辨率不足。

因此目前的证据支持 descriptor 作为局部连续性坐标，并给出了一个可复现的有限分辨率 atlas 诊断；它仍不支持把 descriptor 直接称为“全局 IK branch label”。要建立全局分支结论，还需要增加初值覆盖、沿相位和任务坐标继续追踪 atlas，并对孤立簇做更高分辨率的连接搜索。

对同一组 4 个状态和 16 个初值，将 continuation waypoint 从 12 增加到 24，并将边残差阈值从 \(5\times10^{-4}\) 收紧到 \(10^{-4}\)，左右臂的连通分量和 descriptor 范围保持不变。这说明当前有限 atlas 结论对这组数值参数不敏感，但仍只覆盖采样到的局部解集。

针对原右臂孤立状态，使用 96 个初值得到 16 个有效解和 2 个 base component。沿 \(\phi\) 双向各延拓 3 步，并在 \(\theta,s\) 的 3\times3 邻域中用 \(A_q^+(-A_x\Delta x)\) predictor 继续追踪。两个 component 在 9 个邻域格点上的最小关节距离均约为 3.13 rad，未出现靠近或合并；两者 descriptor 区间仍有重叠。因此，这个孤立簇更像是配置空间中的候选独立分量，而不是 descriptor 值本身分离出的分支，但还需要更大范围 atlas 才能作全局结论。

随后将任务区域扩大到 \(\theta\pm0.20\)、\(s\pm0.02\)，使用 5\times5 网格、每个 component 的 4 邻域 BFS continuation，以及 \(\phi\) 双向 5 步延拓。两个 component 在共同可达的 5 个网格点上，最小关节距离为 3.12--3.14 rad；没有观察到绕行后重新连接。其余远端网格因闭链校正或关节界限失败而不可达，因此该实验支持局部拓扑分离，但不能把不可达区域解释成全局不可连接。


## 7. 第一轮 paired baseline 验证

新增脚本 [validate_paired_baselines.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_paired_baselines.py) 让 proposed lift、同一 (H/N) 但不加 filter、DLS differential IK 和 warm-start pointwise projection 重放完全相同的初态、任务速度、(r_{nom})、步长和独立 checker。

4 个 trial、每条 100 步、(Delta t=0.005) s、(dot x=(0.08,0.002))、(r=(0.4,-0.4)) 的未 governor 结果为：

| 方法 | 完成 | 平均步数 | 最小 swept 净空中位数 | 碰撞样本 | 关节限位样本 |
|---|---:|---:|---:|---:|---:|
| filtered lift | 1/4 | 66.8 | 5.000 mm | 0 | 0 |
| unfiltered (H/N) | 4/4 | 100 | -19.7 mm | 191 | 30 |
| DLS | 4/4 | 100 | -19.7 mm | 191 | 30 |
| pointwise projection | 2/4 | 93.0 | -17.8 mm | 163 | 0 |

加入 \(\beta_{min}=0.125\) governor 后，filtered lift 完成 2/4，平均 79.8 步，碰撞样本仍为 0；52 步使用了降速，2 条轨迹因无法保持最低进度而安全停止。这个小批次说明：当前可区分的增量主要来自 \(N r\) 安全过滤和 governor，而不是局部 (H/N) 代数本身；DLS 与 unfiltered lift 在本设置下几乎一致，这是应保留的负结果。

该实验仍是 protocol pilot，样本量不足以形成最终统计结论。下一轮需要冻结参数后扩大到多场景、多初态，并增加 branch jump、任务进度和延迟分布的置信区间。

将相同协议扩大到 8 个 paired trials 后，filtered lift + governor 完成 3/8，平均 79.3 步，碰撞样本 0；未过滤 (H/N) 和 DLS 均完成 8/8，但分别产生 366 个碰撞样本；pointwise projection 完成 6/8，产生 338 个碰撞样本。这个结果仍是小批次，但安全性差异在扩大样本后保持。

## 8. 冻结矩阵：多速度、多场景与实现一致性修正

为进入正式比较，新增并冻结了协议文档
[branch_preserving_differential_lift_frozen_matrix_protocol_2026-10-02.md](./branch_preserving_differential_lift_frozen_matrix_protocol_2026-10-02.md)，以及矩阵驱动器
`hil-serl/tools/validate_paired_matrix.py`。矩阵包含 3 个任务速度（gentle、nominal、stress）、3 个墙体场景（nominal、tight、shifted），每个 cell 5 个 paired trials，每条轨迹 100 步，总计 45 个 trial。每个 trial 额外记录任务进度、swivel descriptor jump、每步 latency 分布和停止原因。

首次 45-trial 运行发现 filter 认证的 Euler 候选与执行统计中的 midpoint 候选不一致，因而出现了错误的 filtered 限位样本。现已修正：midpoint/swept 端点若不满足安全约束，filtered 方法直接拒绝该步并记录 `post_integrator_check`，不把它算作已执行的违规轨迹。修正后的正式结果保存在
`outputs/differential_lift_feasibility_20261002/paired_matrix_45trials_fixed.json`。

### 45 个 paired trials 的总体结果

| 方法 | 完成 | 中位最小 swept 净空 | 碰撞样本 | 关节限位样本 | descriptor jumps | 中位任务进度率 |
|---|---:|---:|---:|---:|---:|---:|
| filtered lift + governor | 25/45 | 5.000 mm | 0 | 0 | 0 | 0.9925 |
| 未过滤 (H/N) | 45/45 | -8.63 mm | 1392 | 296 | 0 | 1.0000 |
| DLS | 45/45 | -8.63 mm | 1392 | 296 | 0 | 1.0000 |
| pointwise projection | 42/45 | -7.73 mm | 1328 | 0 | 0 | 1.0000 |

filtered 方法另外有 19 个 `no_feasible_candidate` 安全停止和 1 个 `post_integrator_check` 停止；未过滤 (H/N) 和 DLS 的 median 每步 latency 约 10.6--10.8 ms，pointwise projection 约 15.0 ms，filtered 方法约 61.5 ms，中位 P95 latency 约 61.5--65.8 ms。filtered 的安全性因此以较高计算代价和部分安全停止换取，不能写成实时性优势。

分 cell 的 filtered 完成率为：nominal/gentle 1/5、nominal/nominal 3/5、nominal/stress 2/5；tight 三个速度层分别为 3/5、3/5、2/5；shifted 三个速度层分别为 4/5、4/5、3/5。tight 场景只向内移动两墙各 1 mm，因为当前冻结流形的名义净空约为 9 mm，更大的内移会使初始净空 (d\ge7) mm 的合法样本不存在。

### 证据边界

这轮结果通过了安全过滤的 Gate B：在 45 个 trial 中 filtered 没有碰撞或关节限位样本，且任务进度不是零速保持造成的假完成。它没有通过“论文主体全部成立”的 Gate C：四种方法的 descriptor jump 都为 0，因此 descriptor 连续性在当前任务段上没有区分力；同时 filtered 只有 25/45 完成，说明当前 governor 仍经常安全停止。要声称 branch-preserving differential lift，还必须加入独立多初值 IK/atlas branch label，并设计能诱发逐点 IK 分支切换的任务段；本轮只能支持“安全约束下的局部 differential lift 和明确的安全停止机制”。

## 9. 理论契约探针与 branch-label 诊断

按照新的“理论契约—微元探针—宏观轨迹”流程，先修改草稿中的三个契约：

1. 正则工作域收缩为 \(\mathcal M_\epsilon\)，显式记录 \(\sigma_{\min}(A_q)\) 和 clearance margin；
2. 误差衰减命题改为带条件的连续时间结论，并增加离散 Lyapunov 契约
   \(V_{k+1}-V_k\le-c\Delta tV_k+C\Delta t^2\)；
3. 将 phase action set 区分为 `feasible`、`degenerate-feasible` 和 `robust-feasible`，不再把 Chebyshev 半径大于零当作唯一可行条件。

新增三个独立探针：

- `hil-serl/tools/static_tangent_probe.py`；
- `hil-serl/tools/affine_slice_probe.py`；
- `hil-serl/tools/branch_label_probe.py`。

### 静态切空间契约

100 个 regular 状态的结果保存在 `static_tangent_probe_100.json`：

- \(\sigma_{\min}(A_q)\) 中位数 0.139，5th percentile 仍高于 0.1；
- \(\|A_qN\|_\infty\) 最大 \(6.94\times10^{-16}\)；
- tangent residual 最大 \(6.97\times10^{-16}\)；
- 单个微步的 basis alignment 最小奇异值中位数为 0.999999999995；
- 所有静态契约状态为 `pass`。

这验证了进入时域积分前的局部代数结构，但没有验证碰撞、全局 branch 或长期可行性。

### 一步 affine slice 契约

20 个状态、21×21 个 phase-rate 网格的结果保存在 `affine_slice_probe_20.json`。线性 polygon 与真实一步端点/swept checker 的 grid IoU 中位数为 1.0，没有发现线性 false-positive 或 false-negative。

这个结果只说明在当前 \(\Delta t=0.005\) s、当前速度和局部 clearance 区域内，一阶 safe polygon 没有明显漏切；它不能替代连续时间安全证明。后续需要在更大的速度、步长和 clearance 梯度变化下重复该检查。

### 100 初值 branch-label 探针

在 atlas 已识别的状态
\(x=(-0.352383,-0.138058)\) 上运行 100 个初值的局部 IK，并用 continuation graph 定义 component label。左臂得到 1 个有限分辨率 component，右臂得到 3 个 component。结果和图保存在：

- `branch_label_probe_state6_100start.json`；
- `branch_label_probe_state6_100start.png`。

在相同静态 atlas 上进行 80 步 warm-start Pointwise IK 与 DLS replay 时，两者均未产生 label jump。这是一个负结果：它说明当前任务段还没有形成公平的 branch-stress 条件。

为验证 label 检测器本身，在同一状态使用随机初值的 Pointwise IK 诊断得到 17 次 label transition，DLS 仍为 0 次；结果为：

- `branch_label_probe_state6_random_pointwise.json`；
- `branch_label_probe_state6_random_pointwise.png`。

随机初值结果只能作为检测器和 solver-basin 敏感性的诊断，不能直接作为优于 Pointwise IK 的论文证据。正式主实验仍需找到在公平 warm-start、相同任务段和相同数值预算下能够诱发 Pointwise/DLS branch transition 的任务。
## 10. 公平 branch-stress 搜索

为避免把大步长、闭合失败或标签歧义误判为 branch preservation 证据，新增
`hil-serl/tools/branch_stress_search.py`。它固定 warm-start Pointwise IK，要求：

1. 初始 continuation atlas 在左右臂均至少有两个 component；
2. 跳变前后 label margin 不小于 `0.05`；
3. 跳变前后闭合残差均不超过 `2e-4`；
4. 只有满足上述条件且 Pointwise 有跳变、H+Nr 与 DLS 都无跳变时，才标记为候选压力段。

在两个已有任务状态上，对 `dt={0.01,0.03}`、
`theta_dot={0.8,2.0}`、固定 `slide_dot=0.02` 和 `|r|=0.8` 的 8 个组合进行
首轮搜索（每臂 32 个初值，12 个任务步）。结果保存在：

`outputs/differential_lift_feasibility_20261002/branch_stress_search_smoke.json`

8 个组合均为 `atlas_insufficient`，候选数为 0。这个结果的含义是：当前冻结的
有限分辨率 continuation graph 尚未同时确认两个臂存在两个可靠 component，因而
不能声称找到了公平的 Pointwise 分支跳变段。此前高速度测试中出现的标签变化，
要么 H+Nr/DLS 同时变化，要么闭合误差达到 `1e-2` 量级，已按规则排除。

因此正式 30--50 paired trials 暂不应启动。下一步应增加初值覆盖并沿
`(theta,s,phi)` 做 atlas 延拓；只有 atlas 条件通过后，才冻结压力段并进行方法
对比。当前证据支持的是局部微分 lift 和安全过滤契约，尚不支持全局 branch
preservation 优势。

## 11. 双臂 dense-atlas 扫描

新增 [validate_dual_arm_atlas_segment.py](/home/rocos/sia/AVIATOR/hil-serl/tools/validate_dual_arm_atlas_segment.py)。
该脚本对左右臂分别使用 100--128 个初值，构造 continuation graph，并为每个
component representative 沿局部 \(\phi\) 和 \((\theta,s)\) 网格延拓。只有 component
在至少 80% 的任务网格中保持闭合残差阈值内，才会被标记为 stable。

对已有 8 个任务状态和另外 16 个随机安全状态进行 dense component scan 后，没有
找到左右臂同时拥有至少两个稳定 component 的状态：

- 已知状态中，7 个状态为 `(1,1)`，状态 6 为 `(1,3)`；
- 随机状态中，大多数为 `(1,1)`，少数为 `(1,2)`、`(2,1)` 或 `(3,1)`；
- 没有出现 `(>=2,>=2)`。

代表性结果包括：

- `dual_atlas_state6_128.json`；
- `dual_atlas_scan_state0_128.json` 至 `dual_atlas_scan_state7_128.json`；
- `dual_atlas_scan_random0_100.json` 至 `dual_atlas_scan_random15_100.json`。

因此第 3 个门槛尚未通过，不能冻结 branch-stress 任务段，也不能把后续
30--50 paired trials 解释为 branch preservation 的正式证据。下一步需要扩大
任务状态覆盖，或明确论文只声称单臂/局部分支保持；在此之前不应通过收紧 graph
距离阈值人为制造 component。

## 14. 不同流形与单臂候选检查

为检查结论是否只属于 `manifold_phi_stale_pitch-10deg`，又在
`hil-serl/data/aviator/manifold_phi` 上抽取 8 个随机安全任务点，使用每臂 100
个初值扫描。结果仍未出现双臂同时多 component；其中两个点为 `(1,2)`，其余为
`(1,1)`。

在当前流形中右臂确实能得到 3 个有限分辨率 component 的点上，进一步改变初始
component 并测试高速度 warm-start 轨迹。出现过 Pointwise、DLS 和 H+Nr 同时换
label 的情况；Pointwise 跳变端点闭合残差约为 `4e-2`，因此被判为数值失效，不能
作为 branch-preservation 证据。

这说明“换流形”没有自动产生公平的跳变对照，当前最稳妥的论文定位仍是局部
branch-preserving continuation，而不是声称 H+Nr 在整个机器人 IK 空间中全局保持
分支。

## 15. 抓取几何变体

现有 `nominal`、`shifted` 和 `tight` 场景只改变墙体位置，不改变 IK 抓取几何，
因此不能单独用来检验 branch topology。为此建立了保留原始模型的抓取几何变体：

`AviatorRobot/build/bin/model/aviator_grasp_offset10mm.xml`

左右 handle 在物体局部坐标中沿相反方向平移 10 mm。旋转 (10^circ) 和 (20^circ)
的姿态变体会使该任务点的右臂无有效 IK，因此不作为公平对照；位置变体仍有可行
IK。

在该位置变体的 state 6 附近进行 3×3 任务段扫描，每个网格点每臂 100 个初值：

`task_segment_scan_grasp_offset10mm_state6_3x3_100.json`

结果为：

```text
(1,4) (1,2) (1,3)
(1,2) (1,4) (1,3)
(1,3) (1,2) (1,1)
```

几何变化增加了右臂 component 数，但仍没有左右臂同时多 component。因此当前
实验支持把论文主张收紧为：

> 在存在可识别多 component 的局部任务段上，验证 branch-preserving differential continuation。

## 16. 单臂局部分支对照（冻结协议 v1）

针对右臂存在 2--3 个有限分辨率 component 的任务点，冻结以下协议：

- 每臂 128 个初值（候选筛选另保留 64-start 结果）；
- 100 个任务步，\(\Delta t=0.005\) s；
- \((\dot\theta,\dot s)=(0.16,0.004)\)，\(r=(0.8,-0.8)\)；
- IK residual tolerance \(2\times10^{-5}\)；
- label margin 0.05；
- 每 4 步更新 continuation representative；
- 三种方法使用相同 warm-start、反馈和求解器预算；
- 只有 label margin 合格且跳变前后闭合误差均不超过 \(2\times10^{-4}\) 才计为有效 branch jump。

汇总脚本为 [summarize_single_arm_branch_comparison.py](/home/rocos/sia/AVIATOR/hil-serl/tools/summarize_single_arm_branch_comparison.py)，
结果为：

`outputs/differential_lift_feasibility_20261002/single_arm_branch_comparison_frozen_v1.json`

三组单臂候选段中，H+Nr、DLS 和 Pointwise 的有效 branch jump 均为 0。H+Nr/DLS
的最大闭合误差约为 \(10^{-6}\)；Pointwise 的最大闭合误差为
\(4.8\times10^{-4}\)、\(5.5\times10^{-4}\) 和 \(1.1\times10^{-3}\)，部分超过
冻结阈值，但没有形成可计数的合法 branch jump。

因此这轮实验可以证明：在所选局部段上，H+Nr 保持了连续闭合，但还不能证明它
相对于 Pointwise/DLS 具有 branch-preservation 的成功率优势。后续正式 paired
trials 仍应等待一个满足“基线出现合法跳变、H+Nr 不跳”的压力段；否则应将主结果
定位为局部连续性、闭合误差和安全停止机制。

随后新增 [single_arm_stress_search.py](/home/rocos/sia/AVIATOR/hil-serl/tools/single_arm_stress_search.py)，
在单臂多 component 状态上扫描更大的步长和任务速度。当前两组搜索结果为：

- `single_arm_stress_search_current.json`：8 个组合，0 个合法对照候选；
- `single_arm_stress_search_stale_state6.json`：12 个组合，0 个合法对照候选。

搜索仍使用同一闭合误差阈值和 label margin。没有把大步长造成的求解器失效计作
branch jump，因此正式 30--50 paired trials 继续暂缓。

## 17. 重定位后的 45-trial 正式矩阵

按照“局部在线延拓 + 安全监管”的新主线，重新运行 3 个速度层、3 个墙体场景、
每格 5 个 paired trials，共 45 个 trial。新报告为：

`outputs/differential_lift_feasibility_20261002/paired_matrix_reframed_45trials.json`

主要结果如下：

| 方法 | 完成 | 碰撞采样 | 关节限位采样 | 安全/失败记录 |
|---|---:|---:|---:|---|
| `lift_filtered` | 26/45 | 0 | 0 | 17 `no_feasible_candidate`，2 `post_integrator_check` |
| `lift_unfiltered` | 45/45 | 1154 | 130 | 38 `complete_with_violations` |
| `dls` | 45/45 | 1154 | 130 | 38 `complete_with_violations` |
| `pointwise_projection` | 44/45 | 1133 | 0 | 35 `complete_with_violations`，1 projection residual |

`lift_filtered` 的任务进度中位数约为 1.0，但完成率只有 26/45；这说明它能够
推进任务并在不可行区域停止，却不能声称在所有场景中可靠完成。其 descriptor
最大单步变化中位数约为 (9.6\times10^{-4}) rad，45 个 trial 中没有 descriptor
jump。平均单步延迟中位数约为 83.5 ms，明显高于未过滤基线的约 11--15 ms。

该矩阵现在支持的论文结论是：完整 filter/governor 能以较高计算代价消除碰撞和
关节限位样本，并将不可行状态转化为可解释的安全停止；它还没有证明更高的完整
任务成功率，也没有证明相对于 Pointwise/DLS 的 branch-preservation 优势。

## 13. 任务段网格扫描

为检查多 component 是否只在孤立任务点缺失，新增
`hil-serl/tools/scan_task_segment_components.py`，在 state 6 周围的
\((\theta,s)\) 邻域进行 3×3 网格扫描。每个网格点左右臂各使用 100 个初值，
并用相同 continuation graph 参数建立 component label。

结果保存在：

`outputs/differential_lift_feasibility_20261002/task_segment_scan_state6_3x3_100.json`

9 个网格点中没有一个满足双臂同时多 component。component 数分别为：

```text
(1,3) (1,3) (1,3)
(1,2) (1,3) (1,1)
(1,3) (1,3) (1,1)
```

因此在当前任务段附近，右臂的多 component 结构持续存在，但左臂始终只有一个
有限分辨率 component。第 3 个门槛仍未通过，后续不启动正式 paired trials。

## 12. 当前门槛状态

| 门槛 | 状态 | 证据 |
|---|---|---|
| 每臂 100--200 初值 | 已完成首轮 | 100--128 starts 的 dense scan |
| \\((\\theta,s,\\phi)\\) atlas 延拓 | 已实现并在 state 6 运行 | `dual_atlas_state6_128.json` |
| 双臂同时存在两个稳定 component | 未通过 | 24 个状态无 `(>=2,>=2)` |
| 独立 descriptor 与闭合误差 | 已接入 | component extension 输出 descriptor/residual |
| Pointwise/DLS 跳、H+Nr 不跳 | 未开始正式冻结 | 由于上一门槛未通过 |
| 30--50 paired trials | 暂缓 | 不能把当前数据解释为 branch-preservation 统计 |

当前最重要的判断不是增加 trial 数量，而是确认“双臂多 component”是否确实是
该机器人和任务族的存在条件。如果进一步的任务覆盖仍没有出现该结构，应将论文
命题收紧为“在存在可识别多 component 的局部任务段上进行 branch-preserving
continuation”，或改为验证单臂分支保持；不能把没有多 component 的任务强行包装
成 branch-switch 对照。

## 18. 理论重构后的冻结状态（2026-10-03）

根据论文重构决定，正式方法的数学契约已冻结为：

- `y=[r_L,r_R,beta]` 是统一的在线决策变量；
- `dot q=h_e+beta h_x+N r`、`dot x=beta dot x_nom`；
- 安全集合是三维 `Y_safe`，而不是二维 phase polygon 加外部 governor；
- 任务误差使用 ISS 残差界，主动约束或降速时记录 `delta`；
- full joint-space QP 与 reduced `y`-QP 的可行性/恢复误差必须通过独立 Node 0 测试；
- branch 结论限制为正则连通 component 内的 local continuity lemma；
- endpoint/collocation validator 是独立安全判定器。

因此，现有 `paired_matrix_reframed_45trials.json` 保留为**重构前实现的预备安全证据**。它验证了 filter/governor 能把碰撞和关节限位转化为可解释的停止，但不能直接作为统一三维 `y`-QP 的正式结果。下一步顺序固定为：

1. 完成解析 Jacobian、分解残差、full/reduced QP、积分阶次和三类 governor 状态的 Node 0 JSON；
2. 通过后冻结代码、阈值和数值预算；
3. 重新运行各场景类 30--50 个 paired trials，并加入 full joint-space QP + 同一 governor；
4. 以任务进度、ISS 残差、停止原因、延迟和独立 component label 一起判定论文主张。

若 Node 0 不能通过，问题先归类为实现或假设边界问题，不增加 trial 数量。

## 20. 统一三维 QP 的正式 v3 矩阵（2026-10-05）

Node 0 通过后，按冻结参数重新运行了 3 个速度层（gentle/nominal/stress）× 3 个
场景（nominal/tight/shifted）× 10 个 paired trials，共 90 个 paired trials。报告为：

`outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json`

本轮使用统一决策变量 `y=[r_L,r_R,beta]`，并加入 `full_joint_qp` 作为同一局部线性
安全约束下的 15 变量强表示对照。总体结果如下：

| 方法 | 完整完成 | 碰撞采样 | 关节限位采样 | descriptor jump | 主要停止原因 |
|---|---:|---:|---:|---:|---|
| `lift_filtered` | 44/90 | 0 | 0 | 0 | 45 `endpoint_validation`，1 `post_integrator_check` |
| `full_joint_qp` | 44/90 | 0 | 0 | 0 | 45 `full_qp_endpoint_validation`，1 `post_integrator_check` |
| `lift_unfiltered` | 90/90 | 2679 | 248 | 0 | 72 `complete_with_violations` |
| `dls` | 90/90 | 2679 | 248 | 0 | 72 `complete_with_violations` |
| `pointwise_projection` | 85/90 | 2610 | 0 | 0 | 66 `complete_with_violations`，5 `projection_residual` |

`lift_filtered` 与 `full_joint_qp` 的中位最小 swept clearance 分别为
`0.00500000375 m` 和 `0.00500000375 m`，中位任务进度均为 `0.975`，中位单步延迟
分别为 `67.26 ms` 和 `67.30 ms`。二者的闭合误差、停止样本和 descriptor 结果也一致，
支持当前实现下 reduced/full formulation 的行为等价。它们的 44/90 完成率同时说明：
安全过滤将一部分非线性 endpoint 不可行状态转化为主动停止，不能把“零碰撞”表述为
“所有任务均成功”。

这轮矩阵支持三点：

1. 统一三维 QP 在真实 MuJoCo 状态上能以零碰撞、零关节越界运行，并记录可解释停止原因；
2. full joint-space QP 与 reduced `y`-QP 在本地约束、积分器和独立检查器下给出一致结果；
3. 未过滤的 DLS/lift 和 pointwise projection 在相同任务进度下出现大量 clearance 违规，
   因而不能把“完成轨迹”直接当作安全完成。

这轮没有观察到任何方法的 descriptor jump。因此，当前数据不证明“Pointwise/DLS
必然发生分支跳变”或“`H+Nr` 全局保持分支”；论文应继续使用“regular connected
component 内的局部连续延拓”和“安全拒绝/降速机制”的表述。`branch jump=0` 是测试
范围内的结果，不是全局拓扑结论。

旧的 v1/v2 矩阵保留用于调试溯源，不与 v3 正式结果合并统计。

## 19. 三维 action filter 与 Node 0 实现状态（2026-10-04）

已将实际在线 filter 扩展为统一的三维动作 `y=[r_L,r_R,beta]`：

- `safety_filter.py::project_reduced_action`：求解带线性半空间和 beta 上下界的三变量严格凸 QP；
- `validate_branch_connector.py::project_action_y`：从真实 MuJoCo 的 `A_q/A_x`、null basis、关节边界和独立 clearance gradient 构造 `Y_safe`；
- `validate_branch_connector.py::connector_step_y`：执行 `h_e+beta h_x+N r`，并使用独立 endpoint/swept checker；
- `validate_paired_baselines.py::full_joint_qp_step`：使用同一局部线性安全约束，在 14 个 `qdot` 加 beta 的全空间中求解强表示基线。

Node 0 真实状态报告：

`outputs/differential_lift_feasibility_20261002/node0_real_contract_24.json`

24 个真实 MuJoCo 状态均通过：

- `sigma_min(A_q)` 中位数约 0.142，最大约 0.169；
- `A_q`、`A_x` 与中心有限差分的最大误差约 (2.9\times10^{-5})；
- null residual 最大约 (5.8\times10^{-16})；
- tangent residual 最大约 (2.0\times10^{-16})；
- full/reduced QP 恢复的 `qdot` 最大误差约 (8.3\times10^{-9})；
- governor 的满速、降速和停止三类状态均通过。

Node 0 通过后已启动新的 9 单元正式矩阵：3 个速度层 × 3 个场景，每单元 10 个 paired trials，即每类场景 30 个 paired trials。旧二维 filter 的 45-trial 结果仍作为历史预备证据保留，不与本轮三维实现混合。

## 21. N1 分支区分任务段搜索（2026-10-05）

N1 按冻结协议新增了真正包含 `lift_filtered` 的 branch probe。每个 case 使用 128 个
初值/臂、相同的 warm-start、80 步轨迹、固定 label margin `0.05` 和 closure tolerance
`2e-4`。独立 descriptor 还增加了 `0.04 rad` 的最小变化门槛；标签变化、descriptor
变化、闭合残差和安全停止必须同时满足，才计为合法 branch jump。

右臂在 state-6 周围的独立 atlas 中，从 128 个初值得到 2 个 base components；两者
都有局部 `(theta,s,phi)` 延拓样本，但严格的 5x5 扩展每个 seed 仅覆盖 5/25 个共同
网格单元，未通过宽范围稳定性门槛。宽的双臂 5x5 atlas 没有得到双臂同时稳定的多个
component，stable counts 为 `[0,0]`。因此后续对照只把右臂多 component 与左臂
原始 LUT 安全姿态配对，并明确记录这是单臂局部诊断。

24 个冻结 stress cases 的严格汇总为：

`outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json`

| 分类 | 数量 |
|---|---:|
| 合法 candidate（Pointwise/DLS jump、lift_filtered 不跳且走到对应步） | 0 |
| 完成比较但无合法 contrast | 20 |
| 静态 atlas component 不足 | 4 |
| 程序失败 | 0 |

所有严格 valid jump 计数均为 0。一个 raw DLS/H+Nr 标签变化的 closure residual 仍在
阈值内，但独立 descriptor 只变化约 `0.0075 rad`，小于冻结的 `0.04 rad`，因此被排除。
该 case 中 `lift_filtered` 还在基线标签变化前因 endpoint validation 停止，不能解释为
branch preservation 证据。

N1 的结论是：当前任务族没有提供可以公平区分 Pointwise、DLS 和 lift_filtered 的合法
branch-switch 段。论文不增加 branch-superiority 主张；主线继续限定为安全监管的局部
在线延拓。N1 的下一步是 N2：对 v3 矩阵和 N1 raw probes 中的 endpoint validation 做
停止原因、任务进度、beta 和 clearance 分解。

# AVIATOR 双臂协同操作研究整理

整理日期：2026-09-18。本文汇总本次讨论的全部主要内容：当前平台能力、不依赖内力约束的协同方案、近期研究与参考文献、狭小空间规划仓库、对偶四元数，以及推荐的实施和实验路线。按主题归并，不逐字保留对话中的进度消息。

文献与仓库信息沿用本次讨论时的核查结果；本次整理没有重新联网检索。文中“当前”指上述核查日期。没有在 AVIATOR 上实现或运行所推荐的新控制算法，方案适配性与性能预期均需实验验证。

## 阅读导航

- [一、平台基础、研究方案与近期文献](#part1)
- [二、狭小空间双臂协同的开源仓库与补充论文](#part2)
- [三、对偶四元数的用途、实现与参考文献](#part3)
- [四、统一技术路线与实施顺序](#part4)
- [五、文档、代码入口与文献库](#part5)

**核心建议：先做对象中心的闭链运动学协同、任务方向可操作度和避碰，再扩展全局规划或学习策略。** 当前不必以内力约束为前提。对偶四元数可作为协同建模工具，TSR 可作为任务表达，Mink/PyRoki/QP 用于局部求解，OMPL/Drake 用于全局路径研究；这些属于不同层次，不要求全部同时接入。

<a id="part1"></a>

## 一、平台基础、研究方案与近期文献

本部分保留第一轮调研的方案、实验设计和 10 篇核心文献。编号 [1]—[10] 仅在本部分使用。

检索日期：2026-09-18。重点：2025—2026 年工作；最新收录预印本为 2026-09-09。本文是针对当前平台的定向调研，不是穷尽性系统综述。主张与适配建议分开陈述，未运行新控制实验。

**建议主线：面向受限座舱两自由度操纵盘的闭链运动学协同、任务方向可操作度优化与约束保持在线重规划。** 先建立可靠模型方法，再考虑学习规划器的初值、冗余姿态或低维动作。当前无需把内力约束作为核心。

### 1. 当前平台能支持什么

依据 `AviatorRobot/README.md`、`src/Aviator.cpp`、`config/aviator.yaml`、`validation/cylinder_grasp/README.md`：

- 左右各 7 自由度，共 14 轴位置指令；操纵盘转动和推拉两轴无 actuator，由双臂经两个 weld 被动带动。
- 这更接近航空操纵盘（yoke），不是可无限连续转动的汽车方向盘。模型工作范围约为转角 ±50°、推拉 −165～0 mm；已有验证路径包含 ±50° 转动和 −160 mm 推拉，不能据此认定整个转角—位移矩形均可达。
- `MoveWheel()` 用统一平滑相位插值转角和位移；`target()` 由操纵盘位姿生成左右法兰目标；`solve()` 分别解两臂 IK，以上一时刻解作种子；随后统一检查关节限位、速度和碰撞。
- 两侧 J2 的任务限制为 85°～95°，规划留出 0.5° 余量。冗余优化应保留该约束，不能靠放宽它获得不公平优势。
- `execute()` 在离散规划点之间线性插值关节角。端点满足闭链关系并不意味着插值期间严格满足；`validate()` 检查中点碰撞，但不是连续闭链误差证明。这是研究切入点，不是已经测得的失效。
- 已有抓取前的离线位置/朝向网格搜索；锁定后保持完整相对位姿，不允许沿握持轴自由转动。现有执行要求两侧同时保持锁定。
- README 的速度说明与当前 YAML 有差异：YAML 实际 `joint_speed: 6.28`，代码取其与 URDF 速度上限的较小值。实验必须记录实际配置，不能直接沿用 README 的 0.7 rad/s。

**证据边界：** weld 仍会在物理求解器中产生约束反力。“不做内力控制”不等于“没有内力”。本平台适合检验几何一致性、跟踪、可达性和避碰；仅凭 weld 仿真不能证明真实夹爪防滑、内力降低或柔顺接触稳定性。

### 2. 不含内力项的统一问题表述

令任务坐标 `x = (θ, s)`，关节为 `qL, qR`。令 `T_W(x)` 为操纵盘坐标系相对世界的位姿，`G_i` 为固定抓取变换。若 `FK_i` 输出 TCP 位姿，则闭链要求：

```text
FK_L(qL) = T_W(x) G_L
FK_R(qR) = T_W(x) G_R
FK_L(qL)^(-1) FK_R(qR) = G_L^(-1) G_R
```

第三式由前两式推出，通常不需要再作为重复的独立等式加入求解器。仅约束双手相对位姿也不够：物体还必须服从操纵盘机构的两自由度运动学。若使用法兰 FK，需加入现有 `tool.Inverse()`，不能混用法兰与 TCP。

在同一坐标表达下微分前两式，可构造 `J_i(q_i) qdot_i = A_i(x) xdot`，并用几何误差反馈抑制积分漂移。推荐优化关节速度或短时域轨迹，同时考虑：

1. 操纵盘转角、位移的跟踪误差。
2. TCP—把手位姿误差和两臂相对位姿误差。
3. 关节位置、速度、加速度限制及轨迹平滑性。
4. 双臂自碰撞、座舱和操纵盘非抓取部件的距离约束。
5. 面向转动/推拉方向的可操作度与关节限位余量。

位置与角度残差必须分别归一化或使用特征长度。避免把米、弧度及不同量纲的雅可比奇异值直接相加。局部等式可通过 QP 强制满足，但离散积分、模型误差和伺服跟踪仍会引入残差；需要投影、反馈和执行误差评估。

### 3. 可探索方案与优先级

| 方案 | AVIATOR 上的具体做法 | 研究价值与局限 | 优先级/参考 |
|---|---|---|---|
| 对象中心的集中式 QP/HQP | 用同一个 `(θ,s)` 同时生成两臂速度；闭链一致性优先，冗余量用于限位和避碰 | 接口改动较小；单纯把 IK 换为 QP 创新不足，要解决受限构型和连续执行问题 | A；[1–3] |
| 闭链流形轨迹规划 | 在任务坐标和冗余坐标中搜索；用 IK/投影回到两臂可行构型，检查区间内误差 | 可研究 IK 分支连续性、约束保持插值、局部图切换；奇异点附近仍需处理退化 | A；[1,2] |
| 任务方向可操作度优化 | 优化转动和推拉方向的关节速度代价、最小余量；协调两侧肘部 | 比单臂通用可操作度更贴近本任务；左右性能要联合评估 | A；[3,6] |
| 约束保持的 MPC/CBF-QP | 短时域重规划，在闭链等式下处理距离约束；困难时减速或停止 | 从“离线拒绝”发展到“在线应对”；需距离梯度、可行性恢复和最坏求解时间分析 | A/B；[3] |
| 运动学主从/角色切换 | 比较左主右从、右主左从、对象中心控制；按可达性/限位余量切换 | 主从是规划参数化，不是让某臂单独施力；切换需连续且满足机构约束 | B；[2] |
| 抓取—姿态—轨迹联合优化 | 扩展现有抓取网格搜索，联合选择抓取前朝向、冗余姿态与整条路径 | 可比较最坏条件可操作度、共同可达域和动作时间；锁定后抓取变换固定 | A/B；[1,6] |
| 受约束扩散策略/流匹配策略 | 学习低维任务增量、冗余姿态或轨迹初值，再经几何投影和验证 | 贴近热点；需要足够多可行路径和跨布局泛化实验。相关直接证据主要来自扩散方法 | B；[4–9] |
| 残差强化学习 | 在模型控制器外学习时间伸缩、冗余偏好或小幅任务残差，投影后执行 | 可研究延迟、标定误差的鲁棒性；本轮重点文献未直接验证此任务，属于扩展建议 | B/C |
| 交替换抓与大角度连续转动 | 独立锁定/释放、单臂持有、重抓接近与接触模式规划 | 当前双锁定逻辑和 ±50° 机构限位不支持，需扩展平台；不能作为短期最小改动路线 | C；[10]仅作任务背景 |

可操作度优化建议：对给定任务方向 `xdot`，在闭链和关节速度盒约束下求可行速度缩放，或最小化归一化关节速度范数。两臂能力取瓶颈并评估方向相关性，比单纯最大化两臂 `det(JJᵀ)` 更直接。此处是针对平台的研究设计。

CBF 局部过滤不自动给出全局避障路径。闭链、避碰和 J2 限制冲突时可能无解；需要可行的减速/停止方案与上层重规划。论文应说明采样周期、距离误差和模型假设，不能仅因使用 CBF 就宣称无条件安全。

### 4. 近期热点及平台适配

**热点一：把协同关系作为规划空间本身的约束。** 2026 年的快速硬约束规划和可微流形图工作，分别关注约束保持搜索及 IK 参数化梯度。[1,2] 对本平台最有价值的是两自由度机构约束与双臂冗余的联合处理。两篇工作主要涉及共同搬运，迁移到操纵盘还需要加入物体—基座的运动限制。

**热点二：协同控制、可操作度和反应式避碰结合。** 2026 年 Chen 等将协作任务空间、可操作度参考与 CBF 过滤相结合。[3] 可借鉴结构，但本平台操纵盘不能像托盘一样任意调整空间姿态，避碰自由度更少。

**热点三：生成策略显式结合运动学和约束。** ADCS 处理相对/绝对位姿及不等式约束；KStar 使用机器人图结构与运动学正则；SafeBimanual 在推理中优化安全代价。[5,7,8] 学到的约束偏好不等同于严格可行性保证，仍应检查或投影输出。

**热点四：姿态和坐标系成为策略的一部分。** ManiDP 关注姿态相关的双臂能力；MoF 研究不同坐标系中动作分布的组合。[4,6] 本任务可比较世界系、操纵盘系和双臂相对系表示；对于纯两自由度跟踪，复杂网络是否必要必须用简单模型基线回答。

**热点五：协同数据的可行增强。** D-CODA 同时增强两侧腕部视觉和关节动作，并通过约束优化保持协调。[9] 当前可以先从规划器导出状态轨迹，但这只是受启发的状态数据生成，不等于复现 D-CODA 的视觉方法。

### 5. 推荐选题与实验设计

**首选题目：面向受限座舱操纵盘的双臂闭链约束保持规划与任务方向可操作度优化。**

核心问题：在 J2 窄范围、座舱碰撞和操纵盘两自由度机构约束下，如何选择整段冗余姿态并保持插值与执行阶段的几何一致性，使转动—推拉复合运动更连续、更快且覆盖更多可行任务？

拟检验的贡献，而非已经证实的创新：

- 以 `(θ,s)` 和两侧局部自运动坐标表达协同规划，在不同 IK 分支间维持可执行连续性。
- 用机构运动方向上的双臂瓶颈能力评价替代单臂、各向同性姿态评价。
- 在可行性不足时联合调整路径和时间，研究任务精度、闭链误差和计算量之间的关系。

**学习方向题目：面向两自由度操纵机构的对象中心生成策略与闭链可行性投影。**

先用模型规划器收集多初始姿态、多目标、多布局的有效示范，策略预测任务空间动作、冗余偏好或短时域初值，再通过 QP/投影层执行。若所有数据只是一条固定插值曲线的重放，学习贡献较弱。应保留一个未参与训练的几何布局和独立目标区间作为测试，避免相邻轨迹点泄漏。

| 实验维度 | 建议安排 |
|---|---|
| 基线 | 当前公共任务参考＋逐臂连续 IK；统一 QP；QP＋方向可操作度；约束保持规划；再添加在线避碰或学习模块 |
| 任务 | 纯转动、纯推拉、转动＋推拉同步、往返复合路径、不同初始关节分支 |
| 难点 | 接近限位、狭小间隙、快速目标变化、动态障碍；均先筛选是否存在可行任务 |
| 泛化 | 操纵盘安装位姿/抓取配置改变时同步更新真实模型；另设控制器估计误差，不能把模型改变与估计误差混在一起 |
| 扰动 | 观测噪声、指令延迟、几何标定误差、不同仿真步长与 weld 参数；记录物理求解器影响 |
| 主要指标 | 任务成功率及失败原因；转角/位移 RMSE；TCP 和相对位姿最大误差；最小碰撞距离；关节限位余量；峰值速度/加速度；规划与在线求解时间分位数 |
| 消融 | 去闭链投影、去方向指标、去联合姿态优化、去在线过滤、固定主从与角色切换 |

**特别要区分规划指标与执行指标：** 对下发关节轨迹直接做 FK 评估命令的闭链误差，同时记录 MuJoCo 实测 TCP/操纵盘误差。只看被 weld 强行闭合后的实测状态，可能掩盖规划器自身的不一致。

共同可达域建议绘制 `(θ,s)` 网格，但要分别报告“至少存在一个静态 IK 解”和“能从当前构型沿连续无碰路径到达”。网格失败也不等于数学意义上的不可达，可能只是当前 IK/搜索失败。所有基线用同一物理配置、同一任务限位和同一采样评估规则。

### 6. 核心参考文献（近期 9 篇＋直接任务背景 1 篇）

| 编号 | 文献、作者与状态 | 内容与使用方式 |
|---|---|---|
| [1] | Thomas Cohn, Seiji Shaw, Harel Biggie, Travis Manderson, Nicholas Roy, Russ Tedrake. **Planning along Differentiable Charts of Constraint Manifolds with General-Purpose IK Solvers**. 2026-09-09，预印本、在审。[arXiv:2609.10905](https://arxiv.org/abs/2609.10905)；[作者项目页](https://tommycohn.com/inverse-function-theorem-parameterization/) | 从正运动学雅可比恢复 IK 参数化梯度，处理可达域边界；首读。不能直接断言现有 TRAC-IK 已满足其参数化和分支条件。 |
| [2] | Borna Paro, Luka Petrović, Ivan Marković. **Fast Coordinated Bimanual Motion Planning With Hard Constraints**. 2026-08-21，预印本。[arXiv:2608.20946](https://arxiv.org/abs/2608.20946) | 主从参数化与约束感知插值；适合比较当前关节插值。正文称代码将在评审完成后公开，不能视为已经可下载复现。 |
| [3] | Zhaoyang Chen, Fenglei Ni, Xin Shu, Yi Ren, Hong Liu. **Reactive collision-free motion generation for tightly coupled coordinated bimanual manipulation**. ENGINEERING Mechanical Engineering, 21, 100873, 2026-02-09，期刊论文。[DOI](https://doi.org/10.1007/s11465-026-0873-7) | 协作任务空间、任务优先级、可操作度与 CBF；模型控制主线的直接参考。本轮读取出版页摘要，未获得订阅全文。 |
| [4] | Dian Wang, Jisang Park, Xiaomeng Xu, Han Zhang, Shuran Song, Jeannette Bohg. **Mixture of Frames Policy: Multi-Frame Action Denoising for Bimanual Mobile Manipulation**. 2026-07-13 预印本；作者页标注 CoRL 2026。[arXiv:2607.11884](https://arxiv.org/abs/2607.11884)；[作者页](https://mofpo.github.io/) | 多坐标系同步去噪。会议状态依据作者页，未核验正式论文集；适合对象中心表示与泛化方向。 |
| [5] | Haolei Tong, Yuezhe Zhang, Sophie Lueth, Georgia Chalvatzaki. **Adaptive Diffusion Constrained Sampling for Bimanual Robot Manipulation**. 2025-05-19 首发，2026-02-25 v5；arXiv 明确标注 ICRA 2026 录用。[arXiv:2505.13667](https://arxiv.org/abs/2505.13667) | 相对/绝对位姿等式与 SDF 不等式的生成采样；适合受约束策略或规划初值。不是单纯无约束扩散。 |
| [6] | Zhuo Li, Junjia Liu, Dianxi Li, Tao Teng, Miao Li, Sylvain Calinon, Darwin Caldwell, Fei Chen. **ManiDP: Manipulability-Aware Diffusion Policy for Posture-Dependent Bimanual Manipulation**. IROS 2025；arXiv 首发 2025-10-27。[arXiv:2510.23016](https://arxiv.org/abs/2510.23016)；[DOI](https://doi.org/10.1109/IROS60139.2025.11246034) | 学习姿态相关可操作度特征。可借鉴速度能力与冗余姿态部分，不据此声称实现力能力验证。 |
| [7] | Qi Lv, Hao Li, Xiang Deng, Rui Shao, Yinchuan Li, Jianye Hao, Longxiang Gao, Michael Yu Wang, Liqiang Nie. **Spatial-Temporal Graph Diffusion Policy with Kinematic Modeling for Bimanual Robotic Manipulation**. CVPR 2025。[CVF 论文](https://openaccess.thecvf.com/content/CVPR2025/papers/Lv_Spatial-Temporal_Graph_Diffusion_Policy_with_Kinematic_Modeling_for_Bimanual_Robotic_CVPR_2025_paper.pdf)；[arXiv:2503.10743](https://arxiv.org/abs/2503.10743) | KStar Diffuser 将机器人结构和可微运动学用于动作生成；学习路线的重要基线。 |
| [8] | Haoyuan Deng, Wenkai Guo, Qianzhun Wang, Zhenyu Wu, Ziwei Wang. **SafeBimanual: Diffusion-based trajectory optimization for safe bimanual manipulation**. CoRL 2025, PMLR 305:3218–3238。[正式论文页](https://proceedings.mlr.press/v305/deng25c.html) | 推理时的安全代价引导，覆盖不同双臂协作模式；启发策略输出过滤，但不等同于硬安全证明。 |
| [9] | I-Chun Arthur Liu, Jason Chen, Gaurav S. Sukhatme, Daniel Seita. **D-CODA: Diffusion for Coordinated Dual-Arm Data Augmentation**. CoRL 2025, PMLR 305:3569–3588。[正式论文页](https://proceedings.mlr.press/v305/liu25f.html) | 结合约束优化的双臂视觉/动作数据增强；目前无视觉策略管线时可作为后续数据路线。 |
| [10] | Yue Dong, Zhangguo Yu, Xuechao Chen, Xin Zhu, Chenzheng Wang, Pierre Gergondet, Qiang Huang. **Bimanual Continuous Steering Wheel Turning by a Dual-Arm Robot**. IEEE/ASME Transactions on Mechatronics, 29(3):1773–1784, 2024。[DOI](https://doi.org/10.1109/TMECH.2023.3316634) | 直接对应双臂转方向盘的背景论文，特意保留，不能当成 2025/2026 新工作。本轮已用 Crossref 核对标题、作者、卷期页和 2024 年 6 月出版日期；未取得原文，不对其细节作复现承诺。 |

建议阅读顺序：[2] → [3] → [1] → [6]；拟做学习方法时再读 [5] → [7] → [8] → [4] → [9]，并获取 [10] 原文作为直接任务比较。

### 7. 检索范围与核验记录

- 检索组合包含 `bimanual steering wheel manipulation 2025 2026`、`dual-arm steering`、`bimanual closed chain constraints`、`bimanual relative pose diffusion`、`manipulability bimanual`、`reactive collision-free bimanual`，并沿硬约束规划的参考文献追踪至 2026 年 9 月新文。
- 当前会话没有技能所列的 academic-search MCP，采用 Web 搜索发现文献，回到 arXiv、CVF、PMLR、出版社、作者项目页核验；直接方向盘论文补用 Crossref REST 元数据。
- arXiv、Crossref、PubMed 端点预检通过；本任务未用 PubMed 检索。
- 部分 IEEE DOI 页面无法打开，CVF HTML 访问返回 403；CVPR 条目用其论文 PDF 搜索结果与 arXiv 页面互补，方向盘条目用 Crossref 核对。不将打不开的页面说成已精读。
- [1,2,5] 另访问了作者方法说明或 arXiv HTML；其他条目主要基于摘要、正式元数据及可见方法描述筛选。本报告不是十篇全文逐页精读。
- 不采用搜索引擎的“几个月前”来决定年份；同一工作预印本和会议版本合并，不重复计算。检索未发现经核验且比 [10] 更新的直接“方向盘转动”论文；这不证明不存在。
- 仓库 CodeGraph 未初始化，因此采用本地文档与定点源代码阅读；没有创建索引。

文献条目的 BibTeX 见同目录 `bimanual_yoke_references.bib`。其中会议条目只填已核验字段，预印本不虚构会议、卷期和页码。

<a id="part2"></a>

## 二、狭小空间双臂协同的开源仓库与补充论文

核查日期：2026-09-18。延续 `bimanual_yoke_review_2026-09-18.md`。本轮核查官方文档、论文元数据和部分源码，未安装或运行这些外部仓库；“适合”是针对 AVIATOR 的技术判断，不是实测性能排序。

### 结论

用户提供的 `uwgraphics/relaxed_ik_core` 默认分支为 `ranged-ik`，README 标题为 RangedIK Core，因此 RangedIK 与该链接不是两个独立候选。两个 TSR 链接是同一个项目。

建议分层选择：

| 层次 | 推荐工具 | 用途 |
|---|---|---|
| 任务表达 | TSR 或自定义 `(θ,s)` 参数化 | 表达物体允许的位姿、抓取前容差与固定抓取变换 |
| 全局路径 | OMPL constrained planning；Drake 闭链示例作为研究参考 | 狭窄通道搜索、绕障、IK 分支和路径连通性 |
| 局部优化 | Mink；PyRoki；自写集中式 QP | 跟踪、冗余姿态、几何一致性和局部避碰 |
| 执行验证 | AVIATOR 现有 MuJoCo 模型与控制通道 | 跟踪、闭链误差、碰撞、限位和超时检查 |

不是建议把所有库同时堆进项目。最小原型优先选 Mink；需要自动微分与轨迹优化实验时选 PyRoki；出现必须绕行、换分支的失败后再接全局规划。

### 1. RangedIK / RelaxedIK Core

- [仓库](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik)
- Wang, Yeping; Praveena, Pragathi; Rakita, Daniel; Gleicher, Michael. **RangedIK: An Optimization-based Robot Motion Generation Method for Ranged-Goal Tasks**. ICRA 2023, 9700–9706. [论文](https://arxiv.org/abs/2302.13935)

论文用加权多目标优化和范围损失表达精确目标、等价目标区间及带偏好区间。适合任务允许某些姿态或位置自由度变化的情形。它是局部运动生成方法，不能据此假定能找到狭窄空间中的全局绕行路径。

核查提交 `1c48d2ae408b4e024ee037641aac1e728267984e`：

- [`configs/example_settings/baxter.yaml`](https://github.com/uwgraphics/relaxed_ik_core/blob/1c48d2ae408b4e024ee037641aac1e728267984e/configs/example_settings/baxter.yaml) 配有左右两个末端，支持多链输入。
- [`src/groove/objective_master.rs`](https://github.com/uwgraphics/relaxed_ik_core/blob/1c48d2ae408b4e024ee037641aac1e728267984e/src/groove/objective_master.rs) 的 `relaxed_ik()` 按链添加末端目标，并包含关节运动平滑、可操作度等代价；其中环境碰撞项的加入语句被注释。
- 该构造中没有看到双臂刚性抓取的相对位姿等式。多链输入不能替代闭链约束。
- 这一局部核查不等于完整审计所有碰撞实现。若采用，应单独检查跨臂碰撞覆盖，不能只看到 SelfCollision 名称就视为完备。
- 仓库区分 RangedIK、RelaxedIK、CollisionIK；README 列出的 RelaxedIK MuJoCo wrapper 不代表 RangedIK 已有同等接口。

AVIATOR 建议：用于锁定前抓取姿态选优、允许容差的任务段或比较基线。固定 weld 后不应让两手分别自由改变抓取方向；这样会把不存在的机械自由度加入规划器。若任务允许误差，应优先在共享操纵盘任务坐标上表达，让两臂保持同一物体运动。

### 2. TSR

- [仓库](https://github.com/personalrobotics/tsr)
- Berenson, Dmitry; Srinivasa, Siddhartha; Kuffner, James. **Task Space Regions: A framework for pose-constrained manipulation planning**. IJRR 30(12), 1435–1460, 2011. DOI: 10.1177/0278364910396389. [作者机构全文](https://personalrobotics.cs.washington.edu/publications/berenson2011task.pdf)

TSR 用参考变换、末端偏置和六维位姿边界描述允许的末端位姿集合。可将不允许变化的维度宽度置零。原论文包含 CBiRRT2 规划框架；当前 Python 仓库主要提供表示、采样、距离、模板和 TSRChain 等能力，不能把安装此库等同于安装完整闭链全局规划器。

核查提交 `3377b14f88475393d1ae10d1633420337f5f5f10` 的 [`tsr_chain.py`](https://github.com/personalrobotics/tsr/blob/3377b14f88475393d1ae10d1633420337f5f5f10/src/tsr/tsr_chain.py)：TSRChain 是依次组合的位姿变换链。将左右手 TSR 放进一个列表，不会自动建立“左右手必须共享同一个操纵盘状态”的并联闭链条件。

本平台建议使用：

```text
共享 x = (θ, s)
T_L = T_W(x) G_L
T_R = T_W(x) G_R
```

两臂应共享同一次物体状态采样。若左右独立采样 θ 和 s，可能分别可达但彼此不相容。抓取前可额外搜索沿把手轴的位置及绕轴角；一旦固定 weld，G_L 与 G_R 固定。位置、角度和坐标轴约定须与当前抓取代理一致。

TSR 适合做任务表达层；狭窄通道、关节可达性和全身碰撞需要下游求解器。方法来自 2011 年，不因仓库维护而成为新方法。

### 3. PyRoki

- [仓库](https://github.com/chungmin99/pyroki)；[官方文档](https://chungmin99.github.io/pyroki/)
- Kim, Chung Min; Yi, Brent; Choi, Hongsuk; Ma, Yi; Goldberg, Ken; Kanazawa, Angjoo. **PyRoki: A Modular Toolkit for Robot Kinematic Optimization**. 2025；仓库给出 IROS 2025 引用。[论文](https://arxiv.org/abs/2505.03728)

Python/JAX 运动学优化工具，适合自定义残差与导数、联合双臂变量和短时域轨迹优化。官方文档说明可通过增广拉格朗日求解器表达硬约束，但最终仍要验证数值残差与求解状态。

核查提交 `388e43e1fc0d0ee382968d3dd72970fd62a0450c` 的[多目标 IK 示例](https://github.com/chungmin99/pyroki/blob/388e43e1fc0d0ee382968d3dd72970fd62a0450c/examples/pyroki_snippets/_solve_ik_with_multiple_targets.py)：将多个末端 pose cost 作用于同一个关节变量，加入 rest cost 和关节限位；没有自动建立固定物体抓取关系或完整座舱碰撞问题。

官方限制包括：没有采样式规划器；运动学模型不原生支持闭环机构；mesh 碰撞用 capsule 等几何近似；JIT 首次编译和形状变化有开销。这不排除对两条开链显式添加闭链等式，但需要自己实现。窄间隙中必须与 AVIATOR 原碰撞模型对照，避免近似几何堵塞实际通道或漏碰。

建议先看 `02_bimanual_ik.py`、`04_ik_with_coll.py`、`05_ik_with_manipulability.py`、`06_online_planning.py`、`07_trajopt.py`。适合做算法原型和离线轨迹优化，不应未经计时就放入现有 1 ms 伺服线程。

### 4. 更直接的代码候选

| 工具 | 已核验的能力 | 对 AVIATOR 的价值及适配边界 |
|---|---|---|
| [Mink](https://github.com/kevinzakka/mink) | MuJoCo 微分 IK、关节位置/速度限制、geom 碰撞、闭链 equality 支持、双臂示例 | 最值得先试；可沿用模型。当前 `solve_ik` 区分优化 tasks 和 equality constraints；需显式选择并验证，不能只加一个高权重任务。属于局部控制器。 |
| [OMPL](https://github.com/ompl/ompl) / [constrained planning 文档](https://ompl.kavrakilab.org/constrainedPlanning.html) | ProjectedStateSpace、AtlasStateSpace、TangentBundleStateSpace | 适合作为 C++ 全局基线。自定义闭链函数/雅可比与 MuJoCo 碰撞回调；需要调容差和插值步长，奇异点/高曲率可能使插值失败。 |
| [Drake 闭链双臂示例](https://github.com/cohnt/constrained-bimanual-planning-example) | 主从最小坐标、约束空间碰撞区域、RRT、GCS、轨迹优化和 TOPPRA | 与刚性双手协作高度相关，有 Python/C++ 示例。示例的 iiwa 解析 IK 不是 AVIATOR 两臂的现成解，迁移参数化是主要工作。 |
| [ROBOTIS cyclo_control](https://github.com/ROBOTIS-GIT/cyclo_control) / [官方说明](https://ai.robotis.com/ai_worker/advanced_motion_controller_ai_worker) | QP 约束控制、双臂 rigid grasp、虚拟物体命令、轨迹过滤 | 很适合参考集中式协同控制架构。依赖其机器人/ROS 接口，需要适配现有独立进程通信；还需加入操纵盘自身两自由度限制。 |
| [Closed-Chain Affordance](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance) | 绕轴转动、直线、螺旋操作的闭链轨迹模型；独立 C++ 库与 ROS2 wrapper | 非常贴近“转＋推拉”机构语义，但已展示的是操纵器—环境闭链，并非现成双臂共同抓持求解器；任意独立转动/推拉也不是固定螺距的单自由度螺旋。 |
| [Pink](https://github.com/pink-kinematics/pink) | Pinocchio＋QP 微分 IK；Mink 的上游思想来源 | 如计划用 Pinocchio 统一运动学可考虑；当前已有 MuJoCo 时优先评估 Mink。 |

Mink 特别注意：完整 AVIATOR 模型中包含两个无驱动物体关节。规划变量可以包含它们，但执行只能下发 14 个机械臂关节目标；不能为方便验证而直接写操纵盘实际状态或新增 actuator。应让它继续由双臂被动带动。

### 5. 与狭小空间更直接相关的补充论文

| 文献 | 年份/状态 | 相关点 | 代码状态 |
|---|---|---|---|
| **Dual-Arm Hierarchical Planning for Laboratory Automation: Vibratory Sieve Shaker Operations** — Haoran Xiao et al. [论文](https://arxiv.org/abs/2509.14531)；[IEEE](https://ieeexplore.ieee.org/document/11247197/) | IROS 2025 | 明确包含 3 cm 间隙的双臂盖子操作、交接和姿态约束运送；先验引导采样＋多步轨迹优化 | 本轮未核验到作者官方实现，不标为已开源 |
| **TCBiRRT: Rapid Motion Planning for Tightly Coupled Dual-arm Space Manipulator Using Task-space Random Expansion** — Jiawei Zhang, Xinhao Miao, Jifeng Guo, Qinghua Li, Chengchao Bai. [论文](https://arxiv.org/abs/2605.27167) | 2026-05-26 预印本 | 在物体任务空间扩展，映射为连续关节路径；含闭链、杂乱环境和重抓机制 | 未核验到官方代码；其重抓不能直接用于当前固定双 weld |
| **Constrained Bimanual Planning with Analytic Inverse Kinematics** — Thomas Cohn, Seiji Shaw, Max Simchowitz, Russ Tedrake. [论文](https://arxiv.org/abs/2309.08770) | ICRA 2024；预印本 2023 | 固定相对位姿的参数化、障碍环境全局规划 | [官方示例](https://github.com/cohnt/constrained-bimanual-planning-example)已核验 |
| **Faster Algorithms for Growing Collision-Free Regions of Constrained Bimanual Configuration Spaces** — Thomas Cohn, Peter Werner, Russ Tedrake. | IROS 2025 workshop，非主会论文 | 在闭链最小坐标中构建无碰区域 | 上述官方示例 README 列出该文，并附 workshop 材料 |
| **A Closed-Chain Approach to Generating Affordance Joint Trajectories for Robotic Manipulators** — Janak Panthi, Farshid Alambeigi, Mitch Pryor. [作者项目页](https://sites.utexas.edu/nrg/cca-planner/) | T-RO 2025 | 将阀门、抽屉等机构操作建模为闭链，考虑末端方向 | [独立 C++ 代码](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance)已核验 |
| **A Globally Guided Dual-Arm Reactive Motion Controller for Coordinated Self-Handover in a Confined Domestic Environment**. [原文入口](https://www.mdpi.com/2313-7673/9/10/629) | Biomimetics 2024, 9(10), 629 | 全局引导＋局部反应式控制，直接讨论受限环境中的双臂协作 | 未核验到官方代码；交接任务与始终共同抓持存在区别 |

上一份报告的 2026-09 可微流形图、2026-08 硬约束规划和 2026-02 反应式闭链避碰仍应保留。本轮增加的是可复用软件以及窄间隙和机构操作相关文献，并非按年份替换全部旧基础方法。

### 6. 推荐研究组合

**工程最短路径：Mink＋现有 MuJoCo 模型。** 建立共享物体参考、闭链等式和独立碰撞校验，与当前 TRAC-IK 轨迹作同场景比较。保留当前通信与执行通道，先离线或低频运行研究规划器。

**优化方法研究：PyRoki＋对象中心约束＋全局初值。** 同时优化 `(qL,qR,θ,s)` 或整段轨迹。对位置与转角误差归一化，设置明确的等式求解残差阈值。若使用软残差，则明确报告为近似闭链。

**狭窄通道与可达域研究：TSR/机构参数化＋OMPL constrained planning。** 不只更换 IK，而是研究共享物体坐标下的采样、连续分支和碰撞约束搜索。可与 Drake 最小坐标方法对照。

一个值得验证的具体选题是：**任务容差驱动的受限空间双臂闭链规划**。容差仅放在任务真正允许的共享物体状态或抓取前选择中；固定抓取闭链保持。比较相同任务精度预算下的成功率、最小碰撞距离、规划耗时和闭链残差。组合已有库本身不构成创新，需证明新的约束表达、搜索策略或泛化能力带来收益。

狭小空间实验必须区分：

1. 每个目标点存在 IK 解，但整条路径没有连续可行解。
2. 全局存在路径，但局部 IK/QP 被限位、障碍或不良初值困住。
3. 碰撞几何近似造成假阻塞或漏碰。
4. 指令轨迹闭链误差被 weld 的物理求解部分掩盖。

建议同一批任务比较当前 IK、RangedIK、Mink/PyRoki 约束局部方法、约束全局规划＋局部跟踪。统一任务容差和实际模型；不能让某方法额外放宽固定抓取关系却仍用相同成功标准。

### 核查范围

主仓库的三项快照 SHA 已记录。其他仓库主要核查 README、官方文档及链接；Mink 另核查 `src/mink/solve_ik.py` 的 `constraints` 参数和等式构建逻辑。没有运行外部仓库、没有评测延迟、没有承诺开箱即用。部分 GitHub 页面 Web 访问失败时，通过 GitHub API/原始源码补读；MDPI 全文入口遭遇限流，因此对应条目基于可检索摘要及出版信息，不声称精读全文。

<a id="part3"></a>

## 三、对偶四元数的用途、实现与参考文献

### 1. 是否有用

**有用，尤其适合当前固定抓取、不引入内力约束的双臂协同建模。** 建议把它放在运动学和控制层：统一描述左右手位姿、相对关系及几何约束，再由优化器计算关节运动。

对偶四元数是一种刚体位姿表示，不能仅凭更换表示就扩大机器人可达域、消除机构奇异性，或找到狭小空间中的绕障路径。它与齐次矩阵、SE(3) 李群方法都可用于表达同一刚体运动；选择应服务于建模清晰度和实现便利性，而非预设其一定更快、更精确。

### 2. 对平台最有价值的协同任务表达

双臂抓住同一个操纵盘时，区分两类量：

- 整体运动：操纵盘的转动和推拉目标。
- 相对关系：左右 TCP 之间的位姿关系保持不变。

这是协作双任务空间（Cooperative Dual-Task Space，CDTS）的核心思路。该框架最初使用对偶四元数建立，已有后续几何代数扩展。[CDTS 扩展论文](https://arxiv.org/abs/2310.19093)

用单位对偶四元数表示位姿，固定抓取的对象中心关系为：

$$
\hat X_L=\hat X_W(\theta,s)\hat G_L,\qquad
\hat X_R=\hat X_W(\theta,s)\hat G_R.
$$

因此：

$$
\hat X_L^{-1}\hat X_R=\hat G_L^{-1}\hat G_R.
$$

其中，左右手共享同一个操纵盘状态 `(θ,s)`；`G_L`、`G_R` 在锁定后固定。这些式子与第一部分的齐次变换表达等价。相对关系由前两式推出，不应无判断地把重复等式全部加入优化器。

经典 CDTS 的“绝对位姿”可由两臂位姿构造，并不必然等于物理操纵盘坐标系。实现时需定义二者的对应关系；已知物体模型时，直接使用操纵盘坐标系更容易对齐任务。

**只固定左右手相对位姿还不够。** 操纵盘受自身机构约束，只能转动和推拉，不能像自由搬运的箱子一样任意平移或倾斜。

### 3. 狭小空间中的三个用途

| 用途 | AVIATOR 上的作用 | 配套方法 |
|---|---|---|
| 相对位姿与协同雅可比 | 把两臂放入统一运动学控制问题，保持抓取关系 | QP/HQP、闭链误差反馈 |
| 点、线、平面几何关系 | 描述把手轴线、工具方向和座舱边界等约束 | 距离函数、碰撞几何及距离雅可比 |
| 冗余与任务优先级 | 在完成操纵盘运动时调整肘部，改善限位和碰撞余量 | 可操作度优化、分层控制、可行性恢复 |

对偶四元数控制相关工作中常用向量场不等式（Vector Field Inequalities，VFI），通过限制接近约束边界的速度，实现局部避碰或姿态限制，可以放入 QP，不要求使用内力反馈。[VFI 全身避碰论文](https://arxiv.org/abs/1906.07322)

VFI 也可以使用其他几何表示实现，不专属于对偶四元数。它不自动解决全局路径连通性；障碍、关节限位与闭链约束冲突时，仍可能需要减速、停止或全局重规划。复杂座舱碰撞不能只靠少数无限直线或平面代替完整几何验证。

### 4. 与其他工具的分工

| 方法/工具 | 主要负责什么 |
|---|---|
| TSR | 定义允许的任务位姿集合和容差范围 |
| 对偶四元数/CDTS | 表达刚体位姿、整体运动和双臂相对关系 |
| RangedIK | 利用任务允许范围生成局部运动 |
| Mink/PyRoki/自写 QP | 求解关节速度、姿态或轨迹 |
| OMPL/Drake | 搜索满足约束的绕障路径和连续构型 |
| MuJoCo/AVIATOR | 物理仿真、指令执行和实测验证 |

以上不是插件式兼容性承诺。库之间的位姿、雅可比、速度坐标系、关节顺序和时间接口仍要适配。尤其不能直接把对偶四元数导数雅可比与已有六维几何雅可比混用。

### 5. DQ Robotics 实现入口

- [DQ Robotics 官方网站](https://dqrobotics.github.io/)
- [C++ 库](https://github.com/dqrobotics/cpp)
- [Python 库](https://github.com/dqrobotics/python)
- [MATLAB 库](https://github.com/dqrobotics/matlab)
- [DQ_CooperativeDualTaskSpace 接口](https://raw.githubusercontent.com/dqrobotics/cpp/master/include/dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h)

官方库提供对偶四元数代数和机器人运动学计算，支持 C++、Python 和 MATLAB。`DQ_CooperativeDualTaskSpace` 可作为双臂协同建模的起点。[官方介绍](https://dqrobotics.github.io/)

建议保留现有 MuJoCo、通信和执行接口，先在上层增加协同误差与雅可比计算：

```text
操纵盘目标 (θ, s)
        ↓
左右 TCP 目标 + 固定抓取关系
        ↓
对偶四元数/CDTS 协同误差与雅可比
        ↓
闭链约束 + 限位 + 避碰 + 冗余目标的 QP
        ↓
14 轴机械臂指令
        ↓
MuJoCo 被动操纵盘运动与反馈
```

采用 DQ Robotics 自己的机器人模型时，先与现有 FK、法兰/TCP 变换、基座安装位姿逐项对齐。也可以保留现有 FK 后转换表示，但其雅可比转换必须正确推导和验证。本轮没有验证 DQ Robotics 与 AVIATOR 的现成适配器，也没有证明该控制器可以满足 1 ms 求解周期。

### 6. 实现和论文论证中的边界

1. 对偶四元数能避免欧拉角的表示奇异性，但不能消除机械臂的运动学奇异性。
2. 单位对偶四元数有约束，八个分量不是八个独立自由度。须处理单位约束、正负号等价以及轨迹上的符号连续性。
3. 不能把八维系数直接作为普通欧氏坐标随意插值；位姿误差中的平移与旋转也需合理尺度。
4. 螺旋插值不自动保证关节可达、无碰撞和闭链执行精度。操纵盘轨迹优先由实际机构坐标生成。
5. 仅把齐次矩阵换成对偶四元数，通常不足以构成论文创新。应比较在同样任务容差与几何模型下的可达性、协同误差、避碰和计算代价。
6. 用运动学闭链控制不等于实现内力控制，也不能仅根据位姿误差下降声称内力下降。

### 7. 对偶四元数与协同控制参考文献

| 编号 | 文献 | 阅读目的 |
|---|---|---|
| DQ-1 | **Dual Position Control Strategies using the Cooperative Dual Task Space**，IROS 2010。[作者全文](https://www.lirmm.fr/~druon/assets/pdf/2010_iros.pdf) | 理解双臂绝对/相对任务表达 |
| DQ-2 | Bruno Vilhena Adorno, Murilo Marques Marinho. **DQ Robotics: A Library for Robot Modeling and Control**. IEEE Robotics & Automation Magazine, 28(3):102–116, 2021；预印本 2019。DOI: 10.1109/MRA.2020.2997920。[论文](https://arxiv.org/abs/1910.11612)；[官方引用](https://dqrobotics.github.io/) | 了解工具与实现；期刊年份按官方条目，不从 DOI 年份推断 |
| DQ-3 | Juan José Quiroz-Omaña, Bruno Vilhena Adorno. **Whole-Body Control With (Self) Collision Avoidance Using Vector Field Inequalities**. IEEE RA-L, 4(4):4048–4053, 2019。DOI: 10.1109/LRA.2019.2928783。[论文](https://arxiv.org/abs/1906.07322) | 将几何关系和避碰不等式接入控制 |
| DQ-4 | Tobias Löw, Sylvain Calinon. **Extending the Cooperative Dual-Task Space in Conformal Geometric Algebra**. ICRA 2024；预印本 2023。DOI: 10.1109/ICRA57147.2024.10610558。[论文](https://arxiv.org/abs/2310.19093) | 了解 CDTS 的共形几何代数扩展及 MPC 实验；不能把它称为纯对偶四元数方法 |

这些是基础和延伸阅读，并非全部为最新论文。近期控制热点可与第一部分的 2026 年闭链规划和反应式避碰工作结合。

<a id="part4"></a>

## 四、统一技术路线与实施顺序

### 1. 三条可选主线

| 主线 | 方法组合 | 适合解决的问题 |
|---|---|---|
| 模型控制 | 对象中心/CDTS + 闭链 QP + 方向可操作度 + 局部避碰 | 跟踪协同性、冗余姿态、限位和动态障碍 |
| 全局约束规划 | TSR/机构参数化 + OMPL 或 Drake 思路 + 局部跟踪 | 狭窄通道、连续 IK 分支、全局路径连通性 |
| 学习辅助 | 受约束生成策略/残差策略 + 可行性投影 + 模型控制 | 多布局泛化、规划初值、数据驱动的冗余或时间选择 |

对偶四元数可以服务于前两条主线，并为第三条提供几何约束；是否采用它，不改变固定抓取和两自由度机构本身的要求。

### 2. 建议实施顺序

1. **建立基线与日志。** 保留当前 TRAC-IK 方法，记录实际配置、完整指令轨迹、实测状态、闭链误差与碰撞距离。首先复核现有测试条件，不把历史日志当作新算法验证结果。
2. **建立协同局部控制原型。** 优先评估 Mink 沿用 MuJoCo 模型，或用已有运动学实现集中式 QP。任务变量是共享 `(θ,s)`，固定抓取关系不能随意放宽。
3. **按建模需要引入 CDTS。** 如采用对偶四元数，先验证 FK 和雅可比转换，再做闭链约束与冗余控制。与同样约束下的 SE(3) 表示对照，而非默认它优于其他表示。
4. **加入任务方向可操作度。** 保留 J2 窄范围和速度限位，改善转动/推拉方向的双臂瓶颈能力。
5. **针对全局失败引入规划器。** 确认局部死锁或分支问题后，用约束采样或最小坐标规划提供可行路径；全局路径的后处理、插值和时间参数化仍需闭链校验。
6. **扩展联合轨迹优化。** 需要自动微分、批量实验或短时域优化时评估 PyRoki；对窄间隙进行原碰撞模型复核。
7. **有足够任务多样性后再研究学习。** 用模型规划器生成合法示范，学习冗余偏好、低维动作或初值，保留输出可行性检查和独立测试布局。

### 3. 推荐选题

- **优先选题：面向受限座舱操纵盘的双臂闭链约束保持规划与任务方向可操作度优化。**
- **容差方向：任务容差驱动的受限空间双臂闭链规划。** 只使用真实任务允许的自由度，不放松不存在的抓取自由度。
- **协同控制方向：基于协作双任务空间的双臂操纵盘约束控制与几何避碰。** 对偶四元数是可选建模工具，贡献应落在控制、可行性处理或实验结果上。
- **学习方向：面向两自由度操纵机构的对象中心生成策略与闭链可行性投影。** 重点是约束保持和跨布局泛化，而非复现单一路径。

以上是待验证的研究方向，不是已经成立的新颖性结论。最终论文选题需进一步精读最接近的工作，确定差异、基线和证据范围。

<a id="part5"></a>

## 五、文档、代码入口与文献库

### 本仓库入口

- [控制与仿真说明](../../AviatorRobot/README.md)
- [控制器实现](../../AviatorRobot/src/Aviator.cpp)
- [控制接口](../../AviatorRobot/include/aviator/Aviator.hpp)
- [运行配置](../../AviatorRobot/config/aviator.yaml)
- [抓取配置](../../AviatorRobot/config/grasp.json)
- [姿态配置](../../AviatorRobot/config/posture.json)
- [抓取搜索工具](../../AviatorRobot/tools/search_grasp.cpp)
- [既有验证记录说明](../../AviatorRobot/validation/cylinder_grasp/README.md)

### 调研与引用文件

- [本综合文档](AVIATOR_双臂协同操作研究整理.md)：建议作为统一阅读入口。
- [第一轮方案与近期文献报告](bimanual_yoke_review_2026-09-18.md)：保留原始专题记录。
- [第二轮仓库与狭小空间论文报告](constrained_bimanual_repositories_2026-09-18.md)：保留源码快照与适配分析。
- [核心参考文献 BibTeX](bimanual_yoke_references.bib)：10 条。
- [仓库与狭小空间补充 BibTeX](constrained_bimanual_supplement.bib)：7 条。

对偶四元数的 4 篇参考文献已在本综合文档第三部分记录；未包含在上述两个既有 BibTeX 文件中。其他正文列出的补充论文也不一定全部已导出为 BibTeX，应以各文件实际内容为准。

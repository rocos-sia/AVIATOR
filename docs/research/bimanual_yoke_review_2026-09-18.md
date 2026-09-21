# AVIATOR 双臂操纵盘协同操作调研

检索日期：2026-09-18。重点：2025—2026 年工作；最新收录预印本为 2026-09-09。本文是针对当前平台的定向调研，不是穷尽性系统综述。主张与适配建议分开陈述，未运行新控制实验。

**建议主线：面向受限座舱两自由度操纵盘的闭链运动学协同、任务方向可操作度优化与约束保持在线重规划。** 先建立可靠模型方法，再考虑学习规划器的初值、冗余姿态或低维动作。当前无需把内力约束作为核心。

## 1. 当前平台能支持什么

依据 `AviatorRobot/README.md`、`src/Aviator.cpp`、`config/aviator.yaml`、`validation/cylinder_grasp/README.md`：

- 左右各 7 自由度，共 14 轴位置指令；操纵盘转动和推拉两轴无 actuator，由双臂经两个 weld 被动带动。
- 这更接近航空操纵盘（yoke），不是可无限连续转动的汽车方向盘。模型工作范围约为转角 ±50°、推拉 −165～0 mm；已有验证路径包含 ±50° 转动和 −160 mm 推拉，不能据此认定整个转角—位移矩形均可达。
- `MoveWheel()` 用统一平滑相位插值转角和位移；`target()` 由操纵盘位姿生成左右法兰目标；`solve()` 分别解两臂 IK，以上一时刻解作种子；随后统一检查关节限位、速度和碰撞。
- 两侧 J2 的任务限制为 85°～95°，规划留出 0.5° 余量。冗余优化应保留该约束，不能靠放宽它获得不公平优势。
- `execute()` 在离散规划点之间线性插值关节角。端点满足闭链关系并不意味着插值期间严格满足；`validate()` 检查中点碰撞，但不是连续闭链误差证明。这是研究切入点，不是已经测得的失效。
- 已有抓取前的离线位置/朝向网格搜索；锁定后保持完整相对位姿，不允许沿握持轴自由转动。现有执行要求两侧同时保持锁定。
- README 的速度说明与当前 YAML 有差异：YAML 实际 `joint_speed: 6.28`，代码取其与 URDF 速度上限的较小值。实验必须记录实际配置，不能直接沿用 README 的 0.7 rad/s。

**证据边界：** weld 仍会在物理求解器中产生约束反力。“不做内力控制”不等于“没有内力”。本平台适合检验几何一致性、跟踪、可达性和避碰；仅凭 weld 仿真不能证明真实夹爪防滑、内力降低或柔顺接触稳定性。

## 2. 不含内力项的统一问题表述

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

## 3. 可探索方案与优先级

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

## 4. 近期热点及平台适配

**热点一：把协同关系作为规划空间本身的约束。** 2026 年的快速硬约束规划和可微流形图工作，分别关注约束保持搜索及 IK 参数化梯度。[1,2] 对本平台最有价值的是两自由度机构约束与双臂冗余的联合处理。两篇工作主要涉及共同搬运，迁移到操纵盘还需要加入物体—基座的运动限制。

**热点二：协同控制、可操作度和反应式避碰结合。** 2026 年 Chen 等将协作任务空间、可操作度参考与 CBF 过滤相结合。[3] 可借鉴结构，但本平台操纵盘不能像托盘一样任意调整空间姿态，避碰自由度更少。

**热点三：生成策略显式结合运动学和约束。** ADCS 处理相对/绝对位姿及不等式约束；KStar 使用机器人图结构与运动学正则；SafeBimanual 在推理中优化安全代价。[5,7,8] 学到的约束偏好不等同于严格可行性保证，仍应检查或投影输出。

**热点四：姿态和坐标系成为策略的一部分。** ManiDP 关注姿态相关的双臂能力；MoF 研究不同坐标系中动作分布的组合。[4,6] 本任务可比较世界系、操纵盘系和双臂相对系表示；对于纯两自由度跟踪，复杂网络是否必要必须用简单模型基线回答。

**热点五：协同数据的可行增强。** D-CODA 同时增强两侧腕部视觉和关节动作，并通过约束优化保持协调。[9] 当前可以先从规划器导出状态轨迹，但这只是受启发的状态数据生成，不等于复现 D-CODA 的视觉方法。

## 5. 推荐选题与实验设计

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

## 6. 核心参考文献（近期 9 篇＋直接任务背景 1 篇）

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

## 7. 检索范围与核验记录

- 检索组合包含 `bimanual steering wheel manipulation 2025 2026`、`dual-arm steering`、`bimanual closed chain constraints`、`bimanual relative pose diffusion`、`manipulability bimanual`、`reactive collision-free bimanual`，并沿硬约束规划的参考文献追踪至 2026 年 9 月新文。
- 当前会话没有技能所列的 academic-search MCP，采用 Web 搜索发现文献，回到 arXiv、CVF、PMLR、出版社、作者项目页核验；直接方向盘论文补用 Crossref REST 元数据。
- arXiv、Crossref、PubMed 端点预检通过；本任务未用 PubMed 检索。
- 部分 IEEE DOI 页面无法打开，CVF HTML 访问返回 403；CVPR 条目用其论文 PDF 搜索结果与 arXiv 页面互补，方向盘条目用 Crossref 核对。不将打不开的页面说成已精读。
- [1,2,5] 另访问了作者方法说明或 arXiv HTML；其他条目主要基于摘要、正式元数据及可见方法描述筛选。本报告不是十篇全文逐页精读。
- 不采用搜索引擎的“几个月前”来决定年份；同一工作预印本和会议版本合并，不重复计算。检索未发现经核验且比 [10] 更新的直接“方向盘转动”论文；这不证明不存在。
- 仓库 CodeGraph 未初始化，因此采用本地文档与定点源代码阅读；没有创建索引。

文献条目的 BibTeX 见同目录 `bimanual_yoke_references.bib`。其中会议条目只填已核验字段，预印本不虚构会议、卷期和页码。

# 狭小空间双臂协同：RangedIK、TSR、PyRoki 与补充仓库

核查日期：2026-09-18。延续 `bimanual_yoke_review_2026-09-18.md`。本轮核查官方文档、论文元数据和部分源码，未安装或运行这些外部仓库；“适合”是针对 AVIATOR 的技术判断，不是实测性能排序。

## 结论

用户提供的 `uwgraphics/relaxed_ik_core` 默认分支为 `ranged-ik`，README 标题为 RangedIK Core，因此 RangedIK 与该链接不是两个独立候选。两个 TSR 链接是同一个项目。

建议分层选择：

| 层次 | 推荐工具 | 用途 |
|---|---|---|
| 任务表达 | TSR 或自定义 `(θ,s)` 参数化 | 表达物体允许的位姿、抓取前容差与固定抓取变换 |
| 全局路径 | OMPL constrained planning；Drake 闭链示例作为研究参考 | 狭窄通道搜索、绕障、IK 分支和路径连通性 |
| 局部优化 | Mink；PyRoki；自写集中式 QP | 跟踪、冗余姿态、几何一致性和局部避碰 |
| 执行验证 | AVIATOR 现有 MuJoCo 模型与控制通道 | 跟踪、闭链误差、碰撞、限位和超时检查 |

不是建议把所有库同时堆进项目。最小原型优先选 Mink；需要自动微分与轨迹优化实验时选 PyRoki；出现必须绕行、换分支的失败后再接全局规划。

## 1. RangedIK / RelaxedIK Core

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

## 2. TSR

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

## 3. PyRoki

- [仓库](https://github.com/chungmin99/pyroki)；[官方文档](https://chungmin99.github.io/pyroki/)
- Kim, Chung Min; Yi, Brent; Choi, Hongsuk; Ma, Yi; Goldberg, Ken; Kanazawa, Angjoo. **PyRoki: A Modular Toolkit for Robot Kinematic Optimization**. 2025；仓库给出 IROS 2025 引用。[论文](https://arxiv.org/abs/2505.03728)

Python/JAX 运动学优化工具，适合自定义残差与导数、联合双臂变量和短时域轨迹优化。官方文档说明可通过增广拉格朗日求解器表达硬约束，但最终仍要验证数值残差与求解状态。

核查提交 `388e43e1fc0d0ee382968d3dd72970fd62a0450c` 的[多目标 IK 示例](https://github.com/chungmin99/pyroki/blob/388e43e1fc0d0ee382968d3dd72970fd62a0450c/examples/pyroki_snippets/_solve_ik_with_multiple_targets.py)：将多个末端 pose cost 作用于同一个关节变量，加入 rest cost 和关节限位；没有自动建立固定物体抓取关系或完整座舱碰撞问题。

官方限制包括：没有采样式规划器；运动学模型不原生支持闭环机构；mesh 碰撞用 capsule 等几何近似；JIT 首次编译和形状变化有开销。这不排除对两条开链显式添加闭链等式，但需要自己实现。窄间隙中必须与 AVIATOR 原碰撞模型对照，避免近似几何堵塞实际通道或漏碰。

建议先看 `02_bimanual_ik.py`、`04_ik_with_coll.py`、`05_ik_with_manipulability.py`、`06_online_planning.py`、`07_trajopt.py`。适合做算法原型和离线轨迹优化，不应未经计时就放入现有 1 ms 伺服线程。

## 4. 更直接的代码候选

| 工具 | 已核验的能力 | 对 AVIATOR 的价值及适配边界 |
|---|---|---|
| [Mink](https://github.com/kevinzakka/mink) | MuJoCo 微分 IK、关节位置/速度限制、geom 碰撞、闭链 equality 支持、双臂示例 | 最值得先试；可沿用模型。当前 `solve_ik` 区分优化 tasks 和 equality constraints；需显式选择并验证，不能只加一个高权重任务。属于局部控制器。 |
| [OMPL](https://github.com/ompl/ompl) / [constrained planning 文档](https://ompl.kavrakilab.org/constrainedPlanning.html) | ProjectedStateSpace、AtlasStateSpace、TangentBundleStateSpace | 适合作为 C++ 全局基线。自定义闭链函数/雅可比与 MuJoCo 碰撞回调；需要调容差和插值步长，奇异点/高曲率可能使插值失败。 |
| [Drake 闭链双臂示例](https://github.com/cohnt/constrained-bimanual-planning-example) | 主从最小坐标、约束空间碰撞区域、RRT、GCS、轨迹优化和 TOPPRA | 与刚性双手协作高度相关，有 Python/C++ 示例。示例的 iiwa 解析 IK 不是 AVIATOR 两臂的现成解，迁移参数化是主要工作。 |
| [ROBOTIS cyclo_control](https://github.com/ROBOTIS-GIT/cyclo_control) / [官方说明](https://ai.robotis.com/ai_worker/advanced_motion_controller_ai_worker) | QP 约束控制、双臂 rigid grasp、虚拟物体命令、轨迹过滤 | 很适合参考集中式协同控制架构。依赖其机器人/ROS 接口，需要适配现有独立进程通信；还需加入操纵盘自身两自由度限制。 |
| [Closed-Chain Affordance](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance) | 绕轴转动、直线、螺旋操作的闭链轨迹模型；独立 C++ 库与 ROS2 wrapper | 非常贴近“转＋推拉”机构语义，但已展示的是操纵器—环境闭链，并非现成双臂共同抓持求解器；任意独立转动/推拉也不是固定螺距的单自由度螺旋。 |
| [Pink](https://github.com/pink-kinematics/pink) | Pinocchio＋QP 微分 IK；Mink 的上游思想来源 | 如计划用 Pinocchio 统一运动学可考虑；当前已有 MuJoCo 时优先评估 Mink。 |

Mink 特别注意：完整 AVIATOR 模型中包含两个无驱动物体关节。规划变量可以包含它们，但执行只能下发 14 个机械臂关节目标；不能为方便验证而直接写操纵盘实际状态或新增 actuator。应让它继续由双臂被动带动。

## 5. 与狭小空间更直接相关的补充论文

| 文献 | 年份/状态 | 相关点 | 代码状态 |
|---|---|---|---|
| **Dual-Arm Hierarchical Planning for Laboratory Automation: Vibratory Sieve Shaker Operations** — Haoran Xiao et al. [论文](https://arxiv.org/abs/2509.14531)；[IEEE](https://ieeexplore.ieee.org/document/11247197/) | IROS 2025 | 明确包含 3 cm 间隙的双臂盖子操作、交接和姿态约束运送；先验引导采样＋多步轨迹优化 | 本轮未核验到作者官方实现，不标为已开源 |
| **TCBiRRT: Rapid Motion Planning for Tightly Coupled Dual-arm Space Manipulator Using Task-space Random Expansion** — Jiawei Zhang, Xinhao Miao, Jifeng Guo, Qinghua Li, Chengchao Bai. [论文](https://arxiv.org/abs/2605.27167) | 2026-05-26 预印本 | 在物体任务空间扩展，映射为连续关节路径；含闭链、杂乱环境和重抓机制 | 未核验到官方代码；其重抓不能直接用于当前固定双 weld |
| **Constrained Bimanual Planning with Analytic Inverse Kinematics** — Thomas Cohn, Seiji Shaw, Max Simchowitz, Russ Tedrake. [论文](https://arxiv.org/abs/2309.08770) | ICRA 2024；预印本 2023 | 固定相对位姿的参数化、障碍环境全局规划 | [官方示例](https://github.com/cohnt/constrained-bimanual-planning-example)已核验 |
| **Faster Algorithms for Growing Collision-Free Regions of Constrained Bimanual Configuration Spaces** — Thomas Cohn, Peter Werner, Russ Tedrake. | IROS 2025 workshop，非主会论文 | 在闭链最小坐标中构建无碰区域 | 上述官方示例 README 列出该文，并附 workshop 材料 |
| **A Closed-Chain Approach to Generating Affordance Joint Trajectories for Robotic Manipulators** — Janak Panthi, Farshid Alambeigi, Mitch Pryor. [作者项目页](https://sites.utexas.edu/nrg/cca-planner/) | T-RO 2025 | 将阀门、抽屉等机构操作建模为闭链，考虑末端方向 | [独立 C++ 代码](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance)已核验 |
| **A Globally Guided Dual-Arm Reactive Motion Controller for Coordinated Self-Handover in a Confined Domestic Environment**. [原文入口](https://www.mdpi.com/2313-7673/9/10/629) | Biomimetics 2024, 9(10), 629 | 全局引导＋局部反应式控制，直接讨论受限环境中的双臂协作 | 未核验到官方代码；交接任务与始终共同抓持存在区别 |

上一份报告的 2026-09 可微流形图、2026-08 硬约束规划和 2026-02 反应式闭链避碰仍应保留。本轮增加的是可复用软件以及窄间隙和机构操作相关文献，并非按年份替换全部旧基础方法。

## 6. 推荐研究组合

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

## 核查范围

主仓库的三项快照 SHA 已记录。其他仓库主要核查 README、官方文档及链接；Mink 另核查 `src/mink/solve_ik.py` 的 `constraints` 参数和等式构建逻辑。没有运行外部仓库、没有评测延迟、没有承诺开箱即用。部分 GitHub 页面 Web 访问失败时，通过 GitHub API/原始源码补读；MDPI 全文入口遭遇限流，因此对应条目基于可检索摘要及出版信息，不声称精读全文。

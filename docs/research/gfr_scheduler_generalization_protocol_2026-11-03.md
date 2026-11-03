# 修复执行器上的调度器增量与 G1–G3 泛化评估

## 目标与冻结内容

先在开发集完成四种调度器小规模配对，再冻结并评估独立测试。固定 seed11 最终 checkpoint_60000；不重训、不选择新 checkpoint。后端固定为 multi-distance + affine-preserving backtracking 的 combined 执行器，保留 midpoint 修正、原 governor、非线性 endpoint/swept 检查、dt=0.005、d_safe=0.005、beta_stop=0.125、r_max=qdot_max=1.5。所有比较使用同一条轨迹、模型、初始 q、时间尺度和安全规则。

## 四种调度器

- `fixed`：名义 r=0；实际动作仍允许同一安全执行器投影和恢复。
- `greedy`：以 1/s 增益将关节中心方向投影到当前零空间，r=clip(Nᵀ(q_mid-q), ±r_max)。这是明确的在线关节居中启发式，不冒称旧 LUT phase-center。
- `teacher`：沿用成功示教的 previous/zero-first 规则；所有 beta 都失败后才扩展25个候选，按可行 beta、clearance 排序。候选检查使用当前 repaired backend；已验证动作仅消费一次。调度器搜索开销和执行器干预分别统计。
- `rl`：冻结60k策略，argmax确定性推理，原40维观测。

## 阶段与数据边界

1. 开发：原 test/traj_0010–0013，4种开发扰动模型，各方法完整执行4条。共16 episodes。这些案例已用于诊断，不作为泛化证据。
2. 接口验收：模型切换后调用计数不漏项、所有方法起点完全一致、接受动作满足冻结checker、文件哈希不变。负性能结果照常保留；不以RL胜出作为测试集选择条件。
3. 在开发运行完成后生成正式 selection_protocol.json，再执行G1–G3。按历史记录与内容签名排除已使用数据，不根据策略表现更换案例。
   - G1：30条新轨迹，两个训练模型 nominal/fixture translation1mm，轮转分配。
   - G2：30条示教覆盖任务，未见扰动模型。
   - G3：另外30条新轨迹，未见扰动模型。
4. G2/G3 每组6模型各5条：fixture translation0.4/0.7mm；base differential translation0.7/1.3mm；fixture rotation0.15deg；base differential rotation0.15deg。方向与既有N16模型定义一致。前两者属于训练误差类型内的新幅值，其余属于未训练误差类型，分别报告；还需对旧模型编译后的几何数组做重复审计。
5. 起点沿用LUT中点＋12次Newton修正。无效起点保留并单独报告，不更换、不调整轨迹速度；有效起点条件下成功率与整体任务成功率应分清。

研究信息条件：当前模型已标定，所有方法得到相同模型。不是未知安装误差识别。当前只验证单个训练seed的迁移，不能声称跨训练seed稳定性。

## 计时与指标

CPU串行、BLAS单线程；所有方法创建相同SAC对象并加载同一checkpoint，只有RL使用它决策。各方法/组先100个预热控制步，重置后从头评估；首次策略编译单独记录。完整周期包含调度、执行、下一观测。JSONL写入与额外审计查询放在计时之外，单独记录整个episode墙钟时间。

保存每步 inclusive 和 exclusive 耗时、函数调用次数。inclusive 存在嵌套，不可相加；exclusive之和等于完整周期。统计策略、lift、候选、distance pairs、distance checker、polygon/projection、swept、observation。Teacher全部搜索候选计入总成本；不重复校验选中候选以人为放大teacher成本。

报告完成率、progress、接受步的端点/扫掠最小clearance、闭合误差、intervention、替代恢复、候选/距离/swept调用次数及单位进度成本、全周期P50/P95/P99、完整任务计算耗时。无动作执行的失败末步不计仿真运动时间，但计入计算耗时。

加速比较只在双方均完成的配对任务上计算任务计算成本比，同时报告全体失败与进度，防止提前失败显得更快。单周期加速与任务级加速分开。每组逐模型与误差类型分层汇总。单seed与小模型数量限制必须保留。

## 输出、执行与停止

输出根目录：`outputs/differential_lift_feasibility_20261103/gfr_scheduler_generalization/`。

`study/run_status.json` 记录当前组、方法、episode、step、progress和进程。每episode立即写JSONL，子进程异常或保护输入改变即停止，保留全部原始数据。正式共360 episodes，加开发16 episodes。CPU串行，先用开发耗时估算正式耗时，不盲目并行。

脚本：`hil-serl/tools/gfr1_run_scheduler_study.py`。运行完成输出对照CSV、逐模型/分层JSON、配对差异与描述性bootstrap区间、报告。全成功/全失败时经验bootstrap会退化，不能把零宽区间当作总体保证。

结果不会自动触发重训。根据当前后端下的学习增量、独立测试下降与耗时构成，再决定多seed或降低策略调用频率；安全检查不能因此跳过。

# 修复执行器后的 SAC 从头训练（2026-11-02）

## 本轮目标

在修复后的执行器中从随机 SAC Actor/Critic 开始训练，观察策略能否完成任务，并减少替代动作恢复、边界修正和候选检查成本。不加载旧 40k、BC 或短预检 checkpoint。

## 已验证的前置条件

- 修复组合在此前全程回归中得到验证；本轮直接复用冻结的 `select_action`，包括最多 3 次 joint-limit-only 积分缺陷补偿，以及策略 governor 耗尽后才触发的替代动作搜索。
- 43 项相关测试通过，包括训练 backend 与冻结选择器同状态一致、独立 checker 验证、失败 reward/terminal 一致，以及 reset 随机状态不受恢复 RNG 消耗影响。
- 保留 100 条完整成功示范、302,125 个 transitions；buffer 哈希不变。
- 新核验覆盖每个示范任务的 reset 和第一步：100/100 一致，观测最大差为 0，奖励最大差约 2.29e-10。此检查不等于重新回放全部历史 transitions；轨迹内部仍使用既有 teacher 接受记录的 provenance。
- 128 transitions 的 CUDA 短预检结束，Actor/Learner 返回码均为 0，保存 checkpoint_64/128。预检 checkpoint 不参与正式训练初始化。
- 短预检日志验证 batch_size=256，其中 demo=128，online=128；256 是 minibatch 大小，不是训练步数上限。

## 正式冻结配置

| 项目 | 值 |
|---|---|
| 初始化 | 随机 SAC，无 BC/旧 SAC 权重 |
| seed | 11 |
| 执行器 | combined_repair_v1 |
| 并行接口 | 16 个逻辑环境，几何 step 顺序执行，GPU 批量策略推理 |
| 示范/在线 minibatch | 50% / 50% |
| 示范数 | 100 条完整成功轨迹 |
| 在线预算 | 最多 60,000 transitions；400 episodes 为次级目标，报告实际数量 |
| critic clipping | 1.0 |
| B4 correction weight | 0.1 |
| dt / d_safe / beta_stop | 0.005 s / 0.005 m / 0.125 |
| checkpoint | 每 5k |
| 训练设备 | CUDA:0 |
| 评估设备 | CPU 独立进程 |

几何、阈值、reward 系数和 40 维观测语义保持不变。训练 replay 保存策略原始动作；实际接受动作保存在 next observation，并用于原有 B4 辅助监督。不能用替代恢复动作覆盖 policy action 后再假装它是策略自己选择的动作。

沿用历史 reward 的 candidate_count 计费口径，同时单独记录实际候选次数（含边界修正）。沿用历史 accepted-step recovery 观测位语义；真正的替代恢复单列 `alternative_recovered`，不把旧 `recovered` 字段当作本轮替代恢复次数。此次不同时修改 reward 或增加观测，后续如改动需独立消融。

## 20k/40k 门控

- 20k 保存完成后启动固定 10 个 development-val episodes 评估，训练继续。
- 40k 保存完成后启动相同 10 个 episodes 评估，并等待 20k/40k 两份报告。
- 两次 completion 都为 0：停止，不进入 60k。
- 至少一次 completion 非零：允许继续 60k，但不等于收敛。
- 评估失败、缺失报告、未知仪器错误或接受动作违反冻结 checker 契约，均不得当作 completion=0 的正常实验结果。
- development-val 已参与此前诊断，不称为全新 held-out 泛化证据。
- 11 月 1 日的新 test10 数据不用于此次训练、demo、门控或 checkpoint 选择。

## 必须阅读的结果

completion、完整 progress、实际执行 clearance、intervention、真实 endpoint/swept 失败候选数、替代恢复次数、边界修正次数、候选评估数、停止原因、Actor/Critic/entropy/temperature 诊断。

修复器可能让弱策略也能完成全程。因此，即使 completion 达到 100%，仍需观察恢复和单位任务进度的候选成本，才能判断 RL 的新增作用。单个 seed 不能支撑稳定性或泛化结论。

## 代码与输出

新代码均位于 `hil-serl/tools/`：

- `gfr1_repaired_env.py`：复用冻结修复器的训练环境适配；拒绝状态保留负奖励和 terminal。
- `gfr1_repaired_entry.py`：训练/评估进程内替换 backend，不改历史配置。
- `run_gfr1_repaired_formal.py`：独立 fresh SAC Actor/Learner 启动器。
- `gfr1_train_repaired_gate.py`：100-demo 审计与异步评估门控。
- `gfr1_repaired_eval.py`：完整轨迹评估，增加替代恢复/边界修正统计。
- `gfr1_audit_repaired_demos.py`、`test_gfr_repaired_training.py`：兼容性和接口测试。

输出根目录：`outputs/differential_lift_feasibility_20261102/gfr_repaired_scratch/`。

- `demo_compatibility_audit.json`：示范兼容性检查。
- `smoke/`：128 步基础设施预检，非学习有效性实验。
- `formal_seed11/frozen_config.json`：参数、来源及代码哈希。
- `formal_seed11/run_status.json`：监督器实时状态。
- `formal_seed11/training/b4_seed11/`：checkpoint、Actor/Learner 与训练诊断日志。
- `formal_seed11/executor_traces/`：每个环境的逐步修复与安全统计。
- `formal_seed11/evaluations/`：独立 checkpoint 评估。

本记录在正式任务启动时写入；最终 completion 和训练结论以评估 JSON 为准。

## 训练与门控完成结果（2026-11-03 核验）

正式训练跑满 60,000 online transitions；Actor、Learner 和三个 checkpoint evaluator 返回码均为 0。训练于北京时间 11 月 2 日约 17:54 结束，最后评估于约 18:28 完成。实际启动 36 个在线 episodes，未宣称完成 400 个 episodes。

| Checkpoint | 完整完成 | 平均 progress | 平均替代恢复次数 | 平均边界修正次数 | 平均干预步数 |
|---|---:|---:|---:|---:|---:|
| 20k | 10/10 | 1.0 | 0.4 | 408.3 | 747.5 |
| 40k | 10/10 | 1.0 | 23.6 | 312.2 | 1274.0 |
| 60k | 10/10 | 1.0 | 0.0 | 593.9 | 1019.2 |

三次报告 instrumentation_unknown=0，accepted_validation_violations=0，全部 termination=end。该结论指既定端点/离散 swept 检查契约，不是连续时间安全保证。

最终 60k 在这 10 个 development-val episodes 不需要替代动作搜索，但仍依赖投影、调速和积分边界修正；不能写成不需要安全执行器。三个 checkpoint 的完整完成率一致，干预成本不单调下降，因此不能仅凭完成率断言优化完全收敛。未启动新的测试或训练。

原始结果：`formal_seed11/gfr1_async_gate_summary.json` 以及 `formal_seed11/evaluations/checkpoint_{20000,40000,60000}.json`。

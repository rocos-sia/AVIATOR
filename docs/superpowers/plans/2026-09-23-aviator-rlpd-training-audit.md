# Aviator RLPD 训练退化核查（2026-09-23）

依据：`2026-09-22-aviator-v01-rl-training.md`，当前提交 `f71153605`，实际 DP CSV/pkl、启动脚本、learner/actor 日志。此次仅诊断，未修改训练代码、权重、数据或 checkpoint；未重跑 50 条最终评估。用户提供的 76%→16% 表格作为待解释的观测，不作为单一根因的实验性证明。

## 结论

存在已验证的奖励定义不一致、replay 动作语义错误、训练轨迹覆盖不足，以及 episode 重置顺序错误。高尺度加速度惩罚和零奖励失败终止进一步造成目标错位。现有证据不支持把退化仅归因于冻结权重或 `/dt²`，也不足以断言策略已经明确学会“主动自杀”。

## 1. Demo 与在线环境不是同一套奖励（最高优先级）

- `reward.py:75–84`：加速度项 `/dt²`；jerk 项是相速度二阶差分的平方 `/dt⁴`。
- `build_dp_dataset.py:193–198`：从第三个 transition 起传入 `a_prev2`，启用 `w_j=0.05` 的 jerk 惩罚。
- `env.py:216–220`：始终 `a_prev2=None`，在线 jerk 恒为零。
- `train_rlpd.py:293–338`：demo 与在线各取一半；`489–500` 加载所有 demo，BC 的零动作过滤设置不适用于此处。

逐条读取 99 个实际 CSV 和 pkl，重新计算 100,940 个非终止 transition；重算奖励转 float32 后与 pkl 完全一致（最大绝对误差 0）。learner 日志确认 demo buffer 为 101,039 条，等于上述 transition 加 99 个终止样本。

| 每步分量 | 全 demo 非终止均值 |
|---|---:|
| R_reserve | 0.0506804 |
| C_v | 0.0348747 |
| 0.1 C_a | 51.2483 |
| 0.05 C_j | 819,434.813 |
| 0.5 C_filter | 0 |
| 已存 reward | −819,486.039 |

最差单步 reward：−3,645,081,088。jerk 占净负回报量级约 99.99375%。这是实际进入训练的输入缺陷；它对最终性能的独立贡献仍需消融实验，而不能仅从数值推断全部因果。

## 2. Replay 把 safe action 当成 nominal action，破坏奖励和状态转移语义

`vectorized_actor.py:169–184` 将动作覆盖为 `phi_dot_safe/scale`，而环境按原始 nominal action 计算 C_filter，并把原始 action 写入下一观测的 a_prev 和历史。

因此 critic 收到的 `(s, a_safe, r, s')` 不是该环境执行 `a_safe` 会产生的 transition。对同一个 safe action，不同 nominal action 可产生不同惩罚，actor 更新又在 nominal action 空间查询 Q。过滤越频繁，问题覆盖越广；失败时还会把触发失败的动作记成零动作。

实际 manifold 上，seed=0 的第一步 nominal `[10.45,10.45]` 被过滤为约 `[0,2.96763]`。reward 为 −8898.235；若以记录的 safe action 重新解释该步，单 C_filter 就差 82.594，下一观测的动作历史也不同。

## 3. 400 条 RL 轨迹实际上最多重复 8 条

`vectorized_actor.py:73` 每次 episode 重置都调用相同的 `reset(seed=self._seed+i)`；`env.py:112–114` 重新初始化 RNG 后立刻选轨迹。

在实际环境上连续同 seed reset，轨迹数组完全相同。启动脚本未传 seed，`train_rlpd.py:44` 默认 42，因此 8 个 worker 重复种子 42…49 对应的轨迹，不能覆盖 400 条训练集。8 个环境在当前实现中也由 Python 循环串行 step，并非计划中的 8 个子进程。

## 4. 新 episode 第一动作由旧 episode 的 terminal observation 产生

actor 先 sample_actions，再调用 vector step；vector step 内部才 reset 上一步结束的 env。动作对应旧 terminal observation，却施加到新初态。`self.obs` 的原位替换还改变 actor 持有的源观测数组。

用实际 vector 类及最小 fake env 验证：terminal observation 为 1 时采样的动作 `[1,1]`，随后 reset 到观测 0，仍执行 `[1,1]`，记录的源观测已变为 0。

## 5. 奖励尺度与失败终止确实存在目标漏洞

- `env.py:161–165`：过滤器失败直接返回 reward=0、terminated=True、truncated=True，跳过所有奖励项。
- replay mask=0，失败不再 bootstrap；成功继续执行却可能累积大量负奖励。代码没有明确的成功奖励或失败成本。
- dt=0.01 时，`w_a/dt²=1000`。例如单臂相速度一步变化 0.1，单加速度惩罚就是 10，而正常安全区间内 R_reserve 至多为 1。
- 此处 `/dt²` 本身是加速度平方的正确量纲；问题是未做尺度校准及与任务目标的优先级对齐。去掉它相当于改为“相速度增量平方”，不是保留原物理意义的同一奖励。
- 正常跑完被标成 truncated，在线 mask 仍为 1；demo 则额外追加 mask=0 的零动作终止行。有限任务终止语义也需要统一。

零奖励失败机制支持“提前结束可能更划算”的假设，但 J 与完成率两个汇总点不能证明策略有意寻求失败。还需固定任务前缀、分解奖励、终止原因统计及单因素消融。

## 6. 监控和 spec 验收存在偏差

- `vectorized_actor.py:199–207` 的 mean_episode_return 是“10 个 vector step 内 reward 总和 / 此窗口结束 episode 数”；跨窗口 episode 未累计。窗口没有 episode 结束时除以 1。故 −1.42e6 的峰值不能直接解释为完整 episode return。
- `evaluate_policy.py:119,140` 的 e_task 是 `1−steps/total_steps`，无量纲，不是计划 Task 2.3 中要求小于 0.1 mm 的几何任务误差；失败的尝试步也计入 steps。
- `env.py:17–20` 实际是纯运动学插值；没有计划中的 Newton 投影、MuJoCo 10 子步和真实几何误差复核。插值 d_min 与 qdot 通过不能作为完整物理执行验收。
- 计划 Task 2.3 要求 checkpoint 确定性评估；learner 只有保存，没有周期验证调用，无法及时发现 completion 下降。
- 1M 是 learner 迭代次数，不是 1M 条环境样本；cta_ratio=2 又包含额外 critic 更新。最后保存点是 980000，评估不等于最终 1000000 状态。
- learner 训练循环完成，但日志有结束后的 agentlace/W&B stats callback 线程异常，因此“全程完全无报错”不精确；未见证据表明它导致训练退化。

## 对原判断的修正

1. scale=10.45 符合计划的 `1.1 × max|teacher phi_dot|`；实际左右臂最大值为 `[9.5,1.0]`，但使用统一尺度。过大的右臂探索范围值得校准，不能直接把关节限速 1.5 当相速度上限。
2. 约束是 `|Q_x xdot + Q_phi phi_dot|≤1.5`。实际验证得到安全相速度 2.96763，反证“必被裁到 1.5”的假设。
3. Phase 3 规定的是动作尺度校准，不是奖励重设计；原文给了奖励权重，但对成本公式写作 “as before”，没有完整定义终止收益或奖励量级验收。
4. infeasible 表示当前过滤器未找到安全步，包括多边形为空或有限次回溯失败，并非完整可行性求解器对所有动作的不可行证明。

## 建议处理顺序

1. 先统一 demo/在线 reward、初始历史和终止规则，并重新生成 demo；检查相同 transition 的奖励一致性。
2. replay 保留 policy 的 nominal action，safe action 单独记录；修复 reset 时机，仅首次 reset 设置 seed。
3. 加完整 episode 统计、加权 reward 分量、nominal/safe 动作分布、终止原因和训练轨迹覆盖统计；验证 BC→SAC 初始确定性行为一致。
4. 在统一语义后重新设计/校准完成目标、失败成本和无量纲平滑项，固定 BC 起点做单因素短跑。若改动作尺度，应同步重建 demo、历史动作归一化和 BC；不可只改在线 scale。
5. 以同一组 val 的 completion / infeasible 和真实任务误差验收，定期保存最佳 checkpoint；明确运动学实验与原 MuJoCo spec 的范围差异。

上述是修复路线，尚未实施；不建议仅调低 w_a/w_f 或继续延长这次训练来验证根因。

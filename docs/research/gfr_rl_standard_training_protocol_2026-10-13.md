# GFR-RL 标准训练与验收流程

## 目标

验证 RL 是否能在固定闭链流形和冻结的 endpoint-aware governor 后端上学习 phase-rate 调度。RL 不改变流形，不接管 `beta`，也不替代 endpoint/swept checker。

标准训练预算最多为 60k online transitions，但只有通过 20k 和 40k checkpoint 闸门，才允许继续训练。

## 0. 实验冻结

每次正式运行前保存一份配置快照，至少包含：

- manifold、MuJoCo model、trajectory split；
- `dt`、`d_safe`、`beta_stop`、`qdot_max`、`r_max`；
- demo/online batch 比例；
- actor/critic/temperature 学习率和 clipping；
- 并行环境数、训练 seed、评估 seed；
- completion、clearance、speed 和安全停止定义。

训练过程中不得修改这些内容。每次运行写入独立版本目录，不覆盖旧 checkpoint 或 JSON。

## 1. 速度分层的 nominal 成功示教

示教与探索是两个不同的比例：

- 示教数据配额：100 条完整成功轨迹；
- 在线探索配额：400 个训练 episode；
- learner batch 比例另行冻结，不能把 100:400 直接解释成 batch 中的比例。

100 条示教按 val 轨迹速度分层：

| 速度区间 | 轨迹数 |
|---|---:|
| `0.015--0.05` | 20 |
| `0.05--0.15` | 20 |
| `0.15--0.30` | 20 |
| `0.30--0.45` | 20 |
| `0.45--0.70` | 20 |

每个速度区间还要覆盖多个起点、运动方向和几何条件。示教必须在 nominal model 上由相同的 endpoint/swept checker 接受到终点。旧示教不能通过简单时间缩放冒充新示教，因为速度变化会改变 clearance、governor 和 phase-rate 的可行域。

示教验收条件：

- 100/100 轨迹到达终点；
- 每个 accepted transition 通过 endpoint 和 swept checker；
- 没有 `instrumentation_unknown`；
- 保存 source-task 条件、速度、起点、动作和执行动作；
- 生成 nominal compatibility audit。

## 2. Replay buffer 和训练前预检

demo buffer 与 online replay 分开保存。建议使用预先声明的采样计划：

- `0--10k`：50% demo、50% online，用于稳定初始化；
- `10k--60k`：25% demo、75% online，用于扩大探索；
- 另保存固定 50/50 作为一次性消融，不在看到结果后切换。

训练前先运行 256-transition smoke：

- 检查观测和动作维度；
- 检查 `policy_action_raw → action_nominal → action_executed` 日志；
- 检查 demo/online 实际采样比例；
- 检查 endpoint/swept checker 和 governor 确实被调用；
- 检查 actor、learner 和 checkpoint 正常退出。

预检失败时不进入长训练。

## 3. 正式训练配置

推荐使用 3 个独立 seed 和 16 个并行环境。每 5k 保存 checkpoint，并持续记录：

- completion、episode progress、intervention、recovery 和 rejection；
- clearance、speed、joint-limit 和安全停止原因；
- critic loss、Q 均值、Q 标准差；
- actor loss、policy entropy、temperature；
- demo/online 实际采样比例；
- actor/learner 返回码和耗时。

训练使用固定的随机种子和固定的 source-task/held-out-val 划分。source-task 结果只用于训练监控，held-out-val 结果只用于 checkpoint 闸门和最终报告。

## 4. 20k 并行评估

训练进程达到 20k 后保存 checkpoint，并启动独立评估进程；训练进程继续运行到 40k，不因评估而暂停。评估使用固定的 10 个 source-task episodes，并记录完整指标。

20k 结果只作为早期信号：

- 20k completion 大于 0：说明已经出现可完成行为，训练继续；
- 20k completion 为 0：训练仍继续到 40k，但标记为高风险状态；
- 无论 20k 结果如何，都不能仅凭一次 20k 评估终止训练。

评估进程必须使用独立输出目录，并记录 checkpoint hash、评估 seed 和返回码。若 GPU 资源不足，可以限制评估进程的 GPU 显存或使用独立设备，但不能混用训练和评估的 replay 或参数。

## 5. 40k 并行评估与停止决策

训练进程达到 40k 后保存 checkpoint，并启动第二个独立评估进程。训练可继续短暂运行以完成 checkpoint 持久化，但不应直接进入 60k，直到 20k 和 40k 结果汇总完成。

如果 **20k 和 40k 都是 0 completion**，或者两个 checkpoint 都显示重复的早期
`governor_exhausted`，则终止后续训练，不运行 60k。先检查示教覆盖、完整 horizon
可行性和动作投影，再开启新版本实验。

如果 20k 或 40k 至少一个出现非零 completion，训练可以继续到 60k，但这只表示
“没有触发零完成率停止条件”，不等于已经收敛。40k completion、progress、
intervention、clearance 和 Q/temperature 稳定性仍需作为最终 checkpoint 选择依据。
建议把 `40k completion >= 90%` 作为质量目标，而不是在本轮协议中事后强行解释成
收敛证明。

40k 未通过时停止，不进入 60k，并输出原因分类。不能因为 50k 或 60k 的单个 checkpoint progress 较高而绕过该闸门。

## 6. 可选 60k 延续和最终评估

只有 20k 和 40k 都通过，才允许继续到 60k。60k 仍然需要与 40k 比较，不能默认它更好。

最终 held-out 评估至少包含：

- 每个 seed 至少 50 个 episode，分批运行；
- 五个速度区间分别报告；
- source-task 与 held-out-val 分开报告；
- completion、progress、clearance、intervention、endpoint rejection、safe-stop reason；
- 最终 checkpoint 和完整 checkpoint 曲线同时保留；
- checkpoint 选择规则在实验前冻结，默认报告最终 checkpoint，不事后挑选最佳点。

## 7. 停止规则

遇到以下任一情况立即停止当前节点：

- checkpoint 闸门失败；
- critic/Q 或 temperature 持续发散；
- 大量 episode 重复在同一 clearance 边界耗尽 governor；
- demo 与 nominal checker 不兼容；
- 出现 `instrumentation_unknown`；
- actor/learner 非正常退出；
- 为提高 completion 需要修改 `d_safe`、`beta_stop`、checker 或终止规则。

停止后先修复原因，再开启带新版本号的实验。旧结果保持不可变。

## 8. 论文证据边界

通过 20k 或 40k 闸门只能说明训练过程具备稳定进展，不能单独证明泛化。只有 held-out-val 在速度分层和多 seed 下保持性能，才能支持“RL 提高速度覆盖或执行质量”的条件性结论。任何失败都不能解释为 branch jump。
